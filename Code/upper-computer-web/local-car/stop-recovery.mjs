const clean=line=>String(line).trim().replace(/^(?:arm>\s*)+/,'');
const fields=line=>Object.fromEntries([...line.matchAll(/\b([a-z_]+)=([^\s]+)/g)].map(m=>[m[1],m[2]]));
const ack='OK chassis request accepted (not motion/ACK confirmation)';
const taskLine=line=>line.startsWith('OK chassis task ')&&
 ['state','stage','error','locked','units','origin_valid','feedback_valid','yaw_valid','feedback_age_ms'].every(k=>/^\d+$/.test(fields(line)[k]??''));
const wheelLines=lines=>{
 const wheels=lines.filter(l=>/^OK wheel=[1-4] /.test(l)).map(fields);
 return wheels.length===4&&wheels.every((w,i)=>w.wheel===String(i+1)&&/^-?\d+$/.test(w.raw_rpm??'')&&
  /^[01],[01],[01]$/.test(w.valid??'')&&/^\d+,\d+,\d+$/.test(w.rx_ms??''))?wheels:null;
};
const recoveryResult=lines=>{
 const normal=lines.findLast(l=>l.startsWith('OK bus recover complete '));
 if(normal)return {mission:false,...fields(normal)};
 // Mission reports are asynchronous. Only use those following this request's
 // mission-preserved acceptance; a prior paused snapshot cannot finish it.
 const accepted=lines.findLastIndex(l=>l.startsWith('OK bus recover requested ')&&fields(l).mission_preserved==='1');
 if(accepted<0)return null;
 const report=lines.slice(accepted+1).findLast(l=>l.startsWith('OK grab route '));
 if(!report)return null;
 const result=fields(report);
 if(result.state==='bus_recovery_wait')return {mission:true,failed:true,...result};
 if(result.state==='paused'&&result.reason==='bus_recovered'&&result.stop_confirmed==='1'&&result.auto_resume==='0')
  return {mission:true,...result};
 return null;
};

/** Explicit stop-bar action. Construction, connection and refresh stay passive. */
export class StopRecovery {
 constructor({bridge,changed=()=>{},now=()=>performance.now(),wait=ms=>new Promise(r=>setTimeout(r,ms)),replyTimeoutMs=3500}){
  Object.assign(this,{bridge,changed,now,wait,replyTimeoutMs});this.busy=false;this.revision=0;this.phase='idle';this.message='';this.pending=null;
 }
 show(phase,message){this.phase=phase;this.message=message;this.changed();}
 cancel(message='本次恢复已取消；不会自动继续。'){
  if(!this.busy)return;this.revision++;
  if(this.pending){this.pending.reject(Error(message));this.pending=null;}
  this.show('cancelled',message);
 }
 observeStop(e){if(this.busy&&e.latched&&e.token!==this.stopToken)this.cancel('新的停车请求已取消本次恢复。');}
 check(revision){
  const b=this.bridge;
  if(revision!==this.revision||!b.connected||b.disconnecting||b.connectionRevision!==this.connectionRevision||
   b.commandEpoch!==this.commandEpoch||b.stopToken!==this.stopToken)throw Error('连接或停车状态已改变；本次恢复不再继续。');
 }
 receive(raw){
  const p=this.pending;if(!p)return;const line=clean(raw);
  if(line===p.wire){p.echo=true;return;}
  if(!p.echo)return;
  p.lines.push(line);
  if(line.startsWith('ERR ')||line.startsWith('WARN bus recover ')){this.pending=null;p.reject(Error(line));}
  else if(p.matches(p.lines)){this.pending=null;p.resolve(p.lines);}
 }
 async query(wire,matches,revision,{owned=false,timeout=this.replyTimeoutMs}={}){
  // Only repeat read-only queries after damaged/missing replies. Writes are one-shot.
  const attempts=['chassis task','chassis feedback'].includes(wire)?3:1;
  for(let i=0;i<attempts;i++){
   this.check(revision);let timer,p;
   const reply=new Promise((resolve,reject)=>{
    p={wire,matches,resolve,reject,lines:[],echo:false};this.pending=p;
    timer=setTimeout(()=>{if(this.pending===p)this.pending=null;const e=Error(`等待 ${wire} 完整回复超时；未重发写入。`);e.readTimeout=true;reject(e);},timeout);
   });
   try{const [,lines]=await Promise.all([this.bridge.send(wire,owned),reply]);this.check(revision);return lines;}
   catch(e){if(!e.readTimeout||i+1===attempts)throw e;}
   finally{clearTimeout(timer);if(this.pending===p)this.pending=null;}
  }
 }
 async readTask(revision,owned=false){return fields((await this.query('chassis task',v=>v.some(taskLine),revision,{owned})).findLast(taskLine));}
 async readWheels(revision){return wheelLines(await this.query('chassis feedback',v=>!!wheelLines(v),revision,{owned:true}));}
 async start(){
  if(this.busy)return false;
  const b=this.bridge,revision=++this.revision;this.busy=true;
  this.stopToken=b.stopToken;this.commandEpoch=b.commandEpoch;this.connectionRevision=b.connectionRevision;
  let acquired=false,missionPreserved=false;this.show('recovering','正在读取底盘隔离状态…');
  try{
   this.check(revision);let task=await this.readTask(revision);
   if(!['0','3','4'].includes(task.state))throw Error('底盘任务尚未停止，请等待停车确认后再恢复。');
   if(task.locked!=='0'||task.error!=='0'){
    this.show('recovering','正在恢复电机总线，等待实际检查完成…');
    const lines=await this.query('bus recover',v=>!!recoveryResult(v),revision,{timeout:25000});
    const result=recoveryResult(lines);missionPreserved=result.mission;
    if(result.failed)throw Error(`比赛总线恢复未完成：${result.reason??'unknown'}；比赛进度保留，请处理后重试。`);
    if(missionPreserved){
     task=await this.readTask(revision);
     if(task.locked!=='0'||task.error!=='0'||!['0','3','4'].includes(task.state))
      throw Error('比赛已报告恢复，但当前底盘仍未就绪；请查看设备状态后重试。');
    }else if(result.chassis_ready!=='1'||result.wheels_stopped!=='1')
     throw Error('总线恢复后底盘尚未就绪，请查看具体设备原因。');
   }
   await b.resume();this.check(revision);
   if(b.stopLatched)throw Error('网页停车锁尚未解除，请读取最新停车状态。');
   await b.acquirePreparation();acquired=true;this.check(revision);
   const before=await this.readWheels(revision);
   this.show('recovering','正在开启四轮采集，等待新鲜静止反馈…');
   await this.query('chassis feedback 0',v=>v.includes(ack),revision,{owned:true});
   await this.query('chassis feedback on',v=>v.includes(ack),revision,{owned:true});
   const deadline=this.now()+6000;let fresh=false;
   do{
    const wheels=await this.readWheels(revision);
    fresh=wheels.every((w,i)=>w.valid.startsWith('1,1,')&&Math.abs(Number(w.raw_rpm))<=1&&
     w.rx_ms.split(',').slice(0,2).every((v,j)=>Number(v)>0&&v!==before[i].rx_ms.split(',')[j]));
    if(fresh)break;
    await this.wait(250);this.check(revision);
   }while(this.now()<=deadline);
   if(!fresh)throw Error('未取得四轮新鲜静止反馈；请检查反馈或重试恢复。');
   task=await this.readTask(revision,true);
   if(task.locked!=='0'||task.error!=='0'||(task.origin_valid==='1'&&
    (task.feedback_valid!=='1'||Number(task.feedback_age_ms)>600)))throw Error('底盘或定位反馈仍未就绪，请查看设备状态。');
   await b.releasePreparation();acquired=false;this.check(revision);
   this.show('ready',missionPreserved?
    (task.origin_valid==='1'?'底盘锁定已解除，四轮反馈已恢复；比赛保持暂停，请在比赛卡片显式继续。':'底盘锁定已解除，四轮反馈已恢复；比赛保持暂停，定位参考无效，请先处理参考问题。'):
    (task.origin_valid==='1'?'底盘锁定已解除，四轮反馈已恢复；重新点击测试才运动。':'底盘锁定已解除，四轮反馈已恢复；独立测试仍需准备定位参考。'));return true;
  }catch(e){if(revision===this.revision)this.show('failed',e.message);return false;}
  finally{
   if(acquired){try{await b.releasePreparation();}catch(e){if(revision===this.revision)this.show('failed','释放准备状态失败：'+e.message);}}
   this.busy=false;this.changed();
  }
 }
}
