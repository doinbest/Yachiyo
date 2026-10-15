import {parseChassisTestResult} from './chassis-test-result.mjs';
import {reasonText} from './protocol.mjs';

const accepted='OK chassis request accepted (not motion/ACK confirmation)';
const clean=line=>String(line).trim().replace(/^(?:arm>\s*)+/,'');
const fields=line=>Object.fromEntries([...line.matchAll(/\b([a-z_]+)=([^\s]+)/g)].map(m=>[m[1],m[2]]));
const idle=state=>['idle','done','error','cancelled'].includes(state);
const freshLocation=task=>task.origin_valid==='1'&&task.feedback_valid==='1'&&task.yaw_valid==='1'&&
 /^\d+$/.test(task.feedback_age_ms??'')&&Number(task.feedback_age_ms)<=600;
const normalizeTestWire=wire=>{
 const parts=String(wire).trim().split(/\s+/),values=parts.slice(2,5).map(Number);
 const custom=parts.length>=7,parameter=custom?Number(parts[6]):300,gains=parts.slice(7,9).map(Number),period=parts.length>=10?Number(parts[9]):1,decel=parts.length===11?Number(parts[10]):parameter;
 const parameterValid=Number.isFinite(parameter)&&(parts[5]==='linear'?parameter>=10&&parameter<=5000:Number.isInteger(parameter)&&parameter>=10&&parameter<=2000);
 if((parts.length===11&&(parts[5]!=='linear'||!Number.isFinite(decel)||decel<10||decel>5000))||!Number.isInteger(period)||period<1||period>100||![6,7,9,10,11].includes(parts.length)||(custom&&!parameterValid)||gains.some(v=>!Number.isFinite(v)||v<0||v>1)||parts[0]!=='chassis'||parts[1]!=='test'||!['linear','cosine'].includes(parts[5])||
  values.some(v=>!Number.isFinite(v))||Math.hypot(values[0],values[1])<=0||Math.hypot(values[0],values[1])>3000||values[2]<10||values[2]>5000)return null;
 return `chassis test ${values.join(' ')} ${parts[5]}${custom?' '+parameter:''}${parts.length>=9?' '+gains.join(' '):''}${parts.length>=10?' '+period:''}${parts.length===11?' '+decel:''}`;
};
const stoppedWheels=lines=>{
 const wheels=lines.filter(l=>/^OK wheel=/.test(l)).map(fields);
 if(wheels.length!==4||wheels.some((w,i)=>w.wheel!==String(i+1)||!/^1,1,[01]$/.test(w.valid??'')||
  !/^-?\d+$/.test(w.raw_rpm??'')||Math.abs(Number(w.raw_rpm))>1||!/^\d+,\d+,\d+$/.test(w.rx_ms??'')))return null;
 return wheels.map(w=>w.rx_ms.split(',').slice(0,2).map(Number));
};

/** Explicit single-test lifecycle. Mounting/reconnecting never sends commands. */
export class ChassisTestSession {
 constructor({send,stop,acquire=async()=>{},release=async()=>{},canStart=()=>true,
  now=()=>performance.now(),wait=ms=>new Promise(r=>setTimeout(r,ms)),started=()=>{},changed=()=>{}}){
  Object.assign(this,{send,requestStop:stop,acquire,release,canStart,now,wait,started,changed});
  this.phase='idle';this.message='开始测试直接提交运动；需要时单独查询或准备/恢复状态。';
  this.reports={};this.lastFault=null;this.connected=false;this.revision=0;this.operation=null;this.pending=null;this.id=null;this.nextPoll=0;
 }
 get preparing(){return !!this.operation;}
 get busy(){return this.preparing||['running','stopping'].includes(this.phase);}
 show(phase,message){this.phase=phase;this.message=message;this.changed();}
 check(revision){if(revision!==this.revision||!this.connected)throw Error('本次操作已取消，不会自动续发运动。');}
 cancel(message='已取消本次操作和自动查询；不会自动继续。'){
  this.revision++;this.id=null;
  if(this.pending){const p=this.pending;this.pending=null;clearTimeout(p.timer);p.reject(Error(message));}
  this.show('cancelled',message);
 }
 connection(value){if(this.connected===value)return;this.connected=value;this.reports={};this.lastFault=null;this.cancel(value?'已连接；开始测试直接提交指令，状态可按需查询。':'连接已断开；不会自动继续，车辆停稳状态待确认。');}
 reset(){this.reports={};this.lastFault=null;this.cancel('设备已重启；状态待查询，可按需准备/恢复。');}
 observe(line){
  if(!this.connected)return;
  const key=line.startsWith('OK chassis task ')?'task':line.startsWith('OK imu ready=')?'imu':line.startsWith('OK imu cal_state=')?'cal':null;
  if(key){this.reports[key]={values:fields(line),at:this.now()};this.changed();}
  if(/^ERR chassis test /.test(line)){this.lastFault=fields(line).reason??line;this.changed();}
  else if(/^OK chassis test reason=accepted /.test(line)){this.lastFault=null;this.changed();}
 }
 statusEntries(){
  const task=this.reports.task,imu=this.reports.imu,cal=this.reports.cal;
  const label=(r,text)=>r?`${text}（${Math.max(0,Math.floor((this.now()-r.at)/1000))}秒前回报）`:'尚未读取';
  const t=task?.values??{},i=imu?.values??{},c=cal?.values??{};
  return [
   ['连接',this.connected?'已连接':'未连接'],
   ['电机总线',label(task,t.locked==='1'?'已锁定':t.error!==undefined&&t.error!=='0'?`异常 ${t.error}`:t.locked==='0'?'最近正常':'状态未知')],
   ['IMU',label(cal??imu??task,cal?(c.verified==='1'&&c.control_ready==='1'?'验证通过':`未就绪 · ${reasonText(c.reason??c.cal_state??'待验证')}`):imu?(i.ready==='1'&&i.valid==='1'?'角度有效，验证资格未读取':'角度未就绪'):(t.yaw_valid==='1'?'航向有效':'航向未就绪'))],
   ['定位反馈',label(task,t.origin_valid==='0'?'缺少测试参考':t.feedback_valid==='1'?'最近有效':t.feedback_valid==='0'?'反馈不可用':'状态未知')],
   ['设备任务',label(task,reasonText(t.reason??'未知'))],
   ['最近拒绝',this.lastFault?reasonText(this.lastFault):'无已知拒绝']
  ];
 }
 receive(raw){
  const line=clean(raw),p=this.pending;this.observe(line);
  if(p){
   if(line===p.wire){p.echo=true;return;}
   if(p.echo){
    p.lines.push(line);
    if(line.startsWith('ERR ')||p.matches(line)){
     clearTimeout(p.timer);this.pending=null;
     if(line.startsWith('ERR '))p.reject(Error(`设备拒绝：${reasonText(fields(line).reason??line)}`));
     else p.resolve(p.lines);
    }
   }
  }
  const result=parseChassisTestResult(line);
  if(!this.connected||!['running','stopping'].includes(this.phase)||result?.kind!=='test_result'||result.id!==this.id)return;
  if(result.state===3&&result.stop_confirmed===1)this.show('done',`测试 #${this.id} 已完成，设备已确认停稳。`);
  else if(result.state===4)this.show('failed',`测试 #${this.id} 异常；${result.stop_confirmed?'设备已确认停稳':'停车尚未确认，请查看停止状态'}。`);
  else if(result.state===2||result.state===3)this.show('stopping',`测试 #${this.id} 正在等待停车确认。`);
 }
 async query(wire,matches,revision,owned=true){
  this.check(revision);
  let pending;
  const reply=new Promise((resolve,reject)=>{
   pending={wire,matches,resolve,reject,lines:[],echo:false};
   pending.timer=setTimeout(()=>{if(this.pending===pending)this.pending=null;reject(Error(`等待 ${wire} 回复超时；未自动重发运动指令。`));},12000);
   this.pending=pending;
  });
  try{
   const [,lines]=await Promise.all([Promise.resolve(this.send(wire,owned)).then(ok=>{if(!ok)throw Error('发送未确认，请检查连接与停止锁定。');}),reply]);
   this.check(revision);return lines;
  }finally{clearTimeout(pending.timer);if(this.pending===pending)this.pending=null;}
 }
 async readTask(revision){const lines=await this.query('chassis task',l=>l.startsWith('OK chassis task '),revision);return fields(lines.at(-1));}
 checkIdle(task){
  if(task.locked!=='0'||task.error!=='0'||!['0','3','4'].includes(task.state)||!['0','5'].includes(task.stage))throw Error('底盘正在运动、被占用或总线异常；请先查看任务与停车状态。');
  if(task.units!=='65536')throw Error('位置反馈单位未就绪；请核对底盘配置。');
 }
 async start(wire){
  if(!this.connected||this.busy||!this.canStart())return false;
  wire=normalizeTestWire(wire);
  if(!wire){this.show('failed','测试参数无效，请检查方向、距离和速度。');return false;}
  const revision=++this.revision;this.id=null;this.started();this.operation=Promise.resolve();
  this.show('requesting','正在提交单段测试，等待设备受理…');
  try{
   const lines=await this.query(wire,l=>parseChassisTestResult(l)?.kind==='test',revision,false);
   const result=parseChassisTestResult(lines.at(-1));
   if(!result||result.rejected)throw Error('测试未被设备受理，请查看设备原因。');
   this.id=result.id;this.nextPoll=this.now();this.show('running',`测试 #${this.id} 已受理；正在自动查询进度和停车结果。`);return true;
  }catch(error){if(revision===this.revision)this.show('failed',error.message);return false;}
  finally{this.operation=null;this.changed();}
 }
 async refresh(){
  if(!this.connected||this.busy)return false;
  const revision=++this.revision;this.operation=Promise.resolve();this.show('querying','正在读取底盘与IMU状态…');
  try{
   await this.query('chassis task',l=>l.startsWith('OK chassis task '),revision,false);
   await this.query('imu status',l=>/^OK imu relative_deg=/.test(l),revision,false);
   this.show('idle','状态已更新；查询未改变反馈开关或定位参考。');return true;
  }catch(error){if(revision===this.revision)this.show('failed',error.message);return false;}
  finally{this.operation=null;this.changed();}
 }
 async prepare(){
  if(!this.connected||this.busy||!this.canStart())return false;
  const revision=++this.revision;this.id=null;
  // Keep the operation marker until lock release finishes, including cancellation.
  this.operation=Promise.resolve();this.show('preparing','正在查询底盘、路线和比赛状态…');
  let acquired=false;
  try{
   await this.acquire();acquired=true;this.check(revision);
   let task=await this.readTask(revision);this.checkIdle(task);
   const route=fields((await this.query('chassis route status',l=>/^OK chassis route /.test(l),revision)).at(-1));
   if(!idle(route.state))throw Error('已有路线占用或等待继续；本次独立测试未启动。');
   const mission=fields((await this.query('grab mission status',l=>/^OK grab route /.test(l),revision)).at(-1));
   if(!idle(mission.state))throw Error('比赛仍有运行或待续接进度；本次测试不接管底盘。');
   this.show('preparing','正在读取IMU验证资格…');
   const imu=await this.query('imu status',l=>/^OK imu relative_deg=/.test(l),revision);
   const sensor=fields(imu.find(l=>/^OK imu ready=/.test(l))??''),cal=fields(imu.find(l=>/^OK imu cal_state=/.test(l))??'');
   if(sensor.ready!=='1'||sensor.valid!=='1'||sensor.fresh!=='1'||cal.busy!=='0'||cal.verified!=='1'||cal.control_ready!=='1'||cal.normal_mode!=='1')throw Error('IMU尚未完成有效静止验证；请先在系统与标定页验证，本次不会自动归零。');
   const before=await this.query('chassis feedback',l=>/^OK wheel=4 /.test(l),revision);
   const baseline=before.filter(l=>/^OK wheel=/.test(l)).map(l=>(fields(l).rx_ms??'').split(',').slice(0,2).map(Number));
   this.show('preparing','正在开启四轮反馈，等待新的静止反馈…');
   await this.query('chassis feedback 0',l=>l===accepted,revision);
   await this.query('chassis feedback on',l=>l===accepted,revision);
   const deadline=this.now()+6000;let fresh=false;
   while(this.now()<=deadline){
    const lines=await this.query('chassis feedback',l=>/^OK wheel=4 /.test(l),revision),samples=stoppedWheels(lines);
    fresh=!!samples&&samples.every((s,i)=>s.every((v,j)=>Number.isSafeInteger(v)&&v!==(baseline[i]?.[j]??0)));
    if(fresh)break;
    await this.wait(250);this.check(revision);
   }
   if(!fresh)throw Error('未取得四轮新的静止速度/位置反馈；请展开四轮状态检查，未发送运动。');
   task=await this.readTask(revision);this.checkIdle(task);
   if(task.origin_valid!=='1'){
    this.show('preparing','正在把当前位置和车头建立为测试参考 (0,0,0°)…');
    await this.query('chassis origin 0 0 0',l=>l===accepted,revision);
   }
   this.show('preparing','正在复查定位有效性和反馈新鲜度…');
   const locationDeadline=this.now()+6000;
   do{
    task=await this.readTask(revision);this.checkIdle(task);
    if(freshLocation(task))break;
    await this.wait(250);this.check(revision);
   }while(this.now()<=locationDeadline);
   if(!freshLocation(task))throw Error('定位反馈尚未有效，请查看底盘与四轮状态；未发送运动。');
   await this.release();acquired=false;this.check(revision);
   this.lastFault=null;this.show('ready','测试状态准备完成；未启动运动。可直接开始测试。');return true;
  }catch(error){if(revision===this.revision)this.show('failed',error.message);return false;}
  finally{
   if(acquired){try{await this.release();}catch(error){if(revision===this.revision)this.show('failed','释放准备状态失败：'+error.message);}}
   this.operation=null;this.changed();
  }
 }
 async stop(){this.cancel('已请求停止测试；等待设备确认停车。');this.show('stopping','已请求停止测试；等待设备确认停车。');return this.requestStop();}
 stopEvent(event){
  if(!event.latched)return;
  if(this.phase!=='stopping')this.cancel('停止操作已取消测试准备和自动启动。');
  if(event.status==='confirmed')this.show('stopped','设备已确认停车；解除停止锁定后需重新点击开始。');
  else this.show('stopping',event.status==='timeout'?'停车确认超时；请检查设备，不能视为已停稳。':'正在请求停车，尚未确认停稳。');
 }
 tick(){
  if(!this.connected||this.preparing||this.id===null||!['running','stopping'].includes(this.phase)||this.polling||this.now()<this.nextPoll)return;
  this.nextPoll=this.now()+1000;this.polling=true;
  Promise.resolve(this.send('chassis task',false)).catch(()=>{}).finally(()=>{this.polling=false;});
 }
}

export function mountChassisTestSession({root,changed=()=>{},...options}){
 const doc=root.ownerDocument,text=doc.createElement('p'),stop=doc.createElement('button'),query=doc.createElement('button'),prepare=doc.createElement('button'),summary=doc.createElement('dl');
 text.className='form-note';text.setAttribute('role','status');summary.className='chassis-test-state';summary.setAttribute('aria-label','底盘最近回报状态');
 const cells=['连接','电机总线','IMU','定位反馈','设备任务','最近拒绝'].map(label=>{const box=doc.createElement('div'),dt=doc.createElement('dt'),dd=doc.createElement('dd');dt.textContent=label;box.append(dt,dd);summary.append(box);return dd;});
 stop.type=query.type=prepare.type='button';stop.id='chassis-test-stop';stop.className='danger';stop.textContent='停止测试';query.id='chassis-test-status';query.textContent='查询状态';prepare.id='chassis-test-prepare';prepare.textContent='准备/恢复测试状态';
 const row=doc.createElement('div');row.className='actions';row.append(query,prepare,stop);root.append(summary,row,text);
 const state=new ChassisTestSession({...options,changed:()=>{render();changed();}});
 function render(){text.textContent=state.message;stop.disabled=!state.connected;query.disabled=!state.connected||state.busy;prepare.disabled=!state.connected||state.busy||!state.canStart();state.statusEntries().forEach(([,value],i)=>{cells[i].textContent=value;});}
 stop.onclick=()=>void state.stop();query.onclick=()=>void state.refresh();prepare.onclick=()=>void state.prepare();setInterval(()=>{state.tick();render();},250);render();return state;
}
