import {GRAB_FORM_DEFAULTS,GRAB_PARAMETER_METADATA} from './grab-parameter-metadata.mjs';
export const TRACK_PRESET=Object.freeze({body_speed:160,x_speed:60,accel:300,lambda:2.5,x_weight:2.333333,lead_mm:12,tol_px:5,control_ms:20,batch_ms:0,latency_ms:50,fit_ms:240,horizon_ms:120,grace_ms:250});
const labels={body_speed:'底盘合速度 / mm/s',x_speed:'伸缩速度 / mm/s',accel:'联合加速度 / mm/s²',lambda:'残余误差增益 / s⁻¹',x_weight:'伸缩分配权重',lead_mm:'最大领先 / mm',tol_px:'位置窗口 / px',control_ms:'定时预测间隔（新视觉包即时修正）/ ms',batch_ms:'额外批次限频（0为应答驱动）/ ms',latency_ms:'延迟联调初值 / ms',fit_ms:'拟合窗口 / ms',horizon_ms:'X前瞻 / ms',grace_ms:'短漏检预测 / ms'};
const text={track_observe:'仅观察 · 不运动',track_match:'正在匹配速度',track_follow:'稳定跟随',track_pause:'保持姿态',idle:'空闲',stopping:'停车中',error:'故障停止',settle:'确认停稳',descend:'快速下降夹取',close:'闭爪',lift:'提起物料',complete:'动作完成',track_pick_stopping:'对准后停车确认',track_pick_descend:'目标仍在窗口内 · 下降',track_pick_reacquire:'停车时目标移动 · 继续追踪',track_pick_complete:'已夹取并提起 · 等待下一步',track_observing:'正在观察和估计',track_matching:'位置与速度匹配',track_following:'像素到点后仍持续跟随',track_visual_wait:'视觉暂停 · 新观测恢复后继续',track_window_end:'行程暂不足 · 保持观察，目标返回可追范围后继续',track_window_recovered:'目标已返回可追范围 · 自动续追',track_recovered:'视觉已恢复',cancelled:'人工停止',stop_unconfirmed:'停车尚未取得反馈确认'};
export const trackText=value=>text[value]??value??'—';
export function trackCrc(line){
  let crc=0xffffffff;
  for(const c of line){crc^=c.charCodeAt(0);for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}
  return ((crc^0xffffffff)>>>0).toString(16).padStart(8,'0').toUpperCase();
}
export function parseTrack(raw){
  const line=raw.trim().replace(/^(?:arm>\s*)+/,'');
  if(!/^(?:OK|ERR) grab track\b/.test(line)&&!/^TRACK (?:row|live)\b/.test(line))return null;
  const type=line.startsWith('TRACK live')?'live':line.startsWith('TRACK row')?'row':line.match(/^OK grab track (config|diag|stats|trace_begin|trace_end|log_begin|log_end|log_status)\b/)?.[1]??'state';
  const result={type,raw:line,error:line.startsWith('ERR')};
  for(const match of line.matchAll(/\b([a-z_0-9]+)=([^\s=]+)/g)){
    const value=match[2];result[match[1]]=/^(?:nan|na)$/.test(value)?null:/^[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?$/i.test(value)?Number(value):value;
  }
  if(type==='row'||type==='live'){
    const body=line.match(/^(.*) crc=([\da-f]{8})$/i);
    if(!body||trackCrc(body[1])!==body[2].toUpperCase())return null;
    const data=String(result.data).split(',').map(Number);
    if(data.length!==19||!data.every(Number.isFinite)||!['i','t','seq','rx'].every(k=>Number.isInteger(result[k])&&result[k]>=0))return null;
    Object.assign(result,{e:data.slice(0,2).join(','),target:data.slice(2,4).join(','),actual:data.slice(4,7).join(','),cmd:data.slice(7,10).join(','),x:data[10],forward:data[11],left:data[12],room:data[13],batch:data[14],age:data[15],request_age:data[16],sent_age:data[17],flags:data[18]});
    if(result.io!==undefined){const io=String(result.io).split(',');if(![26,30,34].includes(io.length)||!io.every(v=>/^-?\d+$/.test(v)))return null;result.io=io.map((v,i)=>(i>=4&&i<=7)||i===15?v:Number(v));}
    if(type==='live'&&(!Number.isInteger(result.id)||result.id<1))return null;
  }
  return result;
}
export class TrackingPanelModel{
  constructor(){this.config=new Map();this.state={};this.diag={};this.stats={};this.records=[];this.complete=false;this.current=false;this.count=null;this.ended=false;this.log={enabled:false,gaps:0,received:0};this.live=null;}
  missing(){return this.count===null?[]:Array.from({length:this.count},(_,i)=>i).filter(i=>!this.records.some(r=>r.i===i));}
  receive(line){
    const p=parseTrack(line);if(!p)return null;
    if(p.type==='config'&&typeof p.value==='number')this.config.set(p.key,p.value);
    else if(p.type==='diag')this.diag=p;
    else if(p.type==='stats')this.stats=p;
    else if(p.type==='log_begin'){
      if(this.log.id!==p.id||(p.next===0&&this.log.received>0)){this.log={id:p.id,enabled:true,gaps:0,received:0,next:p.next??0};this.live=null;}
      this.log.enabled=true;this.log.period=p.period_ms;
    }
    else if(p.type==='log_status'){
      if(this.log.id!==p.id){this.log={id:p.id,gaps:0,received:0,next:null};this.live=null;}
      Object.assign(this.log,{enabled:p.enabled===1,period:p.period_ms,count:p.count});
    }
    else if(p.type==='log_end'&&p.id===this.log.id){this.log.enabled=false;this.log.count=p.count;}
    else if(p.type==='live'&&p.id===this.log.id&&(this.log.next===null||p.i>=this.log.next)){
      if(this.log.next===null)this.log.next=p.i; /* Reconnect midstream is not a complete capture. */
      this.log.gaps+=Math.max(0,p.i-this.log.next);this.log.next=p.i+1;this.log.received++;this.live=p;
    }
    else if(p.type==='row'&&this.count!==null&&p.i<this.count){this.records=this.records.filter(r=>r.i!==p.i);this.records.push(p);this.records.sort((a,b)=>a.i-b.i);}
    else if(p.type==='trace_begin'&&Number.isInteger(p.count)&&p.count>=0&&p.count<=128){this.records=[];this.count=p.count;this.ended=false;this.complete=false;}
    else if(p.type==='trace_end')this.ended=true;
    else if(p.state){this.state=p;this.current=true;this.received=Date.now();}
    this.complete=this.count!==null&&this.ended&&this.missing().length===0;
    this.reply=p.raw;return p;
  }
}

export function mountTracking({root,send,canSend=()=>false}){
  const model=new TrackingPanelModel(),inputs=new Map(),readbacks=new Map(),buttons=[];
  let pending=null,busy=false,connected=false,exportRequested=false,tracePending=null,traceTimer=null,traceGeneration=0,repairing=false,poseReply='',poseBusy=false;
  const el=(tag,value,cls)=>{const e=document.createElement(tag);if(value!==undefined)e.textContent=value;if(cls)e.className=cls;return e;};
  const card=el('section',undefined,'tracking-card');card.id='grab-tracking';
  const header=el('div',undefined,'tracking-header');header.append(el('h3','动态追踪与物料夹取'),el('span','观察与参考自动衔接','tracking-badge'));card.append(header);
  card.append(el('p','先准备抓取姿态，再选择颜色。一键追踪自动观察并在缺少参考时建立料台参考；已有参考和剩余预算继续沿用。纯追踪持续跟随；追踪并夹取在目标停留且对准后下降、闭爪、提起，保持物料等待下一步。','form-note'));
  const color=el('select');color.setAttribute('aria-label','追踪颜色');
  for(const [value,label]of [[1,'红'],[2,'黄'],[3,'蓝'],[4,'绿'],[5,'黑'],[6,'浅蓝']]){const o=el('option',label);o.value=value;color.append(o);}color.value=3;
  const actions=el('div',undefined,'actions');actions.append(color);
  const status=el('p','尚无设备追踪状态','tracking-state'),readout=el('dl',undefined,'tracking-readout'),message=el('p','参数草稿与设备读回独立；本页不会自动发送指令。','form-note');
  const diagnostics=el('details',undefined,'advanced');diagnostics.append(el('summary','运动估计与实际节奏'));const diagnosticText=el('pre','未报告');diagnostics.append(diagnosticText);
  const parameters=el('details',undefined,'advanced');parameters.append(el('summary','独立追踪参数 · 显式写入并读回'));
  const parameterGrid=el('div',undefined,'tracking-parameters');
  for(const [key,value]of Object.entries(TRACK_PRESET)){
    const label=el('label',labels[key]),input=el('input'),readback=el('small','设备：未读取');
    input.type='number';input.step='any';input.value=value;input.dataset.trackKey=key;inputs.set(key,input);readbacks.set(key,readback);label.append(input,readback);parameterGrid.append(label);
  }
  parameters.append(parameterGrid);const parameterActions=el('div',undefined,'actions');parameters.append(parameterActions);
  function button(parent,label,wire,operation){const b=el('button',label);b.type='button';b.title=wire;parent.append(b);buttons.push(b);b.onclick=async()=>{try{await operation();}catch(e){message.textContent=e.message;}render();};return b;}
  const pickup=el('div',undefined,'tracking-parameters');
  function numberField(label,value,step,min,max){const wrap=el('label',label),input=el('input');input.type='number';input.step=step;input.min=min;input.max=max;input.value=value;wrap.append(input);pickup.append(wrap);return input;}
  const budget=numberField('首次建立参考：剩余接近余量 / mm',50,1,0,50);
  const pickupZ=numberField('物料夹取 Z 绝对高度 / mm',-80,1,-150,0);
  const pickupSpeed=numberField('物料夹取下降速度 / mm/s',GRAB_FORM_DEFAULTS.z_pick_speed,1,GRAB_PARAMETER_METADATA.z_pick_speed.min,GRAB_PARAMETER_METADATA.z_pick_speed.max);
  pickupSpeed.exclusiveMin=GRAB_PARAMETER_METADATA.z_pick_speed.exclusiveMin;
  const number=(input,label)=>{const value=Number(input.value);if(input.value===''||!Number.isFinite(value)||value<Number(input.min)||(input.exclusiveMin&&value===Number(input.min))||value>Number(input.max))throw new Error(`${label}输入无效`);return value;};
  button(actions,'准备抓取姿态','grab pose work',async()=>{if(await send('grab pose work')){poseReply='抓取姿态已请求 · 等待设备确认到位';poseBusy=true;}});
  button(actions,'一键持续追踪','grab track begin',()=>send(`grab track begin ${color.value} ${number(budget,'接近余量')}`));
  button(actions,'追踪并夹取','grab track pick',()=>send(`grab track pick ${color.value} ${number(pickupZ,'夹取高度')} ${number(pickupSpeed,'下降速度')}`));
  const stop=button(actions,'停止追踪','grab track stop',()=>{cancelTrace();cancel('参数操作已取消');return send('grab track stop');});stop.className='danger';
  button(actions,'读取状态','grab track status',()=>send(poseBusy?'grab status':'grab track status'));
  const logActions=el('div',undefined,'actions'),logStatus=el('p','实时记录默认关闭；启用后由本地服务持续写入磁盘，不会启动机构。','form-note');
  button(logActions,'开启实时记录','grab track log start',async()=>{if(await send('grab track log start'))message.textContent='已请求20 Hz实时诊断；发送忙时跳过诊断记录，缺号会单独标记。';});
  button(logActions,'关闭实时记录','grab track log stop',()=>send('grab track log stop'));
  button(logActions,'读取记录状态','grab track log status',()=>send('grab track log status'));
  button(actions,'导出本段','grab track trace',async()=>{
    cancelTrace();exportRequested=true;busy=true;model.count=null;model.records=[];model.complete=false;model.ended=false;
    if(!await send('grab track trace'))cancelTrace();
    else{message.textContent='正在读取冻结记录；只补读缺失或校验失败的记录。';traceTimer=setTimeout(()=>{model.ended=true;finishTrace();},40000);}
  });
  function cancelTrace(){traceGeneration++;exportRequested=false;repairing=false;clearTimeout(traceTimer);traceTimer=null;if(tracePending){clearTimeout(tracePending.timer);tracePending.resolve(false);tracePending=null;}busy=false;}
  async function finishTrace(){
    if(!exportRequested||repairing)return;
    clearTimeout(traceTimer);repairing=true;const generation=traceGeneration;
    try{
      if(model.count===null)throw new Error('未收到记录数量，请重新导出本段。');
      for(const index of model.missing()){
        for(let attempt=0;attempt<3&&model.missing().includes(index);attempt++){
          if(generation!==traceGeneration)return;
          await new Promise(resolve=>{
            tracePending={index,resolve,timer:setTimeout(()=>{tracePending=null;resolve(false);},3000)};
            Promise.resolve(send(`grab track trace ${index}`)).then(ok=>{if(!ok&&tracePending?.index===index){clearTimeout(tracePending.timer);tracePending=null;resolve(false);}}).catch(()=>{if(tracePending?.index===index){clearTimeout(tracePending.timer);tracePending=null;resolve(false);}});
          });
        }
      }
      if(generation!==traceGeneration)return;
      if(model.missing().length)throw new Error(`仍缺${model.missing().length}条记录，未导出不完整日志；可重试。`);
      model.complete=true;download();message.textContent=`本段${model.records.length}条记录校验完整，已导出。`;
    }catch(e){message.textContent=e.message;}finally{if(generation===traceGeneration){cancelTrace();render();}}
  }
  async function query(key,wire){
    if(pending)throw new Error('已有参数请求，请等待');
    return new Promise((resolve,reject)=>{
      pending={key,resolve,reject,timer:setTimeout(()=>{pending=null;reject(new Error(`追踪参数${key}读回超时`));},6000)};
      Promise.resolve(send(wire)).then(ok=>{if(!ok)cancel('未发送追踪参数');}).catch(e=>cancel(e.message));
    });
  }
  function cancel(reason){if(pending){clearTimeout(pending.timer);const p=pending;pending=null;p.reject(new Error(reason));}busy=false;}
  async function readAll(){busy=true;render();try{for(const key of inputs.keys())await query(key,`grab track get ${key}`);message.textContent='独立追踪参数已读回；未发送运动命令。';}finally{busy=false;render();}}
  async function apply(){
    busy=true;render();try{
      for(const [key,input]of inputs){const value=Number(input.value);if(!Number.isFinite(value)||input.value==='')throw new Error(`${labels[key]}输入无效`);const written=await query(key,`grab track set ${key} ${value}`);const read=await query(key,`grab track get ${key}`);if(Math.abs(read-value)>Math.max(1e-5,Math.abs(value)*1e-5)||Math.abs(read-written)>1e-5)throw new Error(`${key}写后读回不一致`);}
      message.textContent='追踪参数已写入RAM并读回一致；静态取放参数未修改。';
    }finally{busy=false;render();}
  }
  button(parameterActions,'读取全部','grab track get body_speed',readAll);
  button(parameterActions,'应用并核对','grab track set body_speed 160',apply);
  button(parameterActions,'填入追踪预设','local',()=>{for(const [key,value]of Object.entries(TRACK_PRESET))inputs.get(key).value=value;message.textContent='预设只填入草稿，尚未写入设备。';});
  card.append(actions,pickup,el('p','物料默认准心 (334,338)，对准窗口各 ±5 px。原料高度默认 −80 mm；粗加工 −150 mm 是另一工位。夹取参数会写入本次 MCU RAM，并与正式物料取料共用；不会改色环设置。夹起后不自动入仓。首次夹取缺少参考时采用50 mm余量，已有参考不重置。','form-note'),status,readout,message,logActions,logStatus,diagnostics,parameters);root.append(card);
  function render(){
    const s=model.state,old=model.current?'':'历史 · ',value=key=>s[key]??'—';
    status.textContent=poseReply||(s.state?`${old}${trackText(s.state)} · ${trackText(s.reason)}`:'尚无设备追踪状态');
    readout.replaceChildren();
    for(const [label,display]of [['目标像素 / 误差',s.valid===1?`${value('cx')}, ${value('cy')} / ${value('dx')}, ${value('dy')} px`:'未取得新鲜坐标'],['命令前向 / 左向 / X速度',`${value('vf')} / ${value('vl')} / ${value('vx')} mm/s`],['X位置 / 最近端剩余',`${value('x')} / ${value('xr')} mm`],['底盘剩余接近预算',s.ref===1?`${value('room')} mm`:'未建立参考'],['料台参考',s.ref===1?'已建立 · 停止不重置预算':'未建立'],['停车反馈',s.stop_confirmed===1?'实际反馈已确认':s.stop_requested===1?'已请求 · 等待发送及反馈':'未请求']])readout.append(el('dt',label),el('dd',display));
    const d=model.diag;diagnosticText.textContent=`目标运动 ${d.du??'—'}, ${d.dv??'—'} px/s\n前馈 ${d.ff??'—'} mm/s · 修正 ${d.correction??'—'} mm/s\n延迟联调值 ${d.latency??'—'} ms（不是实测拍摄延迟）\n实际批次 ${d.batch??'—'} ms · X反馈年龄 ${d.feedback_age??'—'} ms\n局部位移 ${d.forward??'—'}, ${d.left??'—'} mm\nRMS ${model.stats.rms_u??'—'}, ${model.stats.rms_v??'—'} px · 95% ${model.stats.p95_u??'—'}, ${model.stats.p95_v??'—'} px`;
    logStatus.textContent=`实时记录：${model.log.enabled?'已订阅（断线后需读状态）':'关闭或尚未读回'} · 本页收到 ${model.log.received} 条 · 中间缺号 ${model.log.gaps} 条。完整接收保存在 .embeddedskills/logs/serial/*-rx.bin；“导出本段”仍是最近128条。`;
    if(model.live?.io){const r=model.live,io=r.io;diagnosticText.textContent+=`\n实时请求 ${r.cmd} mm/s · 反馈推算 ${r.actual} mm/s\n已发送轮RPM ${io.slice(0,4).join(',')} · 位置年龄 ${io.slice(8,12).join(',')} ms\nX已发送 ${io[12]} 脉冲 / ${io[13]} RPM · TX年龄 ${io[14]} ms\n总线阶段 ${io[17]} / 错误 ${io[18]} / 锁定 ${io[19]}（发送不等于运动）`;}
    for(const [key,label]of readbacks)label.textContent=model.config.has(key)?`${model.current?'设备':'最近读回'}：${model.config.get(key)}`:'设备：未读取';
    for(const b of buttons)b.disabled=b.title==='local'?busy:!connected||!canSend(b.title)||(busy&&b!==stop);
  }
  function download(){const blob=new Blob([`OK grab track trace_begin count=${model.count}\n`+model.records.map(r=>r.raw).join('\n')+'\nOK grab track trace_end\n'],{type:'text/plain;charset=utf-8'});const url=URL.createObjectURL(blob),a=el('a');a.href=url;a.download=`tracking-${new Date().toISOString().replaceAll(':','-')}.log`;a.click();URL.revokeObjectURL(url);}
  render();return{model,readAll,apply,update:render,receive(line){
    const raw=line.trim().replace(/^(?:arm>\s*)+/,'');
    if(/^OK grab\b/.test(raw)&&/\boperation=pose\b/.test(raw)){
      const reason=raw.match(/\breason=(\S+)/)?.[1],state=raw.match(/\bstate=(\S+)/)?.[1];
      poseBusy=state==='route_pose';poseReply=reason==='work_ready'?'抓取姿态已到位 · 可以启动追踪':reason==='preparing_work'?'准备抓取姿态 · 等待 Z/X/Base 到位':`姿态准备：${reason??raw}`;
      render();return true;
    }
    if(/^ERR grab(?:_busy| pose\b)/.test(raw)){message.textContent=raw;render();return true;}
    const p=model.receive(line);if(!p)return false;
    if(p.state){poseReply='';poseBusy=false;}
    if(p.error){cancelTrace();message.textContent=p.raw;if(pending)cancel(p.raw);}if(p.type==='config'&&pending?.key===p.key){const request=pending;pending=null;clearTimeout(request.timer);request.resolve(p.value);}if(p.type==='row'&&tracePending?.index===p.i){const request=tracePending;tracePending=null;clearTimeout(request.timer);request.resolve(true);}if(p.type==='trace_end'&&exportRequested)finishTrace();render();return true;
  },connection(value){connected=value;model.current=false;poseReply='';poseBusy=false;cancelTrace();if(!value)cancel('连接已变化，参数操作取消；不会自动恢复追踪');render();}};
}
