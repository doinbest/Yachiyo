import {GRAB_FORM_DEFAULTS,GRAB_PARAMETER_METADATA,TRACK_PARAMETER_DEFAULTS,TRACK_COMPARISON_DEFAULTS} from './grab-parameter-metadata.mjs';
import {deviceText,rejectionText} from './device-text.mjs';
import {mountLogArchive} from './log-archive.mjs';
import {createParameterExchange} from './parameter-console.mjs';
export const TRACK_PRESET=Object.freeze(Object.fromEntries(Object.entries(TRACK_PARAMETER_DEFAULTS).filter(([key])=>!['body_accel','x_accel'].includes(key))));
const profileNames={predict:'原预测追踪',visual:'纯视觉追踪',feedback:'纯视觉＋反馈减速'};
const profileDefaults=profile=>profile==='predict'?TRACK_PARAMETER_DEFAULTS:TRACK_COMPARISON_DEFAULTS;
const labels={body_speed:'底盘合速度 / mm/s',x_speed:'伸缩速度 / mm/s',accel:'联合加速度 / mm/s²',lambda:'残余误差增益 / s⁻¹',x_weight:'伸缩分配权重',lead_mm:'最大领先 / mm',tol_px:'位置窗口 / px',control_ms:'控制间隔（新视觉包即时修正）/ ms',batch_ms:'额外批次限频（0为应答驱动）/ ms',latency_ms:'延迟联调初值 / ms',fit_ms:'拟合窗口 / ms',horizon_ms:'X位置目标前瞻 / ms',grace_ms:'短漏检宽限 / ms',body_accel:'底盘加减速联调值 / mm/s²',x_accel:'X加减速联调值 / mm/s²'};
const text={track_observe:'仅观察 · 不运动',track_match:'正在匹配速度',track_follow:'稳定跟随',track_pause:'保持姿态',idle:'空闲',stopping:'停车中',error:'故障停止',settle:'确认停稳',descend:'下降并持续对准',close:'闭爪',lift:'提起物料',complete:'动作完成',track_pick_stopping:'旧流程：夹取高度对准后停车确认',track_pick_descend:'高位进入夹取窗口 · 连续下降并追踪',track_pick_descend_wait:'旧流程：偏差超窗 · 暂停Z并继续追踪',track_pick_close:'Z已到夹取高度 · 闭爪并请求底盘/X停止',track_pick_reacquire:'停车时目标移动 · 继续追踪',track_pick_complete:'已夹取并提起 · 等待下一步',track_observing:'正在观察和估计',track_matching:'位置与速度匹配',track_following:'像素到点后仍持续跟随',track_visual_wait:'视觉暂停 · 新观测恢复后继续',track_window_end:'行程暂不足 · 保持观察，目标返回可追范围后继续',track_window_recovered:'目标已返回可追范围 · 自动续追',track_recovered:'视觉已恢复',cancelled:'人工停止',stop_unconfirmed:'停车尚未取得反馈确认'};
export const trackText=value=>text[value]??deviceText(value);
export function trackCrc(line){
  let crc=0xffffffff;
  for(const c of line){crc^=c.charCodeAt(0);for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}
  return ((crc^0xffffffff)>>>0).toString(16).padStart(8,'0').toUpperCase();
}
export function parsePixel(raw){
  const line=raw.trim().replace(/^(?:arm>\s*)+/,'');
  const marker=line.match(/^OK camera log_(begin|end|status)\b/);
  if(!marker&&!line.startsWith('PIX id='))return null;
  const p={type:marker?`pixel_${marker[1]}`:'pixel',raw:line};
  for(const m of line.matchAll(/\b([a-z_]+)=(\d+)\b/g))p[m[1]]=Number(m[2]);
  const errors=Object.fromEntries([...line.matchAll(/\b(dx|dy)=([^\s]+)/g)].map(m=>[m[1],m[2]]));
  if(Object.keys(errors).length){
    if(!['dx','dy'].every(k=>/^-?\d+$/.test(errors[k]??'')&&Number(errors[k])>=-32768&&Number(errors[k])<=32767))return null;
    p.dx=Number(errors.dx);p.dy=Number(errors.dy);
  }
  if(!marker){const body=line.match(/^(.*) crc=([\da-f]{8})$/i);
    if(!body||trackCrc(body[1])!==body[2].toUpperCase()||!['id','i','t','c','v','m','x','y'].every(k=>Number.isInteger(p[k]))||p.id<1||![0,1].includes(p.v)||![0,1].includes(p.m))return null;
  }
  return p;
}
export function parseTrack(raw){
  const line=raw.trim().replace(/^(?:arm>\s*)+/,'');
  if(!/^(?:OK|ERR) grab track\b/.test(line)&&!/^TRACK (?:row|live)\b/.test(line))return null;
  const type=line.startsWith('TRACK live')?'live':line.startsWith('TRACK row')?'row':line.match(/^OK grab track (config|diag|stats|trace_begin|trace_end|log_begin|log_end|log_status)\b/)?.[1]??'state';
  const result={type,raw:line,error:line.startsWith('ERR')};
  for(const match of line.matchAll(/\b([a-z_0-9]+)=([^\s=]+)/g)){
    const value=match[2];result[match[1]]=/^(?:nan|na)$/.test(value)?null:/^[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?$/i.test(value)?Number(value):value;
  }
  if(type==='state'&&result.base_home_degraded!==undefined&&![0,1].includes(result.base_home_degraded))return null;
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
  constructor(){this.config=new Map();this.state={};this.diag={};this.stats={};this.records=[];this.complete=false;this.current=false;this.count=null;this.ended=false;this.log={enabled:false,gaps:0,received:0};this.pixels={enabled:false,received:0,gaps:0};this.live=null;}
  missing(){return this.count===null?[]:Array.from({length:this.count},(_,i)=>i).filter(i=>!this.records.some(r=>r.i===i));}
  receive(line){
    const pixel=parsePixel(line);
    if(pixel){
      const p=this.pixels;
      if(pixel.type==='pixel_begin'){
        if(p.id!==pixel.id||(pixel.next===0&&p.received))this.pixels={id:pixel.id,received:0,gaps:0,next:pixel.next??0};
        this.pixels.enabled=true;
      }else if(pixel.type==='pixel_status'){
        if(p.id!==pixel.id)this.pixels={id:pixel.id,received:0,gaps:0,next:null};
        Object.assign(this.pixels,{enabled:pixel.enabled===1,count:pixel.count,dropped:pixel.dropped});
      }else if(pixel.type==='pixel_end'&&pixel.id===p.id){
        Object.assign(p,{enabled:false,count:pixel.count,dropped:pixel.dropped});
        if(p.next!==null)p.gaps+=Math.max(0,pixel.count-(p.next??0));
      }else if(pixel.type==='pixel'&&pixel.id===p.id&&(p.next===null||pixel.i>=(p.next??0))){
        p.gaps+=p.next===null?0:Math.max(0,pixel.i-(p.next??0));p.next=pixel.i+1;p.received++;p.last=pixel;
      }
      return pixel;
    }
    const p=parseTrack(line);if(!p)return null;
    if(p.type==='config'&&typeof p.value==='number'){
      const profile=p.profile??'predict';
      this.config.set(profile==='predict'?p.key:`${profile}.${p.key}`,p.value);
    }
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
  const pickupExchange=createParameterExchange({send});let pickupRevision=0;
  let transferBusy=false,transferReply='';
  const el=(tag,value,cls)=>{const e=document.createElement(tag);if(value!==undefined)e.textContent=value;if(cls)e.className=cls;return e;};
  const card=el('section',undefined,'tracking-card');card.id='grab-material-tracking';
  const header=el('div',undefined,'tracking-header');header.append(el('h3','物料夹取与入仓'),el('span','地面 / 物料台转盘','tracking-badge'));card.append(header);
  card.append(el('p','选择物料颜色和目标仓位。地面夹取沿用比赛加工取回：高位静态对准后下降夹取；转盘夹取沿用原料追踪：高位DX/DY进入窗口后连续下降，Z到位即闭爪。随后由STM32完成提起、收X、分度、回仓与下降放料。','form-note'));
  const color=el('select');color.setAttribute('aria-label','追踪颜色');
  for(const [value,label]of [[1,'红'],[2,'黄'],[3,'蓝'],[4,'绿'],[5,'黑'],[6,'浅蓝']]){const o=el('option',label);o.value=value;color.append(o);}color.value=3;
  const actions=el('div',undefined,'actions');actions.append(color);
  const statusGuide=el('p','准备请求被拒绝时，点击“读取抓取任务状态”，在下方“抓取任务状态”查看原因、缺少参数与参考状态。','form-note');
  const statusLink=el('a','查看抓取任务状态反馈 →','context-link');statusLink.href='#vision/grab/status';statusGuide.append(statusLink);
  const status=el('p','尚无设备追踪状态','tracking-state'),readout=el('dl',undefined,'tracking-readout'),message=el('p','参数草稿与设备读回独立；本页不会自动发送指令。','form-note');
  const diagnostics=el('details',undefined,'advanced');diagnostics.append(el('summary','运动估计与实际节奏'));const diagnosticText=el('pre','未报告');diagnostics.append(diagnosticText);
  const parameters=el('details',undefined,'advanced');parameters.append(el('summary','独立追踪参数 · 显式写入并读回'));
  const parameterGrid=el('div',undefined,'tracking-parameters');
  const parameterProfile=el('select');parameterProfile.setAttribute('aria-label','追踪参数模式');
  for(const [value,name]of Object.entries(profileNames)){const option=el('option',name);option.value=value;parameterProfile.append(option);}parameterProfile.value='predict';
  parameters.append(parameterProfile,el('p','三个模式的参数独立保存。纯视觉与反馈减速默认底盘160、X60 mm/s，增益2.0、X前瞻60 ms；原预测组保持原初值。两个验证模式只跟随；反馈模式加减速值为软件联调初值，驱动档位仍按原配置。','form-note'));
  const parameterRows=new Map();
  for(const [key,value]of Object.entries(TRACK_PARAMETER_DEFAULTS)){
    const label=el('label',labels[key]),input=el('input'),readback=el('small','设备：未读取');
    input.type='number';input.step='any';input.value=value;input.dataset.trackKey=key;inputs.set(key,input);readbacks.set(key,readback);parameterRows.set(key,label);label.append(input,readback);parameterGrid.append(label);
  }
  parameters.append(parameterGrid);const parameterActions=el('div',undefined,'actions');parameters.append(parameterActions);
  function button(parent,label,wire,operation){const b=el('button',label);b.type='button';b.title=wire;parent.append(b);buttons.push(b);b.onclick=async()=>{try{await operation();}catch(e){message.textContent=e.message;}render();};return b;}
  const trackingOnly=el('details',undefined,'advanced material-tracking-tools');
  trackingOnly.append(el('summary','单独追踪与夹取'),el('p','共用上方颜色和夹取条件。以下操作不自动入仓；需要入仓时使用上方完整流程。','form-note'));
  const trackingFields=el('div',undefined,'tracking-parameters'),trackingActions=el('div',undefined,'actions');
  trackingOnly.append(trackingFields,trackingActions);
  const pickup=el('div',undefined,'tracking-parameters');
  function numberField(label,value,step,min,max,parent=pickup){const wrap=el('label',label),input=el('input');input.type='number';input.step=step;input.min=min;input.max=max;input.value=value;wrap.append(input);parent.append(wrap);return input;}
  const budget=numberField('首次建立参考：剩余接近余量 / mm',50,1,0,50,trackingFields);
  const groundZ=numberField('地面放取 Z 绝对高度 / mm',GRAB_FORM_DEFAULTS.z_proc_place,1,-150,0);groundZ.dataset.pickupSource='ground';
  const pickupZ=numberField('转盘夹取 Z 绝对高度 / mm',GRAB_FORM_DEFAULTS.z_grab,1,-150,0);pickupZ.dataset.pickupSource='turntable';
  const pickupSpeed=numberField('Z统一速度 / mm/s（下降、抬升、放置、扫码）',GRAB_FORM_DEFAULTS.z_speed,1,GRAB_PARAMETER_METADATA.z_speed.min,GRAB_PARAMETER_METADATA.z_speed.max);
  pickupSpeed.exclusiveMin=GRAB_PARAMETER_METADATA.z_pick_speed.exclusiveMin;
  const pickupConditions=new Map();
  for(const [key,label]of [['track_pick_tol_px','夹取误差窗口 / ±px'],['track_pick_hold_ms','高位窗口保持时间 / ms（0为立即下降）']]){
    const meta=GRAB_PARAMETER_METADATA[key],input=numberField(label,GRAB_FORM_DEFAULTS[key],meta.step,meta.min,meta.max);
    input.dataset.pickupKey=key;pickupConditions.set(key,input);
  }
  const pickupReadback=el('p','夹取条件：尚未读取设备；输入为草稿。','form-note');
  const pickupValues=new Map();
  const lift=numberField('转盘夹取后上升距离 / mm',GRAB_FORM_DEFAULTS.z_lift-GRAB_FORM_DEFAULTS.z_grab,1,0.1,150);lift.dataset.transferKey='lift';
  const drop=numberField('入仓时：从转移高度下降距离 / mm',GRAB_FORM_DEFAULTS.track_store_drop_mm,1,0.1,150);drop.dataset.transferKey='drop';
  const slotLabel=el('label','目标车仓'),storeSlot=el('select');storeSlot.dataset.storeSlot='1';
  for(const [value,label]of [['0','自动：最低编号空仓'],['1','1号料盘'],['2','2号料盘'],['3','3号料盘']]){const option=el('option',label);option.value=value;storeSlot.append(option);}storeSlot.value='1';slotLabel.append(storeSlot);pickup.append(slotLabel);
  const profileLabel=el('label','转盘夹取追踪方式'),storeProfile=el('select');storeProfile.setAttribute('aria-label','入仓夹取追踪方式');
  for(const [value,label]of Object.entries(profileNames)){const option=el('option',label);option.value=value;storeProfile.append(option);}storeProfile.value='feedback';profileLabel.append(storeProfile);pickup.append(profileLabel);
  function showPickupReadback(){pickupReadback.textContent=`设备最近读回：Z抬升/放置 ${pickupValues.get('z_speed')??'未读取'}、夹取 ${pickupValues.get('z_pick_speed')??'未读取'}、扫码 ${pickupValues.get('qr_z_speed')??'未读取'} mm/s；窗口 ±${pickupValues.get('track_pick_tol_px')??'未读取'} px，保持 ${pickupValues.get('track_pick_hold_ms')??'未读取'} ms。`;}
  const number=(input,label)=>{const value=Number(input.value);if(input.value===''||!Number.isFinite(value)||value<Number(input.min)||(input.exclusiveMin&&value===Number(input.min))||value>Number(input.max))throw new Error(`${label}输入无效`);return value;};
  const zSpeedValues=()=>{const speed=number(pickupSpeed,'Z统一速度');return ['z_speed','z_pick_speed','qr_z_speed'].map(key=>[key,speed]);};
  button(actions,'准备抓取姿态','grab pose work',async()=>{if(await send('grab pose work')){poseReply='抓取姿态已请求 · 等待设备确认到位';poseBusy=true;}});
  button(actions,'读取抓取任务状态','grab status',()=>send('grab status'));
  button(trackingActions,'预测追踪','grab track begin',()=>send(`grab track begin ${color.value} ${number(budget,'接近余量')}`));
  button(trackingActions,'纯视觉追踪','grab track visual',()=>send(`grab track visual ${color.value} ${number(budget,'接近余量')}`));
  button(trackingActions,'纯视觉＋反馈减速','grab track feedback',()=>send(`grab track feedback ${color.value} ${number(budget,'接近余量')}`));
  button(trackingActions,'只追踪夹取（不入仓）','grab track pick',async()=>{
    const wire=`grab track pick ${color.value} ${number(pickupZ,'夹取高度')} ${number(pickupSpeed,'下降速度')}`,revision=pickupRevision;
    await applyPickup();
    if(revision!==pickupRevision||!connected||!canSend(wire))throw new Error('操作状态已变化，未启动夹取；请重新操作。');
    await send(wire);
  });
  const startStore=button(actions,'转盘夹取并入仓','grab track pickstore',async()=>{
    const revision=pickupRevision,wire=`grab track pickstore ${color.value} ${storeSlot.value} ${storeProfile.value}`;
    await applyTransfer(true);
    if(revision!==pickupRevision||!connected||!canSend(wire))throw new Error('操作状态已变化，未启动入仓；请重新操作。');
    await send(wire);
  });
  startStore.className='primary';
  const groundStore=button(actions,'地面夹取并入仓','grab store ground',async()=>{
    const revision=pickupRevision,wire=`grab store ground ${storeSlot.value} ${color.value}`;
    await applyGroundTransfer();
    if(revision!==pickupRevision||!connected||!canSend(wire))throw new Error('操作状态已变化，未启动入仓；请重新操作。');
    await send(wire);
  });
  groundStore.className='primary';
  button(trackingActions,'读取Z速度与夹取条件','grab get track_pick_tol_px',readPickup);
  button(trackingActions,'应用Z速度与夹取条件','grab set track_pick_tol_px 100',applyPickup);
  const stop=button(actions,'停止追踪 / 入仓','grab track stop',()=>{cancelTrace();cancel('参数操作已取消');return send(poseBusy||transferBusy?'grab stop':'grab track stop');});stop.className='danger';
  const trackStatus=button(trackingActions,'读取追踪状态','grab track status',()=>send(poseBusy||transferBusy?'grab status':'grab track status'));
  const logActions=el('div',undefined,'actions'),logStatus=el('p','实时记录默认关闭；启用后由本地服务持续写入磁盘，不会启动机构。','form-note');
  const logPeriod=el('select');logPeriod.setAttribute('aria-label','实时记录间隔');
  for(const ms of [200,100,50,500,1000]){const option=el('option',`速度诊断 ${ms} ms / ${1000/ms} Hz`);option.value=ms;logPeriod.append(option);}logPeriod.value=200;
  const motorLabel=el('label','保留速度诊断'),motorLog=el('input');motorLog.type='checkbox';motorLog.checked=true;motorLabel.append(motorLog);logActions.append(motorLabel,logPeriod);
  button(logActions,'开启实时记录','grab track log start',async()=>{
    const period=Number(logPeriod.value);
    if(![50,100,200,500,1000].includes(period))throw new Error('记录间隔无效');
    let parameters='设备参数已读回';
    if(!model.pixels.enabled){try{await readAll(model.current?(model.state.mode??'predict'):parameterProfile.value);}catch(error){parameters=`参数读回不完整（${error.message}）`;}}
    if(!await send('camera log start'))return;
    const speedOk=await send(motorLog.checked?`grab track log start ${period}`:'grab track log stop');
    message.textContent=`已请求逐帧像素记录（不按10/20 Hz抽样）；${parameters}。${motorLog.checked&&speedOk?`另保留${1000/period} Hz速度诊断。`:motorLog.checked?'速度诊断未确认，请读记录状态。':'本次仅记录像素。'}整段持续落盘，可检查缺条与坏校验。`;
  });
  button(logActions,'关闭实时记录','grab track log stop',async()=>{try{await send('grab track log stop');}finally{await send('camera log stop');}});
  button(logActions,'读取记录状态','grab track log status',async()=>{await send('camera log status');await send('grab track log status');});
  button(logActions,'导出本段','grab track trace',async()=>{
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
  async function query(key,wire,profile='predict'){
    if(pending)throw new Error('已有参数请求，请等待');
    return new Promise((resolve,reject)=>{
      pending={key,profile,resolve,reject,timer:setTimeout(()=>{pending=null;reject(new Error(`追踪参数${key}读回超时`));},6000)};
      Promise.resolve(send(wire)).then(ok=>{if(!ok)cancel('未发送追踪参数');}).catch(e=>cancel(e.message));
    });
  }
  function cancel(reason){pickupRevision++;pickupExchange.cancel(reason);if(pending){clearTimeout(pending.timer);const p=pending;pending=null;p.reject(new Error(reason));}busy=false;}
  async function readPickup(){
    if(busy)throw new Error('已有参数请求，请等待');
    busy=true;render();try{
      for(const [key,input]of pickupConditions){const result=await pickupExchange.readGrab(key);pickupValues.set(key,result.value);input.value=result.value;}
      for(const key of ['z_speed','z_pick_speed','qr_z_speed']){const result=await pickupExchange.readGrab(key);pickupValues.set(key,result.value);}
      showPickupReadback();message.textContent='Z速度与夹取条件已读回；输入速度仍是草稿，未发送运动命令。';
    }finally{busy=false;render();}
  }
  async function applyPickup(){
    if(busy)throw new Error('已有参数请求，请等待');
    const values=[...pickupConditions].map(([key,input])=>{
      const value=number(input,key==='track_pick_tol_px'?'夹取误差窗口':'夹取保持时间');
      if(key==='track_pick_hold_ms'&&!Number.isInteger(value))throw new Error('夹取保持时间必须为整数毫秒');
      return[key,value];
    }).concat(zSpeedValues());
    busy=true;render();try{
      for(const [key,value]of values){const result=await pickupExchange.setGrab(key,value);if(!result.confirmed)throw new Error('夹取条件读回不一致，未启动夹取');pickupValues.set(key,result.value);}
       showPickupReadback();message.textContent='Z统一速度与夹取条件已写入并读回确认。';
    }finally{busy=false;render();}
  }
  async function applyTransfer(withPickup){
    if(busy)throw new Error('已有参数请求，请等待');
    const revision=pickupRevision,z=number(pickupZ,'夹取高度'),rise=number(lift,'上升距离'),fall=number(drop,'入仓下降距离'),target=z+rise;
    if(target>0||target<-150||target-fall<-150)throw new Error('上升或入仓下降目标超出Z行程−150～0 mm');
    if(!['0','1','2','3'].includes(String(storeSlot.value))||!Object.hasOwn(profileNames,storeProfile.value))throw new Error('请选择仓位与追踪方式');
    const values=withPickup?[['z_grab',z],['z_lift',target],['track_store_drop_mm',fall]]:[['z_lift',target],['track_store_drop_mm',fall]];
    if(withPickup)await applyPickup();
    if(revision!==pickupRevision||!connected)throw new Error('参数操作已取消，未启动入仓');
    busy=true;render();try{
      for(const [key,value]of values){const result=await pickupExchange.setGrab(key,value);if(!result.confirmed)throw new Error('入仓参数读回不一致，未启动任务');}
      message.textContent=`入仓参数已确认：提起到${target} mm，放料到${target-fall} mm。`;
    }finally{busy=false;render();}
  }
  async function applyGroundTransfer(){
    if(busy)throw new Error('已有参数请求，请等待');
    const revision=pickupRevision,z=number(groundZ,'地面夹取高度'),fall=number(drop,'入仓下降距离');
    const target=GRAB_FORM_DEFAULTS.z_car_lift;
    if(z>=target||target-fall<-150||!['0','1','2','3'].includes(String(storeSlot.value)))throw new Error('地面夹取或入仓高度、仓位无效');
    const values=[['z_proc_place',z],...zSpeedValues(),['z_car_lift',target],['z_place',target-fall]];
    busy=true;render();try{
      for(const [key,value]of values){
        if(revision!==pickupRevision||!connected)throw new Error('参数操作已取消，未启动入仓');
        const result=await pickupExchange.setGrab(key,value);if(!result.confirmed)throw new Error('地面入仓参数读回不一致，未启动任务');
        if(['z_speed','z_pick_speed','qr_z_speed'].includes(key))pickupValues.set(key,result.value);
      }
      showPickupReadback();
      message.textContent=`地面参数已确认：夹取${z} mm，抬升到${target} mm，入仓放料${target-fall} mm。`;
    }finally{busy=false;render();}
  }
  const profileKeys=profile=>Object.keys(TRACK_PARAMETER_DEFAULTS).filter(key=>!(['body_accel','x_accel'].includes(key)&&profile!=='feedback') && !(profile!=='predict'&&['latency_ms','fit_ms'].includes(key)));
  const suffix=profile=>profile==='predict'?'':` ${profile}`;
  async function readAll(selected){const profile=typeof selected==='string'?selected:parameterProfile.value;busy=true;render();try{for(const key of profileKeys(profile))await query(key,`grab track get ${key}${suffix(profile)}`,profile);message.textContent=`${profileNames[profile]}参数已读回；未发送运动命令。`;}finally{busy=false;render();}}
  async function apply(){
    const profile=parameterProfile.value;
    busy=true;render();try{
      for(const key of profileKeys(profile)){const input=inputs.get(key),value=Number(input.value);if(!Number.isFinite(value)||input.value==='')throw new Error(`${labels[key]}输入无效`);const written=await query(key,`grab track set ${key} ${value}${suffix(profile)}`,profile);const read=await query(key,`grab track get ${key}${suffix(profile)}`,profile);if(Math.abs(read-value)>Math.max(1e-5,Math.abs(value)*1e-5)||Math.abs(read-written)>1e-5)throw new Error(`${key}写后读回不一致`);}
      message.textContent=`${profileNames[profile]}参数已写入RAM并读回一致。`;
    }finally{busy=false;render();}
  }
  button(parameterActions,'读取全部','grab track get body_speed',readAll);
  button(parameterActions,'应用并核对','grab track set body_speed 160',apply);
  button(parameterActions,'填入追踪预设','local',()=>{for(const [key,value]of Object.entries(profileDefaults(parameterProfile.value)))inputs.get(key).value=value;message.textContent='预设只填入草稿，尚未写入设备。';});
  parameterProfile.onchange=()=>{cancel('参数模式已切换');for(const [key,input]of inputs)input.value=profileDefaults(parameterProfile.value)[key];message.textContent=`正在编辑${profileNames[parameterProfile.value]}参数；请显式读取设备值。`;render();};
  card.append(actions,statusGuide,pickup,pickupReadback,el('p','地面默认Z−110 mm，静态对准后夹取并抬至Z0；转盘默认Z−45 mm，追踪夹取后按所填距离抬升。入仓默认从转移高度下降25 mm。夹取下降速度默认250 mm/s，使用比赛同一Z驱动加速度配置（默认档位0，直接按目标速度运行）。转盘下降窗口默认±100 px、高位保持0 ms；地面采用静态对准稳定条件。两种入口均先写入并核对所需参数，停止和断线不自动续发；自动选仓按库存，指定仓沿用原规则。','form-note'),status,readout,message);
  const records=el('details',undefined,'advanced material-records');records.append(el('summary','追踪记录与诊断'),logActions,logStatus,diagnostics);mountLogArchive({root:records});
  card.append(trackingOnly,parameters,records);root.append(card);
  function render(){
    const s=model.state,old=model.current?'':'历史 · ',value=key=>s[key]??'—';
    const mode=s.mode??'predict',visual=mode!=='predict';
    const phase=visual&&s.state==='track_match'?'视觉纠偏':visual&&s.state==='track_follow'?'对准保持 · 继续跟随':trackText(s.state);
    const reason=visual&&s.reason==='track_observing'?'等待视觉与停稳反馈':visual&&s.reason==='track_matching'?'按新坐标纠偏':trackText(s.reason);
    status.textContent=poseReply||transferReply||(s.state?`${old}${profileNames[mode]??mode} · ${phase} · ${reason}${s.base_home_degraded===1?' · Base 使用降级参考':''}`:'尚无设备追踪状态');
    readout.replaceChildren();
    for(const [label,display]of [['目标像素 / 误差',s.valid===1?`${value('cx')}, ${value('cy')} / ${value('dx')}, ${value('dy')} px`:'未取得新鲜坐标'],['命令前向 / 左向 / X速度',`${value('vf')} / ${value('vl')} / ${value('vx')} mm/s`],['X位置 / 最近端剩余',`${value('x')} / ${value('xr')} mm`],['底盘剩余接近预算',s.ref===1?`${value('room')} mm`:'未建立参考'],['料台参考',s.ref===1?'已建立 · 停止不重置预算':'未建立'],['停车反馈',s.stop_confirmed===1?'实际反馈已确认':s.stop_requested===1?'已请求 · 等待发送及反馈':'未请求']])readout.append(el('dt',label),el('dd',display));
    const d=model.diag;diagnosticText.textContent=`目标运动 ${d.du??'—'}, ${d.dv??'—'} px/s\n前馈 ${d.ff??'—'} mm/s · 修正 ${d.correction??'—'} mm/s\n延迟联调值 ${d.latency??'—'} ms（不是实测拍摄延迟）\n实际批次 ${d.batch??'—'} ms · 实测反馈最大年龄 ${d.feedback_age??'—'} ms\n控制用局部坐标 ${d.forward??'—'}, ${d.left??'—'} mm\nRMS ${model.stats.rms_u??'—'}, ${model.stats.rms_v??'—'} px · 95% ${model.stats.p95_u??'—'}, ${model.stats.p95_v??'—'} px`;
    logStatus.textContent=`逐帧像素：${model.pixels.enabled?'已订阅':'关闭或尚未读回'} · 本页收到 ${model.pixels.received} 帧 · 缺号 ${model.pixels.gaps} 帧；速度诊断收到 ${model.log.received} 条。像素随每个B2帧记录，间隔选项只控制速度诊断。本页计数受刷新影响；整段数据和完整性以“整段记录与完整 TXT 导出”为准。“导出本段”仍是最近128条。`;
    if(model.live?.io){const r=model.live,io=r.io;diagnosticText.textContent+=`\n实时请求 ${r.cmd} mm/s · 反馈推算 ${r.actual} mm/s\n已发送轮RPM ${io.slice(0,4).join(',')} · 位置年龄 ${io.slice(8,12).join(',')} ms\nX已发送 ${io[12]} 脉冲 / ${io[13]} RPM · TX年龄 ${io[14]} ms\n总线阶段 ${io[17]} / 错误 ${io[18]} / 锁定 ${io[19]}（发送不等于运动）`;}
    const profile=parameterProfile.value;
    for(const [key,label]of readbacks){const stored=profile==='predict'?key:`${profile}.${key}`;label.textContent=model.config.has(stored)?`最近读回：${model.config.get(stored)}`:'设备：未读取';parameterRows.get(key).hidden=!profileKeys(profile).includes(key);}
    parameterProfile.disabled=busy;
    stop.textContent=poseBusy?'停止准备':'停止追踪 / 入仓';
    trackStatus.textContent=poseBusy?'读取准备状态':transferBusy?'读取入仓状态':'读取追踪状态';
    for(const b of buttons)b.disabled=b.title==='local'?busy:!connected||!canSend(b.title)||(busy&&b!==stop);
  }
  function download(){const blob=new Blob([`OK grab track trace_begin count=${model.count}\n`+model.records.map(r=>r.raw).join('\n')+'\nOK grab track trace_end\n'],{type:'text/plain;charset=utf-8'});const url=URL.createObjectURL(blob),a=el('a');a.href=url;a.download=`tracking-${new Date().toISOString().replaceAll(':','-')}.log`;a.click();URL.revokeObjectURL(url);}
  render();return{model,readAll,apply,update:render,receive(line){
    const raw=line.trim().replace(/^(?:arm>\s*)+/,'');
    if(pickupExchange.receive(raw)){render();return true;}
    if(/^(?:OK|ERR) grab\b/.test(raw)&&/\boperation=store\b/.test(raw)){
      const state=raw.match(/\bstate=(\S+)/)?.[1],slot=raw.match(/\bslot=(\S+)/)?.[1],result=raw.match(/\bresult=(\S+)/)?.[1],reason=raw.match(/\breason=(\S+)/)?.[1];
      transferBusy=!['idle','complete','error'].includes(state);poseReply='';poseBusy=false;
      transferReply=`${state==='complete'&&result==='stored'?'入仓完成':state==='idle'?'入仓已停止，可显式继续':state==='error'?'入仓故障，可排除后继续':'物料入仓'} · 仓${slot??'—'} · ${deviceText(state)} · ${deviceText(reason)}${result==='stored'?' · 已记录库存':''}`;
      render();return true;
    }
    if(/^OK grab\b/.test(raw)&&/\boperation=pose\b/.test(raw)){
      const reason=raw.match(/\breason=(\S+)/)?.[1],state=raw.match(/\bstate=(\S+)/)?.[1];
      const stopping=state==='stopping'||/\bstop_requested=1\b/.test(raw);
      poseBusy=['route_pose','homing','stopping'].includes(state);
      poseReply=stopping&&poseBusy?`姿态准备停止中 · ${deviceText(reason)} · ${state==='homing'?'等待回零退出确认':'等待停稳确认'}`:reason==='work_ready'?'抓取姿态已到位 · 可以启动追踪':reason==='preparing_work'?'准备抓取姿态 · 等待 Z/X/Base 到位':state==='homing'?'准备抓取姿态 · 确认 Base 回零与停稳反馈':`姿态准备：${deviceText(reason)}`;
      render();return true;
    }
    if(/^ERR grab(?:_busy| pose\b)/.test(raw)){message.textContent=rejectionText(raw);render();return true;}
    const p=model.receive(line);if(!p)return false;
    if(p.state){poseReply='';poseBusy=false;transferReply='';transferBusy=false;}
    if(p.error){cancelTrace();message.textContent=rejectionText(p.raw);if(pending)cancel(rejectionText(p.raw));}if(p.type==='config'&&pending?.key===p.key&&pending.profile===(p.profile??'predict')){const request=pending;pending=null;clearTimeout(request.timer);request.resolve(p.value);}if(p.type==='row'&&tracePending?.index===p.i){const request=tracePending;tracePending=null;clearTimeout(request.timer);request.resolve(true);}if(p.type==='trace_end'&&exportRequested)finishTrace();render();return true;
  },connection(value){connected=value;model.current=false;poseReply='';poseBusy=false;transferReply='';transferBusy=false;cancelTrace();if(!value)cancel('连接已变化，参数操作取消；不会自动恢复追踪');render();}};
}
