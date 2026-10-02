import {bootProgress,describeStopEvent,visualStateText} from './terminal-format.mjs?v=grab-terminal-20261002';
import {visibleTerminalLogs,createLogRenderScheduler} from './terminal-log.mjs?v=vision-pause-20260929';
import {mountWheelFeedback} from './wheel-feedback.mjs';
import {createAttemptTracker} from './command-attempt.mjs?v=terminal-3';
import {mountPreparation} from './route-preparation.mjs?v=route-20260925';
import {commands,buildCommand,buildGrabPresetCommands,frameCommand,describeReply,formatTerminalLine,replyIsFault,moduleForWire,grabParameters,grabColorOptions,parseGrabStatus,parseGrabRouteStatus,describeGrabMissing,grabPixelText,grabRecoveryText} from './protocol.mjs?v=grab-terminal-20261002';
import {BridgeLink,readOnlyCommand,scopedStopCommand} from './bridge.mjs?v=vision-pause-20260929';
import {mountMap} from './map-view.mjs?v=route-20260925';
import {mountQr} from './qr-panel.mjs?v=bridge-1';
import {mountParameterPage} from './parameter-page.mjs?v=parameter-page-20261001';
import {RadarModel,RadarExchange,parseRadar,radarStateText,radarReasonText} from './radar-model.mjs';
import {mountRadarWorkspace} from './radar-view.mjs';
const $=id=>document.getElementById(id);
const node=(tag,text,cls)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;};
const titles={system:'系统与标定',chassis:'底盘',map:'地图与遥测',qr:'二维码',vision:'视觉任务',params:'参数调试',arm:'机械臂与夹爪'};
const storageKey='sky-atelier-v1';let prefs={};try{prefs=JSON.parse(localStorage.getItem(storageKey)||'{}')||{};}catch{}
$('quiet-grab-log').checked=prefs.quietGrabLog!==false;
$('log-format').value=prefs.logFormat==='raw'?'raw':'chinese';
const chineseLogs=()=>$('log-format').value!=='raw';
const displayReply=line=>chineseLogs()?formatTerminalLine(line):line;
const renderLogs=createLogRenderScheduler(renderLogsNow,fn=>requestAnimationFrame(fn),()=>document.hidden);
document.addEventListener('visibilitychange',()=>{if(!document.hidden)renderLogs();});
function save(){try{localStorage.setItem(storageKey,JSON.stringify(prefs));}catch{toast('本机存储已满，设置未保存。');}}
let toastTimer;function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,3500);}
let active='system',axis='base',mapView,radarView,qrView,prepView,wheelView,paramsView,txCount=0,rxCount=0,filter='ALL',logs=[],unseen=0,errors=0,sending=false,applyingGrabPreset=false,presetCancelled=false,parameterBusy=false;
const radarModel=new RadarModel();try{const saved=JSON.parse(localStorage.getItem('local-car-radar-snapshots')||'null');if(saved){radarModel.snapshots=saved.snapshots??{};radarModel.params=saved.params??{};}}catch{}
let savedRadarSnapshot=null;
const radarExchange=new RadarExchange({send:wire=>executeSend(wire,false,true)});
const status=new Map(),forms=[],pages={};
let grabSnapshot=null,grabRouteSnapshot=null,grabReply=null,grabPanel=null;
function appendLog(direction,text){
  const entry={time:new Date().toLocaleTimeString('zh-CN',{hour12:false}),direction,text,owner:moduleForWire(text)};
  logs.push(entry);if(logs.length>500)logs.shift();
  if(direction==='TX')txCount++;if(direction==='RX')rxCount++;if(replyIsFault(text))errors++;
  if(!$('autoscroll').checked&&visibleTerminalLogs(logs,$('quiet-grab-log').checked,chineseLogs()).includes(entry))unseen++;
  renderLogs();
}
function renderLogsNow(){if(active==='map'&&!document.querySelector('.desk').classList.contains('logs-open'))return;const scroll=$('terminal').scrollTop;const scope=$('log-module').value;const visible=visibleTerminalLogs(logs,$('quiet-grab-log').checked,chineseLogs());const hidden=logs.length-visible.length;$('grab-log-hidden').hidden=!hidden;$('grab-log-hidden').textContent=`已隐藏 ${hidden} 条重复报告或回显`;const entries=visible.filter(l=>(filter==='ALL'||l.direction===filter)&&(scope==='all'||l.owner===(scope==='current'?active:scope)||l.direction==='SYS'));const fragment=document.createDocumentFragment();for(const l of entries){const row=node('div',undefined,`log-entry ${l.direction.toLowerCase()} ${replyIsFault(l.text)?'error':''}`);row.append(node('time',l.time),node('b',l.direction),node('span',l.direction==='RX'?displayReply(l.text):l.text));fragment.append(row);}if(!entries.length)fragment.append(node('p','本模块尚无收发记录，可切换“全部模块”查看。','empty-log'));$('terminal').replaceChildren(fragment);$('log-count').textContent=`${logs.length}${errors?' · 错误 '+errors:''}`;$('tx-count').textContent=txCount;$('rx-count').textContent=rxCount;$('terminal').scrollTop=$('autoscroll').checked?$('terminal').scrollHeight:scroll;$('new-logs').hidden=!unseen;$('new-logs').textContent=`${unseen} 条新记录 · 恢复跟随`;}
let routeLocked=false,bootStage='';
let connectionState='disconnected',lastStopEvent=null;
function receive(line){
  radarView?.receive(line);
  const snapshot=Object.values(radarModel.snapshots).map(s=>s.receivedAt).join(':');if(snapshot!==savedRadarSnapshot){savedRadarSnapshot=snapshot;try{localStorage.setItem('local-car-radar-snapshots',JSON.stringify({snapshots:radarModel.snapshots,params:radarModel.params}));}catch{}}
  if(line.startsWith('@RADAR ')){
    const frame=parseRadar(line);if(frame&&['status','nav'].includes(frame.k)){
      const text=`雷达 ${frame.k} · ${radarStateText(frame.state)} · ${radarReasonText(frame.reason)} · 地图 ${frame.map} / 路径 ${frame.plan}`;
      $('map-recent-result').textContent=text;status.set('map',{line:text,time:Date.now(),live:bridge.connected});if(frame.k==='nav')routeLocked=['running','stopping','waiting','station'].includes(frame.state);renderStatus();updateControls();
    }return;
  }
  if(radarExchange.busy&&(line==='OK radar page_pending'||/^(?:arm>\s*)?radar fetch /.test(line)))return;
  appendLog('RX',line);
  if(/^(OK|ERR) radar\b/.test(line))$('map-recent-result').textContent=`${line.startsWith('ERR')?'设备未受理':'设备回复'} · ${radarReasonText(line.split(' ')[2])}`;
  paramsView?.receive(line);
  const progress=bootProgress(line,bootStage);if(progress!==bootStage){bootStage=progress;$('boot-status').textContent=progress;}
  if(/^(OK|ERR) grab\b/.test(line)){grabReply={line,time:Date.now(),live:bridge.connected};const routeSnapshot=parseGrabRouteStatus(line);if(routeSnapshot)grabRouteSnapshot={data:routeSnapshot,time:Date.now(),live:bridge.connected};const snapshot=parseGrabStatus(line);if(snapshot)grabSnapshot={data:snapshot,time:Date.now(),live:bridge.connected};renderGrabStatus();}
  if(line==='STM32 mechanical arm console ready'){
    paramsView?.cancel('STM32 已重启，旧读回值已清除。');
    status.clear();routeLocked=false;grabSnapshot=null;grabRouteSnapshot=null;grabReply=null;renderGrabStatus();
    for(const view of [wheelView,mapView,radarView,qrView,prepView]){view?.connection(false);view?.connection(bridge.connected);}
    appendLog('SYS','已收到 STM32 启动信息；旧状态已清除。先自动完成 Z/X/Base 回零，再保持静止完成 IMU 5 秒验证。');renderStatus();
  }
  if(/^OK imu zero started/.test(line)){
    mapView?.connection(bridge.connected);prepView?.connection(false);prepView?.connection(bridge.connected);
    appendLog('SYS','Z 轴归零已受理。完成后查询 IMU 状态查看新角度；跑图前重新验证并设置地图起点。');
  }
wheelView?.receive(line);mapView?.receive(line);
const routeState=line.match(/^(?:OK|ERR) chassis route state=(\w+)/)?.[1]??(line.startsWith('@CHASSIS ')?mapView?.snapshot()?.route?.state:null);if(routeState){routeLocked=['running','stopping','waiting','station'].includes(routeState);updateControls();}
qrView?.receive(line);prepView?.receive(line);const owner=moduleForWire(line);if(owner!=='manual'&&!line.startsWith('@CHASSIS ')&&/^(OK |ERR |\[IMU CAL\]|\[MOTOR BUS\]|ARM STM32|BUILD |EVT qr )/.test(line)){status.set(owner,{line,time:Date.now(),live:bridge.connected});renderStatus();}}
function connection(state){const changed=state!==connectionState;if(changed){bootStage='';if($('boot-status'))$('boot-status').textContent='启动状态：连接变化，请读取设备状态';grabRouteSnapshot=null;grabSnapshot=null;grabReply=null;renderGrabStatus();}connectionState=state;if(lastStopEvent)$('stop-status').textContent=describeStopEvent(lastStopEvent,state==='connected'||state==='disconnecting',state==='disconnecting');const connected=state==='connected';if(connected&&bridge.port){const select=$('serial-port');if(!Array.from(select.options).some(o=>o.value===bridge.port)){const o=node('option',bridge.port);o.value=bridge.port;select.append(o);}select.value=bridge.port;}if(changed){wheelView?.connection(connected);mapView?.connection(connected);radarView?.connection(connected);qrView?.connection(connected);prepView?.connection(connected);if(!connected)paramsView?.cancel('连接已变化，旧读回值已清除。');}$('connection-status').textContent=({connected:`本地服务已连接 ${bridge.port}`,connecting:`正在连接 ${$('serial-port').value}…`,disconnecting:'正在停车并断开…',disconnected:'尚未连接'})[state];$('connection-dot').classList.toggle('online',connected);$('connect').disabled=state!=='disconnected'||!$('serial-port').value;$('serial-port').disabled=state!=='disconnected';$('refresh-ports').disabled=state!=='disconnected';$('disconnect').disabled=!connected;$('baud').disabled=state!=='disconnected';if(changed)for(const value of status.values())value.live=false;if(!connected)$('subscription-note').textContent='';updateControls();renderStatus();if(changed&&connected)appendLog('SYS','已连接，尚未自动发送任何指令。');}
function communicationError(error){appendLog('SYS',`通信异常：${error.message}`);toast(error.message);prepView?.refreshConfiguration();updateControls();}
function stopped(){return bridge.stopLatched;}
function bridgeEvent(e){if(e.kind==='preparation'&&e.active===false&&e.owner===bridge.preparationOwner&&prepView?.busy)prepView.cancel('共享准备锁定已结束，请重新准备。');if(e.kind==='tx'&&!/^radar fetch /.test(e.text))appendLog('TX',e.text);if(e.kind==='system')appendLog('SYS',e.text||e.error||'本地服务状态变化');if(e.kind==='stop'){if(e.latched)prepView?.cancel('停止已锁定，准备已取消；解除锁定后也不会自动继续。');lastStopEvent=e;$('stop-status').textContent=describeStopEvent(e,bridge.connected,bridge.disconnecting);$('resume-stop').hidden=!e.latched;updateControls();prepView?.refreshConfiguration();}}
const bridge=new BridgeLink({receive,state:connection,error:communicationError,event:bridgeEvent,configuration:()=>prepView?.refreshConfiguration()});
const preparationStop=wire=>wire==='bus recover'||/^(?:chassis (?:stop|route cancel)|stop (?:all|base|z|x)|vision stop|material stop|grab stop)$/.test(wire);
function mayQuery(wire){return readOnlyCommand(wire)&&(!prepView?.busy||!prepView.pending);}
async function requestStop(){presetCancelled=true;paramsView?.cancel('停止操作已取消参数核对。');attempts.supersede();$('attempt-wire').textContent='chassis stop';$('attempt-status').textContent='请求停车 · 确认见底部状态';$('latest-command').dataset.state='pending';appendLog('SYS','请求停车 → chassis stop');prepView?.cancel('已中止准备并请求停车；请查看停止状态。');try{await bridge.stop();return true;}catch(error){communicationError(error);return false;}}
function showAttempt({wire,state,current}) {
  if(current){
  $('attempt-wire').textContent=wire;
  $('attempt-status').textContent=({pending:'待发送',written:'已写出 · 待设备确认',failed:'未完成 · 查看日志'})[state];
  $('latest-command').dataset.state=state;
  }
  if(state==='pending')appendLog('SYS',`请求 → ${wire}`);
  if(state==='failed')appendLog('SYS',`未完成 → ${wire}（未获写出确认，不自动重发）`);
}
const attempts=createAttemptTracker(showAttempt);
async function send(wire,fromPreparation=false){
  if(['chassis stop','wheel stop'].includes(wire))return requestStop();
  if(preparationStop(wire)||scopedStopCommand(wire)){radarExchange.cancel();presetCancelled=true;paramsView?.cancel('停止操作已取消参数核对。');}
  if(/^system\s+reset$/.test(wire.trim())&&!window.confirm('重启 STM32？\n仅在整车静止、任务空闲时操作。主控将重新初始化，外部驱动器不会断电复位。重启后会自动执行三轴回零，再进行 IMU 5 秒静止验证。'))return false;
  return attempts.run(wire,()=>executeSend(wire,fromPreparation));
}
async function applyGrabPreset(){
  if(applyingGrabPreset)return;
  if(!bridge.connected||bridge.disconnecting||stopped()||prepView?.busy){toast('请先连接设备并结束当前准备或停止锁定。');return;}
  let wires;
  try{wires=buildGrabPresetCommands();}catch(error){toast(error.message);return;}
  if(!wires.length){toast('调试预设没有待发送参数。');return;}
  applyingGrabPreset=true;presetCancelled=false;updateControls();
  let written=0;
  try{
    for(const wire of wires){
      if(presetCancelled||!bridge.connected||stopped())break;
      if(!await send(wire))break;
      written++;
    }
  }finally{
    applyingGrabPreset=false;updateControls();
    toast(`预设已写出 ${written}/${wires.length} 条；请检查设备回复，必要时用 grab get 读回。`);
  }
}
async function executeSend(wire,fromPreparation=false,quiet=false){
  if(stopped()&&!readOnlyCommand(wire)&&!scopedStopCommand(wire)&&wire!=='bus recover'){appendLog('SYS','停止已锁定，本次指令未发送。');toast('停止已锁定，请确认停车后解除锁定。');return false;}
  if(prepView?.busy&&!fromPreparation){if(preparationStop(wire)||wire==='imu cal cancel')prepView.cancel('已中止准备；实际停止请查看设备反馈。');else if(!mayQuery(wire)){toast('准备正在等待专属回复，请稍后查询，或先取消准备。停止始终可用。');return false;}else prepView.cancel('已退出准备并读取状态，避免查询回复混入准备流程。');}
  if(sending&&!fromPreparation&&!scopedStopCommand(wire)){toast('上一条指令仍在发送，本次未排队。');return false;}
  try{frameCommand(wire);sending=Number(sending)+1;updateControls();await bridge.send(wire,fromPreparation);if(!quiet)toast(`已发送：${wire}`);if(wire==='chassis stream off')$('subscription-note').textContent='已请求关闭遥测';return true;}catch(error){appendLog('SYS',error.message);toast(error.message);return false;}finally{sending=Math.max(0,Number(sending)-1);updateControls();}
}
function canSend(wire){return !(routeLocked&&/^chassis route (start|auto)/.test(wire))&&bridge.connected&&!bridge.disconnecting&&(!(applyingGrabPreset||parameterBusy)||scopedStopCommand(wire)||preparationStop(wire))&&(!sending||scopedStopCommand(wire))&&(!stopped()||readOnlyCommand(wire)||scopedStopCommand(wire)||wire==='bus recover')&&(!prepView?.busy||preparationStop(wire)||wire==='imu cal cancel'||mayQuery(wire));}
function updateControls(){qrView?.refreshControls();document.querySelectorAll('[data-send]').forEach(b=>{const wire=b.dataset.stop||b.title||'';b.disabled=!canSend(wire)||(b.dataset.single==='1'&&axis==='all');});$('raw-send').disabled=!bridge.connected||bridge.disconnecting||!!sending||applyingGrabPreset||parameterBusy||(!!prepView?.busy&&!!prepView.pending);$('device-reset').disabled=!canSend('system reset');$('service-stop').disabled=!bridge.available;$('resume-stop').disabled=!bridge.available;paramsView?.update();radarView?.render();for(let i=forms.length-1;i>=0;i--){if(forms[i].root.isConnected)forms[i].update();else forms.splice(i,1);}}
async function refreshPorts(){
  const select=$('serial-port'),previous=select.value;
  $('refresh-ports').disabled=true;
  try {
    const {ports}=await bridge.api('ports');
    const blank=node('option',ports.length?'请选择串口':'未发现串口，请插入后刷新');blank.value='';
    select.replaceChildren(blank,...ports.map(port=>{const o=node('option',`${port.device} · ${port.description}`);o.value=port.device;return o;}));
    select.value=ports.some(p=>p.device===previous)?previous:'';
  } catch(error) {
    const blank=node('option','读取失败，请重启本地服务后刷新');blank.value='';select.replaceChildren(blank);toast(error.message);
  } finally {connection(connectionState);}
}
$('refresh-ports').onclick=()=>void refreshPorts();
$('serial-port').onchange=()=>connection(connectionState);
$('connect').onclick=async()=>{
  if(connectionState!=='disconnected'||!$('serial-port').value)return;
  connection('connecting');
  try{if(!bridge.available)await bridge.attach();await bridge.connect($('serial-port').value,Number($('baud').value));}
  catch(error){connection(bridge.connected?'connected':'disconnected');toast(error.message);}
};
$('disconnect').onclick=()=>void bridge.disconnect().catch(error=>toast(error.message));
window.addEventListener('pagehide',()=>bridge.suspend());
$('device-reset').onclick=()=>void send('system reset');$('service-stop').onclick=()=>void requestStop();$('resume-stop').onclick=()=>void bridge.resume().catch(communicationError);
function renderStatus(){const s=status.get(active);$('reply-title').textContent=s?`${s.live?'':'历史 · '}${describeReply(s.line)}`:(bridge.connected?'已连接 · 请读取状态':'尚未连接');$('reply-text').classList.toggle('error',!!s&&replyIsFault(s.line));$('reply-text').textContent=(s?displayReply(s.line):'')||'状态由设备回复确认，点击按钮不会预先标为成功。';$('reply-time').textContent=s?`最近回复 ${new Date(s.time).toLocaleTimeString('zh-CN',{hour12:false})}`:'尚无本模块回复';}
function definition(id){const c=commands.find(c=>c.id===id);if(!c)throw new Error(`缺少命令定义 ${id}`);return c;}
function action(id,label,preset={},useAxis=false){const c=definition(id),b=node('button',label||c.label);b.type='button';b.dataset.send='1';if(useAxis&&!c.fields.find(f=>f.key==='axis')?.options?.some(([v])=>v==='all'))b.dataset.single='1';const wire=()=>buildCommand(id,{...Object.fromEntries(c.fields.map(f=>[f.key,f.value])),...(typeof preset==='function'?preset():preset),...(useAxis?{axis}: {})});const hint=()=>{try{b.title=wire();}catch{b.title='请选择单轴';}};b.onmouseenter=hint;b.onfocus=hint;hint();b.onclick=()=>{try{void send(wire());}catch(e){toast(e.message);}};return b;}
function actions(container,list){const row=node('div',undefined,'actions');for(const item of list)row.append(action(...(Array.isArray(item)?item:[item])));container.append(row);return row;}
function controlForm(id,{sharedAxis=false,label,idPrefix=''}={}){const c=definition(id),root=node('form',undefined,'control-form');root.append(node('h3',label||c.label));const fields=node('div',undefined,'fields'),inputs={};for(const f of c.fields){if(sharedAxis&&f.key==='axis')continue;const wrap=node('label',f.label);let input;if(f.options){input=node('select');if(f.value===''){const placeholder=node('option','请选择（需人工确认）');placeholder.value='';input.append(placeholder);}for(const [v,t] of f.options){const o=node('option',t);o.value=v;input.append(o);}}else{input=node('input');input.type='number';input.min=f.min;input.max=f.max;input.step=f.step??1;input.required=true;}input.name=f.key;input.value=f.value??'';input.id=`${idPrefix}${id}-${f.key}`;wrap.append(input);fields.append(wrap);inputs[f.key]=input;}root.append(fields);const note=node('p',c.note||'每次执行发送一条指令，以设备回复为准。','form-note');root.append(note);const preview=node('div',undefined,'preview'),code=node('code'),buttons=node('div',undefined,'form-actions'),copy=node('button','复制'),execute=node('button',id==='pos'?'执行相对运动':'执行','primary');copy.type='button';execute.type='submit';if(id.startsWith('grab-set-'))buttons.append(action(id.replace('grab-set-','grab-get-'),'读取设备当前值'));buttons.append(copy,execute);preview.append(code,buttons);const validation=node('p','','validation');validation.setAttribute('aria-live','polite');root.append(preview,validation);let wire='';function update(){if(['chassis-route-start','chassis-route-auto'].includes(id))inputs.speed.disabled=routeLocked;try{wire=buildCommand(id,{...Object.fromEntries(Object.entries(inputs).map(([k,e])=>[k,e.value])),...(sharedAxis?{axis}: {})});code.textContent=wire;validation.textContent='';execute.disabled=!canSend(wire);copy.disabled=false;}catch(e){wire='';code.textContent='请补全有效参数';validation.textContent=e.message;execute.disabled=copy.disabled=true;}if(inputs.seconds&&['chassis-run','chassis-heading'].includes(id)){const t=Number(inputs.seconds.value);note.textContent=`起步 0.5 秒 + 保持 ${Number.isFinite(t)?t:'—'} 秒 + 停车 0.5 秒，总时长 ${Number.isFinite(t)?Math.round((t+1)*1000)/1000:'—'} 秒。${c.note||''}`;}}for(const input of Object.values(inputs)){input.oninput=update;input.onchange=update;}root.onsubmit=e=>{e.preventDefault();update();if(wire)void send(wire);};copy.onclick=()=>void copyText(wire);forms.push({root,update});update();return root;}
async function copyText(text){try{await navigator.clipboard.writeText(text);toast('已复制。');}catch{toast('无法复制，请手动选择文本。');}}
function advanced(container,title,ids){const details=node('details',undefined,'advanced');details.append(node('summary',title));const selector=node('select');selector.setAttribute('aria-label',title);const placeholder=node('option','选择调试操作');placeholder.value='';selector.append(placeholder);for(const id of ids){const c=definition(id),o=node('option',c.label);o.value=id;selector.append(o);}const target=node('div');selector.onchange=()=>{target.replaceChildren();if(selector.value){const c=definition(selector.value);if(c.fields.length)target.append(controlForm(c.id,{sharedAxis:c.fields.some(f=>f.key==='axis')}));else actions(target,[c.id]);}updateControls();};details.append(selector,target);container.append(details);}
function tabs(container,items){const nav=node('div',undefined,'mode-tabs'),body=node('div');const buttons=[];for(const [title,render] of items){const b=node('button',title);b.type='button';b.onclick=()=>{buttons.forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});body.replaceChildren();render(body);updateControls();};nav.append(b);buttons.push(b);}container.append(nav,body);buttons[0].click();}

function buildGrabPreparation(body){
  const prep=node('section',undefined,'grab-preparation');
  prep.append(node('h3','0 · 测试前准备'));
  for(const [axisName,label,mode,homeLabel] of [['base','Base · 底座','near','就近回零'],['x','X · 伸缩轴','collision','碰撞回零'],['z','Z · 升降轴','collision','碰撞回零']]){
    const row=node('div',undefined,'grab-home-row');
    row.append(node('strong',label));
    actions(row,[['home',homeLabel,{axis:axisName,mode}]]);
    prep.append(row);
  }
  prep.append(node('p','上电自动执行：Z 碰撞回零 → X 碰撞回零 → Base 就近回零。若状态提示参考参数缺失，先移走夹持物并确认运动范围安全，再点击下方按钮。它会重新执行三轴回零，各轴读回验证通过后恢复对应默认参考；全部完成后才能开始新任务。上面的单轴按钮只做手动动作，不会自动解锁抓取。','form-note'));
  actions(prep,[['grab-rehome']]);
  body.append(prep);
}
function buildGrabPanel(body){
  actions(body,[['bus-recover','一键恢复电机总线'],['bus-status','查看恢复结果']]);
  body.append(node('p','先选择物料颜色，再验证协同对准和单件视觉抓取。固定位置抓取不使用视觉颜色；路线任务仍按原默认红色。','form-note'));
  buildGrabPreparation(body);
  const colorRow=node('section',undefined,'grab-preparation');colorRow.append(node('h3','任务前选择物料颜色'));
  const colorLabel=node('label','对准与视觉抓取目标 · '),colorSelect=node('select');colorSelect.id='grab-task-color';
  for(const [value,label] of grabColorOptions){const option=node('option',label);option.value=value;colorSelect.append(option);}
  colorSelect.value='3';colorLabel.append(colorSelect);colorRow.append(colorLabel,node('p','选择只影响下次点击的“仅协同对准”和“单件视觉抓取”；当前任务不会中途换色。','form-note'));body.append(colorRow);
  const sequence=node('div',undefined,'grab-steps');
  for(const [id,title,note] of [
    ['grab-fixed','1 · 固定位置抓取','下降夹取 → 提起至 z_lift → Base 转 −180° → 下降至 −60 mm 放置 → 松爪 → 提起至 z_lift。'],
    ['grab-align','2 · 仅协同对准','使用上方选定的物料颜色。上电回零及IMU验证完成后可直接启动；X/Z观察位置为0，目标像素(334,338)。使用待实测的初始响应模型，底盘与X协同对准后停止，不下降夹取。'],
    ['grab-pick','3 · 单件视觉抓取','已在观察区域：有效坐标持续处于像素容差内且停稳反馈满足条件后下降、闭合、提起、转−180°、下降放置、松爪并提起，等待人工检查。']
  ]){
    const step=node('section');step.append(node('h3',title),node('p',note,'form-note'));
    actions(step,[[id==='grab-fixed'?id:`${id}-color`,id==='grab-fixed'?'开始固定位置抓取':undefined,id==='grab-fixed'?{}:()=>({color:colorSelect.value})],['grab-stop',id==='grab-align'?'停止协同对准':'停止本次抓取']]);
    if(id==='grab-align')step.append(node('p','“停止协同对准”发送 grab stop，中止本次任务并请求底盘和机械臂停止。state=stopping 表示正在等待停止确认；state=idle 且 stop_confirmed=1 才表示任务已结束并确认停止。','form-note'));
    sequence.append(step);
  }body.append(sequence);
  const route=node('details',undefined,'advanced');route.append(node('summary','4 · 前四段路线与模拟扫码'));route.append(controlForm('grab-route'));body.append(route);
  const config=node('section',undefined,'grab-config');config.append(node('h3','单件抓取标定与参数'),node('p','已预填启动默认值，输入框不是设备读回值。准心(334,338)和像素响应为联调初值，未完成实车标定。修改后逐项发送并检查回复，仅存MCU RAM；重启恢复默认值。准心应与香橙派设置一致。','form-note'));
  const presetButton=node('button','逐项发送调试预设');presetButton.type='button';presetButton.onclick=()=>void applyGrabPreset();
  config.append(node('p','集中参数在 grab-params.mjs。修改并刷新页面后，点击此按钮才会逐项写出 GRAB_TEST_PRESET；写出不等于设备已接受，请查看 RX 或逐项读回。','form-note'),presetButton);
  for(const group of new Set(grabParameters.map(p=>p.group)))advanced(config,group,grabParameters.filter(p=>p.group===group).map(p=>`grab-set-${p.key}`));body.append(config);
  actions(body,[['grab-status'],['grab-stop']]);
  grabPanel=node('section',undefined,'grab-status');grabPanel.setAttribute('aria-label','单件抓取反馈快照');body.append(grabPanel);renderGrabStatus();
}
function renderGrabStatus(){
  if(!grabPanel?.isConnected)return;
  const r=grabRouteSnapshot?.data;
  const s=grabSnapshot?.data,age=grabSnapshot?Math.max(0,Math.floor((Date.now()-grabSnapshot.time)/1000)):null;
  const current=grabSnapshot?.live&&age<=5;
  const heading=node('h3',s?`${current?'最近快照':'历史快照'} · ${({idle:'空闲',prepare:'观察准备',acquire:'获取目标',align:'协同对准',vision_grace:'短暂漏检 · 减速等待',vision_pause:'视觉暂停 · 不会下降夹取',settle:'水平停稳',descend:'Z 下降',close:'夹爪闭合',lift:'带物料提起',turn:'Base 转向放置处',place:'下降至放置高度',release:'松爪放置',retract:'放置后提起',homing:'上电自动回零',hold:s.mode==='align'?'对准完成 · 未抓取':'等待人工验收',stopping:'正在停止',error:'故障停止',route:'路线运行',scan:'模拟扫码'})[s.state]||s.state}`:'尚无抓取状态');
  const freshness=node('p',s?`收到于 ${new Date(grabSnapshot.time).toLocaleTimeString('zh-CN',{hour12:false})} · ${age} 秒前。点击刷新读取当前状态。`:'点击“刷新抓取状态”读取设备状态；本页不会自动轮询。','form-note');
  const list=node('dl',undefined,'grab-readout');
  const value=(key,unit='')=>s?.[key]===null||s?.[key]===undefined?'—':`${s[key]}${unit}`;
  const rows=[['视觉恢复',grabRecoveryText(s)],['本次或最近中断',value('loss_ms',' ms')],['本任务最长中断',value('loss_max_ms',' ms')],['模式',({fixed:'固定位置',align:'仅对准',pick:'视觉抓取'})[s?.mode]||s?.mode||'—'],['目标颜色',grabColorOptions.find(([id])=>Number(id)===s?.color)?.[1]||value('color')],['视觉协议',s?.protocol||'未报告'],['坐标状态',visualStateText(s?.vision)],['MCU 接收序号',s?.rx_seq===null||s?.rx_seq===undefined?'未报告':String(s.rx_seq)],['响应模型',({initial:'联调初值 · 未实测',custom:'自定义参数 · 不代表已标定',unset:'未设置'})[s?.model]||'—'],['缺少标定',describeGrabMissing(s?.missing)],['参考状态',({boot_pending:'上电待验证',home_verifying:'正在验证三轴',home_verified:'三轴已验证',base_place_turn:'Base 已转向放置处',manual_home_all:'手动全部回零后待验证',manual_home_x:'手动 X 回零后待验证',manual_home_z:'手动 Z 回零后待验证',manual_home_base:'手动 Base 回零后待验证',manual_zero_all:'手动全部清零后待验证',manual_zero_x:'手动 X 清零后待验证',manual_zero_z:'手动 Z 清零后待验证',manual_zero_base:'手动 Base 清零后待验证',manual_base_move:'Base 已手动移动',x_reference_changed:'X 参考失效',z_reference_changed:'Z 参考失效',base_reference_changed:'Base 参考失效',all_reference_changed:'三轴参考失效'})[s?.ref_cause]||s?.ref_cause||'—'],['原因',s?.reason||'—'],['X 实际 / 目标',`${value('x')} / ${value('xt')} mm`],['Base 实际 / 目标',`${value('b')} / ${value('bt')} °`],['Z 实际 / 目标',`${value('z')} / ${value('zt')} mm`],['DX / DY',grabPixelText(s)],['坐标接收间隔',s?.age_source==='rx'?value('age',' ms'):'未报告'],['前向 / 左向速度',`${value('vf')} / ${value('vl')} mm/s`],['任务耗时',value('elapsed',' ms')],['停止请求',s?.stop_requested===1?'已请求':s?.stop_requested===0?'未请求':'—'],['最近停止请求反馈',s?.stop_confirmed===1?'该次已确认（结合当前阶段）':s?.stop_confirmed===0?'尚未确认':'—']];
  for(const [label,text] of rows)list.append(node('dt',label),node('dd',text));
  const reply=node('p',grabReply?`${grabReply.live?'':'历史 · '}${displayReply(grabReply.line)}`:'设备回复将在此显示。','grab-reply');reply.classList.toggle('error',!!grabReply&&replyIsFault(grabReply.line));
  const routeReply=node('div',undefined,'grab-route-status');
  if(r){const old=!grabRouteSnapshot.live||Date.now()-grabRouteSnapshot.time>5000;routeReply.append(node('h3',`${old?'历史 · ':''}组合路线：${({idle:'空闲',driving:'行驶中',scan_wait:'扫码等待 1 秒',display_wait:'等待屏幕发送',handoff:'物料台已交接',error:'路线故障',cancelled:'已取消'})[r.state]||r.state}`),node('p',`第 ${r.segment??'—'} 段 · 来源 ${r.source==='simulated'?'模拟扫码':r.source||'—'} · ${r.code||'—'} · ${r.elapsed_ms??'—'} ms`, 'form-note'),node('p',`路线原因：${r.reason||'—'}`,r.state==='error'?'grab-reply error':'grab-reply'));}
  grabPanel.replaceChildren(routeReply,heading,freshness,list,reply,node('p','ACK / 请求受理不等于到位；取放动作完成不等于物料放置成功，须人工验收。','form-note'));
}

function buildPages(){for(const key of Object.keys(titles).filter(x=>x!=='map')){pages[key]=node('div');pages[key].dataset.page=key;pages[key].hidden=true;$('module-content').append(pages[key]);}
const arm=pages.arm;arm.append(node('p','先选择电机轴，再使能并执行相对运动。夹爪独立控制。','form-note'));arm.append(node('small','当前对象'));const picker=node('div',undefined,'axis-picker');for(const [value,label] of [['base','底座 Base'],['z','升降 Z'],['x','伸缩 X'],['all','全部轴']]){const b=node('button',label);b.classList.toggle('active',value===axis);b.setAttribute('aria-pressed',String(value===axis));b.onclick=()=>{axis=value;picker.querySelectorAll('button').forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});updateControls();};picker.append(b);}arm.append(picker);actions(arm,[['enable','使能',{},true],['disable','失能',{},true],['state','读取状态',{},true],['stop','停止所选轴',{},true]]);arm.append(controlForm('pos',{sharedAxis:true}));const grip=node('div',undefined,'grip-row');grip.append(node('h3','夹爪'));actions(grip,[['grip-open','打开 · 500 µs'],['grip-catch','夹取 · 700 µs']]);arm.append(grip);advanced(arm,'回零、零点与运动参数',['home','origin','zero','config-get','config-set']);
const ch=pages.chassis;
ch.append(node('p','先做整车运动测试；需要按地图走路线时，前往“地图与遥测”准备跑图。停止按钮始终在底部。','form-note'));
actions(ch,[['chassis-task','查看底盘状态']]);
ch.append(controlForm('chassis-run',{label:'整车平移与转向'}));
ch.append(node('p','前向速度：正数前进，负数后退；左向速度：正数左移，负数右移。角速度：正数逆时针。角速度为 0 时保持起步朝向，需 IMU 验证通过。','form-note'));
const wheelSection=node('details',undefined,'advanced');wheelSection.append(node('summary','四轮状态 · 查看反馈快照'));
actions(wheelSection,[['feedback-read','刷新四轮快照']]);wheelView=mountWheelFeedback(wheelSection);ch.append(wheelSection);
const realRoute=node('details',undefined,'real-route-panel');realRoute.id='real-route-panel';realRoute.open=true;realRoute.append(node('summary','准备跑图 · 实车完整路线 · 16段'));
prepView=mountPreparation({root:realRoute,send:wire=>send(wire,true),beginTelemetry:nonce=>mapView.beginPreparedTelemetry(nonce),telemetry:()=>mapView?.snapshot(),getConfig:()=>bridge.config,saveConfig:c=>bridge.saveConfig(c),configAvailable:()=>bridge.available,isStopped:stopped,acquire:()=>bridge.acquirePreparation(),release:()=>bridge.releasePreparation(),changed:updateControls});
realRoute.append(node('p','从 (2250,150,90°) 出发，按地图默认16个站点经过扫码位、原料区、粗加工区、暂存区，最终返回起点。选择手动逐段或自动停稳0.1秒后继续；不自动执行扫码与抓放。','form-note'));
realRoute.append(controlForm('chassis-route-start',{label:'手动分段运行'}));
realRoute.append(controlForm('chassis-route-auto'));
actions(realRoute,[['chassis-route-next','手动下一段'],['chassis-route-cancel','取消路线'],['chassis-route-status','查询实车路线']]);
const realStatus=node('pre','实车路线状态不可用 · 请在地图页手动开启遥测；查询回复见收发记录。','real-route-status');realStatus.id='real-route-status';realStatus.setAttribute('role','status');realRoute.append(realStatus);

realRoute.append(node('p','需先完成驱动反馈配置、IMU 验证与地图锚点设置。路线到点表示编码器反馈估计进入容差，不能替代视觉精对准、扫码或抓取确认。刷新、重连和切换页面不会开始或进入下一段；取消请求与实际停止分别看设备状态。','form-note'));
const routeWorkspace=node('div',undefined,'route-workspace'),routeOps=node('div',undefined,'route-operations');const prepSection=realRoute.querySelector('.route-preparation');for(const child of [...realRoute.children])if(child.tagName!=='SUMMARY'&&child!==prepSection)routeOps.append(child);routeWorkspace.append(prepSection,routeOps);realRoute.append(routeWorkspace);$('map-page').querySelector('.map-sidebar').prepend(realRoute,$('simulation-actions'),$('route-editor'),$('speed-lab-note'));
const advancedMotion=node('details',undefined,'advanced');advancedMotion.append(node('summary','高级运动 · 指定航向与地图位移'));
advancedMotion.append(node('p','指定航向使用 HWT101 模块角度；地图位移使用场地坐标。先完成准备跑图，再使用这些操作。','form-note'));
advancedMotion.append(controlForm('chassis-heading',{label:'保持指定模块航向'}),controlForm('chassis-move',{label:'地图相对位移 · 最长 300 mm'}),controlForm('chassis-origin',{label:'手动设置地图位姿 · 不移动小车'}));ch.append(advancedMotion);
const recovery=node('details',undefined,'advanced');recovery.append(node('summary','故障恢复 · 实物驱动已复位后使用'));
recovery.append(node('p','用于驱动器已实际复位后的总线恢复，不会复位硬件。先停止四轮采集并等待在途事务结束，再确认恢复。底部“解除停止锁定”仅解除网页软件停车锁，两者不同。','form-note'));
actions(recovery,[['feedback-off','停止四轮采集'],['reset-confirmed','确认驱动已复位']]);ch.append(recovery);
tabs(pages.vision,[['单件抓取',buildGrabPanel],['旧版对准调试',body=>{body.append(node('p','保持 Base 当前位置 → 内部请求识别 → 底盘搜索 → 底盘/X 对准。当前终点为对准完成。','form-note'));actions(body,[['material-status','读取任务状态'],['material-stop','停止物料任务']]);body.append(controlForm('material-auto'));}],['相机调试',body=>{body.append(node('p','仅识别，不主动驱动机构；停止识别请求不等于停止物料任务。','form-note'));actions(body,[['camera-status','读取相机状态'],['camera-stop','停止识别请求']]);body.append(controlForm('camera-material'));}],['标定与分步测试',body=>{actions(body,[['vision-status','读取 Base/X 状态'],['vision-stop','停止 Base/X 对准']]);advanced(body,'参考与两种标定路径',['vision-ref','vision-calib-material','vision-calib-chassis']);body.append(controlForm('vision-material',{label:'Base/X 独立对准'}));}]]);
const sys=pages.system;const bootNote=node('p',bootStage||'启动状态：尚未收到本次启动报告','form-note');bootNote.id='boot-status';sys.append(bootNote);actions(sys,[['bus-recover','一键恢复电机总线'],['bus-status','查看恢复结果']]);sys.append(node('p','误操作或通信错误后使用：停止任务、检查全部电机，成功后由你重新启动。不会重启主控、自动归零或续跑；恢复结果见终端。','form-note'));actions(sys,[['info','设备信息'],['help','固件帮助'],['imu-status','读取 IMU 状态']]);sys.append(node('p','上电先完成 Z/X/Base 回零，再静止验证 5 秒；失败后可手动重新验证。原生零偏标定约 20 秒，随后自动验证 30 秒；全过程保持静止。保存请求与断电保持验证分别查看。','form-note'));actions(sys,[['imu-cal-start','原生标定'],['imu-verify','重新验证 5 秒'],['imu-cal-cancel','取消标定/验证']]);sys.append(node('p','Z 轴角度清零：把当前朝向设为 0°，不消除漂移、不标定零偏。保持静止操作；跑图前重新验证 IMU 并设置地图起点。','form-note'));
actions(sys,[['imu-zero','Z 轴角度清零']]);
advanced(sys,'IMU 连续角度输出',['imu-stream-on','imu-stream-off']);
const savedRouteSpeed=Number(prefs.routeSpeed);
const routeSpeed=Number.isInteger(savedRouteSpeed)&&savedRouteSpeed>=10&&savedRouteSpeed<=5000?savedRouteSpeed:5000;
  const useRouteSpeed=speed=>{
    buildCommand('chassis-route-start',{speed});
    prefs.routeSpeed=speed;save();
    for(const id of ['chassis-route-start-speed','chassis-route-auto-speed']){
      const input=$(id);if(input){input.value=String(speed);input.dispatchEvent(new Event('input'));}
    }
  };
  if(routeSpeed!==5000)useRouteSpeed(routeSpeed);
  paramsView=mountParameterPage({root:pages.params,send,canSend,radarExchange,radarModel,
    onBusy:busy=>{parameterBusy=busy;updateControls();},routeSpeed,onRouteSpeed:useRouteSpeed});
  qrView=mountQr({root:pages.qr,send,notify:toast,canSend});}
function route(){const key=location.hash.slice(1);active=titles[key]?key:'system';document.querySelector('.desk').classList.toggle('map-focused',active==='map');document.querySelector('.page-content').scrollTop=0;$('map-page').hidden=active!=='map';$('console-page').hidden=active==='map';for(const [key,page] of Object.entries(pages))page.hidden=key!==active;document.querySelectorAll('#navigation a').forEach(a=>a.setAttribute('aria-current',a.hash===`#${active}`?'page':'false'));$('module-title').textContent=titles[active];$('module-badge').textContent=active==='qr'?'任务码接收':'手动操作';updateControls();renderStatus();renderLogs();}
$('map-log-toggle').onclick=()=>{const open=document.querySelector('.desk').classList.toggle('logs-open');$('map-log-toggle').textContent=open?'收起日志':'展开日志';$('map-log-toggle').setAttribute('aria-expanded',String(open));renderLogs();};
$('raw-form').onsubmit=async e=>{e.preventDefault();const command=$('raw-command').value.trim();if(await send(command))$('raw-command').value='';};document.querySelectorAll('[data-stop]').forEach(b=>{b.dataset.send='1';b.onclick=()=>void send(b.dataset.stop);});
$('log-filters').onclick=e=>{const b=e.target.closest('[data-filter]');if(!b)return;filter=b.dataset.filter;document.querySelectorAll('[data-filter]').forEach(x=>x.classList.toggle('active',x===b));renderLogs();};$('log-module').onchange=renderLogs;$('log-format').onchange=()=>{prefs.logFormat=$('log-format').value;save();renderLogs();renderStatus();renderGrabStatus();};$('quiet-grab-log').onchange=()=>{prefs.quietGrabLog=$('quiet-grab-log').checked;save();unseen=0;renderLogs();};$('clear-log').onclick=()=>{logs=[];txCount=rxCount=unseen=errors=0;renderLogs();};$('terminal').onscroll=()=>{if($('terminal').scrollHeight-$('terminal').clientHeight-$('terminal').scrollTop>25)$('autoscroll').checked=false;};$('autoscroll').onchange=()=>{if($('autoscroll').checked)unseen=0;renderLogs();};$('new-logs').onclick=()=>{$('autoscroll').checked=true;unseen=0;renderLogs();};
$('export-log').onclick=()=>{const content=logs.map(l=>`[${l.time}] ${l.direction} ${l.text}`).join('\r\n');const url=URL.createObjectURL(new Blob(['\ufeff'+content],{type:'text/plain;charset=utf-8'}));const a=node('a');a.href=url;a.download=`yachiyo-${Date.now()}.txt`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
function applyTheme(){const dark=prefs.dark!==false;document.body.classList.toggle('dark',dark);$('theme-toggle').textContent=dark?'◐ 浅色':'◐ 深色';}
$('theme-toggle').onclick=()=>{prefs.dark=!(prefs.dark!==false);save();applyTheme();};
applyTheme();buildPages();mapView=mountMap({send,isPreparing:()=>!!prepView?.busy||stopped(),onStatus:text=>$('subscription-note').textContent=text});radarView=mountRadarWorkspace({model:radarModel,exchange:radarExchange,send,canSend,api:(path,body)=>bridge.api(path,body),mapView,onBusy:busy=>{parameterBusy=busy;updateControls();}});window.addEventListener('hashchange',route);route();qrView.connection(false);updateControls();appendLog('SYS','八千代·巡航已就绪，请选择串口后连接；刷新端口列表不会打开设备或发送指令。');

setInterval(()=>{if(active==='vision'&&(grabSnapshot||grabRouteSnapshot))renderGrabStatus();},1000);
void bridge.start().then(refreshPorts);
