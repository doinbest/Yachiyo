import {bootProgress,describeStopEvent,visualStateText} from './terminal-format.mjs?v=chinese-notices-20261009';
import {visibleTerminalLogs,createLogRenderScheduler} from './terminal-log.mjs?v=vision-pause-20260929';
import {mountWheelFeedback} from './wheel-feedback.mjs';
import {mountTracking} from './grab-track.mjs?v=chinese-notices-20261009';
import {mountStacking} from './stack-panel.mjs?v=stack-20261008';
let stackingPanel;
import {createAttemptTracker} from './command-attempt.mjs?v=terminal-3';
import {mountPreparation} from './route-preparation.mjs?v=chinese-notices-20261009';
import {commands,buildCommand,frameCommand,describeReply,formatTerminalLine,replyIsFault,moduleForWire,grabColorOptions,parseGrabStatus,parseGrabCalibrationStatus,parseGrabRouteStatus,describeGrabRoute,describeGrabMissing,grabPixelText,grabRecoveryText,deviceText,reasonText} from './protocol.mjs?v=chinese-notices-20261009';
import {BridgeLink,readOnlyCommand,scopedStopCommand} from './bridge.mjs?v=calibration-20261009';
import {mountMap} from './map-view.mjs?v=chinese-notices-20261009';
import {mountQr} from './qr-panel.mjs?v=bridge-1';
import {mountParameterPage} from './parameter-page.mjs?v=chinese-notices-20261009';
import {RadarModel,RadarExchange,parseRadar,radarStateText,radarReasonText} from './radar-model.mjs';
import {mountRadarWorkspace} from './radar-view.mjs';
import {MonitorResult,mountMonitorRadar} from './monitor-radar.mjs';
import {mountRadarDirectHost} from './radar-direct-view.mjs?v=single-scan-transition-20261005';
import {RadarHostClient} from './radar-host-client.mjs?v=fast-result-20261005';
import {radarProbeText} from './radar-format.mjs?v=chinese-notices-20261009';
import {parsePageRoute,createNavigationMemory} from './page-navigation.mjs?v=vision-ui-20261008';
import {migrateRoutePreferences,bindRouteSpeed} from './route-settings.mjs';
const $=id=>document.getElementById(id);
const node=(tag,text,cls)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;};
const titles={system:'系统与标定',chassis:'底盘',map:'地图与遥测',qr:'二维码',vision:'视觉任务',params:'参数调试',arm:'机械臂与夹爪'};
const storageKey='sky-atelier-v1';let prefs={};try{prefs=JSON.parse(localStorage.getItem(storageKey)||'{}')||{};}catch{}
prefs=migrateRoutePreferences(prefs);try{localStorage.setItem(storageKey,JSON.stringify(prefs));}catch{}
$('quiet-grab-log').checked=prefs.quietGrabLog!==false;
$('log-format').value=prefs.logFormat==='raw'?'raw':'chinese';
const chineseLogs=()=>$('log-format').value!=='raw';
const displayReply=line=>chineseLogs()?formatTerminalLine(line):line;
const renderLogs=createLogRenderScheduler(renderLogsNow,fn=>requestAnimationFrame(fn),()=>document.hidden);
document.addEventListener('visibilitychange',()=>{if(!document.hidden)renderLogs();});
function save(){try{localStorage.setItem(storageKey,JSON.stringify(prefs));}catch{toast('本机存储已满，设置未保存。');}}
let toastTimer;function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,3500);}
let grabCalibrationSnapshot=null;
let active='system',axis='base',mapView,radarView,radarDirectView,radarHostClient,qrView,prepView,wheelView,paramsView,visionTabs,txCount=0,rxCount=0,filter='ALL',logs=[],unseen=0,errors=0,sending=false,applyingGrabPreset=false,presetCancelled=false,parameterBusy=false;
const navigationMemory=createNavigationMemory();let previousHash='',pendingParameterSource=null;
const radarModel=new RadarModel();try{const saved=JSON.parse(localStorage.getItem('local-car-radar-snapshots')||'null');if(saved){radarModel.snapshots=saved.snapshots??{};radarModel.params=saved.params??{};}}catch{}
const monitorResult=new MonitorResult();
let savedRadarSnapshot=null;
const radarExchange=new RadarExchange({send:wire=>executeSend(wire,false,true,true)});
const status=new Map(),forms=[],pages={};
let grabSnapshot=null,grabRouteSnapshot=null,grabReply=null,grabPanel=null,trackingPanel=null;
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
let trackingConnected=false;
function receiveRadarLiveState(frame){
  const phase=({preparing:'配置中',waiting:'等待完整圈',streaming:'实时更新',stopped:'已停止',error:'采集失败'})[frame.state];
  const text=`实时点云 · ${phase} · ${radarReasonText(frame.reason)} · 圈 ${frame.q} · ${frame.points} 点`;
  $('map-recent-result').textContent=text;status.set('map',{line:text,time:Date.now(),live:bridge.connected});renderStatus();updateControls();
}
function receive(line){
  const hostAccepted=radarDirectView?.receive(line)??radarHostClient?.receive(line)??false;
  if(hostAccepted){
    const reply=line.trim().replace(/^(?:arm>\s*)+/,'');
    if(/^(?:OK|ERR) radar host\b/.test(reply)){
      appendLog('RX',line);$('map-recent-result').textContent=`${reply.startsWith('ERR')?'设备未受理':'雷达设备回复'} · ${reply.split(' ').slice(3).join(' ')}`;
    }else if(!chineseLogs())appendLog('RX',line);
    return;
  }
  radarView?.receive(line);
  const snapshot=Object.values(radarModel.snapshots).map(s=>s.receivedAt).join(':');if(snapshot!==savedRadarSnapshot){savedRadarSnapshot=snapshot;try{localStorage.setItem('local-car-radar-snapshots',JSON.stringify({snapshots:radarModel.snapshots,params:radarModel.params}));}catch{}}
  const frame=parseRadar(line);
  if(frame||line.trim().replace(/^(?:arm>\s*)+/,'').startsWith('@RADAR ')){
    if(frame?.k==='params')paramsView?.refreshRadarParameters();
    if(frame&&['status','nav'].includes(frame.k)){
      const text=`雷达 ${frame.k} · ${radarStateText(frame.state)} · ${radarReasonText(frame.reason)} · 地图 ${frame.map} / 路径 ${frame.plan}`;
      $('map-recent-result').textContent=text;status.set('map',{line:text,time:Date.now(),live:bridge.connected});if(frame.k==='nav')routeLocked=['running','stopping','waiting','station'].includes(frame.state);renderStatus();updateControls();
      if(!chineseLogs())appendLog('RX',line);
    }else if(frame?.k==='diag'||frame?.k==='probe'){
      appendLog('RX',line);
      if(frame.k==='probe'&&Object.hasOwn(frame,'state'))$('map-recent-result').textContent=radarProbeText(frame);
    }
    return;
  }
  if(radarExchange.busy&&(line==='OK radar page_pending'||/^(?:arm>\s*)?radar fetch /.test(line)))return;
  appendLog('RX',line);
  if(/^(OK|ERR) radar\b/.test(line))$('map-recent-result').textContent=`${line.startsWith('ERR')?'设备未受理':'设备回复'} · ${radarReasonText(line.split(' ')[2])}`;
  paramsView?.receive(line);
  trackingPanel?.receive(line);
  const progress=bootProgress(line,bootStage);if(progress!==bootStage){bootStage=progress;$('boot-status').textContent=progress;}
  const calibration=parseGrabCalibrationStatus(line);
  if(calibration){grabCalibrationSnapshot={data:calibration,time:Date.now(),live:bridge.connected};renderGrabCalibrationStatus();}
  if(!calibration&&/^(OK|ERR) grab\b/.test(line)){grabReply={line,time:Date.now(),live:bridge.connected};const routeSnapshot=parseGrabRouteStatus(line);if(routeSnapshot)grabRouteSnapshot={data:routeSnapshot,time:Date.now(),live:bridge.connected};const snapshot=parseGrabStatus(line);if(snapshot){grabSnapshot={data:snapshot,time:Date.now(),live:bridge.connected};stackingPanel?.receive(snapshot);}renderGrabStatus();}
  if(line==='STM32 mechanical arm console ready'){
    if(grabCalibrationSnapshot)grabCalibrationSnapshot.live=false;renderGrabCalibrationStatus();
    stackingPanel?.cancel();
    trackingPanel?.connection(false);trackingPanel?.connection(bridge.connected);
    paramsView?.cancel('STM32 已重启，旧读回标为历史值。',true);
    status.clear();routeLocked=false;grabSnapshot=null;grabRouteSnapshot=null;grabReply=null;renderGrabStatus();
    for(const view of [wheelView,mapView,radarView,qrView,prepView]){view?.connection(false);view?.connection(bridge.connected);}
    appendLog('SYS','已收到 STM32 启动信息；旧状态已清除。先自动完成 Z/X/Base 回零，再保持静止完成 IMU 5 秒验证。');renderStatus();
  }
  if(/^OK imu zero started/.test(line)){
    mapView?.connection(bridge.connected);prepView?.connection(false);prepView?.connection(bridge.connected);
    appendLog('SYS','Z 轴归零已受理。完成后查询 IMU 状态查看新角度；跑图前重新验证并设置地图起点。');
  }
wheelView?.receive(line);mapView?.receive(line);
const routeState=line.trim().replace(/^(?:arm>\s*)+/,'').match(/^(?:OK|ERR) chassis route state=(\w+)/)?.[1]??(line.startsWith('@CHASSIS ')?mapView?.snapshot()?.route?.state:null);if(routeState){routeLocked=['running','stopping','waiting','station'].includes(routeState);updateControls();}
qrView?.receive(line);prepView?.receive(line);const owner=moduleForWire(line);if(owner!=='manual'&&!line.startsWith('@CHASSIS ')&&/^(OK |ERR |\[IMU CAL\]|\[MOTOR BUS\]|ARM STM32|BUILD |EVT qr )/.test(line)){status.set(owner,{line,time:Date.now(),live:bridge.connected});renderStatus();}}
function connection(state){const changed=state!==connectionState;if(changed){if(grabCalibrationSnapshot)grabCalibrationSnapshot.live=false;renderGrabCalibrationStatus();stackingPanel?.connection(state==='connected');routeLocked=false;bootStage='';if($('boot-status'))$('boot-status').textContent='启动状态：连接变化，请读取设备状态';grabRouteSnapshot=null;grabSnapshot=null;grabReply=null;renderGrabStatus();}connectionState=state;if(lastStopEvent)$('stop-status').textContent=describeStopEvent(lastStopEvent,state==='connected'||state==='disconnecting',state==='disconnecting');const connected=state==='connected';if(connected&&bridge.port){const select=$('serial-port');if(!Array.from(select.options).some(o=>o.value===bridge.port)){const o=node('option',bridge.port);o.value=bridge.port;select.append(o);}select.value=bridge.port;}if(changed){wheelView?.connection(connected);mapView?.connection(connected);radarView?.connection(connected);const radarConnection={state,port:bridge.port||$('serial-port').value||'',baudrate:Number($('baud').value)||115200};if(radarDirectView)radarDirectView.connection(radarConnection);else radarHostClient?.connection(radarConnection);qrView?.connection(connected);prepView?.connection(connected);if(!connected)paramsView?.cancel('连接已变化，旧读回标为历史值。',true);}$('connection-status').textContent=({connected:`本地服务已连接 ${bridge.port}`,connecting:`正在连接 ${$('serial-port').value}…`,disconnecting:'正在停车并断开…',disconnected:'尚未连接'})[state];$('connection-dot').classList.toggle('online',connected);$('connect').disabled=state!=='disconnected'||!$('serial-port').value;$('serial-port').disabled=state!=='disconnected';$('refresh-ports').disabled=state!=='disconnected';$('disconnect').disabled=!connected;$('baud').disabled=state!=='disconnected';if(changed)for(const value of status.values())value.live=false;if(!connected)$('subscription-note').textContent='';updateControls();renderStatus();if(changed&&connected)appendLog('SYS','已连接，尚未自动发送任何指令。');}
function communicationError(error){appendLog('SYS',`通信异常：${error.message}`);toast(error.message);prepView?.refreshConfiguration();updateControls();}
function stopped(){return bridge.stopLatched;}
function bridgeEvent(e){if(e.kind==='preparation'&&e.active===false&&e.owner===bridge.preparationOwner&&prepView?.busy)prepView.cancel('共享准备锁定已结束，请重新准备。');if(e.kind==='tx'&&!/^radar (?:host )?fetch /.test(e.text))appendLog('TX',e.text);if(e.kind==='system')appendLog('SYS',e.text||e.error||'本地服务状态变化');if(e.kind==='stop'){if(e.latched)paramsView?.cancel('停止操作已取消参数批次；已读值和草稿保留。');if(e.latched)prepView?.cancel('停止已锁定，准备已取消；解除锁定后也不会自动继续。');lastStopEvent=e;$('stop-status').textContent=describeStopEvent(e,bridge.connected,bridge.disconnecting);$('resume-stop').hidden=!e.latched;updateControls();prepView?.refreshConfiguration();}}
const bridge=new BridgeLink({receive,state:connection,error:communicationError,event:bridgeEvent,configuration:()=>prepView?.refreshConfiguration()});
radarHostClient=new RadarHostClient({fastResults:true,send:wire=>executeSend(wire,false,true,true),onError:error=>{radarDirectView?.refresh();appendLog('SYS',error.message);}});
const preparationStop=wire=>wire==='bus recover'||/^(?:chassis (?:stop|route cancel)|stop (?:all|base|z|x)|radar live stop|grab stop)$/.test(wire);
function mayQuery(wire){return readOnlyCommand(wire)&&(!prepView?.busy||!prepView.pending);}
async function requestStop(){stackingPanel?.cancel();presetCancelled=true;radarExchange.cancel('停止操作已取消雷达读取，保留已完成快照');paramsView?.cancel('停止操作已取消参数核对。');attempts.supersede();$('attempt-wire').textContent='chassis stop';$('attempt-status').textContent='请求停车 · 确认见底部状态';$('latest-command').dataset.state='pending';appendLog('SYS','请求停车 → chassis stop');prepView?.cancel('已中止准备并请求停车；请查看停止状态。');try{await bridge.stop();return true;}catch(error){communicationError(error);return false;}}
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
  if(preparationStop(wire)||scopedStopCommand(wire)){stackingPanel?.cancel();radarExchange.cancel();presetCancelled=true;paramsView?.cancel('停止操作已取消参数核对。');}
  if(/^system\s+reset$/.test(wire.trim())&&!window.confirm('重启 STM32？\n仅在整车静止、任务空闲时操作。主控将重新初始化，外部驱动器不会断电复位。重启后会自动执行三轴回零，再进行 IMU 5 秒静止验证。'))return false;
  return attempts.run(wire,()=>executeSend(wire,fromPreparation));
}

async function executeSend(wire,fromPreparation=false,quiet=false,fromParameters=false){
  if(parameterBusy&&!fromParameters&&!scopedStopCommand(wire)&&!preparationStop(wire)){toast('参数批次正在进行，请等待完成；停止始终可用。');return false;}
  if(stopped()&&!readOnlyCommand(wire)&&!scopedStopCommand(wire)&&wire!=='bus recover'){appendLog('SYS','停止已锁定，本次指令未发送。');toast('停止已锁定，请确认停车后解除锁定。');return false;}
  if(prepView?.busy&&!fromPreparation){if(preparationStop(wire)||wire==='imu cal cancel')prepView.cancel('已中止准备；实际停止请查看设备反馈。');else if(!mayQuery(wire)){toast('准备正在等待专属回复，请稍后查询，或先取消准备。停止始终可用。');return false;}else prepView.cancel('已退出准备并读取状态，避免查询回复混入准备流程。');}
  if(sending&&!fromPreparation&&!scopedStopCommand(wire)){toast('上一条指令仍在发送，本次未排队。');return false;}
  try{frameCommand(wire);sending=Number(sending)+1;updateControls();await bridge.send(wire,fromPreparation);if(!quiet)toast(`已发送：${wire}`);if(wire==='chassis stream off')$('subscription-note').textContent='已请求关闭遥测';return true;}catch(error){appendLog('SYS',error.message);toast(error.message);return false;}finally{sending=Math.max(0,Number(sending)-1);updateControls();}
}
function canSend(wire){return !(routeLocked&&/^chassis route (start|auto)/.test(wire))&&bridge.connected&&!bridge.disconnecting&&(!(applyingGrabPreset||parameterBusy)||scopedStopCommand(wire)||preparationStop(wire))&&(!sending||scopedStopCommand(wire))&&(!stopped()||readOnlyCommand(wire)||scopedStopCommand(wire)||wire==='bus recover')&&(!prepView?.busy||preparationStop(wire)||wire==='imu cal cancel'||mayQuery(wire));}
function updateControls(){const connected=bridge.connected&&!bridge.disconnecting;if(connected!==trackingConnected){trackingConnected=connected;trackingPanel?.connection(connected);}trackingPanel?.update();stackingPanel?.update();qrView?.refreshControls();document.querySelectorAll('[data-send]').forEach(b=>{const wire=b.dataset.stop||b.title||'';b.disabled=!canSend(wire)||(b.dataset.single==='1'&&axis==='all');});$('raw-send').disabled=!bridge.connected||bridge.disconnecting||!!sending||applyingGrabPreset||parameterBusy||(!!prepView?.busy&&!!prepView.pending);$('device-reset').disabled=!canSend('system reset');$('service-stop').disabled=!bridge.available;$('resume-stop').disabled=!bridge.available;paramsView?.update();radarView?.render();for(let i=forms.length-1;i>=0;i--){if(forms[i].root.isConnected){forms[i].mounted=true;forms[i].update();}else if(forms[i].mounted)forms.splice(i,1);}}
async function refreshPorts(){
  const select=$('serial-port'),previous=select.value;
  $('refresh-ports').disabled=true;
  try {
    const ports=await listSharedPorts();
    const blank=node('option',ports.length?'请选择串口':'未发现串口，请插入后刷新');blank.value='';
    select.replaceChildren(blank,...ports.map(port=>{const o=node('option',`${port.device} · ${port.description}`);o.value=port.device;return o;}));
    select.value=ports.some(p=>p.device===previous)?previous:'';
  } catch(error) {
    const blank=node('option','读取失败，请重启本地服务后刷新');blank.value='';select.replaceChildren(blank);toast(error.message);
  } finally {connection(connectionState);}
}
async function listSharedPorts(){const result=await bridge.api('ports');return result.ports||[];}
async function connectShared(port,baudrate=115200){
  if(connectionState!=='disconnected')throw new Error('共享串口正在连接或已经连接');
  if(!port)throw new Error('请先选择串口');
  const selectedBaud=Number(baudrate);if(!Number.isInteger(selectedBaud)||selectedBaud<=0)throw new Error('串口波特率无效');
  const select=$('serial-port');if(!Array.from(select.options).some(option=>option.value===port)){const option=node('option',port);option.value=port;select.append(option);}select.value=port;$('baud').value=String(selectedBaud);
  connection('connecting');
  try{if(!bridge.available)await bridge.attach();await bridge.connect(port,selectedBaud);}
  catch(error){connection(bridge.connected?'connected':'disconnected');throw error;}
}
const disconnectShared=()=>bridge.disconnect();
const sharedConnectionController={
  get state(){return connectionState;},get port(){return bridge.port||$('serial-port').value||'';},get baudrate(){return Number($('baud').value)||115200;},
  listPorts:listSharedPorts,connect:port=>connectShared(port,115200),disconnect:disconnectShared,
};
$('refresh-ports').onclick=()=>void refreshPorts();
$('serial-port').onchange=()=>connection(connectionState);
$('connect').onclick=()=>void connectShared($('serial-port').value,Number($('baud').value)).catch(error=>toast(error.message));
$('disconnect').onclick=()=>void disconnectShared().catch(error=>toast(error.message));
window.addEventListener('pagehide',()=>{bridge.suspend();radarDirectView?.dispose();});
$('device-reset').onclick=()=>void send('system reset');$('service-stop').onclick=()=>void requestStop();$('resume-stop').onclick=()=>void bridge.resume().catch(communicationError);
function renderStatus(){const s=status.get(active);$('reply-title').textContent=s?`${s.live?'':'历史 · '}${describeReply(s.line)}`:(bridge.connected?'已连接 · 请读取状态':'尚未连接');$('reply-text').classList.toggle('error',!!s&&replyIsFault(s.line));$('reply-text').textContent=(s?displayReply(s.line):'')||'状态由设备回复确认，点击按钮不会预先标为成功。';$('reply-time').textContent=s?`最近回复 ${new Date(s.time).toLocaleTimeString('zh-CN',{hour12:false})}`:'尚无本模块回复';}
function definition(id){const c=commands.find(c=>c.id===id);if(!c)throw new Error(`缺少命令定义 ${id}`);return c;}
function action(id,label,preset={},useAxis=false){const c=definition(id),b=node('button',label||c.label);b.type='button';b.dataset.send='1';if(useAxis&&!c.fields.find(f=>f.key==='axis')?.options?.some(([v])=>v==='all'))b.dataset.single='1';const wire=()=>buildCommand(id,{...Object.fromEntries(c.fields.map(f=>[f.key,f.value])),...(typeof preset==='function'?preset():preset),...(useAxis?{axis}: {})});const hint=()=>{try{b.title=wire();}catch{b.title='请选择单轴';}};b.onmouseenter=hint;b.onfocus=hint;hint();b.onclick=()=>{try{void send(wire());}catch(e){toast(e.message);}};return b;}
function actions(container,list){const row=node('div',undefined,'actions');for(const item of list)row.append(action(...(Array.isArray(item)?item:[item])));container.append(row);return row;}
function controlForm(id,{sharedAxis=false,label,idPrefix='',compact=false,executeLabel,note:noteText,alternateCommand}={}){
  const c=definition(id),root=node('form',undefined,'control-form');
  root.classList.toggle('compact-control',compact);root.append(node('h3',label||c.label));
  const fields=node('div',undefined,'fields'),inputs={};
  for(const f of c.fields){
    if(sharedAxis&&f.key==='axis')continue;
    const wrap=node('label',f.label);let input;
    if(f.options){
      input=node('select');
      if(f.value===''){const placeholder=node('option','请选择（需人工确认）');placeholder.value='';input.append(placeholder);}
      for(const [v,t] of f.options){const o=node('option',t);o.value=v;input.append(o);}
    }else{input=node('input');input.type='number';input.min=f.min;input.max=f.max;input.step=f.step??1;input.required=true;}
    input.name=f.key;input.value=f.value??'';input.id=`${idPrefix}${id}-${f.key}`;
    wrap.append(input);fields.append(wrap);inputs[f.key]=input;
  }
  root.append(fields);
  const note=node('p',noteText??c.note??'每次执行发送一条指令，以设备回复为准。','form-note');root.append(note);
  const preview=node('div',undefined,'preview'),code=node('code'),buttons=node('div',undefined,'form-actions');
  const copy=node('button','复制'),execute=node('button',executeLabel||(id==='pos'?'执行相对运动':'执行'),'primary');
  copy.type='button';execute.type='submit';
  const alternate=alternateCommand?node('button',alternateCommand.label):null;
  if(alternate)alternate.type='button';
  if(id.startsWith('grab-set-'))buttons.append(action(id.replace('grab-set-','grab-get-'),'读取设备当前值'));
  buttons.append(copy);preview.append(code,buttons);
  if(compact){
    const footer=node('div',undefined,'compact-form-footer'),details=node('details',undefined,'command-disclosure');
    details.append(node('summary','查看指令'),preview);footer.append(execute);if(alternate)footer.append(alternate);footer.append(details);root.append(footer);
  }else{buttons.append(execute);if(alternate)buttons.append(alternate);root.append(preview);}
  const validation=node('p','','validation');validation.setAttribute('aria-live','polite');root.append(validation);
  let wire='',alternateWire='';
  function update(){
    if(['chassis-route-start','chassis-route-auto'].includes(id))inputs.speed.disabled=routeLocked;
    try{
      const values={...Object.fromEntries(Object.entries(inputs).map(([k,e])=>[k,e.value])),...(sharedAxis?{axis}: {})};
      wire=buildCommand(id,values);
      if(alternate){alternateWire=buildCommand(alternateCommand.id,values);alternate.title=alternateWire;alternate.disabled=!canSend(alternateWire);}
      code.textContent=wire;execute.title=wire;validation.textContent='';execute.disabled=!canSend(wire);copy.disabled=false;
    }catch(e){wire=alternateWire='';code.textContent='请补全有效参数';execute.title='';validation.textContent=e.message;execute.disabled=copy.disabled=true;if(alternate){alternate.title='';alternate.disabled=true;}}
    if(inputs.seconds&&['chassis-run','chassis-heading'].includes(id)){
      const t=Number(inputs.seconds.value);note.textContent=`起步 0.5 秒 + 保持 ${Number.isFinite(t)?t:'—'} 秒 + 停车 0.5 秒，总时长 ${Number.isFinite(t)?Math.round((t+1)*1000)/1000:'—'} 秒。${c.note||''}`;
    }
  }
  for(const input of Object.values(inputs)){input.oninput=update;input.onchange=update;}
  root.onsubmit=e=>{e.preventDefault();update();if(wire)void send(wire);};copy.onclick=()=>void copyText(wire);
  if(alternate)alternate.onclick=()=>{update();if(alternateWire)void send(alternateWire);};
  // Nested pages refresh controls before these forms are attached; retain them until first mount.
  forms.push({root,update,mounted:false});update();return root;
}
async function copyText(text){try{await navigator.clipboard.writeText(text);toast('已复制。');}catch{toast('无法复制，请手动选择文本。');}}
function advanced(container,title,ids){const details=node('details',undefined,'advanced');details.append(node('summary',title));const selector=node('select');selector.setAttribute('aria-label',title);const placeholder=node('option','选择调试操作');placeholder.value='';selector.append(placeholder);for(const id of ids){const c=definition(id),o=node('option',c.label);o.value=id;selector.append(o);}const target=node('div');selector.onchange=()=>{target.replaceChildren();if(selector.value){const c=definition(selector.value);if(c.fields.length)target.append(controlForm(c.id,{sharedAxis:c.fields.some(f=>f.key==='axis')}));else actions(target,[c.id]);}updateControls();};details.append(selector,target);container.append(details);}
function tabs(container,items,navigation=container){
  const nav=node('div',undefined,'mode-tabs'),bodies=[],buttons=[],ids=items.map(([id])=>id);
  nav.setAttribute('role','group');nav.setAttribute('aria-label','视觉任务类型');
  const select=value=>{if(value!=='stack')stackingPanel?.cancel();const selected=Math.max(0,ids.indexOf(value));buttons.forEach((b,i)=>{b.classList.toggle('active',i===selected);b.setAttribute('aria-pressed',String(i===selected));bodies[i].hidden=i!==selected;});updateControls();};
  for(const [id,title,render]of items){
    const b=node('button',title),body=node('div',undefined,'vision-mode');body.id='vision-'+id;
    b.type='button';b.setAttribute('aria-controls',body.id);b.onclick=()=>{location.hash='vision/'+id;};
    render(body);buttons.push(b);bodies.push(body);nav.append(b);
  }
  navigation.append(nav);container.append(...bodies);select(ids[0]);return {select};
}

function buildGrabPreparation(body){
  const prep=node('details',undefined,'vision-recovery');
  prep.append(node('summary','回零与通信恢复'),node('p','上电自动完成三轴回零。参考失效时使用“重新建立三轴参考”；通信异常时查看或恢复电机总线。','form-note'));
  actions(prep,[['grab-rehome'],['bus-recover','恢复电机总线'],['bus-status','读取总线状态']]);
  const axes=node('details',undefined,'advanced');axes.append(node('summary','单轴手动回零'));
  for(const [axisName,label,mode,homeLabel] of [['base','Base · 底座','near','就近回零'],['x','X · 伸缩轴','collision','碰撞回零'],['z','Z · 升降轴','collision','碰撞回零']]){
    const row=node('div',undefined,'grab-home-row');
    row.append(node('strong',label));
    actions(row,[['home',homeLabel,{axis:axisName,mode}]]);
    axes.append(row);
  }
  axes.append(node('p','单轴按钮只执行回零；抓取参考恢复使用上方“三轴参考”。回零前移走夹持物。','form-note'));prep.append(axes);
  body.append(prep);
}
function parameterLink(label,category,key,source='视觉任务'){
  const link=node('a',label+' →','context-link');link.href='#params/'+category+(key?'/'+key:'');link.dataset.parameterLink='1';link.dataset.parameterSource=source;return link;
}
function buildGrabPanel(body){
  body.append(node('h2','物料取放','vision-title'),node('p','选择下次任务的颜色与仓位，再选择取放方式。','form-note'));
  const options=node('section',undefined,'vision-card');options.append(node('h3','任务目标'));
  const fields=node('div',undefined,'fields');
  const colorLabel=node('label','物料颜色'),colorSelect=node('select');colorSelect.id='grab-task-color';
  for(const [value,label]of grabColorOptions){const option=node('option',label);option.value=value;colorSelect.append(option);}colorSelect.value='3';colorLabel.append(colorSelect);
  const slotLabel=node('label','入仓位置'),slotSelect=node('select');slotSelect.id='grab-task-slot';
  for(const [value,label]of [['auto','自动 · 最低编号空仓'],['1','第一仓'],['2','第二仓'],['3','第三仓']]){const option=node('option',label);option.value=value;slotSelect.append(option);}slotSelect.value='auto';slotLabel.append(slotSelect);
  fields.append(colorLabel,slotLabel);options.append(fields,parameterLink('转盘参考与库存','turntable'),node('p','固定夹取自动选仓时沿用固件默认颜色；指定颜色请选择明确仓位。','form-note'));body.append(options);
  const sequence=node('div',undefined,'grab-steps');
  for(const [mode,title,note]of [
    ['align','仅协同对准','底盘与 X 对准所选颜色，停稳后等待，不下降夹取。'],
    ['pick','视觉夹取入仓','视觉对准后夹取、入仓，再返回观察姿态。'],
    ['fixed','固定位置夹取','从已对齐的位置夹取、入仓，再返回观察姿态。']
  ]){
    const step=node('section',undefined,'vision-card');step.append(node('h3',title),node('p',note,'form-note'));
    const begin=node('button','开始'+({fixed:'固定夹取入仓',align:'协同对准',pick:'视觉夹取入仓'})[mode],'primary');begin.type='button';begin.dataset.send='1';
    const wire=()=>mode==='align'?buildCommand('grab-align-color',{color:colorSelect.value}):slotSelect.value==='auto'?(mode==='fixed'?'grab start fixed':buildCommand('grab-pick-color',{color:colorSelect.value})):buildCommand('grab-'+mode+'-slot',{slot:slotSelect.value,color:colorSelect.value});
    const update=()=>{begin.title=wire();updateControls();};colorSelect.addEventListener('change',update);slotSelect.addEventListener('change',update);begin.title=wire();begin.onclick=()=>void send(wire());
    const buttons=node('div',undefined,'actions');buttons.append(begin);step.append(buttons,parameterLink(mode==='fixed'?'取放参数':'视觉参数',mode==='fixed'?'material':'vision'));
    sequence.append(step);
  }body.append(sequence);
  const external=node('details',undefined,'vision-recovery');external.append(node('summary','粗加工取回与车载取出'),node('p','先完成外部水平对准，再取回入仓或从车载仓位取出释放。','form-note'));
  external.append(controlForm('grab-store-proc',{compact:true,executeLabel:'取回入仓'}),controlForm('grab-take',{compact:true,executeLabel:'取出并释放'}));
  actions(external,[['grab-return','返回外侧观察姿态']]);external.append(parameterLink('取放参数','material','z_proc_grab'));body.append(external);
}
function buildRingPanel(body){
  const ring=node('section',undefined,'ring-flow');ring.id='ring-place-test';
  ring.append(node('h2','编号环放置','vision-title'),node('p','粗加工区 · 先对准编号环，再手动装料放置。','form-note'));
  ring.append(controlForm('grab-ring-align',{compact:true,label:'1 · 视觉对准',executeLabel:'对准后抬高',alternateCommand:{id:'grab-ring-align-stay',label:'对准后保持低位'},note:'先横向接近，再随环心接近准心逐步下降；靠画面边缘时暂缓下降。两个按钮共用上方高度：抬高用于装料，保持低位用于观察识别，完成后均停止运动纠偏。'}));
  const load=node('section',undefined,'vision-card ring-load');load.append(node('h3','2 · 手动装料'),node('p','对准完成、Z抬至最高后，放入物料并闭合夹爪。','form-note'));
  actions(load,[['grip-open','打开夹爪 · 2500 µs'],['grip-catch','闭合夹爪 · 1920 µs']]);ring.append(load);
  ring.append(controlForm('grab-ring-place',{compact:true,label:'3 · 下降放置',executeLabel:'下降、松爪并提起',note:'按所设绝对高度下降，松爪后自动提起。'}));
  ring.append(node('p','“停止并退出”结束当前测试；再次放置前需重新对准。','form-note'));body.append(ring);
}
function buildRoutePanel(body){
  body.append(node('h2','路线与搬运','vision-title'),node('p','从右下起点出发，机械臂回零朝后。','form-note'));
  const route=node('section',undefined,'vision-card');
  route.append(controlForm('grab-route',{compact:true,label:'蓝色首件入仓',executeLabel:'启动首件路线'}));
  const inventory=node('details',undefined,'advanced');inventory.append(node('summary','第一仓库存核对'),node('p','上电后库存待核对；按实物确认第一仓为空。','form-note'));
  actions(inventory,[['turntable-inventory-set','确认第一仓为空',{slot:'1',content:'empty'}]]);route.append(inventory);body.append(route);
  const mission=node('section',undefined,'vision-card');mission.append(controlForm('grab-mission',{compact:true,label:'两批完整搬运',executeLabel:'启动两批任务',note:'三件入仓 → 加工取回 → 暂存与同色码垛。工位高度和三仓角度使用实物标定值。'}));
  actions(mission,[['grab-mission-continue','补齐参数后继续']]);
  const poses=node('details',undefined,'advanced');poses.append(node('summary','机械臂姿态'));actions(poses,[['grab-pose-rear'],['grab-pose-work']]);mission.append(poses);body.append(mission);
  body.append(parameterLink('路线与工位参数','material'),parameterLink('准备跑图参数','radar','route_speed'));
}
function renderGrabCalibrationStatus(){
  const view=document.getElementById('grab-calibration-status');if(!view)return;
  const snapshot=grabCalibrationSnapshot,s=snapshot?.data;
  if(!s){view.textContent='尚未读取视觉标定状态。';return;}
  const phase=({idle:'空闲',prepare:'准备停稳',collect:'采样',chassis_move:'底盘移动',x_ack:'X请求应答',x_arrival:'X到位',x_state:'X反馈',settle:'停稳采样',complete:'标定完成',done:'标定完成',stopping:'正在停止',error:'标定失败'})[s.phase]||deviceText(s.phase);
  const response=s.calibrated===1?`前后 ${s.fdx??'—'} / ${s.fdy??'—'} px/mm；X ${s.xdx??'—'} / ${s.xdy??'—'} px/电机°`:'暂无成功标定记录';
  view.textContent=`${snapshot.live?'最近读回':'历史读回'} · ${phase} · 观察姿态${s.ref===1?'已确认':'未确认'} · 上次成功响应：${response} · 原因 ${reasonText(s.reason)}`;
}
function buildGrabCalibrationPanel(body){
  const panel=node('section',undefined,'vision-card');panel.id='grab-calibration';
  panel.append(node('h2','视觉响应标定'),node('p','先在安全观察姿态确认，再选择颜色启动。标定会前后移动底盘并小幅移动X，Base、Z和夹爪保持当前状态；失败或停止保留上一次有效模型。','form-note'));
  panel.append(controlForm('grab-calib-ref',{executeLabel:'确认当前观察姿态'}),controlForm('grab-calib',{executeLabel:'开始视觉响应标定'}));
  actions(panel,[['grab-calib-status','读取标定状态'],['grab-stop','停止标定']]);
  const status=node('p','尚未读取视觉标定状态。','form-note');status.id='grab-calibration-status';panel.append(status);body.append(panel);
  renderGrabCalibrationStatus();
}
function buildVisionWorkspace(body){
  stackingPanel?.dispose();
  body.classList.add('vision-workspace');
  const controls=node('div',undefined,'vision-controls'),modes=node('div',undefined,'vision-modes');
  visionTabs=tabs(modes,[
    ['ring','编号环',buildRingPanel],['stack','码垛',root=>{stackingPanel=mountStacking({root,send:wire=>send(wire),canSend});stackingPanel.connection(bridge.connected);}],['grab','物料取放',buildGrabPanel],
    ['track','动态追踪',root=>{trackingPanel=mountTracking({root,send:wire=>send(wire),canSend});trackingPanel.connection(bridge.connected);}],
    ['route','路线任务',buildRoutePanel],
    ['camera','仅识别',root=>{
      root.append(node('h2','相机识别','vision-title'),node('p','检查目标圆心，不移动底盘或机械臂。','form-note'));
      root.append(controlForm('camera-ring',{compact:true,label:'编号环识别',executeLabel:'识别编号环'}),controlForm('camera-material',{compact:true,executeLabel:'识别物料'}));
      actions(root,[['camera-status','读取相机状态'],['camera-stop','停止识别']]);
    }]
  ],controls);
  const toolbar=actions(controls,[['grab-status','读取任务状态'],['grab-stop','停止并退出']]);toolbar.classList.add('vision-toolbar');toolbar.lastElementChild.classList.add('task-stop');
  grabPanel=node('section',undefined,'grab-status');grabPanel.id='grab-task-status';grabPanel.setAttribute('aria-label','抓取与放置反馈');
  body.append(controls,modes,grabPanel);buildGrabPreparation(body);renderGrabStatus();
}
function renderGrabStatus(){
  stackingPanel?.update();
  if(!grabPanel?.isConnected)return;
  const diagnosticsOpen=grabPanel.querySelector('.grab-status-details')?.open??false;
  const r=grabRouteSnapshot?.data;
  const s=grabSnapshot?.data,age=grabSnapshot?Math.max(0,Math.floor((Date.now()-grabSnapshot.time)/1000)):null;
  const current=grabSnapshot?.live&&age<=5;
  const heading=node('h3',s?`${current?'最近快照':'历史快照'} · ${({route_pose:'机构姿态切换',idle:'空闲',prepare:'观察准备',acquire:'获取目标',align:'协同对准',vision_grace:'短暂漏检 · 减速等待',vision_pause:'视觉暂停 · 不会下降夹取',settle:'水平停稳',descend:'Z 下降',close:'夹爪闭合',lift:'带物料提起',turn:'Base 转向放置处',place:'下降至放置高度',release:'松爪放置',retract:'放置后提起',homing:'上电自动回零',hold:s.operation==='ring_place'&&s.result==='released'?'放置完成 · 已松爪提起':s.operation==='ring_align'?(s.reason==='ring_aligned_low'?'色环对准完成 · 保持低位观察':'色环对准完成 · 等待装料'):s.mode==='align'?'对准完成 · 未抓取':'等待人工验收',stopping:'正在停止',error:'故障停止',route:'路线运行',scan:'模拟扫码',calibration:'视觉响应标定',complete:s.operation==='calibration'?'标定完成 · 已更新视觉响应':'单件完成 · 可开始下一件',done:s.operation==='calibration'?'标定完成 · 已更新视觉响应':'单件完成 · 可开始下一件',car_index:'转盘分度',car_x:'X 到车载取放口',car_descend:'车载下降',car_lift:'退出仓位',outside_turn:'返回外侧',return_observe:'恢复观察姿态'})[s.state]||deviceText(s.state)}`:'尚无抓取状态');
  const freshness=node('p',s?`收到于 ${new Date(grabSnapshot.time).toLocaleTimeString('zh-CN',{hour12:false})} · ${age} 秒前`:'点击上方“读取任务状态”查看设备反馈。','form-note');
  const list=node('dl',undefined,'grab-readout');
  const value=(key,unit='')=>s?.[key]===null||s?.[key]===undefined?'—':`${s[key]}${unit}`;
  const rows=[['操作',deviceText(s?.operation||s?.op)],['场景',deviceText(s?.scene)],['仓位',value('slot')],['完成结果',deviceText(s?.result)],['视觉恢复',grabRecoveryText(s)],['本次或最近中断',value('loss_ms',' ms')],['本任务最长中断',value('loss_max_ms',' ms')],['模式',({fixed:'固定位置',align:'仅对准',pick:'视觉抓取'})[s?.mode]||s?.mode||'—'],['视觉目标',[10,11,12].includes(s?.color)?`${s.color-9}号同心圆`:grabColorOptions.find(([id])=>Number(id)===s?.color)?.[1]||value('color')],['视觉协议',s?.protocol||'未报告'],['坐标状态',visualStateText(s?.vision)],['MCU 接收序号',s?.rx_seq===null||s?.rx_seq===undefined?'未报告':String(s.rx_seq)],['响应模型',({initial:'联调初值 · 未实测',custom:'自定义参数 · 不代表已标定',unset:'未设置'})[s?.model]||'—'],['缺少标定',describeGrabMissing(s?.missing)],['参考状态',({boot_pending:'上电待验证',home_verifying:'正在验证三轴',home_verified:'三轴已验证',base_place_turn:'Base 已转向放置处',manual_home_all:'手动全部回零后待验证',manual_home_x:'手动 X 回零后待验证',manual_home_z:'手动 Z 回零后待验证',manual_home_base:'手动 Base 回零后待验证',manual_zero_all:'手动全部清零后待验证',manual_zero_x:'手动 X 清零后待验证',manual_zero_z:'手动 Z 清零后待验证',manual_zero_base:'手动 Base 清零后待验证',manual_base_move:'Base 已手动移动',x_reference_changed:'X 参考失效',z_reference_changed:'Z 参考失效',base_reference_changed:'Base 参考失效',all_reference_changed:'三轴参考失效'})[s?.ref_cause]||deviceText(s?.ref_cause)],['原因',reasonText(s?.reason)],['X 实际 / 目标',`${value('x')} / ${value('xt')} mm`],['Base 实际 / 目标',`${value('b')} / ${value('bt')} °`],['Z 实际 / 目标',`${value('z')} / ${value('zt')} mm`],['DX / DY',grabPixelText(s)],['坐标接收间隔',s?.age_source==='rx'?value('age',' ms'):'未报告'],['前向 / 左向速度',`${value('vf')} / ${value('vl')} mm/s`],['任务耗时',value('elapsed',' ms')],['停止请求',s?.stop_requested===1?'已请求':s?.stop_requested===0?'未请求':'—'],['最近停止请求反馈',s?.stop_confirmed===1?'该次已确认（结合当前阶段）':s?.stop_confirmed===0?'尚未确认':'—']];
  const details=node('details',undefined,'grab-status-details'),diagnostics=node('dl',undefined,'grab-readout');
  details.open=diagnosticsOpen;details.append(node('summary','完整反馈与诊断'));
  const primaryLabels=new Set(['完成结果','视觉目标','X 实际 / 目标','Z 实际 / 目标','Base 实际 / 目标','DX / DY']);
  for(const [label,text] of rows){const target=primaryLabels.has(label)?list:diagnostics;target.append(node('dt',label),node('dd',text));}
  const reply=node('p',grabReply?`${grabReply.live?'':'历史 · '}${displayReply(grabReply.line)}`:'设备回复将在此显示。','grab-reply');reply.classList.toggle('error',!!grabReply&&replyIsFault(grabReply.line));
  const routeReply=node('div',undefined,'grab-route-status');
  if(r){const old=!grabRouteSnapshot.live||Date.now()-grabRouteSnapshot.time>5000;const progress=(r.mode==='mission'||r.mode==='single')?`工位 ${r.station??'—'} · 入仓 ${r.stored??'—'} / 放出 ${r.placed??'—'}`:`第 ${r.segment??'—'} 段`;routeReply.append(node('h3',`${old?'历史 · ':''}${describeGrabRoute(r)}`),node('p',`${progress} · 来源 ${r.source==='test'?'固定蓝色测试':r.source==='simulated'?'模拟任务码':r.source||'—'} · ${r.code||'—'} · ${r.elapsed_ms??'—'} ms`, 'form-note'),node('p',`路线原因：${r.state==='config_wait'?describeGrabMissing(r.reason):reasonText(r.reason)}`,r.state==='error'?'grab-reply error':'grab-reply'));}
  details.append(diagnostics,reply);
  grabPanel.replaceChildren(routeReply,heading,freshness);
  if(s){
    const reason=node('p',`原因：${reasonText(s.reason)}`,'grab-reason');reason.classList.toggle('error',s.state==='error');grabPanel.append(list,reason);
    if(s.missing&&s.missing!=='none')grabPanel.append(node('p','缺少参数：'+describeGrabMissing(s.missing),'grab-reason error'));
  }
  if(grabReply&&replyIsFault(grabReply.line))grabPanel.append(node('p',displayReply(grabReply.line),'grab-reason error'));
  grabPanel.append(details);
}

function buildPages(){for(const key of Object.keys(titles).filter(x=>x!=='map')){pages[key]=node('div');pages[key].dataset.page=key;pages[key].hidden=true;$('module-content').append(pages[key]);}
const arm=pages.arm;arm.append(node('p','先选择电机轴，再使能并执行相对运动。夹爪独立控制。','form-note'));arm.append(node('small','当前对象'));const picker=node('div',undefined,'axis-picker');for(const [value,label] of [['base','底座 Base'],['z','升降 Z'],['x','伸缩 X'],['all','全部轴']]){const b=node('button',label);b.classList.toggle('active',value===axis);b.setAttribute('aria-pressed',String(value===axis));b.onclick=()=>{axis=value;picker.querySelectorAll('button').forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});updateControls();};picker.append(b);}arm.append(picker);actions(arm,[['enable','使能',{},true],['disable','失能',{},true],['state','读取状态',{},true],['stop','停止所选轴',{},true]]);arm.append(controlForm('pos',{sharedAxis:true}));const grip=node('div',undefined,'grip-row');grip.append(node('h3','夹爪'));actions(grip,[['grip-open','打开 · 2500 µs'],['grip-catch','夹取 · 1920 µs']]);arm.append(grip);arm.append(parameterLink('调整运动参数','motion','base_rpm','机械臂与夹爪'));advanced(arm,'回零与零点',['home','origin','zero']);
const carTray=node('section',undefined,'car-turntable-controls');
carTray.append(node('h3','车载转盘 · ID 8'),node('p','与机械臂轴选择独立。首次设零：失能 → 手动对齐第一仓 → 使能 → 保存机械零点。以后点击就近回零，完成后自动恢复第一仓参考。','form-note'));
actions(carTray,[['turntable-enable','使能转盘'],['turntable-disable','失能转盘'],['turntable-status','读取转盘状态'],['turntable-stop','停止转盘'],['turntable-save_zero','保存机械零点'],['turntable-home','就近回零']]);
carTray.append(controlForm('turntable-jog',{label:'转盘相对角度',idPrefix:'arm-'}));
carTray.append(parameterLink('设置仓位角度与转速','turntable','slot2_deg','机械臂与夹爪'));
arm.append(carTray);
const armCarPlace=node('section',undefined,'arm-car-place');
armCarPlace.append(controlForm('grab-place-car',{idPrefix:'arm-',compact:true,label:'机械臂车载放料',executeLabel:'转向、伸出并放料'}));
actions(armCarPlace,[['grab-pose-rear','收回朝后'],['grab-stop','停止机械臂放料']]);
const baseDetails=node('details',undefined,'advanced');baseDetails.append(node('summary','Base 到位诊断'));actions(baseDetails,[['grab-base-status','读取Base到位诊断']]);armCarPlace.append(baseDetails);arm.append(armCarPlace);

const ch=pages.chassis;
ch.append(node('p','先做整车运动测试；需要按地图走路线时，前往“地图与遥测”准备跑图。停止按钮始终在底部。','form-note'));
actions(ch,[['chassis-task','查看底盘状态']]);
ch.append(controlForm('chassis-run',{label:'整车平移与转向'}));
ch.append(node('p','前向速度：正数前进，负数后退；左向速度：正数左移，负数右移。角速度：正数逆时针。角速度为 0 时保持起步朝向，需 IMU 验证通过。','form-note'));
const wheelSection=node('details',undefined,'advanced');wheelSection.append(node('summary','四轮状态 · 查看反馈快照'));
actions(wheelSection,[['feedback-read','刷新四轮快照']]);wheelView=mountWheelFeedback(wheelSection);ch.append(wheelSection);
const realRoute=node('details',undefined,'glass map-controls map-details real-route-panel');realRoute.id='real-route-panel';realRoute.append(node('summary','准备跑图 · 绕障预演 / 固定路线执行'));
prepView=mountPreparation({root:realRoute,send:wire=>send(wire,true),beginTelemetry:nonce=>mapView.beginPreparedTelemetry(nonce),telemetry:()=>mapView?.snapshot(),getConfig:()=>bridge.config,saveConfig:c=>bridge.saveConfig(c),configAvailable:()=>bridge.available,isStopped:stopped,acquire:()=>bridge.acquirePreparation(),release:()=>bridge.releasePreparation(),changed:updateControls});
realRoute.append(node('p','从 (2250,150,90°) 出发，先斜移到 (2100,300)，再保持90°直行到扫码位 (2100,1200)。按固定15个目标经过扫码位、原料区、粗加工区、暂存区，最终返回起点。手动逐目标或停稳后自动继续；两批搬运在视觉任务页启动。','form-note'));
realRoute.append(controlForm('chassis-route-start',{label:'手动分段运行'}));
realRoute.append(controlForm('chassis-route-auto'),parameterLink('调整路线参数','radar','route_speed','地图 · 实车监控'));
actions(realRoute,[['chassis-route-next','手动下一段'],['chassis-route-cancel','取消路线'],['chassis-route-status','查询实车路线']]);
const realStatus=node('pre','实车路线状态不可用 · 请在地图页手动开启遥测；查询回复见收发记录。','real-route-status');realStatus.id='real-route-status';realStatus.setAttribute('role','status');realRoute.append(realStatus);

realRoute.append(node('p','需先完成驱动反馈配置、IMU 验证与地图锚点设置。路线到点表示编码器反馈估计进入容差，不能替代视觉精对准、扫码或抓取确认。刷新、重连和切换页面不会开始或进入下一段；取消请求与实际停止分别看设备状态。','form-note'));
const routeWorkspace=node('div',undefined,'route-workspace'),routeOps=node('div',undefined,'route-operations');const prepSection=realRoute.querySelector('.route-preparation');for(const child of [...realRoute.children])if(child.tagName!=='SUMMARY'&&child!==prepSection)routeOps.append(child);routeWorkspace.append(prepSection,routeOps);realRoute.append(routeWorkspace);for(const row of realRoute.querySelectorAll('.actions'))row.classList.add('map-action-grid');for(const form of realRoute.querySelectorAll('.control-form'))form.classList.add('map-compact-form');$('online-controls').after(realRoute);
$('simulation-actions').classList.add('map-action-grid');$('sim-controls').querySelector('.simulation-options').before($('simulation-actions'));$('route-editor').classList.add('map-details');$('sim-controls').append($('route-editor'));const speedLab=node('details',undefined,'map-details');speedLab.append(node('summary','速度曲线与模拟说明'),$('speed-lab-note'));$('sim-controls').append(speedLab);
const advancedMotion=node('details',undefined,'advanced');advancedMotion.append(node('summary','高级运动 · 指定航向与地图位移'));
advancedMotion.append(node('p','指定航向使用 HWT101 模块角度；地图位移使用场地坐标。先完成准备跑图，再使用这些操作。','form-note'));
advancedMotion.append(controlForm('chassis-turn'),controlForm('chassis-heading',{label:'保持指定模块航向'}),controlForm('chassis-move',{label:'地图相对位移 · 最长 300 mm'}),controlForm('chassis-origin',{label:'手动设置地图位姿 · 不移动小车'}));ch.append(advancedMotion);
const recovery=node('details',undefined,'advanced');recovery.append(node('summary','故障恢复 · 实物驱动已复位后使用'));
recovery.append(node('p','用于驱动器已实际复位后的总线恢复，不会复位硬件。先停止四轮采集并等待在途事务结束，再确认恢复。底部“解除停止锁定”仅解除网页软件停车锁，两者不同。','form-note'));
actions(recovery,[['feedback-off','停止四轮采集'],['reset-confirmed','确认驱动已复位']]);ch.append(recovery);
buildVisionWorkspace(pages.vision);
const sys=pages.system;buildGrabCalibrationPanel(sys);const bootNote=node('p',bootStage||'启动状态：尚未收到本次启动报告','form-note');bootNote.id='boot-status';sys.append(bootNote);actions(sys,[['bus-recover','一键恢复电机总线'],['bus-status','查看恢复结果']]);sys.append(node('p','误操作或通信错误后使用：停止任务、检查全部电机，成功后由你重新启动。不会重启主控、自动归零或续跑；恢复结果见终端。','form-note'));actions(sys,[['info','设备信息'],['help','固件帮助'],['imu-status','读取 IMU 状态']]);sys.append(node('p','上电先完成 Z/X/Base 回零，再静止验证 5 秒；失败后可手动重新验证。原生零偏标定约 20 秒，随后自动验证 30 秒；全过程保持静止。保存请求与断电保持验证分别查看。','form-note'));actions(sys,[['imu-cal-start','原生标定'],['imu-verify','重新验证 5 秒'],['imu-cal-cancel','取消标定/验证']]);sys.append(node('p','Z 轴角度清零：把当前朝向设为 0°，不消除漂移、不标定零偏。保持静止操作；跑图前重新验证 IMU 并设置地图起点。','form-note'));
actions(sys,[['imu-zero','Z 轴角度清零']]);
advanced(sys,'IMU 连续角度输出',['imu-stream-on','imu-stream-off']);
const routeSpeed=prefs.routeSpeed;
  const routeSpeeds=bindRouteSpeed({prefs,save,inputs:['chassis-route-start-speed','chassis-route-auto-speed','grab-route-speed','grab-mission-speed','radar-nav-speed','route-preview-speed'].map($).filter(Boolean),onChange:speed=>paramsView?.setRouteSpeed(speed)});
  if(prefs.previousRouteSpeed!==undefined)realRoute.append(node('p',`旧版启动速度 ${prefs.previousRouteSpeed} mm/s 已迁移为 200 mm/s，请核对；之后显式保存的 10～5000 mm/s 仍可使用。`,'form-note'));
  paramsView=mountParameterPage({root:pages.params,send:wire=>executeSend(wire,false,true,true),canSend,radarExchange,radarModel,
    onBusy:busy=>{parameterBusy=busy;updateControls();},routeSpeed,onRouteSpeed:speed=>routeSpeeds.set(speed)});
  qrView=mountQr({root:pages.qr,send,notify:toast,canSend});}
function route(){
  const next=parsePageRoute(location.hash),content=document.querySelector('.page-content');
  if(previousHash)navigationMemory.save(previousHash,content.scrollTop);
  if(next.page==='params'&&active!=='params'){navigationMemory.enterParameters(pendingParameterSource);paramsView.setReturn(navigationMemory.source);}
  pendingParameterSource=null;active=next.page;
  document.querySelector('.desk').classList.toggle('map-focused',active==='map');document.querySelector('.desk').classList.toggle('params-focused',active==='params');
  $('map-page').hidden=active!=='map';$('console-page').hidden=active==='map';for(const [key,page]of Object.entries(pages))page.hidden=key!==active;
  if(active==='params')paramsView.selectTab(next.tab,next.key);
  if(active==='vision'){visionTabs?.select(next.tab);grabPanel.hidden=['track','camera'].includes(next.tab);}
  document.querySelector('.feedback-panel').hidden=active==='vision'&&next.tab!=='camera';
  if(active==='map')radarView?.selectTab(next.tab);
  document.querySelectorAll('#navigation a').forEach(a=>a.setAttribute('aria-current',a.hash===`#${active}`?'page':'false'));
  $('module-title').textContent=titles[active];$('module-badge').textContent=active==='qr'?'任务码接收':active==='params'?'仅 RAM · 上电恢复默认 · 空白请读取':'手动操作';
  $('module-badge').classList.toggle('parameter-header-note',active==='params');$('module-badge').title=active==='params'?'设备参数仅存 STM32 RAM，重新上电恢复固件默认值；输入留空表示未读取。路线速度属于浏览器本地设置。':'';
  content.scrollTop=navigationMemory.restore(location.hash);
  if(active==='vision'){
    if(next.key==='status'&&!grabPanel.hidden)grabPanel.scrollIntoView({block:'start'});
  }
  previousHash=location.hash;updateControls();renderStatus();renderLogs();
}
document.addEventListener('click',event=>{
  const link=event.target.closest('a[href^="#params"]');if(!link)return;
  pendingParameterSource=link.hasAttribute('data-parameter-link')?{hash:location.hash||'#'+active,label:link.dataset.parameterSource||titles[active]}:null;
});
$('map-log-toggle').onclick=()=>{const open=document.querySelector('.desk').classList.toggle('logs-open');$('map-log-toggle').textContent=open?'收起日志':'展开日志';$('map-log-toggle').setAttribute('aria-expanded',String(open));renderLogs();};
$('raw-form').onsubmit=async e=>{e.preventDefault();const command=$('raw-command').value.trim();if(await send(command))$('raw-command').value='';};document.querySelectorAll('[data-stop]').forEach(b=>{b.dataset.send='1';b.onclick=()=>void send(b.dataset.stop);});
$('log-filters').onclick=e=>{const b=e.target.closest('[data-filter]');if(!b)return;filter=b.dataset.filter;document.querySelectorAll('[data-filter]').forEach(x=>x.classList.toggle('active',x===b));renderLogs();};$('log-module').onchange=renderLogs;$('log-format').onchange=()=>{prefs.logFormat=$('log-format').value;save();renderLogs();renderStatus();renderGrabStatus();};$('quiet-grab-log').onchange=()=>{prefs.quietGrabLog=$('quiet-grab-log').checked;save();unseen=0;renderLogs();};$('clear-log').onclick=()=>{logs=[];txCount=rxCount=unseen=errors=0;renderLogs();};$('terminal').onscroll=()=>{if($('terminal').scrollHeight-$('terminal').clientHeight-$('terminal').scrollTop>25)$('autoscroll').checked=false;};$('autoscroll').onchange=()=>{if($('autoscroll').checked)unseen=0;renderLogs();};$('new-logs').onclick=()=>{$('autoscroll').checked=true;unseen=0;renderLogs();};
$('export-log').onclick=()=>{const content=logs.map(l=>`[${l.time}] ${l.direction} ${l.text}`).join('\r\n');const url=URL.createObjectURL(new Blob(['\ufeff'+content],{type:'text/plain;charset=utf-8'}));const a=node('a');a.href=url;a.download=`yachiyo-${Date.now()}.txt`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
function applyTheme(){const theme=prefs.theme==='black'||(prefs.theme===undefined&&prefs.dark===false)?'black':'blue';document.body.classList.add('dark');document.body.dataset.theme=theme;$('theme-toggle').textContent=theme==='blue'?'◐ 切换黑色':'◐ 切换蓝色';$('theme-toggle').title=`当前为${theme==='blue'?'蓝色':'黑色'}背景`;}
$('theme-toggle').onclick=()=>{prefs.theme=document.body.dataset.theme==='blue'?'black':'blue';prefs.dark=true;save();applyTheme();};
applyTheme();buildPages();mapView=mountMap({send,isPreparing:()=>!!prepView?.busy||stopped(),onStatus:text=>$('subscription-note').textContent=text});radarDirectView=mountRadarDirectHost({root:$('radar-direct-workspace'),hostClient:radarHostClient,connectionController:sharedConnectionController});radarDirectView.connection({state:connectionState,port:bridge.port||$('serial-port').value||'',baudrate:Number($('baud').value)||115200});radarView=mountRadarWorkspace({model:radarModel,exchange:radarExchange,send,canSend,api:(path,body)=>bridge.api(path,body),mapView,monitorResult,onLiveState:receiveRadarLiveState,onBusy:busy=>{parameterBusy=busy;updateControls();},onTabChange:tab=>{const direct=tab==='radar';$('legacy-map-layout').hidden=direct;radarDirectView.setVisible(direct);}});
mountMonitorRadar({model:radarModel,store:monitorResult,workspace:radarView,api:(path,body)=>bridge.api(path,body),backupConfig:()=>radarDirectView.snapshot().config,preparationRoot:$('real-route-panel').querySelector('.route-preparation')});
window.addEventListener('hashchange',route);route();qrView.connection(false);updateControls();appendLog('SYS','八千代·巡航已就绪，请选择串口后连接；刷新端口列表不会打开设备或发送指令。');

setInterval(()=>{if(active==='vision'&&(grabSnapshot||grabRouteSnapshot))renderGrabStatus();},1000);
void bridge.start().then(refreshPorts);
