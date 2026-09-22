import {mountWheelFeedback} from './wheel-feedback.mjs';
import {createAttemptTracker} from './command-attempt.mjs?v=terminal-3';
import {mountPreparation} from './route-preparation.mjs?v=braking-5';
import {commands,buildCommand,frameCommand,describeReply,moduleForWire} from './protocol.mjs?v=braking-5';
import {BridgeLink,readOnlyCommand} from './bridge.mjs?v=dma-1';
import {mountMap} from './map-view.mjs?v=braking-5';
import {mountQr} from './qr-panel.mjs?v=bridge-1';
const $=id=>document.getElementById(id);
const node=(tag,text,cls)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;};
const titles={system:'系统与标定',chassis:'底盘',map:'地图与遥测',qr:'二维码',vision:'视觉任务',arm:'机械臂与夹爪'};
const storageKey='sky-atelier-v1';let prefs={};try{prefs=JSON.parse(localStorage.getItem(storageKey)||'{}')||{};}catch{}
function save(){try{localStorage.setItem(storageKey,JSON.stringify(prefs));}catch{toast('本机存储已满，设置未保存。');}}
let toastTimer;function toast(text){$('toast').textContent=text;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,3500);}
let active='system',axis='base',mapView,qrView,prepView,wheelView,txCount=0,rxCount=0,filter='ALL',logs=[],unseen=0,errors=0,sending=false;
const status=new Map(),forms=[],pages={};
function appendLog(direction,text){const owner=moduleForWire(text);logs.push({time:new Date().toLocaleTimeString('zh-CN',{hour12:false}),direction,text,owner});if(logs.length>500)logs.shift();if(direction==='TX')txCount++;if(direction==='RX')rxCount++;if(text.startsWith('ERR'))errors++;if(!$('autoscroll').checked)unseen++;renderLogs();}
function renderLogs(){const scroll=$('terminal').scrollTop;const scope=$('log-module').value;const entries=logs.filter(l=>(filter==='ALL'||l.direction===filter)&&(scope==='all'||l.owner===(scope==='current'?active:scope)||l.direction==='SYS'));const fragment=document.createDocumentFragment();for(const l of entries){const row=node('div',undefined,`log-entry ${l.direction.toLowerCase()} ${l.text.startsWith('ERR')?'error':''}`);row.append(node('time',l.time),node('b',l.direction),node('span',l.text));fragment.append(row);}if(!entries.length)fragment.append(node('p','本模块尚无收发记录，可切换“全部模块”查看。','empty-log'));$('terminal').replaceChildren(fragment);$('log-count').textContent=`${logs.length}${errors?' · 错误 '+errors:''}`;$('tx-count').textContent=txCount;$('rx-count').textContent=rxCount;$('terminal').scrollTop=$('autoscroll').checked?$('terminal').scrollHeight:scroll;$('new-logs').hidden=!unseen;$('new-logs').textContent=`${unseen} 条新记录 · 恢复跟随`;}
let routeLocked=false;
let connectionState='disconnected';
function receive(line){appendLog('RX',line);
  if(line==='STM32 mechanical arm console ready'){
    status.clear();routeLocked=false;
    for(const view of [wheelView,mapView,qrView,prepView]){view?.connection(false);view?.connection(bridge.connected);}
    appendLog('SYS','已收到 STM32 启动信息；旧状态已清除，请保持静止，等待 IMU 自动验证 5 秒通过后准备跑图并手动开启订阅。');renderStatus();
  }
  if(/^OK imu zero started/.test(line)){
    mapView?.connection(bridge.connected);prepView?.connection(false);prepView?.connection(bridge.connected);
    appendLog('SYS','Z 轴归零已受理。完成后查询 IMU 状态查看新角度；跑图前重新验证并设置地图起点。');
  }
wheelView?.receive(line);mapView?.receive(line);
const routeState=line.match(/^(?:OK|ERR) chassis route state=(\w+)/)?.[1]??(line.startsWith('@CHASSIS ')?mapView?.snapshot()?.route?.state:null);if(routeState){routeLocked=['running','stopping','waiting'].includes(routeState);updateControls();}
qrView?.receive(line);prepView?.receive(line);const owner=moduleForWire(line);if(owner!=='manual'&&!line.startsWith('@CHASSIS ')&&/^(OK |ERR |\[IMU CAL\]|EVT qr )/.test(line)){status.set(owner,{line,time:Date.now(),live:bridge.connected});renderStatus();}}
function connection(state){connectionState=state;const connected=state==='connected';if(connected&&bridge.port){const select=$('serial-port');if(!Array.from(select.options).some(o=>o.value===bridge.port)){const o=node('option',bridge.port);o.value=bridge.port;select.append(o);}select.value=bridge.port;}wheelView?.connection(connected);mapView?.connection(connected);qrView?.connection(connected);prepView?.connection(connected);$('connection-status').textContent=({connected:`本地服务已连接 ${bridge.port}`,connecting:`正在连接 ${$('serial-port').value}…`,disconnecting:'正在断开…',disconnected:'尚未连接'})[state];$('connection-dot').classList.toggle('online',connected);$('connect').disabled=state!=='disconnected'||!$('serial-port').value;$('serial-port').disabled=state!=='disconnected';$('refresh-ports').disabled=state!=='disconnected';$('disconnect').disabled=!connected;$('baud').disabled=state!=='disconnected';for(const value of status.values())value.live=false;if(!connected)$('subscription-note').textContent='';updateControls();renderStatus();if(connected)appendLog('SYS','已连接，尚未自动发送任何指令。');}
function communicationError(error){appendLog('SYS',`通信异常：${error.message}`);toast(error.message);prepView?.refreshConfiguration();updateControls();}
function stopped(){return bridge.stopLatched;}
function bridgeEvent(e){if(e.kind==='preparation'&&e.active===false&&e.owner===bridge.preparationOwner&&prepView?.busy)prepView.cancel('共享准备锁定已结束，请重新准备。');if(e.kind==='tx')appendLog('TX',e.text);if(e.kind==='system')appendLog('SYS',e.text||e.error||'本地服务状态变化');if(e.kind==='stop'){if(e.latched)prepView?.cancel('停止已锁定，准备已取消；解除锁定后也不会自动继续。');$('stop-status').textContent=e.latched?({requested:'正在提交停车请求',sent:'停车请求已发出，等待设备回复',accepted:e.tx_complete?'停车发送完成，等待四轮反馈确认':'请求已受理，等待停车发送完成',confirmed:'四轮反馈已确认停车',timeout:'停车确认超时，请检查实车',disconnected:'连接已断开，无法确认停车'})[e.status]||'停止已锁定，等待停车确认':e.status==='idle'?'本地服务就绪 · CMD Ctrl+C 请求停车':'停止锁定已解除 · 任务不会自动继续';$('resume-stop').hidden=!e.latched;updateControls();prepView?.refreshConfiguration();}}
const bridge=new BridgeLink({receive,state:connection,error:communicationError,event:bridgeEvent,configuration:()=>prepView?.refreshConfiguration()});
const preparationStop=wire=>/^(?:chassis (?:stop|route cancel)|stop (?:all|base|z|x)|vision stop|material stop)$/.test(wire);
function mayQuery(wire){return readOnlyCommand(wire)&&(!prepView?.busy||!prepView.pending);}
async function requestStop(){attempts.supersede();$('attempt-wire').textContent='chassis stop';$('attempt-status').textContent='请求停车 · 确认见底部状态';$('latest-command').dataset.state='pending';appendLog('SYS','请求停车 → chassis stop');prepView?.cancel('已中止准备并请求停车；请查看停止状态。');try{await bridge.stop();return true;}catch(error){communicationError(error);return false;}}
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
  if(/^system\s+reset$/.test(wire.trim())&&!window.confirm('重启 STM32？\n仅在整车静止、任务空闲时操作。主控将重新初始化，外部驱动器不会断电复位。重启后保持静止，等待 IMU 自动验证 5 秒通过后准备跑图。'))return false;
  return attempts.run(wire,()=>executeSend(wire,fromPreparation));
}
async function executeSend(wire,fromPreparation=false){
  if(stopped()&&!readOnlyCommand(wire)){appendLog('SYS','停止已锁定，本次指令未发送。');toast('停止已锁定，请确认停车后解除锁定。');return false;}
  if(prepView?.busy&&!fromPreparation){if(preparationStop(wire)||wire==='imu cal cancel')prepView.cancel('已中止准备；实际停止请查看设备反馈。');else if(!mayQuery(wire)){toast('准备正在等待专属回复，请稍后查询，或先取消准备。停止始终可用。');return false;}else prepView.cancel('已退出准备并读取状态，避免查询回复混入准备流程。');}
  if(sending&&!fromPreparation){toast('上一条指令仍在发送，本次未排队。');return false;}
  try{frameCommand(wire);sending=Number(sending)+1;updateControls();await bridge.send(wire,fromPreparation);toast(`已发送：${wire}`);if(wire==='chassis stream off')$('subscription-note').textContent='已请求关闭遥测';return true;}catch(error){appendLog('SYS',error.message);toast(error.message);return false;}finally{sending=Math.max(0,Number(sending)-1);updateControls();}
}
function canSend(wire){return !(routeLocked&&/^chassis route (start|auto)/.test(wire))&&bridge.connected&&!sending&&(!stopped()||readOnlyCommand(wire))&&(!prepView?.busy||preparationStop(wire)||wire==='imu cal cancel'||mayQuery(wire));}
function updateControls(){qrView?.refreshControls();document.querySelectorAll('[data-send]').forEach(b=>{const wire=b.dataset.stop||b.title||'';b.disabled=!canSend(wire)||(b.dataset.single==='1'&&axis==='all');});$('raw-send').disabled=!bridge.connected||!!sending||(!!prepView?.busy&&!!prepView.pending);$('device-reset').disabled=!canSend('system reset');$('service-stop').disabled=!bridge.available;$('resume-stop').disabled=!bridge.available;for(let i=forms.length-1;i>=0;i--){if(forms[i].root.isConnected)forms[i].update();else forms.splice(i,1);}}
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
function renderStatus(){const s=status.get(active);$('reply-title').textContent=s?`${s.live?'':'历史 · '}${describeReply(s.line)}`:(bridge.connected?'已连接 · 请读取状态':'尚未连接');$('reply-text').textContent=s?.line||'状态由设备回复确认，点击按钮不会预先标为成功。';$('reply-time').textContent=s?`最近回复 ${new Date(s.time).toLocaleTimeString('zh-CN',{hour12:false})}`:'尚无本模块回复';}
function definition(id){const c=commands.find(c=>c.id===id);if(!c)throw new Error(`缺少命令定义 ${id}`);return c;}
function action(id,label,preset={},useAxis=false){const c=definition(id),b=node('button',label||c.label);b.type='button';b.dataset.send='1';if(useAxis&&!c.fields.find(f=>f.key==='axis')?.options?.some(([v])=>v==='all'))b.dataset.single='1';const wire=()=>buildCommand(id,{...Object.fromEntries(c.fields.map(f=>[f.key,f.value])),...preset,...(useAxis?{axis}: {})});const hint=()=>{try{b.title=wire();}catch{b.title='请选择单轴';}};b.onmouseenter=hint;b.onfocus=hint;hint();b.onclick=()=>{try{void send(wire());}catch(e){toast(e.message);}};return b;}
function actions(container,list){const row=node('div',undefined,'actions');for(const item of list)row.append(action(...(Array.isArray(item)?item:[item])));container.append(row);return row;}
function controlForm(id,{sharedAxis=false,label}={}){const c=definition(id),root=node('form',undefined,'control-form');root.append(node('h3',label||c.label));const fields=node('div',undefined,'fields'),inputs={};for(const f of c.fields){if(sharedAxis&&f.key==='axis')continue;const wrap=node('label',f.label);let input;if(f.options){input=node('select');if(f.value===''){const placeholder=node('option','请选择（需人工确认）');placeholder.value='';input.append(placeholder);}for(const [v,t] of f.options){const o=node('option',t);o.value=v;input.append(o);}}else{input=node('input');input.type='number';input.min=f.min;input.max=f.max;input.step=f.step??1;input.required=true;}input.name=f.key;input.value=f.value??'';input.id=`${id}-${f.key}`;wrap.append(input);fields.append(wrap);inputs[f.key]=input;}root.append(fields);const note=node('p',c.note||'每次执行发送一条指令，以设备回复为准。','form-note');root.append(note);const preview=node('div',undefined,'preview'),code=node('code'),buttons=node('div',undefined,'form-actions'),copy=node('button','复制'),execute=node('button',id==='pos'?'执行相对运动':'执行','primary');copy.type='button';execute.type='submit';buttons.append(copy,execute);preview.append(code,buttons);const validation=node('p','','validation');validation.setAttribute('aria-live','polite');root.append(preview,validation);let wire='';function update(){if(['chassis-route-start','chassis-route-auto'].includes(id))inputs.speed.disabled=routeLocked;try{wire=buildCommand(id,{...Object.fromEntries(Object.entries(inputs).map(([k,e])=>[k,e.value])),...(sharedAxis?{axis}: {})});code.textContent=wire;validation.textContent='';execute.disabled=!canSend(wire);copy.disabled=false;}catch(e){wire='';code.textContent='请补全有效参数';validation.textContent=e.message;execute.disabled=copy.disabled=true;}if(inputs.seconds&&['chassis-run','chassis-heading'].includes(id)){const t=Number(inputs.seconds.value);note.textContent=`起步 0.5 秒 + 保持 ${Number.isFinite(t)?t:'—'} 秒 + 停车 0.5 秒，总时长 ${Number.isFinite(t)?Math.round((t+1)*1000)/1000:'—'} 秒。${c.note||''}`;}}for(const input of Object.values(inputs)){input.oninput=update;input.onchange=update;}root.onsubmit=e=>{e.preventDefault();update();if(wire)void send(wire);};copy.onclick=()=>void copyText(wire);forms.push({root,update});update();return root;}
async function copyText(text){try{await navigator.clipboard.writeText(text);toast('已复制。');}catch{toast('无法复制，请手动选择文本。');}}
function advanced(container,title,ids){const details=node('details',undefined,'advanced');details.append(node('summary',title));const selector=node('select');selector.setAttribute('aria-label',title);const placeholder=node('option','选择调试操作');placeholder.value='';selector.append(placeholder);for(const id of ids){const c=definition(id),o=node('option',c.label);o.value=id;selector.append(o);}const target=node('div');selector.onchange=()=>{target.replaceChildren();if(selector.value){const c=definition(selector.value);if(c.fields.length)target.append(controlForm(c.id,{sharedAxis:c.fields.some(f=>f.key==='axis')}));else actions(target,[c.id]);}updateControls();};details.append(selector,target);container.append(details);}
function tabs(container,items){const nav=node('div',undefined,'mode-tabs'),body=node('div');const buttons=[];for(const [title,render] of items){const b=node('button',title);b.type='button';b.onclick=()=>{buttons.forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});body.replaceChildren();render(body);updateControls();};nav.append(b);buttons.push(b);}container.append(nav,body);buttons[0].click();}
function buildPages(){for(const key of Object.keys(titles).filter(x=>x!=='map')){pages[key]=node('div');pages[key].dataset.page=key;pages[key].hidden=true;$('module-content').append(pages[key]);}
const arm=pages.arm;arm.append(node('p','先选择电机轴，再使能并执行相对运动。夹爪独立控制。','form-note'));arm.append(node('small','当前对象'));const picker=node('div',undefined,'axis-picker');for(const [value,label] of [['base','底座 Base'],['z','升降 Z'],['x','伸缩 X'],['all','全部轴']]){const b=node('button',label);b.classList.toggle('active',value===axis);b.setAttribute('aria-pressed',String(value===axis));b.onclick=()=>{axis=value;picker.querySelectorAll('button').forEach(x=>{x.classList.toggle('active',x===b);x.setAttribute('aria-pressed',String(x===b));});updateControls();};picker.append(b);}arm.append(picker);actions(arm,[['enable','使能',{},true],['disable','失能',{},true],['state','读取状态',{},true],['stop','停止所选轴',{},true]]);arm.append(controlForm('pos',{sharedAxis:true}));const grip=node('div',undefined,'grip-row');grip.append(node('h3','夹爪'));actions(grip,[['grip-open','打开'],['grip-catch','夹取']]);arm.append(grip);advanced(arm,'回零、零点与运动参数',['home','origin','zero','config-get','config-set']);
const ch=pages.chassis;
ch.append(node('p','先做整车运动测试；需要按地图走路线时，前往“地图与遥测”准备跑图。停止按钮始终在底部。','form-note'));
actions(ch,[['chassis-task','查看底盘状态']]);
ch.append(controlForm('chassis-run',{label:'整车平移与转向'}));
ch.append(node('p','前向速度：正数前进，负数后退；左向速度：正数左移，负数右移。角速度：正数逆时针。角速度为 0 时保持起步朝向，需 IMU 验证通过。','form-note'));
const wheelSection=node('details',undefined,'advanced');wheelSection.append(node('summary','四轮状态 · 查看反馈快照'));
actions(wheelSection,[['feedback-read','刷新四轮快照']]);wheelView=mountWheelFeedback(wheelSection);ch.append(wheelSection);
const realRoute=node('details',undefined,'real-route-panel');realRoute.id='real-route-panel';realRoute.open=true;realRoute.append(node('summary','准备跑图 · 实车完整路线 · 16段'));
prepView=mountPreparation({root:realRoute,send:wire=>send(wire,true),beginTelemetry:nonce=>mapView.beginPreparedTelemetry(nonce),telemetry:()=>mapView?.snapshot(),getConfig:()=>bridge.config,saveConfig:c=>bridge.saveConfig(c),configAvailable:()=>bridge.available,isStopped:stopped,acquire:()=>bridge.acquirePreparation(),release:()=>bridge.releasePreparation(),changed:updateControls});
realRoute.append(node('p','从 (2250,150,90°) 出发，按地图默认16个站点经过扫码位、原料区、粗加工区、暂存区，最终返回起点。选择手动逐段或自动停稳1秒后继续；不自动执行扫码与抓放。','form-note'));
realRoute.append(controlForm('chassis-route-start',{label:'手动分段运行'}));
realRoute.append(controlForm('chassis-route-auto'));
actions(realRoute,[['chassis-route-next','手动下一段'],['chassis-route-cancel','取消路线'],['chassis-route-status','查询实车路线']]);
const realStatus=node('pre','实车路线状态不可用 · 请在地图页手动开启遥测；查询回复见收发记录。','real-route-status');realStatus.id='real-route-status';realStatus.setAttribute('role','status');realRoute.append(realStatus);

realRoute.append(node('p','需先完成驱动反馈配置、IMU 验证与地图锚点设置。路线到点表示编码器反馈估计进入容差，不能替代视觉精对准、扫码或抓取确认。刷新、重连和切换页面不会开始或进入下一段；取消请求与实际停止分别看设备状态。','form-note'));
const routeWorkspace=node('div',undefined,'route-workspace'),routeOps=node('div',undefined,'route-operations');const prepSection=realRoute.querySelector('.route-preparation');for(const child of [...realRoute.children])if(child.tagName!=='SUMMARY'&&child!==prepSection)routeOps.append(child);routeWorkspace.append(prepSection,routeOps);realRoute.append(routeWorkspace);$('map-page').insertBefore(realRoute,$('map-page').querySelector('.map-layout'));
const advancedMotion=node('details',undefined,'advanced');advancedMotion.append(node('summary','高级运动 · 指定航向与地图位移'));
advancedMotion.append(node('p','指定航向使用 HWT101 模块角度；地图位移使用场地坐标。先完成准备跑图，再使用这些操作。','form-note'));
advancedMotion.append(controlForm('chassis-heading',{label:'保持指定模块航向'}),controlForm('chassis-move',{label:'地图相对位移 · 最长 300 mm'}),controlForm('chassis-origin',{label:'手动设置地图位姿 · 不移动小车'}));ch.append(advancedMotion);
const recovery=node('details',undefined,'advanced');recovery.append(node('summary','故障恢复 · 实物驱动已复位后使用'));
recovery.append(node('p','用于驱动器已实际复位后的总线恢复，不会复位硬件。先停止四轮采集并等待在途事务结束，再确认恢复。底部“解除停止锁定”仅解除网页软件停车锁，两者不同。','form-note'));
actions(recovery,[['feedback-off','停止四轮采集'],['reset-confirmed','确认驱动已复位']]);ch.append(recovery);
tabs(pages.vision,[['任务执行',body=>{body.append(node('p','Base 回零 → 内部请求识别 → 底盘搜索 → 底盘/X 对准。当前终点为对准完成。','form-note'));actions(body,[['material-status','读取任务状态'],['material-stop','停止物料任务']]);body.append(controlForm('material-auto'));}],['相机调试',body=>{body.append(node('p','仅识别，不主动驱动机构；停止识别请求不等于停止物料任务。','form-note'));actions(body,[['camera-status','读取相机状态'],['camera-stop','停止识别请求']]);body.append(controlForm('camera-material'));}],['标定与分步测试',body=>{actions(body,[['vision-status','读取 Base/X 状态'],['vision-stop','停止 Base/X 对准']]);advanced(body,'参考与两种标定路径',['vision-ref','vision-calib-material','vision-calib-chassis']);body.append(controlForm('vision-material',{label:'Base/X 独立对准'}));}]]);
const sys=pages.system;actions(sys,[['info','设备信息'],['help','固件帮助'],['imu-status','读取 IMU 状态']]);sys.append(node('p','上电后自动验证 5 秒，请保持整车静止；失败后可手动重新验证。原生零偏标定约 20 秒，随后自动验证 30 秒；全过程保持静止。保存请求与断电保持验证分别查看。','form-note'));actions(sys,[['imu-cal-start','原生标定'],['imu-verify','重新验证 5 秒'],['imu-cal-cancel','取消标定/验证']]);sys.append(node('p','Z 轴角度清零：把当前朝向设为 0°，不消除漂移、不标定零偏。保持静止操作；跑图前重新验证 IMU 并设置地图起点。','form-note'));
actions(sys,[['imu-zero','Z 轴角度清零']]);
advanced(sys,'IMU 连续角度输出',['imu-stream-on','imu-stream-off']);
qrView=mountQr({root:pages.qr,send,notify:toast,canSend});}
function route(){const key=location.hash.slice(1);active=titles[key]?key:'system';document.querySelector('.page-content').scrollTop=0;$('map-page').hidden=active!=='map';$('console-page').hidden=active==='map';for(const [key,page] of Object.entries(pages))page.hidden=key!==active;document.querySelectorAll('#navigation a').forEach(a=>a.setAttribute('aria-current',a.hash===`#${active}`?'page':'false'));$('module-title').textContent=titles[active];$('module-badge').textContent=active==='qr'?'任务码接收':'手动操作';updateControls();renderStatus();renderLogs();}
$('raw-form').onsubmit=async e=>{e.preventDefault();const command=$('raw-command').value.trim();if(await send(command))$('raw-command').value='';};document.querySelectorAll('[data-stop]').forEach(b=>{b.dataset.send='1';b.onclick=()=>void send(b.dataset.stop);});
$('log-filters').onclick=e=>{const b=e.target.closest('[data-filter]');if(!b)return;filter=b.dataset.filter;document.querySelectorAll('[data-filter]').forEach(x=>x.classList.toggle('active',x===b));renderLogs();};$('log-module').onchange=renderLogs;$('clear-log').onclick=()=>{logs=[];txCount=rxCount=unseen=errors=0;renderLogs();};$('terminal').onscroll=()=>{if($('terminal').scrollHeight-$('terminal').clientHeight-$('terminal').scrollTop>25)$('autoscroll').checked=false;};$('autoscroll').onchange=()=>{if($('autoscroll').checked)unseen=0;renderLogs();};$('new-logs').onclick=()=>{$('autoscroll').checked=true;unseen=0;renderLogs();};
$('export-log').onclick=()=>{const content=logs.map(l=>`[${l.time}] ${l.direction} ${l.text}`).join('\r\n');const url=URL.createObjectURL(new Blob(['\ufeff'+content],{type:'text/plain;charset=utf-8'}));const a=node('a');a.href=url;a.download=`yachiyo-${Date.now()}.txt`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
function applyTheme(){const dark=prefs.dark!==false;document.body.classList.toggle('dark',dark);$('theme-toggle').textContent=dark?'◐ 浅色':'◐ 深色';}
$('theme-toggle').onclick=()=>{prefs.dark=!(prefs.dark!==false);save();applyTheme();};
applyTheme();buildPages();mapView=mountMap({send,isPreparing:()=>!!prepView?.busy||stopped(),onStatus:text=>$('subscription-note').textContent=text});window.addEventListener('hashchange',route);route();qrView.connection(false);updateControls();appendLog('SYS','八千代·巡航已就绪，请选择串口后连接；刷新端口列表不会打开设备或发送指令。');

void bridge.start().then(refreshPorts);
