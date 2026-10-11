import {RouteProfileRunner,profileResultText,profilePhaseText} from './route-profile.mjs';
import {RADAR_FIXED_MASK,RADAR_GRID,radarStateText} from './radar-model.mjs';
import {DEFAULT_BODY_SIZE,validBodySize} from './route-model.mjs';
import {installationParameters,referenceInstallationToCurrent} from './radar-installation.mjs';
import {appendArmMarker} from './vehicle-marker.mjs';
const same=(a,b)=>a&&b&&['session','origin','scan','map','plan'].every(k=>a[k]===b[k]);
const MONITOR_GRID=Object.freeze([150,550,1000,1400,1850,2250]);
export const monitorGrid=snapshot=>Array.isArray(snapshot?.grid)?snapshot.grid.slice(0,6):snapshot?.g===2?RADAR_GRID:MONITOR_GRID;

/** Scan protection is a firmware rule, not removal of the recorded counts. */
export function monitorProtectedMask(snapshot){
  if(snapshot?.a!==3&&snapshot?.g!==4)return 0;
  const pose=snapshot.pose,grid=monitorGrid(snapshot);
  let mask=(1<<2)|(1<<10)|(1<<14)|(1<<22);
  if(snapshot.g===4){if(snapshot.start_id===1)mask|=1<<24;else if(snapshot.start_id===2)mask|=1<<4;return mask;}
  if(pose){const col=grid.findIndex((edge,i)=>i<5&&pose[0]>=edge&&(pose[0]<grid[i+1]||i===4&&pose[0]===grid[i+1]));
    const row=grid.findIndex((edge,i)=>i<5&&pose[1]>=edge&&(pose[1]<grid[i+1]||i===4&&pose[1]===grid[i+1]));
    const bit=row*5+col;if(col===4&&(row===0||row===4))mask|=1<<bit;
  }
  return mask;
}

/** Gate the real-car entry separately from the local preview. */
export function monitorExecutionGate(model,pair,current){
  if(!pair?.map||!pair?.path||!same(pair.map,pair.path))return '请先扫描并读取完整地图与路线';
  if(!model?.connected)return '请先连接串口，不能执行实车路线';
  if(current===false)return '当前设备不是最新地图与路线，请重新读取同圈结果';
  if(!model.status||model.status.state!=='ready')return '设备尚未完成静止扫描，请等待地图与路线就绪';
  if(model.status.applicable===false||model.status.applicable===0)return '当前地图参数或原点已变化，请重新扫描';
  if(pair.path.start===1)return '右上起点仅支持路径查看，不能执行实车路线';
  if(pair.path.valid===false)return `当前雷达规划不可达 · 第 ${pair.path.failed_leg??'未知'} 段`;
  if(!Array.isArray(pair.path.points)||pair.path.points.length<2)return '当前雷达规划没有可执行路点';
  return '';
}

/** Only publish a coherent capture; incomplete downloads retain the last pair. */
export class MonitorResult {
  pair=null;
  update(map,path){
    if(same(map,path)&&map.counts?.length===25&&Array.isArray(path.points))this.pair={map,path};
    return this.pair;
  }
  current(model){return !!(this.pair&&!this.pair.map.historical&&!this.pair.path.historical&&model.connected&&model.status?.state==='ready'&&model.status.applicable!==false&&model.status.applicable!==0&&same(this.pair.map,model.status));}
}

/** Convert F1 mounting into the competition frame; keep its raw-angle filters. */
export function backupInstallation(config){
  const m=config.map,f=config.filter;
  const values={lidar_x_mm:m.radarXmm,lidar_y_mm:m.radarYmm,zero_deg:90,...config.installation,
    angle_min_tenths:Math.round(f.minAngleDeg*10),angle_max_tenths:Math.round(f.maxAngleDeg*10),
    distance_min_mm:f.minDistanceMm,distance_max_mm:f.maxDistanceMm,
    energy_min:f.minEnergy,energy_max:f.maxEnergy,threshold:m.obstaclePointThreshold};
  return config.coordinate_frame==='current'?values:referenceInstallationToCurrent(values);
}

export function previewBodySize(lengthValue,widthValue){
  const bodySize={length_mm:Number(lengthValue),width_mm:Number(widthValue)};
  if(!validBodySize(bodySize))throw new Error('预演车体长宽必须大于 0 且不超过 300 mm');
  return bodySize;
}

const previewBodySizeText=size=>`${size.length_mm}×${size.width_mm} mm`;

/** Pure preview: C selects all detours/phases; this function has no serial sender. */
export async function monitorPreview(pair,api,speed){
  if(!pair?.map||!same(pair.map,pair.path))throw new Error('请先扫描并读取完整地图与路线');
  const upper=pair.path.start===1;
  const result=await api('radar/plan',{mask:pair.map.mask,points:[],start_mode:upper?'upper':'lower',speed_mm_s:Number(speed)});
  if(!result.valid)return {result,profile:null,runner:null,message:`C 规划不可达 · 第 ${result.failed_leg} 路段`};
  if(Array.isArray(result.points)&&JSON.stringify(result.points)!==JSON.stringify(pair.path.points))
    throw new Error('设备路线与当前 C 核心不一致，不能用新路线预演旧结果；请烧录新固件并重新扫描');
  const profile=result.route_profiles?.planned;
  if(!profile?.phases?.length)return {result,profile:null,runner:null,message:upper?'右上起点仅支持路径查看，尚无实车阶段预览':'C 路线暂无可播放阶段'};
  const runner=new RouteProfileRunner(profile,{mask:result.mask??pair.map.mask});runner.advance(100000);
  return {result,profile,runner,message:profileResultText(runner)};
}

export function mountMonitorRadar({model,store,workspace,api,backupConfig,preparationRoot}){
  const $=id=>document.getElementById(id),ns='http://www.w3.org/2000/svg';
  const element=(tag,text)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
  const svg=(tag,attrs)=>{const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);return e;};
  let preview=null,playing=null,busy=false,executing=false,executionRequestAt=0,last=0;
  const paramsCurrent=()=>model.connected&&model.paramsLive&&model.snapshots?.params?.session===model.identity?.session;
  const prep=element('section');prep.className='monitor-preparation';
  prep.append(element('h3','雷达绕障 · 跑图前预演'));
  prep.append(element('p','使用同圈障碍地图和当前固件 C 路线阶段，仅在网页预演，不启动底盘或机械臂。'));
  const actions=element('div');actions.className='map-buttons map-action-grid';
  const check=element('button','检查绕障路线'),play=element('button','播放预演'),pause=element('button','暂停预演');
  for(const b of [check,play,pause])b.type='button';actions.append(check,play,pause);prep.append(actions);
  const modelFields=element('div');modelFields.className='fields monitor-preview-model';
  const lengthLabel=element('label','预演车体长 / mm'),widthLabel=element('label','预演车体宽 / mm');
  const lengthInput=element('input'),widthInput=element('input');
  for(const [input,id] of [[lengthInput,'monitor-preview-length'],[widthInput,'monitor-preview-width']]){
    input.id=id;input.type='number';input.min='1';input.max='300';input.step='0.1';input.value=String(DEFAULT_BODY_SIZE[id.endsWith('length')?'length_mm':'width_mm']);
  }
  lengthLabel.append(lengthInput);widthLabel.append(widthInput);modelFields.append(lengthLabel,widthLabel);prep.append(modelFields);
  prep.append(element('p','“检查绕障路线”按300×300 mm整格范围检查。播放遇到障碍格风险会提示并继续；明确边界和固定障碍仍停止。橙色标记为机械臂：中左安装、默认朝后，随车旋转。'));
  const resultText=element('p','先完成静止扫描，再检查绕障路线。');resultText.className='map-state';resultText.setAttribute('role','status');prep.append(resultText);
  const execute=element('button','执行扫描后的雷达路线');execute.type='button';execute.id='monitor-radar-execute';execute.className='primary';
  const cancel=element('button','取消雷达路线');cancel.type='button';cancel.id='monitor-radar-cancel';
  const executeActions=element('div');executeActions.className='map-buttons map-action-grid';executeActions.append(execute,cancel);prep.append(executeActions);
  const executeNote=element('p','只执行当前设备最新的右下起点规划路线；可先扫描后准备，也可先准备后扫描。执行前完成准备跑图，网页预演不会驱动车辆。');executeNote.className='map-note';prep.append(executeNote);
  const details=element('details');details.className='map-details';details.append(element('summary','C 路线阶段与工位交接'));const phases=element('ol');details.append(phases);prep.append(details);preparationRoot.append(prep);
  const routeLayer=svg('path',{id:'monitor-preview-path',fill:'none',stroke:'#18b9d1','stroke-width':9,'stroke-dasharray':'18 12','pointer-events':'none'});
  const carLayer=svg('g',{id:'monitor-preview-car','pointer-events':'none'});$('map-world').append(routeLayer,carLayer);
  const installationLayer=svg('g',{id:'monitor-installation-arrow','pointer-events':'none'});$('map-world').append(installationLayer);
  const installation=element('section');installation.className='monitor-installation';installation.append(element('h3','雷达坐标'));
  const direction=element('p');direction.id='monitor-installation-direction';direction.className='map-note';installation.append(direction);
  const installationFields=element('div');installationFields.className='fields';
  const coordinates={lower:{lidar_x_mm:2280,lidar_y_mm:314},upper:{lidar_x_mm:2050,lidar_y_mm:2250}};
  const dirty={lower:false,upper:false};let activeStart=$('monitor-capture').value==='upper'?'upper':'lower';
  const validCoordinate=value=>String(value??'').trim()!==''&&Number.isFinite(Number(value))&&Number(value)>=0&&Number(value)<=2400;
  const validCoordinates=value=>value&&['lidar_x_mm','lidar_y_mm'].every(key=>validCoordinate(value[key]));
  try{
    const stored=JSON.parse(globalThis.localStorage?.getItem('local-car-radar-start-coordinates-v3')??'null');
    if(stored){for(const start of ['lower','upper'])if(validCoordinates(stored[start])){coordinates[start]={...stored[start]};dirty[start]=true;}}
    else{
      const previous=JSON.parse(globalThis.localStorage?.getItem('local-car-radar-installation-draft-v2')??'null');
      let legacy=previous?.draft??JSON.parse(globalThis.localStorage?.getItem('local-car-radar-installation-draft-v1')??'null');
      if(legacy&&!previous&&Object.entries(installationParameters('case1')).every(([key,value])=>legacy[key]===value))legacy=installationParameters('case1','current');
      if(validCoordinates(legacy)){const start=previous?.preset==='upper'?'upper':'lower';coordinates[start]={lidar_x_mm:legacy.lidar_x_mm,lidar_y_mm:legacy.lidar_y_mm};dirty[start]=true;}
    }
  }catch{}
  const installationInputs={};
  for(const[key,labelText]of [['lidar_x_mm','雷达 X / mm'],['lidar_y_mm','雷达 Y / mm']]){
    const label=element('label',labelText),input=element('input');input.type='number';input.value=String(coordinates[activeStart][key]);input.min='0';input.max='2400';input.step='.1';input.id=`monitor-installation-${key}`;
    input.oninput=()=>{rememberCoordinates();render();};label.append(input);installationFields.append(label);installationInputs[key]=input;
  }
  installation.append(installationFields);
  const applyInstallation=element('button','应用坐标并读回'),saveInstallation=element('button','保存本机坐标');applyInstallation.type=saveInstallation.type='button';applyInstallation.id='monitor-installation-apply';saveInstallation.id='monitor-installation-save';
  const coordinateActions=element('div');coordinateActions.className='map-buttons map-action-grid';coordinateActions.append(applyInstallation,saveInstallation);installation.append(coordinateActions);
  const installationState=element('p');installationState.className='map-note';installationState.id='monitor-installation-state';installation.append(installationState);$('radar-monitor-panel').append(installation);
  function rememberCoordinates(){for(const[key,input]of Object.entries(installationInputs)){if(String(coordinates[activeStart][key])!==input.value)dirty[activeStart]=true;coordinates[activeStart][key]=input.value;}}
  function readCoordinates(start){if(!validCoordinates(coordinates[start]))throw new Error('雷达坐标请填写0～2400 mm的有效数值');return Object.fromEntries(Object.entries(coordinates[start]).map(([key,value])=>[key,Number(value)]));}
  function syncCoordinates(){
    rememberCoordinates();activeStart=$('monitor-capture').value==='upper'?'upper':'lower';
    if(paramsCurrent())for(const start of ['lower','upper']){
      const prefix=start==='upper'?'upper_':'',device={lidar_x_mm:model.params[prefix+'lidar_x_mm'],lidar_y_mm:model.params[prefix+'lidar_y_mm']};
      if(!dirty[start]&&validCoordinates(device))coordinates[start]=device;
    }
    for(const[key,input]of Object.entries(installationInputs)){const value=String(coordinates[activeStart][key]);if(input.value!==value)input.value=value;}
  }
  applyInstallation.onclick=()=>work(async()=>{
    rememberCoordinates();const start=activeStart,values=readCoordinates(start);
    await workspace.applyParameters(start==='upper'?{upper_lidar_x_mm:values.lidar_x_mm,upper_lidar_y_mm:values.lidar_y_mm}:{...values,zero_deg:180});
    dirty[start]=false;$('monitor-radar-message').textContent=`${start==='upper'?'右上':'右下'}雷达坐标已读回 · 请重新采集地图`;
  });
  saveInstallation.onclick=()=>{try{rememberCoordinates();globalThis.localStorage?.setItem('local-car-radar-start-coordinates-v3',JSON.stringify({lower:readCoordinates('lower'),upper:readCoordinates('upper')}));$('monitor-radar-message').textContent='两个起点的雷达坐标已保存在本机';}catch(error){$('monitor-radar-message').textContent=error.message;}};
  $('monitor-capture').addEventListener?.('change',()=>render());
  async function work(task){if(busy)return;busy=true;$('monitor-radar-message').textContent='';render();try{await task();}catch(error){$('monitor-radar-message').textContent=error.message;}finally{busy=false;render();}}
  $('monitor-scan').onclick=()=>void work(async()=>{
    const corner=$('monitor-capture').value;
    if(corner!=='upper'&&(!paramsCurrent()||model.params.zero_deg!==180))await workspace.applyParameters({zero_deg:180});
    await workspace.requestWire(`radar scan ${corner==='upper'?'upper':'lower'} 15`);
  });
  $('monitor-stop').onclick=()=>workspace.requestWire('radar stop');
  $('monitor-continuous').onclick=()=>void work(async()=>{
    const corner=$('monitor-capture').value==='upper'?'upper':'lower';
    if(corner==='lower'&&(!paramsCurrent()||model.params.zero_deg!==180))await workspace.applyParameters({zero_deg:180});
    await workspace.startContinuous?.(corner);
  });
  $('monitor-cancel-transfer').onclick=()=>workspace.cancelRead?.();
  $('monitor-fetch-stable').onclick=()=>void work(()=>workspace.readStable?.());
  $('monitor-refresh').onclick=()=>void work(()=>workspace.readResult());
  $('monitor-clear-history').onclick=()=>{
    try{
      if(workspace.clearResults?.()!==true)return;
      $('monitor-radar-message').textContent='历史点云、扫描障碍和雷达路线已清除';
    }catch(error){$('monitor-radar-message').textContent=`本页已清除，本机缓存保存失败：${error.message}`;}
    if(!store.pair){preview=playing=null;last=0;routeLayer.setAttribute('d','');carLayer.replaceChildren();phases.replaceChildren();resultText.textContent='先完成静止扫描，再检查绕障路线。';}
    render();
  };
  $('monitor-params-read').onclick=()=>void work(()=>workspace.readParameters());
  check.onclick=()=>void work(async()=>{
    const pair=store.pair;if(!pair)throw new Error('请先扫描并读取完整地图与路线');
    preview=await monitorPreview(pair,api,$('chassis-route-start-speed')?.value??200);
    preview.capture={...pair.map};playing=null;routeLayer.setAttribute('d','');carLayer.replaceChildren();
    resultText.textContent=`地图 ${pair.map.map} · 正式检查 300×300 mm：${preview.message}`;
    phases.replaceChildren(...(preview.profile?.phases??[]).map(p=>element('li',profilePhaseText(p))));
  });
  play.onclick=()=>{
    if(!preview?.profile)return;
    try{
      const bodySize=previewBodySize(lengthInput.value,widthInput.value);
      playing=new RouteProfileRunner(preview.profile,{mask:preview.result.mask??preview.capture.mask,bodySize,gridPolicy:'warn'});last=0;
      resultText.textContent=`正在预演 · 地图 ${preview.capture.map} · 模型 ${previewBodySizeText(bodySize)} · 不发送运动指令`;
    }catch(error){resultText.textContent=error.message;}
  };
  pause.onclick=()=>{playing=null;resultText.textContent='预演已暂停 · 仅网页动画，车辆状态未改变';};
  execute.onclick=()=>void work(async()=>{
    if(workspace.continuous?.())await workspace.stopContinuousAndRead();
    const pair=store.pair,gate=monitorExecutionGate(model,pair,store.current(model));
    if(gate)throw new Error(gate);
    const speed=Number($('radar-nav-speed')?.value??200);
    if(!Number.isInteger(speed)||speed<10||speed>5000)throw new Error('启动速度范围10～5000 mm/s');
    executing=true;executionRequestAt=Date.now();render();
    const accepted=await workspace.requestWire(`radar nav start ${speed}`);
    if(accepted!==true){executing=false;throw new Error('雷达规划路线未发送，请查看串口状态');}
    resultText.textContent=`已发送雷达规划路线 · ${speed} mm/s · 车辆开始按扫描结果行驶`;
  });
  cancel.onclick=()=>void work(async()=>{
    const accepted=await workspace.requestWire('radar nav cancel');
    if(accepted!==true)throw new Error('取消路线未发送，请查看串口状态');
    executing=false;executionRequestAt=0;resultText.textContent='已请求取消雷达规划路线 · 等待设备停车确认';
  });
  function render(){
    const visible=$('map-page').dataset.mapTab==='monitor';
    routeLayer.style.display=carLayer.style.display=visible?'':'none';
    installationLayer.style.display=visible?'':'none';
    syncCoordinates();direction.textContent=activeStart==='upper'?'右上起点 · 雷达0°固定向下':'右下起点 · 雷达0°固定向左';
    const pair=store.pair,map=pair?.map,path=pair?.path,state=model.status,current=store.current(model),executionGate=monitorExecutionGate(model,pair,current);
    if(executing&&model.nav?.receivedAt>=executionRequestAt&&['done','cancelled','error','idle'].includes(model.nav.state)){executing=false;executionRequestAt=0;}
    $('monitor-map-source').textContent=map?`${current?'当前设备':'历史保留'} · 地图 ${map.map} / 路线 ${path.plan} · ${map.g===4?'两轴 [150,2250) 半开判障':map.g===3?'150～2250 mm判障':map.g===2?'全场旧网格':'旧网格'}`:'尚无完整雷达结果';
    $('monitor-map-source').dataset.current=String(current);
    $('monitor-grid-count').textContent=String(map?Array.from({length:25},(_,i)=>Number(!!((map.mask&~RADAR_FIXED_MASK)&(1<<i)))).reduce((a,b)=>a+b,0):0);
    $('monitor-route-count').textContent=String(path?.points?.length??0);
    $('monitor-path-state').textContent=path?(path.valid?'可达':`第 ${path.failed_leg} 段不可达`):'等待地图';
    $('monitor-scan-state').textContent=state?radarStateText(state.state)+` · 有效完整圈 ${state.window?.[2]??state.c?.[0]??0}/15${workspace.continuous?.()?' · 持续最近15圈':''}`:'待扫描';
    const p=model.params,defaults={angle_min_tenths:0,angle_max_tenths:3600,distance_min_mm:100,distance_max_mm:40000,energy_min:0,energy_max:255,threshold:1};
    const custom=Object.keys(defaults).filter(k=>p[k]!==undefined&&p[k]!==defaults[k]);
    const frozen=map?.pose?`${map.algorithm===1?`15圈稳定窗口 ${map.window?.[0]??'—'}～${map.window?.[1]??'—'} · ${map.stable?.[0]??'—'}稳定点`:map.c===3?'历史3圈合并 · 任一圈命中保留':'本圈冻结'} (${map.pose.join(', ')})${map.f?` · 距离 ${map.f[0]}～${map.f[1]} mm · 原始角 ${map.f[2]/10}～${map.f[3]/10}° · 阈值 ${map.f[4]} · 能量 ${map.f[5]}～${map.f[6]}`:''}；` : '';
    const prefix=activeStart==='upper'?'upper_':'',deviceX=p[prefix+'lidar_x_mm'],deviceY=p[prefix+'lidar_y_mm'];
    installationState.textContent=paramsCurrent()&&validCoordinate(deviceX)&&validCoordinate(deviceY)?`设备坐标 (${deviceX}, ${deviceY}) mm${Number(installationInputs.lidar_x_mm.value)!==deviceX||Number(installationInputs.lidar_y_mm.value)!==deviceY?' · 修改待应用':''}`:'本机坐标 · 连接后可应用并读回';
    const selectedPose=[Number(installationInputs.lidar_x_mm.value),Number(installationInputs.lidar_y_mm.value),activeStart==='upper'?270:180];
    const frozenPose=selectedPose,zero=selectedPose[2],radians=zero*Math.PI/180;
    installationLayer.replaceChildren(svg('circle',{cx:frozenPose[0],cy:frozenPose[1],r:22,fill:'#e89025'}),svg('path',{d:`M${frozenPose[0]} ${frozenPose[1]} l${250*Math.cos(radians)} ${250*Math.sin(radians)}`,stroke:'#e89025','stroke-width':12}),svg('text',{transform:`translate(${frozenPose[0]+290*Math.cos(radians)} ${frozenPose[1]+290*Math.sin(radians)}) scale(1 -1)`,'font-size':45,fill:'#a96208'}));installationLayer.children[2].textContent=`${activeStart==='upper'?'右上':'右下'}雷达0° · 设置`;
    $('monitor-params').textContent=frozen+(Object.keys(p).length?`${paramsCurrent()?'当前RAM':'历史参数 · 请读回'} · 阈值 ${p.threshold}${custom.length?' · 自定义筛选（保留）':' · 默认筛选'}`:'设备参数未读回');
    const scanning=workspace.busy();$('monitor-scan').disabled=busy||scanning||!workspace.canScan();
    $('monitor-clear-history').disabled=busy||scanning||!workspace.canClearResults?.();
    $('monitor-clear-history').title=busy||scanning?'采集或读取中，完成后可清除':'清除本机保存的点云、扫描障碍和雷达路线，保留雷达坐标设置';
    $('monitor-continuous').disabled=busy||scanning||!model.connected;$('monitor-stop').disabled=!model.connected;
    $('monitor-fetch-stable').disabled=busy||!pair||!model.connected;$('monitor-cancel-transfer').disabled=!model.connected;
    applyInstallation.disabled=busy||!model.connected||scanning&&!workspace.continuous?.();
    $('monitor-refresh').disabled=busy||scanning||!model.connected;
    $('monitor-params-read').disabled=busy||scanning||!model.connected;
    check.disabled=busy||!pair;play.disabled=busy||!preview?.profile;pause.disabled=!playing;
    execute.disabled=busy||executing||!!executionGate&&!workspace.continuous?.();cancel.disabled=busy||!model.connected;
    executeNote.textContent=executionGate||'只执行当前设备最新的右下起点规划路线；网页预演不会驱动车辆。';
    if(workspace.message())$('monitor-transfer').textContent=workspace.message();
    const changed=preview&&pair&&!same(preview.capture,pair.map);prep.dataset.historical=String(!!changed);
    if(changed&&!playing)resultText.textContent=`预演基于地图 ${preview.capture.map}；已收到地图 ${pair.map.map}，可重新检查绕障路线。`;
  }
  function frame(now){
    routeLayer.style.display=carLayer.style.display=$('map-page').dataset.mapTab==='monitor'?'':'none';
    if(playing){
      if($('map-page').dataset.mapTab!=='monitor'){playing=null;last=0;resultText.textContent='已离开实车监控，网页预演暂停';}
      else{
        if(last)playing.advance(Math.min((now-last)/1000,.1)*2);last=now;
        routeLayer.setAttribute('d',playing.path.map((p,i)=>`${i?'L':'M'}${p.x_mm},${p.y_mm}`).join(' '));
        const p=playing.pose,size=playing.bodySize;carLayer.replaceChildren(svg('rect',{x:-size.length_mm/2,y:-size.width_mm/2,width:size.length_mm,height:size.width_mm,fill:'#18b9d11a',stroke:'#18b9d1','stroke-width':8,transform:`translate(${p.x_mm} ${p.y_mm}) rotate(${p.yaw_rad*180/Math.PI})`}),svg('circle',{cx:p.x_mm,cy:p.y_mm,r:25,fill:'#18b9d1'}));
        const heading=svg('path',{d:`M 0 0 L ${size.length_mm*.4} 0 M ${size.length_mm*.4-28} -22 L ${size.length_mm*.4} 0 L ${size.length_mm*.4-28} 22`,fill:'none',stroke:'#0086a0','stroke-width':12,transform:`translate(${p.x_mm} ${p.y_mm}) rotate(${p.yaw_rad*180/Math.PI})`});
        carLayer.append(heading);appendArmMarker(carLayer,p,size,playing.armPose);
        if(playing.warnings.length)resultText.textContent=profileResultText(playing);
        if(playing.state!=='running'){resultText.textContent=`${profileResultText(playing)} · 预演模型 ${previewBodySizeText(playing.bodySize)}`;playing=null;last=0;render();}
      }
    }
    requestAnimationFrame(frame);
  }
  render();requestAnimationFrame(frame);setInterval(render,500);return {render};
}
