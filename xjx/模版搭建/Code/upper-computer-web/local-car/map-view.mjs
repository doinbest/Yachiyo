import {deviceText,reasonText} from './device-text.mjs';
import {RouteRunner,defaultSegment,defaultRoute,bodyCorners,turntableGap,routeResultText} from './route-model.mjs?v=two-batch-1';
import {DEFAULT_GEOMETRY,SIM_MAX_RPM,FIELD,inverse,forward,quantize,rotate,midpoint,wheelCenters,FixedStepper,Telemetry,motionDisplay,routePhaseText} from './map-model.mjs?v=chinese-notices-20261009';
import {RouteProfileRunner,profilePhaseText,profileResultText,PROFILE_MAX_RPM} from './route-profile.mjs';
import {monitorGrid} from './monitor-radar.mjs';
import {RouteStopHistory} from './route-monitor.mjs';
import {appendArmMarker} from './vehicle-marker.mjs';

const $=id=>document.getElementById(id);
const fmt=(v,n=1)=>Number.isFinite(v)?v.toFixed(n):'—';
const poseText=p=>p?`X ${fmt(p.x_mm)} · Y ${fmt(p.y_mm)} mm · θ ${fmt(p.yaw_rad*180/Math.PI)}°`:'不可用';
const names=['左前 1','左后 2','右后 3','右前 4'];
const statusName=v=>({estimated:'估计',assumed:'假设',measured:'实测',confirmed:'已确认'}[v]??'未标注');
const ns='http://www.w3.org/2000/svg';
function svg(tag,attrs={},text){const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;return e;}
function fieldDrawing(){
  const root=$('field-static');
  root.append(svg('rect',{x:0,y:0,width:2400,height:2400,fill:'#c5c9ce',stroke:'#607486','stroke-width':8}));
  for(const b of FIELD.blocks)root.append(svg('rect',{x:b.x,y:b.y,width:450,height:450,fill:'#f8edb7',stroke:'#d0b25c','stroke-width':3}));
  const gridLines=Array.from({length:6},()=>svg('path',{stroke:'#607486','stroke-width':5})),gridLabels=[];
  root.append(...gridLines);
  for(let r=0;r<5;r++)for(let c=0;c<5;c++){const label=svg('text',{class:'monitor-grid-label','text-anchor':'middle',fill:'#486070','font-size':36,'pointer-events':'none'},`R${r+1}C${c+1}`);gridLabels.push(label);root.append(label);}
  const setRadarGrid=edges=>{
    for(let i=0;i<6;i++){const mm=edges[i];gridLines[i].setAttribute('d',`M ${mm} ${edges[0]} V ${edges[5]} M ${edges[0]} ${mm} H ${edges[5]}`);gridLines[i].setAttribute('data-radar-grid',mm);}
    for(let r=0;r<5;r++)for(let c=0;c<5;c++)gridLabels[r*5+c].setAttribute('transform',`translate(${(edges[c]+edges[c+1])/2} ${edges[r+1]-130}) scale(1 -1)`);
    $('radar-grid-legend')?.setAttribute('title',`本图判障边界：${edges.join('、')} mm`);
  };
  setRadarGrid(monitorGrid());
  for(const b of [{x:0,y:910,w:150,h:580},{x:910,y:0,w:580,h:150}])root.append(svg('rect',{x:b.x,y:b.y,width:b.w,height:b.h,fill:'#e9cc83',stroke:'#a08046','stroke-width':3}));
  for(const y of [0,2100])root.append(svg('rect',{x:2100,y,width:300,height:300,fill:'#92bfdf',stroke:'#467ba6','stroke-width':3}));
  root.append(svg('circle',{cx:1200,cy:2470,r:150,fill:'#fff3d2',stroke:'#996c3e','stroke-width':5}),svg('path',{d:'M 2400 1100 V 1300',stroke:'#193b59','stroke-width':15}));
  // Variable placement is an illustration, not a measured position.
  for(const [x,y]of [[75,1050],[75,1200],[75,1350],[1050,75],[1200,75],[1350,75]])root.append(svg('circle',{cx:x,cy:y,r:42,fill:'none',stroke:'#76584e','stroke-width':4,'stroke-dasharray':'12 10'}));
  const labels=$('field-labels');
  for(const [x,y,text]of [[0,0,'O (0,0)'],[2450,0,'X →'],[0,2500,'Y ↑'],[1200,2690,'原料区 · 中心 (1200,2470) · Ø300'],[2680,1200,'二维码'],[2400,1020,'(2400,1200)'],[2240,2250,'300'],[2240,150,'返回启停区'],[1200,-140,'粗加工区'],[-160,1360,'暂存区'],[775,775,'450 × 450'],[1625,1625,'450 × 450']])labels.append(svg('text',{x,y:2400-y,'text-anchor':x===0?'start':'middle','font-size':x===2680?44:42,fill:'#294457'},text));
  for(let i=600;i<=2400;i+=600){labels.append(svg('text',{x:i,y:2490,'text-anchor':'middle','font-size':38,fill:'#496277'},String(i)));labels.append(svg('text',{x:-50,y:2410-i,'text-anchor':'end','font-size':38,fill:'#496277'},String(i)));}
  return setRadarGrid;
}
function observeFieldLabels(){
  if(typeof ResizeObserver!=='function')return;
  const map=$('field-map'),labels=[...$('field-labels').children].map(node=>({node,x:Number(node.getAttribute('x')),y:Number(node.getAttribute('y')),size:Number(node.getAttribute('font-size'))}));
  const fit=()=>{
    const rect=map.getBoundingClientRect(),matrix=map.getScreenCTM();
    if(!rect.width||!rect.height||!matrix)return;
    const scale=Math.hypot(matrix.a,matrix.b);
    if(!Number.isFinite(scale)||scale<=0)return;
    map.style.setProperty('--map-label-size',`${Math.max(42,11/scale)}px`);
    map.style.setProperty('--map-count-size',`${Math.max(48,10/scale)}px`);
    const view=map.viewBox.baseVal,padding=2/scale;
    for(const {node,x,y,size} of labels){
      // Keep names and ticks readable without changing map coordinates or scale.
      node.setAttribute('font-size',Math.max(size,(size===38?10:11)/scale));
      node.setAttribute('x',x);node.setAttribute('y',y);
      const box=node.getBBox();
      const dx=Math.max(0,view.x+padding-box.x)-Math.max(0,box.x+box.width-(view.x+view.width-padding));
      const dy=Math.max(0,view.y+padding-box.y)-Math.max(0,box.y+box.height-(view.y+view.height-padding));
      if(dx)node.setAttribute('x',x+dx);
      if(dy)node.setAttribute('y',y+dy);
    }
  };
  new ResizeObserver(fit).observe(map);fit();
}
export function mountMap({send,onStatus=()=>{},isPreparing=()=>false}){
  const stopHistory=new RouteStopHistory();
  let profileOptions={},previewDirty=false;
  const telemetry=new Telemetry();let connected=false,running=false,mode='feedback',runner=null,activeRows=[],wholeRoute=true,previewSource=null;
  let streamBusy=false;
  let pose={x_mm:2250,y_mm:150,yaw_rad:Math.PI/2},path=[pose];let body=[0,0,0],geometry={...DEFAULT_GEOMETRY};
  const layerPaths={command:'command-path',feedback:'feedback-path',device:'radar-device-path',offline:'radar-offline-path',sim:'sim-path'};
  const layerVisible=Object.fromEntries(Object.keys(layerPaths).map(key=>[key,true]));
  const legendButtons=[...$('map-legend').querySelectorAll('[data-map-layer]')];
  const stepper=new FixedStepper();let last=performance.now(),lastRender=0;
  const setRadarGrid=fieldDrawing();observeFieldLabels();
  const offlineFields=[['start-x','初始 X / mm',2250,-5000,5000,1],['start-y','初始 Y / mm',150,-5000,5000,1],['start-theta','初始 θ / °',90,-3600,3600,1],['geom-l','轴距 L / mm · 估计',210,1,2000,1],['geom-w','轮距 W / mm · 估计',240,1,2000,1],['geom-d','轮径 D / mm · 假设',80,1,1000,.1],['geom-g','减速比 G · 假设',1,.01,100,.01]];
  for(const [id,text,value,min,max,step]of offlineFields){const label=document.createElement('label');label.textContent=text;const input=document.createElement('input');Object.assign(input,{id,type:'number',value,min,max,step,required:true});label.append(input);$('sim-fields').append(label);}
  function readInputs(){
    if(!$('sim-form').reportValidity())return false;

    geometry=Object.fromEntries(Object.keys(DEFAULT_GEOMETRY).map((k,i)=>[k,Number($(['geom-l','geom-w','geom-d','geom-g'][i]).value)]));return true;
  }
  const rowKeys=['name','vx','vy','omega','accel','hold','decel','dwell'];
  const phaseNames={accel:'起步',hold:'匀速运行',decel:'减速停车',dwell:'停车停留',done:'完成'};
  const readRows=()=>[...$('route-rows').children].map(tr=>Object.fromEntries(rowKeys.map(k=>{const input=tr.querySelector(`[data-key="${k}"]`);return [k,k==='name'?input.value:Number(input.value)];})));
  function invalidate(message){running=false;runner=null;body=[0,0,0];path=[{...pose}];stepper.reset();$('sim-status').textContent=message;render();}
  function drawRows(rows){
    $('route-rows').replaceChildren(...rows.map((row,i)=>{
      const tr=document.createElement('tr');const no=document.createElement('th');no.textContent=i+1;tr.append(no);
      for(const key of rowKeys){
        const td=document.createElement('td'),input=document.createElement('input');input.dataset.key=key;input.setAttribute('aria-label',`第 ${i+1} 段 ${key}`);input.required=true;
        input.type=key==='name'?'text':'number';input.value=row[key];
        if(key==='name')input.maxLength=40;
        else {input.step=key==='omega'?'.01':'.001';input.min=['vx','vy'].includes(key)?'-100':key==='omega'?'-.15':['accel','decel'].includes(key)?'.001':'0';input.max=['vx','vy'].includes(key)?'100':key==='omega'?'.15':'60';}
        td.append(input);tr.append(td);
      }
      const td=document.createElement('td');
      for(const [action,label]of [['run','运行此段'],['up','↑'],['down','↓'],['delete','删除']]){const b=document.createElement('button');b.type='button';b.dataset.action=action;b.textContent=label;b.setAttribute('aria-label',`第 ${i+1} 段 ${label}`);td.append(b);}
      tr.append(td);return tr;
    }));
  }
  function showSimulation(){mode='offline';}
  function reset(){if(!readInputs())return;previewSource=null;showSimulation();pose={x_mm:Number($('start-x').value),y_mm:Number($('start-y').value),yaw_rad:Number($('start-theta').value)*Math.PI/180};invalidate('历史时间示意已复位 · 从初始位姿开始');}
  function startRoute(rowIndex=null){
    if(!readInputs())return;
    const relevant=rowIndex===null?[...$('route-rows').querySelectorAll('input')]:[...$('route-rows').children[rowIndex].querySelectorAll('input')];
    if(!relevant.every(input=>input.reportValidity()))return;
    try {
      const rows=readRows(),initial=rowIndex===null?{x_mm:Number($('start-x').value),y_mm:Number($('start-y').value),yaw_rad:Number($('start-theta').value)*Math.PI/180}:pose;
      const next=new RouteRunner(rowIndex===null?rows:[rows[rowIndex]],initial,{geometry,integer:$('sim-model').value==='integer'});
      previewSource=null;showSimulation();
      runner=next;wholeRoute=rowIndex===null;activeRows=rowIndex===null?rows.map((_,i)=>i):[rowIndex];pose=runner.pose;path=runner.path;body=[0,0,0];running=true;stepper.reset();last=performance.now();$('sim-status').textContent='路线模拟运行中';render();
    }catch(error){$('sim-status').textContent=error.message;}
  }
  drawRows(defaultRoute());
  $('route-rows').addEventListener('input',()=>invalidate('路线已修改 · 请重新运行'));
  $('route-rows').addEventListener('click',e=>{
    const b=e.target.closest('button');if(!b)return;const i=[...$('route-rows').children].indexOf(b.closest('tr')),action=b.dataset.action;
    if(action==='run'){startRoute(i);return;}
    const rows=readRows();if(action==='delete'){if(rows.length===1)return;rows.splice(i,1);}
    if(action==='up'&&i>0)[rows[i-1],rows[i]]=[rows[i],rows[i-1]];
    if(action==='down'&&i<rows.length-1)[rows[i+1],rows[i]]=[rows[i],rows[i+1]];
    drawRows(rows);invalidate('路线顺序已修改 · 请重新运行');
  });
  $('route-add').addEventListener('click',()=>{const rows=readRows();if(rows.length>=50)return;rows.push(defaultSegment());drawRows(rows);invalidate('已添加一段 · 请设置参数');});
  $('sim-form').addEventListener('submit',e=>{e.preventDefault();startRoute();});
  $('sim-resume').addEventListener('click',()=>{if(runner?.state!=='running')return;showSimulation();running=true;stepper.reset();last=performance.now();$('sim-status').textContent='路线模拟运行中';render();});
  function pauseSimulation(message){
    if(!running)return;
    running=false;stepper.reset();$('sim-status').textContent=message;render();
  }
  $('sim-pause').addEventListener('click',()=>pauseSimulation('模拟已暂停 · 可继续'));
  $('profile-pause').addEventListener('click',()=>pauseSimulation('方案播放已暂停 · 可继续'));
  $('profile-resume').addEventListener('click',()=>{if(!(runner instanceof RouteProfileRunner)||runner.state!=='running'||mode!=='offline'||previewDirty)return;running=true;stepper.reset();last=performance.now();render();});
  $('profile-reset').addEventListener('click',()=>{if(previewSource)startProfile(previewSource,profileOptions);});
  $('sim-reset').addEventListener('click',reset);
  $('sim-fields').addEventListener('input',()=>invalidate('参数已编辑 · 整条路线将使用新初始位姿；单段使用当前位姿'));
  $('sim-model').addEventListener('change',()=>invalidate('模型已切换 · 请重新运行路线'));
  $('map-legend').addEventListener('click',e=>{
    const button=e.target.closest('[data-map-layer]'),key=button?.dataset.mapLayer;
    if(!Object.hasOwn(layerPaths,key))return;
    layerVisible[key]=!layerVisible[key];render();
  });
  function setSource(value){
    const next=value==='offline'?'offline':'feedback';
    if(mode===next)return;
    pauseSimulation('已切换地图场景 · 模拟暂停，可继续');mode=next;render();
  }
  function startProfile(profile,options={}){
    if(!readInputs())return;
    try{
      profileOptions={...options};previewSource=profile;
      runner=new RouteProfileRunner(profile,{...options,gridPolicy:'warn',geometry,integer:$('sim-model').value==='integer'});
      showSimulation();wholeRoute=true;activeRows=[];pose=runner.pose;path=runner.path;body=[0,0,0];running=runner.state==='running';stepper.reset();last=performance.now();$('sim-status').textContent=profileResultText(runner);render();
    }catch(error){$('sim-status').textContent=error.message;}
  }
  function setPreviewSource(profile,options={}){if((profile!==previewSource||options.dirty)&&runner instanceof RouteProfileRunner){pauseSimulation('方案或输入已修改 · 请重新计算后播放');if(profile!==previewSource){runner=null;body=[0,0,0];path=[];}}if(profile!==previewSource&&profile){pose={x_mm:profile.points[0][0],y_mm:profile.points[0][1],yaw_rad:Math.PI};path=[{...pose}];}previewSource=profile;profileOptions={mask:options.mask??0};previewDirty=!!options.dirty;render();}
  function renderLayers(){
    const offline=mode==='offline';
    for(const [key,id]of Object.entries(layerPaths)){
      const available=key==='device'||(['offline','sim'].includes(key)?offline:!offline);
      $(id).style.display=available&&layerVisible[key]?'':'none';
    }
    $('route-annotations').style.display=(offline?layerVisible.offline:layerVisible.device)?'':'none';
    $('radar-stations').style.display=$('radar-device-path').style.display;
    $('route-stops').style.display=$('sim-path').style.display;
    for(const button of legendButtons){
      button.hidden=button.dataset.mapContext==='online'?offline:button.dataset.mapContext==='offline'?!offline:false;
      button.setAttribute('aria-pressed',String(layerVisible[button.dataset.mapLayer]));
    }
  }
  $('monitor-toggle').addEventListener('click',async()=>{
    if(!connected||isPreparing()||streamBusy)return;
    const updating=telemetry.session!==null;streamBusy=true;
    try{
      if(updating){
        render();
        if(await send('chassis stream off'))telemetry.stop('更新已暂停');
        else telemetry.status='暂停更新未发送 · 请重试';
      }else{
        let nonce=0;while(!nonce)nonce=crypto.getRandomValues(new Uint32Array(1))[0];
        telemetry.begin(nonce,performance.now());render();
        if(!await send(`chassis stream on ${nonce}`))telemetry.stop('开启未发送 · 请重试');
      }
    }finally{streamBusy=false;render();}
  });
  function drawPath(id,points){let d='',pen=false;for(const p of points){if(!p){pen=false;continue;}d+=`${pen?'L':'M'}${p.x_mm.toFixed(2)},${p.y_mm.toFixed(2)} `;pen=true;}$(id).setAttribute('d',d);}
  function drawCar(p,g,offline){const car=$('map-car');car.replaceChildren();if(!p||!g)return;if(offline)car.append(svg('polygon',{points:bodyCorners(p).map(v=>v.join(',')).join(' '),fill:'#1286b512',stroke:'#0a6b90','stroke-width':5}));const corners=wheelCenters(g).map(v=>{const [x,y]=rotate(v,p.yaw_rad);return [p.x_mm+x,p.y_mm+y];});car.append(svg('polygon',{points:corners.map(v=>v.join(',')).join(' '),fill:'#1286b52e',stroke:'#08658b','stroke-width':7,'stroke-dasharray':'16 10'}));for(let i=0;i<4;i++){const [x,y]=corners[i];car.append(svg('circle',{cx:x,cy:y,r:18,fill:i===0?'#08658b':'#fff',stroke:'#08658b','stroke-width':6}));}
    const [dx,dy]=rotate([g.wheelbase_mm*.7,0],p.yaw_rad);car.append(svg('path',{d:`M ${p.x_mm} ${p.y_mm} l ${dx} ${dy}`,stroke:'#064e70','stroke-width':12,'marker-end':'url(#heading-arrow)'}));appendArmMarker(car,p,{length_mm:300,width_mm:300},offline?runner?.armPose:'rear');}
  function render(){
    const offline=mode==='offline',updating=telemetry.session!==null;
    $('monitor-toggle').disabled=!connected||isPreparing()||streamBusy;
    $('monitor-toggle').textContent=updating?'暂停更新':'开始监控';
    $('monitor-toggle').setAttribute('aria-pressed',String(updating));
    $('telemetry-status').textContent=!connected?'未连接设备':telemetry.status.includes('未发送')||telemetry.status==='配置不兼容'?telemetry.status:updating?(telemetry.stale?'状态更新中断 · 等待恢复':telemetry.state?'监控中':'等待设备状态'):telemetry.status==='更新已暂停'?'更新已暂停':'已连接 · 点击开始监控';
    $('monitor-live-data').hidden=!connected||!telemetry.state;
    $('map-route-status').hidden=!telemetry.state;
    onStatus(telemetry.status);
    const g=offline?geometry:telemetry.config?.geometry;
    const displayPose=offline?pose:(telemetry.feedback??(telemetry.stale?telemetry.lastFeedback:null));
    $('map-pose').textContent=poseText(displayPose)+(!offline&&telemetry.stale?' · 最后位置（已过期）':'');$('map-car').style.opacity=!offline&&telemetry.stale?'0.35':'1';
    const currentProfile=runner instanceof RouteProfileRunner;
    $('profile-pause').disabled=!offline||!running||!currentProfile;
    $('profile-resume').disabled=!offline||running||!currentProfile||runner.state!=='running'||previewDirty;
    $('profile-reset').disabled=!offline||!previewSource||previewDirty;
    $('profile-play-status').textContent=currentProfile?(running?profileResultText(runner):runner.state==='running'?'方案播放已暂停':profileResultText(runner)):'点击计算，再播放当前方案';
    $('map-source-label').textContent=offline?(currentProfile?'当前固件 C 阶段 · 理想反馈模型':!runner&&previewSource?'当前固件 C 阶段 · 未播放':'历史16段时间示意 · 手工编辑'):'车辆位置 · 反馈估计';
    $('sim-start').disabled=running;
    $('sim-resume').disabled=running||runner?.state!=='running';
    $('sim-pause').disabled=!running;
    [...$('route-rows').children].forEach((tr,i)=>{tr.classList.toggle('route-active',!!runner&&activeRows[runner.index]===i);tr.querySelector('[data-action="run"]').disabled=running;});
    $('route-progress').textContent=runner?(currentProfile?`${profilePhaseText(runner.rows[runner.index])} · 理想角度/距离完成 · 累计 ${fmt(runner.elapsed,2)} s / ${fmt(runner.distance)} mm`:`第 ${activeRows[runner.index]+1} 段 · ${runner.rows[runner.index].name} · ${phaseNames[runner.phase]} · 累计 ${fmt(runner.elapsed,2)} s / ${fmt(runner.distance)} mm`):'';
    $('route-live').textContent=$('sim-status').textContent+(runner?' · '+$('route-progress').textContent:'');
    $('route-results').replaceChildren(...(runner?.stops??[]).map(stop=>{const li=document.createElement('li');li.textContent=`${currentProfile?stop.index+1:activeRows[stop.index]+1}. ${stop.name} · ${fmt(stop.time,2)} s · ${poseText(stop.pose)} · 距转盘 ${fmt(turntableGap(stop.pose))} mm`;return li;}));
    $('profile-stop-results').replaceChildren(...(currentProfile?runner.stops:[]).map(stop=>{const li=document.createElement('li');li.textContent=`目标 ${runner.rows[stop.index].segment} · ${poseText(stop.pose)} · ${fmt(stop.time,2)} s`;return li;}));
    const stopMarks=$('route-stops');stopMarks.replaceChildren();
    if(offline){
      // Group nearby labels only; each recorded footprint and exact stop pose stays unchanged.
      const labels=[];
      for(const stop of runner?.stops??[]){
        stopMarks.append(svg('polygon',{points:bodyCorners(stop.pose).map(v=>v.join(',')).join(' '),fill:'none',stroke:'#097da670','stroke-width':3}));
        const group=labels.find(g=>Math.hypot(g.pose.x_mm-stop.pose.x_mm,g.pose.y_mm-stop.pose.y_mm)<30);
        const number=currentProfile?runner.rows[stop.index].segment:activeRows[stop.index]+1;
        if(group){if(!group.numbers.includes(number))group.numbers.push(number);}
        else labels.push({pose:stop.pose,numbers:[number]});
      }
      if(!currentProfile)for(const {pose:p,numbers} of labels){
        stopMarks.append(svg('circle',{cx:p.x_mm,cy:p.y_mm,r:26,fill:'#fff',stroke:'#097da6','stroke-width':6}));
        stopMarks.append(svg('text',{transform:`translate(${p.x_mm+40} ${p.y_mm+35}) scale(1 -1)`,'font-size':42,fill:'#065775'},numbers.join('/')));
      }
    }
    drawPath('sim-path',path);drawPath('command-path',telemetry.paths.command);drawPath('feedback-path',telemetry.paths.feedback);drawCar(displayPose,g,offline);renderLayers();
    $('map-geometry').textContent=g?`L ${g.wheelbase_mm} · W ${g.track_mm} · D ${g.wheel_diameter_mm} mm · G ${g.gear_ratio}${offline?' · 轴距/轮距估计，轮径/减速比假设':` · ${telemetry.config.rev} · 固件配置 · 轴距:${statusName(telemetry.config.geometry_status?.wheelbase)} / 轮距:${statusName(telemetry.config.geometry_status?.track)} / 轮径:${statusName(telemetry.config.geometry_status?.diameter)} / 比:${statusName(telemetry.config.geometry_status?.gear_ratio)}`}`:'在线几何未收到 · 不使用离线参数';
    const motion=motionDisplay(telemetry.route,telemetry.distance,telemetry.feedback?telemetry.feedback.yaw_rad*180/Math.PI:null);
    $('real-route-status').textContent=motion.text;$('map-route-status').textContent=motion.text;
    const activeMotion=telemetry.route?.segment>0?telemetry.route:telemetry.distance??telemetry.route;
    $('monitor-stop-state').textContent=activeMotion?.stop_confirmed?'停车已确认':activeMotion?.state==='stopping'?'正在停车':activeMotion?.state==='running'?'运行中':activeMotion?`${deviceText(activeMotion.state)} · 停车未确认`:'停车待确认';
    $('monitor-current-target').textContent=activeMotion?.segment>0?`目标 ${activeMotion.segment}${activeMotion.total?` / ${activeMotion.total}`:''}${activeMotion.phase?` · ${routePhaseText(activeMotion.phase)}`:''}`:telemetry.distance?'单段移动':'—';
    const motionReason=activeMotion?.reason;
    $('monitor-live-reason').hidden=!motionReason||['none','complete','reached','auto_continue','auto_dwell','station_handoff'].includes(motionReason);
    $('monitor-live-reason').textContent=motionReason?reasonText(motionReason):'';
    $('monitor-service-stop').textContent=$('stop-status').textContent;
    const command=telemetry.state?.target.body;
    $('monitor-command-speed').textContent=command?`${fmt(Math.hypot(command[0],command[1]))} mm/s${Math.abs(command[2])>1e-7?` · ${fmt(command[2]*180/Math.PI)}°/s`:''}`:'—';
    $('monitor-command-speed').title=command?`vx ${fmt(command[0])} · vy ${fmt(command[1])} mm/s · ω ${fmt(command[2]*180/Math.PI)}°/s`:'';
    $('monitor-plan-speed').textContent=activeMotion?.phase?.includes('turn')?'转向阶段':Number.isFinite(activeMotion?.phase_speed_mm_s)?`${fmt(activeMotion.phase_speed_mm_s,0)} mm/s`:'—';
    $('monitor-plan-speed').title=activeMotion?`本次启动 ${fmt(activeMotion.speed_mm_s,0)} mm/s；显示当前阶段限速`:'';
    const latest=stopHistory.stops.at(-1);
    $('monitor-last-stop').hidden=$('monitor-stop-history').hidden=!latest;
    $('monitor-last-stop-title').textContent=latest?`最近停车 · ${latest.segment?`目标 ${latest.segment}`:'单段移动'}（记录）`:'最近停车';
    $('monitor-final-stop').textContent=latest?poseText(latest.pose):'';
    $('monitor-stop-delta').textContent=latest?`计划 (${latest.target.join(', ')}) · ΔX ${fmt(latest.pose.x_mm-latest.target[0])} / ΔY ${fmt(latest.pose.y_mm-latest.target[1])} mm`:'';
    $('monitor-history-count').textContent=latest?` · ${stopHistory.stops.length} 次停靠`:'';
    $('monitor-stop-results').replaceChildren(...stopHistory.stops.map((stop,i)=>{
      const li=document.createElement('li');
      li.textContent=`${i+1}. ${stop.segment?`目标 ${stop.segment}`:'单段位移'} · ${deviceText(stop.state)} · ${reasonText(stop.reason)} · 启动 ${fmt(stop.speed,0)} / 限速 ${fmt(stop.limit,0)} mm/s · 计划 (${stop.target.join(', ')}) · 停车 ${poseText(stop.pose)} · MCU ${stop.time} ms · 原点版本 ${stop.generation??'—'}`;
      return li;
    }));
    const routeTarget=$('real-route-target');routeTarget.replaceChildren();
    if(!offline&&motion.target){
      const [x,y,heading]=motion.target,angle=heading*Math.PI/180;
      routeTarget.append(svg('circle',{cx:x,cy:y,r:42,fill:'none',stroke:'#c74333','stroke-width':7}));
      routeTarget.append(svg('path',{d:`M ${x-60} ${y} H ${x+60} M ${x} ${y-60} V ${y+60} M ${x} ${y} l ${100*Math.cos(angle)} ${100*Math.sin(angle)}`,stroke:'#c74333','stroke-width':5}));
      routeTarget.append(svg('text',{transform:`translate(${x+65} ${y+65}) scale(1 -1)`,'font-size':42,fill:'#c74333'},motion.label));
    }
    const r=telemetry.state;
    $('online-details').textContent=r?`最近一帧 MCU ${r.t_ms} ms · 会话 ${r.session} · 周期 ${telemetry.periodMs/1000} s\n任务 ${r.task?.state==='done'&&r.task?.reason==='timed_complete'?'时间任务结束（未表示到达地图目标） · ':''}${deviceText(r.task?.state)} · ${reasonText(r.task?.reason)} · ID ${r.task?.id??'—'}\n发送 ${r.tx?.stage??'—'} · 错误 ${r.tx?.error??'—'} · 丢帧 ${r.dropped??'—'}\n航向 ${r.yaw?.valid?fmt(r.yaw.rad,3)+' rad':'不可用'}\n位置反馈单位/圈：${telemetry.config?.position_units_per_rev||'待用户确认（手册65536）'}。不等同坐标。\n反馈符号：${telemetry.config?.feedback_sign??'未标注'}；右侧电机原始正负不等于逻辑前后。`:`尚未收到设备遥测。开启后显示配置、状态与更新时间。`;
    const target=offline?inverse(body,g,currentProfile?PROFILE_MAX_RPM:SIM_MAX_RPM):r?.target.rpm;
    const approx=offline?quantize(target):null;
    $('wheel-body').replaceChildren(...names.map((name,i)=>{const tr=document.createElement('tr');const fresh=!!r?.feedback.speed_valid[i]&&((r.t_ms-r.feedback.speed_ms[i])>>>0)<=600;const posFresh=!!r?.feedback.position_valid[i]&&((r.t_ms-r.feedback.position_ms[i])>>>0)<=600;for(const value of [name,fmt(target?.[i],2),offline?fmt(approx[i],0):fresh?fmt(r.feedback.rpm[i],2):'不可用',offline?'—':posFresh?fmt(r.feedback.pos[i],0):'不可用']){const td=document.createElement('td');td.textContent=value;tr.append(td);}return tr;}));
    $('wheel-approx-title').textContent=offline?'整数 RPM':telemetry.config?.feedback_sign==='motor_raw'?'原始反馈 RPM':'反馈 RPM';
    $('map-body-speed').textContent=offline?`输入 vx ${fmt(body[0])} · vy ${fmt(body[1])} mm/s · ω ${fmt(body[2]*180/Math.PI,1)} °/s`:r?`目标 vx ${fmt(r.target.body[0])} · vy ${fmt(r.target.body[1])} mm/s · ω ${fmt(r.target.body[2]*180/Math.PI,1)} °/s`:'目标速度不可用';
  }
  function frame(now){
    const elapsed=now-last;last=now;telemetry.tick(now);
    if($('map-page').hidden)pauseSimulation('已离开地图页 · 模拟暂停，可继续');
    if(running&&runner){
      if(!stepper.advance(elapsed,dt=>{if(!running)return;runner.advance(dt*Number($('sim-rate').value));pose=runner.pose;path=runner.path;body=runner.body;if(runner.state!=='running'){running=false;$('sim-status').textContent=runner instanceof RouteProfileRunner?profileResultText(runner):routeResultText(runner,{wholeRoute,rowNumber:activeRows[runner.index]+1});}})){
        running=false;$('sim-status').textContent='页面停顿超过 250 ms · 模拟已暂停，可继续';
      }
    }
    if(now-lastRender>100){render();lastRender=now;}requestAnimationFrame(frame);
  }
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pauseSimulation('页面已隐藏 · 模拟暂停，可继续');});
  render();requestAnimationFrame(frame);
  return {setSource,setPreviewSource,startProfile,setRadarGrid,beginPreparedTelemetry(nonce){pauseSimulation('准备实车跑图 · 离线模拟暂停');mode='feedback';$('sim-controls').hidden=true;$('route-editor').hidden=true;$('simulation-actions').hidden=true;$('online-controls').hidden=false;telemetry.begin(nonce,performance.now());render();},snapshot(){telemetry.tick(performance.now());return telemetry;},receive(line){telemetry.accept(line,performance.now());stopHistory.observe(telemetry);},connection(value){connected=value;telemetry.stop(value?'已连接 · 遥测需手动开启':'串口未连接');render();}};
}
