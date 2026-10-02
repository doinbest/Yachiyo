import {RouteRunner,defaultSegment,defaultRoute,bodyCorners,turntableGap,routeResultText} from './route-model.mjs?v=two-batch-1';
import {DEFAULT_GEOMETRY,SIM_MAX_RPM,FIELD,inverse,forward,quantize,rotate,midpoint,wheelCenters,FixedStepper,Telemetry,motionDisplay} from './map-model.mjs?v=route-20260925';

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
  for(let i=0;i<=2400;i+=300){root.append(svg('path',{d:`M ${i} 0 V 2400 M 0 ${i} H 2400`,stroke:'#9ca9b5','stroke-width':2}));}
  for(const b of FIELD.blocks)root.append(svg('rect',{x:b.x,y:b.y,width:450,height:450,fill:'#f8edb7',stroke:'#d0b25c','stroke-width':3}));
  for(const b of [{x:0,y:910,w:150,h:580},{x:910,y:0,w:580,h:150}])root.append(svg('rect',{x:b.x,y:b.y,width:b.w,height:b.h,fill:'#e9cc83',stroke:'#a08046','stroke-width':3}));
  for(const y of [0,2100])root.append(svg('rect',{x:2100,y,width:300,height:300,fill:'#92bfdf',stroke:'#467ba6','stroke-width':3}));
  root.append(svg('circle',{cx:1200,cy:2470,r:150,fill:'#fff3d2',stroke:'#996c3e','stroke-width':5}),svg('path',{d:'M 2400 1100 V 1300',stroke:'#193b59','stroke-width':15}));
  // Variable placement is an illustration, not a measured position.
  for(const [x,y]of [[75,1050],[75,1200],[75,1350],[1050,75],[1200,75],[1350,75]])root.append(svg('circle',{cx:x,cy:y,r:42,fill:'none',stroke:'#76584e','stroke-width':4,'stroke-dasharray':'12 10'}));
  const labels=$('field-labels');
  for(const [x,y,text]of [[0,0,'O (0,0)'],[2450,0,'X →'],[0,2500,'Y ↑'],[1200,2690,'原料区 · 中心 (1200,2470) · Ø300'],[2680,1200,'二维码'],[2400,1020,'(2400,1200)'],[2240,2250,'300'],[2240,150,'返回启停区'],[1200,-140,'粗加工区'],[-160,1360,'暂存区'],[775,775,'450 × 450'],[1625,1625,'450 × 450']])labels.append(svg('text',{x,y:2400-y,'text-anchor':x===0?'start':'middle','font-size':x===2680?44:42,fill:'#294457'},text));
  for(let i=600;i<=2400;i+=600){labels.append(svg('text',{x:i,y:2490,'text-anchor':'middle','font-size':38,fill:'#496277'},String(i)));labels.append(svg('text',{x:-50,y:2410-i,'text-anchor':'end','font-size':38,fill:'#496277'},String(i)));}
}
export function mountMap({send,onStatus=()=>{},isPreparing=()=>false}){
  const telemetry=new Telemetry();let connected=false,running=false,mode='offline',runner=null,activeRows=[],wholeRoute=true;
  let pose={x_mm:2250,y_mm:150,yaw_rad:Math.PI/2},path=[pose];let body=[0,0,0],geometry={...DEFAULT_GEOMETRY};
  const stepper=new FixedStepper();let last=performance.now(),lastRender=0;
  fieldDrawing();
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
  function reset(){if(!readInputs())return;pose={x_mm:Number($('start-x').value),y_mm:Number($('start-y').value),yaw_rad:Number($('start-theta').value)*Math.PI/180};invalidate('模拟已复位 · 从初始位姿开始');}
  function startRoute(rowIndex=null){
    if(!readInputs())return;
    const relevant=rowIndex===null?[...$('route-rows').querySelectorAll('input')]:[...$('route-rows').children[rowIndex].querySelectorAll('input')];
    if(!relevant.every(input=>input.reportValidity()))return;
    try {
      const rows=readRows(),initial=rowIndex===null?{x_mm:Number($('start-x').value),y_mm:Number($('start-y').value),yaw_rad:Number($('start-theta').value)*Math.PI/180}:pose;
      const next=new RouteRunner(rowIndex===null?rows:[rows[rowIndex]],initial,{geometry,integer:$('sim-model').value==='integer'});
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
  $('sim-resume').addEventListener('click',()=>{if(runner?.state!=='running')return;running=true;stepper.reset();last=performance.now();$('sim-status').textContent='路线模拟运行中';render();});
  function pauseSimulation(message){
    if(!running)return;
    running=false;stepper.reset();$('sim-status').textContent=message;render();
  }
  $('sim-pause').addEventListener('click',()=>pauseSimulation('模拟已暂停 · 可继续'));
  $('sim-reset').addEventListener('click',reset);
  $('sim-fields').addEventListener('input',()=>invalidate('参数已编辑 · 整条路线将使用新初始位姿；单段使用当前位姿'));
  $('sim-model').addEventListener('change',()=>invalidate('模型已切换 · 请重新运行路线'));
  $('map-source').addEventListener('change',e=>{pauseSimulation('已切换数据来源 · 模拟暂停，可继续');mode=e.target.value;$('sim-controls').hidden=mode!=='offline';$('route-editor').hidden=mode!=='offline';$('simulation-actions').hidden=mode!=='offline';$('online-controls').hidden=mode==='offline';render();});
  $('stream-on').addEventListener('click',async()=>{
    if(isPreparing())return;
    let nonce=0;while(!nonce)nonce=crypto.getRandomValues(new Uint32Array(1))[0];
    telemetry.begin(nonce,performance.now());render();
    if(!await send(`chassis stream on ${nonce}`))telemetry.stop('开启未发送 · 请重试');render();
  });
  $('stream-off').addEventListener('click',async()=>{if(await send('chassis stream off'))telemetry.stop();render();});
  function drawPath(id,points){let d='',pen=false;for(const p of points){if(!p){pen=false;continue;}d+=`${pen?'L':'M'}${p.x_mm.toFixed(2)},${p.y_mm.toFixed(2)} `;pen=true;}$(id).setAttribute('d',d);}
  function drawCar(p,g,offline){const car=$('map-car');car.replaceChildren();if(!p||!g)return;if(offline)car.append(svg('polygon',{points:bodyCorners(p).map(v=>v.join(',')).join(' '),fill:'#1286b512',stroke:'#0a6b90','stroke-width':5}));const corners=wheelCenters(g).map(v=>{const [x,y]=rotate(v,p.yaw_rad);return [p.x_mm+x,p.y_mm+y];});car.append(svg('polygon',{points:corners.map(v=>v.join(',')).join(' '),fill:'#1286b52e',stroke:'#08658b','stroke-width':7,'stroke-dasharray':'16 10'}));for(let i=0;i<4;i++){const [x,y]=corners[i];car.append(svg('circle',{cx:x,cy:y,r:18,fill:i===0?'#08658b':'#fff',stroke:'#08658b','stroke-width':6}));}
    const [dx,dy]=rotate([g.wheelbase_mm*.7,0],p.yaw_rad);car.append(svg('path',{d:`M ${p.x_mm} ${p.y_mm} l ${dx} ${dy}`,stroke:'#064e70','stroke-width':12,'marker-end':'url(#heading-arrow)'}));}
  function render(){
    const offline=mode==='offline';$('stream-on').disabled=!connected||isPreparing();$('stream-off').disabled=!connected||isPreparing();
    $('telemetry-status').textContent=telemetry.status;onStatus(telemetry.status);
    const g=offline?geometry:telemetry.config?.geometry;
    const displayPose=offline?pose:mode==='command'?(telemetry.command??(telemetry.stale?telemetry.lastCommand:null)):(telemetry.feedback??(telemetry.stale?telemetry.lastFeedback:null));
    $('map-pose').textContent=poseText(displayPose)+(!offline&&telemetry.stale?' · 最后位置（已过期）':'');$('map-car').style.opacity=!offline&&telemetry.stale?'0.35':'1';
    $('map-source-label').textContent=offline?'simulated · 离线模拟':mode==='command'?'command_estimate · 指令推算':'feedback_estimate · 反馈估计';
    $('sim-start').disabled=running;
    $('sim-resume').disabled=running||runner?.state!=='running';
    $('sim-pause').disabled=!running;
    [...$('route-rows').children].forEach((tr,i)=>{tr.classList.toggle('route-active',!!runner&&activeRows[runner.index]===i);tr.querySelector('[data-action="run"]').disabled=running;});
    $('route-progress').textContent=runner?`第 ${activeRows[runner.index]+1} 段 · ${runner.rows[runner.index].name} · ${phaseNames[runner.phase]} · 累计 ${fmt(runner.elapsed,2)} s / ${fmt(runner.distance)} mm`:'';
    $('route-live').textContent=$('sim-status').textContent+(runner?' · '+$('route-progress').textContent:'');
    $('route-results').replaceChildren(...(runner?.stops??[]).map(stop=>{const li=document.createElement('li');li.textContent=`${activeRows[stop.index]+1}. ${stop.name} · ${fmt(stop.time,2)} s · ${poseText(stop.pose)} · 距转盘 ${fmt(turntableGap(stop.pose))} mm`;return li;}));
    const stopMarks=$('route-stops');stopMarks.replaceChildren();
    if(offline){
      // Group nearby labels only; each recorded footprint and exact stop pose stays unchanged.
      const labels=[];
      for(const stop of runner?.stops??[]){
        stopMarks.append(svg('polygon',{points:bodyCorners(stop.pose).map(v=>v.join(',')).join(' '),fill:'none',stroke:'#097da670','stroke-width':3}));
        const group=labels.find(g=>Math.hypot(g.pose.x_mm-stop.pose.x_mm,g.pose.y_mm-stop.pose.y_mm)<30);
        if(group)group.numbers.push(activeRows[stop.index]+1);
        else labels.push({pose:stop.pose,numbers:[activeRows[stop.index]+1]});
      }
      for(const {pose:p,numbers} of labels){
        stopMarks.append(svg('circle',{cx:p.x_mm,cy:p.y_mm,r:26,fill:'#fff',stroke:'#097da6','stroke-width':6}));
        stopMarks.append(svg('text',{transform:`translate(${p.x_mm+40} ${p.y_mm+35}) scale(1 -1)`,'font-size':42,fill:'#065775'},numbers.join('/')));
      }
    }
    drawPath('sim-path',offline?path:[]);drawPath('command-path',offline?[]:telemetry.paths.command);drawPath('feedback-path',offline?[]:telemetry.paths.feedback);drawCar(displayPose,g,offline);
    $('map-geometry').textContent=g?`L ${g.wheelbase_mm} · W ${g.track_mm} · D ${g.wheel_diameter_mm} mm · G ${g.gear_ratio}${offline?' · 轴距/轮距估计，轮径/减速比假设':` · ${telemetry.config.rev} · 固件配置 · 轴距:${statusName(telemetry.config.geometry_status?.wheelbase)} / 轮距:${statusName(telemetry.config.geometry_status?.track)} / 轮径:${statusName(telemetry.config.geometry_status?.diameter)} / 比:${statusName(telemetry.config.geometry_status?.gear_ratio)}`}`:'在线几何未收到 · 不使用离线参数';
    const motion=motionDisplay(telemetry.route,telemetry.distance);
    $('real-route-status').textContent=motion.text;$('map-route-status').textContent=motion.text;
    const routeTarget=$('real-route-target');routeTarget.replaceChildren();
    if(!offline&&motion.target){
      const [x,y,heading]=motion.target,angle=heading*Math.PI/180;
      routeTarget.append(svg('circle',{cx:x,cy:y,r:42,fill:'none',stroke:'#c74333','stroke-width':7}));
      routeTarget.append(svg('path',{d:`M ${x-60} ${y} H ${x+60} M ${x} ${y-60} V ${y+60} M ${x} ${y} l ${100*Math.cos(angle)} ${100*Math.sin(angle)}`,stroke:'#c74333','stroke-width':5}));
      routeTarget.append(svg('text',{transform:`translate(${x+65} ${y+65}) scale(1 -1)`,'font-size':42,fill:'#c74333'},motion.label));
    }
    const r=telemetry.state;
    $('online-details').textContent=r?`最近一帧 MCU ${r.t_ms} ms · 会话 ${r.session} · 周期 ${telemetry.periodMs/1000} s\n任务 ${r.task?.state==='done'&&r.task?.reason==='timed_complete'?'时间任务结束（未表示到达地图目标） · ':''}${r.task?.state??'—'} · ${r.task?.reason??'—'} · ID ${r.task?.id??'—'}\n发送 ${r.tx?.stage??'—'} · 错误 ${r.tx?.error??'—'} · 丢帧 ${r.dropped??'—'}\n航向 ${r.yaw?.valid?fmt(r.yaw.rad,3)+' rad':'不可用'}\n位置反馈单位/圈：${telemetry.config?.position_units_per_rev||'待用户确认（手册65536）'}。不等同坐标。\n反馈符号：${telemetry.config?.feedback_sign??'未标注'}；右侧电机原始正负不等于逻辑前后。`:`等待本会话配置及状态帧，等待上限 ${telemetry.timeoutMs/1000} 秒；无线分段发送会有延迟。`;
    const target=offline?inverse(body,g,SIM_MAX_RPM):r?.target.rpm;
    const approx=offline?quantize(target):null;
    $('wheel-body').replaceChildren(...names.map((name,i)=>{const tr=document.createElement('tr');const fresh=!!r?.feedback.speed_valid[i]&&((r.t_ms-r.feedback.speed_ms[i])>>>0)<=600;const posFresh=!!r?.feedback.position_valid[i]&&((r.t_ms-r.feedback.position_ms[i])>>>0)<=600;for(const value of [name,fmt(target?.[i],2),offline?fmt(approx[i],0):fresh?fmt(r.feedback.rpm[i],2):'不可用',offline?'—':posFresh?fmt(r.feedback.pos[i],0):'不可用']){const td=document.createElement('td');td.textContent=value;tr.append(td);}return tr;}));
    $('wheel-approx-title').textContent=offline?'整数 RPM':telemetry.config?.feedback_sign==='motor_raw'?'原始反馈 RPM':'反馈 RPM';
    $('map-body-speed').textContent=offline?`输入 vx ${fmt(body[0])} · vy ${fmt(body[1])} mm/s · ω ${fmt(body[2],3)} rad/s`:r?`目标 vx ${fmt(r.target.body[0])} · vy ${fmt(r.target.body[1])} mm/s · ω ${fmt(r.target.body[2],3)} rad/s`:'目标速度不可用';
  }
  function frame(now){
    const elapsed=now-last;last=now;telemetry.tick(now);
    if($('map-page').hidden)pauseSimulation('已离开地图页 · 模拟暂停，可继续');
    if(running&&runner){
      if(!stepper.advance(elapsed,dt=>{if(!running)return;runner.advance(dt*Number($('sim-rate').value));pose=runner.pose;path=runner.path;body=runner.body;if(runner.state!=='running'){running=false;$('sim-status').textContent=routeResultText(runner,{wholeRoute,rowNumber:activeRows[runner.index]+1});}})){
        running=false;$('sim-status').textContent='页面停顿超过 250 ms · 模拟已暂停，可继续';
      }
    }
    if(now-lastRender>100){render();lastRender=now;}requestAnimationFrame(frame);
  }
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pauseSimulation('页面已隐藏 · 模拟暂停，可继续');});
  render();requestAnimationFrame(frame);
  return {setSource(value){if(mode===value)return;$('map-source').value=value;$('map-source').dispatchEvent(new Event('change'));},beginPreparedTelemetry(nonce){pauseSimulation('准备实车跑图 · 离线模拟暂停');mode='feedback';$('map-source').value=mode;$('sim-controls').hidden=true;$('route-editor').hidden=true;$('simulation-actions').hidden=true;$('online-controls').hidden=false;telemetry.begin(nonce,performance.now());render();},snapshot(){telemetry.tick(performance.now());return telemetry;},receive(line){telemetry.accept(line,performance.now());},connection(value){connected=value;telemetry.stop(value?'已连接 · 遥测需手动开启':'串口未连接');render();}};
}
