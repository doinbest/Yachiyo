import {RADAR_PARAMETERS,INITIAL_VIEW,STATIONS,radarStateText,radarReasonText,zoomView,panView,importSnapshot} from './radar-model.mjs';
const $=id=>document.getElementById(id),ns='http://www.w3.org/2000/svg';
const element=(tag,text)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
const svg=(tag,attrs)=>{const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);return e;};
const pathData=points=>(points??[]).map((p,i)=>`${i?'L':'M'}${p[0]},${p[1]}`).join(' ');
const GRID=[150,550,1000,1400,1850,2250];

export function mountRadarParameters({root,exchange,model,canSend,onBusy}){
  const section=element('details');section.className='parameter-group';section.append(element('summary','雷达 · 下次静止扫描参数'));
  section.append(element('p','应用后读回核对，只修改当前RAM。安装参考预设2170/230/180对应车在起点(2250,150,90°)，可调，尚未实测标定。设备扫描姿态另行显示，参数变化后旧图保留。'));
  const rows=[];
  async function work(status,task){onBusy(true);status.textContent='等待设备参数…';update();try{await task();}catch(error){status.textContent=error.message;}finally{onBusy(false);update();}}
  for(const p of RADAR_PARAMETERS){
    const row=element('div');row.className='parameter-row';const label=element('label',p.label),input=element('input');Object.assign(input,{type:'number',min:p.min,max:p.max,step:p.key==='zero_deg'?'.1':'1',value:String(p.value)});
    const current=element('span','设备值：未读取'),read=element('button','读取'),apply=element('button','应用'),status=element('span');read.type=apply.type='button';status.setAttribute('role','status');
    read.onclick=()=>void work(status,async()=>{await exchange.get();current.textContent=`设备值：${model.params[p.key]??'未报告'}`;status.textContent='已读回当前RAM参数';});
    apply.onclick=()=>void work(status,async()=>{const result=await exchange.set(p.key,input.value);current.textContent=`设备值：${result.value}`;status.textContent=result.confirmed?'设备读回与输入一致':'读回与输入不一致，未确认生效';});
    row.append(label,input,current,read,apply,status);section.append(row);rows.push({key:p.key,read,apply,current});
  }
  function update(){for(const r of rows){if(model.params[r.key]!==undefined)r.current.textContent=`${model.paramsLive?'设备值':'历史设备值'}：${model.params[r.key]}`;r.read.disabled=!canSend('radar get')||exchange.busy;r.apply.disabled=!canSend('radar set threshold 3')||exchange.busy;}}
  root.append(section);update();return {update,cancel(){for(const r of rows)r.current.textContent='设备值：历史，请主动读回';update();}};
}

export function mountRadarWorkspace({model,exchange,send,canSend,api,mapView,onBusy=()=>{}}){
  let tab='monitor',offline=importSnapshot({mask:0x00050140,counts:Array(25).fill(0),coordinate_frame:'current'}),offlinePath=null,view={...INITIAL_VIEW},pan=null;
  const map=$('field-map'),world=$('map-world');
  const asPoint=(event,node=map)=>new DOMPoint(event.clientX,event.clientY).matrixTransform(node.getScreenCTM().inverse());
  const applyView=()=>map.setAttribute('viewBox',`${view.x} ${view.y} ${view.width} ${view.height}`);
  const zoom=(event,ratio)=>{view=zoomView(view,asPoint(event),ratio);applyView();};
  map.addEventListener('contextmenu',e=>{e.preventDefault();zoom(e,.8);});
  map.addEventListener('wheel',e=>{if(e.deltaY){e.preventDefault();zoom(e,e.deltaY<0?.8:1.25);}},{passive:false});
  map.addEventListener('pointerdown',e=>{if(e.button!==0)return;pan={id:e.pointerId,start:{x:e.clientX,y:e.clientY},view:{...view},matrix:map.getScreenCTM().inverse()};map.setPointerCapture(e.pointerId);e.preventDefault();});
  map.addEventListener('pointermove',e=>{if(pan){const start=new DOMPoint(pan.start.x,pan.start.y).matrixTransform(pan.matrix),current=new DOMPoint(e.clientX,e.clientY).matrixTransform(pan.matrix);view=panView(pan.view,{x:current.x-start.x,y:current.y-start.y});applyView();}const p=asPoint(e,world);$('map-cursor').textContent=`鼠标 X ${p.x.toFixed(1)} · Y ${p.y.toFixed(1)} mm`;});
  const finish=e=>{if(pan&&map.hasPointerCapture(e.pointerId))map.releasePointerCapture(e.pointerId);pan=null;};map.addEventListener('pointerup',finish);map.addEventListener('pointercancel',finish);
  $('map-view-reset').onclick=()=>{view={...INITIAL_VIEW};applyView();};
  function selectTab(value){
    tab=value;for(const b of $('map-tabs').children){const active=b.dataset.mapTab===tab;b.classList.toggle('active',active);b.setAttribute('aria-selected',String(active));}
    $('radar-build-panel').hidden=tab!=='radar';$('radar-offline-panel').hidden=tab!=='offline';$('radar-monitor-panel').hidden=tab!=='monitor';
    const offlineTab=tab==='offline';mapView.setSource(offlineTab?'offline':'feedback');
    for(const id of ['real-route-panel','online-controls'])$(id).hidden=tab!=='monitor';for(const id of ['sim-controls','simulation-actions','route-editor','speed-lab-note'])$(id).hidden=!offlineTab;render();
  }
  $('map-tabs').onclick=e=>{const b=e.target.closest('[data-map-tab]');if(b)selectTab(b.dataset.mapTab);};
  async function operate(work){onBusy(true);try{await work();}catch(error){$('radar-transfer').textContent=error.message;}finally{onBusy(false);render();}}
  exchange.onProgress=({kind,page,pages})=>{$('radar-transfer').textContent=`读取 ${kind} · ${page}/${pages} 页 · 完整后显示`;};
  for(const b of document.querySelectorAll('[data-radar-wire]'))b.onclick=()=>void send(b.dataset.radarWire);
  const currentId=kind=>kind==='path'?(model.status?.plan??model.snapshots.path?.plan):['points','cloud'].includes(kind)?(model.status?.scan??model.snapshots.map?.scan):(model.status?.map??model.snapshots.map?.map);
  for(const kind of ['map','path','points','cloud'])$(`radar-fetch-${kind}`).onclick=()=>{
    if(!canSend(`radar fetch ${kind} ${currentId(kind)??0} 0`)){$('radar-transfer').textContent='请连接设备并结束当前准备';return;}
    void operate(async()=>{
      if(['points','cloud'].includes(kind)){
        if(model.status?.points_valid===false||model.status?.points_valid===0)throw new Error('设备点云无效，保留已下载历史点云；请等完整新圈成功');
        const stored=model.snapshots.map;if(!stored||stored.map!==model.status?.map||stored.scan!==currentId(kind))await exchange.fetch('map',model.status?.map);
      }
      await exchange.fetch(kind,currentId(kind));$('radar-transfer').textContent=`${kind}完整快照已读回`;if(kind==='points')$('radar-show-points').checked=true;
    });
  };
  $('radar-cancel-transfer').onclick=()=>exchange.cancel();
  const start=kind=>{const speed=Number($('radar-nav-speed').value);if(!Number.isInteger(speed)||speed<10||speed>5000){$('radar-transfer').textContent='速度范围10～5000 mm/s';return;}void send(`radar ${kind} start ${speed}`);};$('radar-nav-start').onclick=()=>start('nav');$('radar-task-start').onclick=()=>start('task');
  for(const id of ['radar-show-obstacles','radar-show-path','radar-show-points','radar-show-counts'])$(id).onchange=render;
  const offlineInputs={};for(const p of RADAR_PARAMETERS){const label=element('label',p.label),input=element('input');Object.assign(input,{type:'number',value:p.value,min:p.min,max:p.max,step:p.key==='zero_deg'?'.1':'1'});label.append(input);$('radar-offline-params').append(label);offlineInputs[p.key]=input;}
  function setOffline(snapshot){
    offline=importSnapshot(snapshot);offlinePath=null;const params={...Object.fromEntries(RADAR_PARAMETERS.map(p=>[p.key,p.value])),...model.params,...offline.params};if(offline.pose){params.lidar_x_mm=offline.pose[0];params.lidar_y_mm=offline.pose[1];params.zero_deg=offline.pose[2];}for(const[key,input]of Object.entries(offlineInputs))input.value=String(params[key]);for(const input of $('radar-mask-cells').querySelectorAll('input'))input.checked=!!(offline.mask&(1<<Number(input.dataset.cell)));$('radar-offline-status').textContent='离线副本已载入；可改过滤或手工障碍后本机重新规划';render();
  }
  for(let row=4;row>=0;row--)for(let col=0;col<5;col++){const label=element('label',`R${row+1}C${col+1}`),input=element('input');input.type='checkbox';input.dataset.cell=row*5+col;input.onchange=()=>{if(!offline)return;const bit=1<<Number(input.dataset.cell);offline.mask=input.checked?offline.mask|bit:offline.mask&~bit;offline.manual_mask=input.checked?(offline.manual_mask??0)|bit:(offline.manual_mask??0)&~bit;render();};label.prepend(input);$('radar-mask-cells').append(label);input.checked=!!(offline.mask&(1<<Number(input.dataset.cell)));}
  $('radar-use-device').onclick=()=>{if(!model.snapshots.map){$('radar-offline-status').textContent='先读取完整设备地图';return;}setOffline({snapshots:model.snapshots,params:model.params});};
  $('radar-import').onchange=async()=>{const file=$('radar-import').files[0];if(!file)return;try{const value=JSON.parse(await file.text()),frame=$('radar-import-frame').value;if(value.snapshots?.map)value.snapshots.map.coordinate_frame=frame;else value.coordinate_frame=frame;setOffline(value);}catch(error){$('radar-offline-status').textContent=error.message;}};
  $('radar-export').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify({v:1,coordinate_frame:'current',snapshots:model.snapshots,params:model.params,offline,offlinePath},null,2)],{type:'application/json'})),a=element('a');a.href=url;a.download=`radar-${Date.now()}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  $('radar-offline-plan').onclick=async()=>{
    if(!offline){$('radar-offline-status').textContent='先载入完整地图副本';return;}const params=Object.fromEntries(Object.entries(offlineInputs).map(([key,input])=>[key,Number(input.value)])),useCloud=$('radar-offline-input').value==='cloud',cloud=offline.cloud;if(useCloud&&!cloud?.points){$('radar-offline-status').textContent='重新过滤需要完整点云或显示点云快照';return;}
    $('radar-offline-plan').disabled=true;$('radar-offline-status').textContent='本机C核心计算中…';try{offlinePath=await api('radar/plan',{mask:useCloud?(offline.manual_mask??0):offline.mask,params,points:useCloud?cloud.points:[]});offline.params=params;offline.receivedAt=Date.now();$('radar-offline-status').textContent=offlinePath.valid?`${offlinePath.algorithm} · 离线预览；未发送实车指令`:`规划不可达 · 第 ${offlinePath.failed_leg} 路段`;render();}catch(error){$('radar-offline-status').textContent=error.message;}finally{$('radar-offline-plan').disabled=false;}
  };
  function render(){
    const mapSnapshot=model.snapshots.map,path=model.snapshots.path,state=model.status,nav=model.nav;
    $('radar-map-age').textContent=model.describe(tab==='offline'?offline:mapSnapshot)+(state?.applicable===false||state?.applicable===0?' · 参数/原点已变化，请重新扫描':'')+(mapSnapshot?.coverage!==undefined?` · 覆盖 ${(mapSnapshot.coverage/10).toFixed(1)}°`:'');$('radar-path-age').textContent=`设备：${model.describe(path)}${offlinePath?'；虚线：本机离线预览':''}`;
    $('radar-status').textContent=state?`${radarStateText(state.state)} · ${radarReasonText(state.reason)} · 扫描 ${state.scan} / 地图 ${state.map} / 规划 ${state.plan}`:'点击读取状态，确认设备雷达配置';$('radar-nav-status').textContent=nav?`${radarStateText(nav.state)} · ${radarReasonText(nav.reason)} · ${nav.index}/${nav.total} · ${STATIONS[nav.station]??'通行'} · 访问 ${nav.visit}${nav.state==='stopping'?((nav.stopped??nav.stop_confirmed)?' · 停车已确认':' · 停车待确认'):''}`:'尚无设备导航状态';
    $('radar-diagnostics').textContent=JSON.stringify({status:state,nav},null,2);
    $('radar-execution-reason').textContent=state?.map===0?'尚无设备地图，请静止扫描':state?.applicable===false||state?.applicable===0?'当前地图原点或参数已变化，请重新扫描':path?.plan===state?.plan&&path?.valid===false?`第 ${path.failed_leg} 路段不可达，请调整障碍或扫描过滤参数`:'执行由设备检查当前地图和底盘准备；无需下载完整点云';
    $('radar-param-summary').textContent=Object.keys(model.params).length?`雷达 (${model.params.lidar_x_mm},${model.params.lidar_y_mm}) mm · 零角 ${model.params.zero_deg}° · 阈值 ${model.params.threshold}`:'参数尚未读回 · 参数调试页主动读取';
    const obstacles=$('radar-obstacles');obstacles.replaceChildren();const mask=tab==='offline'?(offlinePath?.mask??offline?.mask??mapSnapshot?.mask??0):(mapSnapshot?.mask??0);if($('radar-show-obstacles').checked)for(let i=0;i<25;i++)if(mask&(1<<i)){const row=Math.floor(i/5),col=i%5;obstacles.append(svg('rect',{x:GRID[col],y:GRID[row],width:GRID[col+1]-GRID[col],height:GRID[row+1]-GRID[row],fill:'#d9444433',stroke:'#c34343','stroke-width':4}));}
    const counts=$('radar-counts');counts.replaceChildren();const cellCounts=tab==='offline'?(offlinePath?.counts??offline?.counts):mapSnapshot?.counts;if($('radar-show-counts').checked&&cellCounts)for(let i=0;i<25;i++){const row=Math.floor(i/5),col=i%5,x=(GRID[col]+GRID[col+1])/2,y=(GRID[row]+GRID[row+1])/2,label=svg('text',{transform:`translate(${x} ${y}) scale(1 -1)`,'text-anchor':'middle','font-size':48,fill:'#1b4d65'});label.textContent=String(cellCounts[i]??0);counts.append(label);}
    $('radar-device-path').setAttribute('d',$('radar-show-path').checked&&path?.valid!==false?pathData(path?.points):'');$('radar-device-path').style.opacity=!model.connected||state&&(state.map!==path?.map||state.plan!==path?.plan)?'.4':'1';$('radar-offline-path').setAttribute('d',tab==='offline'&&offlinePath?.valid?pathData(offlinePath.points):'');
    const marks=$('radar-stations');marks.replaceChildren();if(path?.points&&$('radar-show-path').checked)for(const p of path.points)if(p[2])marks.append(svg('circle',{cx:p[0],cy:p[1],r:20,fill:'#eaf9f3',stroke:'#008e64','stroke-width':5}));
    const cloudLayer=$('radar-cloud');cloudLayer.replaceChildren();const cloud=tab==='offline'?offline?.cloud:[model.snapshots.points,model.snapshots.cloud].filter(Boolean).sort((a,b)=>b.receivedAt-a.receivedAt)[0];if($('radar-show-points').checked&&cloud?.points){const pose=cloud.pose??(tab==='offline'?offline?.pose:mapSnapshot?.pose);if(pose)for(const p of cloud.points.filter((_,i)=>i%Math.max(1,Math.ceil(cloud.points.length/360))===0)){const angle=(pose[2]-p[0]/10)*Math.PI/180;cloudLayer.append(svg('circle',{cx:pose[0]+p[1]*Math.cos(angle),cy:pose[1]+p[1]*Math.sin(angle),r:7,fill:'#1674a9'}));}cloudLayer.style.opacity=!model.connected||!state?.points_valid||state?.scan!==cloud.scan?'.3':'1';}
    $('radar-points-age').hidden=!$('radar-show-points').checked;$('radar-points-age').textContent=cloud?`${model.describe(cloud)} · ${cloud.points.length} 点${cloud.k==='points'?'（显示抽样）':''}${cloud.truncated?' · 单圈缓存已截断':''}`:'尚无已下载点云';
    for(const b of document.querySelectorAll('[data-radar-wire]'))b.disabled=!canSend(b.dataset.radarWire);$('radar-nav-start').disabled=!canSend(`radar nav start ${$('radar-nav-speed').value}`);$('radar-task-start').disabled=!canSend(`radar task start ${$('radar-nav-speed').value}`);
  }
  selectTab('monitor');render();setInterval(render,1000);return {receive(line){const recognized=model.accept(line);exchange.receive(line);if(recognized)render();return recognized;},connection(value){model.connection(value);if(!value)exchange.cancel('连接已变化，旧完整快照保留');render();},render,selectTab,snapshot:()=>model};
}
