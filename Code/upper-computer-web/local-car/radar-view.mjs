import {RADAR_PARAMETERS,INITIAL_VIEW,STATIONS,radarStateText,radarReasonText,importSnapshot,parseRadar} from './radar-model.mjs';
import {RouteProfileRunner,profilePhaseText,profileResultText} from './route-profile.mjs';
import {radarProbeText} from './radar-format.mjs';
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

export function mountRadarWorkspace({model,exchange,send,canSend,api,mapView}){
  let tab='monitor',offline=importSnapshot({mask:0x00050140,counts:Array(25).fill(0),coordinate_frame:'current'}),offlinePath=null;
  let transfer=null,scanActivity=null,pendingResult=null,cancelledCapture=null,scheduled=false,internalCancel=false;
  const captureKey=f=>f?`${f.session}:${f.origin}:${f.scan}:${f.map}`:null;
  const resultKey=f=>f?`${captureKey(f)}:${f.plan}`:null;
  const sameResult=target=>model.status?.state==='ready'&&resultKey(model.status)===resultKey(target);
  const optional=kind=>['points','cloud'].includes(kind);
  const selectedProfile=()=>offlinePath?.route_profiles?.[$('route-profile-choice').value||'fixed'];
  function renderProfile(){
    const profile=selectedProfile();$('route-profile-start').disabled=!profile;
    mapView.setPreviewSource?.(profile??null);
    $('route-profile-grid').textContent=offlinePath?(offlinePath.valid?'C 网格可达 · 车体外框另行检查':`C 网格不可达 · 第 ${offlinePath.failed_leg} 路段`):'C 网格尚未计算';
    $('route-profile-phases').replaceChildren(...(profile?.phases??[]).map(p=>element('li',profilePhaseText(p))));
    if(!profile){$('route-profile-status').textContent='点击本机 C 核心重新规划，读取当前固件阶段规则';return;}
    try{const preview=new RouteProfileRunner(profile,{mask:offlinePath.mask??0});preview.advance(100000);$('route-profile-status').textContent=profileResultText(preview);}
    catch(error){$('route-profile-status').textContent=error.message;}
  }
  $('route-profile-choice').onchange=()=>{renderProfile();render();};
  $('route-profile-start').onclick=()=>{const profile=selectedProfile();if(profile)mapView.startProfile(profile,{mask:offlinePath.mask??0});};
  const map=$('field-map'),world=$('map-world');
  map.setAttribute('viewBox',`${INITIAL_VIEW.x} ${INITIAL_VIEW.y} ${INITIAL_VIEW.width} ${INITIAL_VIEW.height}`);
  map.addEventListener('pointermove',e=>{const p=new DOMPoint(e.clientX,e.clientY).matrixTransform(world.getScreenCTM().inverse());$('map-cursor').textContent=`鼠标 X ${p.x.toFixed(1)} · Y ${p.y.toFixed(1)} mm`;});
  function selectTab(value){
    tab=value;for(const b of $('map-tabs').children){const active=b.dataset.mapTab===tab;b.classList.toggle('active',active);b.setAttribute('aria-selected',String(active));}
    $('map-workspace-title').textContent=({monitor:'实车监控',radar:'雷达建图',offline:'离线对照'})[tab];
    $('radar-build-panel').hidden=tab!=='radar';$('radar-offline-panel').hidden=tab!=='offline';$('radar-monitor-panel').hidden=tab!=='monitor';
    const offlineTab=tab==='offline';
    mapView.setSource(offlineTab?'offline':'feedback');
    for(const id of ['real-route-panel','online-controls'])$(id).hidden=tab!=='monitor';for(const id of ['sim-controls','simulation-actions','route-editor','speed-lab-note'])$(id).hidden=!offlineTab;render();
  }
  $('map-tabs').onclick=e=>{const b=e.target.closest('[data-map-tab]');if(b){selectTab(b.dataset.mapTab);if(globalThis.location)location.hash='map/'+b.dataset.mapTab;}};
  async function operate(kind,work){
    if(transfer||exchange.busy)return;
    const operation={kind};operation.done=new Promise(resolve=>{operation.finish=resolve;});transfer=operation;render();
    try{await work(operation);}catch(error){if(!operation.obsolete)$('radar-transfer').textContent=error.message;}
    finally{if(transfer===operation)transfer=null;operation.finish();render();}
  }
  function checkRead(operation){if(operation.cancelled||operation.obsolete)throw new Error('读取已取消，保留已完成快照');}
  function supersedeRead(reason){if(transfer){transfer.obsolete=true;internalCancel=true;exchange.cancel(reason);internalCancel=false;}}
  function scheduleResult(){
    if(scheduled||!pendingResult)return;scheduled=true;
    queueMicrotask(()=>{
      scheduled=false;
      if(!pendingResult||transfer||exchange.busy)return;
      const target=pendingResult;
      if(!sameResult(target)){pendingResult=null;return;}
      if(!canSend(`radar fetch map ${target.map} 0`))return;
      pendingResult=null;
      void operate('result',async operation=>{
        await exchange.fetch('map',target.map);
        checkRead(operation);
        if(!sameResult(target))throw new Error('新扫描已开始，已完成历史地图保留');
        await exchange.fetch('path',target.plan);
        checkRead(operation);
        const path=model.snapshots.path;
        $('radar-transfer').textContent=path?.valid===false?`地图已显示 · 第 ${path.failed_leg} 路段不可达`:'地图与规划路径已显示 · 点云可按需读取';
      });
    });
  }
  exchange.subscribeState(({type})=>{
    if(type==='cancel')model.pending.clear();
    if(type==='cancel'&&!internalCancel){if(transfer)transfer.cancelled=true;pendingResult=null;cancelledCapture=scanActivity?.key??null;scanActivity=null;}
    if(type==='idle')scheduleResult();
  });
  function observeScan(f){
    if(!['status','diag'].includes(f.k))return;
    if(['preparing','wait_wrap','collecting'].includes(f.state)){
      const key=captureKey(f);if(cancelledCapture===key)return;
      if(!scanActivity){pendingResult=null;if(transfer&&!optional(transfer.kind))supersedeRead('新扫描开始，旧结果读取已取消');scanActivity={key,observed:true};}
      else{if(scanActivity.key===null)scanActivity.key=key;scanActivity.observed=true;}
      return;
    }
    if(['stopped','error'].includes(f.state)){scanActivity=null;pendingResult=null;cancelledCapture=null;if(transfer?.kind==='result')supersedeRead('扫描已结束，保留历史结果');return;}
    if(f.state!=='ready'||!scanActivity||scanActivity.key===null||scanActivity.explicit&&!scanActivity.accepted&&!scanActivity.observed)return;
    if(captureKey(f)===scanActivity.key)return;
    scanActivity=null;
    if(!f.map||!f.plan||f.applicable===false||f.applicable===0)return;
    pendingResult={...f};$('radar-transfer').textContent='采集与规划已完成 · 等待读取地图与路径';
    if(optional(transfer?.kind))supersedeRead('新地图已完成，优先读取地图与路径');scheduleResult();
  }
  function observeScanReply(line){
    if(!scanActivity?.explicit)return false;
    const reply=line.trim().replace(/^(?:arm>\s*)+/,'');
    if(reply==='OK radar scan_accepted'){scanActivity.accepted=true;return true;}
    if(/^ERR radar (?:stationary_capture_required|query_in_progress|scan_busy_or_parameters)$/.test(reply)){scanActivity=null;$('radar-transfer').textContent=`扫描未受理 · ${radarReasonText(reply.split(' ')[2])}`;return true;}
    return false;
  }
  exchange.onProgress=({kind,page,pages})=>{const label=({map:'地图',path:'规划路径',points:'可选显示点云',cloud:'可选完整点云'})[kind]??kind;$('radar-transfer').textContent=`下载 ${label} · ${page}/${pages} 页 · 完整后显示`;};
  for(const b of document.querySelectorAll('[data-radar-wire]'))b.onclick=()=>{
    const wire=b.dataset.radarWire;if(!canSend(wire)||wire==='radar scan'&&(exchange.busy||transfer||scanActivity))return;
    if(wire==='radar scan'){cancelledCapture=null;pendingResult=null;scanActivity={key:captureKey(model.status),explicit:true,accepted:false,observed:false};$('radar-transfer').textContent='已请求静止扫描 · 完成后自动显示地图与路径';}
    if(wire==='radar stop'||wire==='radar nav cancel')exchange.cancel();
    void Promise.resolve(send(wire)).then(ok=>{if(wire==='radar scan'&&ok!==true)scanActivity=null;render();});
  };
  const currentId=kind=>kind==='path'?(model.status?.plan??model.snapshots.path?.plan):['points','cloud'].includes(kind)?(model.status?.scan??model.snapshots.map?.scan):(model.status?.map??model.snapshots.map?.map);
  for(const kind of ['map','path','points','cloud'])$(`radar-fetch-${kind}`).onclick=async()=>{
    if(!canSend(`radar fetch ${kind} ${currentId(kind)??0} 0`)){$('radar-transfer').textContent='请连接设备并结束当前准备';return;}
    if(!optional(kind)&&optional(transfer?.kind)){
      const previous=transfer;supersedeRead('优先刷新地图或路径，已取消可选点云读取');const revision=exchange.revision;
      await previous.done;if(revision!==exchange.revision||!canSend(`radar fetch ${kind} ${currentId(kind)??0} 0`))return;
    }
    void operate(kind,async operation=>{
      if(['points','cloud'].includes(kind)){
        if(model.status?.points_valid===false||model.status?.points_valid===0)throw new Error('设备点云无效，保留已下载历史点云；请等完整新圈成功');
        const stored=model.snapshots.map;if(!stored||stored.map!==model.status?.map||stored.scan!==currentId(kind))await exchange.fetch('map',model.status?.map);
      }
      checkRead(operation);await exchange.fetch(kind,currentId(kind));checkRead(operation);$('radar-transfer').textContent=`${kind}完整快照已读回`;if(kind==='points')$('radar-show-points').checked=true;
    });
  };
  $('radar-cancel-transfer').onclick=()=>{exchange.cancel();$('radar-transfer').textContent='读取已取消，保留已完成快照';render();};
  const start=kind=>{const speed=Number($('radar-nav-speed').value);if(!Number.isInteger(speed)||speed<10||speed>5000){$('radar-transfer').textContent='速度范围10～5000 mm/s';return;}void send(`radar ${kind} start ${speed}`);};$('radar-nav-start').onclick=()=>start('nav');$('radar-task-start').onclick=()=>start('task');
  for(const id of ['radar-show-obstacles','radar-show-points','radar-show-counts'])$(id).onchange=render;
  function invalidateOfflinePlan(){offlinePath=null;$('radar-offline-status').textContent='离线输入已修改，请重新规划当前 C 阶段';renderProfile();render();}
  const offlineInputs={};for(const p of RADAR_PARAMETERS){const label=element('label',p.label),input=element('input');Object.assign(input,{type:'number',value:p.value,min:p.min,max:p.max,step:p.key==='zero_deg'?'.1':'1'});label.append(input);$('radar-offline-params').append(label);offlineInputs[p.key]=input;input.onchange=invalidateOfflinePlan;}
  function setOffline(snapshot){
    offline=importSnapshot(snapshot);offlinePath=null;const params={...Object.fromEntries(RADAR_PARAMETERS.map(p=>[p.key,p.value])),...model.params,...offline.params};if(offline.pose){params.lidar_x_mm=offline.pose[0];params.lidar_y_mm=offline.pose[1];params.zero_deg=offline.pose[2];}for(const[key,input]of Object.entries(offlineInputs))input.value=String(params[key]);for(const input of $('radar-mask-cells').querySelectorAll('input'))input.checked=!!(offline.mask&(1<<Number(input.dataset.cell)));$('radar-offline-status').textContent='离线副本已载入；可改过滤或手工障碍后本机重新规划';renderProfile();render();
  }
  for(let row=4;row>=0;row--)for(let col=0;col<5;col++){const label=element('label',`R${row+1}C${col+1}`),input=element('input');input.type='checkbox';input.dataset.cell=row*5+col;input.onchange=()=>{if(!offline)return;const bit=1<<Number(input.dataset.cell);offline.mask=input.checked?offline.mask|bit:offline.mask&~bit;offline.manual_mask=input.checked?(offline.manual_mask??0)|bit:(offline.manual_mask??0)&~bit;invalidateOfflinePlan();};label.prepend(input);$('radar-mask-cells').append(label);input.checked=!!(offline.mask&(1<<Number(input.dataset.cell)));}
  $('radar-use-device').onclick=()=>{if(!model.snapshots.map){$('radar-offline-status').textContent='先读取完整设备地图';return;}setOffline({snapshots:model.snapshots,params:model.params});};
  $('radar-import').onchange=async()=>{const file=$('radar-import').files[0];if(!file)return;try{const value=JSON.parse(await file.text()),frame=$('radar-import-frame').value;if(value.snapshots?.map)value.snapshots.map.coordinate_frame=frame;else value.coordinate_frame=frame;setOffline(value);}catch(error){$('radar-offline-status').textContent=error.message;}};
  $('radar-export').onclick=()=>{const url=URL.createObjectURL(new Blob([JSON.stringify({v:1,coordinate_frame:'current',snapshots:model.snapshots,params:model.params,offline,offlinePath},null,2)],{type:'application/json'})),a=element('a');a.href=url;a.download=`radar-${Date.now()}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
  $('radar-offline-plan').onclick=async()=>{
    if(!offline){$('radar-offline-status').textContent='先载入完整地图副本';return;}const params=Object.fromEntries(Object.entries(offlineInputs).map(([key,input])=>[key,Number(input.value)])),useCloud=$('radar-offline-input').value==='cloud',cloud=offline.cloud;if(useCloud&&!cloud?.points){$('radar-offline-status').textContent='重新过滤需要完整点云或显示点云快照';return;}
    const speed=Number($('radar-nav-speed').value);if(!Number.isInteger(speed)||speed<10||speed>5000){$('radar-offline-status').textContent='启动速度范围10～5000 mm/s';return;}
    $('radar-offline-plan').disabled=true;$('radar-offline-status').textContent='本机C核心计算中…';try{offlinePath=await api('radar/plan',{mask:useCloud?(offline.manual_mask??0):offline.mask,params,points:useCloud?cloud.points:[],speed_mm_s:speed});offline.params=params;offline.receivedAt=Date.now();$('radar-offline-status').textContent=offlinePath.valid?`${offlinePath.algorithm} · 离线预览；未发送实车指令`:`规划不可达 · 第 ${offlinePath.failed_leg} 路段`;renderProfile();render();}catch(error){$('radar-offline-status').textContent=error.message;}finally{$('radar-offline-plan').disabled=false;}
  };
  function render(){
    const mapSnapshot=model.snapshots.map,path=model.snapshots.path,state=model.status,nav=model.nav;
    $('radar-map-age').textContent=model.describe(tab==='offline'?offline:mapSnapshot)+(state?.applicable===false||state?.applicable===0?' · 参数/原点已变化，请重新扫描':'')+(mapSnapshot?.coverage!==undefined?` · 覆盖 ${(mapSnapshot.coverage/10).toFixed(1)}°`:'');$('radar-path-age').textContent=`设备：${model.describe(path)}${offlinePath?'；虚线：本机离线预览':''}`;
    $('radar-status').textContent=state?`采集与规划：${radarStateText(state.state)} · ${radarReasonText(state.reason)} · 扫描 ${state.scan} / 地图 ${state.map} / 规划 ${state.plan}`:'点击读取状态，确认设备雷达配置';$('radar-nav-status').textContent=nav?`${radarStateText(nav.state)} · ${radarReasonText(nav.reason)} · ${nav.index}/${nav.total} · ${STATIONS[nav.station]??'通行'} · 访问 ${nav.visit}${nav.state==='stopping'?((nav.stopped??nav.stop_confirmed)?' · 停车已确认':' · 停车待确认'):''}`:'尚无设备导航状态';
    $('radar-probe-status').textContent=radarProbeText(model.probe);
    $('radar-diagnostics').textContent=JSON.stringify({status:state,nav,probe:model.probe},null,2);
    $('radar-execution-reason').textContent=state?.map===0?'尚无设备地图，请静止扫描':state?.applicable===false||state?.applicable===0?'当前地图原点或参数已变化，请重新扫描':path?.plan===state?.plan&&path?.valid===false?`第 ${path.failed_leg} 路段不可达，请调整障碍或扫描过滤参数`:'执行由设备检查当前地图和底盘准备；无需下载完整点云';
    $('radar-param-summary').textContent=Object.keys(model.params).length?`雷达 (${model.params.lidar_x_mm},${model.params.lidar_y_mm}) mm · 零角 ${model.params.zero_deg}° · 阈值 ${model.params.threshold}`:'参数尚未读回 · 参数调试页主动读取';
    const obstacles=$('radar-obstacles');obstacles.replaceChildren();const mask=tab==='offline'?(offlinePath?.mask??offline?.mask??mapSnapshot?.mask??0):(mapSnapshot?.mask??0);if($('radar-show-obstacles').checked)for(let i=0;i<25;i++)if(mask&(1<<i)){const row=Math.floor(i/5),col=i%5;obstacles.append(svg('rect',{x:GRID[col],y:GRID[row],width:GRID[col+1]-GRID[col],height:GRID[row+1]-GRID[row],fill:'#d9444433',stroke:'#c34343','stroke-width':4}));}
    const counts=$('radar-counts');counts.replaceChildren();const cellCounts=tab==='offline'?(offlinePath?.counts??offline?.counts):mapSnapshot?.counts;if($('radar-show-counts').checked&&cellCounts)for(let i=0;i<25;i++){const row=Math.floor(i/5),col=i%5,x=(GRID[col]+GRID[col+1])/2,y=(GRID[row]+GRID[row+1])/2,label=svg('text',{transform:`translate(${x} ${y}) scale(1 -1)`,'text-anchor':'middle','font-size':48,fill:'#1b4d65'});label.textContent=String(cellCounts[i]??0);counts.append(label);}
    $('radar-device-path').setAttribute('d',path?.valid!==false?pathData(path?.points):'');$('radar-device-path').style.opacity=!model.connected||state&&(state.map!==path?.map||state.plan!==path?.plan)?'.4':'1';$('radar-offline-path').setAttribute('d',tab==='offline'?(selectedProfile()?.points?pathData(selectedProfile().points):offlinePath?.valid?pathData(offlinePath.points):''):'');
    const marks=$('radar-stations');marks.replaceChildren();if(path?.valid!==false&&path?.points)for(const p of path.points)if(p[2])marks.append(svg('circle',{cx:p[0],cy:p[1],r:20,fill:'#eaf9f3',stroke:'#008e64','stroke-width':5}));
    const cloudLayer=$('radar-cloud');cloudLayer.replaceChildren();const cloud=tab==='offline'?offline?.cloud:[model.snapshots.points,model.snapshots.cloud].filter(Boolean).sort((a,b)=>b.receivedAt-a.receivedAt)[0];if($('radar-show-points').checked&&cloud?.points){const pose=cloud.pose??(tab==='offline'?offline?.pose:mapSnapshot?.pose);if(pose)for(const p of cloud.points.filter((_,i)=>i%Math.max(1,Math.ceil(cloud.points.length/360))===0)){const angle=(pose[2]-p[0]/10)*Math.PI/180;cloudLayer.append(svg('circle',{cx:pose[0]+p[1]*Math.cos(angle),cy:pose[1]+p[1]*Math.sin(angle),r:7,fill:'#1674a9'}));}cloudLayer.style.opacity=!model.connected||!state?.points_valid||state?.scan!==cloud.scan?'.3':'1';}
    $('radar-points-age').hidden=!$('radar-show-points').checked;$('radar-points-age').textContent=cloud?`${model.describe(cloud)} · ${cloud.points.length} 点${cloud.k==='points'?'（显示抽样）':''}${cloud.truncated?' · 单圈缓存已截断':''}`:'尚无已下载点云';
    for(const b of document.querySelectorAll('[data-radar-wire]'))b.disabled=!canSend(b.dataset.radarWire)||b.dataset.radarWire==='radar scan'&&!!(exchange.busy||transfer||scanActivity);
    for(const kind of ['map','path','points','cloud'])$(`radar-fetch-${kind}`).disabled=!!(exchange.busy||transfer)&&!(!optional(kind)&&optional(transfer?.kind))||!canSend(`radar fetch ${kind} ${currentId(kind)??0} 0`);
    $('radar-cancel-transfer').disabled=!(exchange.busy||transfer||pendingResult);$('radar-nav-start').disabled=!canSend(`radar nav start ${$('radar-nav-speed').value}`);$('radar-task-start').disabled=!canSend(`radar task start ${$('radar-nav-speed').value}`);scheduleResult();
  }
  renderProfile();selectTab('monitor');render();setInterval(render,1000);return {receive(line){const recognized=model.accept(line);exchange.receive(line);const replyChanged=observeScanReply(line),frame=parseRadar(line);if(frame)observeScan(frame);if(recognized||replyChanged)render();return recognized;},connection(value){model.connection(value);if(!value){scanActivity=null;pendingResult=null;cancelledCapture=null;exchange.cancel('连接已变化，旧完整快照保留');}render();},render,selectTab,snapshot:()=>model};
}
