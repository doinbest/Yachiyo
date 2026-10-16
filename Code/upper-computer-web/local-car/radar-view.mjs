import {RADAR_PARAMETERS,RADAR_GRID,RADAR_FIXED_MASK,INITIAL_VIEW,STATIONS,radarStateText,radarReasonText,importSnapshot,parseRadar,diagnosticSnapshot,isCompleteRadarCloud,preferredRadarCloud} from './radar-model.mjs';
import {RouteProfileRunner,profilePhaseText,profileResultText} from './route-profile.mjs';
import {mountRouteAnnotations,routeSegments} from './route-annotations.mjs';
import {radarProbeText,radarStartText} from './radar-format.mjs';
import {radarLowerReferenceFromWorld,radarWorldFromLowerReference,projectFrozenRadarPoint,stableFieldPoint} from './radar-installation.mjs';
import {RadarLiveModel,projectRadarPoint} from './radar-live.mjs';
import {MonitorResult,monitorGrid,monitorProtectedMask} from './monitor-radar.mjs';
const $=id=>document.getElementById(id),ns='http://www.w3.org/2000/svg';
const element=(tag,text)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
const svg=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const[k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;return e;};
const pathData=points=>(points??[]).map((p,i)=>`${i?'L':'M'}${p[0]},${p[1]}`).join(' ');
const GRID=RADAR_GRID;

export function mountRadarParameters({root,exchange,model,canSend,onBusy}){
  const section=element('details');section.className='parameter-group';section.append(element('summary','雷达 · 下次静止扫描参数'));
  section.append(element('p','下方默认情况2(2280,314)，0°朝180°，使用精确Q16矩阵与原点；右上固定(2250,2250,270°)和单位矩阵。正常建图采集15有效完整圈生成稳定XY。几何与过滤变化重建15圈窗口，应用只修改RAM；旧图保留冻结参数。'));
  const rows=[];
  async function work(status,task){onBusy(true);status.textContent='等待设备参数…';update();try{await task();}catch(error){status.textContent=error.message;}finally{onBusy(false);update();}}
  for(const p of RADAR_PARAMETERS){
    const row=element('div');row.className='parameter-row';const label=element('label',p.label),input=element('input');Object.assign(input,{type:'number',min:p.min,max:p.max,step:p.key==='zero_deg'?'.1':'1',value:String(p.value)});
    const current=element('span','设备值：未读取'),read=element('button','读取'),apply=element('button','应用'),status=element('span');read.type=apply.type='button';status.setAttribute('role','status');
    read.onclick=()=>void work(status,async()=>{await exchange.get();current.textContent=`设备值：${model.params[p.key]??'未报告'}`;status.textContent='已读回当前RAM参数';});
    input.readOnly=p.key==='grid_version';apply.onclick=()=>{if(p.key==='grid_version')return;void work(status,async()=>{const result=await exchange.set(p.key,input.value);current.textContent=`设备值：${result.value}`;status.textContent=result.confirmed?'设备读回与输入一致':'读回与输入不一致，未确认生效';});};
    row.append(label,input,current,read,apply,status);section.append(row);rows.push({key:p.key,read,apply,current});
  }
  function update(){for(const r of rows){if(model.params[r.key]!==undefined)r.current.textContent=`${model.paramsLive?'设备值':'历史设备值'}：${model.params[r.key]}`;r.read.disabled=!canSend('radar get')||exchange.busy;r.apply.disabled=r.key==='grid_version'||!canSend('radar set threshold 3')||exchange.busy;}}
  root.append(section);update();return {update,cancel(){for(const r of rows)r.current.textContent='设备值：历史，请主动读回';update();}};
}

export function mountRadarWorkspace({model,exchange,send,canSend,api,mapView,onBusy=()=>{},onLiveState=()=>{},onTabChange=()=>{},onResultsCleared=()=>{},monitorResult=new MonitorResult()}){
  let tab='monitor',offline=importSnapshot({mask:0x00050140,counts:Array(25).fill(0),coordinate_frame:'current'}),offlinePath=null,profileBusy=false,offlineDirty=false,focusedSegment=null,forceAll=false,offlineChoice='fixed';
  let transfer=null,scanActivity=null,selectionActivity=null,pendingResult=null,cancelledCapture=null,scheduled=false,internalCancel=false;
  let manualStart=null,headingOutcome=null,headingNeedsScan=false,headingRenderKey=null;
  let diagnostic=null,diagnosticScheduled=false;
  const live=new RadarLiveModel();let liveView='field',liveRange=4000,liveRequested=false,liveMessage='';
  const headingDrafts={upper:null,lower:null};
  const captureKey=f=>f?`${f.session}:${f.origin}:${f.scan}:${f.map}`:null;
  const resultKey=f=>f?`${captureKey(f)}:${f.plan}`:null;
  const sameResult=target=>model.status?.state==='ready'&&resultKey(model.status)===resultKey(target);
  const optional=kind=>['points','cloud','stable','diagnostic'].includes(kind);
  const scanWire=wire=>/^radar (?:debug )?scan(?: auto| upper| lower)?$/.test(wire)||/^radar scan (upper|lower) (?:1|3|15)$/.test(wire);
  let continuousRequested=false,stoppingContinuous=false;
  const continuous=()=>model.connected&&!!(continuousRequested||model.status?.live_active&&model.status?.mapping);
  const interactionBusy=()=>!!(transfer||exchange.busy||scanActivity||selectionActivity||live.running||liveRequested||continuous()||['preparing','wait_wrap','collecting'].includes(model.status?.state));
  const canClearResults=()=>!interactionBusy()&&!!(monitorResult.pair||['map','path','points','cloud','stable'].some(kind=>model.snapshots[kind]));
  function clearResults(){
    if(!canClearResults())return false;
    model.clearResults();monitorResult.pair=null;focusedSegment=null;forceAll=false;
    $('radar-transfer').textContent='本机历史点云、扫描障碍和雷达路线已清除';
    render();onResultsCleared();return true;
  }
  const paramsCurrent=()=>model.connected&&model.paramsLive&&model.snapshots.params?.session===model.identity?.session;
  const geometrySettings=(corner,angle)=>corner==='upper'?[]:[['zero_deg',angle]];
  const draftMatches=()=>manualStart&&headingDrafts[manualStart]!==null&&paramsCurrent()&&geometrySettings(manualStart,headingDrafts[manualStart]).every(([key,value])=>Number.isFinite(model.params[key])&&Math.abs(model.params[key]-value)<1e-5);
  const selectedScanAllowed=()=>manualStart&&!interactionBusy()&&canSend(`radar scan ${manualStart}`)&&(manualStart==='upper'||headingDrafts[manualStart]===null||draftMatches());
  const currentStart=()=>{const start=model.snapshots.start,state=model.status;return start&&!start.historical&&model.connected&&state&&['session','origin','scan'].every(k=>start[k]===state[k])?start:null;};
  const reusableCircle=()=>currentStart()?.auto===true&&model.status?.state==='ready';
  const unresolvedStart=()=>currentStart()?.selected===0||['start_ambiguous','start_both_conflict'].includes(model.status?.reason);
  function executionGate(){
    const start=currentStart(),state=model.status,path=model.snapshots.path;
    if(unresolvedStart())return radarReasonText('start_unresolved');
    if(start?.selected===1||path?.start===1&&state&&['session','origin','map','plan'].every(k=>path[k]===state[k]))return radarReasonText('upper_start_plan_only');
    return '';
  }
  const annotations=mountRouteAnnotations({root:$('route-annotations'),onSelect:number=>{forceAll=false;focusedSegment=number;render();}});
  for(const id of ['route-show-numbers','route-show-arrows'])$(id).onchange=()=>render();
  $('route-show-all').onclick=()=>{forceAll=tab==='monitor'?!forceAll:true;focusedSegment=null;render();};
  $('offline-copy-speed').onclick=()=>{$('route-preview-speed').value=$('radar-nav-speed').value;$('route-preview-speed').dispatchEvent?.(new Event('change'));invalidateOfflinePlan();};
  $('route-preview-speed').addEventListener('change',invalidateOfflinePlan);
  $('route-preview-speed').addEventListener('input',invalidateOfflinePlan);
  $('monitor-capture').onchange=()=>render();
  const selectedPlan=()=>tab==='offline'?offlinePath:null;
  const selectedProfile=()=>selectedPlan()?.route_profiles?.[$('route-profile-choice').value||'fixed'];
  function renderProfile(){
    const plan=selectedPlan(),profile=selectedProfile(),playable=!!profile?.phases?.length;$('route-profile-start').disabled=!playable;
    $('route-profile-source').textContent=`离线地图副本 · ${offlineDirty?'输入已修改，以下为上次计算结果':'与实车设置独立'}`;
    $('route-profile-plan').disabled=profileBusy;
    $('route-profile-start').disabled=!playable||(tab==='offline'&&offlineDirty);
    if(tab==='offline')mapView.setPreviewSource?.(playable?profile:null,{mask:plan?.mask??0,dirty:offlineDirty});
    $('route-profile-grid').textContent=plan?(plan.valid?'C 网格可达 · 车体外框另行检查':`C 网格不可达 · 第 ${plan.failed_leg} 路段`):'C 网格尚未计算';
    $('route-profile-phases').replaceChildren(...(profile?.phases??[]).map(p=>{const li=element('li');li.dataset.segment=String(p.segment);const text=p.kind==='turn'?profilePhaseText(p).replace(/上限 [\d.]+ mm\/s/,`转向上限 ${profile.turn_speed_deg_s}°/s`):profilePhaseText(p);const b=element('button',text);b.type='button';b.className='route-phase-select';b.onclick=()=>{focusedSegment=p.segment;render();};li.append(b);li.textContent=text;li.replaceChildren(b);return li;}));
    if(profile&&!playable&&plan?.plan_only){$('route-profile-status').textContent='上起点仅显示 C 几何路径，暂无可播放的运动阶段';return;}
    if(!profile){$('route-profile-status').textContent='点击本机 C 核心重新规划，读取当前固件阶段规则';return;}
    try{const preview=new RouteProfileRunner(profile,{mask:plan.mask??0});preview.advance(100000);$('route-profile-status').textContent=profileResultText(preview)+` · 计划终点 (${profile.phases.at(-1).target.join(', ')}) · 预演${preview.issue?'中止':'最终停止'} X ${preview.pose.x_mm.toFixed(1)} / Y ${preview.pose.y_mm.toFixed(1)} mm / θ ${(preview.pose.yaw_rad*180/Math.PI).toFixed(1)}°`;}
    catch(error){$('route-profile-status').textContent=error.message;}
  }
  $('route-profile-choice').onchange=()=>{offlineChoice=$('route-profile-choice').value;focusedSegment=null;renderProfile();render();};
  $('route-profile-start').onclick=()=>{if(tab!=='offline')return;const profile=selectedProfile(),plan=selectedPlan();if(profile?.phases?.length&&!offlineDirty)mapView.startProfile(profile,{mask:plan.mask??0});};
  $('route-profile-plan').onclick=async()=>{
    if(tab==='offline')await $('radar-offline-plan').onclick();
  };
  const map=$('field-map'),world=$('map-world');
  map.setAttribute('viewBox',`${INITIAL_VIEW.x} ${INITIAL_VIEW.y} ${INITIAL_VIEW.width} ${INITIAL_VIEW.height}`);
  map.addEventListener('pointermove',e=>{const p=new DOMPoint(e.clientX,e.clientY).matrixTransform(world.getScreenCTM().inverse());$('map-cursor').textContent=`鼠标 X ${p.x.toFixed(1)} · Y ${p.y.toFixed(1)} mm`;});
  function selectTab(value){
    tab=value;$('map-page').dataset.mapTab=tab;for(const b of $('map-tabs').children){const active=b.dataset.mapTab===tab;b.classList.toggle('active',active);b.setAttribute('aria-selected',String(active));}
    $('map-workspace-title').textContent=({monitor:'实车监控',radar:'雷达建图',offline:'离线对照'})[tab];
    $('radar-build-panel').hidden=tab!=='radar';$('radar-offline-panel').hidden=tab!=='offline';$('radar-monitor-panel').hidden=tab!=='monitor';
    $('route-plan-panel').hidden=tab!=='offline';
    for(const toolbar of document.querySelectorAll('.radar-live-toolbar'))toolbar.hidden=tab!=='radar';
    $('radar-live-age').hidden=tab!=='radar';
    $('route-profile-choice').value=offlineChoice;
    $('route-plan-title').textContent='2 · 设置方案与检查路线';
    $('route-profile-plan').textContent='计算路线';
    $('route-profile-start').textContent='播放方案 · 仅本机';
    $('route-preview-speed-field').hidden=tab!=='offline';$('offline-copy-speed').hidden=tab!=='offline';
    $('route-plan-help').textContent='预演速度独立保存，不改变实车启动速度；修改后重新计算。搬运任务不计取放耗时。';
    $('profile-play-controls').hidden=tab!=='offline';$('map-experiments').hidden=tab!=='offline';
    $('radar-offline-panel').append($('route-plan-panel'));
    focusedSegment=null;forceAll=false;renderProfile();
    const offlineTab=tab==='offline';
    mapView.setSource(offlineTab?'offline':'feedback');
    for(const id of ['real-route-panel','online-controls'])$(id).hidden=tab!=='monitor';for(const id of ['sim-controls','simulation-actions','route-editor','speed-lab-note'])$(id).hidden=!offlineTab;render();onTabChange(tab);
  }
  $('map-tabs').onclick=e=>{const b=e.target.closest('[data-map-tab]');if(b){selectTab(b.dataset.mapTab);if(globalThis.location)location.hash='map/'+b.dataset.mapTab;}};
  for(const corner of ['upper','lower'])$(`radar-select-${corner}`).onclick=()=>{if(interactionBusy())return;manualStart=corner;headingOutcome=null;render();};
  function chooseHeading(angle){if(interactionBusy()||manualStart==='upper')return;headingDrafts[manualStart]=angle;headingOutcome=null;render();Array.from($('radar-heading-options').children).find(child=>child.dataset.worldAngle===String(angle))?.focus?.();}
  function renderHeading(){
    const busy=interactionBusy(),draft=manualStart?headingDrafts[manualStart]:null,known=manualStart==='upper'?270:manualStart?radarWorldFromLowerReference('lower',model.params.zero_deg):null,live=paramsCurrent();
    const source=known===null?'设备方向参数未读取':`${live?'设备参数读回':'历史参数读回'}：地图 ${known}°（zero_deg ${model.params.zero_deg}°）`;
    const corner=manualStart==='upper'?'右上':'右下';
    let text=manualStart==='upper'?'右上独立固定安装 (2250,2250) · 0°向下270° · 单位矩阵 · 选择起点不会修改下方设备参数':manualStart?`${corner}当前安装 (${model.params.lidar_x_mm??2280},${model.params.lidar_y_mm??314}) · ${source}；箭头仅编辑0°草稿，不重置位置/矩阵/筛选`:'先选择右上或右下起点，再编辑下方雷达0°草稿';
    if(draft!==null){
      const outcome=headingOutcome?.corner===manualStart&&headingOutcome.world===draft?headingOutcome.text:null;
      text=`${corner} · 雷达0°草稿 ${draft}° · ${source} · ${outcome??(draftMatches()?'草稿与设备读回一致 · 请新扫描':'待应用 · 应用后需新扫描')}`;
      if(outcome&&headingOutcome.confirmed&&!draftMatches())text=`${corner} · 雷达0°草稿 ${draft}° · ${source} · 待应用 · 请读回或重新应用后新扫描`;
    }
    if(transfer?.kind==='heading')text+=' · 应用并读回中…';
    const sectorZero=draft??known??(manualStart==='upper'?270:180),sectorEnd=((sectorZero-90)%360+360)%360;
    const inward=[sectorZero,sectorZero-45,sectorZero-90].every(angle=>{const radians=angle*Math.PI/180;return Math.cos(radians)<=1e-8&&(manualStart==='upper'?Math.sin(radians)<=1e-8:Math.sin(radians)>=-1e-8);});
    if(manualStart)text+=' · 建图原始角0°～360°；阴影仅表示场内方向，不是筛选范围';
    $('radar-heading-status').textContent=text;
    for(const choice of ['upper','lower']){const button=$(`radar-select-${choice}`);button.disabled=busy;button.setAttribute('aria-pressed',String(choice===manualStart));button.classList.toggle('active',choice===manualStart);}
    $('radar-heading-apply').textContent='应用本次建图设置';
    $('radar-heading-apply').disabled=!manualStart||manualStart==='upper'||draft===null||busy||!geometrySettings(manualStart,draft).every(([key,value])=>canSend(`radar set ${key} ${value}`));
    $('radar-scan-selected').disabled=!selectedScanAllowed();
    const layer=$('radar-heading-options');layer.style.display=tab==='radar'&&manualStart?'':'none';
    const key=`${manualStart}:${draft}:${known}:${busy}`;if(headingRenderKey===key)return;headingRenderKey=key;layer.replaceChildren();
    if(!manualStart)return;
    const x=manualStart==='upper'?2250:(model.params.lidar_x_mm??2280),y=manualStart==='upper'?2250:(model.params.lidar_y_mm??314),angles=manualStart==='upper'?[270]:[90,180];
    const radial=angle=>{const radians=angle*Math.PI/180;return [Number((x+900*Math.cos(radians)).toFixed(3)),Number((y+900*Math.sin(radians)).toFixed(3))];};
    const from=radial(sectorZero),to=radial(sectorZero-90),color=inward?'#18806d':'#bc741d';
    layer.append(svg('path',{d:`M${x} ${y} L${from[0]} ${from[1]} A900 900 0 0 0 ${to[0]} ${to[1]} Z`,fill:inward?'#22a58622':'#d88b2222',stroke:color,'stroke-width':7,'pointer-events':'none','data-scan-sector':'true','data-zero-deg':sectorZero,'data-end-deg':sectorEnd,'data-inward':inward}));
    layer.append(svg('text',{transform:`translate(${to[0]} ${to[1]}) scale(1 -1)`,'text-anchor':'middle','font-size':42,fill:color,'pointer-events':'none'},'顺时针90°边界'));
    layer.append(svg('circle',{cx:x,cy:y,r:27,fill:'#fff',stroke:'#e89025','stroke-width':10}));
    for(const angle of angles){
      const label=angle===90?'向上':angle===270?'向下':'向左',selected=draft===angle;
      const button=svg('g',{class:'radar-heading-choice',transform:`translate(${x} ${y}) rotate(${angle})`,role:'button',tabindex:busy||manualStart==='upper'?-1:0,'aria-label':`雷达0°${label}，地图${angle}度，${manualStart==='upper'?'独立固定方向':'只选择草稿'}`,'aria-pressed':selected,'aria-disabled':busy||manualStart==='upper','data-world-angle':angle});button.dataset.worldAngle=String(angle);
      button.append(svg('rect',{x:55,y:-125,width:370,height:250,rx:35,fill:'transparent',class:'radar-heading-hit'}),svg('path',{d:'M90 -38 L270 -38 L270 -85 L385 0 L270 85 L270 38 L90 38 Z',class:'radar-heading-arrow'}));
      button.onclick=()=>chooseHeading(angle);button.addEventListener('keydown',event=>{if(['Enter',' '].includes(event.key)){event.preventDefault();chooseHeading(angle);}});layer.append(button);
      const radians=angle*Math.PI/180,text=svg('text',{transform:`translate(${x+470*Math.cos(radians)} ${y+470*Math.sin(radians)}) scale(1 -1)`,'text-anchor':'middle','dominant-baseline':'middle',class:'radar-heading-label'});text.textContent=`0°${label}`;layer.append(text);
    }
  }
  $('radar-heading-apply').onclick=()=>{
    const corner=manualStart,angle=headingDrafts[corner],settings=geometrySettings(corner,angle);
    if(!corner||corner==='upper'||angle===null||interactionBusy()||!settings.every(([key,value])=>canSend(`radar set ${key} ${value}`)))return;
    void operate('heading',async operation=>{
      onBusy(true);
      try{
        let confirmed=true;
        for(const[key,value]of settings){const result=await exchange.set(key,value);checkRead(operation);confirmed=confirmed&&result.confirmed;}
        confirmed=confirmed&&settings.every(([key,value])=>Number.isFinite(model.params[key])&&Math.abs(model.params[key]-value)<1e-5);
        headingOutcome={corner,world:angle,confirmed,text:confirmed?'0°方向已读回一致 · 位置、矩阵与筛选保留 · 请新扫描':'方向读回不一致 · 草稿保留，请重新应用后新扫描'};
      }catch(error){headingOutcome={corner,world:angle,confirmed:false,text:`应用未确认 · 草稿保留，需应用后新扫描 · ${error.message}`};}
      finally{onBusy(false);}
    });
  };
  $('radar-scan-selected').onclick=()=>{if(selectedScanAllowed())requestWire(`radar scan ${manualStart}`);};
  const liveStartAllowed=()=>manualStart&&!interactionBusy()&&canSend(`radar live start ${manualStart}`)&&(headingDrafts[manualStart]===null||draftMatches());
  $('radar-live-start').onclick=()=>{
    if(!liveStartAllowed())return;
    liveRequested=true;liveMessage='实时点云已请求 · 等待设备受理';render();
    void Promise.resolve(send(`radar live start ${manualStart}`)).then(ok=>{if(ok!==true){liveRequested=false;liveMessage='实时点云写出未确认';render();}});
  };
  $('radar-live-stop').onclick=()=>{if(!canSend('radar live stop'))return;liveMessage='已请求停止实时点云 · 等待设备状态';void send('radar live stop');renderLive();};
  $('radar-live-view').value=liveView;$('radar-live-range').value=String(liveRange);
  $('radar-live-view').onchange=()=>{liveView=$('radar-live-view').value==='center'?'center':'field';renderLive();};
  $('radar-live-range').onchange=()=>{const range=Number($('radar-live-range').value);if([1000,2000,4000,8000].includes(range))liveRange=range;renderLive();};
  function renderLive(){
    const displayed=live.displayed(),pose=displayed?.pose??(live.state?.id?live.state.pose:null),center=liveView==='center',padding=liveRange*1.08;
    if(center){$('field-map').setAttribute('hidden','');$('radar-live-map').removeAttribute('hidden');}
    else{$('field-map').removeAttribute('hidden');$('radar-live-map').setAttribute('hidden','');}
    $('radar-live-range').disabled=!center;
    $('radar-live-map').setAttribute('viewBox',`${-padding} ${-padding} ${padding*2} ${padding*2}`);
    const overlay=$('radar-live-points'),points=$('radar-live-center-points');overlay.replaceChildren();points.replaceChildren();
    const historical=displayed?.historical||!live.connected||!live.state?.active;
    overlay.style.opacity=points.style.opacity=historical?'.5':'1';overlay.setAttribute('data-preview',String(displayed?.complete===false));
    if(displayed)for(const point of displayed.points){const projected=projectRadarPoint(point,pose);if(!projected)continue;overlay.append(svg('circle',{cx:projected[0],cy:projected[1],r:13,fill:'#e50073',stroke:'#fff','stroke-width':3}));points.append(svg('circle',{cx:projected[0]-pose[0],cy:projected[1]-pose[1],r:liveRange/160,fill:'#ff5da8',stroke:'#fff','stroke-width':liveRange/650}));}
    const grid=$('radar-live-center-grid');grid.replaceChildren();
    for(let i=1;i<=4;i++){const radius=liveRange*i/4;grid.append(svg('circle',{cx:0,cy:0,r:radius,fill:'none',stroke:'#476879','stroke-width':liveRange/500}),svg('text',{transform:`translate(${liveRange*.025} ${radius-liveRange*.025}) scale(1 -1)`,fill:'#c6dfeb','font-size':liveRange*.038},`${radius/1000} m`));}
    grid.append(svg('path',{d:`M ${-liveRange} 0 H ${liveRange} M 0 ${-liveRange} V ${liveRange}`,fill:'none',stroke:'#94b8c9','stroke-width':liveRange/400,'stroke-dasharray':`${liveRange/55} ${liveRange/55}`}),svg('text',{transform:`translate(${liveRange*.84} ${liveRange*.055}) scale(1 -1)`,fill:'#e9f4fa','font-size':liveRange*.05},'+X'),svg('text',{transform:`translate(${liveRange*.025} ${liveRange*.9}) scale(1 -1)`,fill:'#e9f4fa','font-size':liveRange*.05},'+Y'));
    const heading=$('radar-live-center-heading'),fieldHeading=$('radar-live-field-heading');heading.replaceChildren();fieldHeading.replaceChildren();
    if(pose){
      const length=liveRange*.3;heading.setAttribute('transform',`rotate(${pose[2]})`);
      heading.append(svg('circle',{cx:0,cy:0,r:liveRange*.016,fill:'#ffe468'}),svg('path',{d:`M 0 0 H ${length} M ${length-liveRange*.05} ${-liveRange*.03} L ${length} 0 L ${length-liveRange*.05} ${liveRange*.03}`,stroke:'#ffe468',fill:'none','stroke-width':liveRange*.007}));
      const radians=pose[2]*Math.PI/180,label=svg('text',{transform:`translate(${(length+liveRange*.07)*Math.cos(radians)} ${(length+liveRange*.07)*Math.sin(radians)}) scale(1 -1)`,fill:'#ffe468','text-anchor':'middle','font-size':liveRange*.055});label.textContent='0°';grid.append(label);
      fieldHeading.append(svg('path',{d:'M 0 0 H 260 M 205 -35 L 260 0 L 205 35',transform:`translate(${pose[0]} ${pose[1]}) rotate(${pose[2]})`,stroke:'#ba0060',fill:'none','stroke-width':9}));
    }
    const description=live.describe();$('radar-live-status').textContent=liveMessage?`${liveMessage} · ${description}`:description;
    $('radar-live-age').textContent=displayed?`${description} · ${pose?'冻结名义位姿 ('+pose.join(', ')+')，不表示实测定位':''}`:'实时点云与单圈地图、诊断快照独立；开始后更新，不自动运动。';
    $('radar-live-start').disabled=!liveStartAllowed();$('radar-live-stop').disabled=!canSend('radar live stop');
  }
  const diagnosticWire=()=>`radar debug scan ${manualStart??'auto'}`;
  const diagnosticAllowed=()=>!interactionBusy()&&canSend(diagnosticWire())&&(!manualStart||headingDrafts[manualStart]===null||draftMatches());
  $('radar-debug-scan').onclick=()=>{
    if(!diagnosticAllowed())return;
    diagnostic={current:true,phase:'scanning',baseline:model.rawStatus?.id??null,id:null,metadata:null,bundle:null,message:'诊断静止扫描一圈中 · 完成后下载同圈完整点云和原始包'};
    requestWire(diagnosticWire());
  };
  $('radar-debug-download').onclick=()=>{
    if(interactionBusy())return;
    const metadata=model.rawStatus;
    if(!metadata||!['ready','error','failed','stopped'].includes(metadata.state))return;
    diagnostic={current:true,phase:'waiting',id:metadata.id,metadata:{...metadata},bundle:null,message:'准备下载本次诊断 · 完整前不能导出'};render();
  };
  function saveSnapshot(snapshot,prefix){const url=URL.createObjectURL(new Blob([JSON.stringify(snapshot,null,2)],{type:'application/json'})),a=element('a');a.href=url;a.download=`${prefix}-${Date.now()}.json`;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  $('radar-debug-export').onclick=()=>{if(diagnostic?.bundle)saveSnapshot(diagnostic.bundle,'radar-diagnostic');};
  async function operate(kind,work,{propagate=false}={}){
    if(transfer||exchange.busy){if(propagate)throw new Error('当前结果读取仍在进行，请等待完成');return;}
    const operation={kind};operation.done=new Promise(resolve=>{operation.finish=resolve;});transfer=operation;render();
    try{await work(operation);}catch(error){if(!operation.obsolete)$('radar-transfer').textContent=error.message;if(propagate)throw error;}
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
      if(!canSend(`radar result ${target.plan}`))return;
      pendingResult=null;
      void operate('result',async operation=>{
        await exchange.result(target);
        checkRead(operation);
        if(!sameResult(target)&&!continuous())throw new Error('新扫描已开始，已完成历史地图保留');
        const path=model.snapshots.path;
        $('radar-transfer').textContent=path?.valid===false?`地图已显示 · 第 ${path.failed_leg} 路段不可达`:'地图与规划路径已显示 · 点云可按需读取';
        if(model.snapshots.map?.algorithm===1&&model.snapshots.stable?.scan===target.scan){$('radar-show-points').checked=true;$('radar-transfer').textContent+=' · 稳定XY已显示';}
      });
    });
  }
  function scheduleDiagnostic(){
    if(diagnosticScheduled||diagnostic?.phase!=='waiting'||pendingResult||transfer||exchange.busy)return;
    diagnosticScheduled=true;
    queueMicrotask(()=>{
      diagnosticScheduled=false;
      const job=diagnostic;
      if(job?.phase!=='waiting'||pendingResult||transfer||exchange.busy||!canSend(`radar fetch raw ${job.id} 0`))return;
      job.phase='downloading';
      void operate('diagnostic',async operation=>{
        try{
          const target=job.metadata;
          if(target.complete){
            if(target.map){
              if(captureKey(model.snapshots.map)!==captureKey(target))await exchange.fetch('map',target.map);
              checkRead(operation);
              if(captureKey(model.snapshots.path)!==captureKey(target)||model.snapshots.path.plan!==model.snapshots.map.plan)await exchange.fetch('path',model.snapshots.map.plan);
              checkRead(operation);
            }
            if(captureKey(model.snapshots.cloud)!==captureKey(target)||!model.snapshots.cloud.complete){
              if(model.status?.session!==target.session||model.status?.scan!==target.scan||!model.status?.points_valid)throw new Error('设备已开始新圈，本次诊断点云不可下载；已完成历史诊断仍保留');
              await exchange.fetch('cloud',target.scan);checkRead(operation);
            }
          }
          await exchange.fetch('raw',job.id);checkRead(operation);
          if(captureKey(model.snapshots.raw)!==captureKey(target))throw new Error('设备诊断记录已更新，请重新读取诊断状态');
          job.bundle=diagnosticSnapshot(model,job.id);job.phase='complete';
          job.message=job.bundle.snapshots.map?'同圈地图、路径、完整点云和原始包已完整下载 · 可导出诊断':job.bundle.snapshots.cloud?'起点未确定 · 同圈完整点云和原始包已完整下载 · 可导出诊断':job.metadata.state==='ready'?'采集完成，原始记录未覆盖完整圈，不能完整重放 · 可导出部分原始诊断':'原始诊断已下载 · 本次未完成有效圈，仅导出原始诊断';
          $('radar-transfer').textContent=job.message;
        }catch(error){job.phase='failed';job.message=`诊断下载未完成 · ${error.message} · 可重新下载`;throw error;}
      });
    });
  }
  exchange.subscribeState(({type})=>{
    if(type==='cancel')model.pending.clear();
    if(type==='cancel'&&!internalCancel){if(transfer)transfer.cancelled=true;pendingResult=null;cancelledCapture=scanActivity?.key??null;scanActivity=null;selectionActivity=null;if(diagnostic&&!diagnostic.bundle){diagnostic.phase='failed';diagnostic.message='诊断已取消 · 已完成历史数据保留；未完整下载，不能导出本次诊断';}}
    if(type==='idle')scheduleResult();
  });
  function observeScan(f){
    if(!['status','diag','progress'].includes(f.k))return;
    if(f.k==='progress'){continuousRequested=false;return;}
    if(stoppingContinuous)return;
    if(continuous()&&['preparing','wait_wrap','collecting'].includes(f.state))return;
    if(continuous()&&f.state==='ready'&&f.map&&f.plan&&resultKey(f)!==resultKey(model.snapshots.map)){pendingResult={...f};scanActivity=null;scheduleResult();return;}
    if(['preparing','wait_wrap','collecting'].includes(f.state)){
      const key=captureKey(f);if(cancelledCapture===key)return;
      selectionActivity=null;
      if(!scanActivity){pendingResult=null;if(transfer&&!optional(transfer.kind))supersedeRead('新扫描开始，旧结果读取已取消');scanActivity={key,observed:true};}
      else{if(scanActivity.key===null)scanActivity.key=key;scanActivity.observed=true;}
      return;
    }
    if(['stopped','error'].includes(f.state)){scanActivity=null;selectionActivity=null;pendingResult=null;cancelledCapture=null;if(transfer?.kind==='result')supersedeRead('扫描已结束，保留历史结果');return;}
    if(f.state==='ready'&&selectionActivity){
      const start=currentStart(),request=selectionActivity;
      if(start?.reason==='manual_selection'&&start.selected===request.selected&&['session','scan'].every(k=>f[k]===request[k])&&resultKey(f)!==request.key){
        selectionActivity=null;pendingResult={...f};$('radar-transfer').textContent='起点已选择 · 复用本圈下载地图与路径';scheduleResult();
      }
      return;
    }
    if(f.state!=='ready'||!scanActivity||scanActivity.key===null||scanActivity.explicit&&!scanActivity.accepted&&!scanActivity.observed)return;
    if(captureKey(f)===scanActivity.key)return;
    scanActivity=null;
    if(!f.map||!f.plan||f.applicable===false||f.applicable===0)return;
    headingNeedsScan=false;
    pendingResult={...f};$('radar-transfer').textContent='采集与规划已完成 · 等待读取地图与路径';
    if(optional(transfer?.kind))supersedeRead('新地图已完成，优先读取地图与路径');scheduleResult();
  }
  function observeScanReply(line){
    const reply=line.trim().replace(/^(?:arm>\s*)+/,'');
    if(selectionActivity&&reply.startsWith('ERR radar ')){selectionActivity=null;$('radar-transfer').textContent=`起点选择未受理 · ${radarReasonText(reply.split(' ')[2])}`;return true;}
    if(!scanActivity?.explicit)return false;
    if(reply==='OK radar scan_accepted'||reply==='OK radar debug_scan_accepted'){scanActivity.accepted=true;return true;}
    if(/^ERR radar (?:stationary_capture_required|query_in_progress|scan_busy_or_parameters)$/.test(reply)){scanActivity=null;if(diagnostic?.phase==='scanning'){diagnostic.phase='failed';diagnostic.message=`诊断扫描未受理 · ${radarReasonText(reply.split(' ')[2])}`;}$('radar-transfer').textContent=`扫描未受理 · ${radarReasonText(reply.split(' ')[2])}`;return true;}
    return false;
  }
  exchange.onProgress=({kind,page,pages})=>{const label=({map:'地图',path:'规划路径',points:'可选显示点云',cloud:'完整点云',raw:'原始包'})[kind]??kind;$('radar-transfer').textContent=`下载 ${label} · ${page}/${pages} 页 · 完整后显示`;if(diagnostic?.phase==='downloading'){diagnostic.message=`下载同圈 ${label} · ${page}/${pages} 页 · 全部完成后可导出`;$('radar-debug-status').textContent=diagnostic.message;}};
  function requestWire(wire){
    if(!canSend(wire)||scanWire(wire)&&interactionBusy()||/^radar start (upper|lower)$/.test(wire)&&(interactionBusy()||!reusableCircle()))return;
    if(scanWire(wire)){if(diagnostic&&!wire.startsWith('radar debug ')){diagnostic.current=false;if(!diagnostic.bundle)diagnostic.phase='historical';}cancelledCapture=null;pendingResult=null;selectionActivity=null;scanActivity={key:captureKey(model.status),explicit:true,accepted:false,observed:false};$('radar-transfer').textContent='已请求静止扫描 · 完成后自动显示地图与路径';}
    if(/^radar start (upper|lower)$/.test(wire)){selectionActivity={...model.status,key:resultKey(model.status),selected:wire.endsWith('upper')?1:2};pendingResult=null;$('radar-transfer').textContent='已请求选择起点 · 复用完整圈重新建图与规划';}
    if(wire==='radar stop')exchange.cancel();
    return Promise.resolve(send(wire)).then(ok=>{if(scanWire(wire)&&ok!==true){scanActivity=null;if(diagnostic?.phase==='scanning'){diagnostic.phase='failed';diagnostic.message='诊断扫描写出未确认，请重新扫描';}}if(/^radar start (upper|lower)$/.test(wire)&&ok!==true)selectionActivity=null;render();return ok;});
  }
  for(const b of document.querySelectorAll('[data-radar-wire]'))b.onclick=()=>requestWire(b.dataset.radarWire);
  const currentId=kind=>kind==='path'?(model.status?.plan??model.snapshots.path?.plan):['points','cloud','stable'].includes(kind)?(model.status?.scan??model.snapshots.map?.scan):(model.status?.map??model.snapshots.map?.map);
  for(const kind of ['map','path','points','cloud'])$(`radar-fetch-${kind}`).onclick=async()=>{
    if(optional(kind)&&unresolvedStart()){$('radar-transfer').textContent='起点未确定，请先人工选择后读取本圈点云';return;}
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
  const start=kind=>{const gate=executionGate();if(gate){$('radar-execution-reason').textContent=gate;return;}const speed=Number($('radar-nav-speed').value);if(!Number.isInteger(speed)||speed<10||speed>5000){$('radar-transfer').textContent='速度范围10～5000 mm/s';return;}void send(`radar ${kind} start ${speed}`);};$('radar-nav-start').onclick=()=>start('nav');$('radar-task-start').onclick=()=>start('task');
  for(const id of ['radar-show-obstacles','radar-show-points','radar-show-counts'])$(id).onchange=render;
  function invalidateOfflinePlan(){offlineDirty=true;$('radar-offline-status').textContent='输入已修改，保留上次结果供对照；请重新计算路线';renderProfile();render();}
  const offlineInputs={};for(const p of RADAR_PARAMETERS.slice(0,10)){const label=element('label',p.label),input=element('input');Object.assign(input,{type:'number',value:p.value,min:p.min,max:p.max,step:p.key==='zero_deg'?'.1':'1'});label.append(input);$('radar-offline-params').append(label);offlineInputs[p.key]=input;input.onchange=invalidateOfflinePlan;}
  function setOffline(snapshot){
    if(snapshot.offline_settings){const settings=snapshot.offline_settings;if(Number.isInteger(settings.speed_mm_s)&&settings.speed_mm_s>=10&&settings.speed_mm_s<=5000){$('route-preview-speed').value=String(settings.speed_mm_s);$('route-preview-speed').dispatchEvent?.(new Event('change'));}if(['fixed','planned','task_fixed','task_planned','task_fixed_cosine','task_planned_cosine','task_fixed_smooth','task_planned_smooth'].includes(settings.choice)){offlineChoice=settings.choice;$('route-profile-choice').value=offlineChoice;}}
    offline=importSnapshot(snapshot);offline.loaded_from_snapshot=true;offlinePath=null;offlineDirty=false;focusedSegment=null;const params={...Object.fromEntries(RADAR_PARAMETERS.map(p=>[p.key,p.value])),...model.params,...offline.params};
    if(offline.pose){
      if(offline.start_id===1){const lower=[4500-offline.pose[0],2400-offline.pose[1],radarLowerReferenceFromWorld('upper',offline.pose[2])];for(const[key,index]of [['lidar_x_mm',0],['lidar_y_mm',1],['zero_deg',2]])params[key]=lower[index];}
      else{params.lidar_x_mm=offline.pose[0];params.lidar_y_mm=offline.pose[1];params.zero_deg=offline.pose[2];}
    }
    if(offline.start_id===1){offlineChoice='planned';$('route-profile-choice').value='planned';}
    for(const[key,input]of Object.entries(offlineInputs))input.value=String(params[key]);for(const input of $('radar-mask-cells').querySelectorAll('input'))input.checked=!!(offline.mask&(1<<Number(input.dataset.cell)));$('radar-offline-status').textContent=`${offline.start_id===1?'右上':'离线'}副本已载入；可改过滤或手工障碍后本机重新规划`;renderProfile();render();
  }
  for(let row=4;row>=0;row--)for(let col=0;col<5;col++){const label=element('label',`R${row+1}C${col+1}`),input=element('input');input.type='checkbox';input.dataset.cell=row*5+col;input.onchange=()=>{if(!offline)return;const bit=1<<Number(input.dataset.cell);offline.mask=input.checked?offline.mask|bit:offline.mask&~bit;offline.manual_mask=input.checked?(offline.manual_mask??0)|bit:(offline.manual_mask??0)&~bit;invalidateOfflinePlan();};label.prepend(input);$('radar-mask-cells').append(label);input.checked=!!(offline.mask&(1<<Number(input.dataset.cell)));}
  $('radar-use-default').onclick=()=>setOffline({mask:RADAR_FIXED_MASK,counts:Array(25).fill(0),coordinate_frame:'current'});
  $('radar-use-device').onclick=()=>{if(!model.snapshots.map){$('radar-offline-status').textContent='先读取完整设备地图';return;}setOffline({snapshots:model.snapshots,params:model.params});};
  $('radar-import').onchange=async()=>{const file=$('radar-import').files[0];if(!file)return;try{const value=JSON.parse(await file.text()),frame=$('radar-import-frame').value;if(frame!=='auto'){if(value.offline)value.offline.coordinate_frame=frame;if(value.snapshots?.map)value.snapshots.map.coordinate_frame=frame;else value.coordinate_frame=frame;}setOffline(value);}catch(error){$('radar-offline-status').textContent=error.message;}};
  $('radar-export').onclick=()=>{const snapshots=Object.fromEntries(Object.entries(model.snapshots).filter(([kind])=>kind!=='raw'));saveSnapshot({v:1,coordinate_frame:'current',snapshots,params:model.params,offline,offlinePath,offline_settings:{speed_mm_s:Number($('route-preview-speed').value),choice:offlineChoice}},'radar');};
  $('radar-offline-plan').onclick=async()=>{
    if(!offline){$('radar-offline-status').textContent='先载入完整地图副本';return;}const params=Object.fromEntries(Object.entries(offlineInputs).map(([key,input])=>[key,Number(input.value)])),useCloud=$('radar-offline-input').value==='cloud',cloud=offline.cloud;
    if(useCloud&&offline.algorithm===1){$('radar-offline-status').textContent='原始云仅最后一圈，不能重建15圈稳定窗口；请复用冻结稳定地图，或重新采集15圈。';return;}
    if(useCloud&&!isCompleteRadarCloud(cloud)){const reason=cloud?.truncated?'当前云已截断':cloud?.k==='points'?'当前云为显示抽样':cloud?.complete===false?'当前云未完整':cloud?.points?'当前云未声明完整':'尚无同圈完整点云';$('radar-offline-status').textContent=`${reason}，不能重建完整圈轮廓；请读取同圈完整点云，或按现有/人工障碍规划`;return;}
    const speed=Number($('route-preview-speed').value||200);if(!Number.isInteger(speed)||speed<10||speed>5000){$('radar-offline-status').textContent='启动速度范围10～5000 mm/s';return;}
    $('radar-offline-plan').disabled=true;$('route-profile-plan').disabled=true;$('radar-offline-status').textContent='本机C核心计算中…';try{offlinePath=await api('radar/plan',{mask:useCloud?(offline.manual_mask??0):offline.mask,params,points:useCloud?cloud.points:[],cloud_complete:useCloud&&isCompleteRadarCloud(cloud),start_mode:offline.start_mode??'manual',speed_mm_s:speed});offlineDirty=false;offline.params=params;offline.receivedAt=Date.now();$('radar-offline-status').textContent=offlinePath.valid?`${offlinePath.algorithm} · 离线预览；未发送实车指令`:`规划不可达 · 第 ${offlinePath.failed_leg} 路段`;renderProfile();render();}catch(error){$('radar-offline-status').textContent=error.message;}finally{$('radar-offline-plan').disabled=false;$('route-profile-plan').disabled=false;}
  };
  function render(){
    const startMarker=$('monitor-start-marker');startMarker.replaceChildren();
    if(tab==='monitor'){
      const upper=$('monitor-capture').value==='upper',y=upper?2250:150;
      startMarker.append(svg('circle',{cx:2250,cy:y,r:55,fill:'#fff1d6',stroke:'#e38a15','stroke-width':9}),svg('text',{transform:`translate(2180 ${y+(upper?-80:85)}) scale(1 -1)`,'text-anchor':'end','font-size':48,fill:'#b66d0f'},upper?'右上起点':'右下起点'));
    }
    const pair=monitorResult.update(model.snapshots.map,model.snapshots.path);
    const profile=selectedProfile(),geometryProfile=tab==='offline'?{points:offlinePath?.points??[]}:pair?.path;
    const realRoute=mapView.snapshot?.()?.route;
    const annotationProfile=profile??geometryProfile;
    const current=forceAll?null:tab==='monitor'&&profile&&!profile.phases?.some(p=>p.segment===realRoute?.segment)?null:tab==='monitor'?realRoute?.segment??null:null;
    annotations.render(annotationProfile,{selected:focusedSegment,current,online:tab==='monitor',numbers:$('route-show-numbers').checked,arrows:$('route-show-arrows').checked});
    $('route-show-all').textContent=tab==='monitor'&&forceAll?'跟随当前目标':'显示全部路段';
    $('route-focus-status').textContent=focusedSegment!==null?`正在查看 ${profile?'目标':'路点'} ${focusedSegment} · 点击重复编号可依次切换`:tab==='monitor'&&current?`当前目标 ${current}`:'全部路段 · 点击编号或阶段查看';
    $('offline-source-summary').textContent=`${offline.loaded_from_snapshot?'载入的地图副本':'默认场地'}${offline.map!==undefined?' · 地图 '+offline.map:''} · ${offline.start_id===1?'右上起点':offline.start_id===2?'右下起点':'手工场地'} · ${offlineDirty?'输入已修改，结果待重算':'仅作用于本机'}`;
    const mapSnapshot=tab==='monitor'?pair?.map:model.snapshots.map,path=tab==='monitor'?pair?.path:model.snapshots.path,state=model.status,nav=model.nav;
    const GRID=tab==='monitor'?monitorGrid(mapSnapshot):RADAR_GRID;
    mapView.setRadarGrid?.(GRID);
    const gate=executionGate(),scanning=!!scanActivity||['preparing','wait_wrap','collecting'].includes(state?.state),startSnapshot=model.snapshots.start,unresolved=unresolvedStart();
    $('radar-start-status').textContent=!scanning&&unresolved&&!currentStart()?`起点：未确定 · ${radarReasonText(state.reason)}`:radarStartText(startSnapshot,{pending:scanning,historical:!!startSnapshot&&(!currentStart()||scanning)});
    renderHeading();
    $('radar-map-age').textContent=model.describe(tab==='offline'?offline:mapSnapshot)+(headingNeedsScan?' · 建图参数已应用，保留历史图；请新扫描':state?.applicable===false||state?.applicable===0?(unresolved?' · 起点未确定，保留历史图；请人工选择':' · 参数/原点已变化，请重新扫描'):'')+(mapSnapshot?.coverage!==undefined?` · 覆盖 ${(mapSnapshot.coverage/10).toFixed(1)}°`:'');$('radar-path-age').textContent=`设备：${model.describe(path)}${offlinePath?'；虚线：本机离线预览':''}`;
    $('radar-status').textContent=state?.state==='ready'&&unresolved?`本圈采集完成 · ${radarReasonText(state.reason)} · 扫描 ${state.scan} · ${state.map?`显示历史地图 ${state.map} / 规划 ${state.plan}（不适用本圈）`:'尚未发布本圈地图和规划'}`:state?`采集与规划：${radarStateText(state.state)} · ${radarReasonText(state.reason)} · 扫描 ${state.scan} / 地图 ${state.map} / 规划 ${state.plan}`:'点击读取状态，确认设备雷达配置';$('radar-nav-status').textContent=nav?`${radarStateText(nav.state)} · ${radarReasonText(nav.reason)} · ${nav.index}/${nav.total} · ${STATIONS[nav.station]??'通行'} · 访问 ${nav.visit}${nav.state==='stopping'?((nav.stopped??nav.stop_confirmed)?' · 停车已确认':' · 停车待确认'):''}`:'尚无设备导航状态';
    $('radar-probe-status').textContent=radarProbeText(model.probe);
    $('radar-diagnostics').textContent=JSON.stringify({status:state,start:startSnapshot,nav,probe:model.probe},null,2);
    $('radar-execution-reason').textContent=gate|| (state?.map===0?'尚无设备地图，请静止扫描':state?.applicable===false||state?.applicable===0?'雷达标定或扫描参数已变化，请重新采集或重算地图':path?.plan===state?.plan&&path?.valid===false?`第 ${path.failed_leg} 路段不可达，请调整障碍或扫描过滤参数`:'地图与底盘准备分别检查；可先扫描后准备，执行前须有新鲜起点反馈');
    $('radar-param-summary').textContent=Object.keys(model.params).length?`雷达 (${model.params.lidar_x_mm},${model.params.lidar_y_mm}) mm · 零角 ${model.params.zero_deg}° · 阈值 ${model.params.threshold}`:'参数尚未读回 · 参数调试页主动读取';
    const obstacles=$('radar-obstacles');obstacles.replaceChildren();const mask=tab==='offline'?(offlinePath?.mask??offline?.mask??mapSnapshot?.mask??0):(mapSnapshot?.mask??0);if($('radar-show-obstacles').checked)for(let i=0;i<25;i++)if(mask&(1<<i)){const row=Math.floor(i/5),col=i%5,fixed=!!(RADAR_FIXED_MASK&(1<<i));obstacles.append(svg('rect',{x:GRID[col],y:GRID[row],width:GRID[col+1]-GRID[col],height:GRID[row+1]-GRID[row],fill:fixed?'#e3b94555':'#d9444466',stroke:fixed?'#a78322':'#c34343','stroke-width':4,'data-cell':i,'data-obstacle':fixed?'fixed':'scan'}));}
    const counts=$('radar-counts');counts.replaceChildren();const cellCounts=tab==='offline'?(offlinePath?.counts??offline?.counts):mapSnapshot?.counts,protectedMask=tab==='monitor'?monitorProtectedMask(mapSnapshot):0;if($('radar-show-counts').checked&&cellCounts)for(let i=0;i<25;i++){const row=Math.floor(i/5),col=i%5,x=(GRID[col]+GRID[col+1])/2,y=(GRID[row]+GRID[row+1])/2,protectedCell=!!(protectedMask&(1<<i)),label=svg('text',{transform:`translate(${x} ${y}) scale(1 -1)`,'text-anchor':'middle','font-size':protectedCell?38:48,fill:protectedCell?'#607c88':'#1b4d65'});label.textContent=String(cellCounts[i]??0)+(protectedCell?' · 保护':'');counts.append(label);}
    const historicalMap=!model.connected||mapSnapshot?.historical||state&&(state.applicable===false||state.applicable===0||state.map!==mapSnapshot?.map);obstacles.style.opacity=counts.style.opacity=tab!=='offline'&&historicalMap?'.4':'1';
    $('radar-device-path').setAttribute('d',path?.valid!==false?pathData(path?.points):'');$('radar-device-path').style.opacity=!model.connected||path?.historical||state&&(state.applicable===false||state.applicable===0||state.map!==path?.map||state.plan!==path?.plan)?'.4':'1';$('radar-offline-path').setAttribute('d',tab==='monitor'&&selectedProfile()?pathData(selectedProfile().points):tab==='offline'?(offlinePath?.plan_only&&$('route-profile-choice').value==='planned'?pathData(offlinePath.valid?offlinePath.points:[]):selectedProfile()?.points?pathData(selectedProfile().points):offlinePath?.valid?pathData(offlinePath.points):''):'');
    for(const li of $('route-profile-phases').children)li.classList.toggle('route-phase-focused',Number(li.dataset.segment)===(focusedSegment??current));
    if(focusedSegment!==null||current!==null){$('radar-device-path').style.opacity='.12';$('radar-offline-path').style.opacity='.12';}else $('radar-offline-path').style.opacity='1';
    $('route-annotations').style.opacity=tab==='monitor'&&!profile&&historicalMap?'.4':'1';
    const marks=$('radar-stations');marks.replaceChildren();if(path?.valid!==false&&path?.points)for(const p of path.points)if(p[2])marks.append(svg('circle',{cx:p[0],cy:p[1],r:20,fill:'#eaf9f3',stroke:'#008e64','stroke-width':5}));
    const cloudLayer=$('radar-cloud');cloudLayer.replaceChildren();
    const stable=tab==='offline'?offline?.stable:model.snapshots.stable;
    const frozen=tab==='offline'?offline:mapSnapshot??model.snapshots.map;
    const sameStable=stable&&['session','origin','scan','map'].every(k=>stable[k]===frozen?.[k]);
    const cloud=sameStable?stable:tab==='offline'?offline?.cloud:preferredRadarCloud(model.snapshots.cloud,model.snapshots.points);
    if($('radar-show-points').checked&&cloud?.points){
      const sameCapture=['session','origin','scan','map'].every(k=>cloud[k]===frozen?.[k]);
      const capture=sameCapture?{...frozen,...cloud}:cloud;
      for(const point of cloud.points){
        const projected=sameStable?stableFieldPoint(point):projectFrozenRadarPoint(point,capture);
        if(projected)cloudLayer.append(svg('circle',{cx:projected[0],cy:projected[1],r:7,fill:sameStable?'#008b70':'#1674a9'}));
      }
      cloudLayer.style.opacity=!model.connected||cloud.historical||state?.scan!==cloud.scan?'.3':'1';
    }
    $('radar-points-age').hidden=!$('radar-show-points').checked;$('radar-points-age').textContent=cloud?`${model.describe(cloud)} · ${cloud.points.length} 点${sameStable?'（15圈稳定XY）':cloud.k==='points'?'（preview显示抽样）':'（最后圈原始）'}${cloud.truncated?' · 单圈缓存已截断':''}`:'尚无已下载点云';
    for(const b of document.querySelectorAll('[data-radar-wire]'))b.disabled=!canSend(b.dataset.radarWire)||scanWire(b.dataset.radarWire)&&interactionBusy()||/^radar start (upper|lower)$/.test(b.dataset.radarWire)&&(interactionBusy()||!reusableCircle());
    for(const kind of ['map','path','points','cloud'])$(`radar-fetch-${kind}`).disabled=optional(kind)&&unresolved||!!(exchange.busy||transfer)&&!(!optional(kind)&&optional(transfer?.kind))||!canSend(`radar fetch ${kind} ${currentId(kind)??0} 0`);
    $('radar-debug-scan').disabled=!diagnosticAllowed();$('radar-debug-scan').textContent=`诊断扫描一圈 · ${manualStart==='upper'?'右上':manualStart==='lower'?'右下':'自动识别'}`;
    const raw=model.rawStatus;$('radar-debug-download').disabled=interactionBusy()||!raw||!['ready','error','failed','stopped'].includes(raw.state)||!canSend(`radar fetch raw ${raw?.id??0} 0`);
    $('radar-debug-export').disabled=!diagnostic?.bundle;$('radar-debug-export').textContent=diagnostic?.current===false?'导出上次诊断':'导出同圈诊断';$('radar-export').disabled=!!diagnostic?.current&&!diagnostic.bundle;
    $('radar-export-status').textContent=diagnostic?.current&&!diagnostic.bundle?'诊断下载未完成，导出暂不可用；雷达建图页可查看进度或重新下载。':'';
    $('radar-debug-status').textContent=diagnostic?(diagnostic.current===false?'上次诊断 · ':'')+diagnostic.message:'诊断扫描会记录原始包；完整点云和原始包下载完成后，才能导出同圈诊断。普通扫描不记录原始包。';
    $('radar-cancel-transfer').disabled=!(exchange.busy||transfer||pendingResult);$('radar-nav-start').disabled=!!gate||!canSend(`radar nav start ${$('radar-nav-speed').value}`);$('radar-task-start').disabled=!!gate||!canSend(`radar task start ${$('radar-nav-speed').value}`);renderLive();scheduleResult();scheduleDiagnostic();
  }
  async function applyParameters(values){
    if(interactionBusy()&&!continuous()||transfer||exchange.busy)throw new Error('请等当前扫描/结果读取完成');
    const entries=Object.entries(values).filter(([key])=>key!=='grid_version').sort(([a],[b])=>a==='situation'?-1:b==='situation'?1:0);
    const requiresWindow=entries.some(([key])=>key!=='threshold');
    for(const[key,value]of entries){const p=RADAR_PARAMETERS.find(p=>p.key===key);if(!p||!Number.isFinite(value)||value<p.min||value>p.max)throw new Error(`参数 ${key} 超出范围`);}
    onBusy(true);
    try{
      await exchange.run(async revision=>{
        for(const[key,value]of entries){
          await exchange.read(`radar set ${key} ${value}`,()=>false,revision,line=>line==='OK radar parameter_updated_ram');
          headingNeedsScan=requiresWindow;for(const kind of ['map','path','points','cloud','stable'])if(model.snapshots[kind])model.snapshots[kind].historical=true;
        }
        const pages=await exchange.settings(revision);
        for(const[key,value]of entries)if(Math.abs(pages.find(p=>p.key===key)?.value-value)>1e-5||!pages.some(p=>p.key===key))throw new Error(`参数 ${key} 读回不一致`);
      });
      headingNeedsScan=requiresWindow;for(const kind of ['map','path','points','cloud','stable'])if(model.snapshots[kind])model.snapshots[kind].historical=true;
      if(!requiresWindow){await send('radar plan');}
    }finally{onBusy(false);render();}
  }
  async function readResult(strict=false){
    return operate('read-status',async operation=>{
      const state=await exchange.run(revision=>exchange.read('radar status',f=>f.k==='status',revision));
      if(!state.map||!state.plan)throw new Error('尚无扫描完成的地图与路线');
      checkRead(operation);await exchange.result(state);checkRead(operation);
      if(resultKey(model.snapshots.map)!==resultKey(state)||resultKey(model.snapshots.path)!==resultKey(state))throw new Error('设备已更新，请重新读取同圈结果');
      $('radar-transfer').textContent=`${state.state==='ready'?'当前':'保留'}地图与规划路径已完整读取`;
    },{propagate:strict});
  }
  async function readStable(){return operate('stable',async operation=>{await exchange.fetch('stable',currentId('stable'));checkRead(operation);$('radar-show-points').checked=true;$('radar-transfer').textContent='完整稳定XY已读取';});}
  async function startContinuous(corner='lower'){
    const wire=`radar live map start ${corner}`;if(interactionBusy()||!canSend(wire))return false;
    continuousRequested=true;$('radar-transfer').textContent='持续建图已请求 · 首次采集15有效圈，之后逐圈最近15圈';
    try{const accepted=await send(wire);if(accepted!==true)continuousRequested=false;render();return accepted;}catch(error){continuousRequested=false;$('radar-transfer').textContent=error.message;render();throw error;}
  }
  async function stopContinuousAndRead(){
    if(!model.connected)throw new Error('串口已断开 · 已保留历史图，不能执行');
    stoppingContinuous=true;const previous=transfer;
    try{
      if(await requestWire('radar stop')!==true)throw new Error('停止采集未写出，不能执行路线');
      if(previous)await previous.done;
      await exchange.run(async revision=>{
        const deadline=Date.now()+exchange.timeoutMs;
        do{
          await exchange.read('radar status',f=>f.k==='progress'&&f.live_active===false,revision);
          if(model.status?.state==='ready'||model.status?.reason!=='processing')return;
          $('radar-transfer').textContent='采集已停止 · 完整窗口正在生成地图与路线';
          if(Date.now()>=deadline)throw new Error('完整窗口仍在处理，请稍后读取结果；历史图保留');
          await new Promise(resolve=>setTimeout(resolve,exchange.intervalMs||50));
        }while(revision===exchange.revision);
        throw new Error('读取已取消');
      });
      continuousRequested=false;
      if(model.status?.state!=='ready'||!model.status.map||!model.status.plan)throw new Error('停止已确认，尚无完整15圈结果；请重新采集');
      await readResult(true);monitorResult.update(model.snapshots.map,model.snapshots.path);
    }finally{stoppingContinuous=false;render();}
  }
  renderProfile();selectTab('monitor');render();setInterval(render,1000);return {requestWire,readResult,readStable,startContinuous,continuous,stopContinuousAndRead,cancelRead:()=>exchange.cancel(),clearResults,canClearResults,applyParameters,readParameters:()=>exchange.get(),busy:interactionBusy,canScan:()=>canSend('radar scan'),message:()=>$('radar-transfer').textContent,receive(line){
    const previousLiveState=live.state;
    if(live.accept(line)){liveRequested=false;liveMessage='';if(previousLiveState!==live.state){render();onLiveState(live.state);}else renderLive();return true;}
    const radarReply=line.trim().replace(/^(?:arm>\s*)+/,'');
    if(continuousRequested&&/^ERR radar\b/.test(radarReply)){continuousRequested=false;$('radar-transfer').textContent=`持续建图未受理 · ${radarReasonText(radarReply.split(' ')[2])}`;render();}
    if(liveRequested&&/^ERR radar\b/.test(radarReply)){liveRequested=false;liveMessage=`实时采集未受理 · ${radarReply}`;render();}
    const recognized=model.accept(line),exchanged=exchange.receive(line),headingAccepted=exchanged&&transfer?.kind==='heading'&&radarReply==='OK radar parameter_updated_ram';
    if(headingAccepted){headingNeedsScan=true;for(const kind of ['map','path','points','cloud'])if(model.snapshots[kind])model.snapshots[kind].historical=true;}
    const replyChanged=observeScanReply(line),frame=parseRadar(line);if(frame){observeScan(frame);if(frame.k==='raw'&&frame.page===0&&diagnostic?.phase==='scanning'&&frame.id!==diagnostic.baseline){diagnostic.id=frame.id;diagnostic.metadata={...frame};if(['ready','error','failed','stopped'].includes(frame.state)){diagnostic.phase='waiting';diagnostic.message='本次诊断采集结束 · 等待同圈地图、点云与原始包下载';}}}if(recognized||replyChanged||headingAccepted)render();return recognized;
  },connection(value){model.connection(value);live.connection(value);if(!value){continuousRequested=false;liveRequested=false;liveMessage='';scanActivity=null;selectionActivity=null;pendingResult=null;cancelledCapture=null;exchange.cancel('连接已变化，旧完整快照保留');}render();},render,selectTab,snapshot:()=>model};
}
