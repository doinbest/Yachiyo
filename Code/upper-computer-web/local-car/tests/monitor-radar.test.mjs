import test from 'node:test';
import assert from 'node:assert/strict';
import {MonitorResult,monitorPreview,backupInstallation,monitorGrid,mountMonitorRadar,previewBodySize,monitorExecutionGate,monitorProtectedMask} from '../monitor-radar.mjs';
const id={session:1,origin:1,scan:3,map:4,plan:5};
const map=(extra={})=>({...id,k:'map',g:2,mask:0x50140,counts:Array(25).fill(0),pose:[230,230,90],...extra});
const path=(extra={})=>({...id,k:'path',valid:true,points:[[2250,150,1,0],[2100,1200,5,1]],...extra});

test('preview body size accepts adjustable dimensions but never exceeds the competition limit',()=>{
  assert.deepEqual(previewBodySize('299','298'),{length_mm:299,width_mm:298});
  assert.throws(()=>previewBodySize('300.1','300'),/不超过 300/);
});

test('F1-aligned scan protection is marked only for captures carrying the new rule version',()=>{
  assert.equal(monitorProtectedMask(map()),0,'old historical captures keep their original interpretation');
  assert.equal(monitorProtectedMask(map({a:3,pose:[2250,150,180]})),0x404414);
  assert.equal(monitorProtectedMask(map({a:3,pose:[2250,2250,270]})),0x1404404);
  assert.equal(monitorProtectedMask(map({a:3,pose:[1200,1200,180]})),0x404404);
});

test('inset captures retain protection at both exact installation corner boundaries',()=>{
  assert.equal(monitorProtectedMask(map({g:3,a:3,pose:[2250,150,180]})),0x404414);
  assert.equal(monitorProtectedMask(map({g:3,a:3,pose:[2250,2250,270]})),0x1404404);
  assert.equal(monitorProtectedMask(map({g:3,a:3,pose:[2170,230,180]})),0x404414);
  assert.equal(monitorProtectedMask(map({g:3,a:3,pose:[2251,150,180]})),0x404404);
  assert.equal(monitorProtectedMask(map({g:3,a:3,pose:[2250,149,180]})),0x404404);
});

test('a local preview never replaces a received device route with a different C result',async()=>{
  await assert.rejects(()=>monitorPreview({map:map(),path:path()},async()=>({valid:true,points:[[2250,150,1,0],[1200,350,2,1]]}),200),/设备路线.*不一致/);
});

test('monitor commits map and route together and retains previous pair while next route downloads',()=>{
  const store=new MonitorResult();
  assert.equal(store.update(map(),null),null);
  const first=store.update(map(),path());assert.equal(first.map.map,4);
  assert.equal(store.update(map({scan:4,map:6,plan:7}),path()),first);
  const next=store.update(map({scan:4,map:6,plan:7}),path({scan:4,map:6,plan:7,valid:false,failed_leg:2,points:[]}));
  assert.equal(next.map.map,6);assert.equal(next.path.valid,false);
  assert.equal(store.current({connected:true,status:{...id,state:'ready',applicable:true}}),false);
  assert.equal(store.current({connected:true,status:{...next.map,state:'ready',applicable:true}}),true);
  assert.equal(store.current({connected:false,status:next.map}),false);
});

test('prepare preview consumes current full-field map and exact C planned phases, sends no device commands',async()=>{
  const calls=[],pair={map:map({mask:0x51140}),path:path()};
  const profile={points:[[2250,150],[2250,300]],turn_speed_deg_s:100,phases:[{kind:'move',target:[2250,300,90],speed_mm_s:200,phase:'move',direction:'forward',segment:1,station:5,visit:1}]};
  const result=await monitorPreview(pair,async(endpoint,body)=>{calls.push({endpoint,body});return {valid:true,mask:body.mask,route_profiles:{planned:profile}};},200);
  assert.equal(calls.length,1);assert.equal(calls[0].endpoint,'radar/plan');
  assert.equal(calls[0].body.mask,0x51140);assert.equal(calls[0].body.start_mode,'lower');
  assert.equal(calls[0].body.points.length,0);assert.equal(result.profile,profile);
  assert.equal(result.runner.state,'done');assert.equal(result.runner.mask,0x51140);
  await assert.rejects(()=>monitorPreview(null,()=>assert.fail('no API'),200),/完整地图/);
});

test('a parameter change marks the retained pair historical before fresh status arrives',()=>{
  const store=new MonitorResult(),model={connected:true,status:{...id,state:'ready',applicable:true}};
  const snapshot=map();store.update(snapshot,path());assert.equal(store.current(model),true);
  snapshot.historical=true;assert.equal(store.current(model),false);
  snapshot.historical=false;store.pair.path.historical=true;assert.equal(store.current(model),false);
});

test('current and old unversioned captures use inset geometry while frozen g2 captures cover the full field',()=>{
  assert.deepEqual(monitorGrid(map()),[0,550,1000,1400,1850,2400]);
  assert.deepEqual(monitorGrid(map({g:3})),[150,550,1000,1400,1850,2250]);
  assert.deepEqual(monitorGrid(null),[150,550,1000,1400,1850,2250]);
  assert.deepEqual(monitorGrid({mask:1}),[150,550,1000,1400,1850,2250]);
});

test('unreachable and upper-start routes report planning result without fabricating phases',async()=>{
  const failed=await monitorPreview({map:map(),path:path()},async()=>({valid:false,failed_leg:3,points:[]}),200);
  assert.equal(failed.profile,null);assert.match(failed.message,/第 3/);
  const upper=await monitorPreview({map:map(),path:path({start:1})},async(_,body)=>{
    assert.equal(body.start_mode,'upper');return {valid:true,plan_only:true,route_profiles:{}};
  },200);
  assert.equal(upper.profile,null);assert.match(upper.message,/右上/);
});

test('real radar execution requires a current reachable lower-start device plan',()=>{
  const currentModel={connected:true,status:{...id,state:'ready',applicable:true}};
  const pair={map:map(),path:path()};
  assert.equal(monitorExecutionGate(currentModel,pair,true),'');
  assert.match(monitorExecutionGate(currentModel,{map:pair.map,path:{...pair.path,start:1}},true),/右上起点/);
  assert.match(monitorExecutionGate(currentModel,{map:pair.map,path:{...pair.path,valid:false}},true),/不可达/);
  assert.match(monitorExecutionGate(currentModel,pair,false),/当前设备/);
  assert.match(monitorExecutionGate({...currentModel,connected:false},pair,false),/串口/);
});

test('monitor execution button sends the current planned route, while preview remains local',async()=>{
  class Element{
    children=[];dataset={};style={};attributes={};value='current';disabled=false;
    append(...items){this.children.push(...items);}replaceChildren(...items){this.children=items;}
    setAttribute(k,v){this.attributes[k]=v;}
  }
  const saved={document:globalThis.document,setInterval:globalThis.setInterval,requestAnimationFrame:globalThis.requestAnimationFrame};
  const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  const wires=[],root=new Element(),store=new MonitorResult(),snapshot=map();store.update(snapshot,path());
  const model={connected:true,status:{...id,state:'ready',applicable:true},params:{},paramsLive:false};
  globalThis.document={getElementById:get,createElement:()=>new Element(),createElementNS:()=>new Element()};
  globalThis.setInterval=()=>0;globalThis.requestAnimationFrame=()=>0;
  try{
    get('radar-nav-speed').value='200';
    mountMonitorRadar({model,store,workspace:{busy:()=>false,canScan:()=>true,message:()=>'',requestWire:async wire=>{wires.push(wire);return true;},readResult:async()=>{},readParameters:async()=>{},applyParameters:async()=>{}},api:()=>assert.fail('preview API should not run'),backupConfig:()=>({}),preparationRoot:root});
    const buttons=[];const walk=e=>{for(const child of e.children??[]){if(child.id)buttons.push(child);walk(child);}};walk(root);
    const execute=buttons.find(button=>button.id==='monitor-radar-execute');assert.ok(execute);assert.equal(execute.disabled,false);
    assert.deepEqual(wires,[],'mount must never configure or move the device');
    get('monitor-scan').onclick();await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(wires,['radar scan lower 3'],'three-circle capture uses RAM static pose, not chassis feedback');
    wires.length=0;
    execute.onclick();await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(wires,['radar nav start 200']);
  }finally{Object.assign(globalThis,saved);}
});

test('backup mounting enters the competition frame with a 90 degree counterclockwise transform',()=>{
  const config={map:{radarXmm:230,radarYmm:230,obstaclePointThreshold:1},filter:{minAngleDeg:0,maxAngleDeg:360,minDistanceMm:100,maxDistanceMm:40000,minEnergy:0,maxEnergy:255}};
  const before=structuredClone(config),params=backupInstallation(config);
  assert.equal(params.lidar_x_mm,2170);assert.equal(params.lidar_y_mm,230);assert.equal(params.zero_deg,180);
  assert.equal(params.angle_max_tenths,3600);assert.deepEqual(config,before);
  const asymmetric=backupInstallation({...config,map:{...config.map,radarXmm:320,radarYmm:410}});
  assert.equal(asymmetric.lidar_x_mm,1990);assert.equal(asymmetric.lidar_y_mm,320);
});

test('monitor corner scan applies full-circle point-count defaults before scanning',async()=>{
  class Element{
    children=[];dataset={};style={};attributes={};value='current';disabled=false;
    append(...items){this.children.push(...items);}replaceChildren(...items){this.children=items;}
    setAttribute(k,v){this.attributes[k]=v;}
  }
  const saved={document:globalThis.document,setInterval:globalThis.setInterval,requestAnimationFrame:globalThis.requestAnimationFrame};
  const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  globalThis.document={getElementById:get,createElement:()=>new Element(),createElementNS:()=>new Element()};
  globalThis.setInterval=()=>0;globalThis.requestAnimationFrame=()=>0;
  try{
    const actions=[];
    const backup={map:{radarXmm:230,radarYmm:230,obstaclePointThreshold:1},filter:{minAngleDeg:0,maxAngleDeg:360,minDistanceMm:100,maxDistanceMm:40000,minEnergy:0,maxEnergy:255}};
    const model={connected:true,params:{},paramsLive:false};
    mountMonitorRadar({model,store:new MonitorResult(),workspace:{busy:()=>false,canScan:()=>true,message:()=>'',readParameters:async()=>actions.push('read'),applyParameters:async p=>actions.push(p),requestWire:async wire=>actions.push(wire)},api:()=>assert.fail('no planning request'),backupConfig:()=>backup,preparationRoot:new Element()});
    get('monitor-capture').value='lower';get('monitor-scan').onclick();await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(actions,[{lidar_x_mm:2250,lidar_y_mm:150,zero_deg:180,angle_min_tenths:0,angle_max_tenths:3600,threshold:1,distance_min_mm:100,distance_max_mm:40000,energy_min:0,energy_max:255},'radar scan lower 3']);
    actions.length=0;
    get('monitor-capture').value='backup';get('monitor-scan').onclick();await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(actions,['read',backupInstallation(backup),'radar scan lower 3'],'read current RAM before copying only changed mounting values');
    Object.assign(model,{params:{...backupInstallation(backup),lidar_y_mm:150},paramsLive:true,identity:{session:1},snapshots:{params:{session:1}}});
    actions.length=0;get('monitor-scan').onclick();await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(actions,[{lidar_y_mm:230},'radar scan lower 3'],'do not repeat unchanged filter writes');
  }finally{Object.assign(globalThis,saved);}
});

test('copied mounting projects raw cardinal samples into the rotated competition cells',()=>{
  const config={map:{radarXmm:230,radarYmm:230,obstaclePointThreshold:1},filter:{minAngleDeg:0,maxAngleDeg:360,minDistanceMm:100,maxDistanceMm:40000,minEnergy:0,maxEnergy:255}};
  const p=backupInstallation(config),grid=[0,550,1000,1400,1850,2400];
  for(const [angle,wantX,wantY,wantCell]of [[0,1170,230,'R1C3'],[90,2170,1230,'R3C5']]){
    const radians=(p.zero_deg-angle)*Math.PI/180;
    const x=p.lidar_x_mm+1000*Math.cos(radians),y=p.lidar_y_mm+1000*Math.sin(radians);
    assert.ok(Math.abs(x-wantX)<1e-8);assert.ok(Math.abs(y-wantY)<1e-8);
    const col=grid.findIndex((edge,i)=>i<5&&x>=edge&&x<grid[i+1]);
    const row=grid.findIndex((edge,i)=>i<5&&y>=edge&&y<grid[i+1]);
    assert.equal(`R${row+1}C${col+1}`,wantCell);
  }
});

test('monitor overlay hides outside monitor and persisted mounting is labelled historical',()=>{
  class Element{
    children=[];dataset={};style={};attributes={};value='current';
    append(...items){this.children.push(...items);}replaceChildren(...items){this.children=items;}
    setAttribute(k,v){this.attributes[k]=v;}
  }
  const saved={document:globalThis.document,setInterval:globalThis.setInterval,requestAnimationFrame:globalThis.requestAnimationFrame};
  const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  globalThis.document={getElementById:get,createElement:()=>new Element(),createElementNS:()=>new Element()};
  globalThis.setInterval=()=>0;globalThis.requestAnimationFrame=()=>0;
  try{
    get('map-page').dataset.mapTab='monitor';
    const model={connected:true,paramsLive:false,params:{lidar_x_mm:230,lidar_y_mm:230,zero_deg:90,threshold:1}};
    const view=mountMonitorRadar({model,store:new MonitorResult(),workspace:{busy:()=>false,canScan:()=>true,message:()=>''},api:()=>assert.fail('no API'),backupConfig:()=>({}),preparationRoot:new Element()});
    assert.match(get('monitor-params').textContent,/历史.*请读回/);
    const layers=get('map-world').children;
    assert.equal(layers[0].style.display,'');
    get('map-page').dataset.mapTab='offline';view.render();
    assert.equal(layers[0].style.display,'none');assert.equal(layers[1].style.display,'none');
    model.paramsLive=true;model.identity={session:2};model.snapshots={params:{session:1}};
    view.render();assert.match(get('monitor-params').textContent,/历史/);
    model.snapshots.params.session=2;view.render();assert.doesNotMatch(get('monitor-params').textContent,/历史/);
    get('map-page').dataset.mapTab='monitor';view.render();assert.equal(layers[0].style.display,'');
  }finally{Object.assign(globalThis,saved);}
});

test('monitor source labels current inset captures and historical full-field captures by grid version',()=>{
  class Element{
    children=[];dataset={};style={};attributes={};value='current';
    append(...items){this.children.push(...items);}replaceChildren(...items){this.children=items;}
    setAttribute(k,v){this.attributes[k]=v;}
  }
  const saved={document:globalThis.document,setInterval:globalThis.setInterval,requestAnimationFrame:globalThis.requestAnimationFrame};
  const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  globalThis.document={getElementById:get,createElement:()=>new Element(),createElementNS:()=>new Element()};
  globalThis.setInterval=()=>0;globalThis.requestAnimationFrame=()=>0;
  try{
    const store=new MonitorResult(),model={connected:true,status:{...id,state:'ready',applicable:true},params:{}};
    store.update(map({g:3}),path());
    const view=mountMonitorRadar({model,store,workspace:{busy:()=>false,canScan:()=>true,message:()=>''},api:()=>assert.fail('no API'),backupConfig:()=>({}),preparationRoot:new Element()});
    assert.match(get('monitor-map-source').textContent,/当前设备.*150～2250 mm判障/);
    assert.doesNotMatch(get('monitor-map-source').textContent,/旧网格/);
    store.update(map({g:2,historical:true}),path({historical:true}));view.render();
    assert.match(get('monitor-map-source').textContent,/历史保留.*全场旧网格/);
    store.update(map({g:undefined,historical:true}),path({historical:true}));view.render();
    assert.match(get('monitor-map-source').textContent,/历史保留.*旧网格/);
    assert.doesNotMatch(get('monitor-map-source').textContent,/全场旧网格/);
  }finally{Object.assign(globalThis,saved);}
});
