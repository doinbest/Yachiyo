import test from 'node:test';
import assert from 'node:assert/strict';
import {RadarModel,RadarExchange} from '../radar-model.mjs';
import {mountRadarWorkspace,mountRadarParameters} from '../radar-view.mjs';

function surface(){
  class Element{
    constructor(tag='div'){this.tag=tag;this.children=[];this.dataset={};this.style={};this.attributes={};this.events={};this.value='';this.checked=false;this.classList={toggle(){}};}
    append(...children){this.children.push(...children);}
    prepend(...children){this.children.unshift(...children);}
    replaceChildren(...children){this.children=children;}
    setAttribute(k,v){this.attributes[k]=String(v);}
    addEventListener(k,fn){this.events[k]=fn;}
    dispatchEvent(event){this.events[event.type]?.(event);return !event.defaultPrevented;}
    click(){this.onclick?.();}
    querySelectorAll(selector){return this.children.flatMap(c=>[...(selector==='input'&&c.tag==='input'?[c]:[]),...c.querySelectorAll(selector)]);}
    getScreenCTM(){return {inverse:()=>this.world?{a:1,d:-1,e:-20,f:2430}:{a:1,d:1,e:-20,f:-30}};}
  }
  const nodes=new Map(),get=id=>{if(id==='radar-show-path')return null;if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  get('radar-show-obstacles').checked=true;
  get('map-world').world=true;get('radar-nav-speed').value='100';get('radar-offline-input').value='mask';
  for(const name of ['monitor','radar','offline']){const e=new Element('button');e.dataset.mapTab=name;get('map-tabs').append(e);}
  const wireButtons=['radar scan','radar scan auto','radar stop','radar status','radar plan','radar start status','radar start upper','radar start lower'].map(wire=>{const e=get(wire);e.dataset.radarWire=wire;return e;});
  const saved={document:globalThis.document,DOMPoint:globalThis.DOMPoint,setInterval:globalThis.setInterval};
  globalThis.document={getElementById:get,createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag),querySelectorAll:selector=>selector==='[data-radar-wire]'?wireButtons:[]};
  globalThis.DOMPoint=class{constructor(x,y){this.x=x;this.y=y;}matrixTransform(m){return {x:this.x*m.a+m.e,y:this.y*m.d+m.f};}};
  globalThis.setInterval=()=>0;
  return {get,Element,restore(){Object.assign(globalThis,saved);}};
}

test('radar workspace fits the full field without intercepting zoom or drag, keeps viewport through tabs/data and sends no motion',async()=>{
  const ui=surface(),wires=[],sources=[],requests=[],model=new RadarModel(),exchange=new RadarExchange({send:async wire=>{wires.push(wire);return true;}});
  try{
    const workspace=mountRadarWorkspace({model,exchange,send:wire=>wires.push(wire),canSend:()=>false,mapView:{setSource:value=>sources.push(value)},api:async(path,body)=>{requests.push({path,body});return requests.length===1?{valid:true,algorithm:'radar_map_c_v1',mask:0,points:[[2250,150,1,0]]}:{valid:false,failed_leg:2,algorithm:'radar_map_c_v1',mask:0,points:[]};}});
    assert.deepEqual(wires,[]);assert.deepEqual(sources,['feedback']);
    const map=ui.get('field-map'),view='-320 -370 3170 2990';
    assert.equal(map.attributes.viewBox,view);
    for(const type of ['wheel','contextmenu','pointerdown','pointerup','pointercancel']){
      const event={type,deltaY:-100,button:0,pointerId:1,clientX:100,clientY:200,defaultPrevented:false,preventDefault(){this.defaultPrevented=true;}};
      assert.equal(map.dispatchEvent(event),true,`${type} keeps its normal browser behavior`);
      assert.equal(map.attributes.viewBox,view);
    }
    map.events.pointermove({clientX:100,clientY:200});assert.equal(ui.get('map-cursor').textContent,'鼠标 X 80.0 · Y 2230.0 mm');
    workspace.selectTab('offline');workspace.receive('@RADAR '+JSON.stringify({v:1,k:'status',session:1,origin:1,scan:0,map:0,plan:0,page:0,pages:1,state:'idle',points_valid:false}));
    assert.deepEqual(sources,['feedback','offline']); // Navigation changes display only.
    assert.equal(map.attributes.viewBox,view);assert.equal(ui.get('real-route-panel').hidden,true);
    const base={v:1,session:1,origin:1,scan:1,map:1,plan:1};
    workspace.receive('@RADAR '+JSON.stringify({...base,k:'map',page:0,pages:6,mask:4,pose:[2170,230,180]}));
    for(let page=1;page<6;page++)workspace.receive('@RADAR '+JSON.stringify({...base,k:'map',page,pages:6,offset:(page-1)*5,counts:[1,2,3,4,5]}));
    ui.get('radar-show-counts').checked=true;workspace.render();assert.equal(ui.get('radar-counts').children.length,25);
    workspace.receive('@RADAR '+JSON.stringify({...base,k:'status',page:0,pages:1,applicable:false,state:'ready'}));
    assert.match(ui.get('radar-execution-reason').textContent,/请重新扫描/);
    workspace.receive('@RADAR '+JSON.stringify({...base,k:'status',page:0,pages:1,applicable:true,state:'ready'}));
    workspace.receive('@RADAR '+JSON.stringify({...base,k:'path',page:0,pages:1,offset:0,valid:false,failed_leg:2,points:[]}));
    assert.match(ui.get('radar-execution-reason').textContent,/第 2 路段不可达/);
    workspace.receive('@RADAR '+JSON.stringify({...base,k:'nav',page:0,pages:1,state:'stopping',reason:'cancelled',index:1,total:8,station:5,visit:1,stopped:true}));
    assert.match(ui.get('radar-nav-status').textContent,/停车已确认/);
    await ui.get('radar-offline-plan').onclick();assert.equal(requests[0].path,'radar/plan');assert.equal(requests[0].body.points.length,0);assert.deepEqual(wires,[]);
    assert.equal(ui.get('radar-offline-path').attributes.d,'M2250,150');
    await ui.get('radar-offline-plan').onclick();assert.match(ui.get('radar-offline-status').textContent,/第 2 路段/);
    workspace.connection(false);workspace.connection(true);workspace.selectTab('radar');
    assert.equal(map.attributes.viewBox,view);assert.deepEqual(wires,[]);assert.deepEqual(sources,['feedback','offline','feedback']);
  }finally{ui.restore();}
});

const frame=value=>'@RADAR '+JSON.stringify({v:1,session:9,origin:2,scan:3,map:7,plan:8,page:0,pages:1,...value});
const flush=()=>new Promise(setImmediate);
function liveWorkspace(ui,{respond=true,canSend=()=>true}={}){
  const wires=[],busy=[];let workspace,answer=respond;
  const reply=wire=>{
    const [, ,kind,idText,pageText]=wire.split(' '),id=Number(idText),page=Number(pageText);
    if(kind==='map')return frame({k:kind,origin:model.status?.origin??2,scan:model.status?.scan??id-4,map:id,plan:model.status?.plan??id+1,page,pages:3,...(page===0?{mask:4,pose:[2170,230,180]}:{offset:page===1?0:7,counts:Array(page===1?7:18).fill(page)})});
    if(kind==='path')return frame({k:kind,origin:model.status?.origin??2,scan:model.status?.scan??id-5,map:model.status?.map??id-1,plan:id,page,pages:2,start:model.snapshots.start?.selected??2,valid:true,offset:page===0?0:3,points:page===0?[[2250,150,1,0],[2100,300,0,0],[2100,1200,5,1]]:[[350,1200,4,2]]});
    return frame({k:kind,page:0,pages:1,offset:0,total:1,points:[[900,200,50]]});
  };
  const exchange=new RadarExchange({timeoutMs:100,send:async wire=>{wires.push(wire);if(answer&&wire.startsWith('radar fetch '))queueMicrotask(()=>workspace.receive(reply(wire)));return true;}});
  const model=new RadarModel();
  workspace=mountRadarWorkspace({model,exchange,send:async wire=>{wires.push(wire);return true;},canSend,mapView:{setSource(){}},api:async()=>({}),onBusy:value=>busy.push(value)});
  workspace.connection(true);
  return {workspace,model,exchange,wires,busy,reply,setRespond:value=>{answer=value;},async close(){exchange.cancel();await flush();ui.restore();}};
}

test('a completed observed scan automatically displays compact map and path exactly once',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'status',state:'collecting',points_valid:false,applicable:true,scan:2,map:6,plan:7}));
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',points_valid:true,applicable:true}));await flush();
    assert.deepEqual(h.wires,['radar fetch map 7 0','radar fetch map 7 1','radar fetch map 7 2','radar fetch path 8 0','radar fetch path 8 1']);
    assert.equal(h.model.snapshots.map.counts.length,25);assert.equal(ui.get('radar-obstacles').children.length,1);
    assert.equal(ui.get('radar-device-path').attributes.d,'M2250,150 L2100,300 L2100,1200 L350,1200');
    assert.match(ui.get('radar-status').textContent,/采集与规划/);assert.match(ui.get('radar-transfer').textContent,/地图.*路径.*已显示/);
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',points_valid:true,applicable:true}));await flush();assert.equal(h.wires.length,5);
  }finally{await h.close();}
});

test('ready on connection or page navigation does not fetch; an explicit scan can fetch its unseen completion',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'status',state:'ready',points_valid:true,applicable:true}));h.workspace.selectTab('radar');await flush();assert.deepEqual(h.wires,[]);
    ui.get('radar scan').onclick();await flush();
    h.workspace.receive('OK radar scan_accepted');
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',points_valid:true,applicable:true,scan:4,map:8,plan:9}));await flush();
    assert.deepEqual(h.wires,['radar scan','radar fetch map 8 0','radar fetch map 8 1','radar fetch map 8 2','radar fetch path 9 0','radar fetch path 9 1']);
    h.workspace.connection(false);h.workspace.connection(true);h.workspace.selectTab('monitor');h.workspace.receive(frame({k:'status',state:'ready',scan:4,map:8,plan:9}));await flush();assert.equal(h.wires.length,6);
  }finally{await h.close();}
});

test('a first scan request needs observed scan progression before accepting a queued old ready frame',async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    ui.get('radar scan').onclick();await flush();
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();assert.deepEqual(h.wires,['radar scan']);
    h.workspace.receive('OK radar scan_accepted');h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();assert.deepEqual(h.wires,['radar scan']);
    h.workspace.receive(frame({k:'status',state:'preparing'}));h.setRespond(true);h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9}));await flush();
    assert.equal(h.model.snapshots.path?.plan,9);assert.equal(h.wires.length,6);
  }finally{await h.close();}
});

test('a rejected explicit scan cannot arm later ready statuses for automatic fetching',async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    ui.get('radar scan').onclick();await flush();h.workspace.receive('ERR radar stationary_capture_required');
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();assert.deepEqual(h.wires,['radar scan']);
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9}));await flush();
    assert.deepEqual(h.wires,['radar scan']);assert.equal(ui.get('radar scan').disabled,false);
  }finally{await h.close();}
});

test('a known baseline still waits for scan acceptance when no active scan state was seen',async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    h.workspace.receive(frame({k:'status',state:'ready',scan:2,map:6,plan:7}));ui.get('radar scan').onclick();await flush();
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();assert.deepEqual(h.wires,['radar scan']);
    h.workspace.receive('OK radar scan_accepted');h.setRespond(true);h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();
    assert.equal(h.model.snapshots.path?.plan,8);assert.equal(h.wires.length,6);
  }finally{await h.close();}
});

for(const kind of ['map','path'])test(`manual ${kind} refresh preempts an optional point read`,async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    h.workspace.receive(frame({k:'status',state:'ready',points_valid:true}));h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));
    ui.get('radar-fetch-points').onclick();await flush();assert.equal(ui.get(`radar-fetch-${kind}`).disabled,false);
    h.setRespond(true);ui.get(`radar-fetch-${kind}`).onclick();await flush();
    assert.equal(h.exchange.busy,false);assert.equal(h.model.snapshots.points,undefined);
    assert.deepEqual(h.wires,kind==='map'?['radar fetch points 3 0','radar fetch map 7 0','radar fetch map 7 1','radar fetch map 7 2']:['radar fetch points 3 0','radar fetch path 8 0','radar fetch path 8 1']);
  }finally{await h.close();}
});

test('scan result waits for occupied parameter exchange and downloads once that read is idle',async()=>{
  let parameterBusy=true;const ui=surface(),h=liveWorkspace(ui,{canSend:()=>!parameterBusy});
  try{
    const reading=h.exchange.get();reading.catch(()=>{});await flush();
    h.workspace.receive(frame({k:'status',state:'preparing',scan:2,map:6,plan:7}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',applicable:true}));await flush();assert.deepEqual(h.wires,['radar get']);
    h.workspace.receive(frame({k:'status',state:'collecting'}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',applicable:true,scan:4,map:8,plan:9}));
    h.workspace.receive(frame({k:'params',rev:1,key:'threshold',value:3}));await reading;parameterBusy=false;h.workspace.render();await flush();
    assert.equal(h.model.snapshots.path.plan,9);assert.deepEqual(h.wires,['radar get','radar fetch map 8 0','radar fetch map 8 1','radar fetch map 8 2','radar fetch path 9 0','radar fetch path 9 1']);
  }finally{await h.close();}
});

test('unreachable plans are downloaded and shown once; read errors preserve history without automatic retry',async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    h.workspace.receive(frame({k:'status',state:'collecting',scan:2,map:6,plan:7}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();
    for(let page=0;page<3;page++){h.workspace.receive(h.reply(`radar fetch map 7 ${page}`));await flush();}
    h.workspace.receive(frame({k:'path',valid:false,failed_leg:2,offset:0,points:[]}));await flush();
    assert.equal(h.model.snapshots.path.valid,false);assert.equal(ui.get('radar-device-path').attributes.d,'');assert.match(ui.get('radar-transfer').textContent,/第 2 路段不可达/);
    h.workspace.receive(frame({k:'status',state:'collecting'}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9}));await flush();
    h.workspace.receive('ERR radar snapshot_changed_or_page_range');await flush();const count=h.wires.length;
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9}));await flush();
    assert.equal(h.wires.length,count);assert.equal(h.model.snapshots.map.map,7);assert.match(ui.get('radar-transfer').textContent,/ERR radar/);
  }finally{await h.close();}
});

test('optional point reads keep navigation available and a new completed result preempts them',async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    h.workspace.receive(frame({k:'status',state:'ready',points_valid:true,applicable:true}));
    h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));
    ui.get('radar-fetch-points').onclick();await flush();
    assert.deepEqual(h.wires,['radar fetch points 3 0']);assert.deepEqual(h.busy,[]);
    assert.equal(ui.get('radar-nav-start').disabled,false);assert.equal(ui.get('radar-fetch-points').disabled,true);assert.equal(ui.get('radar scan').disabled,true);
    h.workspace.receive(frame({k:'points',page:0,pages:2,offset:0,total:2,points:[[900,200,50]]}));await flush();
    h.workspace.receive(frame({k:'status',state:'collecting',points_valid:false}));h.setRespond(true);
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',points_valid:true,applicable:true,scan:4,map:8,plan:9}));await flush();
    assert.equal(h.model.snapshots.path.plan,9);assert.equal(h.model.snapshots.points,undefined);assert.deepEqual(h.busy,[]);
    h.workspace.receive(frame({k:'points',page:1,pages:2,offset:1,total:2,points:[[1800,300,50]]}));assert.equal(h.model.snapshots.points,undefined,'late pages cannot publish a canceled partial point cloud');
  }finally{await h.close();}
});

test('a newer scan supersedes the old result read and cancel suppresses repeated ready frames',async()=>{
  const ui=surface(),h=liveWorkspace(ui,{respond:false});
  try{
    h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));
    h.workspace.receive(frame({k:'status',state:'collecting',scan:2,map:6,plan:7}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();
    assert.deepEqual(h.wires,['radar fetch map 7 0']);
    h.workspace.receive(frame({k:'status',state:'preparing'}));await flush();assert.equal(h.exchange.busy,false);assert.equal(h.model.snapshots.map.map,7);
    h.setRespond(true);h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9}));await flush();assert.equal(h.model.snapshots.path.plan,9);
    h.setRespond(false);h.workspace.receive(frame({k:'status',state:'collecting',scan:4,map:8,plan:9}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:5,map:9,plan:10}));await flush();
    ui.get('radar-cancel-transfer').onclick();await flush();const count=h.wires.length;
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:5,map:9,plan:10}));await flush();assert.equal(h.wires.length,count);assert.equal(h.model.snapshots.map.map,8);
  }finally{await h.close();}
});

test('stopped and failed captures retain history without fetching old ready data',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));
    for(const state of ['stopped','error']){
      h.workspace.receive(frame({k:'status',state:'collecting'}));h.workspace.receive(frame({k:'status',state,reason:'cancelled'}));h.workspace.receive(frame({k:'status',state:'ready'}));await flush();
    }
    assert.deepEqual(h.wires,[]);assert.equal(h.model.snapshots.map.map,7);
  }finally{await h.close();}
});

test('cancel at the final map page cannot start the following automatic path read',async()=>{
  const ui=surface(),h=liveWorkspace(ui),progress=h.exchange.onProgress;
  try{
    h.exchange.onProgress=value=>{progress(value);if(value.kind==='map'&&value.page===value.pages)ui.get('radar-cancel-transfer').onclick();};
    h.workspace.receive(frame({k:'status',state:'collecting',scan:2,map:6,plan:7}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete'}));await flush();
    assert.deepEqual(h.wires,['radar fetch map 7 0','radar fetch map 7 1','radar fetch map 7 2']);
    assert.equal(h.model.snapshots.map.map,7);assert.equal(h.model.snapshots.path,undefined);assert.match(ui.get('radar-transfer').textContent,/取消/);
  }finally{await h.close();}
});

test('cancel after a point-read prerequisite map does not resume the optional point read',async()=>{
  const ui=surface(),h=liveWorkspace(ui),progress=h.exchange.onProgress;
  try{
    h.exchange.onProgress=value=>{progress(value);if(value.kind==='map'&&value.page===value.pages)ui.get('radar-cancel-transfer').onclick();};
    h.workspace.receive(frame({k:'status',state:'ready',points_valid:true}));ui.get('radar-fetch-points').onclick();await flush();
    assert.deepEqual(h.wires,['radar fetch map 7 0','radar fetch map 7 1','radar fetch map 7 2']);assert.equal(h.model.snapshots.points,undefined);
  }finally{await h.close();}
});

test('a stopped canceled capture permits the next externally observed scan with the same baseline IDs',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'status',state:'collecting'}));h.exchange.cancel();h.workspace.receive(frame({k:'status',state:'stopped'}));
    h.workspace.receive(frame({k:'status',state:'preparing'}));h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9}));await flush();
    assert.equal(h.model.snapshots.path?.plan,9);assert.equal(h.wires.length,5);
  }finally{await h.close();}
});

function classification(h,{selected=0,reason='ambiguous',scan=4,map=7,plan=8,auto=true,origin=2}={}){
  h.workspace.receive(frame({k:'start',id:scan,origin,scan,map,plan,page:0,pages:2,auto,selected,reason}));
  h.workspace.receive(frame({k:'start',id:scan,origin,scan,map,plan,page:1,pages:2,qr:[0,4],raw:[0,5],conflicts:[0,2]}));
}

test('an unresolved automatic capture keeps the old map and path as dim history without fetching them',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));
    h.workspace.receive(frame({k:'path',start:2,valid:true,offset:0,points:[[2250,150,1,0],[2100,1200,5,1]]}));
    ui.get('radar scan auto').onclick();await flush();assert.match(ui.get('radar-start-status').textContent??'',/扫描中|等待本圈/);
    h.workspace.receive('OK radar scan_accepted');h.workspace.receive(frame({k:'status',state:'collecting'}));classification(h);
    h.workspace.receive(frame({k:'status',state:'ready',reason:'start_ambiguous',scan:4,map:7,plan:8,applicable:false,points_valid:false}));await flush();
    assert.deepEqual(h.wires,['radar scan auto']);assert.equal(h.model.snapshots.map.map,7);
    assert.match(ui.get('radar-start-status').textContent,/未确定.*人工选择/);assert.match(ui.get('radar-map-age').textContent,/历史/);
    assert.doesNotMatch(ui.get('radar-map-age').textContent,/参数\/原点已变化/);assert.equal(ui.get('radar-obstacles').style.opacity,'.4');assert.equal(ui.get('radar-device-path').style.opacity,'.4');
    assert.equal(ui.get('radar-nav-start').disabled,true);assert.equal(ui.get('radar-task-start').disabled,true);
    ui.get('radar-nav-start').onclick();assert.deepEqual(h.wires,['radar scan auto']);assert.match(ui.get('radar-execution-reason').textContent,/起点.*未确定/);
  }finally{await h.close();}
});

test('an unresolved status explains the start choice even if its score page was not received',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));
    h.workspace.receive(frame({k:'status',state:'collecting'}));h.workspace.receive(frame({k:'start',id:4,scan:4,page:0,pages:2,auto:true,selected:0,reason:'both_conflict'}));
    h.workspace.receive(frame({k:'status',state:'ready',reason:'start_both_conflict',scan:4,applicable:false,points_valid:false}));await flush();
    assert.match(ui.get('radar-start-status').textContent,/未确定.*人工选择/);assert.match(ui.get('radar-map-age').textContent,/起点未确定/);assert.doesNotMatch(ui.get('radar-map-age').textContent,/参数\/原点已变化/);assert.deepEqual(h.wires,[]);
  }finally{await h.close();}
});

test('an unresolved start rejects optional point reads directly without downloading the old map',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'map',mask:4,pose:[2170,230,180],offset:0,counts:Array(25).fill(0)}));classification(h);
    h.workspace.receive(frame({k:'status',state:'ready',reason:'start_ambiguous',scan:4,applicable:false,points_valid:true}));
    for(const kind of ['points','cloud']){ui.get(`radar-fetch-${kind}`).onclick();await flush();assert.deepEqual(h.wires,[]);assert.match(ui.get('radar-transfer').textContent,/起点未确定.*人工选择.*点云/);}
  }finally{await h.close();}
});

test('an upper result displays its plan and prevents both execution buttons without sending navigation',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'status',state:'collecting'}));classification(h,{selected:1,reason:'clear_regions',map:8,plan:9});
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9,applicable:true,points_valid:true}));await flush();
    assert.equal(h.model.snapshots.path.start,1);assert.match(ui.get('radar-start-status').textContent,/右上.*仅规划/);
    assert.equal(ui.get('radar-nav-start').disabled,true);assert.equal(ui.get('radar-task-start').disabled,true);assert.match(ui.get('radar-execution-reason').textContent,/上起点.*仅.*规划/);
    const count=h.wires.length;ui.get('radar-nav-start').onclick();ui.get('radar-task-start').onclick();h.workspace.selectTab('radar');h.workspace.selectTab('monitor');await flush();assert.equal(h.wires.length,count);
  }finally{await h.close();}
});

test('manual start selection refreshes map and path from the same complete circle and can restore lower execution',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    classification(h);h.workspace.receive(frame({k:'status',state:'ready',reason:'start_ambiguous',scan:4,applicable:false,points_valid:false}));
    ui.get('radar start upper').onclick();await flush();classification(h,{selected:1,reason:'manual_selection',map:8,plan:9});
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:8,plan:9,applicable:true,points_valid:true}));await flush();
    assert.deepEqual(h.wires,['radar start upper','radar fetch map 8 0','radar fetch map 8 1','radar fetch map 8 2','radar fetch path 9 0','radar fetch path 9 1']);
    ui.get('radar start lower').onclick();await flush();classification(h,{selected:2,reason:'manual_selection',map:9,plan:10});
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:4,map:9,plan:10,applicable:true,points_valid:true}));await flush();
    assert.equal(h.model.snapshots.path.start,2);assert.equal(ui.get('radar-nav-start').disabled,false);assert.equal(h.wires.filter(w=>w.startsWith('radar scan')).length,0);
    assert.equal(h.model.snapshots.map.scan,4);assert.equal(h.model.snapshots.path.scan,4);
    assert.match(ui.get('radar-start-status').textContent,/右下.*人工/);
  }finally{await h.close();}
});

test('selecting an unresolved circle can publish its capture origin while the old map used another origin',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    classification(h);h.workspace.receive(frame({k:'status',state:'ready',reason:'start_ambiguous',scan:4,applicable:false,points_valid:false}));
    ui.get('radar start lower').onclick();await flush();classification(h,{selected:2,reason:'manual_selection',map:8,plan:9,origin:3});
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',origin:3,scan:4,map:8,plan:9,applicable:true,points_valid:true}));await flush();
    assert.equal(h.model.snapshots.path?.origin,3);assert.equal(h.model.snapshots.map?.scan,4);assert.equal(h.wires.length,6);
  }finally{await h.close();}
});

test('historical start diagnostics do not claim a current choice after reconnect or during a new capture',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    classification(h,{selected:2,reason:'manual_reference',scan:3,auto:false});h.workspace.receive(frame({k:'status',state:'ready',applicable:true}));
    h.workspace.connection(false);h.workspace.connection(true);h.workspace.selectTab('radar');assert.deepEqual(h.wires,[]);assert.match(ui.get('radar-start-status').textContent??'',/历史/);
    h.workspace.receive(frame({k:'status',state:'collecting'}));assert.match(ui.get('radar-start-status').textContent,/扫描中|等待本圈/);assert.deepEqual(h.wires,[]);
  }finally{await h.close();}
});

test('first auto scan with a foreground active status accepts its fresh completed result without low-frequency updates',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    ui.get('radar scan auto').onclick();await flush();h.workspace.receive('OK radar scan_accepted');
    h.workspace.receive(frame({k:'status',state:'preparing',scan:0,map:0,plan:0}));classification(h,{selected:2,reason:'clear_regions',scan:1,map:1,plan:1});
    h.workspace.receive(frame({k:'status',state:'ready',reason:'complete',scan:1,map:1,plan:1,applicable:true,points_valid:true}));await flush();
    assert.deepEqual(h.wires,['radar scan auto','radar fetch map 1 0','radar fetch map 1 1','radar fetch map 1 2','radar fetch path 1 0','radar fetch path 1 1']);assert.equal(ui.get('radar scan auto').disabled,false);
  }finally{await h.close();}
});

test('an unresolved completed circle names the retained historical map rather than claiming the current map is ready',async()=>{
  const ui=surface(),h=liveWorkspace(ui);
  try{
    h.workspace.receive(frame({k:'status',state:'ready',reason:'start_ambiguous',scan:4,map:2,plan:2,applicable:false,points_valid:true}));
    assert.match(ui.get('radar-status').textContent,/本圈采集完成/);
    assert.match(ui.get('radar-status').textContent,/扫描 4.*历史地图 2.*规划 2/);
    assert.match(ui.get('radar-status').textContent,/不适用本圈/);
    assert.doesNotMatch(ui.get('radar-status').textContent,/完整地图就绪/);
    h.workspace.receive(frame({k:'status',state:'ready',reason:'start_both_conflict',scan:1,map:0,plan:0,applicable:false,points_valid:true}));
    assert.match(ui.get('radar-status').textContent,/尚未发布本圈地图和规划/);
    assert.doesNotMatch(ui.get('radar-status').textContent,/历史地图 0/);
    assert.deepEqual(h.wires,[]);
  }finally{await h.close();}
});

test('upper copy and export/import preserve the corner, lower reference parameters, geometric path and manual obstacles',async()=>{
  const ui=surface(),model=new RadarModel(),wires=[],requests=[],previews=[];let exported;
  const originalCreate=URL.createObjectURL,originalRevoke=URL.revokeObjectURL;
  try{
    URL.createObjectURL=blob=>{exported=blob;return 'blob:radar-offline-test';};URL.revokeObjectURL=()=>{};
    model.params={lidar_x_mm:2170,lidar_y_mm:230,zero_deg:180};
    const base={session:9,origin:2,scan:3,map:7,plan:8};
    model.snapshots={map:{...base,k:'map',mask:0,counts:Array(25).fill(0),pose:[2330,2170,0],coordinate_frame:'current'},start:{...base,selected:1},path:{...base,start:1,points:[[2250,2250,1,0],[2100,1200,5,1]]}};
    const fixed={points:[[2250,150,1,0],[2100,300,0,0]],phases:[{kind:'move',phase:'dock',direction:'forward',segment:1,station:5,visit:1,target:[2100,300,90],speed_mm_s:200}],speed_mm_s:200,turn_speed_deg_s:100};
    const workspace=mountRadarWorkspace({model,exchange:new RadarExchange({send:async wire=>{wires.push(wire);return true;}}),send:wire=>wires.push(wire),canSend:()=>false,mapView:{setSource(){},startProfile:profile=>previews.push(profile)},api:async(path,body)=>{requests.push({path,body});return {valid:true,algorithm:'radar_map_c_v1',mask:body.mask,start:{selected:1},plan_only:true,pose:[2330,2170,0],points:[[2250,2250,1,0],[2100,1200,5,1]],route_profiles:{fixed,planned:{points:[],phases:[],speed_mm_s:200,turn_speed_deg_s:100}}};}});
    workspace.selectTab('offline');ui.get('radar-use-device').onclick();await ui.get('radar-offline-plan').onclick();
    assert.equal(requests[0].body.start_mode,'upper');assert.deepEqual([requests[0].body.params.lidar_x_mm,requests[0].body.params.lidar_y_mm,requests[0].body.params.zero_deg],[2170,230,180]);
    assert.equal(ui.get('radar-offline-path').attributes.d,'M2250,2250 L2100,1200');assert.equal(ui.get('route-profile-start').disabled,true);assert.match(ui.get('route-profile-status').textContent,/仅.*几何路径|几何路径.*播放/);
    ui.get('route-profile-start').onclick();assert.deepEqual(previews,[]);
    const obstacle=ui.get('radar-mask-cells').querySelectorAll('input').find(input=>Number(input.dataset.cell)===21);obstacle.checked=true;obstacle.onchange();
    ui.get('radar-export').onclick();const exportedText=await exported.text();ui.get('radar-use-device').onclick();
    ui.get('radar-import-frame').value='current';ui.get('radar-import').files=[{text:async()=>exportedText}];await ui.get('radar-import').onchange();await ui.get('radar-offline-plan').onclick();
    assert.equal(requests[1].body.start_mode,'upper');assert.equal(requests[1].body.mask,1<<21);assert.deepEqual([requests[1].body.params.lidar_x_mm,requests[1].body.params.lidar_y_mm,requests[1].body.params.zero_deg],[2170,230,180]);assert.equal(ui.get('radar-offline-path').attributes.d,'M2250,2250 L2100,1200');
    ui.get('route-profile-choice').value='fixed';ui.get('route-profile-choice').onchange();assert.equal(ui.get('radar-offline-path').attributes.d,'M2250,150 L2100,300');assert.equal(ui.get('route-profile-start').disabled,false);assert.deepEqual(wires,[]);
  }finally{URL.createObjectURL=originalCreate;URL.revokeObjectURL=originalRevoke;ui.restore();}
});

test('a pose-only upper import recovers the lower sensor reference before requesting upper C projection',async()=>{
  const ui=surface(),requests=[];
  try{
    mountRadarWorkspace({model:new RadarModel(),exchange:new RadarExchange({send:async()=>true}),send(){},canSend:()=>false,mapView:{setSource(){}},api:async(path,body)=>{requests.push(body);return {valid:true,mask:body.mask,points:[]};}});
    ui.get('radar-import-frame').value='current';ui.get('radar-import').files=[{text:async()=>JSON.stringify({mask:0,start_id:1,pose:[2320,2180,350],coordinate_frame:'current'})}];await ui.get('radar-import').onchange();await ui.get('radar-offline-plan').onclick();
    assert.equal(requests[0].start_mode,'upper');assert.deepEqual([requests[0].params.lidar_x_mm,requests[0].params.lidar_y_mm,requests[0].params.zero_deg],[2180,220,170]);
  }finally{ui.restore();}
});

test('planning paths and station marks render without a second visibility checkbox',()=>{
  const ui=surface(),model=new RadarModel(),wires=[];
  try{
    const workspace=mountRadarWorkspace({model,exchange:new RadarExchange({send:async wire=>{wires.push(wire);return true;}}),send:wire=>wires.push(wire),canSend:()=>false,mapView:{setSource(){}},api:async()=>({})});
    ui.get('radar-device-path').style.display='none';ui.get('radar-stations').style.display='none';
    workspace.receive('@RADAR '+JSON.stringify({v:1,k:'path',session:1,origin:1,scan:1,map:1,plan:1,page:0,pages:1,offset:0,valid:true,points:[[2250,150,1,0],[2100,1200,5,1]]}));
    assert.equal(ui.get('radar-device-path').attributes.d,'M2250,150 L2100,1200');assert.equal(ui.get('radar-stations').children.length,2);
    workspace.render();assert.equal(ui.get('radar-device-path').style.display,'none');assert.equal(ui.get('radar-stations').style.display,'none');assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});

test('radar parameter group mounts without writes and preserves pending inputs on readback',()=>{
  const ui=surface(),wires=[],model=new RadarModel(),exchange=new RadarExchange({send:async wire=>{wires.push(wire);return true;}});
  try{
    const root=new ui.Element(),group=mountRadarParameters({root,exchange,model,canSend:()=>false,onBusy(){}});
    const input=root.children[0].children[2].children[1];input.value='2199';
    model.params.lidar_x_mm=2170;model.paramsLive=true;group.update();
    assert.equal(input.value,'2199');assert.deepEqual(wires,[]);
    model.connection(false);group.cancel();assert.match(root.children[0].children[2].children[2].textContent,/历史设备值/);
  }finally{ui.restore();}
});

test('offline current preview renders only C-supplied phases, separates grid and footprint checks, and sends no TX',async()=>{
  const ui=surface(),wires=[],requests=[],previews=[];
  try{
    ui.get('radar-nav-speed').value='200';ui.get('route-profile-choice').value='fixed';
    const phase={kind:'turn',phase:'arrival_turn',direction:'ccw',segment:5,station:4,visit:1,target:[350,1200,0],speed_mm_s:100};
    const fixed={points:[[350,1200,0,0],[350,1200,4,1]],phases:[phase],speed_mm_s:200,turn_speed_deg_s:100};
    const planned={points:[[2100,300,0,0],[2100,1200,5,1]],phases:[{...phase,kind:'move',phase:'dock',direction:'forward',target:[2100,1200,90]}],speed_mm_s:200,turn_speed_deg_s:100};
    const workspace=mountRadarWorkspace({model:new RadarModel(),exchange:new RadarExchange({send:async wire=>{wires.push(wire);return true;}}),send:wire=>wires.push(wire),canSend:()=>false,mapView:{setSource(){},startProfile:(profile,options)=>previews.push({profile,options})},api:async(path,body)=>{requests.push({path,body});return {valid:true,algorithm:'radar_map_c_v1',mask:0,points:planned.points,route_profiles:{fixed,planned}};}});
    workspace.selectTab('offline');await ui.get('radar-offline-plan').onclick();
    assert.equal(requests[0].body.speed_mm_s,200);assert.match(ui.get('route-profile-grid').textContent,/网格可达/);
    assert.match(ui.get('route-profile-phases').children[0].textContent,/目标 5.*到站转向.*100/);
    assert.match(ui.get('route-profile-status').textContent,/目标 5.*到站转向.*非行车区/);
    ui.get('route-profile-start').onclick();assert.equal(previews[0].profile,fixed);assert.deepEqual(wires,[]);
    ui.get('route-profile-choice').value='planned';ui.get('route-profile-choice').onchange();
    assert.match(ui.get('route-profile-status').textContent,/外框检查通过/);assert.equal(ui.get('radar-offline-path').attributes.d,'M2100,300 L2100,1200');
    ui.get('route-profile-start').onclick();assert.equal(previews[1].profile,planned);assert.deepEqual(wires,[]);
    const obstacle=ui.get('radar-mask-cells').querySelectorAll('input')[0];obstacle.checked=true;obstacle.onchange();
    assert.equal(ui.get('route-profile-start').disabled,true);assert.equal(ui.get('radar-offline-path').attributes.d,'');assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});
