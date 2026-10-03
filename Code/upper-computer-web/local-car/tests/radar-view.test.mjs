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
    querySelectorAll(selector){return this.children.flatMap(c=>[...(selector==='input'&&c.tag==='input'?[c]:[]),...c.querySelectorAll(selector)]);}
    getScreenCTM(){return {inverse:()=>this.world?{a:1,d:-1,e:-20,f:2430}:{a:1,d:1,e:-20,f:-30}};}
  }
  const nodes=new Map(),get=id=>{if(id==='radar-show-path')return null;if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  get('radar-show-obstacles').checked=true;
  get('map-world').world=true;get('radar-nav-speed').value='100';get('radar-offline-input').value='mask';
  for(const name of ['monitor','radar','offline']){const e=new Element('button');e.dataset.mapTab=name;get('map-tabs').append(e);}
  const saved={document:globalThis.document,DOMPoint:globalThis.DOMPoint,setInterval:globalThis.setInterval};
  globalThis.document={getElementById:get,createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag),querySelectorAll:()=>[]};
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
