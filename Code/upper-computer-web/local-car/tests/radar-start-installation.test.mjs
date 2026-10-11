import test from 'node:test';
import assert from 'node:assert/strict';
import {mountMonitorRadar,MonitorResult} from '../monitor-radar.mjs';

function surface(storage=new Map()){
  class Element{
    constructor(tag='div'){this.tag=tag;this.children=[];this.dataset={};this.style={};this.attributes={};this.value='';this.events={};}
    append(...items){this.children.push(...items);}replaceChildren(...items){this.children=items;}
    setAttribute(k,v){this.attributes[k]=String(v);}
    addEventListener(k,fn){this.events[k]=fn;}
  }
  const saved={document:globalThis.document,setInterval:globalThis.setInterval,requestAnimationFrame:globalThis.requestAnimationFrame,localStorage:globalThis.localStorage};
  const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
  globalThis.document={getElementById:get,createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag)};
  globalThis.setInterval=globalThis.requestAnimationFrame=()=>0;
  globalThis.localStorage={getItem:k=>storage.get(k)??null,setItem:(k,v)=>storage.set(k,v)};
  get('map-page').dataset.mapTab='monitor';get('monitor-capture').value='current';
  const find=id=>[...nodes.values()].flatMap(function walk(e){return [e,...e.children.flatMap(walk)];}).find(e=>e.id===id);
  const model={connected:true,params:{},paramsLive:false},writes=[];
  const workspace={busy:()=>false,canScan:()=>true,message:()=>'',applyParameters:async p=>writes.push(p),requestWire:async wire=>writes.push({wire}),startContinuous:async corner=>writes.push({continuous:corner})};
  const view=mountMonitorRadar({model,store:new MonitorResult(),workspace,api:()=>assert.fail('no planning'),preparationRoot:new Element()});
  return {get,find,model,writes,view,workspace,restore:()=>Object.assign(globalThis,saved)};
}
const settled=()=>new Promise(resolve=>setImmediate(resolve));

test('lower single and continuous capture confirm the fixed direction before scanning without fresh parameters',async()=>{
  for(const [id,expected]of [['monitor-scan',{wire:'radar scan lower 15'}],['monitor-continuous',{continuous:'lower'}]]){
    const ui=surface();try{
      ui.model.params.zero_deg=90;
      ui.get(id).onclick();await settled();
      assert.deepEqual(ui.writes,[{zero_deg:180},expected]);
    }finally{ui.restore();}
  }
});

test('confirmed lower direction avoids another parameter write and failed confirmation does not scan',async()=>{
  const ui=surface();try{
    Object.assign(ui.model,{paramsLive:true,identity:{session:1},snapshots:{params:{session:1}},params:{zero_deg:180}});
    ui.get('monitor-scan').onclick();await settled();
    assert.deepEqual(ui.writes,[{wire:'radar scan lower 15'}]);
    ui.writes.length=0;ui.model.paramsLive=false;
    ui.workspace.applyParameters=async()=>{throw new Error('direction readback failed');};
    ui.get('monitor-scan').onclick();await settled();
    assert.deepEqual(ui.writes,[]);
    assert.match(ui.get('monitor-radar-message').textContent,/direction readback failed/);
  }finally{ui.restore();}
});

test('start selection fixes direction, exposes only X/Y, and does not write hardware',async()=>{
  const ui=surface();try{
    assert.equal(ui.find('monitor-installation-preset'),undefined);
    assert.equal(ui.find('monitor-installation-zero_deg'),undefined);
    const x=ui.find('monitor-installation-lidar_x_mm'),y=ui.find('monitor-installation-lidar_y_mm');
    assert.ok(x&&y);assert.equal(x.value,'2280');assert.equal(y.value,'314');
    ui.get('monitor-capture').value='upper';ui.view.render();
    assert.equal(x.value,'2050');assert.equal(y.value,'2250');
    assert.match(ui.find('monitor-installation-direction').textContent,/向下/);
    assert.equal(ui.get('map-world').children.find(e=>e.attributes.id==='monitor-installation-arrow').children[0].attributes.cx,'2050');
    x.value='2045';y.value='2240';ui.view.render();
    await ui.find('monitor-installation-apply').onclick();await settled();
    assert.deepEqual(ui.writes,[{upper_lidar_x_mm:2045,upper_lidar_y_mm:2240}]);
    ui.get('monitor-capture').value='current';ui.view.render();
    assert.equal(x.value,'2280');assert.equal(y.value,'314');
    assert.match(ui.find('monitor-installation-direction').textContent,/向左/);
    await ui.find('monitor-installation-apply').onclick();await settled();
    assert.deepEqual(ui.writes[1],{lidar_x_mm:2280,lidar_y_mm:314,zero_deg:180});
    ui.get('monitor-capture').value='upper';ui.view.render();assert.equal(x.value,'2045');
  }finally{ui.restore();}
});

test('both coordinate drafts survive reload without automatic writes or overwriting edits on readback',async()=>{
  const storage=new Map();let ui=surface(storage);try{
    ui.find('monitor-installation-lidar_x_mm').value='2275';
    ui.get('monitor-capture').value='upper';ui.view.render();
    ui.find('monitor-installation-lidar_x_mm').value='2040';
    ui.find('monitor-installation-save').onclick();assert.deepEqual(ui.writes,[]);
  }finally{ui.restore();}
  ui=surface(storage);try{
    assert.equal(ui.find('monitor-installation-lidar_x_mm').value,'2275');
    ui.get('monitor-capture').value='upper';ui.view.render();assert.equal(ui.find('monitor-installation-lidar_x_mm').value,'2040');
    Object.assign(ui.model,{paramsLive:true,identity:{session:1},snapshots:{params:{session:1}},params:{upper_lidar_x_mm:2055,upper_lidar_y_mm:2250}});
    ui.view.render();assert.equal(ui.find('monitor-installation-lidar_x_mm').value,'2040');assert.deepEqual(ui.writes,[]);
  }finally{ui.restore();}
});

test('empty coordinates are rejected instead of becoming zero',async()=>{
  const ui=surface();try{
    ui.find('monitor-installation-lidar_x_mm').value='';
    ui.find('monitor-installation-apply').onclick();await settled();
    assert.deepEqual(ui.writes,[]);assert.match(ui.get('monitor-radar-message').textContent,/坐标/);
  }finally{ui.restore();}
});
