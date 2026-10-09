import {GRAB_FORM_DEFAULTS} from '../grab-params.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {parseTrack,TRACK_PRESET,trackText,TrackingPanelModel,mountTracking,trackCrc} from '../grab-track.mjs';
import {readOnlyCommand,scopedStopCommand} from '../bridge.mjs';

test('tracking faults use Chinese reasons while tracking protocol remains English',()=>{
  const raw='ERR grab track state=error reason=feedback_stale';
  const status=parseTrack(raw);
  assert.match(trackText(status.reason),/反馈.*过期/);
  assert.match(trackText('bus_unavailable'),/总线.*不可用/);
  assert.match(trackText('reference_missing'),/参考.*未建立/);
  assert.equal(status.reason,'feedback_stale');assert.equal(status.raw,raw);
});

test('live diagnostics are passive and do not fill the frozen trace array',()=>{
  for(const wire of ['grab track log start','grab track log start 100','grab track log stop','grab track log status'])assert.equal(readOnlyCommand(wire),true);
  const model=new TrackingPanelModel();
  model.receive('OK grab track log_begin id=2 period_ms=50 next=0');
  for(const i of [0,2]){
    const row=`TRACK live id=2 i=${i} t=${50+i*50} seq=1 rx=40 data=2,3,0,0,0,0,0,60,4,8,20,0,0,50,80,70,0,20,3 io=10,11,12,13,100,101,102,103,20,30,40,50,8200,25,15,167936,3,3,0,0,255,40,0,8210,26,5,0,0,0,0,50,60,70,80`;
    model.receive(`${row} crc=${trackCrc(row)}`);
  }
  assert.equal(model.live.i,2);assert.equal(model.live.io[13],25);
  assert.equal(model.log.gaps,1);assert.equal(model.records.length,0);
  model.receive('OK grab track log_end id=2 count=3');assert.equal(model.log.enabled,false);
  assert.equal(scopedStopCommand('grab track log stop'),false);
});

test('reconnecting to live logging accepts future rows without inventing early capture',()=>{
  const model=new TrackingPanelModel();
  model.receive('OK grab track log_status enabled=1 id=7 period_ms=50 count=100');
  const row='TRACK live id=7 i=99 t=5000 seq=20 rx=4990 data=2,3,0,0,0,0,0,0,0,0,20,0,0,50,80,70,0,20,1';
  model.receive(`${row} crc=${trackCrc(row)}`);
  assert.equal(model.live.i,99);assert.equal(model.log.gaps,0);assert.equal(model.log.received,1);
  model.receive('OK grab track log_begin id=7 period_ms=50 next=0'); // MCU reset may reuse id.
  assert.equal(model.live,null);assert.equal(model.log.received,0);assert.equal(model.log.next,0);
});

test('tracking parameters are independent and explicit readbacks',()=>{
  assert.equal(TRACK_PRESET.body_speed,160);assert.equal(TRACK_PRESET.x_speed,60);
  assert.equal(TRACK_PRESET.control_ms,20);assert.equal(TRACK_PRESET.batch_ms,0);
  const model=new TrackingPanelModel();
  assert.equal(model.config.size,0);
  model.receive('OK grab track config key=x_speed value=55');
  assert.equal(model.config.get('x_speed'),55);
});
test('tracking reports and trace preserve numeric evidence',()=>{
  const report=parseTrack('OK grab track state=track_follow reason=track_following valid=1 dx=-2 dy=4 vf=63.2 vl=-12 vx=20 x=32 room=41 xr=32 stop_requested=0 stop_confirmed=0 seq=7');
  assert.equal(report.vf,63.2);assert.equal(trackText(report.state),'稳定跟随');
  assert.equal(parseTrack('OK grab state=align'),null);
  const model=new TrackingPanelModel();
  model.receive('OK grab track trace_begin count=1');
  const row='TRACK row i=0 t=50 seq=1 rx=40 data=2,3,0,0,0,0,0,60,4,8,20,0,0,50,80,70,0,20,3';
  model.receive(`${row} crc=${trackCrc(row)}`);
  model.receive('OK grab track trace_end');
  assert.equal(model.complete,true);assert.equal(model.records.length,1);
  model.receive(row.replace('data=2,3','data=2,9')+` crc=${trackCrc(row)}`);
  assert.equal(model.records.length,1);assert.equal(model.records[0].e,'2,3');
});
test('only status/get/frozen trace are read-only; track stop is priority',()=>{
  for(const wire of ['grab track status','grab track trace','grab track trace 7','grab track get x_speed'])assert.equal(readOnlyCommand(wire),true);
  for(const wire of ['grab track observe','grab track reference','grab track start','grab track set body_speed 200'])assert.equal(readOnlyCommand(wire),false);
  assert.equal(scopedStopCommand('grab track stop'),true);
});

function surface(){
  class Element{
    constructor(tag){this.tag=tag;this.children=[];this.dataset={};this.textContent='';}
    append(...items){this.children.push(...items);}
    replaceChildren(...items){this.children=items;}
    setAttribute(){}
    click(){downloads++;}
  }
  const saved=globalThis.document;let downloads=0;
  globalThis.document={createElement:tag=>new Element(tag)};
  const root=new Element('div'),all=()=>{const walk=e=>[e,...e.children.flatMap(walk)];return walk(root);};
  return{root,all,downloads:()=>downloads,restore(){globalThis.document=saved;}};
}
test('mount/reconnect are passive; one-click entry and preparation are explicit',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);panel.connection(false);panel.connection(true);panel.update();
    assert.deepEqual(wires,[]);
    assert.equal(ui.all().some(e=>e.textContent==='仅观察'),false);
    assert.equal(ui.all().some(e=>e.textContent==='建立料台参考'),false);
    await ui.all().find(e=>e.title==='grab pose work').onclick();
    panel.receive('arm> OK grab state=route_pose operation=pose reason=preparing_work');
    assert.equal(ui.all().some(e=>e.textContent==='准备抓取姿态 · 等待 Z/X/Base 到位'),true);
    panel.receive('OK grab state=complete operation=pose reason=work_ready');
    assert.equal(ui.all().some(e=>e.textContent==='抓取姿态已到位 · 可以启动追踪'),true);
    await ui.all().find(e=>e.title==='grab track begin').onclick();
    assert.deepEqual(wires,['grab pose work','grab track begin 3 50']);
    panel.receive('OK grab track state=track_observe reason=track_observing');
    panel.connection(false);assert.equal(panel.model.current,false);
    panel.connection(true);assert.equal(wires.length,2);
    await ui.all().find(e=>e.title==='grab track pick').onclick();
    assert.equal(wires.at(-1),`grab track pick 3 -80 ${GRAB_FORM_DEFAULTS.z_pick_speed}`);
    await ui.all().find(e=>e.title==='grab track stop').onclick();
    assert.equal(wires.at(-1),'grab track stop');
  }finally{ui.restore();}
});
test('rejected or disconnected trace request cannot trigger an unrelated later download',async()=>{
  const ui=surface();
  try{
    const panel=mountTracking({root:ui.root,send:()=>true,canSend:()=>true});panel.connection(true);
    await ui.all().find(e=>e.title==='grab track trace').onclick();
    panel.receive('ERR grab track trace_running; stop first');
    panel.receive('OK grab track trace_end');
    assert.equal(ui.downloads(),0);
    await ui.all().find(e=>e.title==='grab track trace').onclick();panel.connection(false);
    panel.receive('OK grab track trace_end');assert.equal(ui.downloads(),0);
  }finally{ui.restore();}
});
test('frozen export repairs a missing row before downloading, without motion commands',async()=>{
  const ui=surface(),wires=[];let panel;
  const rows=[0,1].map(i=>`TRACK row i=${i} t=${50+i*50} seq=${i+1} rx=${40+i*50} data=2,3,0,0,0,0,0,60,4,8,20,0,0,50,80,70,0,20,3`);
  try{
    panel=mountTracking({root:ui.root,send:wire=>{
      wires.push(wire);
      if(wire==='grab track trace 1')queueMicrotask(()=>panel.receive(`${rows[1]} crc=${trackCrc(rows[1])}`));
      return true;
    },canSend:()=>true});panel.connection(true);
    await ui.all().find(e=>e.title==='grab track trace').onclick();
    panel.receive('OK grab track trace_begin count=2');
    panel.receive(`${rows[0]} crc=${trackCrc(rows[0])}`);
    assert.equal(panel.model.complete,false);assert.equal(ui.downloads(),0);
    panel.receive('OK grab track trace_end');
    await new Promise(resolve=>setImmediate(resolve));
    assert.deepEqual(wires,['grab track trace','grab track trace 1']);
    assert.equal(panel.model.complete,true);assert.equal(ui.downloads(),1);
  }finally{panel?.connection(false);ui.restore();}
});
