import {GRAB_FORM_DEFAULTS} from '../grab-params.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {parsePixel,parseTrack,TRACK_PRESET,trackText,TrackingPanelModel,mountTracking,trackCrc} from '../grab-track.mjs';
import {readOnlyCommand,scopedStopCommand} from '../bridge.mjs';

test('tracking faults use Chinese reasons while tracking protocol remains English',()=>{
  const raw='ERR grab track state=error reason=feedback_stale';
  const status=parseTrack(raw);
  assert.match(trackText(status.reason),/反馈.*过期/);
  assert.match(trackText('bus_unavailable'),/总线.*不可用/);
  assert.match(trackText('reference_missing'),/参考.*未建立/);
  assert.equal(status.reason,'feedback_stale');assert.equal(status.raw,raw);
  assert.match(trackText('track_pick_descend'),/50.*下降.*追踪/);
  assert.match(trackText('track_pick_descend_wait'),/50.*暂停.*Z/);
  assert.match(trackText('track_pick_close'),/夹取高度.*闭爪/);
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
    append(...items){this.children.push(...items);for(const item of items)item.parentElement=this;}
    replaceChildren(...items){this.children=items;}
    setAttribute(){}
    click(){downloads++;}
  }
  const saved=globalThis.document;let downloads=0;
  globalThis.document={createElement:tag=>new Element(tag)};
  const root=new Element('div'),all=()=>{const walk=e=>[e,...e.children.flatMap(walk)];return walk(root);};
  return{root,all,downloads:()=>downloads,restore(){globalThis.document=saved;}};
}
test('pose rejection points to an explicit task status query without sending or retrying automatically',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);
    const query=ui.all().find(e=>e.tag==='button'&&e.textContent==='读取抓取任务状态');
    assert.ok(query,'preparation needs a clearly named task status button');
    assert.equal(query.title,'grab status');
    assert.deepEqual(wires,[]);
    const raw='ERR grab pose rejected; inspect grab status';
    assert.equal(panel.receive(raw),true);
    assert.deepEqual(wires,[],'receiving a rejection must stay passive');
    assert.ok(ui.all().some(e=>e.textContent.includes('视觉任务')&&e.textContent.includes('点击“读取抓取任务状态”')));
    const feedback=ui.all().find(e=>e.tag==='a'&&e.textContent==='查看抓取任务状态反馈 →');
    assert.equal(feedback?.href,'#vision/grab/status');
    await query.onclick();
    assert.deepEqual(wires,['grab status']);
    panel.connection(false);panel.connection(true);
    assert.deepEqual(wires,['grab status']);
  }finally{ui.restore();}
});
test('comparison presets restore speed with gain 2 and short lookahead while predict defaults remain independent',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);
    const profile=ui.all().find(e=>e.tag==='select'&&e.children.some(c=>c.value==='feedback')&&typeof e.onchange==='function');
    const input=key=>ui.all().find(e=>e.dataset?.trackKey===key);
    const preset=ui.all().find(e=>e.tag==='button'&&e.textContent==='填入追踪预设');
    for(const mode of ['visual','feedback']){
      profile.value=mode;profile.onchange();
      for(const [key,value]of Object.entries({body_speed:160,x_speed:60,lambda:2,horizon_ms:60,accel:300,body_accel:300,x_accel:120,tol_px:5,lead_mm:12,control_ms:20,batch_ms:0,grace_ms:250}))assert.equal(Number(input(key).value),value);
      input('lambda').value=9;await preset.onclick();assert.equal(Number(input('lambda').value),2);
    }
    profile.value='predict';profile.onchange();
    assert.equal(Number(input('body_speed').value),160);assert.equal(Number(input('x_speed').value),60);
    assert.equal(Number(input('lambda').value),2.5);assert.equal(Number(input('horizon_ms').value),120);
    assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});
test('comparison buttons explicitly start separate visual profiles and reconnect stays passive',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);
    const visual=ui.all().find(e=>e.title==='grab track visual');
    const feedback=ui.all().find(e=>e.title==='grab track feedback');
    assert.ok(visual && feedback);
    assert.deepEqual(wires,[]);
    await visual.onclick();assert.equal(wires.at(-1),'grab track visual 3 50');
    await feedback.onclick();assert.equal(wires.at(-1),'grab track feedback 3 50');
    panel.receive('OK grab track state=track_match reason=track_matching mode=feedback');
    assert.ok(ui.all().some(e=>e.textContent.includes('纯视觉＋反馈减速')));
    const count=wires.length;panel.connection(false);panel.connection(true);assert.equal(wires.length,count);
  }finally{ui.restore();}
});
test('profile readbacks and edits stay separate, including under the stop latch',async()=>{
  for(const wire of ['grab track get x_speed visual','grab track get x_accel feedback'])assert.equal(readOnlyCommand(wire),true);
  for(const wire of ['grab track visual 3 50','grab track feedback 3 50','grab track get x_accel unknown'])assert.equal(readOnlyCommand(wire),false);
  const model=new TrackingPanelModel();
  model.receive('OK grab track config key=x_speed value=60 profile=predict');
  model.receive('OK grab track config key=x_speed value=25 profile=visual');
  assert.equal(model.config.get('x_speed'),60);assert.equal(model.config.get('visual.x_speed'),25);
  const ui=surface(),wires=[];let panel;
  try{
    panel=mountTracking({root:ui.root,send:wire=>{
      wires.push(wire);const m=wire.match(/^grab track (get|set) (\w+)(?: ([\d.]+))? feedback$/);
      if(m)queueMicrotask(()=>panel.receive(`OK grab track config key=${m[2]} value=${m[3]??(m[2]==='x_accel'?120:TRACK_PRESET[m[2]]??300)} profile=feedback`));return true;
    },canSend:()=>true});panel.connection(true);
    const select=ui.all().find(e=>e.tag==='select'&&e.children.some(o=>o.value==='feedback')&&typeof e.onchange==='function');
    select.value='feedback';select.onchange();assert.deepEqual(wires,[]);
    await ui.all().find(e=>e.textContent==='读取全部').onclick();
    assert.ok(wires.includes('grab track get x_accel feedback'));
    assert.ok(wires.every(w=>w.endsWith(' feedback')));
    assert.ok(!wires.some(w=>/fit_ms|latency_ms/.test(w)));
  }finally{panel?.connection(false);ui.restore();}
});
test('record start saves actual parameters before subscribing; reconnect stays passive',async()=>{
  const ui=surface(),wires=[];let panel;
  try{
    panel=mountTracking({root:ui.root,send:wire=>{
      wires.push(wire);
      const key=wire.match(/^grab track get (\w+)$/)?.[1];
      if(key)queueMicrotask(()=>panel.receive(`OK grab track config key=${key} value=${TRACK_PRESET[key]}`));
      return true;
    },canSend:()=>true});
    panel.connection(true);
    await ui.all().find(e=>e.textContent==='开启实时记录').onclick();
    assert.equal(wires.filter(w=>w.startsWith('grab track get ')).length,13);
    assert.equal(wires.at(-1),'grab track log start 200');
    assert.equal(wires.at(-2),'camera log start');
    assert.equal(wires.some(w=>/^grab track (begin|start|pick)/.test(w)),false);
    const count=wires.length;panel.connection(false);panel.connection(true);
    assert.equal(wires.length,count);
  }finally{ui.restore();}
});

test('a missing parameter reply does not prevent recording useful diagnostics',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return !wire.startsWith('grab track get ');},canSend:()=>true});
    panel.connection(true);
    await ui.all().find(e=>e.textContent==='开启实时记录').onclick();
    assert.equal(wires.at(-1),'grab track log start 200');
    assert.ok(ui.all().some(e=>e.textContent.includes('参数读回不完整')));
  }finally{ui.restore();}
});

test('pixel logs keep signed camera errors independent of the reference coordinates',()=>{
  const body='PIX id=1 i=0 t=1 c=3 v=1 m=1 x=334 y=338 dx=-38 dy=7';
  const pixel=parsePixel(`${body} crc=${trackCrc(body)}`);
  assert.equal(pixel.dx,-38);assert.equal(pixel.dy,7);
  for(const suffix of ['dx=1','dx=-32769 dy=0','dx=0 dy=32768']){
    const line=`PIX id=1 i=1 t=2 c=3 v=1 m=1 x=372 y=345 ${suffix}`;
    assert.equal(parsePixel(`${line} crc=${trackCrc(line)}`),null);
  }
});

test('tracking keeps Base approximate-reference warning visible',()=>{
  const ui=surface();
  try{
    const panel=mountTracking({root:ui.root,send:()=>true,canSend:()=>true});
    panel.receive('OK grab track state=track_follow reason=track_following mode=visual base_home_degraded=1');
    assert.ok(ui.all().some(e=>e.textContent.includes('Base 使用降级参考')));
    assert.equal(parseTrack('OK grab track state=track_follow base_home_degraded=2'),null);
  }finally{ui.restore();}
});

test('pickup window and hold are explicit, confirmed before one motion command, and passive on reconnect',async()=>{
  const ui=surface(),wires=[],values=new Map([['track_pick_tol_px',5],['track_pick_hold_ms',200]]);let panel;
  try{
    panel=mountTracking({root:ui.root,send:wire=>{
      wires.push(wire);const set=wire.match(/^grab set (track_pick_\w+) ([\d.]+)$/),get=wire.match(/^grab get (track_pick_\w+)$/);
      if(set){values.set(set[1],Number(set[2]));queueMicrotask(()=>panel.receive('OK grab set'));}
      if(get)queueMicrotask(()=>panel.receive(`OK grab config key=${get[1]} value=${values.get(get[1])}`));
      return true;
    },canSend:()=>true});panel.connection(true);
    const input=key=>ui.all().find(e=>e.dataset?.pickupKey===key);
    assert.equal(Number(input('track_pick_tol_px').value),100);assert.equal(Number(input('track_pick_hold_ms').value),10);
    assert.deepEqual(wires,[]);
    input('track_pick_tol_px').value=8;input('track_pick_hold_ms').value=300;
    await ui.all().find(e=>e.title==='grab track pick').onclick();
    assert.deepEqual(wires.slice(0,4),['grab set track_pick_tol_px 8','grab get track_pick_tol_px','grab set track_pick_hold_ms 300','grab get track_pick_hold_ms']);
    assert.equal(wires.filter(w=>w.startsWith('grab track pick')).length,1);
    await ui.all().find(e=>e.textContent==='读取夹取条件').onclick();
    assert.deepEqual(wires.slice(-2),['grab get track_pick_tol_px','grab get track_pick_hold_ms']);
    const count=wires.length;panel.connection(false);panel.connection(true);panel.update();assert.equal(wires.length,count);
    input('track_pick_hold_ms').value='1.5';
    await ui.all().find(e=>e.title==='grab track pick').onclick();assert.equal(wires.length,count);
  }finally{panel?.connection(false);ui.restore();}
});

test('integrated pickup stores settings before a single firmware-owned transfer and reconnect stays passive',async()=>{
  const ui=surface(),wires=[],values=new Map();let panel;
  try{
    panel=mountTracking({root:ui.root,canSend:()=>true,send:wire=>{
      wires.push(wire);const set=wire.match(/^grab set (\w+) ([\d.-]+)$/),get=wire.match(/^grab get (\w+)$/);
      if(set){values.set(set[1],Number(set[2]));queueMicrotask(()=>panel.receive('OK grab set'));}
      if(get)queueMicrotask(()=>panel.receive(`OK grab config key=${get[1]} value=${values.get(get[1])}`));
      return true;
    }});panel.connection(true);
    assert.deepEqual(wires,[]);
    const slot=ui.all().find(e=>e.dataset?.storeSlot);slot.value='2';
    const lift=ui.all().find(e=>e.dataset?.transferKey==='lift');lift.value='40';
    await ui.all().find(e=>e.title==='grab track pickstore').onclick();
    assert.equal(values.get('z_grab'),-50);assert.equal(values.get('z_lift'),-10);
    assert.equal(values.get('track_store_drop_mm'),40);
    assert.equal(wires.at(-1),'grab track pickstore 3 2 feedback');
    assert.equal(wires.filter(w=>/^grab track (?:pickstore|store)/.test(w)).length,1);
    const count=wires.length;panel.connection(false);panel.connection(true);assert.equal(wires.length,count);
    panel.receive('OK grab state=car_index mode=fixed operation=store slot=2 result=pending reason=running');
    await ui.all().find(e=>e.title==='grab track stop').onclick();assert.equal(wires.at(-1),'grab stop');
  }finally{panel?.connection(false);ui.restore();}
});

test('integrated transfer cancels late height confirmation after stop, disconnect or mismatch',async()=>{
  for(const scenario of ['reject','mismatch','stop','disconnect']){
    const ui=surface(),wires=[],values=new Map();let panel;
    try{
      panel=mountTracking({root:ui.root,canSend:()=>true,send:wire=>{
        wires.push(wire);const m=wire.match(/^grab (set|get) (\w+)(?: (-?[\d.]+))?$/);
        if(m)queueMicrotask(()=>{
          if(m[1]==='set'){
            values.set(m[2],Number(m[3]));
            if(m[2]==='z_lift'&&scenario==='reject'){panel.receive('ERR grab invalid_parameter');return;}
            panel.receive('OK grab set');
          }else{
            if(m[2]==='z_lift'&&scenario==='stop')ui.all().find(e=>e.title==='grab track stop').onclick();
            if(m[2]==='z_lift'&&scenario==='disconnect')panel.connection(false);
            panel.receive(`OK grab config key=${m[2]} value=${m[2]==='z_lift'&&scenario==='mismatch'?-10:values.get(m[2])}`);
          }
        });return true;
      }});panel.connection(true);
      await ui.all().find(e=>e.title==='grab track pickstore').onclick();
      assert.ok(wires.includes('grab set z_lift 0'));
      assert.equal(wires.some(w=>/^grab track (?:pickstore|store)/.test(w)),false,scenario);
      if(scenario==='stop'||scenario==='disconnect')assert.equal(wires.includes('grab set track_store_drop_mm 40'),false);
    }finally{panel?.connection(false);ui.restore();}
  }
});

test('pickup confirmation rejection, mismatch, stop and disconnect never launch pickup',async()=>{
  for(const scenario of ['reject','mismatch','stop','disconnect']){
    const ui=surface(),wires=[];let panel;
    try{
      panel=mountTracking({root:ui.root,canSend:()=>true,send:wire=>{
        wires.push(wire);
        if(wire.startsWith('grab set track_pick_'))queueMicrotask(()=>{
          if(scenario==='reject')panel.receive('ERR grab invalid_parameter');
          else if(scenario==='stop')ui.all().find(e=>e.title==='grab track stop').onclick();
          else if(scenario==='disconnect')panel.connection(false);
          else panel.receive('OK grab set');
        });
        if(wire==='grab get track_pick_tol_px')queueMicrotask(()=>panel.receive('OK grab config key=track_pick_tol_px value=9'));
        return true;
      }});panel.connection(true);
      await ui.all().find(e=>e.title==='grab track pick').onclick();
      panel.connection(true);panel.update();
      assert.equal(wires.some(w=>w.startsWith('grab track pick')),false,scenario);
      if(scenario==='stop')assert.equal(wires.at(-1),'grab track stop');
    }finally{panel?.connection(false);ui.restore();}
  }
});

test('pickup parameter reads remain available in stop lock, writes and start are disabled',async()=>{
  const ui=surface();let panel;
  try{
    panel=mountTracking({root:ui.root,send:()=>true,canSend:readOnlyCommand});panel.connection(true);
    assert.equal(ui.all().find(e=>e.textContent==='读取夹取条件').disabled,false);
    assert.equal(ui.all().find(e=>e.textContent==='应用夹取条件并核对').disabled,true);
    assert.equal(ui.all().find(e=>e.title==='grab track pick').disabled,true);
  }finally{panel?.connection(false);ui.restore();}
});

test('mount/reconnect are passive; one-click entry and preparation are explicit',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{
      wires.push(wire);
      if(/^grab set track_pick_/.test(wire))queueMicrotask(()=>panel.receive('OK grab set'));
      const key=wire.match(/^grab get (track_pick_\w+)$/)?.[1];
      if(key)queueMicrotask(()=>panel.receive(`OK grab config key=${key} value=${GRAB_FORM_DEFAULTS[key]}`));
      return true;
    },canSend:()=>true});
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
    assert.equal(wires.at(-1),`grab track pick 3 ${GRAB_FORM_DEFAULTS.z_grab} ${GRAB_FORM_DEFAULTS.z_pick_speed}`);
    await ui.all().find(e=>e.title==='grab track stop').onclick();
    assert.equal(wires.at(-1),'grab track stop');
  }finally{ui.restore();}
});
test('pose reference verification and cancellation keep status and stop on the pose workflow',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);
    await ui.all().find(e=>e.title==='grab pose work').onclick();
    panel.receive('OK grab state=homing operation=pose reason=base_reference_verifying home_scope=base');
    assert.ok(ui.all().some(e=>e.textContent==='准备抓取姿态 · 确认 Base 回零与停稳反馈'));
    assert.ok(ui.all().some(e=>e.textContent==='停止准备'));
    await ui.all().find(e=>e.title==='grab track status').onclick();
    await ui.all().find(e=>e.title==='grab track stop').onclick();
    /* Boot verification drains/aborts while its actual state remains homing. */
    panel.receive('OK grab state=homing operation=pose reason=home_cancelled stop_requested=1 stop_confirmed=0');
    assert.ok(ui.all().some(e=>e.textContent==='姿态准备停止中 · 回零已取消 · 等待回零退出确认'));
    panel.receive('OK grab state=stopping operation=pose reason=cancelled stop_confirmed=0');
    await ui.all().find(e=>e.title==='grab track status').onclick();
    assert.deepEqual(wires,['grab pose work','grab status','grab stop','grab status']);
    panel.receive('OK grab state=idle operation=pose reason=cancelled stop_confirmed=1');
    await ui.all().find(e=>e.title==='grab pose work').onclick();
    assert.equal(wires.at(-1),'grab pose work');
    panel.receive('OK grab state=complete operation=pose reason=work_ready');
    assert.ok(ui.all().some(e=>e.textContent==='抓取姿态已到位 · 可以启动追踪'));
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

test('per-frame pixels retain missed detections, validate CRC and report gaps without duplicates',()=>{
  const model=new TrackingPanelModel();
  model.receive('OK camera log_begin id=1 next=0 mode=per_frame');
  for(const [i,v,m] of [[0,1,1],[1,0,1],[3,1,0]]){
    const body=`PIX id=1 i=${i} t=${17*i} c=3 v=${v} m=${m} x=334 y=338`;
    const line=`${body} crc=${trackCrc(body)}`;
    assert.equal(parsePixel(line).v,v);model.receive(line);model.receive(line);
    assert.equal(parsePixel(line.replace('x=334','x=335')),null);
  }
  model.receive('OK camera log_end id=1 count=5 queued=4 dropped=1');
  assert.equal(model.pixels.received,3);assert.equal(model.pixels.gaps,2);
  assert.equal(model.pixels.enabled,false);
  for(const command of ['camera log start','camera log stop','camera log status'])assert.equal(readOnlyCommand(command),true);
});

test('pixel-only recording and closing recording never start or stop motion',async()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return !wire.startsWith('grab track get ');},canSend:()=>true});
    panel.connection(true);
    ui.all().find(e=>e.type==='checkbox').checked=false;
    await ui.all().find(e=>e.textContent==='开启实时记录').onclick();
    assert.deepEqual(wires.slice(-2),['camera log start','grab track log stop']);
    await ui.all().find(e=>e.textContent==='关闭实时记录').onclick();
    assert.deepEqual(wires.slice(-2),['grab track log stop','camera log stop']);
    assert.equal(wires.includes('grab stop'),false);
  }finally{ui.restore();}
});

test('unified material panel retains tracking and logging in collapsed groups without I/O',()=>{
  const ui=surface(),wires=[];
  try{
    const panel=mountTracking({root:ui.root,send:wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);
    for(const title of ['单独追踪与夹取','独立追踪参数 · 显式写入并读回','追踪记录与诊断']){
      const summary=ui.all().find(e=>e.tag==='summary'&&e.textContent===title);
      assert.ok(summary);assert.equal(summary.parentElement.tag,'details');assert.ok(!summary.parentElement.open);
    }
    for(const wire of ['grab pose work','grab track pickstore','grab track store','grab track begin','grab track visual','grab track feedback','grab track pick','grab track stop','grab track log start'])
      assert.equal(ui.all().filter(e=>e.tag==='button'&&e.title===wire).length,1,wire);
    assert.equal(ui.all().filter(e=>e.dataset?.storeSlot).length,1);
    assert.equal(ui.all().find(e=>e.title==='grab track pickstore').className,'primary');
    panel.connection(false);panel.connection(true);assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});
