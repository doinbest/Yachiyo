import test from 'node:test';
import assert from 'node:assert/strict';
import {ChassisTestSession,mountChassisTestSession} from '../chassis-test-session.mjs';

const ack='OK chassis request accepted (not motion/ACK confirmation)';
const task=(extra='')=>`OK chassis task state=0 reason=idle dt_ms=0 seq=0 stage=0 error=0 ack=2 locked=0 units=65536 mode=0 target_deg=0 yaw_deg=0 yaw_valid=1 omega=0 origin_valid=1 origin_mm=0,0 origin_deg=0 feedback_valid=1 feedback_mm=0,0 feedback_deg=0 feedback_age_ms=20 ${extra}`.trim();
const wheels=(stamp=100,valid='1,1,1',rpm=0)=>[1,2,3,4].map(i=>`OK wheel=${i} raw_rpm=${rpm} raw_units=0 flags=0x01 valid=${valid} rx_ms=${stamp},${stamp},${stamp}`);
const imu=['OK imu ready=1 valid=1 fresh=1 yaw_deg=0','OK imu cal_state=VALID reason=verified HAL=0 run_id=1 busy=0 verified=1 control_ready=1 normal_mode=1','OK imu relative_deg=0 drift_dps=0 rms_deg=0 save_requested=0 save_readback_ok=0'];
const result=(state=3,stopped=1,id=7)=>`OK chassis test_result id=${id} state=${state} profile=cosine elapsed_ms=3000 peak_mm_s=490 error_mm=3 stop_ms=200 stop_confirmed=${stopped}`;
function fixture(options={}){
 let stamp=100,clock=0;const wires=[],events=[];
 const s=new ChassisTestSession({now:()=>clock,wait:async ms=>{clock+=ms;},
  acquire:async()=>events.push('acquire'),release:async()=>events.push('release'),
  stop:async()=>{events.push('stop');return true;},canStart:()=>true,
  send:async(wire,owned)=>{
   wires.push(wire);events.push(wire);
   const defaults=wire==='chassis task'?[task()]:wire==='chassis route status'?['OK chassis route state=idle reason=idle']:wire==='grab mission status'?['OK grab route state=idle mode=mission']:wire==='imu status'?imu:wire==='chassis feedback'?wheels(stamp+=100):wire.startsWith('chassis test ')?['OK chassis test reason=accepted id=7 profile=cosine distance_mm=1000 speed_mm_s=500; request only, arrival requires feedback']:[ack];
   const reply=options.reply?.(wire,defaults,{owned,s,wires})??defaults;
   queueMicrotask(()=>{s.receive(wire);for(const line of reply)s.receive(line);});
   return true;
  }});
 s.connection(true);
 return {s,wires,events,advance:ms=>{clock+=ms;}};
}
test('per-test acceleration is submitted directly and invalid acceleration sends nothing',async()=>{
 const f=fixture();assert.equal(await f.s.start('chassis test -1000 0 1000 linear 800'),true);
 assert.deepEqual(f.wires.filter(w=>w.startsWith('chassis test ')),['chassis test -1000 0 1000 linear 800']);
 for(const wire of ['chassis test 1000 0 1000 linear 0','chassis test 1000 0 1000 linear nan','chassis test 1000 0 1000 linear 5001','chassis test 1000 0 1000 cosine 0','chassis test 1000 0 1000 cosine 200 nan 0']){
  const bad=fixture();assert.equal(await bad.s.start(wire),false);assert.deepEqual(bad.wires,[]);
 }
});
test('start submits only one motion without preparation or acquiring a lease',async()=>{
 const f=fixture();assert.deepEqual(f.wires,[]);
 assert.equal(await f.s.start('chassis test 1000 0 500 cosine'),true);
 assert.deepEqual(f.wires,['chassis test 1000 0 500 cosine']);
 assert.equal(f.wires.filter(w=>w.startsWith('chassis test ')).length,1);
 assert.equal(f.wires.some(w=>w.startsWith('chassis origin ')),false);
 assert.equal(f.events.includes('acquire'),false);assert.equal(f.events.includes('release'),false);
 assert.equal(f.s.phase,'running');
 f.s.receive(result(1,0,99));assert.equal(f.s.phase,'running');
 f.s.receive(result());assert.equal(f.s.phase,'done');
});
test('missing reference is established only after fresh stopped feedback and read back',async()=>{
 let origin=false;
 const f=fixture({reply(wire,lines){if(wire==='chassis origin 0 0 0')origin=true;
  if(wire==='chassis task'&&!origin)return [task().replace('origin_valid=1','origin_valid=0').replace('feedback_valid=1','feedback_valid=0')];return lines;}});
 assert.equal(await f.s.prepare(),true);
 const i=f.wires.indexOf('chassis origin 0 0 0');assert.ok(i>f.wires.indexOf('chassis feedback on'));
 assert.ok(f.wires.slice(i+1).includes('chassis task'));
});
for(const [name,reply] of [
 ['stale wheel cache',(w,l)=>w==='chassis feedback'?wheels(100):l],
 ['wheel moving',(w,l)=>w==='chassis feedback'?wheels(100,'1,1,1',10):l],
 ['paused mission',(w,l)=>w==='grab mission status'?['OK grab route state=paused mode=mission']:l],
 ['unverified IMU',(w,l)=>w==='imu status'?imu.map(x=>x.replace('verified=1','verified=0').replace('control_ready=1','control_ready=0')):l]
])test(`${name} prevents motion and origin replacement`,async()=>{
 const f=fixture({reply});assert.equal(await f.s.prepare(),false);
 assert.equal(f.wires.some(w=>/^chassis (test|origin) /.test(w)),false);assert.equal(f.s.phase,'failed');
});
test('stop during feedback preparation cancels the eventual motion and waits for stop evidence',async()=>{
 const f=fixture({reply(w,l,{s}){if(w==='chassis feedback on')void s.stop();return l;}});
 assert.equal(await f.s.prepare(),false);
 assert.ok(f.events.includes('stop'));assert.equal(f.wires.some(w=>w.startsWith('chassis test ')),false);
 assert.notEqual(f.s.phase,'done');
 f.s.stopEvent({latched:true,status:'confirmed'});assert.equal(f.s.phase,'stopped');
});
test('disconnect cancels preparation; reconnect stays passive',async()=>{
 const f=fixture({reply(w,l,{s}){if(w==='imu status')s.connection(false);return l;}});
 assert.equal(await f.s.prepare(),false);
 assert.equal(f.wires.some(w=>w.startsWith('chassis test ')),false);
 const count=f.wires.length;f.s.connection(true);assert.equal(f.wires.length,count);
});
test('running test polls its result and does not treat unconfirmed completion as success',async()=>{
 const f=fixture();await f.s.start('chassis test 1000 0 500 cosine');
 f.s.receive(result(3,0));assert.notEqual(f.s.phase,'done');
 f.advance(2000);f.s.tick();await new Promise(r=>setTimeout(r,0));
 assert.equal(f.wires.at(-1),'chassis task');
 f.s.cancel();
});

test('missing feedback age cannot be accepted as fresh localization',async()=>{
 const f=fixture({reply:(w,l)=>w==='chassis task'?l.map(s=>s.replace('feedback_age_ms=20','')):l});
 assert.equal(await f.s.prepare(),false);
 assert.equal(f.wires.some(w=>w.startsWith('chassis test ')),false);
});
test('short invalid wheel feedback recovers within the preparation window',async()=>{
 let count=0;
 const f=fixture({reply:(w,l)=>w==='chassis feedback'&&++count<4?wheels(count*100,'0,0,0'):l});
 assert.equal(await f.s.prepare(),true);
 assert.equal(f.wires.some(w=>w.startsWith('chassis test ')),false);assert.equal(f.s.phase,'ready');
});
test('a stop arriving while the preparation lock releases prevents the final motion',async()=>{
 const f=fixture();f.s.release=async()=>{void f.s.stop();};
 assert.equal(await f.s.prepare(),false);
 assert.equal(f.wires.some(w=>w.startsWith('chassis test ')),false);
});
test('numeric spellings accepted by the form are normalized before a single motion',async()=>{
 for(const spelling of ['500.','+500','5e2']){
  const f=fixture();assert.equal(await f.s.start(`chassis test 1000 0 ${spelling} cosine`),true);
  assert.equal(f.wires.at(-1),'chassis test 1000 0 500 cosine');
 }
});

test('cosine K and PD are retained on direct submission',async()=>{
 const f=fixture();const wire='chassis test 2000 0 1000 cosine 120 0.1 0.01';
 assert.equal(await f.s.start(wire),true);
 assert.deepEqual(f.wires.filter(w=>w.startsWith('chassis test ')),[wire]);
});

test('independent curve period survives direct submission and rejects invalid values',async()=>{
 const f=fixture();assert.equal(await f.s.start('chassis test 1000 0 1000 cosine 200 0.02 0 5'),true);
 assert.deepEqual(f.wires.filter(w=>w.startsWith('chassis test ')),['chassis test 1000 0 1000 cosine 200 0.02 0 5']);
 for(const p of ['0','101','1.5','nan']){const g=fixture();assert.equal(await g.s.start('chassis test 1000 0 1000 linear 300 0.02 0 '+p),false);assert.deepEqual(g.wires,[]);}
});

test('consecutive completed tests submit once each without repeated status checks',async()=>{
 const f=fixture();const wire='chassis test 1000 0 500 cosine';
 assert.equal(await f.s.start(wire),true);f.s.receive(result());
 assert.equal(await f.s.start(wire),true);assert.deepEqual(f.wires,[wire,wire]);
 assert.equal(f.events.includes('acquire'),false);
});
test('device rejection is shown and does not automatically repair or replay motion',async()=>{
 const f=fixture({reply:(w,l)=>w.startsWith('chassis test ')?['ERR chassis test reason=feedback_invalid id=0 profile=cosine']:l});
 assert.equal(await f.s.start('chassis test 1000 0 500 cosine'),false);
 assert.deepEqual(f.wires,['chassis test 1000 0 500 cosine']);assert.equal(f.s.lastFault,'feedback_invalid');
 assert.equal(f.s.phase,'failed');assert.match(f.s.message,/设备拒绝/);
});
test('explicit status query is read-only and keeps reports labelled as past observations',async()=>{
 const f=fixture();assert.equal(await f.s.refresh(),true);assert.deepEqual(f.wires,['chassis task','imu status']);
 assert.equal(f.events.includes('acquire'),false);assert.equal(f.s.id,null);
 assert.match(f.s.statusEntries().find(([k])=>k==='定位反馈')[1],/最近有效/);
 f.advance(5000);assert.match(f.s.statusEntries().find(([k])=>k==='定位反馈')[1],/5秒前回报/);
 const n=f.wires.length;f.s.connection(false);f.s.connection(true);assert.equal(f.wires.length,n);
 assert.equal(f.s.statusEntries().find(([k])=>k==='定位反馈')[1],'尚未读取');
});
test('explicit preparation preserves a valid reference and never starts motion',async()=>{
 const f=fixture();assert.equal(await f.s.prepare(),true);assert.equal(f.s.phase,'ready');
 assert.equal(f.wires.some(w=>/^chassis (test|origin) /.test(w)),false);
 const n=f.wires.length;assert.equal(await f.s.start('chassis test 1000 0 500 cosine'),true);
 assert.deepEqual(f.wires.slice(n),['chassis test 1000 0 500 cosine']);
});
test('stopping or disconnecting while motion acceptance is pending cancels polling and never retries',async()=>{
 for(const mode of ['stop','disconnect']){
  const f=fixture({reply(w,l,{s}){if(w.startsWith('chassis test ')){if(mode==='stop')void s.stop();else s.connection(false);}return l;}});
  assert.equal(await f.s.start('chassis test 1000 0 500 cosine'),false);
  assert.deepEqual(f.wires,['chassis test 1000 0 500 cosine']);f.advance(2000);f.s.tick();assert.equal(f.wires.length,1);
 }
});

test('status panel shows observed faults and ages, mounts and reconnects without sends',()=>{
 const elements=[],wires=[];let clock=0,timer;
 const doc={createElement(tag){const e={tag,ownerDocument:doc,children:[],textContent:'',append(...a){this.children.push(...a);},setAttribute(){}};elements.push(e);return e;}};
 const root=doc.createElement('section'),original=globalThis.setInterval;globalThis.setInterval=fn=>{timer=fn;return 0;};
 try{
  const s=mountChassisTestSession({root,now:()=>clock,send:w=>{wires.push(w);return true;},stop:async()=>true});
  s.connection(true);assert.deepEqual(wires,[]);
  assert.equal(elements.find(e=>e.id==='chassis-test-status').textContent,'查询状态');
  assert.equal(elements.find(e=>e.id==='chassis-test-prepare').disabled,false);
  s.receive(task());assert.match(s.statusEntries().find(([k])=>k==='IMU')[1],/航向有效/);clock=5000;timer();
  assert.ok(elements.some(e=>e.tag==='dd'&&/最近有效（5秒前回报）/.test(e.textContent)));
  s.receive('ERR chassis test reason=feedback_invalid id=0 profile=linear');
  assert.ok(elements.some(e=>e.tag==='dd'&&e.textContent===s.statusEntries().find(([k])=>k==='最近拒绝')[1]));
  s.connection(false);assert.equal(elements.find(e=>e.id==='chassis-test-status').disabled,true);
  s.connection(true);timer();assert.deepEqual(wires,[]);
 }finally{globalThis.setInterval=original;}
});

test('asymmetric ramp parameters pass directly through without preparation',async()=>{
 const f=fixture(),wire='chassis test -2000 0 1000 linear 500 0.02 0 1 300';
 assert.equal(await f.s.start(wire),true);assert.deepEqual(f.wires,[wire]);
 for(const decel of ['0','5001','nan']){const b=fixture();assert.equal(await b.s.start('chassis test 1000 0 1000 linear 500 0.02 0 1 '+decel),false);assert.deepEqual(b.wires,[]);}
});
