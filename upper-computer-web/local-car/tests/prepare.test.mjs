import test from 'node:test';
import assert from 'node:assert/strict';
import {RoutePreparation,validCarConfig} from '../route-preparation.mjs';
const config={profile:'receive',units_per_rev:65536,directions_confirmed:true};

function fixture() {
  let now=0, snapshot=null;
  const wires=[];
  const p=new RoutePreparation({send:async wire=>{wires.push(wire);return true;},now:()=>now,
    beginTelemetry:nonce=>{assert.equal(nonce,42);snapshot=null;},telemetry:()=>snapshot,nonce:()=>42,getConfig:()=>config});
  p.connection(true);
  function reply(...lines){p.receive(`arm> ${wires.at(-1)}`);for(const line of lines)p.receive(line);}
  function imu(ready,run=1){reply('OK imu ready=1 valid=1 fresh=1 yaw_deg=0',
    `OK imu cal_state=DONE reason=none HAL=0 run_id=${run} busy=0 verified=${ready} control_ready=${ready} normal_mode=1`,
    'OK imu relative_deg=0 drift_dps=0 rms_deg=0');}
  function throughFeedback(){p.start();reply('OK chassis request accepted (not motion/ACK confirmation)');
    reply('OK chassis task state=0 reason=idle dt_ms=0 seq=0 stage=5 error=0 ack=2 locked=0 units=65536');
    reply('OK chassis route state=idle segment=0 id=0 reason=idle feedback_valid=0 stop_confirmed=0');
    reply('OK chassis request accepted (not motion/ACK confirmation)');
    reply('OK chassis request accepted (not motion/ACK confirmation)');
    reply('OK chassis task state=0 reason=idle dt_ms=0 seq=0 stage=5 error=0 ack=2 locked=0 units=65536');
    reply('OK chassis request accepted (not motion/ACK confirmation)');
    reply('OK chassis request accepted (not motion/ACK confirmation)');
    now+=2100;p.tick();
    reply(...[1,2,3,4].map(i=>`OK wheel=${i} raw_rpm=0 raw_units=0 flags=0x03 valid=1,1,1 rx_ms=20,20,20`));}
  return {p,wires,reply,imu,throughFeedback,advance:ms=>{now+=ms;p.tick();},setSnapshot:v=>{snapshot=v;}};
}
function goodSnapshot(){return {session:42,feedback:{x_mm:2250,y_mm:150,yaw_rad:Math.PI/2},state:{t_ms:100,
  localization:{origin_valid:true},task:{state:'idle'},route:{state:'idle'},target:{rpm:[0,0,0,0]},
  feedback:{rpm:[0,0,0,0],speed_valid:[true,true,true,true],speed_ms:[90,90,90,90]},tx:{error:0}}};}

test('prepare performs only setup, uses ten-second verification and requires physical origin confirmation',()=>{
  const f=fixture();f.throughFeedback();f.imu(0);assert.equal(f.wires.at(-1),'imu verify 10');
  f.reply('OK imu verify started cal_ms=0 verify_ms=10000 keep_still=1 stream=off');
  f.advance(10000);assert.equal(f.wires.at(-1),'imu verify 10');
  f.advance(1500);assert.equal(f.wires.at(-1),'imu status');f.imu(1,2);
  assert.equal(f.p.phase,'confirm');assert.ok(!f.wires.some(w=>w.startsWith('chassis origin')));
  f.p.confirmOrigin();assert.equal(f.wires.at(-1),'chassis origin 2250 150 90');
  f.reply('OK chassis request accepted (not motion/ACK confirmation)');assert.equal(f.wires.at(-1),'chassis stream on 42');
  f.reply('OK chassis request accepted (not motion/ACK confirmation)');f.setSnapshot(goodSnapshot());f.p.tick();
  assert.equal(f.p.phase,'ready');assert.equal(f.wires.filter(w=>w==='imu verify 10').length,1);
  assert.ok(f.wires.every(w=>!/^wheel |^chassis (route (start|next)|move|run|heading)|^enable/.test(w)));
});

test('existing verified IMU skips verification; connection changes never resume preparation',()=>{
  const f=fixture();f.throughFeedback();f.imu(1);assert.equal(f.p.phase,'confirm');
  assert.ok(!f.wires.includes('imu verify 10'));const n=f.wires.length;
  f.p.connection(false);f.p.confirmOrigin();f.p.connection(true);f.advance(40000);assert.equal(f.wires.length,n);
});

test('saved configuration is applied after idle checks; busy route, missing echo and delayed response cannot advance',()=>{
  const f=fixture();f.p.start();f.p.receive('OK chassis request accepted (not motion/ACK confirmation)');
  assert.equal(f.wires.length,1);f.reply('OK chassis request accepted (not motion/ACK confirmation)');
  f.reply('OK chassis task state=0 reason=idle dt_ms=0 seq=0 stage=5 error=0 ack=2 locked=0 units=0');
  assert.equal(f.p.phase,'route');f.reply('OK chassis route state=idle segment=0');
  assert.equal(f.wires.at(-1),'chassis profile receive');
  f.reply('OK chassis request accepted (not motion/ACK confirmation)');assert.equal(f.wires.at(-1),'chassis units 65536');
  f.reply('OK chassis request accepted (not motion/ACK confirmation)');assert.equal(f.wires.at(-1),'chassis task');
  f.reply('OK chassis task state=0 stage=5 ack=2 locked=0 units=0');assert.equal(f.p.phase,'failed');
  const n=f.wires.length;f.p.receive('OK chassis task state=0 ack=2 locked=0 units=65536');assert.equal(f.wires.length,n);
  const g=fixture();g.p.start();g.reply('OK chassis request accepted (not motion/ACK confirmation)');
  g.reply('OK chassis task state=0 reason=idle stage=5 error=0 ack=2 locked=0 units=65536');
  g.reply('OK chassis route state=waiting segment=1');assert.equal(g.p.phase,'failed');
});

test('cancellation, send failure and timeout never progress to origin',async()=>{
  const f=fixture();f.throughFeedback();f.imu(0);f.reply('OK imu verify started cal_ms=0 verify_ms=10000 keep_still=1 stream=off');
  f.p.cancel();const n=f.wires.length;f.advance(20000);f.imu(1,2);assert.equal(f.wires.length,n);
  const g=fixture();g.p.start();g.advance(12001);assert.equal(g.p.phase,'failed');
  const p=new RoutePreparation({send:async()=>false,getConfig:()=>config});p.connection(true);p.start();await Promise.resolve();assert.equal(p.phase,'failed');
});

test('Emm preset supplies protocol constants without a measurement checkbox; wheel direction remains required',()=>{
  for(const c of [null,{}, {...config,directions_confirmed:false}]){
    assert.equal(validCarConfig(c),false);const wires=[];const p=new RoutePreparation({send:w=>wires.push(w),getConfig:()=>c});p.connection(true);
    assert.equal(p.start(),false);assert.equal(wires.length,0);
  }
  const f=fixture();f.p.getConfig=()=>({profile:'none',units_per_rev:null,directions_confirmed:true});
  assert.equal(validCarConfig(f.p.getConfig()),true);
  f.throughFeedback();
  assert.ok(f.wires.includes('chassis profile receive'));
  assert.ok(f.wires.includes('chassis units 65536'));
  assert.equal(f.p.phase,'imu');
});

test('stop while origin confirmation waits prevents delayed replies and timer from starting any new request',()=>{
  const f=fixture();f.throughFeedback();f.imu(1);assert.equal(f.p.phase,'confirm');const count=f.wires.length;
  f.p.cancel('停止已锁定');assert.equal(f.p.confirmOrigin(),false);f.advance(20000);f.imu(1);assert.equal(f.wires.length,count);
});

test('failed or wrong verification and stale telemetry cannot mark ready',()=>{
  const f=fixture();f.throughFeedback();f.imu(0);f.reply('OK imu verify started cal_ms=0 verify_ms=30000 keep_still=1 stream=off');
  assert.equal(f.p.phase,'failed');
  const g=fixture();g.throughFeedback();g.imu(1);g.p.confirmOrigin();g.reply('OK chassis request accepted (not motion/ACK confirmation)');
  g.reply('OK chassis request accepted (not motion/ACK confirmation)');const r=goodSnapshot();r.session=43;g.setSnapshot(r);g.p.tick();assert.notEqual(g.p.phase,'ready');
  r.session=42;r.state.feedback.rpm[2]=2;g.p.tick();assert.notEqual(g.p.phase,'ready');
  g.setSnapshot(goodSnapshot());g.p.tick();assert.equal(g.p.phase,'ready');g.setSnapshot(null);g.p.tick();assert.notEqual(g.p.phase,'ready');
});
