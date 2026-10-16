import test from 'node:test';
import assert from 'node:assert/strict';
import {StopRecovery} from '../stop-recovery.mjs';

const ack='OK chassis request accepted (not motion/ACK confirmation)';
const task=(locked=0)=>`OK chassis task state=4 reason=cancelled stage=0 error=0 locked=${locked} units=65536 origin_valid=1 feedback_valid=1 yaw_valid=1 feedback_age_ms=20`;
const wheels=stamp=>[1,2,3,4].map(i=>`OK wheel=${i} raw_rpm=0 raw_units=0 valid=1,1,1 rx_ms=${stamp},${stamp},${stamp}`);
function fixture({locked=0,reply,afterResume}={}){
 const wires=[],events=[];let clock=0,stamp=100,unlocked=!locked;
 const bridge={connected:true,disconnecting:false,stopLatched:true,stopToken:7,commandEpoch:3,connectionRevision:1,
  async send(wire,owned){
   wires.push(wire);events.push(wire);
   const normal=wire==='chassis task'?[task(unlocked?0:1)]:wire==='bus recover'?['OK bus recover requested tasks_resumed=0','OK bus recover complete arm_ready=1 chassis_ready=1 wheels_stopped=1 tasks_resumed=0']:wire==='chassis feedback'?wheels(stamp+=100):[ack];
   const lines=reply?.(wire,normal,{s,bridge})??normal;
   if(wire==='bus recover')unlocked=true;
   queueMicrotask(()=>{s.receive(wire);for(const l of lines)s.receive(l);});return {};
  },
  async resume(){events.push('resume');this.stopLatched=false;afterResume?.(s,bridge);},
  async acquirePreparation(){events.push('acquire');},async releasePreparation(){events.push('release');}
 };
 const s=new StopRecovery({bridge,now:()=>clock,wait:async ms=>{clock+=ms;},replyTimeoutMs:10});
 return {s,bridge,wires,events};
}
test('explicit unlock restores fresh four-wheel feedback without movement or origin changes',async()=>{
 const f=fixture();assert.deepEqual(f.wires,[]);
 assert.equal(await f.s.start(),true);assert.equal(f.s.phase,'ready');
 assert.deepEqual(f.wires,['chassis task','chassis feedback','chassis feedback 0','chassis feedback on','chassis feedback','chassis task']);
 assert.equal(f.events.includes('acquire'),true);assert.equal(f.events.at(-1),'release');
 assert.equal(f.wires.some(w=>/^(?:imu |chassis (?:test|origin|run|turn) )/.test(w)),false);
});
test('bus isolation is repaired to completion before unlock and feedback writes',async()=>{
 const f=fixture({locked:1});assert.equal(await f.s.start(),true);
 assert.ok(f.events.indexOf('bus recover')<f.events.indexOf('resume'));
 assert.ok(f.events.indexOf('resume')<f.events.indexOf('chassis feedback on'));
});
test('accepting bus recovery is not enough to unlock before its final result',async()=>{
 let finish;
 const f=fixture({locked:1,reply(w,l,{s}){if(w!=='bus recover')return l;
  finish=()=>s.receive(l[1]);return [l[0]];}});
 const work=f.s.start();await new Promise(setImmediate);
 assert.equal(f.events.includes('resume'),false);assert.equal(f.wires.includes('chassis feedback on'),false);
 finish();assert.equal(await work,true);
});
test('recovery failure does not unlock or enable feedback',async()=>{
 const f=fixture({locked:1,reply:(w,l)=>w==='bus recover'?['WARN bus recover reason=reply_timeout addr=2 locked=1']:l});
 assert.equal(await f.s.start(),false);assert.equal(f.bridge.stopLatched,true);
 assert.equal(f.events.includes('resume'),false);assert.equal(f.wires.includes('chassis feedback on'),false);
 assert.match(f.s.message,/reply_timeout/);
});
test('a new stop during resume cancels feedback setup and is never overwritten',async()=>{
 const f=fixture({afterResume(s,b){b.stopToken=8;b.stopLatched=true;b.commandEpoch++;s.observeStop({latched:true,token:8});}});
 assert.equal(await f.s.start(),false);assert.equal(f.bridge.stopLatched,true);
 assert.equal(f.wires.includes('chassis feedback on'),false);
});
test('stale wheel cache is waited for and never reported ready',async()=>{
 const f=fixture({reply:(w,l)=>w==='chassis feedback'?wheels(100):l});
 assert.equal(await f.s.start(),false);assert.equal(f.s.phase,'failed');assert.match(f.s.message,/新鲜/);
});
test('temporary stale feedback can recover without resending a motion or calibration',async()=>{
 let reads=0;const f=fixture({reply:(w,l)=>w==='chassis feedback'&&++reads<3?wheels(100):l});
 assert.equal(await f.s.start(),true);assert.equal(f.wires.filter(w=>w==='chassis feedback on').length,1);
});
test('a damaged short query can be retried; a mutation is never replayed',async()=>{
 let reads=0;const f=fixture({reply:(w,l)=>w==='chassis task'&&++reads===1?['OK chassis task state=4 locked=0']:l});
 assert.equal(await f.s.start(),true);assert.equal(f.wires.filter(w=>w==='chassis task').length,3);
 assert.equal(f.wires.filter(w=>w==='chassis feedback on').length,1);
});
test('a missing feedback-write reply is reported without automatically repeating the write',async()=>{
 const f=fixture({reply:(w,l)=>w==='chassis feedback on'?[]:l});
 assert.equal(await f.s.start(),false);assert.equal(f.wires.filter(w=>w==='chassis feedback on').length,1);
 assert.equal(f.s.phase,'failed');assert.equal(f.events.at(-1),'release');
});
test('changing the connection during feedback polling cancels the remaining sequence',async()=>{
 const f=fixture({reply:(w,l,{s,bridge})=>{if(w==='chassis feedback on'){bridge.connected=false;s.cancel('连接已断开');}return l;}});
 assert.equal(await f.s.start(),false);assert.notEqual(f.s.phase,'ready');
 assert.equal(f.events.at(-1),'release');
});

const missionAccepted='OK bus recover requested mission_preserved=1 tasks_resumed=0';
const missionRecovered='OK grab route state=paused mode=mission stop_confirmed=1 reason=bus_recovered auto_resume=0';

test('mission recovery completion continues the same unlock and feedback sequence',async()=>{
 const f=fixture({locked:1,reply:(w,l)=>w==='bus recover'?[missionAccepted,missionRecovered]:l});
 assert.equal(await f.s.start(),true);
 assert.equal(f.wires.filter(w=>w==='bus recover').length,1);
 assert.ok(f.events.indexOf('bus recover')<f.events.indexOf('resume'));
 assert.match(f.s.message,/比赛保持暂停/);
 assert.equal(f.wires.some(w=>/^(?:grab mission continue|chassis (?:test|origin|turn|run) )/.test(w)),false);
});

test('mission acceptance, retries and unconfirmed stops do not unlock early',async()=>{
 let finish;
 const f=fixture({locked:1,reply(w,l,{s}){if(w!=='bus recover')return l;
  finish=()=>s.receive(missionRecovered);
  return [missionAccepted,
   'OK grab route state=bus_recovery_retry reason=reply_timeout auto_resume=0',
   missionRecovered.replace('stop_confirmed=1','stop_confirmed=0'),
   missionRecovered.replace('state=paused','state=resuming').replace('auto_resume=0','auto_resume=1')];}});
 const work=f.s.start();await new Promise(setImmediate);
 assert.equal(f.events.includes('resume'),false);
 finish();assert.equal(await work,true);
});

test('mission recovery failure reports its reason without unlocking or replaying recovery',async()=>{
 const f=fixture({locked:1,reply:(w,l)=>w==='bus recover'?[missionAccepted,
  'OK grab route state=bus_recovery_wait reason=motor_disabled stop_confirmed=0 auto_resume=0']:l});
 assert.equal(await f.s.start(),false);assert.match(f.s.message,/motor_disabled/);
 assert.equal(f.events.includes('resume'),false);
 assert.equal(f.wires.filter(w=>w==='bus recover').length,1);
});

test('mission completion from before the current request acceptance is ignored',async()=>{
 let finish;
 const f=fixture({locked:1,reply(w,l,{s}){if(w!=='bus recover')return l;
  finish=()=>s.receive(missionRecovered);return [missionRecovered,missionAccepted];}});
 const work=f.s.start();await new Promise(setImmediate);
 assert.equal(f.events.includes('resume'),false);
 finish();assert.equal(await work,true);
});

test('new stop cancels mission recovery and a late success cannot unlock it',async()=>{
 const f=fixture({locked:1,reply:(w,l)=>w==='bus recover'?[missionAccepted]:l});
 const work=f.s.start();await new Promise(setImmediate);
 f.bridge.stopToken=8;f.bridge.commandEpoch++;
 f.s.observeStop({latched:true,token:8});f.s.receive(missionRecovered);
 assert.equal(await work,false);assert.equal(f.events.includes('resume'),false);
 assert.equal(f.bridge.stopLatched,true);
});

test('mission success still requires current chassis isolation to be cleared',async()=>{
 let reads=0;
 const f=fixture({locked:1,reply(w,l){
  if(w==='bus recover')return [missionAccepted,missionRecovered];
  if(w==='chassis task'&&++reads===2)return [task(1)];return l;}});
 assert.equal(await f.s.start(),false);assert.equal(f.events.includes('resume'),false);
 assert.equal(f.wires.includes('chassis feedback on'),false);
});
