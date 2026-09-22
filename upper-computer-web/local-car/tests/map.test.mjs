import test from 'node:test';
import assert from 'node:assert/strict';
import {DEFAULT_GEOMETRY as g,inverse,forward,quantize,rotate,midpoint,wheelCenters,FixedStepper,Telemetry,FIELD} from '../map-model.mjs';
import {LineDecoder,frameCommand,commands} from '../protocol.mjs';
import * as model from '../map-model.mjs';
const near=(a,b,t=1e-9)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);

test('standalone distance diagnostics share subscribed freshness, with active route taking priority',()=>{
  const m=new Telemetry();
  const route={state:'idle',reason:'none',segment:0,action_id:0,target:[0,0,0],error:[0,0,0],feedback_valid:false,stop_confirmed:false};
  const distance={active:true,state:'running',reason:'none',action_id:9,target:[2250,250,90],error:[0,100,0.1],feedback_valid:true,stop_confirmed:false};
  const packet=t=>({...state(t),route,distance});
  send(m,packet(1),1);assert.equal(m.distance,null);
  m.begin(42,1);send(m,packet(2),2);assert.equal(m.distance,null);
  send(m,config,2);send(m,packet(3),3);assert.deepEqual(m.distance,distance);
  let display=model.motionDisplay(m.route,m.distance);
  assert.deepEqual(display.target,[2250,250,90]);assert.match(display.text,/单段位移/);assert.match(display.text,/未确认停止/);
  const activeRoute={...route,state:'running',segment:2,target:[2100,1200,90]};
  display=model.motionDisplay(activeRoute,m.distance);assert.deepEqual(display.target,[2100,1200,90]);assert.match(display.text,/第 2 \/ 4 段/);
  for(const patch of [{active:1},{state:'waiting'},{state:['running']},{action_id:-1},{target:[1,2]},{error:[0,null,0]},{feedback_valid:1},{stop_confirmed:1}]){
    send(m,{...packet(m.lastTime+1),distance:{...distance,...patch}},20);assert.equal(m.distance,null);
    assert.equal(model.motionDisplay(m.route,m.distance).target,null);
  }
  send(m,{...packet(m.lastTime+1),distance:{...distance,active:false,state:'done',feedback_valid:false,stop_confirmed:true}},30);
  display=model.motionDisplay(m.route,m.distance);assert.match(display.text,/误差不可用/);assert.match(display.text,/已确认停止/);
  assert.equal(model.motionDisplay(null,m.distance).target,null);
  send(m,{...state(m.lastTime+1),route},40);assert.equal(m.distance,null);
  send(m,packet(m.lastTime+1),50);m.tick(651);assert.equal(m.distance,null);
  m.begin(43,700);send(m,packet(800),701);assert.equal(m.distance,null);
  m.stop();assert.equal(m.distance,null);
});

test('optional real route telemetry requires current subscribed state and clears with its lifecycle',()=>{
  const m=new Telemetry();
  const route={state:'waiting',reason:'reached',segment:1,action_id:12,target:[2100,300,90],error:[1,-2,0.2],feedback_valid:true,stop_confirmed:true};
  const packet=(t,extra={})=>({...state(t),route:{...route,...extra}});
  send(m,packet(10),10);assert.equal(m.route,null);
  m.begin(42,10);send(m,packet(20),20);assert.equal(m.route,null);
  send(m,config,20);send(m,packet(30),30);assert.deepEqual(m.route,route);
  assert.match(model.routeStatusText(m.route),/第 1 \/ 4 段/);
  assert.match(model.routeStatusText(m.route),/已确认停止/);
  assert.match(model.routeStatusText(m.route),/误差 ΔX 1.0/);
  for(const patch of [{segment:5},{segment:-1},{action_id:-1},{action_id:4294967296},{state:'arrived'},{state:['waiting']},{feedback_valid:1},{stop_confirmed:'yes'},{target:[0,0]},{error:[0,null,0]},{reason:null}]){
    send(m,packet(m.lastTime+10,patch),100);assert.equal(m.route,null);assert.ok(m.state);
  }
  send(m,packet(m.lastTime+10,{feedback_valid:false,stop_confirmed:false}),200);
  assert.match(model.routeStatusText(m.route),/误差不可用/);
  assert.match(model.routeStatusText(m.route),/未确认停止/);
  m.accept('OK chassis route state=done segment=4 reason=reached',210);
  assert.equal(m.route.state,'waiting');
  send(m,state(m.lastTime+10),220);assert.equal(m.route,null);assert.ok(m.state);
  send(m,packet(m.lastTime+10),230);assert.ok(m.route);
  m.tick(831);assert.equal(m.route,null);
  m.begin(43,900);send(m,packet(1000),900);assert.equal(m.route,null);
  m.stop();assert.equal(m.route,null);
});
test('physical wheel matrix, geometry, round-trip and coordinate rotation',()=>{
  assert.deepEqual(wheelCenters(),[[105,120],[-105,120],[-105,-120],[105,-120]]);
  assert.deepEqual(FIELD.blocks,[{x:550,y:550},{x:1400,y:550},{x:550,y:1400},{x:1400,y:1400}]);
  inverse([100,0,0]).forEach(x=>near(x,23.8732414637843));
  assert.deepEqual(inverse([0,100,0]).map(Math.sign),[-1,1,-1,1]);
  assert.deepEqual(inverse([0,0,.1]).map(Math.sign),[-1,-1,1,1]);
  for(const v of [[0,0,0],[100,0,0],[0,-180,.5],[-90,40,-.3]])forward(inverse(v)).forEach((x,i)=>near(x,v[i]));
  const v=rotate([100,0],Math.PI/2);near(v[0],0);near(v[1],100);
  assert.deepEqual(quantize([-1.5,1.5,-.49,.49]),[-2,2,-0,0]);
  near(forward(quantize(inverse([8,0,0])))[0],8.377580409572781);
});
test('20 ms midpoint converges to analytic circular arc and handles elapsed catchup',()=>{
  let p={x_mm:0,y_mm:0,yaw_rad:0};const w=.5,t=4;
  for(let i=0;i<t/.02;i++)p=midpoint(p,[100,20,w],.02);
  near(p.x_mm,(100*Math.sin(w*t)+20*(Math.cos(w*t)-1))/w,.003);
  near(p.y_mm,(100*(1-Math.cos(w*t))+20*Math.sin(w*t))/w,.003);
  near(p.yaw_rad,w*t);
  const f=new FixedStepper();let count=0;const step=()=>count++;
  assert.equal(f.advance(13,step),true);f.advance(48,step);assert.equal(count,3);near(f.remainder,1);
  assert.equal(f.advance(1000,step),false);assert.equal(count,3);assert.equal(f.remainder,0);
});
const config={v:1,kind:'config',session:42,rev:'geom-20260912-v1',geometry:g,wheel_ids:[1,2,3,4],forward_dir:[0,0,1,1],position_units_per_rev:0,feedback_sign:'motor_raw'};
const state=t=>({v:1,kind:'state',session:42,t_ms:t,target:{body:[100,0,0],rpm:[24,24,24,24]},feedback:{rpm:[23,23,23,23],pos:[0,0,0,0],speed_valid:[true,true,true,true],position_valid:[true,true,true,true],speed_ms:[t,t,t,t],position_ms:[t,t,t,t]},yaw:{valid:true,rad:0},pose:{command:{valid:true,x_mm:t,y_mm:0,yaw_rad:0},feedback:{valid:true,x_mm:t*.9,y_mm:0,yaw_rad:0}}});
const send=(model,r,now)=>model.accept('@CHASSIS '+JSON.stringify(r),now);
test('online uses reported verified polarity rather than forcing offline defaults',()=>{
  const m=new Telemetry();m.begin(42,0);
  send(m,{...config,forward_dir:[1,0,1,1]},0);
  assert.deepEqual(m.config.forward_dir,[1,0,1,1]);
  send(m,{...config,forward_dir:[2,0,1,1]},0);assert.equal(m.config,null);
});
test('paced wireless telemetry waits for config, follows reported period and keeps sample validity checks',()=>{
  const m=new Telemetry();m.begin(42,0);m.tick(2000);assert.equal(m.session,42);
  send(m,{...config,telemetry_period_ms:2000},2100);
  send(m,state(2000),3400);m.tick(5000);assert.ok(m.state);
  send(m,state(4000),5400);assert.ok(m.feedback);
  assert.equal(m.paths.feedback.includes(null),false);
  const stale=state(6000);stale.feedback.position_ms[0]=5000;
  send(m,stale,7400);assert.equal(m.feedback,null);
  m.tick(13401);assert.equal(m.session,42);
  m.begin(42,14000);send(m,{...config,telemetry_period_ms:'2000'},14001);assert.equal(m.config,null);
});
test('session binding, config-first, timestamps, invalid feedback and timeout preserves session and resumes',()=>{
  const m=new Telemetry();m.begin(42,0);send(m,state(0),0);assert.equal(m.state,null);
  send(m,{...config,session:99},0);assert.equal(m.config,null);
  send(m,config,0);send(m,state(10),10);assert.equal(m.command.x_mm,10);assert.equal(m.feedback.x_mm,9);
  send(m,state(10),100);send(m,state(9),110);assert.equal(m.receivedAt,10);
  const bad=state(210);bad.feedback.position_valid[2]=false;send(m,bad,210);assert.equal(m.feedback,null);assert.ok(m.command);assert.equal(m.paths.feedback.at(-1),null);
  send(m,state(810),500);assert.equal(m.paths.command.at(-2),null);
  m.tick(1101);assert.equal(m.session,42);assert.equal(m.command,null);assert.equal(m.state,null);
  send(m,config,1102);send(m,state(1000),1102);assert.equal(m.command.x_mm,1000);
  m.begin(43,1200);send(m,state(1200),1200);assert.equal(m.config,null);
});
test('stale wheel samples, malformed JSON and clock wrap cannot create live substituted feedback',()=>{
  const m=new Telemetry();m.begin(42,0);send(m,config,0);
  const r=state(1000);r.feedback.position_ms[0]=1;send(m,r,10);assert.equal(m.feedback,null);
  m.accept('@CHASSIS null',20);m.accept('@CHASSIS {',20);assert.equal(m.command,null);
  send(m,config,30);send(m,state(0xfffffff0),40);send(m,state(184),240);assert.equal(m.lastTime,184);
  const malformed=state(384);malformed.feedback.speed_ms=[0];send(m,malformed,300);assert.equal(m.state,null);
});
test('position-derived pose is independent of RPM validity; trace reanchor starts a new segment',()=>{
  const m=new Telemetry();m.begin(42,0);send(m,config,0);
  const a=state(10);a.trace_seq=1;a.feedback.speed_valid=[false,false,false,false];send(m,a,10);assert.ok(m.feedback);
  const b=state(210);b.trace_seq=2;send(m,b,210);assert.equal(m.paths.feedback.at(-2),null);assert.equal(m.paths.command.at(-2),null);
  const c=state(610);c.trace_seq=2;send(m,c,610);assert.equal(m.paths.command.at(-2),null);
});
test('fragmented 2 KB telemetry reception retains CRLF; outgoing remains 63 byte limit; B3 removed',()=>{
  const line='@CHASSIS '+JSON.stringify({...state(200),extra:'x'.repeat(1400)});assert.ok(line.length>1800);
  const d=new LineDecoder();assert.deepEqual(d.push(line.slice(0,1000)),[]);assert.deepEqual(d.push(line.slice(1000)+'\r\n'),[line]);
  assert.throws(()=>frameCommand('x'.repeat(64)));assert.equal(frameCommand('chassis stream on 4294967295'),'chassis stream on 4294967295\r\n');
  assert.equal(commands.some(c=>c.id==='vision-ring'||c.id==='vision-calib'),false);
});

test('corrupt frames preserve only a stale display pose and recover without joining paths',()=>{
 const m=new Telemetry();m.begin(42,0);send(m,{...config,telemetry_period_ms:2000},0);send(m,state(100),100);
 m.accept('@CHASSIS {broken',200);assert.equal(m.state,null);assert.equal(m.feedback,null);
 assert.equal(m.lastFeedback.x_mm,90);assert.equal(m.stale,true);assert.equal(m.badFrames,1);
 m.tick(7000);assert.equal(m.session,42);assert.equal(m.state,null);
 send(m,{...state(8000),session:99},8000);assert.equal(m.state,null);
 send(m,state(8100),8100);assert.equal(m.stale,false);assert.equal(m.feedback.x_mm,7290);
 assert.equal(m.paths.feedback.at(-2),null);
 m.stop();assert.equal(m.lastFeedback,null);assert.equal(m.session,null);
});
