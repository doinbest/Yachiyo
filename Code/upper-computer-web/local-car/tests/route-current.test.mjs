import test from 'node:test';
import assert from 'node:assert/strict';
import {commands,buildCommand} from '../protocol.mjs';
import {Telemetry,DEFAULT_GEOMETRY,routeStatusText} from '../map-model.mjs';

test('real route commands default to 200 and map turn requires explicit map yaw',()=>{
  for(const id of ['chassis-route-start','chassis-route-auto','grab-route']){
    assert.equal(commands.find(c=>c.id===id).fields.find(f=>f.key==='speed').value,200);
    assert.match(buildCommand(id,{speed:5000}),/5000$/);
  }
  assert.equal(buildCommand('chassis-turn',{heading:-180}),'chassis turn -180');
  assert.equal(buildCommand('chassis-turn',{heading:90}),'chassis turn 90');
  for(const heading of ['',181,NaN])assert.throws(()=>buildCommand('chassis-turn',{heading}));
  assert.doesNotMatch(commands.find(c=>c.id==='grab-route').note,/第2段|第4段|前四段/);
});

const config={v:1,kind:'config',session:42,rev:'test',geometry:DEFAULT_GEOMETRY,wheel_ids:[1,2,3,4],forward_dir:[0,0,1,1]};
const route={state:'running',reason:'none',segment:192,action_id:7,target:[350,1200,180],error:[1,2,3],feedback_valid:true,stop_confirmed:false};
const meta={action_id:7,total:192,station:4,visit:2,phase:'dock',direction:'backward',speed_mm_s:200,phase_speed_mm_s:100,phase_target:[350,1200,180],planned:true};
const frame=t=>({v:1,kind:'state',session:42,t_ms:t,target:{body:[0,-100,0],rpm:[1,-1,1,-1]},feedback:{rpm:[-1,-1,1,1],pos:[0,0,0,0],speed_valid:[true,true,true,true],position_valid:[true,true,true,true],speed_ms:[t,t,t,t],position_ms:[t,t,t,t]},yaw:{valid:true,rad:1},pose:{command:{valid:true,x_mm:350,y_mm:1200,yaw_rad:Math.PI},feedback:{valid:true,x_mm:349,y_mm:1198,yaw_rad:Math.PI-.1}},route});
const send=(m,r)=>m.accept('@CHASSIS '+JSON.stringify(r),r.t_ms??0);

test('metadata applies to its own frame action, supports 192 targets and never leaks to the next frame/session',()=>{
  const m=new Telemetry();m.begin(42,0);send(m,config);send(m,{...frame(1),route_meta:meta});
  assert.equal(m.route?.total,192);assert.equal(m.route?.phase,'dock');
  assert.match(routeStatusText(m.route),/192 \/ 192/);assert.match(routeStatusText(m.route),/200.*100/);
  assert.deepEqual(m.state.feedback.rpm,[-1,-1,1,1]);
  send(m,frame(2));assert.ok(m.route);assert.equal(m.route.total,undefined);assert.doesNotMatch(routeStatusText(m.route),/\/ 16/);
  send(m,{...frame(3),route_meta:{...meta,action_id:8}});assert.equal(m.route.phase,undefined);
  m.begin(43,4);send(m,{...frame(5),route_meta:meta});assert.equal(m.route,null);
});

test('compact metadata expands without altering existing route wire array',()=>{
  const m=new Telemetry();m.begin(42,0);send(m,config);
  const a={v:2,kind:'state',session:42,t_ms:1,task:['running','none',7,20],target:[0,-100,0,1,-1,1,-1],feedback:[[-1,-1,1,1],[0,0,0,0],15,15,[1,1,1,1],[1,1,1,1],0,0],yaw:[true,1],pose:[[true,350,1200,Math.PI],[true,349,1198,Math.PI-.1]],localization:[1,1,true],route:['running','none',192,7,[350,1200,180],[1,2,3],true,false],tx:[0,0,0,2,true,true,15],route_meta:meta};
  send(m,a);assert.equal(m.route?.phase,'dock');assert.equal(m.route?.total,192);
});
