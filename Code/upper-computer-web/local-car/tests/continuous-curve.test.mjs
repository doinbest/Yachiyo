import test from 'node:test';
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {RouteProfileRunner} from '../route-profile.mjs';
import {routeSegments} from '../route-annotations.mjs';
import {footprintDiagnostic} from '../route-model.mjs';
const cwd=fileURLToPath(new URL('..',import.meta.url));
const result=spawnSync('python',['-X','utf8','-c','import json;from serve import plan_radar_offline;print(json.dumps(plan_radar_offline({"mask":0,"speed_mm_s":1000})))'],{cwd,encoding:'utf8'});
assert.equal(result.status,0,result.stderr);
const profile=JSON.parse(result.stdout).route_profiles.task_fixed;

test('opening joins corridor centre, spreads yaw and exits parallel to QR lane',()=>{
  const p=profile.phases.find(p=>p.phase==='entry_curve')??profile.phases.find(p=>p.kind==='curve');
  assert.equal(p.path.length,26);
  assert.deepEqual(p.path[8].slice(0,3),[2125,350,180]);
  assert.deepEqual(p.path.at(-2).slice(0,3),[2100,950,90]);
  assert.ok(p.path.slice(0,9).every(q=>q[2]===180));
  assert.ok(p.path.slice(9,-1).every(q=>q[0]>=2100&&q[0]<=2125));
});
test('C continuous paths sweep whole 300 mm body without an intermediate stop',()=>{
  const curves=profile.phases.filter(p=>p.kind==='curve');
  assert.equal(curves.length,2);
  for(const phase of curves){
    const p={...profile,points:[phase.path[0].slice(0,2)],initial_yaw_deg:phase.path[0][2],phases:[phase]};
    const r=new RouteProfileRunner(p);r.advance(200);
    assert.equal(r.state,'done',JSON.stringify(r.issue));
    assert.equal(r.stops.length,1);
    assert.ok(r.path.some(q=>Math.abs(q.yaw_rad*180/Math.PI-phase.path[0][2])>10 && Math.abs(q.yaw_rad*180/Math.PI-phase.target[2])>10));
    assert.ok(r.path.every(q=>footprintDiagnostic(q)===null));
    assert.equal(routeSegments(p)[0].points.length,phase.path.length);
    assert.ok(Math.abs(r.pose.yaw_rad*180/Math.PI-phase.target[2])<.001);
    assert.ok(Math.hypot(r.pose.x_mm-phase.target[0],r.pose.y_mm-phase.target[1])<.001);
  }
});
test('first raw station entry translates left before turning at the centre',()=>{
  const phases=profile.phases.filter(p=>p.segment===3);
  assert.deepEqual(phases.map(p=>p.phase),['work_approach','work_turn']);
  assert.deepEqual(phases.map(p=>p.target),[[1200,2100,90],[1200,2100,0]]);
  assert.deepEqual(phases.map(p=>p.direction),['lateral','turn']);
  const p={...profile,points:[[2100,2100]],initial_yaw_deg:90,phases};
  const r=new RouteProfileRunner(p);r.advance(200);
  assert.equal(r.state,'done',JSON.stringify(r.issue));
  assert.equal(r.stops.length,2);
  assert.ok(r.path.every(q=>footprintDiagnostic(q)===null));
  assert.ok(r.path.every(q=>Math.abs(q.yaw_rad-Math.PI/2)<.001 || Math.hypot(q.x_mm-1200,q.y_mm-2100)<.001));
});
test('curve preview carries current progress through playback pause and honors turn limit',()=>{
  const phase=profile.phases.find(p=>p.kind==='curve');
  const p={...profile,points:[phase.path[0].slice(0,2)],initial_yaw_deg:180,turn_speed_deg_s:20,phases:[phase]};
  const r=new RouteProfileRunner(p);let turned=false;
  for(let i=0;i<5000&&r.state==='running';i++){
    const before=r.curveProgress??0;r.advance(.02);
    assert.ok(Math.abs(r.body[2])*180/Math.PI<=20.001);
    if(r.state==='running')assert.ok((r.curveProgress??0)>=before);
    if(Math.abs(r.body[2])>.01)turned=true;
  }
  assert.equal(r.state,'done');assert.ok(turned);assert.equal(r.stops.length,1);
});
