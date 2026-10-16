import test from 'node:test';
import assert from 'node:assert/strict';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {readFileSync} from 'node:fs';
import {RouteProfileRunner} from '../route-profile.mjs';
import {footprintDiagnostic} from '../route-model.mjs';

const cwd=fileURLToPath(new URL('..',import.meta.url));
const result=spawnSync('python',['-X','utf8','-c','import json;from serve import plan_radar_offline;print(json.dumps(plan_radar_offline({"mask":0,"speed_mm_s":1000})))'],{cwd,encoding:'utf8'});
assert.equal(result.status,0,result.stderr);
const profiles=JSON.parse(result.stdout).route_profiles;

test('linear smooth-start preview is explicit and keeps ordinary/cosine profiles',()=>{
  assert.ok(profiles.task_fixed_smooth);
  assert.ok(profiles.task_planned_smooth);
  const opening=profiles.task_fixed_smooth.phases.filter(p=>p.segment===1);
  assert.equal(opening.length,1);
  assert.equal(opening[0].kind,'curve');
  assert.equal(opening[0].speed_mm_s,800);
  assert.deepEqual(opening[0].path[8].slice(0,3),[2125,350,180]);
  assert.deepEqual(opening[0].path.at(-2).slice(0,3),[2100,950,90]);
  assert.ok(opening[0].path.slice(1,-1).every(p=>p[4]===200));
  assert.deepEqual(profiles.task_fixed.phases.filter(p=>p.segment===1).map(p=>p.kind),['move','turn','move']);
  assert.equal(profiles.task_fixed_cosine.phases.find(p=>p.kind==='curve').phase,'entry_curve');
  assert.equal(profiles.task_fixed_smooth.phases.find(p=>p.segment===2&&p.kind==='move').speed_mm_s,800);
});

test('smooth opening sweeps the300mm body and stops only at QR in ideal preview',()=>{
  assert.ok(profiles.task_fixed_smooth);
  const p=profiles.task_fixed_smooth;
  const r=new RouteProfileRunner({...p,phases:p.phases.filter(p=>p.segment===1)});
  r.advance(200);
  assert.equal(r.state,'done',JSON.stringify(r.issue));
  assert.equal(r.stops.length,1);
  assert.ok(r.path.every(q=>footprintDiagnostic(q)===null));
  assert.ok(Math.abs(r.pose.yaw_rad*180/Math.PI-90)<.001);
});

test('offline selector exposes explicit linear smooth profiles without replacing defaults',()=>{
  const html=readFileSync(new URL('../index.html',import.meta.url),'utf8');
  assert.match(html,/value="task_fixed_smooth"/);
  assert.match(html,/value="task_planned_smooth"/);
  assert.match(html,/value="task_fixed"/);
});
