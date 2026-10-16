import test from 'node:test';
import assert from 'node:assert/strict';
import {parameterFields,displayValue,fieldWrites,deviceRequests} from '../parameter-schema.mjs';
import {GRAB_FORM_DEFAULTS} from '../grab-params.mjs';

test('automatic gripper defaults use 100ms without changing stability windows',()=>{
  for(const key of ['close_ms','release_ms','car_pick_wait_ms'])assert.equal(GRAB_FORM_DEFAULTS[key],100);
  assert.equal(GRAB_FORM_DEFAULTS.stable_ms,200);
  assert.equal(GRAB_FORM_DEFAULTS.arrival_hold_ms,200);
});
import {grabParameters,buildCommand} from '../protocol.mjs';

test('firmware defaults and physical ranges reach all parameter entry points',()=>{
  assert.equal(GRAB_FORM_DEFAULTS.z_min,-150);
  assert.equal(GRAB_FORM_DEFAULTS.car_pick_drop_mm,25);
  assert.equal(GRAB_FORM_DEFAULTS.z_grab,-45);
  assert.equal(GRAB_FORM_DEFAULTS.z_place,-25);
  assert.equal(GRAB_FORM_DEFAULTS.track_store_drop_mm,25);
  for(const key of ['z_speed','z_pick_speed','qr_z_speed'])assert.equal(GRAB_FORM_DEFAULTS[key],250);
  assert.equal(GRAB_FORM_DEFAULTS.z_proc_place,-110);
  assert.equal(GRAB_FORM_DEFAULTS.z_rough_align,-100);
  assert.equal(GRAB_FORM_DEFAULTS.z_temp_align,-100);
  assert.equal(GRAB_FORM_DEFAULTS.z_stack_place,-100);
  assert.equal(grabParameters.length,68);
  for(const key of ['head_proc_zero','head_proc_gain','head_temp_zero','head_temp_gain','head_line_zero','head_line_gain','head_tol','head_spread','head_settle_ms','head_proc_turn','head_temp_turn'])assert.ok(!(key in GRAB_FORM_DEFAULTS));
  assert.ok(!grabParameters.some(p=>['ref_u','ref_v'].includes(p.key)));
  assert.ok(!('ref_u' in GRAB_FORM_DEFAULTS) && !('ref_v' in GRAB_FORM_DEFAULTS));
  assert.equal(new Set(grabParameters.map(p=>p.key)).size,68);
  assert.equal(GRAB_FORM_DEFAULTS.track_pick_tol_px,100);
  assert.equal(GRAB_FORM_DEFAULTS.arrival_hold_ms,200);
  assert.equal(GRAB_FORM_DEFAULTS.track_pick_hold_ms,0);
  assert.equal(buildCommand('grab-set-track_pick_tol_px',{value:8}),'grab set track_pick_tol_px 8');
  assert.equal(buildCommand('grab-set-track_pick_tol_px',{value:100}),'grab set track_pick_tol_px 100');
  assert.equal(buildCommand('grab-set-track_pick_hold_ms',{value:0}),'grab set track_pick_hold_ms 0');
  for(const [key,values]of [['track_pick_tol_px',[0,101]],['track_pick_hold_ms',[-1,10001,1.5]]]){
    for(const value of values){
      assert.throws(()=>buildCommand(`grab-set-${key}`,{value}));
      assert.throws(()=>fieldWrites(parameterFields.find(p=>p.key===key),{[key]:value}));
    }
  }
  assert.equal(GRAB_FORM_DEFAULTS.base_home_degrade,1);
  assert.equal(parameterFields.find(p=>p.key==='base_home_degrade').tab,'motion');
  assert.equal(buildCommand('grab-set-base_home_degrade',{value:0}),'grab set base_home_degrade 0');
  assert.throws(()=>buildCommand('grab-set-base_home_degrade',{value:2}));
  assert.throws(()=>buildCommand('grab-set-base_home_degrade',{value:0.5}));
  for(const key of ['z_speed','z_pick_speed','qr_z_speed']) {
    const field=parameterFields.find(p=>p.key===key);
    assert.equal(field.max,333.333);
    assert.deepEqual(fieldWrites(field,{[key]:222.222}),['z_speed','z_pick_speed','qr_z_speed'].map(k=>[k,222.222]));
    assert.equal(buildCommand(`grab-set-${key}`,{value:111.111111}),`grab set ${key} 111.111111`);
    for(const value of [0,333.334]) {
      assert.throws(()=>fieldWrites(field,{[key]:value}));
      assert.throws(()=>buildCommand(`grab-set-${key}`,{value}));
    }
  }
  assert.throws(()=>fieldWrites(parameterFields.find(p=>p.key==='frames'),{frames:3.5}));
  assert.throws(()=>buildCommand('grab-set-base_car_offset',{value:361}));
  assert.match(parameterFields.find(p=>p.key==='x_acc').label,/驱动加速度档位/);
  assert.match(parameterFields.find(p=>p.key==='x_acc').note,/默认X20、Z255/);
});

test('visual mapping uses the measured 450 by 330 mm field at 640 by 480 pixels',()=>{
  const p=GRAB_FORM_DEFAULTS;
  const near=(actual,expected)=>assert.ok(Math.abs(actual-expected)<1e-6,`${actual} != ${expected}`);
  // Test physical distances as well as the response: catch inverse units or cm/mm mistakes.
  near(p.j00*450,640);
  near(p.j11*330,480);
  near(p.j12*330,480);
  near(100/p.j00,70.3125);
  near(100/p.j11,68.75);
  for(const key of ['j01','j02','j10'])assert.equal(p[key],0);
  // Explicit measured responses remain writable, including negative/coupled directions.
  for(const [key,value] of [['j00',-1.7],['j02',0.3],['j12',1.6]]){
    assert.deepEqual(fieldWrites(parameterFields.find(f=>f.key===key),{[key]:value}),[[key,value]]);
    assert.equal(buildCommand(`grab-set-${key}`,{value}),`grab set ${key} ${value}`);
  }
});

test('material Z speed retains its compatibility parameter entry',()=>{
  const speed=parameterFields.find(f=>f.key==='z_pick_speed');
  assert.ok(speed);assert.equal(speed.tab,'material');assert.equal(speed.unit,'mm/s');
  assert.deepEqual(fieldWrites(speed,{z_pick_speed:'111.111'}),['z_speed','z_pick_speed','qr_z_speed'].map(k=>[k,111.111]));
});

test('distance fields preserve absolute firmware coordinates and relative clearance',()=>{
  const values={z_grab:-80,z_lift:-40,z_place:-60,z_car_lift:0};
  const field=key=>parameterFields.find(f=>f.key===key);
  assert.equal(displayValue(field('z_grab'),values),80);
  assert.equal(displayValue(field('z_lift'),values),40);
  assert.equal(displayValue(field('z_car_lift'),values),60);
  assert.deepEqual(fieldWrites(field('z_grab'),{z_grab:'70',z_lift:'40'}),[['z_grab',-70],['z_lift',-30]]);
  assert.deepEqual(fieldWrites(field('z_car_lift'),{z_place:'60',z_car_lift:'30'}),[['z_car_lift',-30]]);
  for(const key of ['z_proc_grab','z_proc_lift','z_temp_place'])assert.equal(field(key),undefined);
  assert.deepEqual(fieldWrites(field('z_proc_place'),{z_proc_place:'130'}),[['z_proc_place',-130]]);
});

test('all device requests cover five tabs once and keep browser settings out of reads',()=>{
  const requests=deviceRequests();
  assert.equal(requests.filter(r=>r.kind==='radar').length,1);
  assert.equal(new Set(requests.map(r=>`${r.kind}:${r.key}`)).size,requests.length);
  assert.equal(requests.some(r=>r.key==='route_speed'),false);
  assert.ok(requests.some(r=>r.key==='z_stack_place'));
  assert.ok(requests.some(r=>r.key==='slot2_deg'));
  assert.equal(new Set(parameterFields.map(f=>f.tab)).size,5);
});

test('claw waiting is an optional motion timing field and units are separate from labels',()=>{
  for(const key of ['close_ms','release_ms','car_pick_wait_ms']){
    const f=parameterFields.find(f=>f.key===key);
    assert.equal(f.tab,'motion');assert.equal(f.group,'夹爪动作时序');assert.equal(f.unit,'ms');
    assert.equal(GRAB_FORM_DEFAULTS[key],100);
    assert.deepEqual(fieldWrites(f,{[key]:450}),[[key,450]]);
  }
  const wait=parameterFields.find(f=>f.key==='close_ms');
  assert.equal(wait.tab,'motion');assert.equal(wait.group,'夹爪动作时序');
  assert.equal(wait.label,'闭爪后等待');assert.equal(wait.unit,'ms');
  assert.match(wait.note,/下一步 Z 提起/);
  for(const field of parameterFields)assert.doesNotMatch(field.label,/ \/ (?:mm|ms|RPM|px|°|0\.1°)/);
  assert.equal(parameterFields.find(f=>f.key==='x_ppm').unit,'脉冲/mm');
  assert.equal(parameterFields.find(f=>f.kind==='arm'&&f.key==='base_rpm').unit,'RPM');
});
