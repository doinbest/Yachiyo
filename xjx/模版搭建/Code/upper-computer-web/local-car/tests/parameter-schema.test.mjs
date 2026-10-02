import test from 'node:test';
import assert from 'node:assert/strict';
import {parameterFields,displayValue,fieldWrites,deviceRequests} from '../parameter-schema.mjs';

test('distance fields preserve absolute firmware coordinates and relative clearance',()=>{
  const values={z_grab:-80,z_lift:-40,z_place:-60,z_car_lift:0};
  const field=key=>parameterFields.find(f=>f.key===key);
  assert.equal(displayValue(field('z_grab'),values),80);
  assert.equal(displayValue(field('z_lift'),values),40);
  assert.equal(displayValue(field('z_car_lift'),values),60);
  assert.deepEqual(fieldWrites(field('z_grab'),{z_grab:'70',z_lift:'40'}),[['z_grab',-70],['z_lift',-30]]);
  assert.deepEqual(fieldWrites(field('z_car_lift'),{z_place:'60',z_car_lift:'30'}),[['z_car_lift',-30]]);
  assert.equal(displayValue(field('z_proc_lift'),{z_proc_grab:null,z_proc_lift:null}),null);
  assert.throws(()=>fieldWrites(field('z_proc_grab'),{z_proc_grab:'80',z_proc_lift:''}),/抬升/);
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
  const wait=parameterFields.find(f=>f.key==='close_ms');
  assert.equal(wait.tab,'motion');assert.equal(wait.group,'夹爪动作时序');
  assert.equal(wait.label,'夹爪到位等待');assert.equal(wait.unit,'ms');
  assert.match(wait.note,/下一步 Z 动作/);
  for(const field of parameterFields)assert.doesNotMatch(field.label,/ \/ (?:mm|ms|RPM|px|°|0\.1°)/);
  assert.equal(parameterFields.find(f=>f.key==='x_ppm').unit,'脉冲/mm');
  assert.equal(parameterFields.find(f=>f.kind==='arm'&&f.key==='base_rpm').unit,'RPM');
});
