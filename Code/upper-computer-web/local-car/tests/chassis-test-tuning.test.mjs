import test from 'node:test';
import assert from 'node:assert/strict';
import {buildCommand} from '../protocol.mjs';
import {parseChassisTestResult} from '../chassis-test-result.mjs';
const base={direction:'forward',distance:1000,speed:1000};
test('independent linear deceleration is transmitted and read from device results',()=>{
 assert.equal(buildCommand('chassis-test',{...base,accel:500,decel:250}),'chassis test 1000 0 1000 linear 500 0.02 0 1 250');
 for(const decel of [0,9,5001,'nan',''])assert.throws(()=>buildCommand('chassis-test',{...base,accel:500,decel}));
 const r=parseChassisTestResult('OK chassis test_result id=1 state=3 profile=linear elapsed_ms=1000 peak_mm_s=100 error_mm=2 stop_ms=100 stop_confirmed=1 accel_mm_s2=500 decel_mm_s2=250');
 assert.equal(r.decel_mm_s2,250);
});
test('independent profile period is appended only when different from 1ms',()=>{
 assert.equal(buildCommand('chassis-test',{...base,curve_period_ms:5}),'chassis test 1000 0 1000 linear 500 0.02 0 5 300');
 for(const v of [0,101,1.5,''])assert.throws(()=>buildCommand('chassis-test',{...base,curve_period_ms:v}));
 const r=parseChassisTestResult('OK chassis test_result id=1 state=3 profile=linear elapsed_ms=1000 peak_mm_s=100 error_mm=2 stop_ms=100 stop_confirmed=1 curve_period_ms=5 control_period_ms=1');
 assert.equal(r.curve_period_ms,5);assert.equal(r.control_period_ms,1);
});
test('single-test tuning sends active profile parameter and heading gains',()=>{
 assert.equal(buildCommand('chassis-test',base),'chassis test 1000 0 1000 linear 500 0.02 0 1 300');
 assert.equal(buildCommand('chassis-test',{...base,heading_kp:0,heading_kd:0}),'chassis test 1000 0 1000 linear 500 0 0 1 300');
 assert.equal(buildCommand('chassis-test',{...base,heading_kp:0,heading_kd:.01}),'chassis test 1000 0 1000 linear 500 0 0.01 1 300');
 assert.equal(buildCommand('chassis-test',{...base,profile:'cosine',cosine_k:400,heading_kp:.1,heading_kd:.01,accel:''}),'chassis test 1000 0 1000 cosine 400 0.1 0.01');
 for(const key of ['heading_kp','heading_kd'])for(const v of [-1,1.01,'nan',''])assert.throws(()=>buildCommand('chassis-test',{...base,[key]:v}));
 for(const v of [0,9,2001,120.5,''])assert.throws(()=>buildCommand('chassis-test',{...base,profile:'cosine',cosine_k:v}));
 assert.equal(buildCommand('chassis-test',{...base,cosine_k:''}),'chassis test 1000 0 1000 linear 500 0.02 0 1 300');
});
test('actual test result includes cosine K and independent P/D scalars',()=>{
 const s=parseChassisTestResult('OK chassis test_result id=1 state=3 profile=cosine elapsed_ms=5000 peak_mm_s=800 error_mm=2 stop_ms=100 stop_confirmed=1 accel_mm_s2=0 cosine_k=400 heading_kp=0.1 heading_kd=0.01');
 assert.equal(s.cosine_k,400);assert.equal(s.heading_kp,.1);assert.equal(s.heading_kd,.01);
});
