// Invoked with actual C executable JSON on stdin by tests/chassis_map_contract_test.py.
import assert from 'node:assert/strict';
import {DEFAULT_GEOMETRY as g,inverse,forward,quantize,scaleWheels,rotate,distancePulses,Telemetry} from '../map-model.mjs';
let input='';for await(const chunk of process.stdin)input+=chunk;
const fixtures=JSON.parse(input);
const near=(actual,expected,tolerance=5e-4)=>assert.ok(Math.abs(actual-expected)<=tolerance,`${actual} != ${expected}`);
const vectorNear=(actual,expected)=>actual.forEach((v,i)=>near(v,expected[i]));
for(const f of fixtures.model){
  if(f.kind==='scenario'){
    const result=scaleWheels(inverse(f.body,g),f.max_rpm);
    vectorNear(result.rpm,f.rpm);vectorNear(inverse(f.body,g,f.max_rpm),f.rpm);near(result.scale,f.scale,1e-6);
    assert.deepEqual(quantize(result.rpm),f.command);vectorNear(forward(result.rpm,g),f.forward);vectorNear(forward(f.command,g),f.quantized_forward);
    if(f.name==='combined_scaled'){assert.ok(result.scale<1);assert.ok(Math.max(...result.rpm.map(Math.abs))<=100);forward(result.rpm,g).forEach((v,i)=>near(v,f.body[i]*result.scale));}
  }else if(f.kind==='rotation90'){vectorNear(rotate([100,0],Math.PI/2),f.map);vectorNear(rotate([100,0],-Math.PI/2),f.body);}
  else if(f.kind==='pulses'){assert.equal(f.forward100,1273);assert.equal(f.rotation90,4500);assert.equal(distancePulses(100,g,f.pulses_per_rev),f.forward100);assert.equal(distancePulses((g.wheelbase_mm+g.track_mm)/2*Math.PI/2,g,f.pulses_per_rev),f.rotation90);}
}
const config=fixtures.telemetry.find(line=>JSON.parse(line.slice(8)).kind==='config');
const state=fixtures.telemetry.find(line=>JSON.parse(line.slice(8)).kind==='state');
assert.ok(config&&state);
const m=new Telemetry();m.begin(12345,0);m.accept(state,0);assert.equal(m.state,null);
m.accept(config,10);assert.ok(m.config);assert.equal(m.config.session,12345);assert.equal(m.config.rev,'geom-20260912-v1');assert.deepEqual(m.config.geometry,g);
m.accept(state,20);assert.ok(m.state);assert.equal(m.state.dropped,7);assert.equal(m.state.feedback.pos[1],-4294967295);assert.equal(m.feedback,null);assert.equal(m.config.feedback_sign,'motor_raw');
const receiveTime=m.receivedAt;m.accept(state,30);assert.equal(m.receivedAt,receiveTime);
m.tick(6021);assert.equal(m.session,12345);assert.equal(m.state,null);m.accept(state,6022);assert.equal(m.state,null);
console.log('C/browser model contract: PASS (7 velocity cases, scaling, quantization, matrix, pulse baselines)');
console.log('Production C telemetry/browser contract: PASS (config-first, nonce, int64, invalid feedback, duplicate, timeout)');

const distanceFrame=fixtures.telemetry.find(line=>JSON.parse(line.slice(8)).distance);
assert.ok(distanceFrame);
const d=new Telemetry();d.begin(12345,0);d.accept(config,0);d.accept(distanceFrame,20);
assert.ok(d.distance);assert.deepEqual(d.distance.target,[10000,10000,0]);
assert.equal(d.distance.action_id,4294967295);assert.equal(d.distance.stop_confirmed,false);
assert.equal(d.route.state,'waiting');assert.equal(d.route.segment,2);
console.log('Production C route/distance optional telemetry contract: PASS');
