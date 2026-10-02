import test from 'node:test';
import assert from 'node:assert/strict';
import {WheelFeedback} from '../wheel-feedback.mjs';
test('wheel snapshots preserve protocol units and do not renew unchanged cached samples',()=>{
  let now=0;const model=new WheelFeedback(()=>now);
  const line='OK wheel=2 raw_rpm=-20 raw_units=-65536 flags=0x03 valid=1,1,1 rx_ms=10,11,12';
  assert.equal(model.receive(line),true);
  assert.equal(model.rows()[1].position,'-65536');
  assert.equal(model.rows()[1].rpm,-20);
  now=6000;model.receive(line);
  assert.equal(model.rows()[1].age,6);
  model.clear();assert.equal(model.rows()[1],null);
  assert.equal(model.receive('OK wheel=7 raw_rpm=0'),false);
  assert.equal(model.receive('OK wheel=2 raw_rpm=bad raw_units=1 flags=0x03 valid=1,1,1 rx_ms=1,2,3'),false);
});
