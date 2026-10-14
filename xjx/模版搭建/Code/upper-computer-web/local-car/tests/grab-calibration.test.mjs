import test from 'node:test';
import assert from 'node:assert/strict';
import * as protocol from '../protocol.mjs';
import {readOnlyCommand, scopedStopCommand} from '../bridge.mjs';

test('calibration moves to grab without retiring independent camera controls',()=>{
  assert.equal(protocol.buildCommand('grab-calib-ref'),'grab calib ref');
  assert.equal(protocol.buildCommand('grab-calib',{target:'3'}),'grab calib 3');
  assert.equal(protocol.buildCommand('grab-calib-status'),'grab calib status');
  assert.equal(protocol.buildCommand('camera-material',{target:'3'}),'camera material 3');
  assert.equal(protocol.buildCommand('camera-ring',{ring:'2'}),'camera ring 2');
  assert.equal(protocol.commands.some(c=>c.id.startsWith('material-')),false);
  for(const target of ['0','7','1.5',''])assert.throws(()=>protocol.buildCommand('grab-calib',{target}));
});

test('calibration queries pass stop locks; start and reference remain explicit operations',()=>{
  assert.equal(readOnlyCommand('grab calib status'),true);
  assert.equal(readOnlyCommand('grab calib ref'),false);
  assert.equal(readOnlyCommand('grab calib 3'),false);
  for(const command of ['grab stop','stop all','camera stop','grab track stop'])
    assert.equal(scopedStopCommand(command),true,command);
});

test('calibration diagnostics cannot replace an ordinary grab task snapshot',()=>{
  const wire='OK grab calib phase=collect ref=1 calibrated=1 reason=none fdx=2 fdy=0.5 xdx=0 xdy=3';
  assert.equal(protocol.parseGrabStatus(wire),null);
  const parsed=protocol.parseGrabCalibrationStatus(wire);
  for(const prefix of ['arm> ','arm> arm> '])
    assert.deepEqual(protocol.parseGrabCalibrationStatus(prefix+wire),parsed);
  assert.equal(parsed.phase,'collect');assert.equal(parsed.ref,1);
  assert.equal(parsed.fdx,2);assert.equal(parsed.xdy,3);
  assert.equal(protocol.parseGrabCalibrationStatus('OK grab calib accepted color=3'),null);
  assert.equal(protocol.parseGrabCalibrationStatus(wire.replace('ref=1','ref=2')),null);
  assert.equal(protocol.parseGrabCalibrationStatus(wire.replace('fdx=2','fdx=nan')).fdx,null);
});
