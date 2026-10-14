import test from 'node:test';
import assert from 'node:assert/strict';
import {commands,buildCommand,moduleForWire,formatTerminalLine} from '../protocol.mjs';
import {readOnlyCommand,scopedStopCommand} from '../bridge.mjs';
test('lidar servo exposes confirmed scan/stow positions and retains manual input',()=>{
  assert.equal(buildCommand('lidar-servo-out'),'lidarservo out');
  assert.equal(buildCommand('lidar-servo-in'),'lidarservo in');
  for(const id of ['lidar-servo-off',... [500,1000,1500,2000,2500].map(v=>`lidar-servo-${v}`)])
    assert.throws(()=>buildCommand(id));
  assert.match(commands.find(c=>c.id==='lidar-servo-out').label,/2450/);
  assert.match(commands.find(c=>c.id==='lidar-servo-in').label,/1450/);
  assert.equal(commands.find(c=>c.id==='lidar-servo-pulse').fields[0].value,1450);
  for(const pulse of [500,1450,2450])
    assert.equal(buildCommand('lidar-servo-pulse',{pulse}),`lidarservo pulse ${pulse}`);
  for(const pulse of [2501,3000,4000,5000])
    assert.equal(buildCommand('lidar-servo-pulse',{pulse}),`lidarservo pulse ${pulse}`);
  for(const pulse of [0,499,5001,1500.5,'NaN','1000 extra'])
    assert.throws(()=>buildCommand('lidar-servo-pulse',{pulse}));
  for(const pulse of ['1500.0','1500.','0500',' 1500 '])
    assert.equal(buildCommand('lidar-servo-pulse',{pulse}),`lidarservo pulse ${Number(pulse)}`);
  assert.equal(buildCommand('lidar-servo-status'),'lidarservo status');
  assert.equal(commands.find(c=>c.id==='lidar-servo-pulse').owner,'arm');
  assert.equal(moduleForWire('OK lidarservo enabled=1 pulse_us=1537'),'arm');
  assert.equal(readOnlyCommand('lidarservo status'),true);
  assert.equal(scopedStopCommand('lidarservo off'),false);
  for(const wire of ['lidarservo out','lidarservo in','lidarservo off extra','lidarservo status extra','lidarservo pulse 500']) {
    assert.equal(readOnlyCommand(wire),false);
    assert.equal(scopedStopCommand(wire),false);
  }
  assert.match(formatTerminalLine('OK lidarservo enabled=0 pulse_us=0'),/尚未设置/);
});
