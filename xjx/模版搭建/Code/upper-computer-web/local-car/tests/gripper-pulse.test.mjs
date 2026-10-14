import test from 'node:test';
import assert from 'node:assert/strict';
import {commands,buildCommand,moduleForWire} from '../protocol.mjs';
import {readOnlyCommand,scopedStopCommand} from '../bridge.mjs';

test('gripper pulse input uses existing duty protocol and preserves calibrated presets',()=>{
  assert.equal(buildCommand('grip-open'),'grip open');
  assert.equal(buildCommand('grip-catch'),'grip catch');
  const definition=commands.find(c=>c.id==='grip-pulse');
  assert.ok(definition);assert.equal(definition.fields[0].value,2120);
  for(const [pulse,duty] of [[500,2.5],[1920,9.6],[2120,10.6],[2300,11.5],[2500,12.5],[2037,10.185]])
    assert.equal(buildCommand('grip-pulse',{pulse}),`grip duty ${duty}`);
  for(const pulse of [0,499,2501,1920.5,'NaN','2000 extra'])assert.throws(()=>buildCommand('grip-pulse',{pulse}));
  assert.equal(buildCommand('grip-pulse',{pulse:'02300.0'}),'grip duty 11.5');
  assert.equal(moduleForWire(buildCommand('grip-pulse',{pulse:2100})),'arm');
  assert.equal(readOnlyCommand('grip duty 10.5'),false);
  assert.equal(scopedStopCommand('grip duty 10.5'),false);
});
