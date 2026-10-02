import test from 'node:test';
import assert from 'node:assert/strict';
import {parsePageRoute,createNavigationMemory} from '../page-navigation.mjs';
test('deep and old routes retain exact context without any I/O',()=>{
  assert.deepEqual(parsePageRoute('#params/material/z_grab'),{page:'params',tab:'material',key:'z_grab'});
  assert.equal(parsePageRoute('#params').tab,'material');assert.equal(parsePageRoute('#map').tab,'monitor');
  const memory=createNavigationMemory();memory.save('#map/radar',321);memory.enterParameters({hash:'#map/radar',label:'雷达建图'});
  assert.equal(memory.source.hash,'#map/radar');assert.equal(memory.restore(memory.source.hash),321);memory.enterParameters(null);assert.equal(memory.source,null);
});
test('legacy visual links select historical diagnostics while ordinary links retain task context',()=>{
  assert.deepEqual(parsePageRoute('#vision/legacy'),{page:'vision',tab:'history',key:'legacy'});
  assert.deepEqual(parsePageRoute('#vision/calibration'),{page:'vision',tab:'history',key:'calibration'});
  assert.deepEqual(parsePageRoute('#vision/history'),{page:'vision',tab:'history',key:undefined});
  assert.deepEqual(parsePageRoute('#vision/grab/status'),{page:'vision',tab:'grab',key:'status'});
  assert.deepEqual(parsePageRoute('#vision/camera'),{page:'vision',tab:'camera',key:undefined});
  assert.equal(parsePageRoute('#vision').tab,'grab');
  const memory=createNavigationMemory();
  memory.save('#vision/legacy',450);memory.enterParameters({hash:'#vision/legacy',label:'历史视觉诊断'});
  assert.equal(memory.restore(memory.source.hash),450);
});
