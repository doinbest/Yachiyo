import test from 'node:test';
import assert from 'node:assert/strict';
import {parsePageRoute,createNavigationMemory} from '../page-navigation.mjs';
test('deep and old routes retain exact context without any I/O',()=>{
  assert.deepEqual(parsePageRoute('#params/material/z_grab'),{page:'params',tab:'material',key:'z_grab'});
  assert.equal(parsePageRoute('#params').tab,'material');assert.equal(parsePageRoute('#map').tab,'monitor');
  const memory=createNavigationMemory();memory.save('#map/radar',321);memory.enterParameters({hash:'#map/radar',label:'雷达建图'});
  assert.equal(memory.source.hash,'#map/radar');assert.equal(memory.restore(memory.source.hash),321);memory.enterParameters(null);assert.equal(memory.source,null);
});
test('visual modes retain context and removed history links fall back to numbered rings',()=>{
  for(const tab of ['legacy','calibration','history','unknown'])
    assert.deepEqual(parsePageRoute('#vision/'+tab),{page:'vision',tab:'ring',key:undefined});
  assert.deepEqual(parsePageRoute('#vision/grab/status'),{page:'vision',tab:'grab',key:'status'});
  for(const tab of ['ring','grab','track','route','camera'])
    assert.deepEqual(parsePageRoute('#vision/'+tab),{page:'vision',tab,key:undefined});
  assert.equal(parsePageRoute('#vision').tab,'ring');
  const memory=createNavigationMemory();
  memory.save('#vision/ring',450);memory.enterParameters({hash:'#vision/ring',label:'编号环放置'});
  assert.equal(memory.restore(memory.source.hash),450);
});
test('radar, monitor and offline hashes remain distinct display-only tabs',()=>{
  assert.deepEqual(parsePageRoute('#map/radar'),{page:'map',tab:'radar',key:undefined});
  assert.deepEqual(parsePageRoute('#map/monitor'),{page:'map',tab:'monitor',key:undefined});
  assert.deepEqual(parsePageRoute('#map/offline'),{page:'map',tab:'offline',key:undefined});
});
