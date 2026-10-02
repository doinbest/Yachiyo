import test from 'node:test';
import assert from 'node:assert/strict';
import {parsePageRoute,createNavigationMemory} from '../page-navigation.mjs';
test('deep and old routes retain exact context without any I/O',()=>{
  assert.deepEqual(parsePageRoute('#params/material/z_grab'),{page:'params',tab:'material',key:'z_grab'});
  assert.equal(parsePageRoute('#params').tab,'material');assert.equal(parsePageRoute('#map').tab,'monitor');
  const memory=createNavigationMemory();memory.save('#map/radar',321);memory.enterParameters({hash:'#map/radar',label:'雷达建图'});
  assert.equal(memory.source.hash,'#map/radar');assert.equal(memory.restore(memory.source.hash),321);memory.enterParameters(null);assert.equal(memory.source,null);
});
