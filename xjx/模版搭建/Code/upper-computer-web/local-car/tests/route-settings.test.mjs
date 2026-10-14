import test from 'node:test';
import assert from 'node:assert/strict';
const settings=await import('../route-settings.mjs').catch(()=>({}));
test('one-time route speed migration retains low speeds and preserves high old value for review',()=>{
  assert.equal(typeof settings.migrateRoutePreferences,'function');
  assert.equal(settings.migrateRoutePreferences({}).routeSpeed,1000);
  for(const speed of [10,100,200])assert.equal(settings.migrateRoutePreferences({routeSpeed:speed}).routeSpeed,speed);
  const old={routeSpeed:5000,offlineRows:[{vx:100}],theme:'blue'};
  const migrated=settings.migrateRoutePreferences(old);
  assert.equal(migrated.routeSpeed,1000);assert.equal(migrated.previousRouteSpeed,5000);
  assert.deepEqual(migrated.offlineRows,old.offlineRows);assert.equal(migrated.theme,'blue');
  assert.equal(settings.migrateRoutePreferences({...migrated,routeSpeed:5000}).routeSpeed,5000);
  assert.equal(old.routeSpeed,5000);
});
test('all route inputs share saved startup speed without sending commands',()=>{
  assert.equal(typeof settings.bindRouteSpeed,'function');
  const inputs=Array.from({length:4},()=>({value:'',dispatchEvent(){},addEventListener(_,fn){this.change=fn;}}));
  let saves=0,prefs=settings.migrateRoutePreferences({routeSpeed:100});
  const binding=settings.bindRouteSpeed({prefs,inputs,save(){saves++;}});
  assert.deepEqual(inputs.map(i=>i.value),['100','100','100','100']);assert.equal(saves,0);
  inputs[2].value='5000';inputs[2].change();assert.equal(prefs.routeSpeed,5000);assert.equal(saves,1);
  assert.deepEqual(inputs.map(i=>i.value),['5000','5000','5000','5000']);
  assert.throws(()=>binding.set(5001));assert.equal(prefs.routeSpeed,5000);
});
