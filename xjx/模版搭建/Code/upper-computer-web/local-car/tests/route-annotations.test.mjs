import test from 'node:test';
import assert from 'node:assert/strict';
import {routeSegments,routeLabels} from '../route-annotations.mjs';
test('C segments retain firmware numbers through intermediate phases and repeated visits',()=>{
  const profile={points:[[100,100]],phases:[
    {segment:1,target:[200,100,0]}, {segment:1,target:[300,100,0]},
    {segment:2,target:[100,100,180]}, {segment:3,target:[300,100,0]}]};
  const segments=routeSegments(profile);
  assert.deepEqual(segments.map(s=>s.number),[1,2,3]);
  assert.deepEqual(segments[0].points,[[100,100],[200,100],[300,100]]);
  const labels=routeLabels(segments);
  assert.deepEqual(labels.find(l=>l.x===300).numbers,[1,3]);
  assert.deepEqual(segments[1].points,[[300,100],[100,100]]);
});
test('geometry-only route is labelled as waypoints rather than invented business targets',()=>{
  const segments=routeSegments({points:[[0,0,1,0],[100,0,0,0],[100,100,5,1]]});
  assert.deepEqual(segments.map(s=>s.number),[1,2]);
  assert.ok(segments.every(s=>s.waypoint));
});
