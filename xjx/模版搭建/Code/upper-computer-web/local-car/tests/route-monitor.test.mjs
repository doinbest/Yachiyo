import test from 'node:test';
import assert from 'node:assert/strict';
import {RouteStopHistory} from '../route-monitor.mjs';
const sample=(overrides={})=>({state:{session:1,t_ms:2000,trace_seq:1,localization:{generation:1}},feedback:{x_mm:2101,y_mm:1198,yaw_rad:1.57},route:{action_id:7,segment:2,phase:'move',phase_target:[2100,1200,90],target:[2100,1200,90],state:'waiting',reason:'reached',speed_mm_s:200,phase_speed_mm_s:50,stop_confirmed:true},...overrides});
test('records confirmed feedback stops with plan speed and target, never a missing pose',()=>{
  const history=new RouteStopHistory();
  history.observe(sample({feedback:null}));assert.equal(history.stops.length,0);
  history.observe(sample());assert.equal(history.stops.length,1);
  assert.deepEqual(history.stops[0].target,[2100,1200,90]);
  assert.equal(history.stops[0].speed,200);assert.equal(history.stops[0].limit,50);
  assert.equal(history.stops[0].pose.x_mm,2101);
});
test('deduplicates repeated confirmation; a new motion and stop records the corrected position',()=>{
  const history=new RouteStopHistory(),s=sample();history.observe(s);history.observe(s);
  assert.equal(history.stops.length,1);
  history.observe(sample({route:{...s.route,stop_confirmed:false,state:'running'}}));
  history.observe(sample({feedback:{...s.feedback,x_mm:2100}}));
  assert.equal(history.stops.length,2);assert.equal(history.stops[1].pose.x_mm,2100);
});
test('keeps history through missing telemetry/reconnect and separates localization generations',()=>{
  const history=new RouteStopHistory();history.observe(sample());history.observe({state:null});
  history.observe(sample({state:{session:2,t_ms:2000,trace_seq:1,localization:{generation:1}}}));
  assert.equal(history.stops.length,1);
  history.observe(sample({state:{session:2,t_ms:2000,trace_seq:2,localization:{generation:2}}}));
  assert.equal(history.stops.length,2);
});
