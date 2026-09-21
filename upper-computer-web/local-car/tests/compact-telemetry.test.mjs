import test from 'node:test';
import assert from 'node:assert/strict';
import {Telemetry,DEFAULT_GEOMETRY} from '../map-model.mjs';
const config={v:1,kind:'config',session:1,rev:'test',geometry:DEFAULT_GEOMETRY,wheel_ids:[1,2,3,4],forward_dir:[0,0,1,1],telemetry_period_ms:2000};
const state={v:2,kind:'state',session:1,t_ms:1000,trace_seq:1,
 task:['idle','none',0,20],target:[0,0,0,0,0,0,0],
 feedback:[[0,0,0,0],[1,-4294967295,3,4],15,15,[990,990,990,990],[990,990,990,990],1,20],
 yaw:[true,0],pose:[[true,10,20,0],[true,10,20,0]],localization:[1,990,true],
 route:['idle','none',0,0,[0,0,0],[0,0,0],true,true],tx:[1,0,0,1,true,true,4],dropped:0};
const line=r=>'@CHASSIS '+JSON.stringify(r);
function model(){const t=new Telemetry();t.begin(1,0);t.accept(line(config),0);return t;}
test('compact state preserves coordinates, raw positions, freshness and stop semantics',()=>{
 const t=model();t.accept(line(state),100);assert.equal(t.status,'遥测接收中');
 assert.equal(t.feedback.x_mm,10);assert.equal(t.state.feedback.pos[1],-4294967295);
 assert.equal(t.route.stop_confirmed,true);assert.equal(t.state.tx.error,0);
 const stale=structuredClone(state);stale.t_ms=3000;t.accept(line(stale),200);assert.equal(t.feedback,null);
});
test('malformed compact arrays clear state rather than inventing valid feedback',()=>{
 for(const change of [s=>s.feedback[2]=16,s=>s.feedback[4].pop(),s=>s.pose[1]=[true,10],s=>s.tx[2]='0',s=>s.route.pop()]){
  const t=model();t.accept(line(state),100);const bad=structuredClone(state);bad.t_ms=3000;change(bad);
  t.accept(line(bad),200);assert.equal(t.state,null);assert.equal(t.feedback,null);
 }
});
