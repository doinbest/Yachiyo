import test from 'node:test';
import assert from 'node:assert/strict';
import {comparison, sampleProfile, mapSegments, exposure} from '../speed-profile.mjs';
import {RouteRunner, defaultRoute, footprintIssue} from '../route-model.mjs';

const near=(a,b,t=1e-7)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
test('same-time comparison travels the same distance with identical peak speed and time',()=>{
  for(const length of [212.132034,900,1750]){
    const [linear,cosine]=comparison(length,1000,600,'time');
    near(linear.duration,cosine.duration);near(linear.peak,cosine.peak);
    near(cosine.maxAcceleration/linear.maxAcceleration,Math.PI/2);
    for(const p of [linear,cosine]){
      near(sampleProfile(p,0).s,0);near(sampleProfile(p,p.duration).s,length);
      near(sampleProfile(p,p.duration).v,0);
      let distance=0;const n=20000,dt=p.duration/n;
      for(let i=0;i<n;i++)distance+=sampleProfile(p,(i+.5)*dt).v*dt;
      near(distance,length,2e-5);
    }
  }
});
test('equal acceleration obeys both limits and smooth profile takes longer',()=>{
  for(const length of [212,900,1750,10000]){
    const pair=comparison(length,500,600,'acceleration');
    assert.ok(pair[1].duration>pair[0].duration);
    for(const p of pair){
      assert.ok(p.peak<=500);near(p.maxAcceleration,600);
      for(let i=0;i<=1000;i++){
        const s=sampleProfile(p,p.duration*i/1000);
        assert.ok(s.s>=-1e-8&&s.s<=length+1e-8);
        assert.ok(Math.abs(s.a)<=600+1e-8);
      }
    }
  }
});
test('smooth ramps join cruise with zero acceleration and scalar derivatives agree',()=>{
  const p=comparison(1750,300,600,'acceleration')[1];
  assert.ok(p.cruise>0);
  for(const t of [0,p.ramp,p.ramp+p.cruise,p.duration])near(sampleProfile(p,t).a,0);
  for(const t of [.12*p.duration,.8*p.duration]){
    const h=1e-5,x=sampleProfile(p,t),before=sampleProfile(p,t-h),after=sampleProfile(p,t+h);
    near((after.s-before.s)/(2*h),x.v,1e-5);near((after.v-before.v)/(2*h),x.a,1e-5);
  }
});
test('map segments reuse all original offline stops and valid car footprint',()=>{
  const segments=mapSegments();assert.equal(segments.length,16);
  const runner=new RouteRunner(defaultRoute(),{x_mm:2250,y_mm:150,yaw_rad:Math.PI/2});
  runner.advance(250);assert.equal(runner.state,'done');
  segments.forEach((s,i)=>{
    near(s.end[0],runner.stops[i].pose.x_mm,.002);near(s.end[1],runner.stops[i].pose.y_mm,.002);
    for(let j=0;j<=20;j++)assert.equal(footprintIssue({x_mm:s.start[0]+(s.end[0]-s.start[0])*j/20,y_mm:s.start[1]+(s.end[1]-s.start[1])*j/20,yaw_rad:Math.PI/2}),null);
  });
});
test('traction illustration can show smooth curve exceeds threshold at equal time',()=>{
  const [linear,cosine]=comparison(1750,1000,600,'time');
  near(exposure(linear,800),0);assert.ok(exposure(cosine,800)>0);
  near(exposure(cosine,cosine.maxAcceleration+1),0);
});
test('invalid inputs cannot generate nonfinite trajectories',()=>{
  for(const args of [[0,500,100,'time'],[900,NaN,100,'time'],[900,500,-1,'time'],[900,500,100,'unknown']])assert.throws(()=>comparison(...args));
});
