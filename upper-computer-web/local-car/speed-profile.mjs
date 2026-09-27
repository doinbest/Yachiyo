// Teaching model only. Units: mm, s. No serial, motor limits, or tyre dynamics.
import {defaultRoute} from './route-model.mjs';
import {rotate} from './map-model.mjs';

function profile(kind,length,peak,ramp,cruise){
  return {kind,length,peak,ramp,cruise,duration:2*ramp+cruise,
    maxAcceleration:peak/ramp*(kind==='cosine'?Math.PI/2:1)};
}
export function comparison(length,maxSpeed,maxAcceleration,mode){
  if(![length,maxSpeed,maxAcceleration].every(x=>Number.isFinite(x)&&x>0)||!['time','acceleration'].includes(mode))throw new RangeError('距离、速度、加速度须大于零，比较方式须有效');
  const peak=Math.min(maxSpeed,Math.sqrt(length*maxAcceleration));
  const ramp=peak/maxAcceleration,cruise=Math.max(0,length/peak-ramp);
  const linear=profile('linear',length,peak,ramp,cruise);
  if(mode==='time')return [linear,profile('cosine',length,peak,ramp,cruise)];
  const smoothPeak=Math.min(maxSpeed,Math.sqrt(2*length*maxAcceleration/Math.PI));
  const smoothRamp=Math.PI*smoothPeak/(2*maxAcceleration);
  return [linear,profile('cosine',length,smoothPeak,smoothRamp,Math.max(0,length/smoothPeak-smoothRamp))];
}
export function sampleProfile(p,t){
  if(!Number.isFinite(t))throw new RangeError('时间须为有限数值');
  if(t<0)return {s:0,v:0,a:0};
  if(t>=p.duration)return {s:p.length,v:0,a:0};
  const accelerating=u=>p.kind==='linear'
    ?{s:p.peak*u*u/(2*p.ramp),v:p.peak*u/p.ramp,a:p.peak/p.ramp}
    :{s:p.peak/2*(u-p.ramp/Math.PI*Math.sin(Math.PI*u/p.ramp)),v:p.peak/2*(1-Math.cos(Math.PI*u/p.ramp)),a:p.peak*Math.PI/(2*p.ramp)*Math.sin(Math.PI*u/p.ramp)};
  if(t<p.ramp)return accelerating(t);
  if(t<p.ramp+p.cruise)return {s:p.peak*(p.ramp/2+t-p.ramp),v:p.peak,a:0};
  const reversed=accelerating(p.duration-t);
  return {s:p.length-reversed.s,v:reversed.v,a:-reversed.a};
}
// Time above an illustrative longitudinal traction-acceleration threshold.
// This does NOT predict slip distance or measure the real floor's friction.
export function exposure(p,threshold){
  if(!Number.isFinite(threshold)||threshold<=0)throw new RangeError('示意上限须大于零');
  if(threshold>=p.maxAcceleration)return 0;
  return p.kind==='linear'?2*p.ramp:2*p.ramp*(1-2*Math.asin(threshold/p.maxAcceleration)/Math.PI);
}
export function mapSegments(){
  let start=[2250,150];
  return defaultRoute().map((r,index)=>{
    const seconds=r.hold+(r.accel+r.decel)/2;
    const delta=rotate([r.vx*seconds,r.vy*seconds],Math.PI/2);
    const end=start.map((v,i)=>v+delta[i]);
    const segment={index,name:r.name,start:[...start],end,length:Math.hypot(...delta)};
    start=end;return segment;
  });
}
