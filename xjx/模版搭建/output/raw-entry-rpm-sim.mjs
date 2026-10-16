import fs from 'node:fs';
import {inverse} from '../Code/upper-computer-web/local-car/map-model.mjs';
import {footprintDiagnostic,bodyCorners} from '../Code/upper-computer-web/local-car/route-model.mjs';
const C=Math.PI*80/60, R=225, dt=.01;
const P=[[2100,1950],[2100,2200],[1900,2142],[1700,2130]], dest=[1200,2100];
function curve(t){const u=1-t;return [0,1].map(k=>u*u*u*P[0][k]+3*u*u*t*P[1][k]+3*u*t*t*P[2][k]+t*t*t*P[3][k]);}
const arc=[];
for(let i=0;i<=1000;i++){const t=i/1000,[x,y]=curve(t),yaw=90*(1-t*t*(3-2*t));arc.push({x,y,yaw,s:i?arc[i-1].s+Math.hypot(x-arc[i-1].x,y-arc[i-1].y):0});}
const len=arc.at(-1).s;
for(let i=0;i<arc.length;i++){
 const a=arc[Math.max(0,i-1)],b=arc[Math.min(1000,i+1)],q=arc[i];
 q.tx=(b.x-a.x)/Math.hypot(b.x-a.x,b.y-a.y);q.ty=(b.y-a.y)/Math.hypot(b.x-a.x,b.y-a.y);
 q.grad=(b.yaw-a.yaw)/(b.s-a.s);
}
function run(rpm,turn){
 const vmax=rpm*C,omega=turn*C/R*180/Math.PI;
 const speeds=arc.map((q,i)=>{
   const a=arc[Math.max(0,i-1)],b=arc[Math.min(1000,i+1)];
   const curvature=Math.hypot(b.tx-a.tx,b.ty-a.ty)/(b.s-a.s);
   return Math.min(50*C,vmax,.65*omega/Math.max(1e-9,Math.abs(q.grad)),Math.sqrt(250/Math.max(curvature,1e-9)));
 });
 // Tangential <=150 and normal <=250 give total acceleration <300 mm/s².
 for(let i=1;i<speeds.length;i++)speeds[i]=Math.min(speeds[i],Math.sqrt(speeds[i-1]**2+300*(arc[i].s-arc[i-1].s)));
 for(let i=998;i>=0;i--)speeds[i]=Math.min(speeds[i],Math.sqrt(speeds[i+1]**2+300*(arc[i+1].s-arc[i].s)));
 const frames=[]; let clock=0,peakAccel=0;
 const emit=(x,y,yaw,vx,vy,deg,phase)=>{
   const a=yaw*Math.PI/180,w=deg*Math.PI/180;
   const wheels=inverse([Math.cos(a)*vx+Math.sin(a)*vy,-Math.sin(a)*vx+Math.cos(a)*vy,w]);
   frames.push({t:clock,x,y,yaw,v:Math.hypot(vx,vy),omega:deg,wheels,phase});
 };
 function line(start,end,yaw,v0,v1,phase){
   const L=Math.hypot(end[0]-start[0],end[1]-start[1]),ux=(end[0]-start[0])/L,uy=(end[1]-start[1])/L,T=.4;
   const peak=Math.min(vmax,L/T-(v0+v1)/2),d0=(v0+peak)*T/2,d1=(v1+peak)*T/2,hold=Math.max(0,(L-d0-d1)/peak),total=2*T+hold;
   peakAccel=Math.max(peakAccel,Math.PI*Math.max(peak-v0,peak-v1)/(2*T));
   const base=clock;
   for(let tm=0;tm<total;tm+=dt){let s,v;
     if(tm<T){v=v0+(peak-v0)*.5*(1-Math.cos(Math.PI*tm/T));s=v0*tm+(peak-v0)*.5*(tm-T/Math.PI*Math.sin(Math.PI*tm/T));}
     else if(tm<T+hold){v=peak;s=d0+peak*(tm-T);}
     else {const z=tm-T-hold;v=peak+(v1-peak)*.5*(1-Math.cos(Math.PI*z/T));s=d0+peak*hold+peak*z+(v1-peak)*.5*(z-T/Math.PI*Math.sin(Math.PI*z/T));}
     clock=base+tm;emit(start[0]+ux*s,start[1]+uy*s,yaw,v*ux,v*uy,0,phase);
   }
   clock=base+total;
 }
 line([2100,1200],P[0],90,0,speeds[0],'右通道直行');
 for(let i=0;i<arc.length;i++){
   const q=arc[i],v=speeds[i];
   if(i)clock+=2*(q.s-arc[i-1].s)/(v+speeds[i-1]);
   emit(q.x,q.y,q.yaw,q.tx*v,q.ty*v,q.grad*v,'绕角平移＋顺时针转向');
 }
 line(P[3],dest,0,speeds.at(-1),0,'斜直线进物料台');
 emit(...dest,0,0,0,0,'到达物料台中间');
 let minEdge=Infinity,minBlock=Infinity;
 for(const f of frames){
   const pose={x_mm:f.x,y_mm:f.y,yaw_rad:f.yaw*Math.PI/180};
   if(footprintDiagnostic(pose))throw Error(JSON.stringify({f,issue:footprintDiagnostic(pose)}));
   const corners=bodyCorners(pose);
   minEdge=Math.min(minEdge,...corners.flatMap(([x,y])=>[x,y,2400-x,2400-y]));
   // Conservative gap to top-right forbidden square, via axis separation.
   minBlock=Math.min(minBlock,Math.max(Math.min(...corners.map(p=>p[0]))-1850,Math.min(...corners.map(p=>p[1]))-1850));
 }
 return {rpm,turn,total:clock,peakAccel,peakRpm:Math.max(...frames.flatMap(f=>f.wheels.map(Math.abs))),minEdge,minBlock,frames:frames.filter((_,i)=>i%3===0||i===frames.length-1)};
}
const results=[120,250,300].flatMap(rpm=>[40,60].map(turn=>run(rpm,turn)));
const data={results,path:arc.filter((_,i)=>i%10===0).map(q=>[q.x,q.y,q.yaw]),model:{diameter:80,wheelbase:210,track:240,body:300,curveCapRpm:50,K:200}};
fs.writeFileSync('output/raw-entry-rpm-data.json',JSON.stringify(data,(_,v)=>typeof v==='number'?Math.round(v*1000)/1000:v));
console.log(JSON.stringify(results.map(({frames,...r})=>r),null,2));
