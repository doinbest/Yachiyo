// Units: mm, seconds, radians. Wheel order: FL, RL, RR, FR.
export const DEFAULT_GEOMETRY = Object.freeze({wheelbase_mm:210,track_mm:240,wheel_diameter_mm:80,gear_ratio:1});
export const FIELD = Object.freeze({size:2400,blocks:[{x:550,y:550},{x:1400,y:550},{x:550,y:1400},{x:1400,y:1400}],blockSize:450});
export const validGeometry=g=>!!g && Object.keys(DEFAULT_GEOMETRY).every(k=>Number.isFinite(g[k])&&g[k]>0&&g[k]<100000);
export const SIM_MAX_RPM=100;
export function scaleWheels(rpm,maxRpm){if(!Number.isFinite(maxRpm)||maxRpm<1||maxRpm>32767)throw new RangeError('maxRpm must be 1..32767');const largest=Math.max(...rpm.map(Math.abs));const scale=largest>maxRpm?maxRpm/largest:1;return {rpm:rpm.map(v=>v*scale),scale};}
export function inverse([vx,vy,w],g=DEFAULT_GEOMETRY,maxRpm=Infinity){const k=(g.wheelbase_mm+g.track_mm)/2,s=60*g.gear_ratio/(Math.PI*g.wheel_diameter_mm);const rpm=[vx-vy-k*w,vx+vy-k*w,vx-vy+k*w,vx+vy+k*w].map(v=>v*s);return maxRpm===Infinity?rpm:scaleWheels(rpm,maxRpm).rpm;}
export function distancePulses(distanceMm,g=DEFAULT_GEOMETRY,pulsesPerRev=3200){return Math.round(Math.abs(distanceMm)*g.gear_ratio*pulsesPerRev/(Math.PI*g.wheel_diameter_mm));}
export function quantize(rpm){return rpm.map(v=>Math.sign(v)*Math.floor(Math.abs(v)+0.5));}
export function forward(rpm,g=DEFAULT_GEOMETRY){const [a,b,c,d]=rpm.map(v=>v*Math.PI*g.wheel_diameter_mm/(60*g.gear_ratio));return [(a+b+c+d)/4,(-a+b-c+d)/4,(-a-b+c+d)/(2*(g.wheelbase_mm+g.track_mm))];}
export function rotate([x,y],theta){const c=Math.cos(theta),s=Math.sin(theta);return [c*x-s*y,s*x+c*y];}
export function midpoint(p,[vx,vy,w],dt){const [dx,dy]=rotate([vx,vy],p.yaw_rad+w*dt/2);return {x_mm:p.x_mm+dx*dt,y_mm:p.y_mm+dy*dt,yaw_rad:p.yaw_rad+w*dt};}
export function wheelCenters(g=DEFAULT_GEOMETRY){const x=g.wheelbase_mm/2,y=g.track_mm/2;return [[x,y],[-x,y],[-x,-y],[x,-y]];}
export class FixedStepper {
  remainder=0;
  reset(){this.remainder=0;}
  advance(elapsedMs,step){
    if(!Number.isFinite(elapsedMs)||elapsedMs<0||elapsedMs>250){this.reset();return false;}
    this.remainder+=elapsedMs;
    while(this.remainder>=20){step(0.02);this.remainder-=20;}
    return true;
  }
}
const vector=(v,n)=>Array.isArray(v)&&v.length===n&&v.every(Number.isFinite);
const flags=v=>Array.isArray(v)&&v.length===4&&v.every(x=>typeof x==='boolean');
const u32=v=>Number.isInteger(v)&&v>=0&&v<=0xffffffff;
const validPose=p=>p?.valid===true&&['x_mm','y_mm','yaw_rad'].every(k=>Number.isFinite(p[k]));
const routeStates={idle:'待命',running:'当前段运行中',stopping:'正在停车',waiting:'等待手动下一段',done:'前四段结束',error:'执行错误',cancelled:'已取消'};
const distanceStates={idle:'待命',running:'运行中',stopping:'正在停车',done:'结束',error:'执行错误'};
const validMotion=r=>!!r&&typeof r.reason==='string'&&u32(r.action_id)&&vector(r.target,3)&&vector(r.error,3)&&typeof r.feedback_valid==='boolean'&&typeof r.stop_confirmed==='boolean';
const validRoute=r=>validMotion(r)&&typeof r.state==='string'&&Object.hasOwn(routeStates,r.state)&&Number.isInteger(r.segment)&&r.segment>=0&&r.segment<=4;
const validDistance=r=>validMotion(r)&&typeof r.active==='boolean'&&typeof r.state==='string'&&Object.hasOwn(distanceStates,r.state);
function motionDetails(motion){
  const [x,y,heading]=motion.target;
  const error=motion.feedback_valid?`误差 ΔX ${motion.error[0].toFixed(1)} · ΔY ${motion.error[1].toFixed(1)} mm · 航向 ${motion.error[2].toFixed(1)}°`:'反馈无效 · 误差不可用';
  return `目标 X ${x.toFixed(1)} · Y ${y.toFixed(1)} mm · θ ${heading.toFixed(1)}°\n${error}\n${motion.stop_confirmed?'已确认停止':'未确认停止'} · 原因 ${motion.reason} · 编码器反馈估计，非视觉精对准`;
}
export function routeStatusText(route){
  if(!validRoute(route))return '实车路线状态不可用 · 请在地图页手动开启遥测；查询回复见收发记录。';
  return `${routeStates[route.state]} · 第 ${route.segment} / 4 段 · 动作 ${route.action_id}\n${motionDetails(route)}`;
}
export function motionDisplay(route,distance){
  if(validRoute(route)&&route.segment>0)return {text:routeStatusText(route),target:route.target,label:`实车目标 ${route.segment}`};
  if(validRoute(route)&&route.segment===0&&validDistance(distance))return {text:`单段位移 · ${distanceStates[distance.state]} · ${distance.active?'任务活动':'任务非活动'} · 动作 ${distance.action_id}\n${motionDetails(distance)}`,target:distance.target,label:'单段位移目标'};
  return {text:routeStatusText(route),target:null,label:''};
}
export class Telemetry {
  constructor(){this.stop('遥测未开启');}
  get periodMs(){return this.config?.telemetry_period_ms??200;}
  get timeoutMs(){return this.config?this.periodMs*3:8000;}
  get route(){return validRoute(this.state?.route)?this.state.route:null;}
  get distance(){return validDistance(this.state?.distance)?this.state.distance:null;}
  stop(reason='遥测已关闭'){this.session=null;this.config=null;this.state=null;this.lastTime=null;this.traceSeq=null;this.receivedAt=null;this.startedAt=null;this.status=reason;this.command=null;this.feedback=null;this.paths={command:[],feedback:[]};}
  begin(session,now){this.stop('等待配置帧');this.session=session;this.startedAt=now;}
  breakPaths(){for(const key of ['command','feedback']){const path=this.paths[key];if(path.length&&path.at(-1)!==null)path.push(null);}this.command=null;this.feedback=null;}
  tick(now){if(this.session!==null && now-(this.receivedAt??this.startedAt)>this.timeoutMs)this.stop('遥测超时 · 请手动重新开启');}
  pushPath(key,pose){const path=this.paths[key];if(pose)path.push(pose);else if(path.length&&path.at(-1)!==null)path.push(null);if(path.length>3000)path.splice(0,path.length-3000);}
  accept(line,now){
    if(!line.startsWith('@CHASSIS'))return false;
    this.tick(now);
    let r;try{r=JSON.parse(line.slice(8).trim());}catch{this.state=null;this.breakPaths();this.status='遥测格式错误';return true;}
    if(!r||typeof r!=='object'||r.v!==1||this.session===null||r.session!==this.session)return true;
    if(r.kind==='config'){
      if((r.telemetry_period_ms!==undefined&&(!Number.isInteger(r.telemetry_period_ms)||r.telemetry_period_ms<200||r.telemetry_period_ms>10000))||!validGeometry(r.geometry)||JSON.stringify(r.wheel_ids)!=='[1,2,3,4]'||!Array.isArray(r.forward_dir)||r.forward_dir.length!==4||!r.forward_dir.every(d=>d===0||d===1)||typeof r.rev!=='string'||!r.rev){this.breakPaths();this.config=null;this.state=null;this.status='配置不兼容';return true;}
      this.config=r;this.receivedAt=now;this.state=null;this.lastTime=null;this.breakPaths();this.status='配置就绪 · 等待状态';return true;
    }
    if(r.kind!=='state'||!this.config)return true;
    if(!u32(r.t_ms)||!vector(r.target?.body,3)||!vector(r.target?.rpm,4)||!vector(r.feedback?.rpm,4)||!vector(r.feedback?.pos,4)||!flags(r.feedback?.speed_valid)||!flags(r.feedback?.position_valid)||!Array.isArray(r.feedback?.speed_ms)||r.feedback.speed_ms.length!==4||!r.feedback.speed_ms.every(u32)||!Array.isArray(r.feedback?.position_ms)||r.feedback.position_ms.length!==4||!r.feedback.position_ms.every(u32)){
      this.state=null;this.breakPaths();this.status='状态字段无效';return true;
    }
    const delta=this.lastTime===null?null:(r.t_ms-this.lastTime)>>>0;
    if(delta!==null&&(delta===0||delta>=0x80000000))return true;
    if((delta!==null&&delta>Math.max(300,this.periodMs*1.5))||(this.traceSeq!==null&&this.traceSeq!==r.trace_seq))this.breakPaths();
    this.traceSeq=r.trace_seq??null;
    this.lastTime=r.t_ms;this.receivedAt=now;this.state=r;
    this.command=validPose(r.pose?.command)?r.pose.command:null;
    const fresh=r.feedback.position_valid.every((v,i)=>v&&((r.t_ms-r.feedback.position_ms[i])>>>0)<=600);
    this.feedback=fresh&&r.yaw?.valid===true&&Number.isFinite(r.yaw.rad)&&validPose(r.pose?.feedback)?r.pose.feedback:null;
    this.pushPath('command',this.command);this.pushPath('feedback',this.feedback);
    this.status='遥测接收中';return true;
  }
}
