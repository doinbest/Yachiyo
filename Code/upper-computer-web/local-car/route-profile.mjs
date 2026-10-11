import {DEFAULT_GEOMETRY,validGeometry,inverse,forward,quantize,rotate,midpoint,routePhaseText,routeDirectionText} from './map-model.mjs';
import {footprintDiagnostic} from './route-model.mjs';
import {DEFAULT_BODY_SIZE,validBodySize} from './route-model.mjs';
export const PROFILE_MAX_RPM=3000;
const PROFILE_ACCEL_MM_S2=300;
const radians=degrees=>degrees*Math.PI/180;
const angle=value=>Math.atan2(Math.sin(value),Math.cos(value));
const finiteTarget=value=>Array.isArray(value)&&value.length===3&&value.every(Number.isFinite);

export function profilePhaseText(p){return `目标 ${p.segment} · ${routePhaseText(p.phase)} · ${routeDirectionText(p.direction)} · 工位 ${p.station} / 访问 ${p.visit} · (${p.target.join(', ')}) · 上限 ${p.speed_mm_s} mm/s${p.arm_pose==='rear'?' · 机械臂朝后':p.arm_pose==='work'?' · 机械臂顺时针90°作业':''}`;}
export function profileResultText(r){
  if(r.issue)return `目标 ${r.issue.segment} · ${routePhaseText(r.issue.phase)}：${r.bodySize.length_mm}×${r.bodySize.width_mm} mm 外框触及${r.issue.obstacle} · X ${r.issue.pose.x_mm.toFixed(1)} / Y ${r.issue.pose.y_mm.toFixed(1)} / θ ${(r.issue.pose.yaw_rad*180/Math.PI).toFixed(1)}°；已在上一有效姿态停止`;
  const warning=r.warnings?.[0];
  if(warning)return `${r.state==='done'?'方案播放结束':'方案播放中'} · 障碍格风险 ${r.warnings.length} 处：目标 ${warning.segment} · ${routePhaseText(warning.phase)} · ${warning.obstacle}；按整格范围估计，需核对实际净空`;
  return r.state==='done'?'C 阶段理想模型结束 · 外框检查通过；未使用实际反馈，须实车验收':'C 阶段理想模型运行中 · 按角度/距离完成阶段';
}

/** Playback consumes C decisions verbatim. It does not select headings or detours. */
export class RouteProfileRunner{
  constructor(profile,{geometry=DEFAULT_GEOMETRY,integer=false,mask=0,bodySize=DEFAULT_BODY_SIZE,gridPolicy='stop'}={}){
    if(!profile||!Array.isArray(profile.points)||!profile.points.length||!profile.points[0].slice(0,2).every(Number.isFinite)||!Array.isArray(profile.phases)||!profile.phases.length||!validGeometry(geometry))throw new Error('当前固件 C 阶段预览不可用，请先重新规划');
    for(const p of profile.phases)if(!['move','turn'].includes(p.kind)||!finiteTarget(p.target)||!Number.isFinite(p.speed_mm_s)||p.speed_mm_s<0||typeof p.phase!=='string'||typeof p.direction!=='string'||!Number.isInteger(p.segment))throw new Error('C 阶段记录无效');
    if(!Number.isFinite(profile.turn_speed_deg_s)||profile.turn_speed_deg_s<=0)throw new Error('请重新规划以获取当前固件转向速度');
    if(!validBodySize(bodySize))throw new Error('预演车体尺寸必须在 0～300 mm 内');
    this.profile=profile;this.rows=profile.phases;this.geometry={...geometry};this.integer=integer;this.mask=mask;this.bodySize={...bodySize};
    this.gridPolicy=gridPolicy;this.warnings=[];
    this.pose={x_mm:profile.points[0][0],y_mm:profile.points[0][1],yaw_rad:radians(Number.isFinite(profile.initial_yaw_deg)?profile.initial_yaw_deg:90)};this.path=[{...this.pose}];
    this.index=0;this.elapsed=0;this.distance=0;this.speed=0;this.body=[0,0,0];this.rpm=[0,0,0,0];this.stops=[];this.state='running';this.issue=null;
    this.check(this.pose);
  }
  get phase(){return this.rows[this.index]?.phase??'done';}
  get armPose(){return this.rows[this.index]?.arm_pose??'rear';}
  check(p){
    const diagnostic=footprintDiagnostic(p,{mask:this.mask,bodySize:this.bodySize});if(!diagnostic)return true;
    const issue={...diagnostic,segment:this.rows[this.index].segment,phase:this.phase,pose:{...p}};
    // A marked cell locates evidence somewhere in that cell; it is not a solid rectangle.
    // Formal checks retain the conservative stop. Playback records the risk and follows C.
    if(diagnostic.source==='grid'&&this.gridPolicy==='warn'){
      if(!this.warnings.some(w=>w.index===this.index&&w.obstacle===issue.obstacle))this.warnings.push({...issue,index:this.index});
      return true;
    }
    this.issue=issue;this.state='error';this.body=[0,0,0];return false;
  }
  finishPhase(){
    const p=this.rows[this.index];this.stops.push({index:this.index,name:profilePhaseText(p),pose:{...this.pose},time:this.elapsed});
    this.speed=0;if(this.index+1===this.rows.length)this.state='done';else this.index++;
  }
  swept(next){
    const previous=this.pose,dx=next.x_mm-previous.x_mm,dy=next.y_mm-previous.y_mm,da=next.yaw_rad-previous.yaw_rad;
    const steps=Math.max(1,Math.ceil(Math.hypot(dx,dy)/5),Math.ceil(Math.abs(da)/radians(1)));
    for(let i=1;i<=steps;i++){
      const p={x_mm:previous.x_mm+dx*i/steps,y_mm:previous.y_mm+dy*i/steps,yaw_rad:previous.yaw_rad+da*i/steps};
      if(!this.check(p))return false;
      this.distance+=Math.hypot(p.x_mm-this.pose.x_mm,p.y_mm-this.pose.y_mm);this.pose=p;this.path.push({...p});
    }
    if(this.path.length>20000)this.path=this.path.filter((_,i)=>i%2===0||i===this.path.length-1);
    return true;
  }
  advance(seconds){
    if(!Number.isFinite(seconds)||seconds<0)throw new Error('模拟时间无效');
    while(seconds>1e-10&&this.state==='running'){
      const p=this.rows[this.index],dt=Math.min(.02,seconds),turn=p.kind==='turn';
      const dx=p.target[0]-this.pose.x_mm,dy=p.target[1]-this.pose.y_mm,remaining=turn?angle(radians(p.target[2])-this.pose.yaw_rad):Math.hypot(dx,dy);
      if(Math.abs(remaining)<1e-8){this.finishPhase();continue;}
      const limit=turn?radians(this.profile.turn_speed_deg_s):p.speed_mm_s;
      if(limit<=0)throw new Error('移动阶段速度必须大于 0');
      this.speed=Math.min(limit,this.speed+(turn?.8:PROFILE_ACCEL_MM_S2)*dt,Math.sqrt(2*(turn?.8:PROFILE_ACCEL_MM_S2)*Math.abs(remaining)));
      const travel=Math.min(this.speed*dt,Math.abs(remaining));
      const planar=turn?[0,0]:rotate([dx/remaining*travel/dt,dy/remaining*travel/dt],-this.pose.yaw_rad);
      const requested=[...planar,turn?Math.sign(remaining)*travel/dt:0];
      // Current firmware default motor ceiling; historical editor retains its 100 RPM model.
      this.rpm=inverse(requested,this.geometry,PROFILE_MAX_RPM);let body=forward(this.integer?quantize(this.rpm):this.rpm,this.geometry);
      if(turn){body=[0,0,Math.sign(remaining)*Math.min(Math.abs(body[2]),Math.abs(remaining)/dt)];}
      else {const norm=Math.hypot(body[0],body[1]);if(norm>limit){body[0]*=limit/norm;body[1]*=limit/norm;}body[2]=0;}
      let next=midpoint(this.pose,body,dt);
      // Ideal distance/angle completion also settles the last quantized step; sweep checks still apply.
      if(travel>=Math.abs(remaining)-1e-9||this.integer&&Math.abs(remaining)<Math.min(turn?1e-3:1,limit*dt)){next=turn?{...this.pose,yaw_rad:this.pose.yaw_rad+remaining}:{...this.pose,x_mm:p.target[0],y_mm:p.target[1]};}
      this.body=body;if(!this.swept(next))break;
      this.elapsed+=dt;seconds-=dt;
    }
    if(this.state!=='running'){this.body=[0,0,0];this.rpm=[0,0,0,0];}
  }
}
