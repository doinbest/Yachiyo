import {DEFAULT_GEOMETRY,SIM_MAX_RPM,FIELD,rotate,validGeometry,inverse,quantize,forward,midpoint} from './map-model.mjs';

export const defaultSegment = (name='新停车点') => ({name,vx:100,vy:0,omega:0,accel:.5,hold:2,decel:.5,dwell:1});
// Initial heading is +Y: positive body vy moves left in map coordinates.
export const defaultRoute = () => [
  {...defaultSegment('左上45°让出扫码距离'),vx:50,vy:50,hold:2.5},
  {...defaultSegment('扫码停车'),hold:8.5},
  {...defaultSegment('提前向左转入上方车道'),hold:8.5},
  {...defaultSegment('第一批取料入车载仓（任务示意）'),vx:0,vy:100,hold:8.5},
  {...defaultSegment('第一批粗加工：按码放齐3件再按序装仓（示意）'),vx:-100,hold:17.5},
  {...defaultSegment('第一批经中心去暂存区'),hold:8.5},
  {...defaultSegment('第一批暂存：按码平放3件（任务示意）'),vx:0,vy:100,hold:8},
  {...defaultSegment('经中心返回原料区'),vx:0,vy:-100,hold:8},
  {...defaultSegment('第二批取料入车载仓（任务示意）'),hold:8.5},
  {...defaultSegment('第二批粗加工：按码放齐3件再按序装仓（示意）'),vx:-100,hold:17.5},
  {...defaultSegment('第二批经中心去暂存区'),hold:8.5},
  {...defaultSegment('第二批暂存：对应第一批同色码垛（示意）'),vx:0,vy:100,hold:8},
  {...defaultSegment('返程经过中央车道'),vx:0,vy:-100,hold:8},
  {...defaultSegment('返程进入右侧车道'),vx:0,vy:-100,hold:8.5},
  {...defaultSegment('返程到右下斜移起点'),vx:-100,hold:8.5},
  {...defaultSegment('返回原右下启停区'),vx:-50,vy:-50,hold:2.5}
];
// Official initial envelope, not wheel-center geometry or an unfolded arm model.
export const BODY_SIZE_MM = 300;
export const DEFAULT_BODY_SIZE = Object.freeze({length_mm:BODY_SIZE_MM,width_mm:BODY_SIZE_MM});
export const TURNTABLE = Object.freeze({x:1200,y:2470,radius:150});
export function validBodySize(size){
  return !!size && Number.isFinite(size.length_mm)&&Number.isFinite(size.width_mm)&&
    size.length_mm>0&&size.length_mm<=BODY_SIZE_MM&&size.width_mm>0&&size.width_mm<=BODY_SIZE_MM;
}
export function bodyCorners(p,size=DEFAULT_BODY_SIZE){
  const halfLength=size.length_mm/2,halfWidth=size.width_mm/2;
  return [[halfLength,halfWidth],[-halfLength,halfWidth],[-halfLength,-halfWidth],[halfLength,-halfWidth]].map(v=>{
    const [x,y]=rotate(v,p.yaw_rad);return [p.x_mm+x,p.y_mm+y];
  });
}
// Use the same numerical tolerance as the outer-boundary check, not a parking allowance.
function insideZone(p,zone){
  return bodyCorners(p).every(([x,y])=>x>=zone.x-1e-7&&x<=zone.x+300+1e-7&&y>=zone.y-1e-7&&y<=zone.y+300+1e-7);
}
export function originalStartZone(p){
  return [{x:2100,y:0},{x:2100,y:2100}].find(zone=>insideZone(p,zone))??null;
}
export function routeResultText(r,{wholeRoute=true,rowNumber=r.index+1}={}){
  if(r.state==='done'){
    if(!wholeRoute)return '本段模拟结束 · 不代表整条路线或搬运任务完成';
    return r.returnedToStart?'模拟路线结束，外框已进入原启停区':'模拟路线结束 · 外框尚未进入原启停区，需末端精定位';
  }
  const issue=({boundary:'车体外框将越界',turntable:'车体将碰到转盘',forbidden_area:'车体将进入非行车区'})[r.reason]??r.reason;
  const returnHint=wholeRoute&&r.startZone&&r.index===r.rows.length-1&&r.reason==='boundary'?' · 返回阶段需末端精定位':'';
  return `第 ${rowNumber} 段：${issue} · 已在上一有效位置停止${returnHint}`;
}
export function turntableGap(p,size=DEFAULT_BODY_SIZE){
  const [x,y]=rotate([TURNTABLE.x-p.x_mm,TURNTABLE.y-p.y_mm],-p.yaw_rad),halfLength=size.length_mm/2,halfWidth=size.width_mm/2;
  return Math.hypot(Math.max(Math.abs(x)-halfLength,0),Math.max(Math.abs(y)-halfWidth,0))-TURNTABLE.radius;
}
function overlaps(corners,box,yaw){
  const other=[[box.x,box.y],[box.x+box.w,box.y],[box.x+box.w,box.y+box.h],[box.x,box.y+box.h]];
  // Separating axes of the rotated car and the axis-aligned forbidden area.
  for(const axis of [[1,0],[0,1],rotate([1,0],yaw),rotate([0,1],yaw)]){
    const a=corners.map(p=>p[0]*axis[0]+p[1]*axis[1]);
    const b=other.map(p=>p[0]*axis[0]+p[1]*axis[1]);
    if(Math.max(...a)<=Math.min(...b)+1e-7||Math.max(...b)<=Math.min(...a)+1e-7)return false;
  }
  return true;
}
export function footprintDiagnostic(p,{mask=0,bodySize=DEFAULT_BODY_SIZE}={}){
  const corners=bodyCorners(p,bodySize);
  if(corners.some(c=>c.some(v=>v < -1e-7||v > 2400+1e-7)))return {reason:'boundary',obstacle:'场地边界'};
  if(turntableGap(p,bodySize)<=0)return {reason:'turntable',obstacle:'原料转盘'};
  const boxes=[...FIELD.blocks.map(b=>({...b,w:450,h:450})),{x:0,y:910,w:150,h:580},{x:910,y:0,w:580,h:150}];
  const fixed=boxes.findIndex(b=>overlaps(corners,b,p.yaw_rad));
  if(fixed>=0)return {reason:'forbidden_area',obstacle:`非行车区 ${fixed+1}`};
  const grid=[0,550,1000,1400,1850,2400];
  for(let i=0;i<25;i++)if(mask&(1<<i)){
    const row=Math.floor(i/5),col=i%5,box={x:grid[col],y:grid[row],w:grid[col+1]-grid[col],h:grid[row+1]-grid[row]};
    if(overlaps(corners,box,p.yaw_rad))return {reason:'forbidden_area',source:'grid',obstacle:`地图障碍 R${row+1}C${col+1}`};
  }
  return null;
}
export function footprintIssue(p,options){return footprintDiagnostic(p,options)?.reason??null;}
const keys=['accel','hold','decel','dwell'];

export function validateRoute(rows){
  if(!Array.isArray(rows)||!rows.length||rows.length>50)throw new Error('路线需要 1–50 段');
  for(const [i,s] of rows.entries()){
    const error=message=>{throw new Error(`第 ${i+1} 段：${message}`);};
    if(typeof s.name!=='string'||!s.name.trim()||s.name.length>40)error('请填写 1–40 字的停车状态');
    if(!['vx','vy','omega',...keys].every(k=>Number.isFinite(s[k])))error('参数必须是有效数字');
    if(Math.hypot(s.vx,s.vy)>100.000001||Math.abs(s.omega)>.15)error('合速度 ≤100 mm/s，角速度 ≤0.15 rad/s');
    if(s.accel<=0||s.decel<=0||s.hold<0||s.dwell<0||s.dwell>60||s.accel+s.hold+s.decel>60)error('起停时间须大于 0，运动合计 ≤60 秒，停留 0–60 秒');
  }
  return rows.map(s=>({...s,name:s.name.trim()}));
}
export class RouteRunner {
  constructor(rows,pose,{geometry=DEFAULT_GEOMETRY,integer=false}={}){
    this.rows=validateRoute(rows);
    if(!validGeometry(geometry)||!pose||!['x_mm','y_mm','yaw_rad'].every(k=>Number.isFinite(pose[k])))throw new Error('几何参数和初始位姿必须有效');
    const issue=footprintIssue(pose);if(issue)throw new Error(`初始300×300 mm车体外框无效：${issue}`);
    this.startZone=originalStartZone(pose);
    this.geometry={...geometry};this.integer=integer;this.pose={...pose};this.path=[{...pose}];
    this.stops=[];this.index=0;this.phaseIndex=0;this.phaseTime=0;this.elapsed=0;this.distance=0;
    this.state='running';this.reason='';this.body=[0,0,0];this.rpm=[0,0,0,0];
  }
  get returnedToStart(){return this.state==='done'&&this.startZone!==null&&insideZone(this.pose,this.startZone);}
  get phase(){return this.state==='done'?'done':keys[this.phaseIndex];}
  factor(t){const s=this.rows[this.index];if(this.phase==='accel')return .5-.5*Math.cos(Math.PI*t/s.accel);if(this.phase==='decel')return .5+.5*Math.cos(Math.PI*t/s.decel);return this.phase==='hold'?1:0;}
  sample(t){const s=this.rows[this.index],f=this.factor(t);return [s.vx*f,s.vy*f,s.omega*f];}
  nextPhase(){
    if(this.phase==='decel')this.stops.push({index:this.index,name:this.rows[this.index].name,pose:{...this.pose},time:this.elapsed});
    this.phaseIndex++;this.phaseTime=0;
    if(this.phaseIndex===4){this.index++;this.phaseIndex=0;if(this.index===this.rows.length){this.state='done';this.index--;}}
  }
  advance(seconds){
    if(!Number.isFinite(seconds)||seconds<0)throw new Error('模拟时间无效');
    while(seconds>1e-10&&this.state==='running'){
      const remaining=this.rows[this.index][this.phase]-this.phaseTime;
      if(remaining<1e-10){this.nextPhase();continue;}
      const dt=Math.min(.02,seconds,remaining),body=this.sample(this.phaseTime+dt/2);
      const rpm=inverse(body,this.geometry,SIM_MAX_RPM);
      const velocity=forward(this.integer?quantize(rpm):rpm,this.geometry);
      const next=midpoint(this.pose,velocity,dt);
      const issue=footprintIssue(next);
      if(issue){
        // Keep the last valid pose; never draw a frame beyond the allowed envelope.
        this.state='error';this.reason=issue;break;
      }
      this.distance+=Math.hypot(next.x_mm-this.pose.x_mm,next.y_mm-this.pose.y_mm);
      this.pose=next;this.path.push({...next});
      // Retain the whole route when thinning a long display trace, rather than joining a gap.
      if(this.path.length>20000)this.path=this.path.filter((_,i)=>i%2===0||i===this.path.length-1);
      this.phaseTime+=dt;this.elapsed+=dt;seconds-=dt;
      if(this.rows[this.index][this.phase]-this.phaseTime<1e-10)this.nextPhase();
    }
    while(this.state==='running'&&this.rows[this.index][this.phase]-this.phaseTime<1e-10)this.nextPhase();
    if(this.state==='running'){this.body=this.sample(this.phaseTime);this.rpm=inverse(this.body,this.geometry,SIM_MAX_RPM);}
    else {this.body=[0,0,0];this.rpm=[0,0,0,0];}
  }
}
