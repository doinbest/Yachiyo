/** Radar 0deg direction viewed from above, relative to the vehicle front. */
export const RADAR_INSTALLATION_DIRECTIONS=[
  {key:'front',label:'车头 · 0°',relativeDeg:0},
  {key:'right',label:'车右 · −90°',relativeDeg:-90},
  {key:'rear',label:'车尾 · 180°',relativeDeg:180},
  {key:'left',label:'车左 · +90°',relativeDeg:90}
];
const numeric=value=>String(value??'').trim()!==''&&Number.isFinite(Number(value))?Number(value):null;
const normalize=value=>Number((((value%360)+360)%360).toFixed(8))%360;

export const RADAR_INSTALLATION_PRESETS=Object.freeze({
  case1:Object.freeze({label:'Caaaar 情况1 · 建图参考',coordinate_frame:'reference',situation:1,x_mm:230,y_mm:230,zero_tenths:900,matrix_q16:[65536,0,0,65536],origin_q16:[15073280,15073280]}),
  case2:Object.freeze({label:'Caaaar 情况2 · 实车当前',coordinate_frame:'current',situation:2,x_mm:2280,y_mm:314,zero_tenths:1800,matrix_q16:[65809,680,-407,66623],origin_q16:[149407997,20569846]}),
  upper:Object.freeze({label:'右上独立安装',coordinate_frame:'current',situation:1,x_mm:2050,y_mm:2250,zero_tenths:2700,matrix_q16:[65536,0,0,65536],origin_q16:[134348800,147456000]})
});

export function radarSnapshotFrame(snapshot,fallback='current'){
  if(['reference','current'].includes(snapshot?.coordinate_frame))return snapshot.coordinate_frame;
  return snapshot?.profile==='f1_host'?(snapshot.situation===2?'current':'reference'):fallback;
}
export function referencePointToCurrent(point){return [2400-point[1],point[0],...point.slice(2)];}
export function referencePoseToCurrent(pose){return [2400-pose[1],pose[0],normalize((pose[2]??0)+90)];}
export function referenceGeometryToCurrent(value){
  if(!value)return value;
  const array=Array.isArray(value),m=array?value.slice(0,4):value.matrix_q16,t=array?value.slice(4,6):value.origin_q16,zero=array?value[6]:value.zero_tenths;
  if(!m||!t)return value;
  // zero also rotates: M' = R M R^-1, t' = R t + (2400,0), all in exact Q16.
  const matrix=[m[3],-m[2]||0,-m[1]||0,m[0]],origin=[2400*65536-t[1],t[0]],heading=normalize(zero/10+90)*10;
  return array?[...matrix,...origin,heading]:{...value,matrix_q16:matrix,origin_q16:origin,zero_tenths:heading};
}
export function referenceInstallationToCurrent(values){
  const p={...values};
  if(Number.isFinite(p.lidar_x_mm)&&Number.isFinite(p.lidar_y_mm))[p.lidar_x_mm,p.lidar_y_mm]=[2400-p.lidar_y_mm,p.lidar_x_mm];
  if(Number.isFinite(p.zero_deg))p.zero_deg=normalize(p.zero_deg+90);
  if(Number.isFinite(p.zero_tenths))p.zero_tenths=normalize(p.zero_tenths/10+90)*10;
  const keys=['m00_q16','m01_q16','m10_q16','m11_q16','tx_q16','ty_q16'];
  if(keys.every(k=>Number.isFinite(values[k]))){
    const g=referenceGeometryToCurrent([...keys.map(k=>values[k]),values.zero_tenths??(values.zero_deg??0)*10]);
    keys.forEach((k,i)=>p[k]=g[i]);p.zero_tenths=g[6];p.zero_deg=g[6]/10;
  }
  return p;
}

/** Draft values only: selecting or storing a preset never writes to hardware. */
export function installationParameters(key='case2',frame){
  const preset=RADAR_INSTALLATION_PRESETS[key];if(!preset)throw new Error('未知安装预设');
  const values={lidar_x_mm:preset.x_mm,lidar_y_mm:preset.y_mm,zero_deg:preset.zero_tenths/10,
    situation:preset.situation,grid_version:4,zero_tenths:preset.zero_tenths,
    m00_q16:preset.matrix_q16[0],m01_q16:preset.matrix_q16[1],m10_q16:preset.matrix_q16[2],m11_q16:preset.matrix_q16[3],
    tx_q16:preset.origin_q16[0],ty_q16:preset.origin_q16[1]};
  return frame==='current'&&preset.coordinate_frame==='reference'?referenceInstallationToCurrent(values):values;
}

export function stableFieldPoint(point){
  const pair=Array.isArray(point)?point.slice(2,4):[point?.x_mm,point?.y_mm];
  return pair.length===2&&pair.every(Number.isFinite)?pair:null;
}

/** Drawing uses capture metadata; editing a draft must not move an old cloud. */
export function projectFrozenRadarPoint(point,snapshot){
  const angle=Array.isArray(point)?point[0]/10:point?.angle_deg,distance=Array.isArray(point)?point[1]:point?.distance_mm;
  if(!Number.isFinite(angle)||!Number.isFinite(distance))return null;
  const rawGeometry=snapshot?.geometry??snapshot?.affine;
  const geometry=Array.isArray(rawGeometry)?{matrix_q16:rawGeometry.slice(0,4),origin_q16:rawGeometry.slice(4,6),zero_tenths:rawGeometry[6]}:rawGeometry;
  const matrix=geometry?.matrix_q16??snapshot?.matrix_q16,origin=geometry?.origin_q16??snapshot?.origin_q16;
  const zero=geometry?.zero_tenths??snapshot?.zero_tenths;
  if(Array.isArray(matrix)&&matrix.length===4&&Array.isArray(origin)&&origin.length===2&&Number.isFinite(zero)){
    const radians=(zero/10-angle)*Math.PI/180,x=distance*Math.cos(radians),y=distance*Math.sin(radians);
    return [(matrix[0]*x+matrix[1]*y+origin[0])/65536,(matrix[2]*x+matrix[3]*y+origin[1])/65536].map(v=>Math.abs(v)<1e-10?0:v);
  }
  const pose=snapshot?.pose;if(!Array.isArray(pose)||pose.length!==3)return null;
  const radians=(pose[2]-angle)*Math.PI/180;
  return [pose[0]+distance*Math.cos(radians),pose[1]+distance*Math.sin(radians)].map(v=>Number(v.toFixed(8)));
}

/** Convert an installation draft to existing zero_deg at nominal lower yaw 90deg. */
export function radarZeroFromRelative(value){
  const angle=numeric(value);return angle===null?null:normalize(90+angle);
}

/** Interpret a parameter value; this does not confirm the physical installation. */
export function radarInstallationFromZero(value){
  const zero=numeric(value);if(zero===null)return null;
  const wrapped=normalize(zero-90),relativeDeg=Number((wrapped>180?wrapped-360:wrapped).toFixed(8));
  const preset=RADAR_INSTALLATION_DIRECTIONS.find(item=>Math.abs(item.relativeDeg-relativeDeg)<1e-8);
  return {direction:preset?.key??'custom',relativeDeg};
}

/** Static corner heading to the lower reference; upper scans face 90deg further CCW. */
export function radarLowerReferenceFromWorld(start,value){
  const angle=numeric(value);return angle===null||!['upper','lower'].includes(start)?null:normalize(angle-(start==='upper'?90:0));
}

/** Interpret a lower-reference readback at a chosen start, without physical confirmation. */
export function radarWorldFromLowerReference(start,value){
  const angle=numeric(value);return angle===null||!['upper','lower'].includes(start)?null:normalize(angle+(start==='upper'?90:0));
}
