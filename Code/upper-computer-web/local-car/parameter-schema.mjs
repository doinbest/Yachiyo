import {grabParameters,turntableParameters} from './protocol.mjs';
import {RADAR_PARAMETERS} from './radar-model.mjs';

export const parameterTabs=[['material','物料取放'],['turntable','车载转盘'],['vision','视觉对准'],['motion','运动与标定'],['radar','雷达与路线']];
const pairs={z_grab:'z_lift',z_place:'z_car_lift'};
const liftToDown=Object.fromEntries(Object.entries(pairs).map(([down,lift])=>[lift,down]));
const positions={z_grab:'原料夹取下降距离',z_lift:'原料夹取后抬升距离',z_place:'车载仓位取放下降距离',z_car_lift:'车载取出后抬升距离',z_proc_place:'加工取放 / 第一层暂存下降距离',z_stack_place:'暂存第二层释放下降距离'};
const materialKeys=new Set(['z_rough_align','z_temp_align',...Object.keys(positions),'base_car_offset','x_speed','z_speed','z_pick_speed','track_store_drop_mm','car_pick_drop_mm']);
const timeKeys=new Set(['close_ms','release_ms','car_pick_wait_ms']);
const motionKeys=new Set(['x_ppm','z_ppm','x_min','x_max','x_pre','z_min','z_max','z_observe','pos_tol','stop_speed','feedback_ms',...timeKeys,'base_home_degrade']);
export const materialScenarios=[
  ['原料夹取','z_grab','z_lift'],['车载取放','z_place','z_car_lift'],
  ['加工取放 / 第一层暂存','z_proc_place'],['暂存第二层','z_stack_place']
];
const zSpeedKeys=new Set(['z_speed','z_pick_speed','qr_z_speed']);
const labels={base_car_offset:'车载 Base 偏转角',x_speed:'自动 X 运动速度',z_speed:'Z 统一任务速度',z_pick_speed:'Z 统一速度（夹取兼容）',close_ms:'闭爪后等待',release_ms:'松爪后等待',car_pick_wait_ms:'车仓开口后等待',x_ppm:'X 尺度',z_ppm:'Z 尺度'};
function separateUnit(field){
  if(field.unit)return field;
  const match=field.label.match(/^(.*)\s\/\s([^/]+)$/);
  if(!match)return field;
  const [,label,tail]=match,[unit,...note]=tail.split(/(?=（)/);
  return {...field,label:label+(note.length?' '+note.join(''):''),unit};
}
export const parameterFields=[
  ...grabParameters.map(p=>({...p,kind:'grab',tab:materialKeys.has(p.key)?'material':motionKeys.has(p.key)?'motion':'vision',
    label:positions[p.key]??labels[p.key]??p.label,group:positions[p.key]?'Z 场景距离':timeKeys.has(p.key)?'夹爪动作时序':p.key==='base_car_offset'?'车载取放位置':materialKeys.has(p.key)?'自动取放速度':p.group,
    min:positions[p.key]?0:p.min,max:positions[p.key]?10000:p.max,unit:positions[p.key]?'mm':p.key==='base_car_offset'?'°':timeKeys.has(p.key)?'ms':p.key.endsWith('_ppm')?'脉冲/mm':p.key==='x_speed'||p.key==='z_speed'||p.key==='z_pick_speed'?'mm/s':'',
    note:p.key==='vision_wait_ms'?'首次等待与停稳后重获颜色的等待窗口；不会延长旧坐标有效期。':p.key==='close_ms'?'闭爪后，到下一步 Z 提起的等待。':p.key==='release_ms'?'松爪后，到提起与库存记账的等待。':p.key==='car_pick_wait_ms'?'实际设置2120 μs开口后，到下降取料的等待；可与Base回仓转向重叠，下降前仍确认车仓和Base到位。':undefined,
    conversion:liftToDown[p.key]?'rise':positions[p.key]?'down':null,down:liftToDown[p.key],lift:pairs[p.key]})),
  ...turntableParameters.map(p=>({...p,kind:'turntable',tab:'turntable',group:p.key.startsWith('slot')?'仓位角度':'分度与反馈'})),
  ...['base','x','z'].flatMap(axis=>[['rpm','普通运动速度 / RPM',1,3000],['acc','驱动加速度档位',axis==='base'?1:0,255],['limit',axis==='z'?'普通运动脉冲限制（固定不限）':'普通运动脉冲限制',axis==='z'?0:1,3200]].map(([part,label,min,max])=>({key:`${axis}_${part}`,kind:'arm',axis,part,label,min,max,step:1,tab:'motion',group:`${axis.toUpperCase()} 轴运动配置`,readOnly:axis==='z'&&part==='limit',note:part==='acc'?'固定X/Z任务、手动位置及视觉连续纠偏使用对应驱动档位；X/Z默认0（不使用驱动曲线，直接按目标速度运行），Base默认20；任务速度由grab参数决定。':undefined}))),
  ...RADAR_PARAMETERS.map(p=>({...p,kind:'radar',tab:'radar',group:'雷达下次静止扫描',step:p.key==='zero_deg'?.1:1})),
  {key:'route_speed',kind:'local',tab:'radar',group:'浏览器本地路线设置',label:'下次路线启动速度',unit:'mm/s',min:10,max:5000,step:1}
].map(separateUnit);
export function displayValue(field,values){
  const value=values[field.key];
  if(value===null||value===undefined)return null;
  if(field.conversion==='down')return -value;
  if(field.conversion==='rise')return Number.isFinite(values[field.down])?value-values[field.down]:null;
  return value;
}
const number=(value,label)=>{if(String(value??'').trim()===''||!Number.isFinite(Number(value)))throw new Error(`请填写${label}`);return Number(value);};
export function fieldWrites(field,drafts){
  const value=number(drafts[field.key],field.label);
  if(value<field.min||(field.exclusiveMin&&value===field.min)||value>field.max||(field.step===1&&!Number.isInteger(value)))throw new Error(`${field.label}超出范围或数值类型不符 ${field.min}～${field.max}`);
  if(field.kind==='grab'&&zSpeedKeys.has(field.key))return [...zSpeedKeys].map(key=>[key,value]);
  if(field.conversion==='down'){
    const target=-value;
    return field.lift?[[field.key,target],[field.lift,target+number(drafts[field.lift],'对应抬升距离')]]:[[field.key,target]];
  }
  if(field.conversion==='rise')return [[field.key,-number(drafts[field.down],'对应下降距离')+value]];
  return [[field.key,value]];
}
export function deviceRequests(){
  const rows=parameterFields.filter(p=>p.kind==='grab'||p.kind==='turntable').map(p=>({kind:p.kind,key:p.key}));
  return [...rows,...['base','x','z'].map(key=>({kind:'arm',key})),{kind:'radar',key:'all'}];
}
