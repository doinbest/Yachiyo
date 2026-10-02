import {grabParameters,turntableParameters} from './protocol.mjs';
import {RADAR_PARAMETERS} from './radar-model.mjs';

export const parameterTabs=[['material','物料取放'],['turntable','车载转盘'],['vision','视觉对准'],['motion','运动与标定'],['radar','雷达与路线']];
const pairs={z_grab:'z_lift',z_proc_grab:'z_proc_lift',z_place:'z_car_lift'};
const liftToDown=Object.fromEntries(Object.entries(pairs).map(([down,lift])=>[lift,down]));
const positions={z_grab:'原料夹取下降距离',z_lift:'原料夹取后抬升距离',z_proc_grab:'粗加工夹取下降距离',z_proc_lift:'粗加工夹取后抬升距离',z_place:'车载仓位取放下降距离',z_car_lift:'车载取出后抬升距离',z_proc_place:'粗加工释放下降距离',z_temp_place:'暂存第一层释放下降距离',z_stack_place:'暂存第二层释放下降距离'};
const materialKeys=new Set([...Object.keys(positions),'x_car','base_car_offset','x_speed','z_speed']);
const motionKeys=new Set(['x_ppm','z_ppm','x_min','x_max','x_pre','z_min','z_max','z_observe','pos_tol','stop_speed','feedback_ms','close_ms']);
export const materialScenarios=[
  ['原料夹取','z_grab','z_lift'],['粗加工夹取','z_proc_grab','z_proc_lift'],['车载取放','z_place','z_car_lift'],
  ['粗加工释放','z_proc_place'],['暂存第一层','z_temp_place'],['暂存第二层','z_stack_place']
];
const labels={x_car:'车载取放口 X 位置',base_car_offset:'车载 Base 偏转角',x_speed:'自动 X 运动速度',z_speed:'自动 Z 运动速度',close_ms:'夹爪到位等待',x_ppm:'X 尺度',z_ppm:'Z 尺度'};
function separateUnit(field){
  if(field.unit)return field;
  const match=field.label.match(/^(.*)\s\/\s([^/]+)$/);
  if(!match)return field;
  const [,label,tail]=match,[unit,...note]=tail.split(/(?=（)/);
  return {...field,label:label+(note.length?' '+note.join(''):''),unit};
}
export const parameterFields=[
  ...grabParameters.map(p=>({...p,kind:'grab',tab:materialKeys.has(p.key)?'material':motionKeys.has(p.key)?'motion':'vision',
    label:positions[p.key]??labels[p.key]??p.label,group:positions[p.key]?'Z 场景距离':p.key==='close_ms'?'夹爪动作时序':p.key==='x_car'||p.key==='base_car_offset'?'车载取放位置':materialKeys.has(p.key)?'自动取放速度':p.group,
    min:positions[p.key]?0:p.min,max:positions[p.key]?10000:p.max,unit:positions[p.key]?'mm':p.key==='x_car'?'mm':p.key==='base_car_offset'?'°':p.key==='close_ms'?'ms':p.key.endsWith('_ppm')?'脉冲/mm':p.key==='x_speed'||p.key==='z_speed'?'mm/s':'',
    note:p.key==='close_ms'?'发出开合指令后，到下一步 Z 动作的等待；通常无需调整。':undefined,
    conversion:liftToDown[p.key]?'rise':positions[p.key]?'down':null,down:liftToDown[p.key],lift:pairs[p.key]})),
  ...turntableParameters.map(p=>({...p,kind:'turntable',tab:'turntable',group:p.key.startsWith('slot')?'仓位角度':'分度与反馈'})),
  ...['base','x','z'].flatMap(axis=>[['rpm','普通运动速度 / RPM',1,3000],['acc','普通运动加速度档位',1,255],['limit',axis==='z'?'普通运动脉冲限制（固定不限）':'普通运动脉冲限制',axis==='z'?0:1,3200]].map(([part,label,min,max])=>({key:`${axis}_${part}`,kind:'arm',axis,part,label,min,max,step:1,tab:'motion',group:`${axis.toUpperCase()} 普通手动运动`,readOnly:axis==='z'&&part==='limit'}))),
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
  if(value<field.min||value>field.max)throw new Error(`${field.label}超出范围 ${field.min}～${field.max}`);
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
