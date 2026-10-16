import {chineseTerminalLine,replyIsFault} from './terminal-format.mjs?v=lidar-poses-20261012';
import {deviceText,reasonText} from './device-text.mjs';
import {missionResumeStatus} from './mission-resume-status.mjs';
import {missionSpeedSettings} from './mission-scale.mjs';
export {deviceText,reasonText};
import {GRAB_FORM_DEFAULTS,GRAB_TEST_PRESET} from './grab-params.mjs?v=car-transfer-speed-20261007';
import {GRAB_PARAMETER_METADATA} from './grab-parameter-metadata.mjs';
export {replyIsFault};
export function positionDegreeRange(axis) {
  return axis === 'z' ? [-3600,3600] : [-360,360];
}
// Source: Code/template/App/arm_console.c. UI labels never go over the wire.
/** One relative Z step, using the operator's measured command pulses/mm. */
export function buildFixedZJog(ppmValue, mmValue) {
  const ppm=Number(ppmValue),mm=Number(mmValue);
  if(String(ppmValue).trim()===''||!Number.isFinite(ppm)||ppm<=0||ppm>100000||
     !Number.isFinite(mm)||mm===0||Math.abs(mm)>20) throw new Error('填写有效的 Z 脉冲/mm；单次位移应在 ±20 mm 内且非零。');
  const pulses=Math.round(Math.abs(mm)*ppm);
  if(pulses<1) throw new Error('位移不足一个命令脉冲。');
  const degrees=Number((Math.sign(mm)*pulses*360/3200).toFixed(6));
  return `pos z ${degrees}`;
}
const choice = (key, label, options, value = options[0][0]) => ({key,label,options,value});
const number = (key,label,value,min,max,step=1) => ({key,label,value,min,max,step});
const axes = [['base','底座 · base'],['z','升降 · z'],['x','伸缩 · x']];
const axis = (all=false) => choice('axis','目标轴',all ? [...axes,['all','全部机械臂轴 · all']] : axes);
const target = () => choice('target','目标颜色',['红色','黄色','蓝色','绿色','黑色','浅蓝色'].map((label,i) => [String(i+1),`${label} · ${i+1}`]));
export const grabColorOptions = target().options;
const command = (id,group,label,template,fields=[],note='') => ({id,group,label,template,fields,note});
export const groups = [['arm','机械臂'],['grip','夹爪'],['chassis','底盘'],['vision','视觉'],['qr','二维码'],['system','系统']];
// Firmware startup defaults; displayed inputs are not device readback.
const grabDefaults=GRAB_FORM_DEFAULTS;
export const grabParameters = [
  ...[['z_ppm','Z 命令脉冲 / mm'],['x_ppm','X 命令脉冲 / mm']].map(([key,label])=>({key,label,group:'尺度与行程'})),
  ...[['z_min','Z 最小位置'],['z_max','Z 最大位置'],['z_grab','Z 夹持位置'],['z_place','Z 放置位置'],['z_lift','Z 提起位置'],['z_observe','Z 观察位置'],['x_min','X 最小位置'],['x_max','X 最大位置'],['x_pre','X 预备位置']].map(([key,label])=>({key,label:`${label} / mm`,group:'尺度与行程'})),
  ...[['pos_tol','轴到位容差 / mm'],['stop_speed','停稳速度阈值 / mm·s⁻¹'],['dx_tol','DX 抓取窗口 / px'],['dy_tol','DY 抓取窗口 / px']].map(([key,label])=>({key,label,group:'反馈与抓取条件'})),
  ...[['close_ms','闭爪后等待 / ms'],['release_ms','松爪后等待 / ms'],['car_pick_wait_ms','车仓开口后等待 / ms'],['stable_ms','精定位稳定时间 / ms'],['arrival_hold_ms','机构到位稳定时间 / ms']].map(([key,label])=>({key,label,group:'反馈与抓取条件'})),
  ...[['feedback_ms','反馈最大年龄 / ms'],['age_ms','有效坐标接收超时 / ms']].map(([key,label])=>({key,label,group:'反馈与抓取条件'})),
  {key:'vision_wait_ms',label:'目标颜色等待窗口 / ms',group:'反馈与抓取条件'},
  {key:'base_home_degrade',label:'Base 回零降级继续（1 开启 / 0 严格）',group:'回零策略'},
  {key:'loss_grace_ms',label:'视觉对准短暂漏检容忍 / ms（0 关闭）',group:'反馈与抓取条件'},
  {key:'frames',label:'稳定坐标观测次数',group:'反馈与抓取条件'},
  {key:'retries',label:'重新对准次数上限',group:'反馈与抓取条件'},
  ...[['j00','DX / 底盘前进'],['j01','DX / 底盘左移'],['j02','DX / X 伸出'],['j10','DY / 底盘前进'],['j11','DY / 底盘左移'],['j12','DY / X 伸出']].map(([key,label])=>({key,label:`${label} / px·mm⁻¹`,group:'准心与像素响应'})),
  ...[['body_speed','底盘速度上限'],['x_speed','X 视觉对准速度上限'],['x_transfer_speed','X 正式取放 / 入仓测试速度'],['z_speed','Z 速度上限']].map(([key,label])=>({key,label:`${label} / mm·s⁻¹`,group:'控制与运动边界'})),
  {key:'z_pick_speed',label:'物料夹取下降速度 / mm·s⁻¹',group:'控制与运动边界'},
  {key:'track_pick_tol_px',label:'追踪夹取误差窗口 / ±px',group:'反馈与抓取条件'},
  {key:'track_pick_hold_ms',label:'高位下降窗口保持时间 / ms',group:'反馈与抓取条件'},
  {key:'track_store_drop_mm',label:'追踪入仓从转移高度下降 / mm',group:'取放场景'},
  {key:'car_pick_drop_mm',label:'车仓取出并保持的下降 / mm',group:'取放场景'},
  {key:'accel',label:'平移加速度上限 / mm·s⁻²',group:'控制与运动边界'},
  {key:'travel_mm',label:'局部底盘行程上限 / mm（0 关闭）',group:'控制与运动边界'},
  {key:'lead_mm',label:'X 目标领先反馈上限 / mm',group:'控制与运动边界'},
  {key:'lambda',label:'视觉收敛增益 / s⁻¹',group:'控制与运动边界'},
  {key:'x_weight',label:'X 分配权重',group:'控制与运动边界'},
  ...['z_rough_align','z_temp_align','z_car_lift','z_proc_place','z_stack_place','base_car_offset'].map(key=>({key,label:({z_rough_align:'粗加工视觉对准Z',z_temp_align:'暂存视觉对准Z',z_car_lift:'车载取出提起Z',z_proc_place:'加工取放 / 第一层暂存Z',z_stack_place:'暂存第二层释放Z',base_car_offset:'车载Base偏转角'})[key],group:'取放场景'})),
  ...[['qr_scan_y','二维码接近点 Y / mm'],['qr_z_min','扫码下降最低 Z / mm'],['qr_z_speed','扫码下降速度 / mm·s⁻¹'],['qr_wait_ms','到位后扫码等待 / ms']].map(([key,label])=>({key,label,group:'二维码测试'})),
  ...[['head_proc_zero','加工色环图像零角 / °'],['head_proc_gain','加工色环车体角 / 图像角'],['head_temp_zero','暂存色环图像零角 / °'],['head_temp_gain','暂存色环车体角 / 图像角'],['head_line_zero','码垛黑线图像零角 / °'],['head_line_gain','码垛黑线车体角 / 图像角'],['head_tol','航向完成容差 / °'],['head_spread','航向观测波动范围 / °'],['head_settle_ms','停稳后接收隔离时间 / ms'],['head_proc_turn','加工已验证可转范围 / ±°'],['head_temp_turn','暂存已验证可转范围 / ±°']].map(([key,label])=>({key,label,group:'航向实测标定'})),
].map(p=>({...p,...GRAB_PARAMETER_METADATA[p.key]}));
export const turntableParameters=[
  ...[['slot1_deg','第一仓位 / °'],['slot2_deg','第二仓位 / °'],['slot3_deg','第三仓位 / °']].map(([key,label])=>({key,label,min:-360,max:360,readOnly:key==='slot1_deg'})),
  {key:'speed_rpm',label:'分度速度 / RPM',min:1,max:5000,step:1},
  {key:'acc',label:'加速度档位',min:0,max:255,step:1},
  {key:'pos_tol_deg',label:'到位容差 / °',min:0.1125,max:180},
  {key:'stable_ms',label:'停稳时间 / ms',min:0,max:60000,step:1},
  {key:'feedback_ms',label:'反馈新鲜度 / ms',min:100,max:60000,step:1}
];
export const commands = [
  ...turntableParameters.map(p=>command(`turntable-get-${p.key}`,'arm',`读取 ${p.label}`,`turntable get ${p.key}`)),
  ...turntableParameters.filter(p=>!p.readOnly).map(p=>command(`turntable-set-${p.key}`,'arm',p.label,`turntable set ${p.key} {value}`,[number('value',p.label,'',p.min,p.max,p.step??'any')])),
  ...[['save_zero','保存机械零点'],['home','就近回零'],['enable','使能转盘'],['disable','失能转盘'],['origin','建立转盘参考'],['stop','停止车载转盘'],['status','读取转盘状态'],['inventory','读取三仓库存']].map(([id,label])=>command(`turntable-${id}`,'arm',label,`turntable ${id}`)),
  command('turntable-empty','arm','确认三仓为空','turntable inventory empty'),
  command('turntable-index','arm','分度到仓位','turntable index {slot}',[choice('slot','仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']])]),
  command('turntable-jog','arm','转盘点动','turntable jog {degree}',[number('degree','相对角度 / °','',-360,360,.1)]),
  command('turntable-inventory-set','arm','校正仓位库存','turntable inventory {slot} {content}',[choice('slot','仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('content','实际库存',[['empty','空仓'],['unknown','待核对'],['1','红色'],['2','黄色'],['3','蓝色'],['4','绿色'],['5','黑色'],['6','浅蓝色']])]),
  command('camera-ring','vision','仅识别编号环 · 不运动','camera ring {ring}',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']])],'只请求所选数字同心圆的坐标；不移动底盘或机械臂。'),
  command('grab-ring-align-options','vision','编号环视觉对准','grab align ring {ring} {z}',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']]),choice('height','对准高度',[['raised','高位对准 · Z=0'],['lowered','下降对准 · 仅对照测试']]),number('z','下降对准 Z 绝对高度 / mm',-100,-150,0,1),choice('finish','完成后动作',[['stay','保持对准高度'],['raise','抬回上限 · 默认Z=0']])],'对准高度和完成后的抬升分别选择；高位模式忽略下降高度输入。'),
  command('grab-ring-align','vision','底盘 + X 对准所选编号环','grab align ring {ring} {z}',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']]),number('z','对准时Z绝对高度 / mm',-100,-150,0,1)],'Z最高处识别编号，先底盘和X横向接近，再随环心接近准心逐步下降；靠画面边缘时暂缓下降。完成后停止视觉控制、Z抬至最高，再手动装料。DX/DY各≤1像素。'),
  command('grab-ring-run','vision','编号环完整取仓放置','grab ring run {ring} {slot} {drop} {carDrop}',[choice('ring','目标编号环',[['1','1号环'],['2','2号环'],['3','3号环']]),choice('slot','取料车仓',[['1','1号料盘'],['2','2号料盘'],['3','3号料盘']]),number('drop','本次放置下降距离 / mm',-GRAB_FORM_DEFAULTS.z_proc_place,0.1,150,0.1),number('carDrop','车仓下降／等量上升距离 / mm',GRAB_FORM_DEFAULTS.car_pick_drop_mm,0.1,150,0.1)],'空爪放在粗加工编号环前，机构已回零。自动准备作业姿态、视觉航向矫正、高位对准指定环、保存位置、指定仓取料、恢复位置并下降放置提起。下降距离从Z=0向下计算，只作用本次，不改比赛默认；航向未验证不取料。不扫码、不跑比赛路线、不重置其他库存。'),
  command('grab-ring-take','vision','编号环自动取仓放置','grab ring take {slot} {drop} {carDrop}',[choice('slot','取料仓位',[['1','1号料盘'],['2','2号料盘'],['3','3号料盘']]),number('drop','放料下降距离 / mm',-GRAB_FORM_DEFAULTS.z_proc_place,0.1,150,0.1),number('carDrop','车仓下降／等量上升距离 / mm',GRAB_FORM_DEFAULTS.car_pick_drop_mm,0.1,150,0.1)],'先完成上方编号环对准并保持空爪。STM32重新核验停稳与视觉，保存实际X/Z，再从所选料盘按本次车仓距离取料，Base转回作业方向、恢复X并按本次放料距离下降释放、提起；无需颜色或库存登记。'),
  command('grab-ring-store','vision','从当前编号环取回车仓并回位','grab ring store {slot} {drop} {carDrop}',[choice('slot','放回车仓',[['1','1号料盘'],['2','2号料盘'],['3','3号料盘']]),number('drop','外部夹取下降距离 / mm',-GRAB_FORM_DEFAULTS.z_proc_place,0.1,150,0.1),number('carDrop','车仓下降／等量上升距离 / mm',GRAB_FORM_DEFAULTS.car_pick_drop_mm,0.1,150,0.1)],'保持当前已对准的工位位置，空爪在作业方向。下降夹取、抬Z收X、转到指定车仓、下降放料、再抬Z、伸X30 mm后Base回作业方向；最终Z=0、X相对收回参考伸出30 mm。无新视觉对准，不扫码跑路，仅作用本次参数。'),
  command('grab-ring-place','vision','下降放置并提起','grab ring place {z}',[number('z','放置Z绝对位置 / mm（可调整）',-150,-150,-0.1,0.1)],'先完成所选环对准，人工放入物料并闭合夹爪。仅Z下降、反馈到位后松爪，再提起至观察高度；不取仓、不改变库存。'),
  command('grab-ring-align-stay','vision','对准后保持低位','grab align ring {ring} {z} stay',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']]),number('z','对准时Z绝对高度 / mm',-100,-150,0,1)],'按所设高度渐进下降，对准完成后保持低位，仅继续识别编号环供观察。'),
  command('grab-stack-align','vision','码垛视觉对准','grab stack align {color}',[choice('color','下层物料颜色',grabColorOptions,'3')],'Z保持最高，直接按视觉DX/DY对准并停稳后等待装料；补偿由香橙派完成。'),
  command('grab-stack-lower','vision','单独调整码垛高度','grab stack lower {z}',[number('z','Z绝对目标 / mm',-100,-150,0,0.1)],'只调整Z，保持夹持；反馈到位后保持高度。'),
  command('grab-stack-release','vision','单独松爪','grab stack release',[],'高度到位后松爪，保持当前Z；不自动提起。'),
  command('grab-stack-lift','vision','提起到最高','grab stack lift',[],'仅Z提起；不重新对准或操作转盘。'),
  command('grab-place-car','vision','机械臂到车载口放下','grab place car',[],'以已验证的后向回零位置为基准转到车载口，X保持收回，Z下降到z_place后自动松爪，并停留在放料姿态。此动作不分度ID8、不修改库存。'),
  command('grab-fixed-slot','vision','指定仓位固定夹取','grab start fixed {slot} {color}',[choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('color','物料颜色',grabColorOptions,'3')]),
  command('grab-pick-slot','vision','指定仓位视觉夹取','grab start pick {color} {slot}',[choice('color','物料颜色',grabColorOptions,'3'),choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']])]),
  command('grab-store-proc','vision','粗加工取回入仓','grab store proc {slot} {color}',[choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('color','物料颜色',grabColorOptions,'3')]),
  command('grab-take','vision','车载取出并释放','grab take {slot} {scene}',[choice('slot','取出仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('scene','释放场景',[['rough','粗加工'],['temp1','暂存第一层'],['temp2','暂存第二层']])]),
  command('grab-take-work','vision','车仓夹取并转到作业朝向','grab take {slot} work {depth}',[choice('slot','夹取几号料盘',[['1','1号料盘'],['2','2号料盘'],['3','3号料盘']]),number('depth','下降 / 等量上升距离 / mm',GRAB_FORM_DEFAULTS.car_pick_drop_mm,0.1,150,0.1)],'按所选料盘直接取出，无需登记库存或颜色。提起收X，ID8先分度到位，Base回车仓参考；等待取料开口稳定（默认300 ms），默认下降40mm闭爪，再上升40mm。1/2号盘提起同时向外伸X30mm，3号盘保持收回；两轴到位后Base转到参考+117.5°并保持闭爪。已中断时选择原仓显式继续，保留本次绝对目标；完成后仓位置空。'),
  command('grab-return','vision','返回外侧观察姿态','grab return'),
  command('grab-base-pos','vision','手动 Base 转到转盘','pos base {degree}',[number('degree','Base 电机轴相对角度 / °',-180,-360,360,0.1)],'自动取放已包含转向，此处仅用于手动定位。默认相对转动 −180°，每次执行都会再转一次，已在转盘方向时无需重复。若抓取处于 hold，先结束本次抓取并确认停止，再操作 Base。'),
  ...[['fixed','固定位置抓取'],['align','仅协同对准'],['pick','单件视觉抓取']].map(([mode,label])=>command(`grab-${mode}`,'vision',label,mode==='align'?'grab align color 3 0':`grab start ${mode}`)),
  ...[['align','仅协同对准'],['pick','单件视觉抓取']].map(([mode,label])=>command(`grab-${mode}-color`,'vision',label,mode==='align'?'grab align color {color} 0':`grab start ${mode} {color}`,[choice('color','物料颜色',grabColorOptions,'3')])),
  command('grab-route','vision','启动蓝色首件入仓测试','grab route {speed}',[number('speed','路线平移速度 / mm·s⁻¹',1000,10,5000)],'右下起点、机械臂朝后；二维码点直接通过，到原料台抓蓝色入第一仓，再收回朝后结束。不覆盖任务码；第一仓需有参考、已配置且为空，先在地图页准备跑图。'),
  command('grab-mission','vision','扫码并启动两批比赛','grab mission qr {speed} {turnSpeed} lower fixed vhigh linear',[number('speed','基准总限速 / mm/s',1000,10,5000),number('turnSpeed','转向速度 / °/s',100,5,360),number('scale','比赛倍率K',1,0.1,5,0.1)],'正式linear底盘平移总限速、各段速度及起步/停车加速度统一乘K；转向、视觉、机构及等待独立。固定任务码在比赛任务页选择。'),
  command('grab-mission-continue','vision','补齐参数后继续本件','grab mission continue'),
  command('grab-mission-pause','vision','暂停比赛','grab mission pause',[],'停车并保留本轮进度，显式继续才恢复。'),
  command('grab-mission-stop','vision','停止比赛','grab mission stop',[],'停车后结束本轮，不再续接；保持夹爪、参数、参考及已完成库存。'),
  command('grab-pose-rear','vision','收回并朝后','grab pose rear',[],'按新鲜停稳位置完成剩余抬Z、收X、转Base；普通停止后可直接重试。手动Base就近回零后可局部验证参考，不重新回零X/Z。'),
  command('grab-pose-work','vision','外侧作业姿态','grab pose work',[],'准备过程中可停止，确认停稳后直接重试；已完成动作不重复。手动Base就近回零后先局部确认本轴参考，再继续准备。'),
  command('grab-base-status','vision','读取Base到位诊断','grab status base'),
  command('grab-status','vision','刷新抓取状态','grab status'),
  command('grab-stop','vision','停止单件抓取','grab stop',[],'停止自动推进并请求机构停止；保留夹持和 Z 承载。停止确认以新鲜反馈为准。'),
  command('grab-rehome','vision','重新建立三轴参考','grab rehome',[],'先移走夹持物并确认机械臂周围无障碍；并行启动Z/X/Base回零后逐轴验证，恢复对应默认参考。普通准备停止可直接重试，只有Base参考待验证时可由准备入口局部恢复。'),
  command('grab-home-accept','vision','确认 Base 近似参考并继续','grab home accept',[],'仅用于本次 Base 回零失败后，人工确认夹爪已接近车仓方向。重新读取位置、实际转速和驱动状态，停稳后接受近似参考；不重发回零或重复 X/Z 动作。'),
  ...grabParameters.map(p=>command(`grab-get-${p.key}`,'vision',`读取 ${p.label}`,`grab get ${p.key}`)),
  ...grabParameters.map(p=>command(`grab-set-${p.key}`,'vision',p.label,`grab set ${p.key} {value}`,[{...number('value',p.label,grabDefaults[p.key]??'',p.min,p.max,p.step??'any'),exclusiveMin:p.exclusiveMin}],p.key in grabDefaults?'预填程序默认值，不是设备读回值；X/Z尺度和位置在上电或 grab rehome 对应轴验证成功后载入，容差和等待时间在启动时载入。单轴手动回零只受理动作，不自动恢复参考参数。修改仅存MCU RAM，运行中拒绝修改。':'仅设置 MCU RAM；复位后需重新配置。运行中拒绝修改，输入值不代表设备已接受。')),
  command('pos','arm','相对转动','pos {axis} {degree}',[axis(),number('degree','电机轴相对角度 / °',10,-360,360,0.1)],'正负值表示方向；这是相对运动，不是绝对位置。Z 轴单次输入范围 −3600～3600°，Base/X 为 −360～360°；固件另有脉冲限制。'),
  ...[['enable','使能'],['disable','取消使能'],['state','查询状态'],['stop','停止']].map(([id,label]) => command(id,'arm',label,`${id} {axis}`,[axis(true)])),
  command('position','arm','读取当前位置','position {axis}',[axis()],'返回值为脉冲数。'),
  command('home','arm','执行回零','home {axis} {mode}',[axis(true),choice('mode','回零方式',[['near','就近 · near'],['dir','指定方向 · dir'],['collision','碰撞 · collision'],['limit','限位 · limit']])],'回零会产生运动；方式需要与实际机构匹配。'),
  command('origin','arm','保存机械零点','origin {axis}',[axis(true)],'将当前机械位置保存为零点，与计数清零不同。'),
  command('zero','arm','当前位置计数清零','zero {axis}',[axis(true)],'只清除当前位置计数，不执行回零运动。'),
  command('config-get','arm','读取运动配置','config {axis}',[axis(true)]),
  command('config-set','arm','设置运动配置','config {axis} {rpm} {acc} {limit}',[axis(true),number('rpm','速度 / RPM',30,1,3000),number('acc','加速度参数',20,0,255),number('limit','脉冲限制',3200,0,3200)],'配置只保存在 RAM，复位后恢复默认；X/Z允许加速度0档（直接按目标速度运行），Base及all保持1～255；只有单独 Z 轴允许 limit=0。'),
  ...[['open','打开 · 2450 µs'],['catch','夹取 · 1920 µs'],['idle','待机松开 · 2450 µs']].map(([id,label]) => command(`grip-${id}`,'grip',label,`grip ${id}`)),
  command('grip-pulse','grip','自定义夹爪脉宽','grip duty {duty}',[number('pulse','脉宽 / μs',2450,500,2500,1)],'自动打开统一2450 μs，夹取1920 μs。仅点击应用时改变当前开口，输入值不改写任务参数。'),
  command('grip-duty','grip','手动设置占空比','grip duty {duty}',[number('duty','PWM 占空比 / %',9.6,2.5,12.5,0.1)],'50 Hz PWM；打开2450 µs = 12.25%，夹取1920 µs = 9.6%。'),
  command('lidar-servo-out','arm','转出扫描 · 2450 μs','lidarservo out',[],'用户实测 TD-8125MG 雷达盘转出脉宽2450 μs；出发前转出，确认机构稳定后扫描地图。'),
  command('lidar-servo-in','arm','收回车内 · 1450 μs','lidarservo in',[],'用户实测 TD-8125MG 雷达盘收回脉宽1450 μs；扫描完成后收回，再出发。'),
  command('lidar-servo-pulse','arm','设置雷达舵机脉宽','lidarservo pulse {pulse}',[number('pulse','脉宽 / μs',1450,500,5000,1)],'手动脉宽调试；实测转出2450 μs、收回1450 μs。仅点击应用才动作。'),
  command('lidar-servo-status','arm','读取雷达舵机输出状态','lidarservo status',[],'查询已设置脉宽，不代表雷达盘实际到位或电流已下降。'),
  command('move','chassis','定距前后移动','chassis {direction} {distance}',[choice('direction','移动方向',[['forward','前进'],['backward','后退']]),number('distance','距离 / mm',10,1,200)],'始终带距离发送，避免误触发持续后退。'),
  command('velocity','chassis','持续速度控制','chassis velocity {forward} {left} {yaw}',[number('forward','前向速度 / mm·s⁻¹',0,-1000,1000),number('left','左向速度 / mm·s⁻¹',0,-1000,1000),number('yaw','偏航速度 / °·s⁻¹',0,-500,500)],'持续运动需主动点击「底盘停止」。关闭网页或断开串口不等于停车。'),
  command('chassis-stop','chassis','底盘停止','chassis stop',[],'固件可能返回 busy；以实际回复和小车状态为准。'),
  command('wheel','chassis','单轮测试','wheel {wheel} {rpm}',[choice('wheel','选择车轮',[['fl','左前 · fl'],['rl','左后 · rl'],['rr','右后 · rr'],['fr','右前 · fr']]),number('rpm','转速 / RPM',10,-100,100)],'正负值表示方向；停止请使用「车轮停止」。'),
  command('wheel-stop','chassis','车轮停止','wheel stop'),
  command('grab-calib-ref','system','确认标定观察姿态','grab calib ref',[],'只确认当前观察姿态，不运动、不回零。'),
  command('grab-calib','system','前后与X视觉响应标定','grab calib {target}',[target()],'会前后移动底盘并小幅移动X；Base、Z和夹爪保持当前状态。成功写入共用视觉模型。'),
  command('grab-calib-status','system','读取视觉标定状态','grab calib status'),
  command('camera-material','vision','相机识别物料','camera material {target}',[target()]),
  command('camera-stop','vision','停止相机请求','camera stop'),
  command('camera-status','vision','相机与 USB 状态','camera status',[],'读取香橙派最新坐标及USB状态；使用B2颜色识别。'),
  command('system-reset','system','重启 STM32','system reset',[],'仅空闲时重启主控；会重新执行初始化，不能代替停车或电机驱动器复位。'),
  command('imu-zero','system','Z 轴角度清零','imu zero',[],'保持静止；只将当前模块角度设为零，不做零偏标定。之后重新验证 IMU 并设置地图起点。'),
  command('info','system','设备信息','info'),
  command('bus-recover','system','一键恢复电机总线','bus recover',[],'停止任务并检查全部 8 台电机（含车载转盘 ID 8）；成功后不会自动续跑。结果见终端，底盘同步缓存未确认时仍保持底盘锁定。'),
  command('bus-status','system','查看总线恢复结果','bus status'),
  command('bus-recovery-status','system','读取本次恢复诊断','bus recovery'),
  command('grab-ring-align-top','vision','在Z=0对环，不下降','grab align ring {ring} 0 stay',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']])],'回到Z=0对环并保持，用于和逐步下降对环作对照；不夹取、不松爪。'),
  command('bus-test','system','单电机通信测试','bus test {id} {rounds}',[
    choice('id','电机 ID',[['1','1 · 左前轮'],['2','2 · 左后轮'],['3','3 · 右后轮'],['4','4 · 右前轮'],['5','5 · Base'],['6','6 · Z'],['7','7 · X'],['8','8 · 车载转盘']]),
    number('rounds','测试轮数',10,1,100)],'每轮只读取所选 ID 的状态、位置和速度，暂停其他普通总线请求；不驱动电机、不解除运动锁定。共享线路上的电气干扰仍会影响测试。'),
  command('bus-test-status','system','查看单电机测试结果','bus test status'),
  command('bus-read','system','电机只读诊断','bus read {id} {field}',[
    choice('id','电机 ID',[['5','5 · Base'],['6','6 · Z'],['7','7 · X'],['8','8 · 车载转盘'],['1','1 · 左前轮'],['2','2 · 左后轮'],['3','3 · 右后轮'],['4','4 · 右前轮']]),
    choice('field','读取项目',[['current','实际相电流 / mA'],['voltage','供电电压 / mV'],['error','位置误差 / °'],['home','驱动回零参数'],['driver','驱动电流上限与到位窗口']])],
    '任务空闲时单次读取，不运动、不改参数。当前车使用Emm布局与65536反馈单位/圈；停止锁定期间可读。'),
  command('bus-test-stop','system','结束单电机测试','bus test stop',[],'结束通信测试；实际停车使用停车入口。'),
  command('help','system','固件指令帮助','help'),
  command('chassis-test','chassis','路线加速度 · 单段位移','chassis test {forward} {left} {speed} {profile}',[choice('direction','相对车头方向',[['forward','↑ 前进'],['back','↓ 后退'],['left','← 左移'],['right','→ 右移'],['fl','↖ 左前'],['fr','↗ 右前'],['bl','↙ 左后'],['br','↘ 右后']],'forward'),number('distance','合位移 / mm',500,1,3000,1),number('speed','目标合速度 / mm/s',1000,10,5000,1),choice('profile','底盘加减速',[['linear','线性 · 自定加速度'],['cosine','余弦 · 自定K值']],'linear'),number('accel','线性起步加速度 / mm/s²',500,10,5000,1),number('decel','线性停车减速度 / mm/s²',300,10,5000,1),number('cosine_k','余弦K值（过渡时间=K×2 ms）',200,10,2000,1),number('curve_period_ms','曲线更新周期 / ms',1,1,100,1),number('heading_kp','航向P / (rad/s)/°',0.1,0,1,0.001),number('heading_kd','航向D / (rad/s)/(°/s)',0,0,1,0.001)],'选择直线加减速方式，短距离自动降低峰值速度。保持起步朝向，斜移距离为合距离；运行到目标并确认停稳后结束。'),
  command('chassis-run','chassis','平移与转向测试','chassis run {vx} {vy} {omega} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-300,300,0.1),number('vy','左向速度 / mm·s⁻¹',0,-300,300,0.1),number('omega_deg','逆时针角速度 / °·s⁻¹',0,-500,500,0.1),number('seconds','保持时间 / s',1,0.1,59,0.1)],'角速度输入单位为 °/s，网页上限500；网页自动换算为固件rad/s。角速度为 0 时自动保持起步航向，需要 IMU 验证通过；非 0 时执行给定转向。起步和停车各 500 ms；总时长为保持时间加 1 秒。'),
  command('chassis-heading','chassis','航向保持测试','chassis heading {vx} {vy} {heading} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-300,300,0.1),number('vy','左向速度 / mm·s⁻¹',0,-300,300,0.1),number('heading','HWT101 模块目标角度 / °','',-180,180,0.1),number('seconds','保持时间 / s',1,0.1,59,0.1)],'需本次启动验证通过；目标角度不是地图角度。起步和停车各 500 ms。'),
  command('chassis-turn','chassis','原地转到地图航向','chassis turn {heading}',[number('heading','地图绝对航向 / °','',-180,180,0.1)],'地图从 +X 逆时针，范围 -180～180°；显式执行后原地转向，按新鲜航向反馈完成。与模块航向保持测试分别使用。'),
  command('chassis-origin','chassis','设置地图初始位姿','chassis origin {x} {y} {heading}',[number('x','地图 X / mm','',-10000,10000,0.1),number('y','地图 Y / mm','',-10000,10000,0.1),number('heading','地图航向 / °','',-180,180,0.1)],'需静止且 IMU 验证有效；只设地图位姿，不执行运动。'),
  command('chassis-task','chassis','读取运动任务','chassis task',[],'时间任务结束不表示到达地图位置或反馈确认停止。'),
  command('chassis-route-start','map','开始第 1 个目标','chassis route start {speed}',[number('speed','路线平移速度 / mm·s⁻¹',1000,10,5000,1)],'此处设置直行、后退和斜移的平移速度；旋转单独采用固件100°/s上限。15个目标沿用启动设置，修改输入不会改变正在执行的路线。'),
  command('chassis-route-auto','map','自动跑固定路线','chassis route auto {speed}',[number('speed','自动路线平移速度 / mm·s⁻¹',1000,10,5000,1)],'此处设置平移速度，旋转采用固件100°/s上限。确认停稳后等待0.1秒自动继续，共15个目标。仅行驶，不自动扫码、抓放；用取消路线或立即停止中止。'),
  ...[['next','手动下一段'],['cancel','取消路线'],['status','查询实车路线']].map(([action,label])=>command(`chassis-route-${action}`,'chassis',label,`chassis route ${action}`)),
  command('chassis-move','chassis','单段相对位移测试','chassis move {dx} {dy}',[number('dx','地图 ΔX / mm','',-300,300,0.1),number('dy','地图 ΔY / mm','',-300,300,0.1)],'地图坐标相对位移：+X 向右、+Y 向上；建议先测 100～300 mm，合位移须大于 0 且不超过 300 mm。MCU 以编码器反馈估计控制当前段，需确认停止后再操作。'),
  command('chassis-hold','chassis','旧版补偿直行测试','chassis hold {speed} {seconds}',[number('speed','前向速度 / mm·s⁻¹',0,-100,100),number('seconds','持续时间 / s',1,1,10)],'旧独立测试：保持启动时模块航向；参数为整数，不使用新版起停规划。'),
  command('chassis-status','chassis','读取旧版直行测试状态','chassis status'),
  command('feedback-read','chassis','读取四轮反馈缓存','chassis feedback',[],'只读缓存，不自动开启轮询；valid 顺序为速度、位置、状态。'),
  command('feedback-select','chassis','选择反馈对象','chassis feedback {wheel}',[choice('wheel','反馈对象',[['0','全部车轮'],['1','左前 · 地址 1'],['2','左后 · 地址 2'],['3','右后 · 地址 3'],['4','右前 · 地址 4']])],'选择对象后轮询保持关闭，需显式开启。'),
  command('feedback-on','chassis','开启反馈轮询','chassis feedback on'),
  command('feedback-off','chassis','关闭反馈轮询','chassis feedback off'),
  command('units','chassis','确认协议位置单位','chassis units {units}',[choice('units','每圈协议位置单位',[['0','尚未确认 · 0'],['65536','实测一圈为 65536 单位']],'')],'本车Emm反馈已确认每圈65536单位，固件默认采用；0仅用于诊断撤销，不是3200命令脉冲数。'),
  command('reset-confirmed','chassis','确认驱动已实际复位','chassis reset-confirmed',[],'仅在实物驱动器已复位后执行；此按钮不会复位驱动器。'),
  command('imu-status','system','读取 IMU 状态','imu status'),
  command('imu-cal-start','system','原生零偏标定','imu cal start',[],'整车静止约 20 秒标定 + 30 秒验证；验证通过后请求模块内部保存。'),
  command('imu-cal-quick','system','比赛快速标定','imu cal quick',[],'整车静止约 20 秒标定 + 10 秒验证；内部标定与保存流程不变，漂移判定标准不变。'),
  command('imu-verify','system','重新验证 5 秒','imu verify'),
  command('imu-cal-cancel','system','取消标定或验证','imu cal cancel',[],'尝试恢复正常模式并清除本次航向控制验证资格。'),
  command('imu-stream-on','system','开启模块角度输出','imu stream on'),
  command('imu-stream-off','system','关闭模块角度输出','imu stream off'),
  command('qr-test-units','qr','确认Emm反馈单位','chassis units 65536',[],'采用已确认的Emm驱动反馈单位；不发送运动。'),
  command('qr-test-origin','qr','设置右上起点','chassis origin 2250 2250 -90',[],'将车放在右上启停区中心、车头向下，完成IMU验证和四轮反馈准备后，显式设置地图原点。'),
  command('qr-test-start','qr','驶向二维码并扫码','grab qr begin {speed}',[number('speed','底盘速度 / mm·s⁻¹',50,10,200)],'右上起点到二维码，原地转180°并停稳后Z上下搜码；Base和X保持原姿态，扫到即停。'),
  command('qr-test-lower-origin','qr','设置右下起点','chassis origin 2250 150 180',[],'车放在右下启停区中心，车头朝左；完成IMU验证后显式设置起点。'),
  command('qr-test-lower-start','qr','驶向二维码并扫码','grab qr begin lower {speed}',[number('speed','底盘速度 / mm·s⁻¹',50,10,200)],'斜移至(2100,300)，顺时针转90°，直行到二维码，原地转180°并停稳后Z上下搜码；扫到即停。'),
  command('qr-test-status','qr','读取扫码测试状态','grab qr status'),
  command('qr-test-stop','qr','停止扫码测试','grab qr stop'),
  command('qr-status','qr','检查支持 / 接收统计','qr status'),
  command('qr-read','qr','读取最新任务码','qr read',[],'只读缓存，不主动触发摄像头扫码。'),
  command('qr-stream-on','qr','跟随新任务码','qr stream on'),
  command('qr-stream-off','qr','关闭任务码跟随','qr stream off'),
];

// Keep stable command groups/IDs; owner is the six-page UI module.
const debugIds = new Set(['position','grip-idle','grip-duty','move','velocity','wheel','wheel-stop','chassis-hold','chassis-status','feedback-read','feedback-select','feedback-on','feedback-off','camera-material','camera-status','camera-stop','imu-stream-on','imu-stream-off']);
const calibrationIds = new Set(['home','origin','zero','config-get','config-set','grab-calib-ref','grab-calib','grab-calib-status','units','reset-confirmed','chassis-origin','imu-cal-start','imu-cal-quick','imu-verify','imu-cal-cancel']);
for (const definition of commands) {
  definition.owner = definition.group === 'grip' ? 'arm' : definition.group;
  definition.section = calibrationIds.has(definition.id)||definition.id.startsWith('grab-set-') ? 'calibration' : debugIds.has(definition.id) ? 'debug' : 'common';
  definition.requiresInput = definition.fields.some(field => field.value === '');
}
commands.find(c => c.id === 'stop').note = 'stop all 会请求停止正在运行的视觉/物料任务；否则停止机械臂轴。它不等于整车所有电机已停止。';

/** Classifies without rewriting text; unknown commands stay manual. */
export function moduleForWire(text) {
  if (typeof text !== 'string') return 'manual';
  const line = text.trim().replace(/^(?:arm> )+/, '').replace(/^(?:OK|ERR)\s+/, '');
  if (/^@(CHASSIS|RADAR)\b/.test(line) || /^(?:(?:OK|ERR|EVT) )?radar\b|^chassis stream\b/.test(line)) return 'map';
  if (/^(?:qr\b|@QR\b|EVT qr\b)/.test(line)) return 'qr';
  if (/^(?:chassis\b|wheel(?:\s|=)|\[CHASSIS\])/.test(line)) return 'chassis';
  if (/^(?:camera|material|vision|grab|VISION)\b/.test(line)) return 'vision';
  if (/^(?:pos|enable|disable|state|stop|position|home|origin|zero|config|grip|turntable|lidarservo)\b/.test(line)) return 'arm';
  if (/^(?:bus|system|console|imu|info|help|HWT101)\b/.test(line) || /^(?:\[IMU\b|\[MOTOR BUS\]|ARM STM32|BUILD |Boot:|STM32 mechanical)/.test(line)) return 'system';
  return 'manual';
}

export function frameCommand(text) {
  if (typeof text !== 'string' || /[^\x20-\x7e\t]/.test(text)) throw new Error('仅允许单行 ASCII 命令，不能包含换行。');
  const line = text.trim();
  if (!line || line.length > 95) throw new Error('命令长度必须为 1～95 字节。');
  if (line.split(/\s+/).length > 12) throw new Error('固件最多接受 12 个字段。');
  return line + '\r\n';
}

export function buildCommand(id, values = {}) {
  if(['grab-ring-run','grab-ring-take','grab-ring-store'].includes(id))values={drop:-GRAB_FORM_DEFAULTS.z_proc_place,carDrop:GRAB_FORM_DEFAULTS.car_pick_drop_mm,...values};
  if(id==='grab-mission'){
    values={scale:1,smoothStart:false,...values};
    if(typeof values.smoothStart!=='boolean')throw new Error('请选择是否启用起步平滑处理。');
    if(values.smoothStart&&((values.start??'lower')!=='lower'||(values.profile??'linear')!=='linear'))throw new Error('起步平滑处理仅支持右下起点和linear模式。');
  }
  if(id==='chassis-test'){const legacyAccel=values.accel;values={profile:'linear',accel:500,cosine_k:200,curve_period_ms:1,heading_kp:0.1,heading_kd:0,...values};if(values.decel===undefined)values.decel=legacyAccel??300;if(values.profile==='cosine')values.accel=values.decel=300;else values.cosine_k=200;}
  // A high alignment uses absolute Z=0; the inactive lower-height draft is ignored.
  if(id==='grab-ring-align-options' && values.height==='raised')values={...values,z:'0'};
  const definition = commands.find(c => c.id === id);
  if (!definition) throw new Error('未知命令。');
  const checked = {};
  for (const field of definition.fields) {
    const raw = String(values[field.key] ?? '').trim();
    if (field.options) {
      if (!field.options.some(([v]) => v === raw)) throw new Error(`请选择${field.label}。`);
    } else {
      const value = Number(raw);
      const [min,max] = id === 'pos' && field.key === 'degree'
        ? positionDegreeRange(checked.axis) : [field.min,field.max];
      if (!raw || !/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(raw) || !Number.isFinite(value) ||
          value < min || (field.exclusiveMin && value === min) || value > max || (field.step === 1 && !Number.isInteger(value)))
        throw new Error(`${field.label}：请输入 ${min}～${max}${field.step === 1 ? ' 的整数' : ' 的数值'}。`);
    }
    checked[field.key] = raw;
  }
  if ((['pos','grab-base-pos'].includes(id) && Number(checked.degree) === 0) || (id === 'wheel' && Number(checked.rpm) === 0))
    throw new Error('运动值不能为 0，请使用对应的停止命令。');
  if (id === 'config-set' && !['x','z'].includes(checked.axis) && Number(checked.acc) === 0)
    throw new Error('只有单独 X/Z 轴允许加速度0档。');
  if (id === 'config-set' && checked.axis !== 'z' && Number(checked.limit) === 0)
    throw new Error('只有单独 Z 轴允许脉冲限制为 0。');
  if (id === 'chassis-test') {
    const [f,l]=({forward:[1,0],back:[-1,0],left:[0,1],right:[0,-1],fl:[1,1],fr:[1,-1],bl:[-1,1],br:[-1,-1]})[checked.direction];
    const component=Math.floor(Number(checked.distance)/Math.hypot(f,l)*1e6)/1e6;
    checked.forward=String(f*component);checked.left=String(l*component);
  }
  if (id === 'chassis-move') {
    const distance = Math.hypot(Number(checked.dx), Number(checked.dy));
    if (distance === 0 || distance > 300) throw new Error('合位移须大于 0 且不超过 300 mm。');
  }
  if (id === 'chassis-run' || id === 'chassis-heading') {
    if (Math.hypot(Number(checked.vx), Number(checked.vy)) > 300)
      throw new Error('平移合速度不能超过 300 mm/s。');
    checked.hold_ms = String(Math.round(Number(checked.seconds) * 1000));
  }
  if (id === 'chassis-run') checked.omega = String(Number((Number(checked.omega_deg) * Math.PI / 180).toFixed(6)));
  // Firmware expects integer text even when a number input contains 1500.0.
  if(id==='lidar-servo-pulse')checked.pulse=String(Number(checked.pulse));
  // Reuse the deployed 50 Hz gripper command: duty percent = pulse us / 200.
  if(id==='grip-pulse')checked.duty=String(Number(checked.pulse)/200);
  if(id==='grab-ring-align-options'){
    const line=`grab align ring ${checked.ring} ${checked.z}${checked.finish==='stay'?' stay':''}`;
    frameCommand(line);return line;
  }
  let line = definition.template.replace(/\{(\w+)\}/g, (_,key) => checked[key]);
  if(id==='grab-mission'){const settings=missionSpeedSettings(checked.speed,checked.scale);if(settings.scale!==1)line+=` scale=${settings.scale}`;if(values.smoothStart)line+=' smooth=1';}
  if(id==='chassis-test'){line+=` ${Number(checked.profile==='linear'?checked.accel:checked.cosine_k)} ${Number(checked.heading_kp)} ${Number(checked.heading_kd)}`;if(Number(checked.curve_period_ms)!==1||(checked.profile==='linear'&&Number(checked.decel)!==Number(checked.accel)))line+=` ${Number(checked.curve_period_ms)}`;if(checked.profile==='linear'&&Number(checked.decel)!==Number(checked.accel))line+=` ${Number(checked.decel)}`;}
  frameCommand(line);
  return line;
}

/** Commands for an explicit operator-triggered RAM preset; never sent on page load. */
export function buildGrabPresetCommands(preset=GRAB_TEST_PRESET) {
  if(!preset || typeof preset!=='object' || Array.isArray(preset)) throw new Error('调试预设必须是参数对象。');
  return Object.entries(preset).map(([key,value])=>buildCommand(`grab-set-${key}`,{value}));
}

export class LineDecoder {
  pending = '';
  push(text) {
    const parts = (this.pending + text).split(/[\r\n]/);
    this.pending = parts.pop() ?? '';
    // The firmware's shell prompt has no newline. Don't let it prefix the next reply.
    this.pending = this.pending.replace(/^(?:arm> )+/, '');
    const lines = parts.map(line => line.replace(/^(?:arm> )+/, '')).filter(Boolean);
    if (this.pending.length > 4096) {
      lines.push(this.pending.slice(0,4096) + ' …[长行截断]');
      this.pending = '';
    }
    return lines;
  }
}

export function describeReply(line) {
  if (replyIsFault(line)) return '设备故障或请求被拒绝';
  if (/^OK bus recover complete/.test(line)) return '总线恢复完成 · 不自动续跑，请检查回零参考';
  if (/^OK grab\b.*\bstate=hold\b/.test(line) && /\boperation=ring_place\b/.test(line) && /\bresult=released\b/.test(line)) return '色环放置完成 · 已松爪提起';
  if (/^OK grab\b.*\bstate=hold\b/.test(line) && /\breason=ring_aligned_low\b/.test(line)) return '色环对准完成 · 保持对准高度观察';
  if (/^OK grab\b.*\bstate=hold\b/.test(line)) return /\bmode=align\b/.test(line) ? '对准动作完成 · 未执行抓取' : '取放动作完成 · 等待人工验收';
  if (/\baccepted\b/.test(line)) return '请求已接受 · 尚未确认到位';
  if (/^OK state \S+ .*\breached=1\b/.test(line)) return '所查询轴报告已到位';
  if (/^OK state \S+ .*\breached=0\b/.test(line)) return '所查询轴报告未到位';
  if (line.startsWith('OK')) return '收到设备回复';
  return '收到设备数据';
}

/** Parse one firmware snapshot. ACKs and missing fields cannot become feedback. */
export function parseGrabStatus(line) {
  if (typeof line !== 'string' || !/^(?:OK|ERR) grab state=/.test(line)) return null;
  const result={};
  for (const token of line.split(/\s+/).slice(2)) {
    const match=token.match(/^([a-z_]+)=([^\s=]+)$/);
    if (!match || Object.hasOwn(result,match[1])) return null;
    result[match[1]]=match[2];
  }
  if (!/^[a-z_]+$/.test(result.state??'')) return null;
  for(const key of ['bus_recoveries','auto_resume'])if(result[key]!==undefined){
    if(!/^\d+$/.test(result[key]) || !Number.isSafeInteger(Number(result[key])))return null;
    result[key]=Number(result[key]);
    if(key==='auto_resume' && result[key]>1)return null;
  }
  for (const key of ['x','xt','z','zt','b','bt','dx','dy','age','vf','vl','elapsed','rx_seq','color','slot','recovery_used','recovery_count','recovery_left_ms','loss_ms','loss_max_ms','grace_count','stop_requested','stop_confirmed','base_home_degraded']) {
    const raw=result[key];
    if (raw===undefined || /^(?:nan|na)$/i.test(raw)) {result[key]=null;continue;}
    if (!/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(raw) || !Number.isFinite(Number(raw))) return null;
    result[key]=Number(raw);
    if (['stop_requested','stop_confirmed','base_home_degraded'].includes(key) && ![0,1].includes(result[key])) return null;
    if (Object.hasOwn({recovery_used:4294967295,recovery_count:3,recovery_left_ms:120000,grace_count:3},key) && (!Number.isInteger(result[key]) || result[key]<0 || result[key]>{recovery_used:4294967295,recovery_count:3,recovery_left_ms:120000,grace_count:3}[key])) return null;
    if (['rx_seq','loss_ms','loss_max_ms'].includes(key) && (!Number.isInteger(result[key]) || result[key]<0 || result[key]>0xffffffff)) return null;
    if (['age','elapsed'].includes(key) && result[key]<0) return null;
  }
  return result;
}

/** Read-only sampler diagnostics have their own shape and never become task feedback. */
export function parseGrabCalibrationStatus(line) {
  if(typeof line!=='string')return null;
  line=line.trim().replace(/^(?:arm>\s*)+/,'');
  if(!/^OK grab calib phase=/.test(line))return null;
  const result={};
  for(const token of line.split(/\s+/).slice(3)) {
    const match=token.match(/^([a-z_]+)=([^\s=]+)$/);
    if(!match||Object.hasOwn(result,match[1]))return null;
    result[match[1]]=match[2];
  }
  if(!/^[a-z_]+$/.test(result.phase??'')||!result.reason)return null;
  for(const key of ['ref','calibrated']) {
    if(!/^[01]$/.test(result[key]??''))return null;
    result[key]=Number(result[key]);
  }
  for(const key of ['fdx','fdy','xdx','xdy']) {
    const raw=result[key];
    if(/^(?:na|nan)$/i.test(raw??'')){result[key]=null;continue;}
    if(!/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(raw??'')||!Number.isFinite(Number(raw)))return null;
    result[key]=Number(raw);
  }
  return result;
}

/** Configuration diagnostics can list all missing keys in a single reply. */
export function describeGrabMissing(value) {
  if(!value)return '—';
  if(value==='none')return '无';
  return value.split(',').map(key=>{
    const parameter=grabParameters.find(p=>p.key===key);
    return parameter?`${parameter.label}（${key}）`:deviceText(key);
  }).join('；');
}

/** Route and arm task states are independent; an idle arm cannot clear route errors. */
export function parseGrabRouteStatus(line) {
  if (typeof line !== 'string' || !/^(?:OK|ERR) grab route state=/.test(line)) return null;
  const result={};
  for (const token of line.split(/\s+/).slice(3)) {
    const match=token.match(/^([a-z_]+)=([^\s=]+)$/);
    if (!match || Object.hasOwn(result,match[1])) return null;
    result[match[1]]=match[2];
  }
  if (!/^[a-z_]+$/.test(result.state??'')) return null;
  for(const key of ['start_smooth','entry_smooth']){
    if(result[key]===undefined)continue;
    if(!/^[01]$/.test(result[key]))return null;
    result[key]=Number(result[key]);
  }
  for(const key of ['route_base_speed','route_scale','route_speed']){
    if(result[key]===undefined)continue;
    if(!/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(result[key])||!Number.isFinite(Number(result[key])))return null;
    result[key]=Number(result[key]);
  }
  for (const key of ['segment','elapsed_ms','batch','item','station','visit','stored','placed','slot','ring','color','stop_confirmed']) {
    if (result[key]===undefined) {result[key]=null;continue;}
    if (!/^\d+$/.test(result[key]) || !Number.isSafeInteger(Number(result[key]))) return null;
    result[key]=Number(result[key]);
  }
  return result;
}

export function describeGrabRoute(r) {
  if(r?.mode==='heading')return `独立航向测试 · ${reasonText(r.reason||r.heading_result||r.state)}`;
  if(r?.mode==='single'){const stage=({stop_wait:'停车未确认 · 等待恢复',qr_prepare:'准备实际扫码',qr_scan:'等待本轮扫码',qr_display:'发送实际码到串口屏',qr_departure:'扫码后收回',car_pick:'车仓取出并保持',held_place:'持料视觉对准',stopping:'正在停止',paused:'已暂停',resuming:'恢复当前阶段',preparing_travel:'准备后向姿态',driving:'行驶中',prepare_transfer:'准备抓取',transfer:'抓取入仓',prepare_departure:'准备收回',leaving:'收回朝后',config_wait:`等待补齐 ${describeGrabMissing(r.reason)} 后继续本件`,done:'入仓及收回完成',error:'任务故障',cancelled:'已取消'})[r.state]||r.state;return `蓝色首件 · 第一仓 · ${stage}`;}
  if(!['mission','ring_place'].includes(r?.mode))return `组合路线：${({idle:'空闲',driving:'行驶中',scan_wait:'扫码等待 1 秒',display_wait:'等待屏幕发送',handoff:'物料台已交接',error:'路线故障',cancelled:'已取消',done:'首件任务完成'})[r?.state]||r?.state||'未报告'}`;
  const stage=({cancelling:'正在结束本轮',cancel_wait:'结束中 · 停车尚未确认',stop_wait:'停车未确认 · 等待恢复',qr_prepare:'准备实际扫码',qr_scan:'等待本轮扫码',qr_display:'发送实际码到串口屏',qr_departure:'扫码后收回',empty_ring_align:'空爪对准目标编号环',empty_material_align:'空爪对准第一层同色物料',car_pick:'车仓取出并保持',held_place:'放置所取物料',stopping:'正在停止',paused:'已暂停',resuming:'恢复当前阶段',preparing_travel:'准备后向行驶姿态',driving:'行驶中',departure_heading_prepare:'准备工位离站航向矫正',departure_heading:'工位离站前色环航向矫正',ring_clear:'抬升收回',ring_move:'移动到指定环',transfer:'单件取放',leaving:'恢复后向姿态',return_center:'返回工位中心',config_wait:'等待补齐参数',done:'两批任务结束',error:'任务故障',cancelled:'已取消',prepare_ring:'准备换环',prepare_departure:'准备离站',prepare_transfer:'准备下一件取放'})[r.state]||r.state;
  const recoveryStage=missionResumeStatus(r)?.label||({bus_recovery_stopping:'通信异常 · 保留进度停车',bus_recovering:'恢复电机总线',bus_recovery_verify:'恢复后确认停稳',bus_recovery_wait:'总线恢复待处理',bus_recovery_retry:'通信恢复重试'})[r.state];
  if(r?.mode==='ring_store')return `编号环取回入仓 · ${r.slot??'—'}号料盘 · ${r.state==='done'?'放料、抬Z及机械臂回位完成':recoveryStage||stage}`;
  if(r?.mode==='ring_place')return `编号环自动放置 · ${r.ring??'—'}号环 · ${r.slot??'—'}号料盘 · ${r.state==='done'?'取仓放置完成':recoveryStage||stage}`;
  return `${r.batch===2?'第二批':'第一批'} · 第 ${r.item??'—'} 件 · ${recoveryStage||stage}${r.state==='config_wait'?`：补齐 ${describeGrabMissing(r.reason)} 后继续本件`:''}`;
}

/** Compact display only; callers retain the original line for parsing and export. */
export function formatTerminalLine(line) { return chineseTerminalLine(line); }


/** Current pixel error only; absent provenance never implies valid coordinates. */
export function grabPixelText(snapshot) {
  return snapshot?.vision==='Ok' && Number.isFinite(snapshot.dx) && Number.isFinite(snapshot.dy)
    ? `${snapshot.dx} / ${snapshot.dy} px` : '—';
}

/** Display only MCU-reported recovery progress; never drive control or a local timer. */
export function grabRecoveryText(s) {
  if(!s || s.recovery_used===null || s.recovery_used===undefined)return '未报告';
  const used=`${s.recovery_used}次`;
  if(s.reason==='vision_recover_timeout')return `恢复窗口超时；已恢复 ${used}，任务停止`;
  if(s.reason==='vision_recover_limit')return `恢复次数耗尽（${used}）；任务停止`;
  if(s.state==='vision_grace')return `短暂漏检，沿原运动趋势减速；中断 ${s.loss_ms??'未报告'} ms；恢复观测 ${s.grace_count??'未报告'}/3；不会下降夹取`;
  if(s.reason==='vision_gap_recovered')return `短暂漏检已恢复，未消耗停车恢复次数；中断 ${s.loss_ms??'未报告'} ms；重新检查对准`;
  if(s.state==='vision_pause') {
    if(s.recovery_left_ms===null || s.recovery_left_ms===undefined || s.stop_confirmed!==1)return `漏检，正在停车；已恢复 ${used}；不会下降夹取`;
    const waiting=s.reason==='vision_reacquire_wait'?'已重发目标颜色识别，等待新坐标':'等待目标恢复';
    return `${waiting}（${s.recovery_count??'未报告'}/3）；设备报告剩余 ${s.recovery_left_ms} ms；已恢复 ${used}；不会下降夹取`;
  }
  if(s.reason==='vision_recovered')return `已恢复对准（${used}）；重新检查对准与停稳`;
  if(s.reason==='cancelled')return '任务已取消，待恢复资格已清除';
  return `本次任务已恢复 ${used}`;
}
