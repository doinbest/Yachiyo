import {chineseTerminalLine,replyIsFault} from './terminal-format.mjs?v=grab-terminal-20261002';
import {GRAB_FORM_DEFAULTS,GRAB_TEST_PRESET} from './grab-params.mjs?v=car-transfer-speed-20261007';
import {GRAB_PARAMETER_METADATA} from './grab-parameter-metadata.mjs';
export {replyIsFault};
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
  ...[['close_ms','夹爪闭合 / 松开等待 / ms'],['stable_ms','精定位稳定时间 / ms']].map(([key,label])=>({key,label,group:'反馈与抓取条件'})),
  ...[['feedback_ms','反馈最大年龄 / ms'],['age_ms','有效坐标接收超时 / ms']].map(([key,label])=>({key,label,group:'反馈与抓取条件'})),
  {key:'vision_wait_ms',label:'目标颜色等待窗口 / ms',group:'反馈与抓取条件'},
  {key:'loss_grace_ms',label:'视觉对准短暂漏检容忍 / ms（0 关闭）',group:'反馈与抓取条件'},
  {key:'frames',label:'稳定坐标观测次数',group:'反馈与抓取条件'},
  {key:'retries',label:'重新对准次数上限',group:'反馈与抓取条件'},
  {key:'ref_u',label:'观察姿态准心 U / px',group:'准心与像素响应'},
  {key:'ref_v',label:'观察姿态准心 V / px',group:'准心与像素响应'},
  ...[['j00','DX / 底盘前进'],['j01','DX / 底盘左移'],['j02','DX / X 伸出'],['j10','DY / 底盘前进'],['j11','DY / 底盘左移'],['j12','DY / X 伸出']].map(([key,label])=>({key,label:`${label} / px·mm⁻¹`,group:'准心与像素响应'})),
  ...[['body_speed','底盘速度上限'],['x_speed','X 视觉对准速度上限'],['x_transfer_speed','X 正式取放 / 入仓测试速度'],['z_speed','Z 速度上限']].map(([key,label])=>({key,label:`${label} / mm·s⁻¹`,group:'控制与运动边界'})),
  {key:'z_pick_speed',label:'物料夹取下降速度 / mm·s⁻¹',group:'控制与运动边界'},
  {key:'accel',label:'平移加速度上限 / mm·s⁻²',group:'控制与运动边界'},
  {key:'travel_mm',label:'局部底盘行程上限 / mm（0 关闭）',group:'控制与运动边界'},
  {key:'lead_mm',label:'X 目标领先反馈上限 / mm',group:'控制与运动边界'},
  {key:'lambda',label:'视觉收敛增益 / s⁻¹',group:'控制与运动边界'},
  {key:'x_weight',label:'X 分配权重',group:'控制与运动边界'},
  ...['z_proc_grab','z_proc_lift','z_car_lift','z_proc_place','z_temp_place','z_stack_place','x_car','base_car_offset'].map(key=>({key,label:({z_proc_grab:'粗加工夹取Z',z_proc_lift:'粗加工提起Z',z_car_lift:'车载取出提起Z',z_proc_place:'粗加工释放Z',z_temp_place:'暂存第一层释放Z',z_stack_place:'暂存第二层释放Z',x_car:'车载取放口X',base_car_offset:'车载Base偏转角'})[key],group:'取放场景'})),
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
  command('grab-ring-align','vision','底盘 + X 对准所选编号环','grab align ring {ring} {z}',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']]),number('z','对准时Z绝对高度 / mm',-100,-150,0,1)],'Z最高处识别编号，先底盘和X横向接近，再随环心接近准心逐步下降；靠画面边缘时暂缓下降。完成后停止视觉控制、Z抬至最高，再手动装料。DX/DY各≤1像素。'),
  command('grab-ring-place','vision','下降放置并提起','grab ring place {z}',[number('z','放置Z绝对位置 / mm（可调整）',-150,-150,-0.1,0.1)],'先完成所选环对准，人工放入物料并闭合夹爪。仅Z下降、反馈到位后松爪，再提起至观察高度；不取仓、不改变库存。'),
  command('grab-ring-align-stay','vision','对准后保持低位','grab align ring {ring} {z} stay',[choice('ring','识别环号',[['1','1号环'],['2','2号环'],['3','3号环']]),number('z','对准时Z绝对高度 / mm',-100,-150,0,1)],'按所设高度渐进下降，对准完成后保持低位，仅继续识别编号环供观察。'),
  command('grab-stack-align','vision','颜色中心对准并补偿','grab stack align {color}',[choice('color','下层物料颜色',grabColorOptions,'3')],'Z保持最高，先对准绿色准心，再执行码垛专用补偿（画面向右39、向下1像素），停稳后等待装料；不修改物料准心。'),
  command('grab-stack-lower','vision','单独调整码垛高度','grab stack lower {z}',[number('z','Z绝对目标 / mm',-100,-150,0,0.1)],'只调整Z，保持夹持；反馈到位后保持高度。'),
  command('grab-stack-release','vision','单独松爪','grab stack release',[],'高度到位后松爪，保持当前Z；不自动提起。'),
  command('grab-stack-lift','vision','提起到最高','grab stack lift',[],'仅Z提起；不重新对准或操作转盘。'),
  command('grab-place-car','vision','机械臂到车载口放下','grab place car',[],'以已验证的后向回零位置为基准转到车载口，X伸到x_car，Z下降到z_place后自动松爪，并停留在放料姿态。此动作不分度ID8、不修改库存。'),
  command('grab-fixed-slot','vision','指定仓位固定夹取','grab start fixed {slot} {color}',[choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('color','物料颜色',grabColorOptions,'3')]),
  command('grab-pick-slot','vision','指定仓位视觉夹取','grab start pick {color} {slot}',[choice('color','物料颜色',grabColorOptions,'3'),choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']])]),
  command('grab-store-proc','vision','粗加工取回入仓','grab store proc {slot} {color}',[choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('color','物料颜色',grabColorOptions,'3')]),
  command('grab-take','vision','车载取出并释放','grab take {slot} {scene}',[choice('slot','取出仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('scene','释放场景',[['rough','粗加工'],['temp1','暂存第一层'],['temp2','暂存第二层']])]),
  command('grab-return','vision','返回外侧观察姿态','grab return'),
  command('grab-base-pos','vision','手动 Base 转到转盘','pos base {degree}',[number('degree','Base 电机轴相对角度 / °',-180,-360,360,0.1)],'自动取放已包含转向，此处仅用于手动定位。默认相对转动 −180°，每次执行都会再转一次，已在转盘方向时无需重复。若抓取处于 hold，先结束本次抓取并确认停止，再操作 Base。'),
  ...[['fixed','固定位置抓取'],['align','仅协同对准'],['pick','单件视觉抓取']].map(([mode,label])=>command(`grab-${mode}`,'vision',label,mode==='align'?'grab align color 3 0':`grab start ${mode}`)),
  ...[['align','仅协同对准'],['pick','单件视觉抓取']].map(([mode,label])=>command(`grab-${mode}-color`,'vision',label,mode==='align'?'grab align color {color} 0':`grab start ${mode} {color}`,[choice('color','物料颜色',grabColorOptions,'3')])),
  command('grab-route','vision','启动蓝色首件入仓测试','grab route {speed}',[number('speed','路线平移速度 / mm·s⁻¹',200,10,5000)],'右下起点、机械臂朝后；二维码点直接通过，到原料台抓蓝色入第一仓，再收回朝后结束。不覆盖任务码；第一仓需有参考、已配置且为空，先在地图页准备跑图。'),
  command('grab-mission','vision','启动后向机械臂两批任务','grab mission {speed}',[number('speed','路线平移速度 / mm·s⁻¹',200,10,5000)],'右下起点，朝后回零；二维码点先通过。按当前任务码三件入仓、加工取回、暂存与同色码垛。无任务码时用明确模拟码156+123+516+231。'),
  command('grab-mission-continue','vision','补齐参数后继续本件','grab mission continue'),
  command('grab-pose-rear','vision','收回并朝后','grab pose rear'),
  command('grab-pose-work','vision','外侧作业姿态','grab pose work'),
  command('grab-base-status','vision','读取Base到位诊断','grab status base'),
  command('grab-status','vision','刷新抓取状态','grab status'),
  command('grab-stop','vision','停止单件抓取','grab stop',[],'停止自动推进并请求机构停止；保留夹持和 Z 承载。停止确认以新鲜反馈为准。'),
  command('grab-rehome','vision','重新建立三轴参考','grab rehome',[],'先移走夹持物并确认机械臂周围无障碍；按 Z、X、Base 顺序重新回零并读回验证。各轴确认后恢复对应默认参考，三轴完成后才能开始新任务。'),
  ...grabParameters.map(p=>command(`grab-get-${p.key}`,'vision',`读取 ${p.label}`,`grab get ${p.key}`)),
  ...grabParameters.map(p=>command(`grab-set-${p.key}`,'vision',p.label,`grab set ${p.key} {value}`,[{...number('value',p.label,grabDefaults[p.key]??'',p.min,p.max,p.step??'any'),exclusiveMin:p.exclusiveMin}],p.key in grabDefaults?'预填程序默认值，不是设备读回值；X/Z尺度和位置在上电或 grab rehome 对应轴验证成功后载入，容差和等待时间在启动时载入。单轴手动回零只受理动作，不自动恢复参考参数。修改仅存MCU RAM，运行中拒绝修改。':'仅设置 MCU RAM；复位后需重新配置。运行中拒绝修改，输入值不代表设备已接受。')),
  command('pos','arm','相对转动','pos {axis} {degree}',[axis(),number('degree','电机轴相对角度 / °',10,-360,360,0.1)],'正负值表示方向；这是相对运动，不是绝对位置。本面板单次输入限 ±360°，固件另有脉冲限制。'),
  ...[['enable','使能'],['disable','取消使能'],['state','查询状态'],['stop','停止']].map(([id,label]) => command(id,'arm',label,`${id} {axis}`,[axis(true)])),
  command('position','arm','读取当前位置','position {axis}',[axis()],'返回值为脉冲数。'),
  command('home','arm','执行回零','home {axis} {mode}',[axis(true),choice('mode','回零方式',[['near','就近 · near'],['dir','指定方向 · dir'],['collision','碰撞 · collision'],['limit','限位 · limit']])],'回零会产生运动；方式需要与实际机构匹配。'),
  command('origin','arm','保存机械零点','origin {axis}',[axis(true)],'将当前机械位置保存为零点，与计数清零不同。'),
  command('zero','arm','当前位置计数清零','zero {axis}',[axis(true)],'只清除当前位置计数，不执行回零运动。'),
  command('config-get','arm','读取运动配置','config {axis}',[axis(true)]),
  command('config-set','arm','设置运动配置','config {axis} {rpm} {acc} {limit}',[axis(true),number('rpm','速度 / RPM',30,1,3000),number('acc','加速度参数',20,1,255),number('limit','脉冲限制',3200,0,3200)],'配置只保存在 RAM，复位后恢复默认；只有单独 Z 轴允许 limit=0。'),
  ...[['open','打开 · 2500 µs'],['catch','夹取 · 1920 µs'],['idle','待机松开 · 2500 µs']].map(([id,label]) => command(`grip-${id}`,'grip',label,`grip ${id}`)),
  command('grip-duty','grip','手动设置占空比','grip duty {duty}',[number('duty','PWM 占空比 / %',9.6,2.5,12.5,0.1)],'50 Hz PWM；打开2500 µs = 12.5%，夹取1920 µs = 9.6%。'),
  command('move','chassis','定距前后移动','chassis {direction} {distance}',[choice('direction','移动方向',[['forward','前进'],['backward','后退']]),number('distance','距离 / mm',10,1,200)],'始终带距离发送，避免误触发持续后退。'),
  command('velocity','chassis','持续速度控制','chassis velocity {forward} {left} {yaw}',[number('forward','前向速度 / mm·s⁻¹',0,-1000,1000),number('left','左向速度 / mm·s⁻¹',0,-1000,1000),number('yaw','偏航速度 / °·s⁻¹',0,-360,360)],'持续运动需主动点击「底盘停止」。关闭网页或断开串口不等于停车。'),
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
  command('help','system','固件指令帮助','help'),
  command('chassis-run','chassis','平移与转向测试','chassis run {vx} {vy} {omega} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-300,300,0.1),number('vy','左向速度 / mm·s⁻¹',0,-300,300,0.1),number('omega_deg','逆时针角速度 / °·s⁻¹',0,-100,100,0.1),number('seconds','保持时间 / s',1,0.1,59,0.1)],'角速度输入单位为 °/s，上限100；网页自动换算为固件rad/s。角速度为 0 时自动保持起步航向，需要 IMU 验证通过；非 0 时执行给定转向。起步和停车各 500 ms；总时长为保持时间加 1 秒。'),
  command('chassis-heading','chassis','航向保持测试','chassis heading {vx} {vy} {heading} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-300,300,0.1),number('vy','左向速度 / mm·s⁻¹',0,-300,300,0.1),number('heading','HWT101 模块目标角度 / °','',-180,180,0.1),number('seconds','保持时间 / s',1,0.1,59,0.1)],'需本次启动验证通过；目标角度不是地图角度。起步和停车各 500 ms。'),
  command('chassis-turn','chassis','原地转到地图航向','chassis turn {heading}',[number('heading','地图绝对航向 / °','',-180,180,0.1)],'地图从 +X 逆时针，范围 -180～180°；显式执行后原地转向，按新鲜航向反馈完成。与模块航向保持测试分别使用。'),
  command('chassis-origin','chassis','设置地图初始位姿','chassis origin {x} {y} {heading}',[number('x','地图 X / mm','',-10000,10000,0.1),number('y','地图 Y / mm','',-10000,10000,0.1),number('heading','地图航向 / °','',-180,180,0.1)],'需静止且 IMU 验证有效；只设地图位姿，不执行运动。'),
  command('chassis-task','chassis','读取运动任务','chassis task',[],'时间任务结束不表示到达地图位置或反馈确认停止。'),
  command('chassis-route-start','map','开始第 1 个目标','chassis route start {speed}',[number('speed','路线平移速度 / mm·s⁻¹',200,10,5000,1)],'此处设置直行、后退和斜移的平移速度；旋转单独采用固件100°/s上限。15个目标沿用启动设置，修改输入不会改变正在执行的路线。'),
  command('chassis-route-auto','map','自动跑固定路线','chassis route auto {speed}',[number('speed','自动路线平移速度 / mm·s⁻¹',200,10,5000,1)],'此处设置平移速度，旋转采用固件100°/s上限。确认停稳后等待0.1秒自动继续，共15个目标。仅行驶，不自动扫码、抓放；用取消路线或立即停止中止。'),
  ...[['next','手动下一段'],['cancel','取消路线'],['status','查询实车路线']].map(([action,label])=>command(`chassis-route-${action}`,'chassis',label,`chassis route ${action}`)),
  command('chassis-move','chassis','单段相对位移测试','chassis move {dx} {dy}',[number('dx','地图 ΔX / mm','',-300,300,0.1),number('dy','地图 ΔY / mm','',-300,300,0.1)],'地图坐标相对位移：+X 向右、+Y 向上；建议先测 100～300 mm，合位移须大于 0 且不超过 300 mm。MCU 以编码器反馈估计控制当前段，需确认停止后再操作。'),
  command('chassis-hold','chassis','旧版补偿直行测试','chassis hold {speed} {seconds}',[number('speed','前向速度 / mm·s⁻¹',0,-100,100),number('seconds','持续时间 / s',1,1,10)],'旧独立测试：保持启动时模块航向；参数为整数，不使用新版起停规划。'),
  command('chassis-status','chassis','读取旧版直行测试状态','chassis status'),
  command('feedback-read','chassis','读取四轮反馈缓存','chassis feedback',[],'只读缓存，不自动开启轮询；valid 顺序为速度、位置、状态。'),
  command('feedback-select','chassis','选择反馈对象','chassis feedback {wheel}',[choice('wheel','反馈对象',[['0','全部车轮'],['1','左前 · 地址 1'],['2','左后 · 地址 2'],['3','右后 · 地址 3'],['4','右前 · 地址 4']])],'选择对象后轮询保持关闭，需显式开启。'),
  command('feedback-on','chassis','开启反馈轮询','chassis feedback on'),
  command('feedback-off','chassis','关闭反馈轮询','chassis feedback off'),
  command('units','chassis','确认协议位置单位','chassis units {units}',[choice('units','每圈协议位置单位',[['0','尚未确认 · 0'],['65536','实测一圈为 65536 单位']],'')],'先测量一圈反馈；不是命令脉冲数或物理编码器分辨率。'),
  command('reset-confirmed','chassis','确认驱动已实际复位','chassis reset-confirmed',[],'仅在实物驱动器已复位后执行；此按钮不会复位驱动器。'),
  command('imu-status','system','读取 IMU 状态','imu status'),
  command('imu-cal-start','system','原生零偏标定','imu cal start',[],'整车静止约 20 秒标定 + 30 秒验证；验证通过后请求模块内部保存。'),
  command('imu-verify','system','重新验证 5 秒','imu verify'),
  command('imu-cal-cancel','system','取消标定或验证','imu cal cancel',[],'尝试恢复正常模式并清除本次航向控制验证资格。'),
  command('imu-stream-on','system','开启模块角度输出','imu stream on'),
  command('imu-stream-off','system','关闭模块角度输出','imu stream off'),
  command('qr-status','qr','检查支持 / 接收统计','qr status'),
  command('qr-read','qr','读取最新任务码','qr read',[],'只读缓存，不主动触发摄像头扫码。'),
  command('qr-stream-on','qr','跟随新任务码','qr stream on'),
  command('qr-stream-off','qr','关闭任务码跟随','qr stream off'),
];

// Keep stable command groups/IDs; owner is the six-page UI module.
const debugIds = new Set(['position','grip-idle','grip-duty','move','velocity','wheel','wheel-stop','chassis-hold','chassis-status','feedback-read','feedback-select','feedback-on','feedback-off','camera-material','camera-status','camera-stop','imu-stream-on','imu-stream-off']);
const calibrationIds = new Set(['home','origin','zero','config-get','config-set','grab-calib-ref','grab-calib','grab-calib-status','units','reset-confirmed','chassis-origin','imu-cal-start','imu-verify','imu-cal-cancel']);
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
  if (/^(?:pos|enable|disable|state|stop|position|home|origin|zero|config|grip|turntable)\b/.test(line)) return 'arm';
  if (/^(?:bus|system|console|imu|info|help|HWT101)\b/.test(line) || /^(?:\[IMU\b|\[MOTOR BUS\]|ARM STM32|BUILD |Boot:|STM32 mechanical)/.test(line)) return 'system';
  return 'manual';
}

export function frameCommand(text) {
  if (typeof text !== 'string' || /[^\x20-\x7e\t]/.test(text)) throw new Error('仅允许单行 ASCII 命令，不能包含换行。');
  const line = text.trim();
  if (!line || line.length > 63) throw new Error('命令长度必须为 1～63 字节。');
  if (line.split(/\s+/).length > 6) throw new Error('固件最多接受 6 个字段。');
  return line + '\r\n';
}

export function buildCommand(id, values = {}) {
  const definition = commands.find(c => c.id === id);
  if (!definition) throw new Error('未知命令。');
  const checked = {};
  for (const field of definition.fields) {
    const raw = String(values[field.key] ?? '').trim();
    if (field.options) {
      if (!field.options.some(([v]) => v === raw)) throw new Error(`请选择${field.label}。`);
    } else {
      const value = Number(raw);
      if (!raw || !/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(raw) || !Number.isFinite(value) ||
          value < field.min || (field.exclusiveMin && value === field.min) || value > field.max || (field.step === 1 && !Number.isInteger(value)))
        throw new Error(`${field.label}：请输入 ${field.min}～${field.max}${field.step === 1 ? ' 的整数' : ' 的数值'}。`);
    }
    checked[field.key] = raw;
  }
  if ((['pos','grab-base-pos'].includes(id) && Number(checked.degree) === 0) || (id === 'wheel' && Number(checked.rpm) === 0))
    throw new Error('运动值不能为 0，请使用对应的停止命令。');
  if (id === 'config-set' && checked.axis !== 'z' && Number(checked.limit) === 0)
    throw new Error('只有单独 Z 轴允许脉冲限制为 0。');
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
  const line = definition.template.replace(/\{(\w+)\}/g, (_,key) => checked[key]);
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
  if (/^OK grab\b.*\bstate=hold\b/.test(line) && /\breason=ring_aligned_low\b/.test(line)) return '色环对准完成 · 保持低位观察';
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
  for (const key of ['x','xt','z','zt','b','bt','dx','dy','age','vf','vl','elapsed','rx_seq','color','slot','recovery_used','recovery_count','recovery_left_ms','loss_ms','loss_max_ms','grace_count','stop_requested','stop_confirmed']) {
    const raw=result[key];
    if (raw===undefined || /^(?:nan|na)$/i.test(raw)) {result[key]=null;continue;}
    if (!/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(raw) || !Number.isFinite(Number(raw))) return null;
    result[key]=Number(raw);
    if (['stop_requested','stop_confirmed'].includes(key) && ![0,1].includes(result[key])) return null;
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
    return parameter?`${parameter.label}（${key}）`:key;
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
  for (const key of ['segment','elapsed_ms','batch','item','station','visit','stored','placed']) {
    if (result[key]===undefined) {result[key]=null;continue;}
    if (!/^\d+$/.test(result[key]) || !Number.isSafeInteger(Number(result[key]))) return null;
    result[key]=Number(result[key]);
  }
  return result;
}

export function describeGrabRoute(r) {
  if(r?.mode==='single'){const stage=({preparing_travel:'准备后向姿态',driving:'行驶中',prepare_transfer:'准备抓取',transfer:'抓取入仓',prepare_departure:'准备收回',leaving:'收回朝后',config_wait:`等待补齐 ${describeGrabMissing(r.reason)} 后继续本件`,done:'入仓及收回完成',error:'任务故障',cancelled:'已取消'})[r.state]||r.state;return `蓝色首件 · 第一仓 · ${stage}`;}
  if(r?.mode!=='mission')return `组合路线：${({idle:'空闲',driving:'行驶中',scan_wait:'扫码等待 1 秒',display_wait:'等待屏幕发送',handoff:'物料台已交接',error:'路线故障',cancelled:'已取消',done:'首件任务完成'})[r?.state]||r?.state||'未报告'}`;
  const stage=({preparing_travel:'准备后向行驶姿态',driving:'行驶中',ring_clear:'抬升收回',ring_move:'移动到指定环',transfer:'单件取放',leaving:'恢复后向姿态',return_center:'返回工位中心',config_wait:'等待补齐参数',done:'两批任务结束',error:'任务故障',cancelled:'已取消',prepare_ring:'准备换环',prepare_departure:'准备离站',prepare_transfer:'准备下一件取放'})[r.state]||r.state;
  return `${r.batch===2?'第二批':'第一批'} · 第 ${r.item??'—'} 件 · ${stage}${r.state==='config_wait'?`：补齐 ${describeGrabMissing(r.reason)} 后继续本件`:''}`;
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
