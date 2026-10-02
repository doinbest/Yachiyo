import {chineseTerminalLine,replyIsFault} from './terminal-format.mjs?v=grab-terminal-20261002';
import {GRAB_FORM_DEFAULTS,GRAB_TEST_PRESET} from './grab-params.mjs?v=grab-preset-20260930';
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
  ...[['z_ppm','Z 命令脉冲 / mm'],['x_ppm','X 命令脉冲 / mm']].map(([key,label])=>({key,label,group:'尺度与行程',min:0.0001,max:100000})),
  ...[['z_min','Z 最小位置'],['z_max','Z 最大位置'],['z_grab','Z 夹持位置'],['z_place','Z 放置位置'],['z_lift','Z 提起位置'],['z_observe','Z 观察位置'],['x_min','X 最小位置'],['x_max','X 最大位置'],['x_pre','X 预备位置']].map(([key,label])=>({key,label:`${label} / mm`,group:'尺度与行程',min:-10000,max:10000})),
  ...[['pos_tol','轴到位容差 / mm'],['stop_speed','停稳速度阈值 / mm·s⁻¹'],['dx_tol','DX 抓取窗口 / px'],['dy_tol','DY 抓取窗口 / px']].map(([key,label])=>({key,label,group:'反馈与抓取条件',min:0.0001,max:100})),
  ...[['close_ms','夹爪闭合 / 松开等待 / ms',1],['stable_ms','精定位稳定时间 / ms',200]].map(([key,label,min])=>({key,label,group:'反馈与抓取条件',min,max:120000,step:1})),
  ...[['feedback_ms','反馈最大年龄 / ms'],['age_ms','有效坐标接收超时 / ms']].map(([key,label])=>({key,label,group:'反馈与抓取条件',min:50,max:5000,step:1})),
  {key:'loss_grace_ms',label:'蓝色对准漏检容忍 / ms（0 关闭）',group:'反馈与抓取条件',min:0,max:300,step:1},
  {key:'frames',label:'稳定坐标观测次数',group:'反馈与抓取条件',min:3,max:100,step:1},
  {key:'retries',label:'重新对准次数上限',group:'反馈与抓取条件',min:0,max:10,step:1},
  {key:'ref_u',label:'观察姿态准心 U / px',group:'准心与像素响应',min:0,max:639,step:1},
  {key:'ref_v',label:'观察姿态准心 V / px',group:'准心与像素响应',min:0,max:479,step:1},
  ...[['j00','DX / 底盘前进'],['j01','DX / 底盘左移'],['j02','DX / X 伸出'],['j10','DY / 底盘前进'],['j11','DY / 底盘左移'],['j12','DY / X 伸出']].map(([key,label])=>({key,label:`${label} / px·mm⁻¹`,group:'准心与像素响应',min:-10000,max:10000})),
  ...[['body_speed','底盘速度上限'],['x_speed','X 速度上限'],['z_speed','Z 速度上限']].map(([key,label])=>({key,label:`${label} / mm·s⁻¹`,group:'控制与运动边界',min:0.0001,max:100})),
  {key:'accel',label:'平移加速度上限 / mm·s⁻²',group:'控制与运动边界',min:0.0001,max:1000},
  {key:'travel_mm',label:'局部底盘行程上限 / mm（0 关闭）',group:'控制与运动边界',min:0,max:10000},
  {key:'lead_mm',label:'X 目标领先反馈上限 / mm',group:'控制与运动边界',min:0.0001,max:10000},
  {key:'lambda',label:'视觉收敛增益 / s⁻¹',group:'控制与运动边界',min:0.001,max:10},
  {key:'x_weight',label:'X 分配权重',group:'控制与运动边界',min:0.01,max:100},
  ...['z_proc_grab','z_proc_lift','z_car_lift','z_proc_place','z_temp_place','z_stack_place','x_car','base_car_offset'].map(key=>({key,label:({z_proc_grab:'粗加工夹取Z',z_proc_lift:'粗加工提起Z',z_car_lift:'车载取出提起Z',z_proc_place:'粗加工释放Z',z_temp_place:'暂存第一层释放Z',z_stack_place:'暂存第二层释放Z',x_car:'车载取放口X',base_car_offset:'车载Base偏转角'})[key],group:'取放场景',min:-10000,max:10000})),
];
export const turntableParameters=[
  ...[['slot1_deg','第一仓位 / °'],['slot2_deg','第二仓位 / °'],['slot3_deg','第三仓位 / °']].map(([key,label])=>({key,label,min:-360,max:360,readOnly:key==='slot1_deg'})),
  {key:'speed_rpm',label:'分度速度 / RPM',min:1,max:5000,step:1},
  {key:'acc',label:'加速度档位',min:0,max:255,step:1},
  {key:'pos_tol_deg',label:'到位容差 / °',min:0.1125,max:180},
  {key:'stable_ms',label:'停稳时间 / ms',min:0,max:60000,step:1},
  {key:'feedback_ms',label:'反馈新鲜度 / ms',min:100,max:60000,step:1}
];
export const commands = [
  ...turntableParameters.map(p=>command(`turntable-get-${p.key}`,'vision',`读取 ${p.label}`,`turntable get ${p.key}`)),
  ...turntableParameters.filter(p=>!p.readOnly).map(p=>command(`turntable-set-${p.key}`,'vision',p.label,`turntable set ${p.key} {value}`,[number('value',p.label,'',p.min,p.max,p.step??'any')])),
  ...[['origin','建立转盘参考'],['stop','停止车载转盘'],['status','读取转盘状态'],['inventory','读取三仓库存']].map(([id,label])=>command(`turntable-${id}`,'vision',label,`turntable ${id}`)),
  command('turntable-empty','vision','确认三仓为空','turntable inventory empty'),
  command('turntable-index','vision','分度到仓位','turntable index {slot}',[choice('slot','仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']])]),
  command('turntable-jog','vision','转盘点动','turntable jog {degree}',[number('degree','相对角度 / °','',-360,360,.1)]),
  command('turntable-inventory-set','vision','校正仓位库存','turntable inventory {slot} {content}',[choice('slot','仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('content','实际库存',[['empty','空仓'],['unknown','待核对'],['1','红色'],['2','黄色'],['3','蓝色'],['4','绿色'],['5','黑色'],['6','浅蓝色']])]),
  command('grab-fixed-slot','vision','指定仓位固定夹取','grab start fixed {slot} {color}',[choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('color','物料颜色',grabColorOptions,'3')]),
  command('grab-pick-slot','vision','指定仓位视觉夹取','grab start pick {color} {slot}',[choice('color','物料颜色',grabColorOptions,'3'),choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']])]),
  command('grab-store-proc','vision','粗加工取回入仓','grab store proc {slot} {color}',[choice('slot','入仓位置',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('color','物料颜色',grabColorOptions,'3')]),
  command('grab-take','vision','车载取出并释放','grab take {slot} {scene}',[choice('slot','取出仓位',[['1','第一仓'],['2','第二仓'],['3','第三仓']]),choice('scene','释放场景',[['rough','粗加工'],['temp1','暂存第一层'],['temp2','暂存第二层']])]),
  command('grab-return','vision','返回外侧观察姿态','grab return'),
  command('grab-base-pos','vision','手动 Base 转到转盘','pos base {degree}',[number('degree','Base 电机轴相对角度 / °',-180,-360,360,0.1)],'自动取放已包含转向，此处仅用于手动定位。默认相对转动 −180°，每次执行都会再转一次，已在转盘方向时无需重复。若抓取处于 hold，先结束本次抓取并确认停止，再操作 Base。'),
  ...[['fixed','固定位置抓取'],['align','仅协同对准'],['pick','单件视觉抓取']].map(([mode,label])=>command(`grab-${mode}`,'vision',label,`grab start ${mode}`)),
  ...[['align','仅协同对准'],['pick','单件视觉抓取']].map(([mode,label])=>command(`grab-${mode}-color`,'vision',label,`grab start ${mode} {color}`,[choice('color','物料颜色',grabColorOptions,'3')])),
  command('grab-route','vision','前四段扫码与单件抓取','grab route {speed}',[number('speed','实车路线速度 / mm·s⁻¹','',10,5000)],'从路线起点显式启动；第2段模拟扫码，第4段抓取。自动取放并提起后等待人工验收，不进入第5段。先在地图页准备跑图。'),
  command('grab-status','vision','刷新抓取状态','grab status'),
  command('grab-stop','vision','停止单件抓取','grab stop',[],'停止自动推进并请求机构停止；保留夹持和 Z 承载。停止确认以新鲜反馈为准。'),
  command('grab-rehome','vision','重新建立三轴参考','grab rehome',[],'先移走夹持物并确认机械臂周围无障碍；按 Z、X、Base 顺序重新回零并读回验证。各轴确认后恢复对应默认参考，三轴完成后才能开始新任务。'),
  ...grabParameters.map(p=>command(`grab-get-${p.key}`,'vision',`读取 ${p.label}`,`grab get ${p.key}`)),
  ...grabParameters.map(p=>command(`grab-set-${p.key}`,'vision',p.label,`grab set ${p.key} {value}`,[number('value',p.label,grabDefaults[p.key]??'',p.min,p.max,p.step??'any')],p.key in grabDefaults?'预填程序默认值，不是设备读回值；X/Z尺度和位置在上电或 grab rehome 对应轴验证成功后载入，容差和等待时间在启动时载入。单轴手动回零只受理动作，不自动恢复参考参数。修改仅存MCU RAM，运行中拒绝修改。':'仅设置 MCU RAM；复位后需重新配置。运行中拒绝修改，输入值不代表设备已接受。')),
  command('pos','arm','相对转动','pos {axis} {degree}',[axis(),number('degree','电机轴相对角度 / °',10,-360,360,0.1)],'正负值表示方向；这是相对运动，不是绝对位置。本面板单次输入限 ±360°，固件另有脉冲限制。'),
  ...[['enable','使能'],['disable','取消使能'],['state','查询状态'],['stop','停止']].map(([id,label]) => command(id,'arm',label,`${id} {axis}`,[axis(true)])),
  command('position','arm','读取当前位置','position {axis}',[axis()],'返回值为脉冲数。'),
  command('home','arm','执行回零','home {axis} {mode}',[axis(true),choice('mode','回零方式',[['near','就近 · near'],['dir','指定方向 · dir'],['collision','碰撞 · collision'],['limit','限位 · limit']])],'回零会产生运动；方式需要与实际机构匹配。'),
  command('origin','arm','保存机械零点','origin {axis}',[axis(true)],'将当前机械位置保存为零点，与计数清零不同。'),
  command('zero','arm','当前位置计数清零','zero {axis}',[axis(true)],'只清除当前位置计数，不执行回零运动。'),
  command('config-get','arm','读取运动配置','config {axis}',[axis(true)]),
  command('config-set','arm','设置运动配置','config {axis} {rpm} {acc} {limit}',[axis(true),number('rpm','速度 / RPM',30,1,3000),number('acc','加速度参数',20,1,255),number('limit','脉冲限制',3200,0,3200)],'配置只保存在 RAM，复位后恢复默认；只有单独 Z 轴允许 limit=0。'),
  ...[['open','打开 · 500 µs'],['catch','夹取 · 700 µs'],['idle','待机松开 · 500 µs']].map(([id,label]) => command(`grip-${id}`,'grip',label,`grip ${id}`)),
  command('grip-duty','grip','手动设置占空比','grip duty {duty}',[number('duty','PWM 占空比 / %',2.5,2.5,12.5,0.1)],'仅用于手动标定；500 µs = 2.5%，700 µs = 3.5%。'),
  command('move','chassis','定距前后移动','chassis {direction} {distance}',[choice('direction','移动方向',[['forward','前进'],['backward','后退']]),number('distance','距离 / mm',10,1,200)],'始终带距离发送，避免误触发持续后退。'),
  command('velocity','chassis','持续速度控制','chassis velocity {forward} {left} {yaw}',[number('forward','前向速度 / mm·s⁻¹',0,-1000,1000),number('left','左向速度 / mm·s⁻¹',0,-1000,1000),number('yaw','偏航速度 / °·s⁻¹',0,-360,360)],'持续运动需主动点击「底盘停止」。关闭网页或断开串口不等于停车。'),
  command('chassis-stop','chassis','底盘停止','chassis stop',[],'固件可能返回 busy；以实际回复和小车状态为准。'),
  command('wheel','chassis','单轮测试','wheel {wheel} {rpm}',[choice('wheel','选择车轮',[['fl','左前 · fl'],['rl','左后 · rl'],['rr','右后 · rr'],['fr','右前 · fr']]),number('rpm','转速 / RPM',10,-100,100)],'正负值表示方向；停止请使用「车轮停止」。'),
  command('wheel-stop','chassis','车轮停止','wheel stop'),
  command('vision-status','vision','视觉状态','vision status'),
  command('camera-material','vision','相机识别物料','camera material {target}',[target()]),
  command('camera-stop','vision','停止相机请求','camera stop'),
  command('camera-status','vision','相机与 USB 状态','camera status',[],'读取香橙派最新坐标及USB状态；使用B2颜色识别。'),
  command('material-auto','vision','物料搜索与对准','material auto {target}',[target()],'已包含相机识别与对准；ALIGNED 仅表示对准完成，不表示完整搬运完成。'),
  command('material-status','vision','物料任务状态','material status'),
  command('material-stop','vision','停止物料任务','material stop'),
  command('vision-ref','vision','设置视觉参考点','vision ref'),
  command('vision-calib-material','vision','机械臂物料标定','vision calib material {target}',[target()]),
  command('vision-calib-chassis','vision','底盘物料标定','vision calib chassis {target}',[target()]),
  command('vision-material','vision','物料对准','vision material {target}',[target()]),
  command('vision-stop','vision','停止视觉任务','vision stop'),
  command('system-reset','system','重启 STM32','system reset',[],'仅空闲时重启主控；会重新执行初始化，不能代替停车或电机驱动器复位。'),
  command('imu-zero','system','Z 轴角度清零','imu zero',[],'保持静止；只将当前模块角度设为零，不做零偏标定。之后重新验证 IMU 并设置地图起点。'),
  command('info','system','设备信息','info'),
  command('bus-recover','system','一键恢复电机总线','bus recover',[],'停止任务并检查全部 7 台电机；成功后不会自动续跑。结果见终端，底盘同步缓存未确认时仍保持底盘锁定。'),
  command('bus-status','system','查看总线恢复结果','bus status'),
  command('help','system','固件指令帮助','help'),
  command('chassis-run','chassis','平移与转向测试','chassis run {vx} {vy} {omega} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-100,100,0.1),number('vy','左向速度 / mm·s⁻¹',0,-100,100,0.1),number('omega','逆时针角速度 / rad·s⁻¹',0,-0.15,0.15,0.01),number('seconds','保持时间 / s',1,0.1,59,0.1)],'角速度为 0 时自动保持起步航向，需要 IMU 验证通过；非 0 时执行给定转向。起步和停车各 500 ms；总时长为保持时间加 1 秒。'),
  command('chassis-heading','chassis','航向保持测试','chassis heading {vx} {vy} {heading} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-100,100,0.1),number('vy','左向速度 / mm·s⁻¹',0,-100,100,0.1),number('heading','HWT101 模块目标角度 / °','',-180,180,0.1),number('seconds','保持时间 / s',1,0.1,59,0.1)],'需本次启动验证通过；目标角度不是地图角度。起步和停车各 500 ms。'),
  command('chassis-origin','chassis','设置地图初始位姿','chassis origin {x} {y} {heading}',[number('x','地图 X / mm','',-10000,10000,0.1),number('y','地图 Y / mm','',-10000,10000,0.1),number('heading','地图航向 / °','',-180,180,0.1)],'需静止且 IMU 验证有效；只设地图位姿，不执行运动。'),
  command('chassis-task','chassis','读取运动任务','chassis task',[],'时间任务结束不表示到达地图位置或反馈确认停止。'),
  command('chassis-route-start','map','开始第 1 段','chassis route start {speed}',[number('speed','实车路线速度 / mm·s⁻¹',5000,10,5000,1)],'16段沿用启动速度，靠近目标自动减速；修改输入不会改变正在执行的路线。'),
  command('chassis-route-auto','map','自动跑完整路线','chassis route auto {speed}',[number('speed','自动路线速度 / mm·s⁻¹',5000,10,5000,1)],'确认停稳后等待0.1秒自动继续，共16段。仅行驶，不自动扫码、抓放；用取消路线或立即停止中止。'),
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
const debugIds = new Set(['position','grip-idle','grip-duty','move','velocity','wheel','wheel-stop','chassis-hold','chassis-status','feedback-read','feedback-select','feedback-on','feedback-off','camera-material','camera-status','camera-stop','vision-material','imu-stream-on','imu-stream-off']);
const calibrationIds = new Set(['home','origin','zero','config-get','config-set','vision-ref','vision-calib-material','vision-calib-chassis','units','reset-confirmed','chassis-origin','imu-cal-start','imu-verify','imu-cal-cancel']);
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
  if (/^(?:camera|material|vision|grab|turntable|VISION)\b/.test(line)) return 'vision';
  if (/^(?:pos|enable|disable|state|stop|position|home|origin|zero|config|grip)\b/.test(line)) return 'arm';
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
          value < field.min || value > field.max || (field.step === 1 && !Number.isInteger(value)))
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
    if (Math.hypot(Number(checked.vx), Number(checked.vy)) > 100)
      throw new Error('平移合速度不能超过 100 mm/s。');
    checked.hold_ms = String(Math.round(Number(checked.seconds) * 1000));
  }
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
  if (/^OK grab\b.*\bstate=hold\b/.test(line)) return /\bmode=align\b/.test(line) ? '对准动作完成 · 未执行抓取' : '取放动作完成 · 等待人工验收';
  if (/^OK (?:material|vision)\b.*\bstate=ALIGNED\b/.test(line)) return '对准完成';
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
    if (Object.hasOwn({recovery_used:2,recovery_count:3,recovery_left_ms:1500,grace_count:3},key) && (!Number.isInteger(result[key]) || result[key]<0 || result[key]>{recovery_used:2,recovery_count:3,recovery_left_ms:1500,grace_count:3}[key])) return null;
    if (['rx_seq','loss_ms','loss_max_ms'].includes(key) && (!Number.isInteger(result[key]) || result[key]<0 || result[key]>0xffffffff)) return null;
    if (['age','elapsed'].includes(key) && result[key]<0) return null;
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
  for (const key of ['segment','elapsed_ms']) {
    if (result[key]===undefined) {result[key]=null;continue;}
    if (!/^\d+$/.test(result[key]) || !Number.isSafeInteger(Number(result[key]))) return null;
    result[key]=Number(result[key]);
  }
  return result;
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
  const used=`${s.recovery_used}/2`;
  if(s.reason==='vision_recover_timeout')return `恢复窗口超时；已恢复 ${used}，任务停止`;
  if(s.reason==='vision_recover_limit')return `恢复次数耗尽（${used}）；任务停止`;
  if(s.state==='vision_grace')return `短暂漏检，沿原运动趋势减速；中断 ${s.loss_ms??'未报告'} ms；恢复观测 ${s.grace_count??'未报告'}/3；不会下降夹取`;
  if(s.reason==='vision_gap_recovered')return `短暂漏检已恢复，未消耗停车恢复次数；中断 ${s.loss_ms??'未报告'} ms；重新检查对准`;
  if(s.state==='vision_pause') {
    if(s.recovery_left_ms===null || s.recovery_left_ms===undefined || s.stop_confirmed!==1)return `漏检，正在停车；已恢复 ${used}；不会下降夹取`;
    const waiting=s.reason==='vision_reacquire_wait'?'已重发蓝色识别，等待新坐标':'等待目标恢复';
    return `${waiting}（${s.recovery_count??'未报告'}/3）；设备报告剩余 ${s.recovery_left_ms} ms；已恢复 ${used}；不会下降夹取`;
  }
  if(s.reason==='vision_recovered')return `已恢复对准（${used}）；重新检查对准与停稳`;
  if(s.reason==='cancelled')return '任务已取消，待恢复资格已清除';
  return `本次任务已恢复 ${used}`;
}
