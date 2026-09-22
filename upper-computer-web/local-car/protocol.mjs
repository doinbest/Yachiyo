// Source: template/App/arm_console.c. UI labels never go over the wire.
const choice = (key, label, options, value = options[0][0]) => ({key,label,options,value});
const number = (key,label,value,min,max,step=1) => ({key,label,value,min,max,step});
const axes = [['base','底座 · base'],['z','升降 · z'],['x','伸缩 · x']];
const axis = (all=false) => choice('axis','目标轴',all ? [...axes,['all','全部机械臂轴 · all']] : axes);
const target = () => choice('target','目标颜色',['红色','黄色','蓝色','绿色','黑色','浅蓝色'].map((label,i) => [String(i+1),`${label} · ${i+1}`]));
const command = (id,group,label,template,fields=[],note='') => ({id,group,label,template,fields,note});
export const groups = [['arm','机械臂'],['grip','夹爪'],['chassis','底盘'],['vision','视觉'],['qr','二维码'],['system','系统']];
export const commands = [
  command('pos','arm','相对转动','pos {axis} {degree}',[axis(),number('degree','电机轴相对角度 / °',10,-360,360,0.1)],'正负值表示方向；这是相对运动，不是绝对位置。本面板单次输入限 ±360°，固件另有脉冲限制。'),
  ...[['enable','使能'],['disable','取消使能'],['state','查询状态'],['stop','停止']].map(([id,label]) => command(id,'arm',label,`${id} {axis}`,[axis(true)])),
  command('position','arm','读取当前位置','position {axis}',[axis()],'返回值为脉冲数。'),
  command('home','arm','执行回零','home {axis} {mode}',[axis(true),choice('mode','回零方式',[['near','就近 · near'],['dir','指定方向 · dir'],['collision','碰撞 · collision'],['limit','限位 · limit']])],'回零会产生运动；方式需要与实际机构匹配。'),
  command('origin','arm','保存机械零点','origin {axis}',[axis(true)],'将当前机械位置保存为零点，与计数清零不同。'),
  command('zero','arm','当前位置计数清零','zero {axis}',[axis(true)],'只清除当前位置计数，不执行回零运动。'),
  command('config-get','arm','读取运动配置','config {axis}',[axis(true)]),
  command('config-set','arm','设置运动配置','config {axis} {rpm} {acc} {limit}',[axis(true),number('rpm','速度 / RPM',30,1,3000),number('acc','加速度参数',20,1,255),number('limit','脉冲限制',3200,0,3200)],'配置只保存在 RAM，复位后恢复默认；只有单独 Z 轴允许 limit=0。'),
  ...[['open','打开'],['catch','抓取'],['idle','待机']].map(([id,label]) => command(`grip-${id}`,'grip',label,`grip ${id}`)),
  command('grip-duty','grip','设置占空比','grip duty {duty}',[number('duty','PWM 占空比 / %',7.5,2.5,12.5,0.1)]),
  command('move','chassis','定距前后移动','chassis {direction} {distance}',[choice('direction','移动方向',[['forward','前进'],['backward','后退']]),number('distance','距离 / mm',10,1,200)],'始终带距离发送，避免误触发持续后退。'),
  command('velocity','chassis','持续速度控制','chassis velocity {forward} {left} {yaw}',[number('forward','前向速度 / mm·s⁻¹',0,-1000,1000),number('left','左向速度 / mm·s⁻¹',0,-1000,1000),number('yaw','偏航速度 / °·s⁻¹',0,-360,360)],'持续运动需主动点击「底盘停止」。关闭网页或断开串口不等于停车。'),
  command('chassis-stop','chassis','底盘停止','chassis stop',[],'固件可能返回 busy；以实际回复和小车状态为准。'),
  command('wheel','chassis','单轮测试','wheel {wheel} {rpm}',[choice('wheel','选择车轮',[['fl','左前 · fl'],['rl','左后 · rl'],['rr','右后 · rr'],['fr','右前 · fr']]),number('rpm','转速 / RPM',10,-100,100)],'正负值表示方向；停止请使用「车轮停止」。'),
  command('wheel-stop','chassis','车轮停止','wheel stop'),
  command('vision-status','vision','视觉状态','vision status'),
  command('camera-material','vision','相机识别物料','camera material {target}',[target()]),
  command('camera-stop','vision','停止相机请求','camera stop'),
  command('camera-status','vision','相机与 USB 状态','camera status',[],'当前 USB 使用 B2 颜色识别；不支持旧 sys.ping# 或 B3 圆环编号协议。'),
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
  command('help','system','固件指令帮助','help'),
  command('chassis-run','chassis','平移与转向测试','chassis run {vx} {vy} {omega} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-100,100,0.1),number('vy','左向速度 / mm·s⁻¹',0,-100,100,0.1),number('omega','逆时针角速度 / rad·s⁻¹',0,-0.15,0.15,0.01),number('seconds','保持时间 / s',1,0.1,59,0.1)],'角速度为 0 时自动保持起步航向，需要 IMU 验证通过；非 0 时执行给定转向。起步和停车各 500 ms；总时长为保持时间加 1 秒。'),
  command('chassis-heading','chassis','航向保持测试','chassis heading {vx} {vy} {heading} {hold_ms}',[number('vx','前向速度 / mm·s⁻¹',0,-100,100,0.1),number('vy','左向速度 / mm·s⁻¹',0,-100,100,0.1),number('heading','HWT101 模块目标角度 / °','',-180,180,0.1),number('seconds','保持时间 / s',1,0.1,59,0.1)],'需本次启动验证通过；目标角度不是地图角度。起步和停车各 500 ms。'),
  command('chassis-origin','chassis','设置地图初始位姿','chassis origin {x} {y} {heading}',[number('x','地图 X / mm','',-10000,10000,0.1),number('y','地图 Y / mm','',-10000,10000,0.1),number('heading','地图航向 / °','',-180,180,0.1)],'需静止且 IMU 验证有效；只设地图位姿，不执行运动。'),
  command('chassis-task','chassis','读取运动任务','chassis task',[],'时间任务结束不表示到达地图位置或反馈确认停止。'),
  ...[['start','开始第 1 段'],['next','手动下一段'],['cancel','取消路线'],['status','查询实车路线']].map(([action,label])=>command(`chassis-route-${action}`,'chassis',label,`chassis route ${action}`)),
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
  definition.section = calibrationIds.has(definition.id) ? 'calibration' : debugIds.has(definition.id) ? 'debug' : 'common';
  definition.requiresInput = definition.fields.some(field => field.value === '');
}
commands.find(c => c.id === 'stop').note = 'stop all 会请求停止正在运行的视觉/物料任务；否则停止机械臂轴。它不等于整车所有电机已停止。';

/** Classifies without rewriting text; unknown commands stay manual. */
export function moduleForWire(text) {
  if (typeof text !== 'string') return 'manual';
  const line = text.trim().replace(/^(?:arm> )+/, '').replace(/^(?:OK|ERR)\s+/, '');
  if (/^@CHASSIS\b/.test(line) || /^chassis stream\b/.test(line)) return 'map';
  if (/^(?:qr\b|@QR\b|EVT qr\b)/.test(line)) return 'qr';
  if (/^(?:chassis\b|wheel(?:\s|=)|\[CHASSIS\])/.test(line)) return 'chassis';
  if (/^(?:camera|material|vision)\b/.test(line)) return 'vision';
  if (/^(?:pos|enable|disable|state|stop|position|home|origin|zero|config|grip)\b/.test(line)) return 'arm';
  if (/^(?:system|console|imu|info|help|HWT101)\b/.test(line) || /^\[IMU\b/.test(line)) return 'system';
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
  if ((id === 'pos' && Number(checked.degree) === 0) || (id === 'wheel' && Number(checked.rpm) === 0))
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
  if (line.startsWith('ERR')) return '设备拒绝或执行错误';
  if (/^OK (?:material|vision)\b.*\bstate=ALIGNED\b/.test(line)) return '对准完成';
  if (/\baccepted\b/.test(line)) return '请求已接受 · 尚未确认到位';
  if (/^OK state \S+ .*\breached=1\b/.test(line)) return '所查询轴报告已到位';
  if (/^OK state \S+ .*\breached=0\b/.test(line)) return '所查询轴报告未到位';
  if (line.startsWith('OK')) return '收到设备回复';
  return '收到设备数据';
}
