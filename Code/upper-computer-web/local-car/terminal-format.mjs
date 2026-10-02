import {radarReasonText} from './radar-format.mjs';
export const visualStateText=state=>({Idle:'无活动视觉请求',Off:'USB 未枚举',Wait:'等待首个回复',Lost:'已收到报文，当前无有效坐标',Ok:'有效坐标',Stale:'坐标已过期'})[state]??'未报告';
// Presentation only. Never feed translated text back into a protocol parser.
const names={vision_grace:'短暂漏检，减速等待',vision_gap_decelerating:'沿原运动趋势减速，等待连续有效坐标',vision_gap_recovered:'短暂漏检已恢复，未消耗停车恢复次数',vision_pause:'视觉暂停，不会下降夹取',vision_pause_stopping:'漏检，正在停车',vision_recover_wait:'已停稳，等待目标恢复',vision_reacquire_wait:'已重发目标颜色识别，等待新坐标',vision_recovered:'已恢复对准',vision_recover_timeout:'恢复窗口超时',vision_recover_limit:'恢复次数耗尽',vision_usb_off:'视觉 USB 断开',vision_request_lost:'视觉请求失效',vision_stale:'有效坐标接收超时',vision_lost:'视觉坐标失效',feedback_stale:'机械反馈过期',chassis_tx:'底盘通信故障',base:'Base 底座',x:'X 伸缩轴',z:'Z 升降轴',all:'全部轴',fixed:'固定取放',align:'仅协同对准',pick:'视觉取放',idle:'空闲',prepare:'准备中',acquire:'等待视觉',homing:'正在回零',descend:'下降夹取',close:'夹紧等待',lift:'提起',turn:'Base 转向',place:'下降放置',release:'松爪等待',retract:'提起复位',hold:'动作完成，等待验收',stopping:'正在停止',error:'故障',running:'运行中',waiting:'等待中',station:'站点等待',done:'已完成',complete:'已完成',failed:'失败',none:'无',cancelled:'已取消',pending_tx:'等待停车发送',monitoring:'等待四轮停稳',stopped:'四轮已确认停稳',missing_feedback:'缺少新鲜反馈',timeout:'超时',tx_failed:'发送失败',new_motion:'出现新动作，原停车确认失效',reply_timeout:'驱动回复超时',uart_error:'串口通信错误',motor_request:'电机请求失败',motor_feedback:'电机反馈异常',config_missing:'缺少参数',home_z:'Z 碰撞回零',home_x:'X 碰撞回零',home_base:'Base 就近回零',home_complete:'三轴回零完成',home_stop_unconfirmed:'回零停止未确认',aligned_wait_check:'对准完成，等待验收',placed_wait_check:'取放完成，等待验收',VERIFYING:'静止验证中',DONE:'完成',PASS:'通过',FAIL:'失败',Idle:'空闲',Wait:'等待相机回复',ALIGNED:'对准完成',ERROR:'故障',unlimited:'不限',initial:'初始模型',custom:'自定义模型'};
const fields={ref_cause:'参考状态',loss_ms:'本次或最近中断(ms)',loss_max_ms:'本任务最长中断(ms)',grace_count:'减速期间恢复观测(/3)',recovery_used:'已恢复次数(/2)',recovery_count:'连续恢复观测(/3)',recovery_left_ms:'设备报告剩余(ms)',state:'状态',mode:'模式',reason:'原因',missing:'缺少参数',axis:'轴',step:'步骤',elapsed:'耗时(ms)',elapsed_ms:'耗时(ms)',rpm:'转速(RPM)',acc:'加速度',limit_pulses:'脉冲限制',raw_units:'位置原始单位',raw_rpm:'转速原始值',en:'使能',reached:'到位',degree:'角度(°)',pulses:'命令脉冲',raw:'原始状态',flags:'状态位',stop_requested:'已请求停止',stop_confirmed:'停止已确认',tx_complete:'停车发送完成',wheels_stopped:'四轮停稳确认',locked:'总线锁定',bus_locked:'总线锁定',fault_ms:'故障时刻(ms)',owner:'事务来源',addr:'电机ID',func:'功能码',active:'活动',wait:'等待应答',arm_ready:'机械臂就绪',chassis_ready:'底盘就绪',tasks_resumed:'已自动续跑',home_reference:'回零参考',color:'Color',cx:'CX',cy:'CY',dx:'像素水平偏差',dy:'像素垂直偏差',x:'X位置(mm)',xt:'X目标(mm)',z:'Z位置(mm)',zt:'Z目标(mm)',b:'Base角度(°)',bt:'Base目标(°)',model:'视觉模型',protocol:'视觉协议',age_source:'时间来源',rx_seq:'MCU接收序号',vision:'坐标状态',age:'坐标接收间隔(ms)',usb:'USB已枚举',fn:'功能码',target:'目标',frame:'收到过匹配报文',current_valid:'当前数据有效',target_valid:'目标有效',seq:'MCU接收序号',data_tick:'MCU接收时刻(ms)',invalid_reports:'本请求无效坐标报文数',rx_bytes:'收到字节数',checksum_errors:'校验错误',overflows:'溢出',frame_age_ms:'最近匹配报文接收间隔(ms)',last_frame_ms:'最后收帧时刻(ms)',vf:'前向速度(mm/s)',vl:'左向速度(mm/s)',yaw_deg:'航向(°)',target_deg:'目标航向(°)',yaw_valid:'航向有效',verified:'验证通过',control_ready:'控制就绪',keep_still:'需保持静止',elapsed_s:'已用时(s)',total_s:'总时间(s)',verify_elapsed_s:'验证用时(s)',wheel:'轮ID',valid:'有效标志',segment:'路段',error:'错误',ack:'应答模式',units:'单位/圈',duty:'占空比',rx_overflow:'接收溢出',reply_dropped:'普通回复丢失',urgent_dropped:'紧急回复丢失',debug_dropped:'旧调试样本替换或丢弃',reply_pending:'回复队列字节',reply_peak:'回复队列峰值',rx_peak:'接收队列峰值',rx_bytes_total:'累计接收字节',tx_bytes:'累计发送字节',rx_restarts:'接收重启次数'};
const colors={1:'red',2:'yellow',3:'blue',4:'green',5:'black',6:'light_blue'};
const referenceNames={boot_pending:'上电待验证',home_verifying:'三轴正在验证',home_verified:'三轴已验证',base_place_turn:'Base 已转向放置处',manual_home_all:'手动全部回零后待验证',manual_home_x:'手动 X 回零后待验证',manual_home_z:'手动 Z 回零后待验证',manual_home_base:'手动 Base 回零后待验证',manual_zero_all:'手动全部清零后待验证',manual_zero_x:'手动 X 清零后待验证',manual_zero_z:'手动 Z 清零后待验证',manual_zero_base:'手动 Base 清零后待验证',manual_base_move:'Base 已手动移动',x_reference_changed:'X 参考失效',z_reference_changed:'Z 参考失效',base_reference_changed:'Base 参考失效',all_reference_changed:'全部参考失效'};
const value=v=>referenceNames[v]??names[v]??v;
export function replyIsFault(line){
  return /^\s*(?:arm>\s*)?ERR\b/.test(line)||/\b(?:locked|bus_locked)=1\b/.test(line)||/\bstate=(?:error|ERROR|FAULT|FAILED)\b/.test(line)||/\bresult=FAIL\b/.test(line)||/\breason=(?:motor_\w+|home_\w*(?:fail|timeout|unconfirmed)\w*|\w*(?:timeout|failed|fault)|uart_error|config_missing|bus_unavailable|vision_recover_limit|vision_usb_off|vision_request_lost|vision_stale|vision_lost|feedback_stale|chassis_tx)\b/.test(line)||/^OK bus recover complete\b.*\b(?:arm_ready|chassis_ready)=0\b/.test(line);
}
function uartErrors(raw){
  const n=Number(raw);if(!Number.isInteger(n)||n<0)return raw;
  const labels=[[1,'奇偶校验错误'],[2,'噪声'],[4,'帧错误'],[8,'接收溢出'],[16,'DMA错误']].filter(([bit])=>n&bit).map(([,label])=>label);
  if(n&~31)labels.push(`未知错误位 ${raw}`);
  return `${labels.join('、')||'无'}（${raw}）`;
}
function translateToken(token){
  const pair=token.match(/^([a-z_]+)=(.*)$/i);
  if(!pair)return value(token);
  const [,key,v]=pair;
  if(key==='uart_error')return `串口错误：${uartErrors(v)}`;
  if(key==='vision')return `坐标状态：${visualStateText(v)}`;
  if(key==='age_source' && v==='rx')return '时间来源：MCU 接收';
  if(!fields[key])return token; // New firmware fields stay visible verbatim.
  return `${fields[key]}：${key==='color'?(colors[v]??v):value(v)}`;
}
/** Keep grab progress readable; original reports remain the source for diagnostics. */
function grabStatusSummary(line){
  const status={};
  for(const token of line.trim().split(/\s+/).slice(2)){
    const match=token.match(/^([a-z_]+)=([^\s=]+)$/);
    if(!match||Object.hasOwn(status,match[1]))return null;
    status[match[1]]=match[2];
  }
  if(!/^[a-z_]+$/.test(status.state??''))return null;
  const parts=['mode','state','reason'].filter(key=>status[key]!==undefined).map(key=>
    key==='reason'&&status.reason==='reference_invalid'?'原因：三轴参考失效':translateToken(`${key}=${status[key]}`));
  const finitePixel=raw=>/^[+-]?(?:\d+\.?\d*|\.\d+)$/.test(raw??'')&&Number.isFinite(Number(raw));
  if(status.vision==='Ok'&&finitePixel(status.dx)&&finitePixel(status.dy))
    parts.push(`像素偏差 dx/dy：${Number(status.dx)} / ${Number(status.dy)} px`);
  if(status.missing&&status.missing!=='none')parts.push(translateToken(`missing=${status.missing}`));
  const referenceChanged=/^(?:manual_(?:home|zero)_(?:all|x|z|base)|manual_base_move|base_place_turn|(?:x|z|base|all)_reference_changed)$/.test(status.ref_cause??'');
  if(referenceChanged||status.reason==='reference_invalid'){
    if(status.ref_cause&&status.ref_cause!=='none')parts.push(translateToken(`ref_cause=${status.ref_cause}`));
    if(status.reason==='config_missing'||status.reason==='reference_invalid')parts.push('请重新建立三轴参考');
  }
  if(status.stop_confirmed==='1')parts.push('停止：该次已确认');
  else if(status.stop_requested==='1')parts.push('停止：已请求，等待确认');
  else if(status.state==='stopping')parts.push('停止：等待确认');
  return `${replyIsFault(line)?'故障 · ':''}抓取任务：${parts.join('；')}`;
}
export function chineseTerminalLine(original){
  const line=original.replace(/^(?:arm>\s*)+/, '');
  const radar=line.match(/^(OK|ERR) radar ([a-z_]+)$/);if(radar)return `雷达${radar[1]==='ERR'?'未受理':'设备回复'}：${radarReasonText(radar[2])}`;
  if(/^(?:OK|ERR|EVT) grab state=/.test(line)){
    const summary=grabStatusSummary(line);
    if(summary)return summary;
  }
  if(line.startsWith('VISION RX ')){
    const f=Object.fromEntries(line.slice(10).trim().split(/\s+/).map(t=>t.split('='))),color=colors[f.color];
    if(!color)return original;
    if(f.valid==='0')return `Color = ${color}，当前无有效视觉数据`;
    if(f.valid!=='1')return original;
    if(f.fresh==='0')return `Color = ${color}，坐标已过期`;
    if(f.fresh!=='1'||!/^\d+$/.test(f.cx)||!/^\d+$/.test(f.cy)||Number(f.cx)>639||Number(f.cy)>479)return original;
    return `Color = ${color}，CX = ${Number(f.cx)}，CY = ${Number(f.cy)}`;
  }
  if(/^OK camera state=/.test(line)){
    const state=line.match(/\bstate=(\w+)/)?.[1],protocol=line.match(/\bprotocol=(\w+)/)?.[1];
    return `相机：${visualStateText(state)}；视觉协议：${protocol??'未报告'}；`+line.split(/\s+/).slice(2).filter(t=>!/^state=|^protocol=/.test(t)).map(translateToken).join('；');
  }
  if(line==='STM32 mechanical arm console ready')return 'STM32 已启动，正在准备三轴回零';
  if(line.startsWith('Boot:'))return '上电流程：Z/X 碰撞回零 → Base 就近回零 → IMU 静止验证 5 秒；遥测默认关闭';
  let m=line.match(/^ARM STM32F407 USART1=(\d+) UART5=(\d+)$/);
  if(m)return `主控 STM32F407；控制台 ${m[1]} 波特；电机总线 ${m[2]} 波特`;
  if(line.startsWith('BUILD '))return `固件构建信息：${line.slice(6).replace('motorbus_diag=1','电机总线诊断开启').replace('date=','日期 ').replace('time=','时间 ')}`;
  if(line==='chassis FL=id1 RL=id2 RR=id3 FR=id4')return '底盘电机：左前 1、左后 2、右后 3、右前 4';
  if(line==='arm base=id5 z=id6 x=id7 pos=relative unit=degree')return '机械臂：Base=5、Z=6、X=7；pos 为相对角度指令，单位 °';
  if(line.startsWith('arm positive_dir '))return '机械臂正方向：'+line.slice(17).replace(/\b(base|z|x)=([01])/g,(_,a,d)=>`${value(a)} ${d==='0'?'顺时针':'逆时针'}`).replace(' (0=CW 1=CCW)','');
  if(line.startsWith('[MOTOR BUS] '))return `电机总线${/locked=1\b/.test(line)?'已锁定':'未锁定'}；`+line.slice(12).split(/\s+/).map(translateToken).join('；');
  if(/^OK bus recover requested\b/.test(line))return '总线恢复请求已受理；任务不会自动继续';
  if(/^OK bus recover complete\b/.test(line))return `${replyIsFault(line)?'电机总线未完全恢复':'电机总线恢复完成'}；${line.slice(24).trim().split(/\s+/).filter(t=>!/^tasks_resumed=0$|^home_reference=check$/.test(t)).map(translateToken).join('；')}；任务不会自动继续，请检查回零参考`;
  if(/^OK chassis stop id=/.test(line))return '四轮停车：'+line.slice(16).split(/\s+/).map(translateToken).join('；');
  if(/^OK stop \w+ sent$/.test(line))return `${value(line.split(' ')[2])}停止指令已发送，尚未确认停稳`;
  if(/^OK (?:pos|home) /.test(line)&&/\baccepted\b/.test(line))return `${line.startsWith('OK home')?'回零':'运动'}请求已受理，尚未确认到位；`+line.split(/\s+/).slice(2).filter(t=>t!=='accepted').map(translateToken).join('；');
  if(/^OK chassis request accepted/.test(line))return '底盘请求已受理，尚未确认驱动应答或运动完成';
  if(/^OK config /.test(line))return '运动配置：'+line.slice(10).split(/\s+/).map(translateToken).join('；');
  const group=line.match(/^(OK|ERR|EVT) (bus|console|state|position|enable|disable|zero|origin|grip|camera|vision|material|grab|chassis|imu|qr|system)\b\s*(.*)$/);
  if(group){
    const labels={bus:'总线',console:'控制台诊断',state:'轴状态',position:'轴位置',enable:'使能',disable:'关闭使能',zero:'位置清零',origin:'回零点',grip:'夹爪',camera:'相机',vision:'Base/X 对准',material:'物料任务',grab:'抓取任务',chassis:'底盘',imu:'IMU',qr:'二维码',system:'系统'};
    const words={status:'状态',route:'路线',recover:'恢复',rehome:'重新建立三轴参考',get:'参数读取',set:'参数设置',accepted:'请求已受理，未确认到位',sent:'已发送，未确认动作完成',stop:'停止请求已处理',start:'启动',rejected:'请求被拒绝',stats:'统计',last:'最近数据',saved:'已保存',stream:'连续输出',on:'开启',off:'关闭',pending:'等待执行',partial:'部分完成'};
    return `${replyIsFault(line)?'故障 · ':''}${labels[group[2]]}：`+group[3].split(/\s+/).map(t=>words[t]??translateToken(t)).join('；');
  }
  if(/^\[IMU (?:CAL|SAVE|VERIFY)\]/.test(line))return line.replace(/^\[IMU (CAL|SAVE|VERIFY)\]/,(_,k)=>({CAL:'IMU 标定/验证',SAVE:'IMU 保存状态',VERIFY:'IMU 验证数据'})[k]).split(/\s+/).map(translateToken).join('；');
  return original;
}

/** Startup progress derives only from MCU reports, never a browser countdown. */
export function bootProgress(line,current=''){
  if(line==='STM32 mechanical arm console ready')return '正在回零';
  if(!current)return current;
  if(/^OK grab state=homing\b/.test(line)){
    const reason=line.match(/\breason=(\w+)/)?.[1];
    return value(reason??'homing');
  }
  if(/\breason=home_complete\b/.test(line))return '回零完成，等待 IMU 静止验证';
  if(/\breason=home_/.test(line)&&replyIsFault(line))return '启动失败：'+chineseTerminalLine(line);
  if(/^\[IMU CAL\]/.test(line)){
    if(/\bresult=PASS\b/.test(line)&&/\bcontrol_ready=1\b/.test(line))return '已就绪';
    if(/\bresult=FAIL\b/.test(line))return '验证失败：'+chineseTerminalLine(line);
    if(/\bstate=VERIFYING\b/.test(line))return '正在验证 IMU，请保持静止';
  }
  return current;
}

export function describeStopEvent(e,connected,disconnecting){
  if(!e.latched)return e.status==='idle'?'本地服务就绪 · CMD Ctrl+C 请求停车':'停止锁定已解除 · 任务不会自动继续';
  if(!connected&&['timeout','disconnected'].includes(e.status))return '串口已断开，停止未确认';
  return ({requested:'正在提交停车请求',sent:'停车请求已发出，等待设备回复',accepted:e.tx_complete?'停车发送完成，等待四轮反馈确认':'请求已受理，等待停车发送完成',confirmed:'四轮反馈已确认停车',timeout:disconnecting?'串口即将断开，停止未确认':'停车确认超时，停止未确认',disconnected:'串口已断开，停止未确认'})[e.status]||'停止已锁定，等待停车确认';
}
