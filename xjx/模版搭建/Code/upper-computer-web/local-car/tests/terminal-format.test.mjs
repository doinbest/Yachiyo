import {bootProgress,describeStopEvent,replyIsFault} from '../terminal-format.mjs';
import {visibleTerminalLogs} from '../terminal-log.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {formatTerminalLine,describeReply} from '../protocol.mjs';
test('processed pickup distinguishes observation alignment and committed descent',()=>{
 assert.match(formatTerminalLine('OK grab state=align mode=pick reason=processed_alignment'),/在观察高度对准加工物料/);
 assert.match(formatTerminalLine('OK grab state=descend mode=pick reason=processed_descending'),/保持水平位置下降夹取/);
 // Historical firmware reports remain readable.
 const text=formatTerminalLine('OK grab state=descend mode=pick reason=pickup_alignment_reacquire');
 assert.match(text,/保持夹取高度重新对准/);
 assert.doesNotMatch(text,/提起/);
});
test('recovery evidence separates the current step and original fault in Chinese',()=>{
 assert.match(formatTerminalLine('OK bus attempt stage=rx_restart reason=uart_error addr=1 func=0x00 uart_error=0x00000002 tx_sent=0 elapsed_ms=500'),/本次恢复.*接收重启/);
 assert.match(formatTerminalLine('OK bus first reason=reply_timeout addr=7 func=0x36 uart_error=0x00000000'),/首次故障/);
});

test('Base approximate reference remains visible during later tasks without fault styling',()=>{
  const line='OK grab state=complete mode=fixed reason=work_ready base_home_degraded=1 ref_cause=base_home_degraded missing=none';
  assert.match(formatTerminalLine(line),/Base.*降级.*参考/);
  assert.equal(replyIsFault(line),false);
  assert.match(formatTerminalLine('OK grab home_degraded axis=base failed_home_flags=0x0B reference=approximate motion=quiet'),/降级/);
});

test('motor diagnostics distinguish actual current, current limit and drive home settings',()=>{
  assert.match(formatTerminalLine('OK bus read id=5 phase_ma=2500'),/Base.*实际相电流.*2500 mA/);
  assert.match(formatTerminalLine('OK bus read id=5 driver=Emm current_limit_ma=3000'),/闭环电流上限.*3000 mA/);
  assert.match(formatTerminalLine('OK bus read id=5 home_mode=0 home_dir=0 home_rpm=30 timeout_ms=10000'),/回零速度.*30 RPM.*回零超时.*10000 ms/);
});

test('single ID probe keeps raw replies, current FE and retained motion lock visible',()=>{
  assert.match(formatTerminalLine('OK bus test start id=4 rounds=10 locked=1 read_only=1'),/ID4.*10 轮.*只读.*运动锁定仍保留/);
  assert.match(formatTerminalLine('OK bus test reply id=4 round=1 func=0x3A reply_ms=2 hex=04:3A:03:6B'),/状态.*2 ms.*04 3A 03 6B/);
  assert.match(formatTerminalLine('ERR bus test read id=4 round=1 func=0x36 reason=uart_error uart_error=0x00000004'),/位置.*帧错误/);
  assert.match(formatTerminalLine('ERR bus test read id=4 round=1 func=0x35 reason=reply_timeout uart_error=0x00000000'),/速度.*应答超时/);
  assert.match(formatTerminalLine('ERR bus test read id=4 round=1 func=0x35 reason=tx_timeout uart_error=0x00000000'),/速度.*发送超时/);
  assert.match(formatTerminalLine('ERR bus test read id=4 round=1 func=0x35 reason=transport_error uart_error=0x00000000'),/速度.*传输失败/);
  const result='OK bus test complete id=4 rounds=10 active=0 attempts=30 sent=30 received=30 timeouts=0 uart_errors=0 rejected=0 max_reply_ms=3 last_uart_error=0x00000000 locked=1 reason=done';
  assert.match(formatTerminalLine(result),/已完成.*发送 30.*有效回复 30.*3 ms.*运动锁定仍保留/);
  assert.equal(replyIsFault(result),true);
});

test('cache recovery distinguishes zeroing, verification and stopped completion',()=>{
  const progress=formatTerminalLine('OK bus recovery active=1 success=0 reason=zeroing_chassis addr=2 checked=0xff locked=1');
  assert.match(progress,/清零四轮同步缓存/);
  const done=formatTerminalLine('OK bus recover complete arm_ready=1 chassis_ready=1 reason=complete tasks_resumed=0 home_reference=check chassis_cache=cleared wheels_stopped=1');
  assert.match(done,/同步缓存：已清除/);
  assert.match(done,/四轮停稳确认：1/);
  assert.match(done,/不会自动继续/);
});

test('bus motion status describes isolation without claiming measured movement',()=>{
  const locked=formatTerminalLine('OK bus motion locked=1 error=2 feedback_allowed=0');
  assert.match(locked,/运动隔离状态/);
  assert.doesNotMatch(locked,/检测到运动/);
  assert.match(locked,/通信或发送异常/);
  const healthy=formatTerminalLine('OK bus motion locked=0 error=0 feedback_allowed=1');
  assert.match(healthy,/运动隔离状态/);
  assert.doesNotMatch(healthy,/故障|检测到运动/);
});

test('recovery settling and persistent motion retain measured RPM and sample time',()=>{
  const progress=formatTerminalLine('OK bus recovery active=1 success=0 reason=waiting_stopped addr=5 checked=0x00 locked=1 speed_addr=5 speed_rpm=-20 speed_ms=61234');
  assert.match(progress,/等待新鲜速度确认停稳/);
  assert.match(progress,/速度采样电机ID：5/);
  assert.match(progress,/速度\(RPM\)：-20/);
  assert.match(progress,/速度采样时刻\(ms\)：61234/);
  const failed=formatTerminalLine('ERR bus recover reason=motor_moving addr=5 locked=1 speed_addr=5 speed_rpm=-20 speed_ms=61234');
  assert.match(failed,/电机仍在运动/);
  assert.match(failed,/反馈转速\(RPM\)：-20/);
  assert.match(failed,/速度采样时刻\(ms\)：61234/);
  assert.equal(replyIsFault('ERR bus recover reason=motor_moving addr=5 locked=1'),true);
});

test('common device and recovery replies explain actual completion scope',()=>{
  assert.match(formatTerminalLine('ARM STM32F407 USART1=115200 UART5=115200'),/控制台.*115200.*电机总线/);
  assert.match(formatTerminalLine('OK config z rpm=500 acc=120 limit_pulses=unlimited'),/Z.*500.*120.*不限/);
  const recovery='OK bus recover complete arm_ready=1 chassis_ready=1 reason=complete tasks_resumed=0 home_reference=check';
  assert.match(formatTerminalLine(recovery),/恢复完成.*不会自动继续.*回零/);
  assert.match(formatTerminalLine('OK pos base degree=-180.00 pulses=-1600 accepted'),/已受理.*未确认到位/);
  assert.match(formatTerminalLine('OK stop all sent'),/已发送.*未确认/);
});

test('OK is not success when bus is locked or recovery only partially succeeds',()=>{
  const bus='[MOTOR BUS] locked=1 reason=uart_error fault_ms=779 owner=0 addr=1 func=0xFE active=1 wait=0 uart_error=0x00000006';
  assert.match(formatTerminalLine(bus),/锁定.*噪声.*帧错误/);
  assert.match(describeReply(bus),/故障/);
  const partial='OK bus recover complete arm_ready=1 chassis_ready=0 reason=failed tasks_resumed=0 home_reference=check';
  assert.match(formatTerminalLine(partial),/未完全恢复/);
  assert.match(describeReply(partial),/故障/);
  assert.match(describeReply('OK grab state=stopping mode=align reason=motor_request missing=none'),/故障/);
});

test('visual invalid means no valid data, not a proven recognition miss',()=>{
  assert.equal(formatTerminalLine('VISION RX fn=B4 color=1 valid=0 fresh=0 cx=0 cy=0'),'Color = red，当前无有效视觉数据');
  assert.equal(formatTerminalLine('VISION RX fn=B4 color=1 valid=1 fresh=1 cx=334 cy=338'),'Color = red，CX = 334，CY = 338');
  assert.equal(formatTerminalLine('NEW_DEVICE something=unexpected'),'NEW_DEVICE something=unexpected');
  assert.match(formatTerminalLine('OK config z rpm=500 acc=120 limit_pulses=unlimited new_field=42'),/new_field=42/);
});

test('startup follows homing and actual IMU reports, without premature readiness',()=>{
  assert.equal(bootProgress('STM32 mechanical arm console ready',''),'已启动，等待设备状态');
  assert.match(bootProgress('OK grab state=homing reason=home_x','正在回零'),/X.*回零/);
  assert.match(bootProgress('OK grab state=idle reason=home_complete','正在回零'),/等待.*验证/);
  assert.match(bootProgress('[IMU CAL] state=VERIFYING elapsed_s=2','等待验证'),/正在验证/);
  assert.equal(bootProgress('[IMU CAL] result=PASS state=DONE verified=1 control_ready=1','正在验证'),'已就绪');
  assert.match(bootProgress('OK grab state=error reason=home_timeout','正在回零'),/失败.*回零超时/);
  assert.match(bootProgress('OK grab state=homing reason=home_bus_fault stop_requested=1','正在回零'),/失败.*总线.*故障/);
});

test('disabled boot homing waits for IMU and preserves the manual homing entry',()=>{
  const banner='Boot: arm auto-home=off; IMU verify=5s; keep still; manual homing: grab rehome; stream=off; imu cal start: 20s native calibration + 30s verification';
  assert.equal(formatTerminalLine('STM32 mechanical arm console ready'),'STM32 已启动');
  assert.match(formatTerminalLine(banner),/自动回零已关闭.*IMU.*5 秒.*grab rehome/);
  assert.equal(bootProgress(banner,'已启动，等待设备状态'),'自动回零已关闭，等待 IMU 静止验证');
  assert.equal(bootProgress('[IMU CAL] result=PASS state=DONE verified=1 control_ready=1','自动回零已关闭，等待 IMU 静止验证'),'已就绪');
  assert.equal(bootProgress('Boot: Z/X collision home -> Base near home -> IMU verify=5s','已启动，等待设备状态'),'正在回零');
});

test('parallel boot homes describe the tray wait and failure in Chinese',()=>{
  const banner='Boot: Z/X collision home + Base/ID8 near home in parallel -> IMU verify=5s; enable_verify=on';
  assert.match(formatTerminalLine(banner),/Base\/ID8.*并行执行.*全部完成后.*IMU/);
  assert.equal(bootProgress(banner,'尚未启动'),'正在回零');
  const wait='OK grab state=homing mode=fixed reason=home_wait_turntable home_started=7 home_verified=7';
  assert.equal(replyIsFault(wait),false);
  assert.match(formatTerminalLine(wait),/等待ID8车仓回零完成/);
  const fault='ERR grab state=homing reason=home_turntable_failed';
  assert.equal(replyIsFault(fault),true);
  assert.match(formatTerminalLine(fault),/ID8车仓.*参考.*转盘状态.*电机总线/);
});

test('Base-last boot describes parallel linear/tray homes and the pre-Base tray wait',()=>{
  const banner='Boot: Z/X collision home + ID8 near home in parallel -> Base near home -> IMU verify=5s; enable_verify=on';
  assert.match(formatTerminalLine(banner),/Z\/X.*ID8.*并行.*确认完成后.*Base.*最后.*IMU/);
  const wait='OK grab state=homing mode=fixed reason=home_wait_turntable home_started=3 home_verified=3';
  assert.match(formatTerminalLine(wait),/等待ID8.*回零完成/);
  assert.doesNotMatch(formatTerminalLine(wait),/机械臂回零已验证/);
  assert.equal(bootProgress(banner,'已启动，等待设备状态'),'正在回零');
});

test('homing enable recovery is progress, while failed verification is a fault',()=>{
  for(const reason of ['home_enable_wait','home_enable_recovery']) {
    const line=`OK grab state=homing mode=fixed reason=${reason} axis=z enable_recovery=1`;
    assert.equal(replyIsFault(line),false);
    assert.match(formatTerminalLine(line),/使能/);
    assert.match(bootProgress(line,'正在回零'),/使能/);
  }
  for(const reason of ['home_enable_failed','home_position_changed']) {
    const line=`OK grab state=homing mode=fixed reason=${reason} stop_requested=1`;
    assert.equal(replyIsFault(line),true);
    assert.match(bootProgress(line,'正在回零'),/失败/);
  }
  assert.match(formatTerminalLine('OK grab home_enable_recovery axis=z home_flags=0x03 motor_flags=0x02 raw=-141'),/补发一次使能.*Z/);
});

test('disconnect timeout wording follows the actual port state',()=>{
  const event={latched:true,status:'timeout'};
  assert.equal(describeStopEvent(event,false,false),'串口已断开，停止未确认');
  assert.match(describeStopEvent(event,true,true),/即将断开/);
  assert.equal(describeStopEvent({latched:true,status:'confirmed'},false,false),'四轮反馈已确认停车');
});


test('camera status never changes startup progress',()=>{
  const line='OK camera state=Wait protocol=B2 age_source=rx usb=1 active=1 frame=0';
  for(const stage of ['', '已就绪', '正在回零']) assert.equal(bootProgress(line,stage),stage);
});

test('recovery faults stay faults even in OK stopping replies; raw text remains intact',()=>{
  for(const reason of ['vision_recover_timeout','vision_recover_limit','vision_usb_off','vision_request_lost','feedback_stale','chassis_tx']) {
    const raw=`OK grab state=stopping reason=${reason} recovery_used=2 recovery_count=0 recovery_left_ms=na`;
    assert.equal(replyIsFault(raw),true,reason);
    assert.match(formatTerminalLine(raw),/故障/);
    assert.equal(visibleTerminalLogs([{direction:'RX',text:raw}],false,false)[0].text,raw);
  }
  assert.match(formatTerminalLine('OK grab state=vision_pause reason=vision_recover_wait recovery_used=1 recovery_count=2 recovery_left_ms=300'),/不会下降夹取.*等待目标恢复/);
  assert.equal(replyIsFault('OK grab state=vision_pause reason=vision_recover_wait'),false);
});


test('blue grace keeps the recovery meaning while hiding observation and duration details',()=>{
  const line=formatTerminalLine('OK grab state=vision_grace reason=vision_gap_decelerating loss_ms=170 loss_max_ms=200 grace_count=2 stop_requested=0 stop_confirmed=0');
  assert.match(line,/短暂漏检，减速等待/);
  assert.match(line,/沿原运动趋势减速，等待连续有效坐标/);
  assert.doesNotMatch(line,/中断\(ms\)|恢复观测|已请求停止|停止已确认/);
  assert.match(formatTerminalLine('OK grab state=align reason=vision_gap_recovered'),/未消耗停车恢复次数/);
});

test('failed boot homing reports pending reference, not a missing scale setting',()=>{
  const raw='OK grab state=error mode=fixed reason=home_bus_fault missing=home_pending stop_requested=1 stop_confirmed=0';
  assert.match(formatTerminalLine(raw),/回零.*总线.*故障/);
  assert.match(formatTerminalLine(raw),/回零未完成/);
  assert.doesNotMatch(formatTerminalLine(raw),/缺少参数|x_ppm/);
  const detail='ERR grab home_request axis=z action=home result=tx_error bus_locked=1 bus_reason=uart_error uart_error=0x00000002 fault_ms=29';
  assert.match(formatTerminalLine(detail),/回零请求.*Z.*串口通信错误.*噪声/);
  assert.match(bootProgress(raw,'正在回零'),/失败/);
});

test('grab summaries retain mode, state, reason and current pixels without normal diagnostics',()=>{
  const raw='arm> OK grab state=align mode=pick reason=running missing=none ref_cause=home_verified x=12.000 xt=13.000 z=0.000 zt=nan b=0.00 bt=nan dx=-12 dy=7 protocol=B2 age_source=rx rx_seq=118 vision=Ok color=3 model=initial age=21 vf=2.500 vl=0.250 stop_requested=0 stop_confirmed=0 elapsed=2500 recovery_used=1 recovery_count=0 recovery_left_ms=na loss_ms=170 loss_max_ms=200 grace_count=0';
  const summary=formatTerminalLine(raw);
  assert.match(summary,/模式：视觉取放.*状态：仅协同对准.*原因：运行中/);
  assert.match(summary,/像素偏差 dx\/dy：-12 \/ 7 px/);
  assert.doesNotMatch(summary,/缺少参数|参考状态|X位置|X目标|Z位置|Z目标|Base角度|Base目标|NaN|nan|B2|时间来源|序号|视觉模型|接收间隔|速度|耗时|恢复次数|恢复观测|中断\(ms\)|已请求停止|停止已确认/);
  assert.equal(visibleTerminalLogs([{direction:'RX',text:raw}],false,false)[0].text,raw);
});

test('grab summaries never display stale, absent or nonfinite pixels as current',()=>{
  for(const tail of ['vision=Lost dx=-12 dy=7','vision=Stale dx=-12 dy=7','dx=-12 dy=7','vision=Ok dx=na dy=na','vision=Ok dx=NaN dy=7','vision=Ok dx=7 dy=Infinity']) {
    const summary=formatTerminalLine(`OK grab state=vision_pause mode=pick reason=vision_recover_wait missing=none ref_cause=home_verified ${tail} recovery_count=2 recovery_left_ms=350`);
    assert.match(summary,/视觉暂停，不会下降夹取.*已停稳，等待目标恢复/);
    assert.doesNotMatch(summary,/像素偏差|NaN|Infinity|DX|DY|恢复观测|剩余/);
  }
});

test('recovery limit faults preserve the failure cause and pending or confirmed stop',()=>{
  const raw='OK grab state=vision_pause mode=pick reason=vision_recover_limit missing=none ref_cause=home_verified dx=na dy=na protocol=B2 age_source=rx rx_seq=119 vision=Lost color=3 model=initial age=700 vf=0.000 vl=0.000 stop_requested=1 stop_confirmed=0 elapsed=6000 recovery_used=2 recovery_count=0 recovery_left_ms=na';
  const pending=formatTerminalLine(raw);
  assert.match(pending,/故障 · 抓取任务.*模式：视觉取放.*视觉暂停.*原因：恢复次数耗尽/);
  assert.match(pending,/停止：已请求，等待确认/);
  assert.doesNotMatch(pending,/像素偏差|缺少参数|参考状态|接收序号|已恢复次数|设备报告剩余/);
  assert.match(formatTerminalLine(raw.replace('state=vision_pause','state=error').replace('stop_confirmed=0','stop_confirmed=1')),/停止：该次已确认/);
});

test('missing parameters remain visible and reference changes explain how to restore the gate',()=>{
  const missing='OK grab state=idle mode=pick reason=config_missing missing=ref_u,ref_v,j00,j01,j02,j10,j11,j12 ref_cause=base_place_turn stop_requested=0 stop_confirmed=0 elapsed=6400';
  const summary=formatTerminalLine(missing);
  assert.match(summary,/原因：缺少参数.*缺少参数：ref_u,ref_v,j00,j01,j02,j10,j11,j12/);
  assert.match(summary,/参考状态：Base 已转向放置处.*请重新建立三轴参考/);
  for(const cause of ['manual_home_all','manual_zero_x','manual_base_move','x_reference_changed']) {
    assert.match(formatTerminalLine(missing.replace('base_place_turn',cause)),/参考状态：.*请重新建立三轴参考/);
  }
  const ordinary=formatTerminalLine('OK grab state=idle mode=pick reason=config_missing missing=z_ppm ref_cause=home_verified');
  assert.match(ordinary,/缺少参数：z_ppm/);
  assert.doesNotMatch(ordinary,/参考状态|重新建立三轴参考/);
});

test('reference-invalid status remains explicit even with no missing parameter list',()=>{
  const summary=formatTerminalLine('OK grab state=idle mode=align reason=reference_invalid missing=none ref_cause=base_reference_changed stop_requested=0 stop_confirmed=0');
  assert.match(summary,/原因：三轴参考失效.*参考状态：Base 参考失效.*请重新建立三轴参考/);
  assert.doesNotMatch(summary,/缺少参数|已请求停止|停止已确认/);
});

test('grab parameter reads and route reports retain their existing detail display',()=>{
  assert.match(formatTerminalLine('OK grab get x_ppm=80'),/参数读取.*x_ppm=80/);
  assert.match(formatTerminalLine('ERR grab route state=error segment=2 reason=display_failed elapsed_ms=1400'),/路线.*路段：2.*屏幕.*失败.*耗时\(ms\)：1400/);
});

test('fault reasons are Chinese across modules while raw replies and numbers stay intact',()=>{
  for(const [raw,want]of [
    ['OK grab state=error reason=matrix_rank missing=none',/视觉响应矩阵.*无效/],
    ['OK grab state=stopping reason=axis_disabled_or_stall missing=none',/机械臂.*失能.*堵转/],
    ['OK grab state=stopping reason=motor_ack_timeout missing=none',/电机.*应答超时/],
    ['ERR turntable reference_required action=index',/转盘.*参考.*分度/],
    ['OK chassis status reason=anchor_invalid',/底盘.*地图起点参考.*无效/],
    ['OK imu status reason=drift_or_noise',/IMU.*漂移.*噪声/],
    ['OK bus status reason=rx_restart_failed addr=7',/总线.*接收.*失败.*7/],
    ['ERR manual axis_busy_or_fault addr=6 command_not_sent',/手动操作.*运动.*故障.*6.*未发送/],
    ['ERR grab track pickup height/speed; inspect z_grab z_lift z_pick_speed',/夹取高度.*下降速度.*检查.*提起高度/],
    ['OK turntable status state=idle reason=home_complete',/转盘.*回零完成/],
  ]){
    const translated=formatTerminalLine(raw);assert.match(translated,want);
    const reason=raw.match(/reason=(\w+)/)?.[1];if(reason)assert.ok(!translated.includes(reason));
    assert.equal(visibleTerminalLogs([{direction:'RX',text:raw}],false,false)[0].text,raw);
  }
});

test('rejection instructions explain status queries in Chinese without inventing a cause',()=>{
  const text=formatTerminalLine('ERR grab stack rejected; inspect grab status');
  assert.match(text,/码垛.*请求被拒绝.*读取.*抓取任务状态/);
  assert.match(text,/视觉任务.*点击“读取抓取任务状态”.*“抓取任务状态”.*原因/);
  assert.match(formatTerminalLine('ERR grab pose rejected; inspect grab status'),/姿态准备.*请求被拒绝.*视觉任务.*点击“读取抓取任务状态”/);
  assert.doesNotMatch(text,/rejected|inspect|\bbusy\b|回零|总线故障/);
  assert.match(formatTerminalLine('ERR grab_busy use grab status or grab stop'),/抓取任务.*忙.*读取.*停止/);
  assert.match(formatTerminalLine('ERR camera usb_unconfigured'),/相机.*USB.*未枚举/);
  assert.doesNotMatch(formatTerminalLine('OK turntable status state=idle reason=home_complete'),/三轴|四轮/);
});

test('negative Base boot reports opening before direction home',()=>{
  const banner='Boot: Z/X collision home + ID8 near home in parallel -> gripper open 2500us -> Base negative direction home -> IMU verify=5s; enable_verify=on';
  assert.match(formatTerminalLine(banner),/X\/Z 完成后开爪2500.*Base负向回零.*逆时针/);
});

test('turntable degraded reference remains explicit in status',()=>{
  const line='OK turntable status state=arrived ref=1 slot=1 arrived=1 moving=0 fresh=1 angle_deg=0 target_deg=0 speed_rpm=0 reason=home_approximate ref_degraded=1';
  assert.match(formatTerminalLine(line),/近似回零/);
  assert.match(formatTerminalLine(line),/近似参考标志/);
});


test('mission status translates the adopted alignment mode',()=>{
 assert.match(formatTerminalLine('OK grab route state=paused mode=mission align=raised'),/工位视觉对准.*高位.*Z=0/);
 assert.match(formatTerminalLine('OK grab route state=paused mode=mission align=lowered'),/工位视觉对准.*下降.*工位高度/);
});

test('recoverable bus conditions display warnings',()=>{
 assert.match(formatTerminalLine('WARN bus rx_restarted count=2 uart_error=0x00000002 tasks_interrupted=0'),/警告.*接收已恢复/);
 assert.match(formatTerminalLine('WARN bus sync_ack_ignored mask=0x01 zero_cache_acked=1 wheels_stopped=1'),/警告.*同步应答异常/);
});
