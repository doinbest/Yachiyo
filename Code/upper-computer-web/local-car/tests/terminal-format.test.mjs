import {bootProgress,describeStopEvent,replyIsFault} from '../terminal-format.mjs';
import {visibleTerminalLogs} from '../terminal-log.mjs';
import test from 'node:test';
import assert from 'node:assert/strict';
import {formatTerminalLine,describeReply} from '../protocol.mjs';

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
  assert.equal(bootProgress('STM32 mechanical arm console ready',''),'正在回零');
  assert.match(bootProgress('OK grab state=homing reason=home_x','正在回零'),/X.*回零/);
  assert.match(bootProgress('OK grab state=idle reason=home_complete','正在回零'),/等待.*验证/);
  assert.match(bootProgress('[IMU CAL] state=VERIFYING elapsed_s=2','等待验证'),/正在验证/);
  assert.equal(bootProgress('[IMU CAL] result=PASS state=DONE verified=1 control_ready=1','正在验证'),'已就绪');
  assert.match(bootProgress('OK grab state=error reason=home_timeout','正在回零'),/失败.*home_timeout/);
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
  assert.match(formatTerminalLine('ERR grab route state=error segment=2 reason=display_failed elapsed_ms=1400'),/路线.*路段：2.*display_failed.*耗时\(ms\)：1400/);
});
