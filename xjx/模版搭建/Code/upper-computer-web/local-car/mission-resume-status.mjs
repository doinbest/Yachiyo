import {reasonText} from './device-text.mjs';

// Parking proof and permission to resume a route are separate facts.
export function missionResumeStatus(snapshot){
  if(snapshot?.state==='imu_recovery_wait'){
    const stopped=String(snapshot.stop_confirmed)==='1';
    return stopped
      ?{label:'已停稳，等待IMU定位恢复',detail:`本轮阶段与库存保留 · ${reasonText(snapshot.reason)}。等待新鲜IMU、定位及四轮反馈；通过后保持暂停，再显式继续。`}
      :{label:'IMU异常，停车待确认',detail:`本轮阶段与库存保留 · ${reasonText(snapshot.reason)}。正在核验真实停车，暂不能继续。`};
  }
  if(snapshot?.state==='paused' && snapshot.reason==='imu_recovered')
    return {label:'IMU已恢复，比赛暂停',detail:'IMU、原定位参考及新鲜四轮反馈已核验，本轮进度保留；点击“继续当前阶段”才恢复运动。'};
  const waiting=['resume_stop_wait','resume_bus_wait','resume_origin_invalid','resume_feedback_wait','resume_speed_wait','station_reference_changed'];
  if(snapshot?.state!=='stop_wait' && !(snapshot?.state==='resuming' && waiting.includes(snapshot.reason)))return null;
  const stopped=String(snapshot.stop_confirmed)==='1',reason=snapshot.reason;
  if(!stopped)return {label:'停车待确认',detail:`停车待确认 · ${reasonText(reason)}。处理当前故障后继续，重新核验停稳。`};
  if(reason==='resume_origin_invalid')return {label:'已停稳，定位参考失效',detail:'已停稳，定位参考失效；需恢复实际位置参考，不能把途中位置重新标为比赛起点。'};
  if(reason==='station_reference_changed')return {label:'已停稳，定位参考发生变化',detail:'已停稳，定位参考或位置单位发生变化，需处理参考变化后续接。'};
  if(reason==='feedback_invalid' || reason==='resume_feedback_wait' || reason==='resume_speed_wait')return {label:'已停稳，等待定位反馈恢复',detail:`已停稳，等待定位反馈恢复 · ${reasonText(reason)}。本轮进度保留。`};
  return {label:'已停稳，续接待处理',detail:`已停稳，续接待处理 · ${reasonText(reason)}。本轮进度保留。`};
}
