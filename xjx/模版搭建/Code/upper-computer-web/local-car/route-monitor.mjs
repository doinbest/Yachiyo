/** Records only coordinates validated by Telemetry in the same confirmed-stop frame. */
export class RouteStopHistory {
  stops=[];
  confirmations=new Map();
  observe(telemetry){
    const frame=telemetry.state;
    if(!frame)return;
    const motion=telemetry.route?.segment>0?telemetry.route:telemetry.distance;
    if(!motion||motion.action_id===0)return;
    const key=JSON.stringify([frame.localization?.generation??frame.trace_seq,motion.action_id,motion.segment??0,motion.phase,motion.phase_target??motion.target]);
    if(!motion.stop_confirmed){this.confirmations.delete(key);return;}
    if(!telemetry.feedback||this.confirmations.has(key))return;
    this.confirmations.set(key,true);
    this.stops.push({segment:motion.segment??0,phase:motion.phase,state:motion.state,reason:motion.reason,
      target:[...(motion.phase_target??motion.target)],speed:motion.speed_mm_s,limit:motion.phase_speed_mm_s,
      pose:{...telemetry.feedback},time:frame.t_ms,generation:frame.localization?.generation??frame.trace_seq});
  }
}
