const STATES={idle:'空闲',preparing:'配置雷达',wait_wrap:'等待完整圈起点',collecting:'采集一圈',ready:'完整圈已采集',stopped:'已停止扫描',error:'异常',running:'行驶中',stopping:'等待停车确认',waiting:'到点停留',station:'工位交接',done:'路线完成',cancelled:'已取消'};
const REASONS={idle:'等待操作',none:'无',configuring:'正在配置雷达',operator_stop:'人工停止',complete:'完整一圈成功',collecting:'采集一圈',waiting_wrap:'等待角度回绕',angle_resync:'角度重新同步',unit_changed:'距离单位变化，重新同步',rx_resync:'接收间隔恢复，重新同步',frame_resync:'报文重新同步',command_failed:'雷达配置命令失败',scan_timeout:'完整圈采集超时',scan_stop_requested:'已请求停止扫描',stationary_capture_required:'底盘须停稳后扫描',scan_accepted:'扫描已受理',scan_busy_or_parameters:'扫描正忙或参数不适用',plan_ready:'设备规划完成',leg_unreachable:'路段不可达',no_complete_map:'尚无完整地图',parameter_updated_ram:'RAM参数已更新，下次扫描使用',parameter_range_or_key:'参数名或范围不正确',snapshot_changed_or_page_range:'快照已更新或页码无效',navigation_stop_requested:'已请求取消路线',scan_in_progress:'正在扫描，请等完整圈结束',map_origin_or_parameters_changed:'原点或参数已变化，请重新扫描',actuator_busy:'机构正忙，请等当前动作结束',navigation_accepted:'路线执行已受理',navigation_or_grab_not_ready:'底盘或抓取准备未完成',cancelled:'人工取消',feedback_invalid:'位置反馈不可用',route_complete:'全部路线到点',station_handoff:'等待工位动作交接',auto_dwell:'停稳后短暂停留',operator_next_required:'等待手动下一段',arrival_unconfirmed:'到点尚未确认',action_mismatch:'底盘动作编号不匹配',position_correcting:'修正到点误差',position_not_reached:'位置尚未到达',feedback_arrived:'位置反馈确认到点',stop_unconfirmed:'停车尚未确认',bus_fault:'电机总线故障',stop_failed:'停车失败',stop_rejected:'停车命令未受理',sync_cache_uncertain:'同步状态待恢复',running:'行驶中',stopping:'正在停车',anchor_invalid:'地图起点参考无效',imu_invalid:'IMU尚未就绪',bus_unavailable:'电机总线不可用',feedback_moving:'底盘仍在运动'};
export const radarStateText=value=>STATES[value]??value??'—';
const PROBE_REASONS={not_run:'尚未执行通信验证',probe_accepted:'序列号查询已受理，等待发送及雷达回复',probe_busy_or_uart:'雷达串口正忙或尚未初始化，请等当前命令结束',query_in_progress:'正在验证通信，请等查询结束再扫描',query_sending:'发送 LUUIDH 序列号查询',query_waiting:'等待 PRODUCT SN 身份回复',sn_reply:'已收到序列号回复',alarm_only:'只收到报警，查询应答未确认',reply_unknown:'收到数据，但未确认完整身份回复',no_reply:'查询窗口内没有收到数据',tx_busy:'USART2 持续忙，查询未发出',tx_timeout:'USART2 发送完成超时',tx_error:'USART2 拒绝发送查询'};
const START_REASONS={start_ambiguous:'上下起点都缺少明确区分，请人工选择',start_both_conflict:'上下候选区域都有冲突，请人工选择',start_unresolved:'起点尚未确定，请人工选择',upper_start_plan_only:'上起点仅支持建图与规划，暂不执行实车路线',start_selected:'起点已选择，已复用本圈建图与规划',start_selection_unavailable:'尚无可复用的自动扫描完整圈，请先扫描',clear_regions:'必空区域条件唯一匹配',ambiguous:'上下候选无法区分，请人工选择',both_conflict:'上下候选区域都有冲突，请人工选择',manual_selection:'人工选择起点',manual_reference:'当前下起点参考'};
export const radarReasonText=value=>START_REASONS[value]??PROBE_REASONS[value]??REASONS[value]??value??'—';
export function radarStartText(start,{pending=false,historical=false}={}){
  if(pending)return '起点结果：扫描中，等待本圈识别结果';
  if(!start)return '起点结果：尚未读取';
  const selected=({0:'未确定',1:'右上起点',2:'右下起点'})[start.selected]??'未确定';
  return `${historical?'历史结果 · ':''}起点：${selected} · ${start.reason==='manual_selection'?'人工选择':start.auto?'自动识别':'下起点参考'} · ${radarReasonText(start.reason)}${start.selected===1?' · 仅规划':''}`;
}
export function radarProbeText(p){
  if(!p||p.state==='idle')return '尚未验证雷达通信';
  const state=p.state==='confirmed'&&p.tx===true?'往返通信已确认':({transmitting:'查询发送中',waiting:'等待雷达查询回复',unconfirmed:'查询应答未确认',error:'查询发送异常',cancelled:'通信验证已取消'})[p.state]??'查询结果未确认';
  return `${p.historical?'历史结果 · ':''}${state} · MCU USART2 ${p.tx===true?'发送完成':'发送未完成'} · ${radarReasonText(p.reason)}${p.sn?` · SN ${p.sn}`:''}`;
}
