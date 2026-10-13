/* Exercise the production state machine; only device/time/console endpoints are fake. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_bus.h"
#include "../template/App/GrabTask.c"

static QR_SnapshotTypeDef qr_snapshot;
void QR_SnapshotGet(QR_SnapshotTypeDef *s){*s=qr_snapshot;}
static uint32_t clock_ms;
static MechanicalArm_EventTypeDef pending;
static bool pending_valid, move_arrives = true, camera_valid = true, imu_valid = true;
static float physical[3], destination[3];
static unsigned positions, closures, openings, stops, camera_requests, velocity_requests;
static unsigned grip_custom_calls;
static float grip_custom_duty;
static MechanicalArm_AxisTypeDef last_stop_axis;
static uint16_t z_position_rpm;
static uint8_t z_position_acc;
static uint8_t x_position_acc, configured_x_acc=20, configured_z_acc=120;
static int32_t z_position_pulses;
static Camera_SnapshotTypeDef camera;
static Camera_ColorTypeDef requested_color;
static HAL_StatusTypeDef camera_tx_result=HAL_OK;
static unsigned camera_attempts;
static unsigned camera_stops;
static Mecanum_Status_t bus;
static int64_t wheel_position[4];
static uint32_t wheel_feedback_delay;
static uint32_t wheel_position_time[4];
static bool wheel_moving, base_arrives=true, base_stale=false, base_mode2_absolute=false;
static unsigned base_moves, homes, aborts;
static unsigned test_homed_mask;
static bool abort_stays_running;
static uint8_t home_flags=3, base_state_flags=3;
static bool pixel_center_override;
static uint16_t pixel_center_x,pixel_center_y;
static int32_t measured_base_rpm;
static unsigned speed_reads;
static uint8_t home_axis_flags[3]; /* 0xff keeps the shared legacy fixture. */
static uint8_t linear_state_flags[2]={3,3};
static unsigned enables[3], recovery_enables, disabled_seen;
static unsigned home_disabled_axis=1;
static int home_disabled_reads;
static bool pre_enable_succeeds=true, recovery_enable_succeeds=true;
static MechanicalArm_ResultTypeDef recovery_enable_result=MECHANICAL_ARM_RESULT_OK;
static unsigned boot_verify_calls;
static MechanicalArm_ResultTypeDef home_submit_result;
static MotorBus_Diagnostic_t home_bus_diag;
static char home_error_log[512];
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *out) { *out=home_bus_diag; }
static bool wheel_stale;
static unsigned feedback_selected, selection_busy;
static char output[2048], camera_log[320], progress_log[320], accepted_log[128];
static unsigned camera_logs;
static Turntable_Inventory_t tray_inventory[3];
static Turntable_Status_t tray_status;
static bool tray_ready=true, tray_arrives=true, tray_moving;
static unsigned tray_indexes, tray_stops, tray_verifies, tray_angle_moves;
static uint8_t tray_unset_slot;
Turntable_Inventory_t Turntable_InventoryGet(uint8_t slot)
{ Turntable_Inventory_t unknown={TURNTABLE_UNKNOWN,0}; return slot>=1 && slot<=3?tray_inventory[slot-1]:unknown; }
bool Turntable_InventorySet(uint8_t slot,Turntable_InventoryState_t state,uint8_t color)
{ if(slot<1 || slot>3) return false; tray_inventory[slot-1].state=state; tray_inventory[slot-1].color=color; return true; }
uint8_t Turntable_FindEmpty(void)
{ unsigned i; for(i=0;i<3;i++) if(tray_inventory[i].state==TURNTABLE_EMPTY) return (uint8_t)(i+1); return 0; }
bool Turntable_ReferenceValid(void) { return tray_ready; }
bool Turntable_SlotConfigured(uint8_t slot) { return slot>=1 && slot<=3 && slot!=tray_unset_slot; }
bool Turntable_Index(uint8_t slot)
{
  if((TrackPickArmed && TrackStoreSlot && tracking_mode()) ||
     (Status.state!=GRAB_INDEX && (Operation==OP_CAR_PICK || ProcessedStoreActive))){
    assert(fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<2.01f);
  }else{
    assert(fabsf(physical[0]-Config[C_X_MIN])<.11f);
    assert(physical[1]>=Config[Operation==OP_STORE?lift_key():C_Z_CAR_LIFT]-.11f);
  }
  assert(!tray_moving); tray_indexes++; tray_status.slot=slot;tray_status.target_deg=(slot-1U)*90.0f;
  tray_status.arrived=false; tray_status.reason="indexing"; tray_moving=true; return true;
}
bool Turntable_SlotAngleGet(uint8_t slot,float *out)
{ if(!out || !Turntable_SlotConfigured(slot))return false;*out=(slot-1U)*90.0f;return true; }
bool Turntable_AngleStart(float angle,uint8_t slot,bool positive_half_turn)
{
  assert(!tray_moving && physical[1]>=ProcClearance.lift_z-Config[C_POS_TOL]);
  assert(slot==3 || (slot==0 && positive_half_turn));
  tray_angle_moves++;tray_status.slot=slot;tray_status.target_deg=angle;
  tray_status.arrived=false;tray_status.reason="indexing";tray_moving=true;return true;
}
bool Turntable_AngleVerify(float angle,uint8_t slot)
{
  assert(!tray_moving);tray_verifies++;tray_status.slot=slot;tray_status.target_deg=angle;
  tray_status.feedback_valid=true;
  tray_status.arrived=fabsf(tray_status.angle_deg-angle)<=2.5f;
  tray_status.reason=tray_status.arrived?"arrived":"position_changed";return true;
}
bool Turntable_VerifySlot(uint8_t slot){tray_verifies++;assert(slot==tray_status.slot);tray_status.feedback_valid=true;return true;}
bool Turntable_IsBusy(void) { return tray_moving; }
void Turntable_Stop(void) { tray_stops++; tray_moving=false; tray_status.moving=false; tray_status.reason="stopped"; }
void Turntable_StatusGet(Turntable_Status_t *out) { *out=tray_status; out->moving=tray_moving; }
uint32_t HAL_GetTick(void) { return clock_ms; }
#ifndef GRAB_REAL_CALIBRATION_SERVICE
void GrabCalibration_Init(void) {}
GrabCalibration_ResultTypeDef GrabCalibration_Start(Camera_ColorTypeDef color)
{ (void)color;return GRAB_CAL_RESULT_OK; }
void GrabCalibration_Process(void) {}
uint8_t GrabCalibration_MotorEventHandle(const MechanicalArm_EventTypeDef *event)
{ (void)event;return 0; }
void GrabCalibration_Cancel(void) {}
GrabCalibration_StateTypeDef GrabCalibration_StateGet(void) { return GRAB_CAL_STATE_COLLECT; }
const char *GrabCalibration_StateNameGet(void) { return "collect"; }
const char *GrabCalibration_ErrorNameGet(void) { return "none"; }
uint8_t GrabCalibration_DataGet(GrabCalibration_CalibrationDataTypeDef *data) { (void)data;return 0; }
#endif
static bool chassis_pose_route, chassis_pose_motion;
bool ChassisRoute_IsBusy(void) { return chassis_pose_route; }
bool ChassisMotion_IsBusy(void) { return chassis_pose_motion; }
bool Mecanum_IsBusy(void) { return false; }
void Mecanum_TimingGet(Mecanum_Timing_t *out) { memset(out,0,sizeof *out); }
void Mecanum_TrackingTiming(bool enabled,uint32_t interval_ms) { (void)enabled;(void)interval_ms; }
bool HWT101_Cal_IsBusy(void) { return false; }
void HWT101_Cal_BootVerifyArm(void) { assert(!BootActive && homes==3); boot_verify_calls++; }
uint8_t MechanicalArm_IsBusy(void) { return pending_valid; }
static MechanicalArm_ResultTypeDef queue(MechanicalArm_ActionTypeDef action, MechanicalArm_AxisTypeDef axis)
{
  if (pending_valid) return MECHANICAL_ARM_RESULT_BUSY;
  memset(&pending, 0, sizeof pending); pending.Action = action; pending.Axis = axis;
  pending.Result = MECHANICAL_ARM_RESULT_OK; pending.Transmitted = pending.Acknowledged = 1;
  pending_valid = true; return MECHANICAL_ARM_RESULT_NONE;
}
MechanicalArm_ResultTypeDef MechanicalArm_Enable(MechanicalArm_AxisTypeDef axis)
{
  unsigned i=axis==MECHANICAL_ARM_AXIS_BASE?2:axis==MECHANICAL_ARM_AXIS_X?0:1;
  MechanicalArm_ResultTypeDef result=queue(MECHANICAL_ARM_ACTION_ENABLE,axis);
  if(result==MECHANICAL_ARM_RESULT_NONE) {
    enables[i]++;
    if(BootHomeSent) recovery_enables++;
  }
  return result;
}
MechanicalArm_ResultTypeDef MechanicalArm_Home(MechanicalArm_AxisTypeDef axis,uint8_t mode)
{
  if(home_submit_result!=MECHANICAL_ARM_RESULT_NONE)return home_submit_result;
  assert(axis==BootAxes[homes] && mode==(axis==MECHANICAL_ARM_AXIS_BASE?1:2));
  homes++; test_homed_mask|=1U<<axis; return queue(MECHANICAL_ARM_ACTION_HOME,axis);
}
MechanicalArm_ResultTypeDef MechanicalArm_HomeStateRead(MechanicalArm_AxisTypeDef axis)
{ return queue(MECHANICAL_ARM_ACTION_HOME_STATE,axis); }
MechanicalArm_ResultTypeDef MechanicalArm_HomeAbort(MechanicalArm_AxisTypeDef axis)
{ aborts++; if(!abort_stays_running) { home_flags=3;home_axis_flags[axis]=3; } return queue(MECHANICAL_ARM_ACTION_HOME_ABORT,axis); }
MechanicalArm_ResultTypeDef MechanicalArm_PositionRead(MechanicalArm_AxisTypeDef axis)
{ return queue(MECHANICAL_ARM_ACTION_POSITION_READ, axis); }
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef axis)
{ return queue(MECHANICAL_ARM_ACTION_STATE, axis); }
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis)
{ pending_valid = false; stops++;last_stop_axis=axis; return queue(MECHANICAL_ARM_ACTION_STOP, axis); }
MechanicalArm_ResultTypeDef MechanicalArm_StopScheduled(MechanicalArm_AxisTypeDef axis)
{ if(pending_valid)return MECHANICAL_ARM_RESULT_BUSY; stops++;last_stop_axis=axis; return queue(MECHANICAL_ARM_ACTION_STOP,axis); }
static uint16_t x_position_rpm;
static MechanicalArm_PositionModeTypeDef base_position_mode;
MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef axis, int32_t pulses,
 uint16_t rpm, uint8_t acc, MechanicalArm_PositionModeTypeDef mode)
{
  assert(rpm > 0); (void)acc;
  if(axis==MECHANICAL_ARM_AXIS_X) { x_position_rpm=rpm; x_position_acc=acc; }
  if(axis == MECHANICAL_ARM_AXIS_BASE) {
    assert(mode == MECHANICAL_ARM_POSITION_RELATIVE_CURRENT || mode == MECHANICAL_ARM_POSITION_ABSOLUTE);
    base_position_mode=mode;
    if(Operation==OP_CAR_PICK && Status.state==GRAB_RETURN_TURN)
    {
      assert(CarPickClosed && physical[1]>=CarPickLiftTarget-.11f);
      assert(fabsf(physical[0]-(Config[C_X_MIN]+(CarPickSlot<=2U?30.0f:0.0f)))<.11f);
    }
    else assert(fabsf(physical[0]-Config[C_X_MIN])<.11f);
    if(pending_valid) return MECHANICAL_ARM_RESULT_BUSY;
    destination[2]=(mode==MECHANICAL_ARM_POSITION_ABSOLUTE || base_mode2_absolute?0:physical[2])+pulses*360.0f/3200; base_moves++; positions++;
    return queue(MECHANICAL_ARM_ACTION_POSITION,axis);
  }
  assert(mode == MECHANICAL_ARM_POSITION_ABSOLUTE ||
    (axis==MECHANICAL_ARM_AXIS_X && mode==MECHANICAL_ARM_POSITION_RELATIVE_CURRENT && (CalibrationTask && Status.state==GRAB_CALIBRATION)));
  if (pending_valid) return MECHANICAL_ARM_RESULT_BUSY;
  if(axis==MECHANICAL_ARM_AXIS_Z) { z_position_rpm=rpm; z_position_acc=acc; z_position_pulses=pulses; }
  unsigned idx=axis==MECHANICAL_ARM_AXIS_X?0:1;
  destination[idx]=(mode==MECHANICAL_ARM_POSITION_RELATIVE_CURRENT?physical[idx]:0)+pulses/100.0f;
  positions++; return queue(MECHANICAL_ARM_ACTION_POSITION, axis);
}
MechanicalArm_ResultTypeDef MechanicalArm_SpeedRead(MechanicalArm_AxisTypeDef axis)
{ speed_reads++;return queue(MECHANICAL_ARM_ACTION_SPEED_READ,axis); }
uint8_t MechanicalArm_ConfigGet(MechanicalArm_AxisTypeDef axis, MechanicalArm_ConfigTypeDef *out)
{ out->SpeedRpm=30; out->Acceleration=axis==MECHANICAL_ARM_AXIS_X?configured_x_acc:
    axis==MECHANICAL_ARM_AXIS_Z?configured_z_acc:20; out->PulseLimit=3200; return 1; }
void Arm_GripperSet(Arm_GripperStatusTypeDef status)
{ if (status == ARM_GRIPPER_CATCH) closures++; else if (status == ARM_GRIPPER_OPEN) openings++; }
void Arm_GripperDutySet(float duty){grip_custom_calls++;grip_custom_duty=duty;}
#ifndef GRAB_COMMAND_SIM
static char tracking_fault_log[160];
static char tracking_motor_fault_log[200];
static char tracking_estimate_log[320];
static char tracking_live_log[640];
static unsigned tracking_live_rows;
bool ConsoleTx_Tracking(const char *text,uint16_t size)
{ assert(size<sizeof tracking_live_log);memcpy(tracking_live_log,text,size);tracking_live_log[size]=0;tracking_live_rows++;return true; }
void ConsoleTx_TrackingCancel(void) { tracking_live_log[0]=0; }
bool ConsoleTx_Write(const uint8_t *text, uint16_t size)
{ assert(size < sizeof output); memcpy(output, text, size); output[size] = 0;
  if(strstr(output,"OK grab start")) { assert(size<sizeof accepted_log); memcpy(accepted_log,output,size+1); }
  if(strstr(output,"ERR grab home_request")) { assert(size<sizeof home_error_log); memcpy(home_error_log,output,size+1); }
  if(strstr(output,"OK grab track feedback_fault ")) { assert(size<sizeof tracking_fault_log); memcpy(tracking_fault_log,output,size+1); }
  if(strstr(output,"OK grab track motor_fault ")) { assert(size<sizeof tracking_motor_fault_log); memcpy(tracking_motor_fault_log,output,size+1); }
  if(strstr(output,"OK grab track estimate ")) { assert(size<sizeof tracking_estimate_log); memcpy(tracking_estimate_log,output,size+1); }
  return true; }
bool ConsoleTx_Debug(unsigned source,const char *text,uint16_t size)
{
  assert((source==CONSOLE_DEBUG_VISION || source==CONSOLE_DEBUG_CAMERA) && size<=320);
  if(source==CONSOLE_DEBUG_CAMERA) { assert(size<sizeof camera_log); memcpy(camera_log,text,size); camera_log[size]=0; camera_logs++; }
  if(source==CONSOLE_DEBUG_VISION) { memcpy(progress_log,text,size); progress_log[size]=0; }
  return true;
}
void ConsoleTx_DebugCancel(unsigned source) { assert(source==CONSOLE_DEBUG_CAMERA); }
#endif
HAL_StatusTypeDef Camera_RingStart(uint8_t ring)
{ return Camera_MaterialStart((Camera_ColorTypeDef)(9U+ring)); }
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef color)
{
  camera_attempts++;
  if(camera_tx_result!=HAL_OK) return camera_tx_result;
  requested_color=color; camera_requests++;
  camera.RequestActive=1; camera.RequestFunction=0xB2; camera.RequestTarget=color;
  camera.HasFrame=camera.HasValidData=camera.TargetValid=0; camera.InvalidCount=0;
  memset(&camera.Data,0,sizeof camera.Data);
  return HAL_OK;
}
void Camera_RequestStop(void) { camera_stops++; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out) { *out = camera; }
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *out)
{ memset(out, 0, sizeof *out); out->last_update_ms = clock_ms; return imu_valid; }
void Mecanum_Feedback_Enable(bool enabled) { (void)enabled; }
bool Mecanum_Feedback_Select(uint8_t selected)
{ if(selection_busy) { selection_busy--; return false; } feedback_selected=selected; return true; }
bool Mecanum_CanStopCleanly(void) { return !pending_valid; }
bool Mecanum_Test_Stop(void)
{ bus.stage = MECANUM_STAGE_STOPPED; bus.tx_complete = true; bus.sent_ms = clock_ms; return true; }
void Mecanum_StatusGet(Mecanum_Status_t *out) { *out = bus; }
static uint8_t wheel_state_flags[4]={3,3,3,3};
bool Mecanum_FeedbackGet(unsigned wheel, Mecanum_Feedback_t *out)
{
  assert(wheel < 4); memset(out, 0, sizeof *out);
  if(feedback_selected && wheel+1!=feedback_selected) return false;
  out->speed_valid = out->position_valid = out->state_valid = true;
  out->speed_ms = out->position_ms = out->state_ms = clock_ms-wheel_feedback_delay;
  if(wheel_position_time[wheel])out->position_ms=wheel_position_time[wheel];
  if(wheel_stale) out->speed_ms = out->position_ms = out->state_ms = clock_ms-3000U;
  out->position_raw=wheel_position[wheel];
  out->speed_rpm = wheel_moving ? 30 : 0; out->state_flags = wheel_state_flags[wheel]; return true;
}
bool Mecanum_Velocity_Request(float f, float l, float yaw, uint8_t acc)
{ (void)f; (void)l; (void)yaw; (void)acc; velocity_requests++; bus.stage = MECANUM_STAGE_SENT; return true; }

static void setting(const char *key, const char *value)
{
  char *args[] = {"grab", "set", (char *)key, (char *)value};
  assert(GrabTask_Command(4, args)); if(strstr(output,"ERR"))fprintf(stderr,"setting %s=%s failed: %s\n",key,value,output); assert(strstr(output, "ERR") == NULL);
}
static void reset(void)
{
  accepted_log[0]=progress_log[0]=0;
  home_error_log[0]=0;home_submit_result=MECHANICAL_ARM_RESULT_NONE;
  memset(&home_bus_diag,0,sizeof home_bus_diag);home_bus_diag.reason="none";
  clock_ms = 1000; pending_valid = false; positions = closures = openings = stops = 0;
  grip_custom_calls=0;grip_custom_duty=0;
  last_stop_axis=MECHANICAL_ARM_AXIS_ALL;
  memset(wheel_state_flags,3,sizeof wheel_state_flags);
  z_position_rpm=0; z_position_acc=0; z_position_pulses=0;
  configured_x_acc=20; configured_z_acc=120; x_position_acc=0;
  requested_color=CAMERA_COLOR_RED; camera_requests = velocity_requests = 0; wheel_moving = false;
  camera_tx_result=HAL_OK; camera_attempts=camera_stops=0;
  wheel_stale = false; base_arrives=true; base_stale=false; base_mode2_absolute=false; base_moves=0; homes=aborts=0; test_homed_mask=0; home_flags=3; base_state_flags=3; boot_verify_calls=0; camera_logs=0; camera_log[0]=0;
  memset(linear_state_flags,3,sizeof linear_state_flags);memset(enables,0,sizeof enables);
  recovery_enables=disabled_seen=0;home_disabled_reads=0;
  abort_stays_running=false;
  memset(home_axis_flags,0xff,sizeof home_axis_flags);
  measured_base_rpm=0;speed_reads=0;
  pixel_center_override=false;
  home_disabled_axis=1;
  pre_enable_succeeds=recovery_enable_succeeds=true;recovery_enable_result=MECHANICAL_ARM_RESULT_OK;
  feedback_selected = selection_busy = 0;
  move_arrives = camera_valid = imu_valid = true;
  memset(physical, 0, sizeof physical); memset(destination, 0, sizeof destination);
  memset(wheel_position,0,sizeof wheel_position); wheel_feedback_delay=0;
  memset(wheel_position_time,0,sizeof wheel_position_time);
  memset(&camera, 0, sizeof camera); memset(&bus, 0, sizeof bus);
  bus.ack_profile = MECANUM_ACK_RECEIVE; GrabTask_Init();
  tray_ready=tray_arrives=true; tray_moving=false; tray_indexes=tray_stops=tray_verifies=tray_angle_moves=0; tray_unset_slot=0;
  memset(&tray_status,0,sizeof tray_status); tray_status.reason="idle"; tray_status.feedback_valid=true;
  for(unsigned i=0;i<3;i++) { tray_inventory[i].state=TURNTABLE_EMPTY; tray_inventory[i].color=0; }
}
static void configure(bool vision)
{
  setting("z_ppm","100"); setting("z_min","-20"); setting("z_max","20");
  setting("z_place","-5"); setting("z_grab","0"); setting("z_lift","10"); setting("pos_tol","0.1");
  setting("stop_speed","0.5"); setting("close_ms","300"); setting("feedback_ms","2000");
  setting("x_ppm","100"); setting("x_min","-20"); setting("x_max","20");
  setting("z_car_lift","10");
  setting("x_pre","0"); setting("z_observe","10"); setting("age_ms","500");
  if (!vision) return;
  /* Legacy fixtures retain their original response; boot-default tests below
   * exercise the current tuning without these explicit overrides. */
  setting("lambda","0.3"); setting("body_speed","10"); setting("x_speed","5");
  setting("loss_grace_ms","0"); /* Legacy immediate-pause cases; grace has its own suite. */
  setting("accel","10"); setting("travel_mm","50"); setting("lead_mm","2");
  setting("dx_tol","2"); setting("dy_tol","2");
  setting("j00","1"); setting("j01","0"); setting("j02","0");
  setting("j10","0"); setting("j11","1"); setting("j12","1");
}
static void step(bool new_capture, unsigned phase, int dx, int dy)
{
  (void)phase;
  clock_ms += 50;
  if(tray_moving && tray_arrives) { tray_moving=false; tray_status.arrived=true; tray_status.angle_deg=tray_status.target_deg; tray_status.feedback_valid=true; tray_status.reason="arrived"; }
  if (pending_valid)
  {
    MechanicalArm_EventTypeDef event = pending; unsigned axis = event.Axis == MECHANICAL_ARM_AXIS_BASE ? 2 : (event.Axis == MECHANICAL_ARM_AXIS_X ? 0 : 1);
    pending_valid = false;
    if (event.Action == MECHANICAL_ARM_ACTION_POSITION && move_arrives && (axis!=2 || base_arrives)) physical[axis] = destination[axis];
    if (event.Action == MECHANICAL_ARM_ACTION_POSITION_READ)
      event.CurrentPositionRaw = (int64_t)lroundf(physical[axis] * (axis==2 ? 3200.0f/360 : 100.0f) * 65536.0f / 3200.0f) *
        ((axis == 0 && MECHANICAL_ARM_X_POSITIVE_DIR) || (axis==2 && MECHANICAL_ARM_BASE_POSITIVE_DIR) ? -1 : 1);
    if(event.Action==MECHANICAL_ARM_ACTION_SPEED_READ)event.CurrentSpeedRpm=measured_base_rpm;
    if(event.Action==MECHANICAL_ARM_ACTION_ENABLE) {
      if(BootHomeSent) {
        event.Result=recovery_enable_result;
        if(recovery_enable_succeeds && event.Result==MECHANICAL_ARM_RESULT_OK)home_disabled_reads=0;
      }
      if(pre_enable_succeeds) {
        if(axis==2)base_state_flags|=1U;else linear_state_flags[axis]|=1U;
      }
    }
    if(event.Axis!=MECHANICAL_ARM_AXIS_ALL) {
      uint8_t flags=axis==2?base_state_flags:linear_state_flags[axis];
      if(event.Action==MECHANICAL_ARM_ACTION_STATE && BootActive && BootHomeSent && axis==home_disabled_axis && home_disabled_reads) {
        flags&=(uint8_t)~1U;disabled_seen++;
        if(home_disabled_reads>0)home_disabled_reads--;
      }
      /* Each drive reports its own running bit; an unstarted drive is idle. */
      uint8_t drive_home=home_axis_flags[event.Axis]==0xff?home_flags:home_axis_flags[event.Axis];
      uint8_t homing=test_homed_mask&(1U<<event.Axis)?drive_home:(drive_home&~0x0cU);
      event.StateFlags[event.Axis]=event.Action==MECHANICAL_ARM_ACTION_HOME_STATE?homing:flags;
    }
    if(!(base_stale && axis==2)) (void)GrabTask_MotorEventHandle(&event);
  }
  camera.UsbConfigured=1; camera.LastFrameTick=clock_ms;
  camera.RequestActive=1; camera.HasFrame=1; camera.HasValidData=camera.TargetValid=camera_valid;
  if(!camera_valid) camera.InvalidCount++;
  camera.RequestFunction=0xB2; camera.RequestTarget=requested_color; camera.Data.Function=0xB2;
  camera.Data.Target = requested_color;
  if(new_capture) { camera.Data.Sequence++; camera.Data.Tick = clock_ms; }
  camera.Data.CX = (uint16_t)((RingTest?378:334) + dx); camera.Data.CY = (uint16_t)((RingTest?333:338) - dy);
  if(pixel_center_override) { camera.Data.CX=pixel_center_x;camera.Data.CY=pixel_center_y; }
  camera.Data.DX=(int16_t)dx;camera.Data.DY=(int16_t)dy;

#ifdef TEST_GRAB_BEFORE_PROCESS
  TEST_GRAB_BEFORE_PROCESS();
#endif
  GrabTask_Process();
}
static GrabTask_Status_t status(void) { GrabTask_Status_t s; GrabTask_StatusGet(&s); return s; }
static void run_to(GrabTask_State_t desired, unsigned limit, unsigned phase)
{
  while (status().state != desired && limit--) step(true, phase, 0, 0);
  if(status().state!=desired) fprintf(stderr,"want=%u state=%s reason=%s prep=%u pending=%u stop=%u z=%.2f target=%.2f t=%lu\n",
    desired,status().state_name,status().reason,PrepareStep,PendingAction,status().stop_confirmed,
    status().z_mm,status().z_target_mm,(unsigned long)clock_ms);
  assert(status().state == desired);
}
static void car_x_default_survives_reset_and_verified_home(void)
{
  reset();char *read[]={"grab","get","x_car"};
  assert(GrabTask_Command(3,read));assert(strstr(output,"ERR grab config"));
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,250,0);
  assert(GrabTask_ConfigReady("fixed"));
  assert(Config[C_Z_PROC_PLACE]==-130 && Config[C_Z_ROUGH_ALIGN]==-100);
}
static void missing_and_fixed(void)
{
  reset(); assert(!GrabTask_ConfigReady("fixed")); assert(!GrabTask_Start("fixed"));
  assert(!positions && !openings && !stops);
  configure(false); assert(GrabTask_Start("fixed"));
  run_to(GRAB_COMPLETE, 600, 0); assert(closures == 1 && openings == 2 && positions >= 9 && base_moves == 2);
  assert(!GrabTask_IsBusy()); assert(status().stop_confirmed);
  GrabTask_Stop(); assert(status().state==GRAB_COMPLETE && !GrabTask_IsBusy()); assert(closures == 1 && openings == 2);
}
static void ack_is_not_arrival(void)
{
  reset(); configure(false); setting("z_grab","-5"); move_arrives = false;
  assert(GrabTask_Start("fixed")); run_to(GRAB_DESCEND, 100, 0);
  { unsigned i; for (i = 0; i < 60; i++) step(true,0,0,0); }
  assert(status().state == GRAB_DESCEND && closures == 0);
  /* A slow/missing arrival must not fail only because 15 seconds elapsed. */
  { unsigned i; for (i = 0; i < 2500; i++) step(true,0,0,0); }
  assert(status().state == GRAB_DESCEND && closures == 0);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(closures == 0);
}
static void initial_boundary_uses_position_tolerance(void)
{
  reset(); configure(false); setting("z_max","0"); setting("z_car_lift","0"); setting("z_grab","-5"); setting("z_lift","-1");
  physical[1]=.05f; /* Small home-counter residual inside the 0.1 mm test tolerance. */
  assert(GrabTask_Start("fixed")); run_to(GRAB_DESCEND,100,0);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0);
  reset(); configure(false); setting("z_max","0"); setting("z_car_lift","0"); setting("z_grab","-5"); setting("z_lift","-1");
  physical[1]=.2f;
  assert(GrabTask_Start("fixed")); run_to(GRAB_ERROR,100,0);
  assert(!positions && !openings && !strcmp(status().reason,"initial_axis_limit"));
}
static void repeated_snapshot_and_success(void)
{
  unsigned i; reset(); configure(true); assert(GrabTask_Start("pick"));
  run_to(GRAB_ALIGN,160,1);
  for(i=0;i<4;i++) step(false,1,0,0);
  assert(status().state==GRAB_ALIGN && CaptureCount<=1 && !closures);
  run_to(GRAB_SETTLE,160,1);
  assert(!CaptureCount && !VisualStable && !closures);
  for(i=0;i<4;i++) step(false,1,0,0);
  assert(status().state==GRAB_SETTLE && !CaptureCount && !closures);
  run_to(GRAB_COMPLETE,600,1); assert(closures==1);
}
static void placement_reference_restored_before_restart(void)
{
  reset(); configure(true); assert(GrabTask_Start("pick"));
  run_to(GRAB_COMPLETE,700,1);
  assert(!strcmp(status().reference_cause,"transfer_return_verified") && GrabTask_ConfigReady("pick"));
  assert(!strcmp(status().missing,"none"));
  status_write(false);
  assert(strstr(output,"reason=stored missing=none ref_cause=transfer_return_verified"));
  assert(GrabTask_Start("pick")); /* No forced rehome after the owned round trip. */
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0);
}

static void invalid_and_cancel(void)
{
  char *nan[] = {"grab","set","z_ppm","nan"};
  reset(); configure(true); assert(GrabTask_Command(4,nan)); assert(strstr(output,"ERR"));
  assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,160,1);
  step(true,1,20,20); assert(velocity_requests > 0);
  camera_valid = false; step(false,1,0,0);
  assert(status().state == GRAB_VISION_PAUSE);
  run_to(GRAB_ERROR,320,1); assert(!closures);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1); assert(!closures);
}
static void solver_and_configuration(void)
{
  float v[3]; char *read[] = {"grab","get","x_ppm"};
  reset(); assert(GrabTask_Command(3,read)); assert(strstr(output,"value=unset"));
  configure(true); assert(GrabTask_ConfigReady("pick"));
  assert(solve(3,4,0,v));
  assert(fabsf(v[0]+.9f)<.001f && fabsf(v[1]+v[2]+1.2f)<.001f);
  assert(v[1]!=0 && v[2]!=0); /* Left and X jointly correct the same pixel direction. */
  assert(solve(3000,4000,19.9f,v));
  assert(hypotf(v[0],v[1])<=10.001f && fabsf(v[2])<=5.001f);
  setting("j11","0"); setting("j12","0");
  assert(!GrabTask_Start("pick") && positions==0);
}
static void direct_align_and_complete_missing_report(void)
{
  unsigned i; char *start[]={"grab","start","align"};
  reset(); configure(true);
  assert(GrabTask_Start("align")); /* No separate X test or opt-in flag. */
  run_to(GRAB_ALIGN,180,1);
  assert(camera_requests==1 && !strcmp(status().mode,"align"));
  for(i=0;i<20;i++) step(true,1,10,10);
  assert(velocity_requests && positions>2 && !closures && !base_moves);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1);
  reset(); GrabTask_BootHomeStart(); run_to(GRAB_IDLE,250,0);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(GrabTask_Command(3,start));
  assert(strstr(output,"mode=align reason=config_missing"));
  assert(strstr(output,"missing=j00,j01,j02,j10,j11,j12 "));
  assert(!positions && !camera_requests && !closures);
  /* Checking fixed readiness must not rewrite the rejected request's mode. */
  assert(GrabTask_ConfigReady("fixed") && !strcmp(status().mode,"align"));
}
static void boot_defaults_start_align_without_settings(void)
{
  float v[3]; unsigned before;
  reset(); GrabTask_BootHomeStart(); run_to(GRAB_IDLE,250,0);
  assert(GrabTask_ConfigReady("align"));
  /* 2026-09-29 bounded live probes: forward increases CX, left decreases CY.
   * At the retracted X bound, negative errors need these positive body axes.
   * Check isolated errors so swapping the body columns cannot pass. */
  assert(solve(-30,0,0,v) && v[0]>0 && fabsf(v[1])<1e-6f && fabsf(v[2])<1e-6f);
  /* Camera extends toward the same material as left body translation.
   * At zero X may extend, at maximum it may retract, never beyond either bound. */
  assert(solve(0,-20,0,v) && fabsf(v[0])<1e-6f && v[1]>0 && v[2]>0);
  assert(solve(20,20,0,v) && v[0]<0 && v[1]<0 && v[2]==0);
  assert(solve(-20,-20,0,v) && v[0]>0 && v[1]>0 && v[2]>0);
  assert(Config[C_X_MAX]==130);
  assert(solve(20,20,130,v) && v[0]<0 && v[1]<0 && v[2]<0);
  assert(solve(-20,-20,130,v) && v[0]>0 && v[1]>0 && v[2]==0);
  /* Interior X must help rather than cancel left translation. Check error
   * contraction against positive physical X gains, not the configured J. */
  for(unsigned sign=0;sign<2;sign++) {
    float dy=sign?20:-20;
    assert(solve(0,dy,50,v) && fabsf(v[0])<1e-6f && v[1]*v[2]>0);
    for(unsigned gain=1;gain<=4;gain*=2)
      assert(dy*(2*v[1]+gain*v[2])<0);
  }
  assert(GrabTask_Start("align"));
  run_to(GRAB_ALIGN,180,1);
  assert(camera_requests==1 && Axes[0].target==0 && Axes[1].target==0);
  assert(!closures && base_moves==1 && fabsf(physical[2]-130)<.15f);
  run_to(GRAB_HOLD,250,2);
  assert(!closures && base_moves==1 && !strcmp(status().reason,"color_aligned_low"));
  status_write(false); assert(strstr(output,"model=initial"));
  before=positions; GrabTask_Stop(); run_to(GRAB_IDLE,100,2); assert(positions==before);
}
static void freshness_and_horizontal_gate(void)
{
  unsigned i;
  reset(); configure(false); wheel_moving=true; assert(GrabTask_Start("fixed"));
  for(i=0;i<30;i++) step(true,0,0,0);
  assert(positions==0 && closures==0);
  GrabTask_Stop(); wheel_moving=false; run_to(GRAB_IDLE,100,0);
  reset(); configure(true); assert(GrabTask_Start("pick")); run_to(GRAB_ALIGN,160,1);
  /* Re-reading the same receive snapshot must not refresh its age. */
  for(i=0;i<12;i++) step(false,2,0,0);
  assert(status().state==GRAB_STOPPING || status().state==GRAB_ERROR);
  assert(closures==0);
  reset(); configure(false); assert(GrabTask_Start("fixed")); run_to(GRAB_DESCEND,100,0);
  wheel_stale=true; step(true,0,0,0);
  assert(status().state==GRAB_STOPPING && closures==0);
}
static void bounded_drift_retry(void)
{
  unsigned i; reset(); configure(true); assert(GrabTask_Start("pick"));
  run_to(GRAB_SETTLE,180,2);
  /* Several fresh drift frames during STOPPING are one attempt, not several retries. */
  for(i=0;i<3;i++) step(true,2,10,10);
  assert(status().state==GRAB_SETTLE && Retries==0);
  while(status().state==GRAB_SETTLE) step(true,2,10,10);
  assert(status().state==GRAB_ALIGN && Retries==1 && closures==0);
}
static void feedback_bounds(void)
{
  reset(); configure(true); assert(GrabTask_Start("pick")); run_to(GRAB_ALIGN,160,1);
  physical[0]=21; move_arrives=false;
  /* One full six-axis-read round interleaved with visual commands (50 ms/step).
   * Its phase changes with the preceding preparation motion. */
  { unsigned i; for(i=0;i<13;i++) step(true,1,10,10); }
  assert(status().state==GRAB_STOPPING || status().state==GRAB_ERROR);
  assert(closures==0);
}
static void fractional_x_and_all_wheels(void)
{
  unsigned i, before;
  reset(); configure(true); setting("j11","20"); setting("j12","20");
  setting("dy_tol","0.1"); feedback_selected=1; selection_busy=2;
  assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  assert(feedback_selected==0); before=positions;
  for(i=0;i<50;i++) step(true,2,0,1);
  assert(positions>before); /* Several sub-pulse corrections eventually produce a pulse. */
}
static void fractional_x_survives_alternating_bus_wait(void)
{
  unsigned i, j, before;
  reset(); configure(true); setting("j11","20"); setting("j12","20");
  setting("dy_tol","0.1");
  assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1); before=positions;
  for(i=0;i<20;i++)
  {
    /* An in-flight wheel batch owns IO: each blocked control
     * opportunity is followed by one free 100ms opportunity worth 0.105 pulse. */
    for(j=0;j<2;j++)
    { bus.stage=MECANUM_STAGE_WHEELS; bus.sent_ms=clock_ms+50; step(true,2,0,1); }
    bus.stage=MECANUM_STAGE_SENT;
    bus.sent_ms=clock_ms-1000;
    for(j=0;j<2;j++) step(true,2,0,1);
  }
  assert(positions>before);
  assert(fabsf(physical[0])<=.03f); /* Busy intervals were skipped, not caught up later. */
}
static void stale_wheels_cannot_confirm_stop(void)
{
  unsigned i;
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_HOLD,600,0);
  GrabTask_Stop();
  while(!StopEvidenceStarted) step(true,0,0,0);
  step(true,0,0,0); assert(WheelQuiet);
  wheel_stale=true;
  for(i=0;i<15;i++) step(true,0,0,0);
  assert(status().state==GRAB_STOPPING && !status().stop_confirmed);
}
static void placement_feedback_and_cancel(void)
{
  unsigned i, j;
  const GrabTask_State_t phases[]={GRAB_TURN,GRAB_PLACE,GRAB_RELEASE,GRAB_RETRACT};
  reset(); configure(false); physical[2]=37; assert(GrabTask_Start("fixed"));
  run_to(GRAB_COMPLETE,650,0);
  assert(fabsf(physical[2]-37)<.12f && physical[1]==0 && base_moves==2);
  assert(!strcmp(status().reason,"stored"));
  reset(); configure(false); base_arrives=false; assert(GrabTask_Start("fixed"));
  run_to(GRAB_TURN,200,0);
  for(i=0;i<40;i++) step(true,0,0,0);
  assert(status().state==GRAB_TURN && openings==1 && physical[1]==10 && base_moves==1);
  for(i=0;i<2500;i++) step(true,0,0,0);
  assert(status().state==GRAB_TURN && openings==1);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(openings==1);
  reset(); configure(false); assert(GrabTask_Start("fixed")); run_to(GRAB_PLACE,400,0);
  move_arrives=false;
  for(i=0;i<40;i++) step(true,0,0,0);
  assert(status().state==GRAB_PLACE && openings==1);
  for(i=0;i<2500;i++) step(true,0,0,0);
  assert(status().state==GRAB_PLACE && openings==1);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(openings==1);
  reset(); configure(false); assert(GrabTask_Start("fixed")); run_to(GRAB_TURN,200,0);
  base_stale=true; run_to(GRAB_STOPPING,100,0);
  for(i=0;i<2500;i++) step(true,0,0,0);
  assert(status().state==GRAB_ERROR && openings==1 && !status().stop_confirmed);
  assert(!strcmp(status().reason,"stop_unconfirmed"));
  for(j=0;j<sizeof phases/sizeof phases[0];j++) {
    unsigned before;
    reset(); configure(false); assert(GrabTask_Start("fixed")); run_to(phases[j],400,0);
    before=openings; GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(openings==before);
    before=positions; for(i=0;i<20;i++) step(true,0,0,0); assert(positions==before);
  }
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_HOLD,250,2);
  assert(!base_moves && !closures && openings==1);
  reset(); configure(false); setting("z_place","21"); assert(!GrabTask_Start("fixed"));
  setting("z_place","10"); assert(!GrabTask_Start("fixed"));
}
static void base_home_waits_for_other_origins(void)
{
  /* Either linear drive still moving must defer Base, while Z/X both start. */
  for(unsigned held=MECHANICAL_ARM_AXIS_Z;held<=MECHANICAL_ARM_AXIS_X;held++) {
    reset();home_axis_flags[held]=7;GrabTask_BootHomeStart();
    for(unsigned i=0;i<200;i++)step(false,0,0,0);
    assert(homes==2 && !enables[2] && !boot_verify_calls);
    home_axis_flags[held]=3;run_to(GRAB_IDLE,250,0);
    assert(homes==3 && boot_verify_calls==1);
  }
  /* A stopped linear drive outside its zero window also defers Base. */
  for(unsigned held=0;held<2;held++) {
    reset();physical[held]=2;GrabTask_BootHomeStart();
    for(unsigned i=0;i<200;i++)step(false,0,0,0);
    assert(homes==2 && !enables[2] && !boot_verify_calls);
    physical[held]=0;run_to(GRAB_IDLE,250,0);
    assert(homes==3 && boot_verify_calls==1);
  }
  /* The car tray must also finish before the loaded Base begins. */
  reset();tray_moving=true;tray_arrives=false;GrabTask_BootHomeStart();
  for(unsigned i=0;i<200;i++)step(false,0,0,0);
  assert(homes==2 && !enables[2] && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_wait_turntable"));
  tray_moving=false;run_to(GRAB_IDLE,250,0);
  assert(homes==3 && boot_verify_calls==1);

  reset();tray_moving=true;tray_arrives=false;GrabTask_BootHomeStart();
  for(unsigned i=0;i<200;i++)step(false,0,0,0);
  GrabTask_Stop();run_to(GRAB_IDLE,150,0);tray_moving=false;
  for(unsigned i=0;i<100;i++)step(false,0,0,0);
  assert(homes==2 && !enables[2] && !boot_verify_calls && tray_stops);

  reset();tray_ready=false;GrabTask_BootHomeStart();run_to(GRAB_ERROR,250,0);
  assert(homes==2 && !enables[2] && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_turntable_failed"));

  /* Base's motion timeout starts at its own HOME, after the tray wait. */
  reset();tray_moving=true;tray_arrives=false;GrabTask_BootHomeStart();
  for(unsigned i=0;i<900;i++)step(false,0,0,0);
  assert(homes==2 && status().state==GRAB_HOMING);
  tray_moving=false;home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=7;
  for(unsigned i=0;i<800;i++)step(false,0,0,0);
  assert(homes==3 && status().state==GRAB_HOMING && !boot_verify_calls);
  home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=3;run_to(GRAB_IDLE,150,0);
  assert(boot_verify_calls==1);

  /* Once Base starts, cancellation interrupts Base only, with no late restart. */
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=7;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250 && homes<3;i++)step(false,0,0,0);
  assert(homes==3 && !boot_verify_calls);
  GrabTask_Stop();run_to(GRAB_IDLE,150,0);
  for(unsigned i=0;i<100;i++)step(false,0,0,0);
  assert(homes==3 && aborts==1 && !boot_verify_calls);

  reset();setting("base_home_degrade","0");home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;GrabTask_BootHomeStart();
  run_to(GRAB_ERROR,250,0);
  assert(homes==3 && aborts==1 && !RearReferenceValid && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_driver_fault"));
}
static void boot_homing(void)
{
  unsigned i;
  reset(); physical[2]=360; base_state_flags=1; GrabTask_BootHomeStart(); assert(GrabTask_IsBusy() && !GrabTask_Start("fixed"));
  for(i=0;i<70 && !homes;i++)step(false,0,0,0);
  assert(homes==1);home_flags=7;
  for(i=0;i<70;i++) step(false,0,0,0);
  assert(homes==2 && status().state==GRAB_HOMING && !boot_verify_calls);
  /* Z/X start together; Base waits for their verified origins. */
  home_flags=3; physical[1]=2;
  for(i=0;i<20;i++) step(false,0,0,0);
  assert(homes==2 && !boot_verify_calls); /* Idle HOME flags alone are not verified origins. */
  physical[1]=0; run_to(GRAB_IDLE,250,0);
  assert(boot_verify_calls==1 && homes==3 && !strcmp(status().reason,"home_complete") && Config[C_Z_GRAB]==-50 && Config[C_Z_PLACE]==-40 && Config[C_Z_TEMP_ALIGN]==-100 && Config[C_Z_STACK_PLACE]==-100);
  assert(GrabTask_ConfigReady("fixed") && GrabTask_ConfigReady("align"));
  assert(!strcmp(status().missing,"none"));
  for(i=0;i<50;i++) step(false,0,0,0);
  assert(homes==3);
  reset(); GrabTask_BootHomeStart();
  for(i=0;i<70 && !homes;i++)step(false,0,0,0);
  home_flags=11; run_to(GRAB_ERROR,150,0);
  assert(homes==2 && aborts==2 && !boot_verify_calls && !strcmp(status().reason,"home_driver_fault"));
  reset(); GrabTask_BootHomeStart();
  for(i=0;i<70 && !homes;i++)step(false,0,0,0);
  home_flags=7;
  for(i=0;i<10;i++) step(false,0,0,0);
  unsigned issued=homes;
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(homes==issued && aborts==issued);
  for(i=0;i<50;i++) step(false,0,0,0);
  assert(homes==issued);
  reset(); GrabTask_BootHomeStart(); GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(!homes && !aborts && !boot_verify_calls);
  reset();GrabTask_BootHomeStart();home_flags=7;
  for(i=0;i<100 && homes<2;i++)step(false,0,0,0);
  assert(homes==2 && !boot_verify_calls);
  GrabTask_Stop();run_to(GRAB_IDLE,150,0);
  assert(aborts==2 && tray_stops && !boot_verify_calls);
  for(i=0;i<50;i++)step(false,0,0,0);
  assert(homes==2); /* No late Base HOME after cancellation. */

  reset();GrabTask_BootHomeStart();home_flags=7;
  for(i=0;i<100 && homes<2;i++)step(false,0,0,0);
  abort_stays_running=true;GrabTask_Stop();run_to(GRAB_ERROR,150,0);
  assert(aborts==2 && !boot_verify_calls && !strcmp(status().reason,"home_stop_unconfirmed"));
  /* A drive that fails to exit HOME cannot prevent aborts to the other drives. */

  reset();GrabTask_BootHomeStart();tray_moving=true;tray_arrives=false;
  for(i=0;i<250;i++)step(false,0,0,0);
  assert(homes==2 && !boot_verify_calls && status().state==GRAB_HOMING);
  assert(!strcmp(status().reason,"home_wait_turntable"));
  tray_moving=false;run_to(GRAB_IDLE,100,0);assert(boot_verify_calls==1);
  reset();GrabTask_BootHomeStart();tray_ready=false;
  run_to(GRAB_ERROR,250,0);
  assert(homes==2 && !boot_verify_calls && !strcmp(status().reason,"home_turntable_failed"));
  reset();tray_ready=false;
  char *manual[]={"grab","rehome"};
  assert(GrabTask_Command(2,manual));run_to(GRAB_IDLE,250,0);
  assert(homes==3 && boot_verify_calls==1); /* Manual three-axis recovery has no new ID8 dependency. */
}
static void base_home_keeps_full_motion_window_after_preparation(void)
{
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=7;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250 && (BootAxisIndex!=2 || BootStep!=8);i++)step(false,0,0,0);
  assert(homes==2 && BootAxisIndex==2 && BootStep==8);
  home_submit_result=MECHANICAL_ARM_RESULT_BUSY;
  for(unsigned i=0;i<80;i++)step(false,0,0,0);
  assert(homes==2 && !boot_verify_calls);
  home_submit_result=MECHANICAL_ARM_RESULT_NONE;
  for(unsigned i=0;i<20 && homes<3;i++)step(false,0,0,0);
  assert(homes==3);
  uint32_t issued=clock_ms;
  clock_ms=issued+59850U;step(false,0,0,0);
  assert(status().state==GRAB_HOMING && !stops && !boot_verify_calls);
  clock_ms=issued+60001U;run_to(GRAB_ERROR,150,0);
  assert(!strcmp(status().reason,"home_timeout") && aborts==1 && !boot_verify_calls);
}
static void boot_enable_and_disabled_feedback_recovery(void)
{
  reset();home_flags=0x0b;GrabTask_BootHomeStart();
  for(unsigned i=0;i<100 && !homes;i++)step(false,0,0,0);
  assert(homes==1 && enables[1]==1); /* A previous HOME failure cannot block a new HOME. */
  home_flags=3;run_to(GRAB_IDLE,250,0);assert(boot_verify_calls==1);

  reset();linear_state_flags[1]=2;GrabTask_BootHomeStart();
  for(unsigned i=0;i<90 && !homes;i++)step(false,0,0,0);
  assert(homes==1 && enables[1]==1 && !recovery_enables);
  run_to(GRAB_IDLE,250,0);
  assert(enables[0]==1 && enables[2]==1 && boot_verify_calls==1);

  reset();linear_state_flags[1]=2;pre_enable_succeeds=false;GrabTask_BootHomeStart();
  run_to(GRAB_ERROR,180,0);
  assert(!homes && enables[1]==1 && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_enable_failed")); /* ACK alone is not enabled feedback. */

  reset();home_disabled_reads=1;GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);
  assert(disabled_seen==1 && !recovery_enables && homes==3 && boot_verify_calls==1);

  for(unsigned axis=0;axis<3;axis++) {
    reset();home_disabled_axis=axis;home_disabled_reads=-1;physical[2]=360;GrabTask_BootHomeStart();
    run_to(GRAB_IDLE,350,0);
    assert(disabled_seen>=2 && recovery_enables==1 && enables[axis]==2);
    assert(homes==3 && boot_verify_calls==1); /* Re-enable does not restart HOME. */
    assert(fabsf(RearBase-360)<.1f); /* Base's saved multi-turn origin remains intact. */
  }

  reset();home_disabled_reads=-1;recovery_enable_succeeds=false;GrabTask_BootHomeStart();
  run_to(GRAB_ERROR,250,0);
  assert(homes==2 && recovery_enables==1 && !boot_verify_calls && isnan(Config[C_Z_PPM]));
  assert(!strcmp(status().reason,"home_enable_failed"));

  reset();home_disabled_reads=-1;recovery_enable_result=MECHANICAL_ARM_RESULT_ACK_TIMEOUT;GrabTask_BootHomeStart();
  run_to(GRAB_ERROR,250,0);
  assert(homes==2 && recovery_enables==1 && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_feedback_failed"));

  reset();home_disabled_reads=-1;GrabTask_BootHomeStart();physical[1]=2;
  for(unsigned i=0;i<120;i++)step(false,0,0,0);
  assert(homes==2 && !recovery_enables && !boot_verify_calls);
  physical[1]=0;run_to(GRAB_IDLE,350,0);assert(recovery_enables==1 && boot_verify_calls==1);

  reset();home_disabled_axis=2;home_disabled_reads=-1;physical[2]=360;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250 && !recovery_enables;i++)step(false,0,0,0);
  assert(recovery_enables==1);physical[2]+=GRAB_BASE_HOME_STABLE_TOL_DEG+1;run_to(GRAB_ERROR,150,0);
  assert(homes==3 && !boot_verify_calls && !RearReferenceValid);
  assert(!strcmp(status().reason,"home_position_changed"));

  reset();home_disabled_reads=-1;GrabTask_BootHomeStart();
  for(unsigned i=0;i<120 && !disabled_seen;i++)step(false,0,0,0);
  assert(disabled_seen && !recovery_enables);GrabTask_Stop();run_to(GRAB_IDLE,150,0);
  for(unsigned i=0;i<60;i++)step(false,0,0,0);
  assert(homes==2 && !recovery_enables && !boot_verify_calls);

  reset();home_disabled_reads=-1;GrabTask_BootHomeStart();
  for(unsigned i=0;i<150 && !recovery_enables;i++)step(false,0,0,0);
  assert(recovery_enables==1 && pending_valid);GrabTask_Stop();run_to(GRAB_IDLE,150,0);
  assert(homes==2 && recovery_enables==1 && !boot_verify_calls);

  reset();home_disabled_reads=-1;GrabTask_BootHomeStart();
  for(unsigned i=0;i<90 && !homes;i++)step(false,0,0,0);
  linear_state_flags[1]=10;run_to(GRAB_ERROR,180,0);
  assert(homes==2 && !recovery_enables && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_axis_fault"));

  reset();home_flags=0x23;GrabTask_BootHomeStart();run_to(GRAB_ERROR,150,0);
  assert(!homes && !enables[1] && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_driver_fault"));

  reset();linear_state_flags[1]=2;GrabTask_BootHomeStart();
  for(unsigned i=0;i<90 && !enables[1];i++)step(false,0,0,0);
  assert(enables[1]==1 && !homes && pending_valid);
  GrabTask_Stop();run_to(GRAB_IDLE,120,0);
  assert(!homes && enables[1]==1 && !boot_verify_calls);
}
static void boot_submit_wait_and_fault_diagnostics(void)
{
  reset();GrabTask_BootHomeStart();home_submit_result=MECHANICAL_ARM_RESULT_BUSY;
  for(unsigned i=0;i<70;i++)step(false,0,0,0);
  assert(status().state==GRAB_HOMING && !homes && !stops && !aborts && !boot_verify_calls);
  assert(!strcmp(status().missing,"home_pending"));
  home_submit_result=MECHANICAL_ARM_RESULT_NONE;run_to(GRAB_IDLE,250,0);
  assert(homes==3 && boot_verify_calls==1 && !strcmp(status().missing,"none"));

  reset();GrabTask_BootHomeStart();home_submit_result=MECHANICAL_ARM_RESULT_TX_ERROR;
  home_bus_diag.locked=true;home_bus_diag.reason="uart_error";
  home_bus_diag.uart_error=HAL_UART_ERROR_NE;home_bus_diag.tick_ms=29;
  home_bus_diag.owner=MOTOR_BUS_OWNER_COUNT;
  run_to(GRAB_ERROR,150,0);
  assert(!homes && !aborts && !boot_verify_calls);
  assert(!strcmp(status().reason,"home_bus_fault") && !strcmp(status().missing,"home_pending"));
  assert(strstr(home_error_log,"axis=z") && strstr(home_error_log,"result=tx_error"));
  assert(strstr(home_error_log,"bus_locked=1") && strstr(home_error_log,"bus_reason=uart_error"));
  assert(strstr(home_error_log,"uart_error=0x00000002") && strstr(home_error_log,"fault_ms=29"));
  status_write(false);assert(strstr(output,"missing=home_pending") && !strstr(output,"missing=x_ppm"));
}
static void base_home_requires_stable_feedback(void)
{
  unsigned i;
  reset(); GrabTask_BootHomeStart();
  for(i=0;i<250 && (BootLaunching || BootAxisIndex!=2);i++) step(false,0,0,0);
  assert(BootAxisIndex==2 && !boot_verify_calls);
  for(i=0;i<40;i++) { physical[2]+=5; step(false,0,0,0); }
  assert(status().state==GRAB_HOMING && !boot_verify_calls);
  status_write(false);
  assert(strstr(output,"axis=base") && strstr(output,"raw=") && strstr(output,"home_flags=0x03"));
  run_to(GRAB_IDLE,100,0); assert(boot_verify_calls==1);
  for(i=0;i<40;i++) step(false,0,0,0);
  assert(boot_verify_calls==1);
  reset(); base_state_flags=0; pre_enable_succeeds=false;GrabTask_BootHomeStart();run_to(GRAB_ERROR,300,0);
  assert(homes==2 && !boot_verify_calls && !strcmp(status().reason,"home_enable_failed"));
}
static void coordinate_logging(void)
{
  unsigned before;
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,160,1);
  camera.Data.CX=350; camera.Data.CY=321; camera.Data.DX=-9; camera.Data.DY=17;
  camera.Data.Sequence=123;
  status_write(true);
  assert(camera_logs && strstr(camera_log,"cx=350 cy=321 dx=-9 dy=17"));
  assert(strstr(camera_log,"rx_seq=123") && strstr(camera_log,"valid=1 fresh=1"));
  clock_ms+=600; status_write(true); assert(strstr(camera_log,"fresh=0"));
  camera.HasValidData=camera.TargetValid=0; status_write(true);
  Status.dx=22; Status.dy=-33; status_write(false);
  assert(strstr(output,"dx=na dy=na") && !strstr(output,"dx=22"));
  assert(strstr(camera_log,"valid=0") && strstr(camera_log,"cx=na cy=na"));
  assert(!strstr(camera_log,"cx=350"));
  before=camera_logs; GrabTask_Stop(); status_write(true); assert(camera_logs==before);
  reset(); configure(false); assert(GrabTask_Start("fixed")); run_to(GRAB_COMPLETE,600,0); assert(!camera_logs);
}
static void reference_invalidation(void)
{
  MechanicalArm_EventTypeDef event={0};
  reset(); configure(true); assert(GrabTask_ConfigReady("pick"));
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE); assert(!GrabTask_ConfigReady("pick"));
  assert(GrabTask_ConfigReady("fixed")); configure(true);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_X); assert(!GrabTask_ConfigReady("pick"));
  assert(!GrabTask_ConfigReady("fixed"));
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_Z); assert(!GrabTask_ConfigReady("fixed"));
  configure(true); event.Action=MECHANICAL_ARM_ACTION_ZERO; event.Axis=MECHANICAL_ARM_AXIS_X;
  event.Transmitted=1; event.Result=MECHANICAL_ARM_RESULT_OK;
  assert(!GrabTask_MotorEventHandle(&event)); assert(!GrabTask_ConfigReady("pick"));
  configure(true); event.Axis=MECHANICAL_ARM_AXIS_BASE;
  assert(!GrabTask_MotorEventHandle(&event)); assert(!GrabTask_ConfigReady("pick"));
  assert(!strcmp(status().reference_cause,"manual_zero_base"));
}
static void b2_stability_boundaries(void)
{
  unsigned i, before;
  reset(); configure(true); camera_valid=false; assert(GrabTask_Start("pick"));
  run_to(GRAB_ACQUIRE,180,1); before=positions;
  for(i=0;i<60;i++) step(false,1,0,0);
  assert(status().state==GRAB_ACQUIRE && !closures && positions==before && !velocity_requests);
  camera_valid=true; run_to(GRAB_ALIGN,20,1);
  /* Parser has seen valid -> invalid -> valid in one USB batch. */
  camera.InvalidCount++; step(true,1,0,0);
  assert(status().state==GRAB_VISION_PAUSE && !CaptureCount && !VisualStable && !closures);
  camera_valid=false; run_to(GRAB_ERROR,320,1); assert(!strcmp(status().reason,"vision_recover_timeout"));
  camera_valid=true;
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1);
  step(false,1,0,0); camera.InvalidCount=0; assert(GrabTask_Start("align")); run_to(GRAB_HOLD,250,1);
  assert(!closures);
  reset(); configure(true); assert(GrabTask_Start("pick")); run_to(GRAB_ALIGN,180,1);
  camera.Data.Function=0xb4; camera.Data.Sequence++; GrabTask_Process();
  assert(status().state==GRAB_STOPPING && !closures);
  /* Count and duration are independent gates, including millisecond rollover. */
  reset(); configure(true); Axes[0].quiet=WheelQuiet=true; Axes[0].speed=0;
  Status.dx=Status.dy=0; FreshVision=true; clock_ms=0xfffffff0U; stable_reset();
  visual_stability(clock_ms); visual_stability(clock_ms+50U); visual_stability(clock_ms+100U);
  assert(CaptureCount==3 && !VisualStable);
  visual_stability(clock_ms+200U); assert(VisualStable);
  stable_reset(); visual_stability(clock_ms); visual_stability(clock_ms+250U); assert(!VisualStable);
  FreshVision=false; visual_stability(clock_ms+300U); assert(CaptureCount==2);
  FreshVision=true; visual_stability(clock_ms+300U); assert(VisualStable);
  Status.dx=3; visual_stability(clock_ms+350U); assert(!CaptureCount && !VisualStable);
  /* Receive sequence zero after UINT32_MAX is new, but rereading it is not. */
  reset(); configure(true); RequestTick=0xffffffe0U; HaveCapture=true; LastSequence=0xffffffffU;
  camera.UsbConfigured=camera.RequestActive=camera.HasValidData=camera.TargetValid=1;
  camera.RequestFunction=camera.Data.Function=0xb2; camera.RequestTarget=camera.Data.Target=1;
  camera.Data.Sequence=0; camera.Data.Tick=5; camera.Data.CX=334; camera.Data.CY=338;
  assert(vision_read(10) && FreshVision && Vision.Sequence==0);
  assert(vision_read(20) && !FreshVision);
  LastInvalidCount=0xffffffffU; camera.InvalidCount=0; Status.state=GRAB_ALIGN;
  assert(!vision_read(30) && status().state==GRAB_VISION_PAUSE);
}

static void brief_loss_restarts_alignment_request_after_stop(void)
{
  unsigned before, moves, n;
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  before=camera_requests; moves=velocity_requests;
  camera.InvalidCount++; step(true,1,20,20);
  assert(!strcmp(status().state_name,"vision_pause"));
  for(n=0;n<7;n++) { step(true,1,20,20); assert(!strcmp(status().state_name,"vision_pause")); }
  assert(velocity_requests==moves && !closures && camera_requests==before);
  run_to(GRAB_ALIGN,30,1);
  assert(!VisualStable && CaptureCount==0 && !closures && camera_requests==before+1);
  run_to(GRAB_HOLD,200,1); assert(!closures && !base_moves);
}

static void pause_until_stopped(void)
{
  unsigned i;
  camera.InvalidCount++; step(true,1,20,20);
  assert(status().state==GRAB_VISION_PAUSE && !RecoveryWaiting);
  status_write(false); assert(strstr(output,"recovery_left_ms=na"));
  for(i=0;i<80 && !RecoveryWaiting;i++) step(true,1,20,20);
  assert(RecoveryWaiting && !RecoveryCount && status().stop_confirmed);
  assert(status().recovery_left_ms==12000);
}
static void restart_after_stop_and_verified_rehome(void)
{
  char *rehome[]={"grab","rehome"};
  reset(); configure(true);
  assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,160,1);
  assert(GrabTask_Command(2,rehome));
  assert(strstr(output,"ERR grab rehome rejected") && status().state==GRAB_ALIGN);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0);
  assert(GrabTask_ConfigReady("align"));
  for(unsigned i=0;i<5 && pending_valid;i++) step(false,0,0,0);
  assert(GrabTask_Start("align"));
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0);
  for(unsigned i=0;i<5 && pending_valid;i++) step(false,0,0,0);

  /* A changed pickup reference cannot be revived by stop or HOME ACK alone. */
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_ALL);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(!GrabTask_ConfigReady("align"));
  physical[0]=physical[1]=0; /* Simulate collision-home arrival in the fake plant. */
  bus.locked=true;
  assert(GrabTask_Command(2,rehome));
  assert(strstr(output,"ERR grab rehome rejected") && status().state==GRAB_IDLE);
  bus.locked=false;
  assert(GrabTask_Command(2,rehome));
  assert(status().state==GRAB_HOMING && !GrabTask_ConfigReady("align"));
  for(unsigned i=0;i<70 && !homes;i++)step(false,0,0,0);
  home_flags=7;
  for(unsigned i=0;i<70;i++) step(false,0,0,0);
  assert(homes==2 && !GrabTask_ConfigReady("align"));
  home_flags=3; run_to(GRAB_IDLE,250,0);
  assert(homes==3 && boot_verify_calls==1 && GrabTask_ConfigReady("align"));
  assert(!strcmp(status().reference_cause,"home_verified"));
  assert(GrabTask_Start("align"));

  reset(); configure(true);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_ALL);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(GrabTask_Command(2,rehome));
  home_flags=11; run_to(GRAB_ERROR,150,0);
  assert(!GrabTask_ConfigReady("align") && !boot_verify_calls);
  /* A failed boot is itself a valid reason to request verified rehoming. */
  home_flags=3;homes=0;
  assert(GrabTask_Command(2,rehome));assert(status().state==GRAB_HOMING);
  run_to(GRAB_IDLE,250,0);
  assert(GrabTask_ConfigReady("align") && boot_verify_calls==1);
}
static void alignment_reacquire_boundaries(void)
{
  unsigned before,moves,i;
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  before=camera_requests; pause_until_stopped(); moves=velocity_requests;
  assert(camera_requests==before); /* No new request until standstill is confirmed. */
  camera_tx_result=HAL_BUSY;
  step(true,1,20,20); step(true,1,20,20);
  assert(camera_requests==before && !RecoveryCount && status().state==GRAB_VISION_PAUSE);
  camera_tx_result=HAL_OK; step(true,1,20,20);
  assert(camera_requests==before+1 && requested_color==CAMERA_COLOR_BLUE && !RecoveryCount);
  assert(!camera.HasValidData && !camera.HasFrame); /* Request flushes the previous snapshot. */
  assert(!strcmp(status().reason,"vision_reacquire_wait"));
  for(i=0;i<2;i++) { step(true,1,20,20); assert(status().state==GRAB_VISION_PAUSE); }
  step(true,1,20,20); assert(status().state==GRAB_ALIGN && RecoveryUsed==1);
  assert(camera_requests==before+1 && velocity_requests==moves && !closures);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); before=camera_requests; camera_tx_result=HAL_BUSY;
  run_to(GRAB_ERROR,320,1);
  assert(camera_requests==before && !strcmp(status().reason,"vision_recover_timeout"));
  assert(status().stop_confirmed);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); before=camera_requests; camera_tx_result=HAL_ERROR;
  step(true,1,20,20); assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"camera_tx"));
  run_to(GRAB_ERROR,80,1); assert(camera_requests==before && status().stop_confirmed);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); before=camera_attempts; wheel_moving=true; camera_valid=false;
  step(true,1,20,20); assert(!status().stop_confirmed && camera_attempts==before);
  wheel_moving=false;
  GrabTask_Stop(); run_to(GRAB_IDLE,80,1); assert(camera_attempts==before);

  reset(); configure(true); assert(GrabTask_Start("pick")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); before=camera_requests; run_to(GRAB_ALIGN,20,1);
  assert(camera_requests==before+1 && requested_color==CAMERA_COLOR_RED);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  before=camera_requests;
  for(i=0;i<4;i++) {
    pause_until_stopped(); step(true,1,20,20);
    assert(camera_requests==before+i+1 && !RecoveryCount);
    step(false,1,20,20); assert(!RecoveryCount); /* No new receive sequence after request. */
    run_to(GRAB_ALIGN,6,1); assert(RecoveryUsed==i+1);
  }
  camera.InvalidCount++; step(true,1,20,20);
  assert(status().state==GRAB_VISION_PAUSE && RecoveryUsed==4);
  run_to(GRAB_ALIGN,80,1); assert(camera_requests==before+5 && RecoveryUsed==5);
}
static void recovery_rollover_and_settle(void)
{
  unsigned i, moves;
  reset(); clock_ms=0xffffd000U; configure(true); assert(GrabTask_Start("pick"));
  run_to(GRAB_SETTLE,220,1); pause_until_stopped();
  step(true,1,0,0); assert(!RecoveryRestartPending); /* Pick now shares the B2 restart. */
  /* Shift the whole feedback timeline close to wrap without making feedback old. */
  { uint32_t delta=0xffffff80U-clock_ms;
    clock_ms+=delta; RecoveryTick+=delta; StopTick+=delta; WheelQuietTick+=delta;
    LastPoll+=delta; StateTick+=delta;
    for(i=0;i<3;i++) { Axes[i].position_tick+=delta; Axes[i].state_tick+=delta; Axes[i].quiet_since+=delta; }
    for(i=0;i<4;i++) WheelLastPositionTick[i]+=delta;
  }
  camera.Data.Sequence=RecoverySequence=0xfffffffeU;
  moves=velocity_requests;
  wheel_moving=true;
  for(i=0;i<3;i++) step(true,1,0,0);
  assert(RecoveryCount==3 && RecoverySequence==1 && status().state==GRAB_VISION_PAUSE);
  assert(!status().stop_confirmed && !closures && velocity_requests==moves);
  wheel_moving=false;
  for(i=0;i<6 && status().state!=GRAB_ALIGN;i++) step(false,1,0,0);
  assert(status().state==GRAB_ALIGN && camera.Data.Sequence==1);
  assert(!CaptureCount && !VisualStable && !closures && RecoveryUsed==1);
  step(false,1,0,0); assert(status().state==GRAB_ALIGN && !CaptureCount);
  run_to(GRAB_SETTLE,20,1); assert(!CaptureCount && !closures);
  run_to(GRAB_DESCEND,50,1); assert(!closures && !base_moves);
}

static void recovery_boundaries_and_faults(void)
{
  unsigned i, before;
  reset(); configure(true); assert(GrabTask_Start("pick")); run_to(GRAB_ALIGN,180,1);
  Retries=2;
  for(i=0;i<4;i++) {
    pause_until_stopped(); before=velocity_requests;
    step(false,1,20,20); assert(!RecoveryCount);
    step(true,1,20,20); assert(RecoveryCount==1);
    camera.InvalidCount++; step(true,1,20,20); assert(RecoveryCount==1);
    camera.Data.Sequence+=5; GrabTask_Process(); assert(RecoveryCount==2);
    assert(velocity_requests==before && !closures);
    RecoveryTick=clock_ms-11950; step(true,1,20,20); /* Exactly 12000 ms is accepted. */
    assert(status().state==GRAB_ALIGN && RecoveryUsed==i+1 && Retries==2 && !CaptureCount);
    assert(fabsf(XIntegral-Axes[0].mm)<.001f && !PreviousVelocity[0]);
  }
  camera.InvalidCount++; step(true,1,20,20);
  assert(status().state==GRAB_VISION_PAUSE && RecoveryUsed==4);
  assert(!RecoveryWaiting && !closures); run_to(GRAB_ALIGN,100,1); assert(RecoveryUsed==5);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1); step(false,1,0,0);
  assert(GrabTask_Start("align") && RecoveryUsed==0);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); step(true,1,20,20); step(true,1,20,20);
  RecoveryTick=clock_ms-11951; step(true,1,20,20);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"vision_recover_timeout"));

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); GrabTask_Stop();
  assert(!RecoveryWaiting && !RecoveryCount); run_to(GRAB_IDLE,100,1); assert(!RecoveryUsed);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); camera.UsbConfigured=0; GrabTask_Process();
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"vision_usb_off"));
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); camera.RequestActive=0; GrabTask_Process();
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"vision_request_lost"));
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); wheel_stale=true; step(true,1,20,20);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"feedback_stale"));
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); bus.locked=true; step(true,1,20,20);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"chassis_tx"));
}

static void travel_limit_still_confirms_stopping(void)
{
  unsigned wheel,i,moves;
  for(wheel=0;wheel<4;wheel++) {
    reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
    moves=velocity_requests; wheel_position[wheel]=65536; wheel_moving=true;
    step(true,1,20,20);
    assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"travel_limit"));
    for(i=0;i<8;i++) step(true,1,20,20);
    assert(status().state==GRAB_STOPPING && !status().stop_confirmed);
    wheel_moving=false; wheel_stale=true;
    for(i=0;i<6;i++) step(true,1,20,20);
    assert(status().state==GRAB_STOPPING && !status().stop_confirmed);
    wheel_stale=false;
    if(wheel&1) { GrabTask_Stop(); run_to(GRAB_IDLE,100,1); }
    else { run_to(GRAB_ERROR,100,1); assert(!strcmp(status().reason,"travel_limit")); }
    assert(status().stop_confirmed && velocity_requests==moves && !closures && !base_moves);
  }
}

static void mode_target_colors(void)
{
  reset(); configure(true); assert(GrabTask_Start("align"));
  run_to(GRAB_ACQUIRE,180,1); assert(requested_color==CAMERA_COLOR_BLUE);
  /* A red reply cannot start blue alignment, even with a matching request. */
  camera.RequestTarget=CAMERA_COLOR_BLUE; camera.Data.Target=CAMERA_COLOR_RED;
  camera.Data.Tick=clock_ms; camera.Data.Sequence++; GrabTask_Process();
  assert(status().state==GRAB_ACQUIRE && !velocity_requests && !closures);
  run_to(GRAB_ALIGN,30,1); status_write(false); assert(strstr(output,"color=3"));
  status_write(true); assert(strstr(camera_log,"color=3 valid=1"));
  run_to(GRAB_HOLD,250,1); assert(!closures && !base_moves);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1);
  step(false,1,0,0); /* Deliver the outstanding feedback reply before restarting. */
  assert(GrabTask_Start("pick")); run_to(GRAB_ALIGN,180,1);
  assert(requested_color==CAMERA_COLOR_RED);
  status_write(false); assert(strstr(output,"color=1"));
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1); assert(!closures);
}

static void repeated_stop_must_finish(void)
{
  unsigned i, moves; char *stop[]={"grab","stop"};
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,160,1);
  step(true,1,20,20); moves=velocity_requests;
  for(i=0;i<100 && status().state!=GRAB_IDLE;i++) {
    assert(GrabTask_Command(2,stop)); step(true,1,20,20);
  }
  assert(status().state==GRAB_IDLE && status().stop_confirmed);
  assert(velocity_requests==moves && !closures);
}
static void command_reports_target_and_acceptance(void)
{
  char *start[]={"grab","start","align"};
  reset(); configure(true); assert(GrabTask_Command(3,start));
  assert(strstr(accepted_log,"OK grab start align accepted color=3"));
  status_write(true); assert(strstr(progress_log,"mode=align") && strstr(progress_log,"color=3"));
  assert(strstr(progress_log,"protocol=B2") && strstr(progress_log,"stop_confirmed=0"));
}

static void tuned_initial_response_is_faster_and_bounded(void)
{
  float v[3], previous=0; unsigned i;
  reset(); GrabTask_BootHomeStart(); run_to(GRAB_IDLE,250,0);
  /* Live run: dx=-95..-57, dy approximately zero. The new profile should
   * exceed the old 10 mm/s cap here, then slow as it approaches the window. */
  assert(solve(-95,0,0,v) && v[0]>118.7f && v[0]<118.8f);
  assert(solve(-61,0,0,v) && v[0]>76.2f && v[0]<76.3f);
  assert(solve(-6,0,0,v) && v[0]>0 && v[0]<8.0f);
  assert(solve(95,0,0,v) && v[0]<-118.7f && v[0]>-118.8f);
  assert(solve(-3000,-3000,50,v));
  assert(hypotf(v[0],v[1])<=160.001f && fabsf(v[2])<=60.001f);
  /* Exercise the production acceleration limiter over consecutive updates. */
  pending_valid=false; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  LastControl=clock_ms; Status.dx=-95; Status.dy=0; FreshVision=true;
  memset(PreviousVelocity,0,sizeof PreviousVelocity); XIntegral=Axes[0].mm;
  for(i=0;i<10;i++) {
    clock_ms+=100; align_control(clock_ms);
    assert(Status.forward_mm_s>=previous && Status.forward_mm_s-previous<=30.001f);
    previous=Status.forward_mm_s;
  }
  assert(previous>118.7f && previous<118.8f);
  Status.dx=0;
  for(i=0;i<11;i++) { clock_ms+=100; align_control(clock_ms); }
  assert(fabsf(Status.forward_mm_s)<1e-5f);
}
static void travel_limit_zero_disables_only_travel_check(void)
{
  reset(); assert(Config[C_TRAVEL]==0);
  configure(true); setting("travel_mm","0");
  assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  wheel_position[0]=65536; /* More than the former 100 mm default. */
  step(true,1,20,20);
  assert(status().state==GRAB_ALIGN && !strcmp(status().reason,"running"));
  assert(Config[C_TRAVEL]==0 && !closures);
}

static void selected_target_colors(void)
{
  unsigned color;
  char selected[2]={'1',0};
  char *align[]={"grab","start","align",selected};
  char *pick[]={"grab","start","pick",selected};
  char *fixed[]={"grab","start","fixed",selected};
  for(color=1;color<=6;color++)
  {
    selected[0]=(char)('0'+color);
    reset(); configure(true); assert(GrabTask_Command(4,align));
    assert(strstr(accepted_log,"OK grab start align accepted"));
    run_to(GRAB_ACQUIRE,180,1); assert(requested_color==color);
    status_write(false); assert(strstr(output,"color=") && target_color()==color);
    reset(); configure(true); assert(GrabTask_Command(4,pick));
    assert(strstr(accepted_log,"OK grab start pick accepted"));
    run_to(GRAB_ACQUIRE,180,1); assert(requested_color==color);
  }
  reset(); configure(true); assert(GrabTask_Command(4,fixed));
  assert(strstr(output,"ERR grab start color") && status().state==GRAB_IDLE);
  selected[0]='0'; assert(GrabTask_Command(4,align));
  assert(strstr(output,"ERR grab start color") && status().state==GRAB_IDLE);
}

static void grab_z_speed_matches_transfer_defaults(void)
{
  unsigned i;
  reset(); GrabTask_BootHomeStart(); run_to(GRAB_IDLE,250,0);
  assert(GrabTask_Start("fixed")); run_to(GRAB_DESCEND,100,0);
  for(i=0;i<10 && !Axes[1].commanded;i++) step(true,0,0,0);
  /* 1000 RPM is the ceiling; fixed moves now start below it with a MCU ramp. */
  assert(z_position_rpm>0 && z_position_rpm<1000 && z_position_pulses==-24000 && z_position_acc==0);
  assert(!closures); /* A submitted descent is not arrival. */
  pending_valid=false; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  enter(GRAB_LIFT);
  assert(axis_move(1,-40));
  assert(z_position_rpm>0 && z_position_rpm<1000 && z_position_pulses==-19200 && z_position_acc==0);
  /* Preserve the existing mm/s setting, including the value sent by the UI. */
  reset(); configure(false); setting("z_ppm","480"); setting("z_speed","27.777778");
  assert(axis_move(1,-10)); assert(z_position_rpm<250 && z_position_pulses==-4800);
  assert(Axes[1].profile.peak<=27.777778f);
  reset(); configure(false); setting("z_ppm","480"); setting("z_speed","5");
  assert(axis_move(1,-10)); assert(z_position_rpm<45 && z_position_pulses==-4800);
  assert(Axes[1].profile.peak==5);
}

static void long_color_wait_and_timeout(void)
{
  for(unsigned pick=0;pick<2;pick++) {
    reset();configure(true);camera_valid=false;
    assert(GrabTask_Start(pick?"pick":"align"));run_to(GRAB_ACQUIRE,180,1);
    unsigned before=velocity_requests;
    for(unsigned i=0;i<200;i++) step(true,1,0,0);
    assert(status().state==GRAB_ACQUIRE && velocity_requests==before && !closures);
    camera_valid=true;step(true,1,20,20);assert(status().state==GRAB_ALIGN);
    pause_until_stopped();camera_valid=false;
    for(unsigned i=0;i<200;i++) step(true,1,20,20);
    assert(status().state==GRAB_VISION_PAUSE && !closures);
    camera_valid=true;run_to(GRAB_ALIGN,10,1);assert(RecoveryUsed==1);
  }
  reset();configure(true);camera_valid=false;setting("vision_wait_ms","2000");
  assert(GrabTask_Start("align"));run_to(GRAB_ACQUIRE,180,1);
  RequestTick=clock_ms-2000;step(false,1,0,0);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"vision_acquire_timeout"));
  assert(!closures);
  reset();configure(true);camera_valid=false;assert(GrabTask_Start("align"));
  run_to(GRAB_ACQUIRE,180,1);GrabTask_Stop();run_to(GRAB_IDLE,100,1);
}

static void ring_tracking_parameters(void)
{
  reset();configure(true);setting("z_min","-150");float original_body=Config[C_BODY_SPEED],original_x=Config[C_X_SPEED];
  RingTest=true;
  assert(align_parameter(C_BODY_SPEED)==original_body);
  assert(align_parameter(C_X_SPEED)==original_x);
  assert(align_parameter(C_ACCEL)==Config[C_ACCEL]);
  assert(align_parameter(C_LAMBDA)==Config[C_LAMBDA]);
  setting("dx_tol","1");setting("dy_tol","1");
  Status.dx=Status.dy=1;assert(in_window());Status.dx=2;assert(!in_window());
  TrackConfig.x_speed_mm_s=42;assert(align_parameter(C_X_SPEED)==original_x);
  RingTest=true;Status.state=GRAB_HOLD;Status.stop_confirmed=true;
  Axes[1].mm=Config[C_Z_MAX]-.002f;
  assert(ring_place_start(-10));assert(RingReturnZ==Config[C_Z_MAX]);
  RingTest=false;
  assert(align_parameter(C_BODY_SPEED)==original_body && align_parameter(C_X_SPEED)==original_x);
}
static void ring_no_preextension(void)
{
  char *cmd[]={"grab","ring","align"};
  reset();configure(true);setting("z_min","-150");setting("x_min","0");setting("x_max","100");RearReferenceValid=RearPose=true;RearBase=0;
  base_arrives=false;
  assert(GrabTask_Command(3,cmd));
  for(unsigned i=0;i<200 && !base_moves;i++)step(true,1,0,0);
  assert(base_moves==1 && fabsf(destination[2]-130)<.2f);
  for(unsigned i=0;i<40;i++)step(true,1,0,0);
  assert(fabsf(destination[0])<.2f && !camera_requests);
  base_arrives=true;physical[2]=destination[2];
  for(unsigned i=0;i<200 && !camera_requests;i++)step(true,1,0,0);
  assert(camera_requests && fabsf(destination[0])<.2f);
  run_to(GRAB_HOLD,250,1);
  assert(requested_color==10 && base_moves==1);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
}
static void ring_approach_before_progressive_descent(void)
{
  char *cmd[]={"grab","ring","align","3","-100"};
  reset();configure(true);setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_lift","0");setting("z_car_lift","0");
  setting("x_max","100");setting("travel_mm","0");
  assert(GrabTask_Command(5,cmd));
  for(unsigned i=0;i<300 && !camera_requests;i++)step(true,1,-330,0);
  assert(camera_requests && fabsf(physical[1])<.1f);
  for(unsigned i=0;i<50;i++)step(true,1,-330,0); /* CX=48: keep the top view. */
  assert(velocity_requests>0 && fabsf(destination[1])<.1f && !RingLowerSent);
  for(unsigned i=0;i<100;i++)step(true,1,-120,0);
  assert(physical[1]<-10 && physical[1]>-80 && !RingLowerSent);
  float partial=physical[1];
  for(unsigned i=0;i<35;i++)step(true,1,0,310); /* CY=23: vertical edge also holds. */
  assert(physical[1]>=partial-5.1f && physical[1]>-80);
  partial=physical[1];
  for(unsigned i=0;i<35;i++)step(true,1,230,0); /* CX=608: pause descent again. */
  assert(physical[1]>=partial-5.1f && physical[1]>-80);
  partial=physical[1];
  for(unsigned i=0;i<3;i++)step(false,1,0,0);
  assert(physical[1]>=partial-.1f); /* Repeated packets cannot advance Z. */
  run_to(GRAB_HOLD,500,1);
  assert(!strcmp(status().result,"aligned") && fabsf(physical[1])<.1f);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
}
static void ring_reference_is_independent_of_material(void)
{
  char *cmd[]={"grab","ring","align","3","-100","stay"};
  reset();configure(true);setting("z_min","-150");setting("x_max","100");setting("travel_mm","0");
  pixel_center_override=true;pixel_center_x=372;pixel_center_y=345;
  assert(GrabTask_Command(6,cmd));run_to(GRAB_ALIGN,300,1);
  for(unsigned i=0;i<100;i++)step(true,1,-45,-5); /* Old center (333,338) is not aligned. */
  assert(status().state!=GRAB_HOLD && status().dx==-45 && status().dy==-5);
  run_to(GRAB_HOLD,500,1);
  assert(!strcmp(status().reason,"ring_aligned_low") && status().dx==0 && status().dy==0);
  assert(status().dx==0 && status().dy==0);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  assert(GrabTask_Start("align"));run_to(GRAB_HOLD,350,1);
  assert(!RingTest && status().dx==0 && status().dy==0);
  assert(status().dx==0 && status().dy==0);
}
static void ring_lower_then_raise_without_alignment(void)
{
  char *cmd[]={"grab","ring","align","3","-100"};
  reset();configure(true);setting("z_min","-150");setting("x_max","100");
  assert(GrabTask_Command(5,cmd));
  for(unsigned i=0;i<300 && !camera_requests;i++)step(true,1,0,0);
  assert(camera_requests && fabsf(physical[1]-Config[C_Z_MAX])<.1f);
  run_to(GRAB_ALIGN,100,1);
  assert(velocity_requests>0 && destination[1]==Config[C_Z_MAX]);
  move_arrives=false;
  for(unsigned i=0;i<80;i++)step(true,1,0,0);
  assert(status().state==GRAB_ALIGN && !RingLowerSent);
  assert(destination[1]>=Config[C_Z_MAX]-5.1f); /* ACK cannot advance descent. */
  move_arrives=true;physical[1]=destination[1];
  for(unsigned i=0;i<300 && !RingLowerSent;i++)step(true,1,0,0);
  assert(RingLowerSent && fabsf(destination[1]+100)<.1f);
  run_to(GRAB_OBSERVE_Z,300,1);
  assert(RingRaise && !HaveCapture);
  float x=destination[0];unsigned requests=camera_requests;
  run_to(GRAB_HOLD,300,1);
  assert(!RingRaise && fabsf(physical[1]-Config[C_Z_MAX])<.1f);
  assert(destination[0]==x && camera_requests==requests && !HaveCapture);
  assert(GrabTask_RingManualAllowed());
  GrabTask_Stop();run_to(GRAB_IDLE,200,1);assert(!GrabTask_RingManualAllowed());
}
static void ring_alignment_can_hold_low_for_observation(void)
{
  char *stay[]={"grab","ring","align","1","-60","stay"};
  char *normal[]={"grab","ring","align","3","-100"};
  char *bad[]={"grab","ring","align","1","-60","invalid"};
  reset();configure(true);setting("z_min","-150");setting("x_max","100");
  assert(GrabTask_Command(6,bad));assert(!positions && status().state==GRAB_IDLE);
  assert(GrabTask_Command(6,stay));
  unsigned before=camera_stops;
  run_to(GRAB_HOLD,500,1);
  assert(!strcmp(status().reason,"ring_aligned_low") && !strcmp(status().result,"aligned"));
  assert(fabsf(physical[1]+60)<.1f && fabsf(destination[1]+60)<.1f);
  assert(camera_stops==before && !closures && GrabTask_RingManualAllowed());
  unsigned moves=positions, velocities=velocity_requests;
  for(unsigned i=0;i<50;i++)step(true,1,100,20); /* Observe new error without correcting it. */
  assert(positions==moves && velocity_requests==velocities && camera_stops==before);
  assert(status().state==GRAB_HOLD && fabsf(physical[1]+60)<.1f);
  assert(status().dx==100 && status().dy==20);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  assert(camera_stops>before && !GrabTask_RingManualAllowed());
  assert(GrabTask_Command(5,normal));run_to(GRAB_HOLD,500,1);
  assert(!strcmp(status().reason,"ring_aligned_ready") && physical[1]==Config[C_Z_MAX]);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
}
static void ring_top_alignment_does_not_raise_again(void)
{
  reset();configure(true);setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_lift","0");setting("z_car_lift","0");
  assert(GrabTask_AlignStart(10,0,false));
  run_to(GRAB_SETTLE,500,1);
  unsigned moves=positions;
  for(unsigned i=0;i<300 && status().state!=GRAB_HOLD;i++){
    step(true,1,0,0);assert(!RingRaise && status().state!=GRAB_OBSERVE_Z);
  }
  assert(status().state==GRAB_HOLD && positions==moves && physical[1]==0);
  assert(!strcmp(status().reason,"ring_aligned_ready") && !HaveCapture);
  assert(GrabTask_RingManualAllowed());
}
static void black_ring_manual_place(void)
{
  char *align[]={"grab","ring","align"};
  char *place[]={"grab","ring","place","-150"};
  reset(); configure(true);setting("z_min","-150"); setting("z_min","-150");setting("x_max","100");
  assert(GrabTask_Command(4,place)); assert(!positions && !openings);
  assert(GrabTask_Command(3,align));
  assert(status().state==GRAB_PREPARE);
  run_to(GRAB_HOLD,250,1);
  assert(requested_color==10 && !closures && !tray_indexes);
  assert(!strcmp(status().operation,"ring_align") && !strcmp(status().result,"aligned"));
  unsigned before=openings, turns=base_moves, index=tray_indexes;
  /* User inserts the load and closes the claw; firmware must only descend/release. */
  assert(GrabTask_Command(4,place)); assert(status().state==GRAB_DESCEND);
  move_arrives=false;
  for(unsigned i=0;i<30;i++)step(true,1,0,0);
  assert(openings==before && status().state==GRAB_DESCEND);
  move_arrives=true; physical[1]=destination[1];
  for(unsigned i=0;i<250 && strcmp(status().result,"released");i++)step(true,1,0,0);
  assert(!strcmp(status().result,"released") && status().state==GRAB_HOLD);
  assert(openings==before+1 && !closures && tray_indexes==index && base_moves==turns);
  assert(fabsf(physical[1])<.11f && tray_inventory[0].state==TURNTABLE_EMPTY); /* Return to the default Z=0 alignment height. */
  GrabTask_Stop(); run_to(GRAB_IDLE,150,1);
}
static void calibration_commands_and_atomic_response(void)
{
  reset();configure(true);setting("travel_mm","0");
  char *start[]={"grab","calib","2"}, *ref[]={"grab","calib","ref"}, *query[]={"grab","calib","status"};
  assert(GrabTask_Command(3,start) && strstr(output,"ref_required"));
  assert(!GrabTask_IsBusy() && !positions && !velocity_requests);
  float saved=Config[C_J00];
  assert(GrabTask_Command(3,ref) && strstr(output,"confirmed"));
  assert(GrabTask_CalibrationReferenceValid() && Config[C_J00]==saved);
  assert(GrabTask_Command(3,start) && GrabTask_IsBusy());
  unsigned before=positions, cameras=camera_requests;
  assert(GrabTask_Command(3,query) && strstr(output,"OK grab calib phase=prepare"));
  assert(positions==before && camera_requests==cameras);
  uint32_t started=StartTick;
  assert(GrabTask_Command(3,start) && StartTick==started && Status.state==GRAB_PREPARE);
  assert(!GrabTask_CalibrationStart(2));
  run_to(GRAB_CALIBRATION,200,0);
  assert(!positions && !base_moves && !openings && !closures);
  assert(GrabTask_Command(3,query) && strstr(output,"phase=collect"));
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  assert(GrabTask_CalibrationReferenceValid());
  float lateral=Config[C_J11];
  assert(GrabTask_VisualResponseSet(3,1,2,4));
  assert(Config[C_J00]==3 && Config[C_J10]==1 && Config[C_J11]==lateral);
  assert(fabsf(Config[C_J02]-2*Config[C_X_PPM]*360/3200)<.001f);
  saved=Config[C_J00];assert(!GrabTask_VisualResponseSet(NAN,0,0,0) && Config[C_J00]==saved);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(GrabTask_CalibrationReferenceValid()); /* P001 is deliberately unchanged. */
}
static void ring_command_default_height(void)
{
  char *generic[]={"grab","align","ring","2"};
  char *legacy[]={"grab","ring","align","2"};
  char *short_alias[]={"grab","ring","align"};
  char **commands[]={generic,legacy,short_alias};
  unsigned counts[]={4,4,3};
  for(unsigned i=0;i<3;i++) {
    reset();configure(true);setting("z_min","-150");setting("x_max","100");
    assert(GrabTask_Command(counts[i],commands[i]));
    assert(GrabTask_IsBusy() && RingAlignZ==0 && RingStayLow);
    run_to(GRAB_HOLD,500,0);
    assert(fabsf(physical[1])<.11f && !RingRaise && !closures);
  }
  char *explicit_high[]={"grab","align","ring","2","0"};
  char *explicit_low[]={"grab","ring","align","2","-100"};
  char *explicit_stay[]={"grab","align","ring","2","-100","stay"};
  char **explicit_commands[]={explicit_high,explicit_low,explicit_stay};
  unsigned explicit_counts[]={5,5,6};
  for(unsigned i=0;i<3;i++) {
    reset();configure(true);setting("z_min","-150");setting("x_max","100");
    assert(GrabTask_Command(explicit_counts[i],explicit_commands[i]));
    assert(GrabTask_IsBusy() && RingAlignZ==(i? -100:0) && RingStayLow==(i==2));
    run_to(GRAB_HOLD,500,0);
    assert(fabsf(physical[1]-(i==2?-100:Config[C_Z_MAX]))<.11f);
  }
}
int main(void)
{
  ring_command_default_height();
  calibration_commands_and_atomic_response();
  reset();
  for(unsigned key=0;key<C_COUNT;key++) {
    const GrabParameter_t *p=&Parameters[key];
    if(isfinite(p->initial)) {
      char value[40]; snprintf(value,sizeof value,"%.9g",p->initial);
      setting(Keys[key],value);
    }
    assert(!config_value_valid(key,p->maximum+1));
    assert(!config_value_valid(key,p->minimum-1));
    assert(!config_value_valid(key,NAN));
    if(p->validation & 1)assert(!config_value_valid(key,p->minimum+.5f));
  }
  reset(); assert(isnan(Config[C_Z_PROC_PLACE]));
  assert(!config_value_valid(C_Z_PICK_SPEED,333.334f));
  assert(!config_value_valid(C_BASE_CAR_OFFSET,361));
  /* Device defaults must round-trip through the public parameter command. */
  reset(); setting("z_pick_speed","111.111111"); setting("z_speed","222.222222");
  reset(); configure(false); configured_x_acc=37; configured_z_acc=149;
  Status.state=GRAB_ALIGN;
  assert(axis_move(0,1)); assert(x_position_acc==37); /* Continuous correction unchanged. */
  pending_valid=false; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  Status.state=GRAB_LIFT;
  assert(axis_move(1,1)); assert(z_position_acc==0);
  ring_reference_is_independent_of_material();
  ring_tracking_parameters();
  ring_approach_before_progressive_descent();
  ring_lower_then_raise_without_alignment();
  ring_no_preextension();
  ring_alignment_can_hold_low_for_observation();
  ring_top_alignment_does_not_raise_again();
  black_ring_manual_place();
  for(unsigned ring=2;ring<=3;ring++) {
    char number[2]={(char)('0'+ring),0};char *cmd[]={"grab","ring","align",number};
    reset();configure(true);setting("z_min","-150");setting("x_max","100");
    assert(GrabTask_Command(4,cmd));run_to(GRAB_HOLD,300,1);
    assert(requested_color==9+ring);
    GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  }
  base_home_waits_for_other_origins();
  base_home_keeps_full_motion_window_after_preparation();
  boot_submit_wait_and_fault_diagnostics();
  boot_enable_and_disabled_feedback_recovery();
  car_x_default_survives_reset_and_verified_home();
  long_color_wait_and_timeout();
  grab_z_speed_matches_transfer_defaults();
  tuned_initial_response_is_faster_and_bounded();
  alignment_reacquire_boundaries();
  repeated_stop_must_finish();
  command_reports_target_and_acceptance();
  mode_target_colors();
  selected_target_colors();
  travel_limit_still_confirms_stopping();
  travel_limit_zero_disables_only_travel_check();
  recovery_rollover_and_settle();
  recovery_boundaries_and_faults();
  brief_loss_restarts_alignment_request_after_stop();
  b2_stability_boundaries();
  boot_defaults_start_align_without_settings();
  direct_align_and_complete_missing_report();
  initial_boundary_uses_position_tolerance();
  missing_and_fixed(); ack_is_not_arrival(); repeated_snapshot_and_success();
  placement_reference_restored_before_restart(); invalid_and_cancel();
  solver_and_configuration(); freshness_and_horizontal_gate(); bounded_drift_retry(); feedback_bounds();
  stale_wheels_cannot_confirm_stop(); fractional_x_and_all_wheels();
  fractional_x_survives_alternating_bus_wait();
  reference_invalidation(); placement_feedback_and_cancel(); boot_homing(); restart_after_stop_and_verified_rehome(); base_home_requires_stable_feedback(); coordinate_logging();
  puts("grab_task_test: PASS"); return 0;
}
