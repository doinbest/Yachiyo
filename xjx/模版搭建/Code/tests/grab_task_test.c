/* Exercise the production state machine; only device/time/console endpoints are fake. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../template/App/GrabTask.c"

static uint32_t clock_ms;
static MechanicalArm_EventTypeDef pending;
static bool pending_valid, move_arrives = true, camera_valid = true, imu_valid = true;
static float physical[3], destination[3];
static unsigned positions, closures, openings, stops, camera_requests, velocity_requests;
static uint16_t z_position_rpm;
static uint8_t z_position_acc;
static int32_t z_position_pulses;
static Camera_SnapshotTypeDef camera;
static Camera_ColorTypeDef requested_color;
static HAL_StatusTypeDef camera_tx_result=HAL_OK;
static unsigned camera_attempts;
static Mecanum_Status_t bus;
static int64_t wheel_position[4];
static bool wheel_moving, base_arrives=true, base_stale=false;
static unsigned base_moves, homes, aborts;
static uint8_t home_flags=3, base_state_flags=3;
static unsigned boot_verify_calls;
static bool wheel_stale;
static unsigned feedback_selected, selection_busy;
static char output[2048], camera_log[320], progress_log[320], accepted_log[128];
static unsigned camera_logs;
static Turntable_Inventory_t tray_inventory[3];
static Turntable_Status_t tray_status;
static bool tray_ready=true, tray_arrives=true, tray_moving;
static unsigned tray_indexes, tray_stops;
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
  assert(fabsf(physical[0]-Config[C_X_MIN])<.11f); /* Real X feedback has exited the tray. */
  assert(physical[1]>=Config[Operation==OP_STORE?lift_key():C_Z_CAR_LIFT]-.11f);
  assert(!tray_moving); tray_indexes++; tray_status.slot=slot;
  tray_status.arrived=false; tray_status.reason="indexing"; tray_moving=true; return true;
}
bool Turntable_IsBusy(void) { return tray_moving; }
void Turntable_Stop(void) { tray_stops++; tray_moving=false; tray_status.moving=false; tray_status.reason="stopped"; }
void Turntable_StatusGet(Turntable_Status_t *out) { *out=tray_status; out->moving=tray_moving; }
uint32_t HAL_GetTick(void) { return clock_ms; }
uint8_t ArmVision_IsBusy(void) { return 0; }
uint8_t MaterialVision_IsBusy(void) { return 0; }
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
bool Mecanum_IsBusy(void) { return false; }
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
MechanicalArm_ResultTypeDef MechanicalArm_Home(MechanicalArm_AxisTypeDef axis,uint8_t mode)
{
  assert(axis==BootAxes[homes] && mode==(axis==MECHANICAL_ARM_AXIS_BASE?0:2));
  homes++; return queue(MECHANICAL_ARM_ACTION_HOME,axis);
}
MechanicalArm_ResultTypeDef MechanicalArm_HomeStateRead(MechanicalArm_AxisTypeDef axis)
{ return queue(MECHANICAL_ARM_ACTION_HOME_STATE,axis); }
MechanicalArm_ResultTypeDef MechanicalArm_HomeAbort(MechanicalArm_AxisTypeDef axis)
{ aborts++; home_flags=3; return queue(MECHANICAL_ARM_ACTION_HOME_ABORT,axis); }
MechanicalArm_ResultTypeDef MechanicalArm_PositionRead(MechanicalArm_AxisTypeDef axis)
{ return queue(MECHANICAL_ARM_ACTION_POSITION_READ, axis); }
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef axis)
{ return queue(MECHANICAL_ARM_ACTION_STATE, axis); }
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis)
{ pending_valid = false; stops++; return queue(MECHANICAL_ARM_ACTION_STOP, axis); }
MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef axis, int32_t pulses,
 uint16_t rpm, uint8_t acc, MechanicalArm_PositionModeTypeDef mode)
{
  assert(rpm > 0); (void)acc;
  if(axis == MECHANICAL_ARM_AXIS_BASE) {
    assert(mode == MECHANICAL_ARM_POSITION_RELATIVE_CURRENT);
    assert(fabsf(physical[0]-Config[C_X_MIN])<.11f);
    if(pending_valid) return MECHANICAL_ARM_RESULT_BUSY;
    destination[2]=physical[2]+pulses*360.0f/3200; base_moves++; positions++;
    return queue(MECHANICAL_ARM_ACTION_POSITION,axis);
  }
  assert(mode == MECHANICAL_ARM_POSITION_ABSOLUTE);
  if (pending_valid) return MECHANICAL_ARM_RESULT_BUSY;
  if(axis==MECHANICAL_ARM_AXIS_Z) { z_position_rpm=rpm; z_position_acc=acc; z_position_pulses=pulses; }
  destination[axis == MECHANICAL_ARM_AXIS_X ? 0 : 1] = pulses / 100.0f;
  positions++; return queue(MECHANICAL_ARM_ACTION_POSITION, axis);
}
uint8_t MechanicalArm_ConfigGet(MechanicalArm_AxisTypeDef axis, MechanicalArm_ConfigTypeDef *out)
{ assert(axis==MECHANICAL_ARM_AXIS_BASE); out->SpeedRpm=30; out->Acceleration=20; out->PulseLimit=3200; return 1; }
void Arm_GripperSet(Arm_GripperStatusTypeDef status)
{ if (status == ARM_GRIPPER_CATCH) closures++; else if (status == ARM_GRIPPER_OPEN) openings++; }
#ifndef GRAB_COMMAND_SIM
bool ConsoleTx_Write(const uint8_t *text, uint16_t size)
{ assert(size < sizeof output); memcpy(output, text, size); output[size] = 0;
  if(strstr(output,"OK grab start")) { assert(size<sizeof accepted_log); memcpy(accepted_log,output,size+1); }
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
void Camera_RequestStop(void) {}
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
bool Mecanum_FeedbackGet(unsigned wheel, Mecanum_Feedback_t *out)
{
  assert(wheel < 4); memset(out, 0, sizeof *out);
  if(feedback_selected && wheel+1!=feedback_selected) return false;
  out->speed_valid = out->position_valid = out->state_valid = true;
  out->speed_ms = out->position_ms = out->state_ms = clock_ms;
  if(wheel_stale) out->speed_ms = out->position_ms = out->state_ms = clock_ms-3000U;
  out->position_raw=wheel_position[wheel];
  out->speed_rpm = wheel_moving ? 30 : 0; out->state_flags = 3; return true;
}
bool Mecanum_Velocity_Request(float f, float l, float yaw, uint8_t acc)
{ (void)f; (void)l; (void)yaw; (void)acc; velocity_requests++; bus.stage = MECANUM_STAGE_SENT; return true; }

static void setting(const char *key, const char *value)
{
  char *args[] = {"grab", "set", (char *)key, (char *)value};
  assert(GrabTask_Command(4, args)); assert(strstr(output, "ERR") == NULL);
}
static void reset(void)
{
  accepted_log[0]=progress_log[0]=0;
  clock_ms = 1000; pending_valid = false; positions = closures = openings = stops = 0;
  z_position_rpm=0; z_position_acc=0; z_position_pulses=0;
  requested_color=CAMERA_COLOR_RED; camera_requests = velocity_requests = 0; wheel_moving = false;
  camera_tx_result=HAL_OK; camera_attempts=0;
  wheel_stale = false; base_arrives=true; base_stale=false; base_moves=0; homes=aborts=0; home_flags=3; base_state_flags=3; boot_verify_calls=0; camera_logs=0; camera_log[0]=0;
  feedback_selected = selection_busy = 0;
  move_arrives = camera_valid = imu_valid = true;
  memset(physical, 0, sizeof physical); memset(destination, 0, sizeof destination);
  memset(wheel_position,0,sizeof wheel_position);
  memset(&camera, 0, sizeof camera); memset(&bus, 0, sizeof bus);
  bus.ack_profile = MECANUM_ACK_RECEIVE; GrabTask_Init();
  tray_ready=tray_arrives=true; tray_moving=false; tray_indexes=tray_stops=0; tray_unset_slot=0;
  memset(&tray_status,0,sizeof tray_status); tray_status.reason="idle"; tray_status.feedback_valid=true;
  for(unsigned i=0;i<3;i++) { tray_inventory[i].state=TURNTABLE_EMPTY; tray_inventory[i].color=0; }
}
static void configure(bool vision)
{
  setting("z_ppm","100"); setting("z_min","-20"); setting("z_max","20");
  setting("z_place","-5"); setting("z_grab","0"); setting("z_lift","10"); setting("pos_tol","0.1");
  setting("stop_speed","0.5"); setting("close_ms","300"); setting("feedback_ms","2000");
  setting("x_ppm","100"); setting("x_min","-20"); setting("x_max","20");
  setting("x_car","5"); setting("z_car_lift","10");
  setting("x_pre","0"); setting("z_observe","10"); setting("age_ms","500");
  if (!vision) return;
  /* Legacy fixtures retain their original response; boot-default tests below
   * exercise the current tuning without these explicit overrides. */
  setting("lambda","0.3"); setting("body_speed","10");
  setting("loss_grace_ms","0"); /* Legacy immediate-pause cases; grace has its own suite. */
  setting("accel","10"); setting("travel_mm","50"); setting("lead_mm","2");
  setting("dx_tol","2"); setting("dy_tol","2"); setting("ref_u","334"); setting("ref_v","338");
  setting("j00","1"); setting("j01","0"); setting("j02","0");
  setting("j10","0"); setting("j11","1"); setting("j12","1");
}
static void step(bool new_capture, unsigned phase, int dx, int dy)
{
  (void)phase;
  clock_ms += 50;
  if(tray_moving && tray_arrives) { tray_moving=false; tray_status.arrived=true; tray_status.reason="arrived"; }
  if (pending_valid)
  {
    MechanicalArm_EventTypeDef event = pending; unsigned axis = event.Axis == MECHANICAL_ARM_AXIS_BASE ? 2 : (event.Axis == MECHANICAL_ARM_AXIS_X ? 0 : 1);
    pending_valid = false;
    if (event.Action == MECHANICAL_ARM_ACTION_POSITION && move_arrives && (axis!=2 || base_arrives)) physical[axis] = destination[axis];
    if (event.Action == MECHANICAL_ARM_ACTION_POSITION_READ)
      event.CurrentPositionRaw = (int64_t)lroundf(physical[axis] * (axis==2 ? 3200.0f/360 : 100.0f) * 65536.0f / 3200.0f) *
        ((axis == 0 && MECHANICAL_ARM_X_POSITIVE_DIR) || (axis==2 && MECHANICAL_ARM_BASE_POSITIVE_DIR) ? -1 : 1);
    if(event.Axis!=MECHANICAL_ARM_AXIS_ALL) event.StateFlags[event.Axis] = event.Action==MECHANICAL_ARM_ACTION_HOME_STATE?home_flags:(axis==2?base_state_flags:3);
    if(!(base_stale && axis==2)) (void)GrabTask_MotorEventHandle(&event);
  }
  camera.UsbConfigured=1; camera.LastFrameTick=clock_ms;
  camera.RequestActive=1; camera.HasFrame=1; camera.HasValidData=camera.TargetValid=camera_valid;
  if(!camera_valid) camera.InvalidCount++;
  camera.RequestFunction=0xB2; camera.RequestTarget=requested_color; camera.Data.Function=0xB2;
  camera.Data.Target = requested_color;
  if(new_capture) { camera.Data.Sequence++; camera.Data.Tick = clock_ms; }
  camera.Data.CX = (uint16_t)(334 + dx); camera.Data.CY = (uint16_t)(338 - dy);

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
  run_to(GRAB_ERROR,100,1); assert(!closures);
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
  setting("x_car","0"); GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(GrabTask_Command(3,start));
  assert(strstr(output,"mode=align reason=config_missing"));
  assert(strstr(output,"missing=ref_u,ref_v,j00,j01,j02,j10,j11,j12 "));
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
  assert(solve(20,20,100,v) && v[0]<0 && v[1]<0 && v[2]<0);
  assert(solve(-20,-20,100,v) && v[0]>0 && v[1]>0 && v[2]==0);
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
  assert(!closures && !base_moves);
  run_to(GRAB_HOLD,250,2);
  assert(!closures && !base_moves && !strcmp(status().reason,"aligned_wait_check"));
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
  { unsigned i; for(i=0;i<5;i++) step(true,1,10,10); }
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
    /* SENT still reserves its first 50ms for readback: each blocked control
     * opportunity is followed by one free 100ms opportunity worth 0.105 pulse. */
    for(j=0;j<2;j++)
    { bus.stage=MECANUM_STAGE_SENT; bus.sent_ms=clock_ms+50; step(true,2,0,1); }
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
static void boot_homing(void)
{
  unsigned i;
  reset(); physical[2]=360; base_state_flags=1; GrabTask_BootHomeStart(); assert(GrabTask_IsBusy() && !GrabTask_Start("fixed"));
  home_flags=7;
  for(i=0;i<70;i++) step(false,0,0,0);
  assert(homes==1 && status().state==GRAB_HOMING); /* ACK / running flag cannot start X. */
  home_flags=3; physical[1]=2;
  for(i=0;i<20;i++) step(false,0,0,0);
  assert(homes==1); /* Idle homing flag without zero feedback cannot advance. */
  physical[1]=0; run_to(GRAB_IDLE,250,0);
  assert(boot_verify_calls==1 && homes==3 && !strcmp(status().reason,"home_complete") && Config[C_Z_GRAB]==-80 && Config[C_Z_PLACE]==-60);
  assert(!GrabTask_ConfigReady("fixed") && GrabTask_ConfigReady("align"));
  setting("x_car","0"); assert(GrabTask_ConfigReady("fixed"));
  for(i=0;i<50;i++) step(false,0,0,0);
  assert(homes==3);
  reset(); GrabTask_BootHomeStart(); home_flags=11; run_to(GRAB_ERROR,150,0);
  assert(homes==1 && aborts==1 && !boot_verify_calls && !strcmp(status().reason,"home_driver_fault"));
  reset(); GrabTask_BootHomeStart(); home_flags=7;
  for(i=0;i<30;i++) step(false,0,0,0);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(homes==1 && aborts==1);
  for(i=0;i<50;i++) step(false,0,0,0);
  assert(homes==1);
  reset(); GrabTask_BootHomeStart(); GrabTask_Stop(); run_to(GRAB_IDLE,100,0); assert(!homes && !aborts && !boot_verify_calls);
}
static void base_home_requires_stable_feedback(void)
{
  unsigned i;
  reset(); GrabTask_BootHomeStart();
  for(i=0;i<250 && BootAxisIndex!=2;i++) step(false,0,0,0);
  assert(BootAxisIndex==2 && !boot_verify_calls);
  for(i=0;i<40;i++) { physical[2]+=5; step(false,0,0,0); }
  assert(status().state==GRAB_HOMING && !boot_verify_calls);
  status_write(false);
  assert(strstr(output,"axis=base") && strstr(output,"raw=") && strstr(output,"home_flags=0x03"));
  run_to(GRAB_IDLE,100,0); assert(boot_verify_calls==1);
  for(i=0;i<40;i++) step(false,0,0,0);
  assert(boot_verify_calls==1);
  reset(); base_state_flags=0; GrabTask_BootHomeStart(); run_to(GRAB_ERROR,300,0);
  assert(!boot_verify_calls && !strcmp(status().reason,"home_axis_fault"));
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
  camera_valid=false; run_to(GRAB_ERROR,100,1); assert(!strcmp(status().reason,"vision_recover_timeout"));
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
  assert(status().recovery_left_ms==1500);
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
  home_flags=7;
  for(unsigned i=0;i<70;i++) step(false,0,0,0);
  assert(homes==1 && !GrabTask_ConfigReady("align"));
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
  run_to(GRAB_ERROR,80,1);
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
  for(i=0;i<2;i++) {
    pause_until_stopped(); step(true,1,20,20);
    assert(camera_requests==before+i+1 && !RecoveryCount);
    step(false,1,20,20); assert(!RecoveryCount); /* No new receive sequence after request. */
    run_to(GRAB_ALIGN,6,1); assert(RecoveryUsed==i+1);
  }
  camera.InvalidCount++; step(true,1,20,20);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"vision_recover_limit"));
  run_to(GRAB_ERROR,80,1); assert(camera_requests==before+2 && status().stop_confirmed);
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
  for(i=0;i<2;i++) {
    pause_until_stopped(); before=velocity_requests;
    step(false,1,20,20); assert(!RecoveryCount);
    step(true,1,20,20); assert(RecoveryCount==1);
    camera.InvalidCount++; step(true,1,20,20); assert(RecoveryCount==1);
    camera.Data.Sequence+=5; GrabTask_Process(); assert(RecoveryCount==2);
    assert(velocity_requests==before && !closures);
    RecoveryTick=clock_ms-1450; step(true,1,20,20); /* Exactly 1500 ms is accepted. */
    assert(status().state==GRAB_ALIGN && RecoveryUsed==i+1 && Retries==2 && !CaptureCount);
    assert(fabsf(XIntegral-Axes[0].mm)<.001f && !PreviousVelocity[0]);
  }
  camera.InvalidCount++; step(true,1,20,20);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"vision_recover_limit"));
  assert(!RecoveryWaiting && !closures); run_to(GRAB_ERROR,100,1);
  GrabTask_Stop(); run_to(GRAB_IDLE,100,1); step(false,1,0,0);
  assert(GrabTask_Start("align") && RecoveryUsed==0);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  pause_until_stopped(); step(true,1,20,20); step(true,1,20,20);
  RecoveryTick=clock_ms-1451; step(true,1,20,20);
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
  assert(solve(-95,0,0,v) && v[0]>19.9f && v[0]<=20.001f);
  assert(solve(-61,0,0,v) && v[0]>18.0f && v[0]<18.6f);
  assert(solve(-6,0,0,v) && v[0]>0 && v[0]<2.0f);
  assert(solve(95,0,0,v) && v[0]<-19.9f && v[0]>=-20.001f);
  assert(solve(-3000,-3000,50,v));
  assert(hypotf(v[0],v[1])<=20.001f && fabsf(v[2])<=5.001f);
  /* Exercise the production acceleration limiter over consecutive updates. */
  pending_valid=false; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  LastControl=clock_ms; Status.dx=-95; Status.dy=0; FreshVision=true;
  memset(PreviousVelocity,0,sizeof PreviousVelocity); XIntegral=Axes[0].mm;
  for(i=0;i<10;i++) {
    clock_ms+=100; align_control(clock_ms);
    assert(Status.forward_mm_s>previous && Status.forward_mm_s-previous<=2.001f);
    previous=Status.forward_mm_s;
  }
  assert(previous>19.9f && previous<=20.001f);
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

static void grab_z_speed_matches_250_rpm(void)
{
  unsigned i;
  reset(); GrabTask_BootHomeStart(); run_to(GRAB_IDLE,250,0);
  setting("x_car","0"); assert(GrabTask_Start("fixed")); run_to(GRAB_DESCEND,100,0);
  for(i=0;i<10 && !z_position_rpm;i++) step(true,0,0,0);
  /* 80 mm * 480 command pulses/mm, at the user's 250 RPM target. */
  assert(z_position_rpm==250 && z_position_pulses==-38400 && z_position_acc==8);
  assert(!closures); /* A submitted descent is not arrival. */
  pending_valid=false; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  assert(axis_move(1,-40));
  assert(z_position_rpm==250 && z_position_pulses==-19200 && z_position_acc==8);
  /* Preserve the existing mm/s setting, including the value sent by the UI. */
  reset(); configure(false); setting("z_ppm","480"); setting("z_speed","27.777778");
  assert(axis_move(1,-10)); assert(z_position_rpm==250 && z_position_pulses==-4800);
  reset(); configure(false); setting("z_ppm","480"); setting("z_speed","5");
  assert(axis_move(1,-10)); assert(z_position_rpm==45 && z_position_pulses==-4800);
}

int main(void)
{
  grab_z_speed_matches_250_rpm();
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

