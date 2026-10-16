/* Exercise actual byte input, dispatch and async prompts, with device boundary fakes. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../template/App/arm_console.c"
#include "../template/Hardware/QR.c"
static char output[16000];
static Camera_FrameObserver_t pixel_observer;
void Camera_FrameObserverSet(Camera_FrameObserver_t observer){pixel_observer=observer;}
static uint16_t pixel_backlog;
void Mecanum_StatusGet(Mecanum_Status_t *s) { memset(s,0,sizeof(*s)); }
bool MotorBus_FeedbackAllowed(void) { return true; }
static uint32_t tick = 100;
static unsigned reset_calls, zero_calls;
static bool urgent_idle=true;
static uint32_t urgent_losses;
void NVIC_SystemReset(void) { reset_calls++; }
bool ConsoleTx_UrgentIdle(void) { return urgent_idle; }
static unsigned camera_calls, motor_calls, stop_calls, notice_clears, manual_arm_stops;
static uint8_t motor_busy;
static bool calib_ref;
static bool grab_busy, grab_route_busy;
static bool mission_pending,mission_recovery_accepted,mission_feedback_allowed;
bool ContestMission_HasPendingWork(void){return mission_pending;}
bool ContestMission_FeedbackPreparationAllowed(void){return mission_feedback_allowed;}
bool ContestMission_BusRecoveryEvidenceGet(MotorBus_Recovery_t *out){(void)out;return false;}
bool ContestMission_RequestBusRecovery(bool auto_resume){assert(!auto_resume);return mission_pending && mission_recovery_accepted;}
static bool turntable_busy;
static unsigned turntable_stops, turntable_commands;
bool Turntable_IsBusy(void){return turntable_busy;}
void Turntable_Stop(void){turntable_stops++;turntable_busy=false;}
bool Turntable_Command(unsigned n,char *t[])
{
  if(n<2 || strcmp(t[0],"turntable"))return false;
  turntable_commands++;
  ArmConsole_Write("OK turntable fake\r\n");return true;
}
bool RadarConsole_Command(unsigned n,char *t[]){(void)n;(void)t;return false;}
static bool radar_scan_busy;
static uint16_t lidar_pulse;
static unsigned lidar_sets;
HAL_StatusTypeDef LidarServo_PulseSet(uint16_t pulse){lidar_sets++;lidar_pulse=pulse;return HAL_OK;}
uint16_t LidarServo_PulseGet(void){return lidar_pulse;}
bool RadarConsole_ScanBusy(void){return radar_scan_busy;}
static unsigned grab_stops;
static bool ring_manual;
bool GrabTask_RingManualAllowed(void){return ring_manual;}
bool GrabTask_IsBusy(void){return grab_busy;}
bool GrabRoute_IsBusy(void){return grab_route_busy;}
bool GrabRoute_CanConfigure(void){return false;}
void GrabTask_Stop(void){grab_stops++;grab_busy=false;}
void GrabRoute_Stop(void){grab_route_busy=false;if(grab_busy)GrabTask_Stop();}
static unsigned mission_pauses,mission_cancels,mission_fields;
bool GrabRoute_Command(unsigned n,char *t[]){
 if(n==4 && !strcmp(t[0],"grab") && !strcmp(t[1],"ring") && !strcmp(t[2],"take")){ArmConsole_Write("OK ring take routed\r\n");return true;}
 if(n==11 || n==12){
  assert(!strcmp(t[0],"grab") && !strcmp(t[1],"mission") && !strcmp(t[2],"fixed"));
  assert(!strcmp(t[8],"vhigh") && !strcmp(t[9],"linear") && !strcmp(t[10],"scale=1.2"));
  if(n==12)assert(!strcmp(t[11],"smooth=1"));
  mission_fields=n;return true;
 }
 if((n==9 || n==10) && !strcmp(t[0],"grab") && !strcmp(t[1],"mission") && !strcmp(t[2],"fixed"))
 {assert(!strcmp(t[8],"lowered"));if(n==10)assert(!strcmp(t[9],"cosine"));mission_fields=n;return true;}
 if(n==3 && !strcmp(t[0],"grab") && !strcmp(t[1],"mission") && (!strcmp(t[2],"pause") || !strcmp(t[2],"stop")))
 {if(!strcmp(t[2],"pause"))mission_pauses++;else mission_cancels++;return true;}
 if(n==3&&!strcmp(t[0],"grab")&&
 (!strcmp(t[1],"route")||!strcmp(t[1],"mission")||!strcmp(t[1],"qr"))&&!strcmp(t[2],"status"))
 {ArmConsole_Write("OK grab route query\r\n");return true;}return false;}
void GrabTask_StatusGet(GrabTask_Status_t *s){memset(s,0,sizeof(*s));}
bool GrabTask_Command(unsigned n,char *t[]){if(n==3&&!strcmp(t[0],"grab")&&!strcmp(t[1],"calib")){
 if(!strcmp(t[2],"status"))ArmConsole_Write("OK grab calib phase=collect ref=1 calibrated=0 reason=none fdx=na fdy=na xdx=na xdy=na\r\n");
 else if(!strcmp(t[2],"ref")){calib_ref=true;ArmConsole_Write("OK grab calib ref confirmed\r\n");}
 else {grab_busy=true;ArmConsole_Write("OK grab calib accepted\r\n");}return true;}
if(n>=3&&!strcmp(t[0],"grab")&&!strcmp(t[1],"stack")){const char*s="OK grab stack routed\r\n";return ConsoleTx_Write((const uint8_t*)s,(uint16_t)strlen(s));}if(n>=3&&!strcmp(t[0],"grab")&&!strcmp(t[1],"track")){const char*s="OK grab track routed\r\n";return ConsoleTx_Write((const uint8_t*)s,(uint16_t)strlen(s));}if(n==2&&!strcmp(t[0],"grab")&&!strcmp(t[1],"status")){const char*s="OK grab state=hold\r\n";return ConsoleTx_Write((const uint8_t*)s,(uint16_t)strlen(s));}return false;}
static bool motion_busy;
static bool continuous_velocity;
static bool bus_locked;
static MotorBus_Recovery_t recovery_fake={.reason="idle"};
static MotorBus_Event_t guard_event;
static bool guard_ready, guard_moving;
static bool probe_active,probe_event_ready;
static uint8_t probe_id;
static MotorBus_Event_t probe_event;
bool MotorBus_ProbeStart(uint8_t address){if(probe_active)return false;probe_active=true;probe_id=address;return true;}
bool MotorBus_ProbeReady(void){return probe_active;}
bool MotorBus_ProbeActive(void){return probe_active;}
void MotorBus_ProbeStop(void){probe_active=false;probe_event_ready=false;}
bool MotorBus_RecoveryStart(void) { recovery_fake.active=true;return true; }
void MotorBus_RecoveryCancel(void) { recovery_fake.active=false; }
void MotorBus_RecoveryGet(MotorBus_Recovery_t *s) { *s=recovery_fake; }
bool Mecanum_RecoveryAfterBusCheck(void) { return true; }
static bool mode_feedback_enabled=true,mode_feedback_valid=true;
static uint8_t mode_feedback_address=3;
static int mode_feedback_rpm;
static MotorBus_ModeTestStatus_t mode_fake;
void Mecanum_Feedback_Enable(bool enabled) { mode_feedback_enabled=enabled; }
void Mecanum_FeedbackSettingsGet(bool *enabled,uint8_t *address)
{if(enabled)*enabled=mode_feedback_enabled;if(address)*address=mode_feedback_address;}
bool Mecanum_Feedback_Select(uint8_t address){mode_feedback_address=address;mode_feedback_enabled=false;return address<=4;}
bool Mecanum_FeedbackGet(unsigned wheel,Mecanum_Feedback_t *f)
{(void)wheel;memset(f,0,sizeof *f);f->speed_valid=f->position_valid=f->state_valid=mode_feedback_valid;
 f->speed_ms=f->position_ms=f->state_ms=tick;f->state_flags=3;f->speed_rpm=mode_feedback_rpm;return true;}
bool MotorBus_ModeTestActive(void){return mode_fake.active;}
bool MotorBus_ModeTestStart(uint8_t target,uint16_t gap)
{if(mode_fake.active)return false;memset(&mode_fake,0,sizeof mode_fake);mode_fake.active=true;mode_fake.target=target;mode_fake.gap_ms=gap;mode_fake.reason="snapshot";return true;}
void MotorBus_ModeTestProcess(void){}
void MotorBus_ModeTestCancel(void){mode_fake.active=false;mode_fake.cancelled=mode_fake.restored=true;mode_fake.reason="cancelled_restored";}
void MotorBus_ModeTestGet(MotorBus_ModeTestStatus_t *s){*s=mode_fake;}
uint32_t MotorBus_ReplySettleMs(void){return mode_fake.active?mode_fake.gap_ms:2;}
void MotorBus_Cancel(MotorBus_Owner_t owner) { (void)owner;guard_ready=false; }
bool MotorBus_OwnerBusy(MotorBus_Owner_t owner) { (void)owner;return guard_ready; }
bool MotorBus_Submit(MotorBus_Owner_t owner,const uint8_t *p,uint8_t n,uint8_t reply,uint32_t token,bool priority)
{
  if(owner==MOTOR_BUS_PROBE) {
    assert(probe_active && p[0]==probe_id && n==(p[1]==0x42?4:3) && !priority);
    memset(&probe_event,0,sizeof probe_event);probe_event.result=MOTOR_BUS_REPLY;
    probe_event.length=reply;probe_event.data[0]=p[0];probe_event.data[1]=p[1];
    probe_event.data[reply-1]=0x6b;probe_event.data[2]=p[1]==0x3a?3:0;
    if(p[1]==0x42) { probe_event.data[2]=33;probe_event.data[3]=21; }
    if(p[1]==0x27) { probe_event.data[2]=0x09;probe_event.data[3]=0xc4; }
    if(p[1]==0x24) { probe_event.data[2]=0x5d;probe_event.data[3]=0xc0; }
    if(p[1]==0x37) { probe_event.data[2]=1;probe_event.data[4]=1; }
    if(p[1]==0x22) {
      probe_event.data[5]=30;probe_event.data[8]=0x27;probe_event.data[9]=0x10;
    }
    if(p[1]==0x42) {
      probe_event.data[14]=0x0b;probe_event.data[15]=0xb8;probe_event.data[31]=8;
    }
    probe_event.token=token;probe_event.tx_completed=true;probe_event.tx_ms=tick;probe_event.tick_ms=tick+2;
    probe_event_ready=true;return true;
  }
  (void)n;(void)reply;(void)token;(void)priority;assert(owner==MOTOR_BUS_GUARD);
  memset(&guard_event,0,sizeof(guard_event));guard_event.result=MOTOR_BUS_REPLY;
  guard_event.data[0]=p[0];guard_event.data[1]=p[1];
  if(p[1]==0x35)guard_event.data[4]=guard_moving?10:0;
  else guard_event.data[2]=3;
  guard_ready=true;return true;
}
bool MotorBus_EventGet(MotorBus_Owner_t owner,MotorBus_Event_t *e)
{
  if(owner==MOTOR_BUS_PROBE){if(!probe_event_ready)return false;*e=probe_event;probe_event_ready=false;return true;}
  if(!guard_ready)return false;
  *e=guard_event;guard_ready=false;return true;
}
bool MotorBus_IsQuarantined(void) { return bus_locked; }
void MotorBus_TimingGet(MotorBus_Timing_t *out) { memset(out,0,sizeof *out); }
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *out)
{
  memset(out,0,sizeof(*out));out->locked=bus_locked;
  out->reason=bus_locked?"uart_error":"none";
  out->tick_ms=12345;out->uart_error=4;
  out->owner=MOTOR_BUS_ARM;out->address=7;out->function=0x3a;
}
static bool rx_error_fake_valid;
static MotorBus_RxErrorSnapshot_t rx_error_fake;
bool MotorBus_RxErrorGet(MotorBus_RxErrorSnapshot_t *out,bool first)
{(void)first;if(!out || !rx_error_fake_valid)return false;*out=rx_error_fake;return true;}
static float forward_value, left_value, yaw_value, grip_duty;
static int32_t motor_pulses;
static MechanicalArm_ConfigTypeDef config = {60, 20, 3200};
static Camera_SnapshotTypeDef snapshot;
static HAL_StatusTypeDef camera_result = HAL_OK;
static char qr_event[128];
static uint32_t qr_event_count, qr_event_dropped;
static MechanicalArm_ResultTypeDef motor_result = MECHANICAL_ARM_RESULT_NONE;
uint32_t HAL_GetTick(void) { return tick; }
void ConsoleTx_Init(UART_HandleTypeDef *uart) {(void)uart;}
bool ConsoleTx_Write(const uint8_t *data,uint16_t length)
{assert(strlen(output)+length<sizeof(output));strncat(output,(const char*)data,length);return true;}
bool ConsoleTx_Urgent(const char *data,uint16_t length){return ConsoleTx_Write((const uint8_t *)data,length);}
bool ConsoleTx_Debug(unsigned source,const char *data,uint16_t length){(void)source;return ConsoleTx_Write((const uint8_t *)data,length);}
void ConsoleTx_BackgroundPause(bool paused){(void)paused;}
void ConsoleTx_DebugCancel(unsigned source){(void)source;}
void ConsoleTx_GetStats(ConsoleTx_Stats_t *out){memset(out,0,sizeof(*out));out->urgent_dropped=urgent_losses;out->reply_pending=pixel_backlog;}
void ConsoleRx_GetStats(ConsoleRx_Stats_t *out){memset(out,0,sizeof(*out));}
uint32_t ConsoleTx_Dropped(void){return 0;}
bool ConsoleTx_Event(const char *data,uint16_t length)
{ assert(length<sizeof(qr_event));if(qr_event[0])qr_event_dropped++;memcpy(qr_event,data,length);qr_event[length]=0;qr_event_count++;return true; }
void ConsoleTx_EventCancel(void) {if(qr_event[0])qr_event_dropped++;qr_event[0]=0;}
uint32_t ConsoleTx_EventDropped(void) {return qr_event_dropped;}
static unsigned feedback_queries,feedback_commands;
bool ChassisTelemetry_Command(unsigned count,char *tokens[]) {
 if(count<2 || strcmp(tokens[0],"chassis"))return false;
 if(count==2 && !strcmp(tokens[1],"task"))
 {ArmConsole_Write("OK chassis task query\r\n");return true;}
 if(!strcmp(tokens[1],"feedback")) {
   if(count==2){feedback_queries++;ArmConsole_Write("OK wheel=1 cached\r\n");return true;}
   if(count==3 && (!strcmp(tokens[2],"on") || !strcmp(tokens[2],"off") ||
      (strlen(tokens[2])==1 && tokens[2][0]>='0' && tokens[2][0]<='4')))
   {feedback_commands++;ArmConsole_Write("OK chassis request accepted\r\n");return true;}
 }
 return false;}
static bool route_reserved;
bool ChassisRoute_IsBusy(void){return route_reserved;}
bool ChassisRoute_Cancel(void){route_reserved=false;motion_busy=false;return true;}
bool ChassisRoute_Command(unsigned n,char *t[]){if(n==3&&!strcmp(t[0],"chassis")&&!strcmp(t[1],"route")&&!strcmp(t[2],"status"))
 {ArmConsole_Write("OK chassis route query\r\n");return true;}return false;}
bool ChassisMotion_IsBusy(void) {return motion_busy;}
bool ChassisMotion_Stop(uint32_t ms) {(void)ms;motion_busy=false;return true;}
static ChassisStop_Status_t stop_status = {0,0,0,false,false,"idle"};
bool ChassisMotion_StopRequest(uint32_t token)
{
  stop_status.id++;stop_status.token=token;stop_status.requested_ms=tick;
  stop_status.tx_complete=stop_status.wheels_stopped=false;stop_status.reason="pending_tx";
  motion_busy=false;return Mecanum_Test_Stop();
}
bool ChassisMotion_StopPending(void){return stop_status.id && !stop_status.wheels_stopped;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *s)
{ memset(s,0,sizeof(*s));s->indefinite_velocity=continuous_velocity; }
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *out){*out=stop_status;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length, uint32_t timeout)
{ (void)uart; assert((uint32_t)length * 10U * 1000U <= 115200U * timeout);
  assert(strlen(output)+length < sizeof(output)); strncat(output,(const char *)data,length); return HAL_OK; }
void OledUi_NoticeSet(const char *text) { if (!text) notice_clears++; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out) { *out = snapshot; }
Camera_VisualStateTypeDef Camera_VisualStateGet(const Camera_SnapshotTypeDef *data)
{
  if (!data->RequestActive) return CAMERA_VIS_IDLE;
  if (!data->UsbConfigured) return CAMERA_VIS_OFF;
  if (!data->HasFrame) return CAMERA_VIS_WAIT;
  if ((uint32_t)(tick-data->LastFrameTick)>CAMERA_DATA_STALE_MS) return CAMERA_VIS_STALE;
  return data->TargetValid ? CAMERA_VIS_OK : CAMERA_VIS_LOST;
}
const char *Camera_VisualStateNameGet(Camera_VisualStateTypeDef state)
{ static const char *const names[]={"Idle","Off","Wait","Ok","Lost","Stale"};return names[state]; }
HAL_StatusTypeDef Camera_RingStart(uint8_t ring)
{ return Camera_MaterialStart((Camera_ColorTypeDef)(9U+ring)); }
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef color)
{ camera_calls++; if (camera_result == HAL_OK) snapshot.RequestTarget = color; return camera_result; }
void Camera_RequestStop(void) { snapshot.RequestActive = 0; }
bool HWT101_Is_Ready(void) { return true; }
bool HWT101_Angle_Get(HWT101_Angle_t *out) { memset(out,0,sizeof(*out)); out->yaw=-12.5f;out->update_count=44;return true; }
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *out,uint32_t age) { (void)out;(void)age;return true; }
bool HWT101_Status_Get(HWT101_Status_t *out) { memset(out,0,sizeof(*out));out->valid_read_count=44;out->i2c_error_count=UINT32_MAX;return true; }
static HWT101_CalStatus_t cal_status;
static unsigned cal_starts, cal_cancels, refresh_calls;
static unsigned quick_cal_starts;
static uint32_t verify_duration;
static bool chassis_busy;
bool HWT101_Cal_IsBusy(void) { return cal_status.busy; }
bool HWT101_Cal_ZeroStart(void) { zero_calls++; cal_status.busy=true; return true; }
bool HWT101_Cal_Start(void)
{
  assert(!chassis_busy && !motor_busy && !grab_busy);
  cal_starts++; cal_status.busy=true; cal_status.state=HWT101_CAL_CALIBRATING; cal_status.native_wait_ms=20000; return true;
}
bool HWT101_Cal_QuickStart(void)
{
  assert(!chassis_busy && !motor_busy && !grab_busy);
  quick_cal_starts++;cal_status.busy=true;cal_status.state=HWT101_CAL_CALIBRATING;
  cal_status.native_wait_ms=20000;cal_status.verify_ms=10000;return true;
}
bool HWT101_Cal_VerifyStart(uint32_t duration)
{
  assert(!chassis_busy && !motor_busy && !grab_busy);
  verify_duration=duration; cal_status.busy=true; cal_status.state=HWT101_CAL_VERIFYING;return true;
}
void HWT101_Cal_Cancel(void) { cal_cancels++;memset(&cal_status,0,sizeof(cal_status)); }
HAL_StatusTypeDef HWT101_Cal_RefreshRegisters(void) { assert(!cal_status.busy);refresh_calls++;return HAL_OK; }
bool HWT101_Cal_GetStatus(HWT101_CalStatus_t *out)
{
  *out=cal_status;out->state_name="CALIBRATING";out->reason="none";out->save_state="NOT_REQUESTED";
  out->result=cal_status.state==HWT101_CAL_DONE?"PASS":cal_status.state==HWT101_CAL_FAILED?"FAIL":cal_status.busy?"PENDING":"NONE";
  return true;
}
bool TJC_Status_Get(TJC_Status_t *out) { memset(out,0,sizeof(*out));out->tx_count=UINT32_MAX;out->rx_frame_count=12345;return true; }
void Arm_GripperSet(Arm_GripperStatusTypeDef status) { grip_duty=(float)status; }
void Arm_GripperDutySet(float duty) { grip_duty=duty; }
bool ChassisMotion_VelocityStart(float forward,float left,float yaw)
{ if(motion_busy)return false;motion_busy=continuous_velocity=true;forward_value=forward;left_value=left;yaw_value=yaw;motor_calls++;return true; }
bool ChassisMotion_VelocityUpdate(float forward,float left,float yaw)
{ if(!motion_busy || !continuous_velocity)return false;forward_value=forward;left_value=left;yaw_value=yaw;motor_calls++;return true; }
static ChassisMotion_HoldStatus_t heading_status={false,"idle",0,0,0};
bool ChassisMotion_HoldStart(float speed,uint32_t duration)
{
  (void)duration;
  if(cal_status.state!=HWT101_CAL_DONE){heading_status.reason="imu_not_verified";return false;}
  forward_value=speed;motor_calls++;motion_busy=true;heading_status.active=true;heading_status.reason="running";return true;
}
bool Mecanum_IsBusy(void) { return chassis_busy || heading_status.active; }
void ChassisMotion_HoldStatusGet(ChassisMotion_HoldStatus_t *s){*s=heading_status;}
bool Mecanum_Test_Stop(void) { stop_calls++;motion_busy=continuous_velocity=false;heading_status.active=false;return true; }
bool Mecanum_Polarity_Move(float distance) { forward_value=distance;motor_calls++;return true; }
bool Mecanum_Wheel_Test(MecanumWheel_t wheel,int16_t rpm) { (void)wheel;forward_value=(float)rpm;motor_calls++;return true; }
uint8_t MechanicalArm_IsBusy(void) { return motor_busy; }
MechanicalArm_AxisTypeDef MechanicalArm_AxisGet(const char *name)
{ if (!strcmp(name,"base"))return MECHANICAL_ARM_AXIS_BASE;if(!strcmp(name,"z"))return MECHANICAL_ARM_AXIS_Z;if(!strcmp(name,"x"))return MECHANICAL_ARM_AXIS_X;if(!strcmp(name,"all"))return MECHANICAL_ARM_AXIS_ALL;return MECHANICAL_ARM_AXIS_INVALID; }
const char *MechanicalArm_AxisNameGet(MechanicalArm_AxisTypeDef axis)
{ const char *names[]={"base","z","x","all","invalid"};return names[axis]; }
MechanicalArm_ResultTypeDef MechanicalArm_Enable(MechanicalArm_AxisTypeDef axis) { (void)axis;motor_calls++;return motor_result; }
MechanicalArm_ResultTypeDef MechanicalArm_Disable(MechanicalArm_AxisTypeDef axis) { return MechanicalArm_Enable(axis); }
MechanicalArm_ResultTypeDef MechanicalArm_PositionRead(MechanicalArm_AxisTypeDef axis) { return MechanicalArm_Enable(axis); }
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef axis) { return MechanicalArm_Enable(axis); }
MechanicalArm_ResultTypeDef MechanicalArm_OriginSet(MechanicalArm_AxisTypeDef axis) { return MechanicalArm_Enable(axis); }
MechanicalArm_ResultTypeDef MechanicalArm_Zero(MechanicalArm_AxisTypeDef axis) { return MechanicalArm_Enable(axis); }
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis) { (void)axis;manual_arm_stops++;stop_calls++;return motor_result; }
MechanicalArm_ResultTypeDef MechanicalArm_Home(MechanicalArm_AxisTypeDef axis,uint8_t mode) { (void)mode;return MechanicalArm_Enable(axis); }
MechanicalArm_ResultTypeDef MechanicalArm_Position(MechanicalArm_AxisTypeDef axis,int32_t pulses) { motor_pulses=pulses;return MechanicalArm_Enable(axis); }
uint8_t MechanicalArm_ConfigGet(MechanicalArm_AxisTypeDef axis,MechanicalArm_ConfigTypeDef *out) { (void)axis;*out=config;return 1; }
uint8_t MechanicalArm_ConfigSet(MechanicalArm_AxisTypeDef axis,uint16_t rpm,uint8_t acceleration,uint32_t limit)
{ (void)axis;config.SpeedRpm=rpm;config.Acceleration=acceleration;config.PulseLimit=limit;return 1; }
static void command(const char *text)
{ output[0]=0; while(*text)ArmConsole_ReceiveData((uint8_t)*text++);ArmConsole_Process();
  for(unsigned i=0;i<12 && ManualGuardActive;i++)ArmConsole_Process(); }
static unsigned occurrences(const char *text)
{ unsigned count=0;const char *at=output;while((at=strstr(at,text))!=NULL){count++;at+=strlen(text);}return count; }
/* The real console, including its early busy guards, must admit only the
 * recovery owner's feedback preparation. Mission stop-proof policy is covered
 * by contest_imu_pause_test; this fake controls that public decision. */
static void test_paused_feedback_admission(void)
{
  unsigned queries=feedback_queries,writes=feedback_commands,motors=motor_calls;
  unsigned stops=stop_calls;
  mission_pending=grab_route_busy=true;
  mission_feedback_allowed=true; /* Preserved PAUSED mission, current stop proven. */
  command("chassis feedback\r");assert(feedback_queries==++queries && !strstr(output,"ERR"));
  command("chassis feedback 0\r");assert(feedback_commands==++writes && !strstr(output,"ERR"));
  command("chassis feedback on\r");assert(feedback_commands==++writes && !strstr(output,"ERR"));
  assert(mission_pending && grab_route_busy && motor_calls==motors && stop_calls==stops);
  {
    const char *blocked[]={"chassis feedback off\r","chassis feedback 1\r",
      "chassis feedback 4\r","chassis feedback 00\r","chassis feedback 0 extra\r",
      "chassis origin 0 0 0\r","chassis velocity 1 0 0\r","pos x 10\r"};
    for(unsigned i=0;i<sizeof blocked/sizeof blocked[0];i++) {
      command(blocked[i]);assert(strstr(output,"grab_busy"));
      assert(feedback_commands==writes && motor_calls==motors && stop_calls==stops);
    }
  }
  mission_feedback_allowed=false; /* PAUSED without current parking proof. */
  command("chassis feedback 0\r");assert(strstr(output,"grab_busy") && feedback_commands==writes);
  command("chassis feedback on\r");assert(strstr(output,"grab_busy") && feedback_commands==writes);
  motion_busy=route_reserved=true; /* Running mission. */
  command("chassis feedback\r");assert(feedback_queries==++queries && !strstr(output,"ERR"));
  command("chassis feedback on\r");assert(strstr(output,"grab_busy") && feedback_commands==writes);
  motion_busy=route_reserved=false;
  recovery_fake.active=true; /* Mission recovery owns all bus activity. */
  command("chassis feedback\r");assert(feedback_queries==++queries && !strstr(output,"ERR"));
  command("chassis feedback 0\r");assert(strstr(output,"grab_busy") && feedback_commands==writes);
  recovery_fake.active=false;
  BusRecoveryPending=2;recovery_fake.active=true; /* Ordinary console recovery. */
  command("chassis feedback\r");assert(feedback_queries==++queries && !strstr(output,"ERR"));
  command("chassis feedback on\r");assert(strstr(output,"operation_busy") && feedback_commands==writes);
  BusRecoveryPending=0;recovery_fake.active=false;
  ResetPending=1; /* Cached data also remains readable during a pending reset. */
  command("chassis feedback\r");assert(feedback_queries==++queries && !strstr(output,"ERR"));
  ResetPending=0;
  /* A newer stop revokes the mission API decision before a queued enable. */
  mission_feedback_allowed=true;
  command("chassis feedback 0\r");assert(feedback_commands==++writes);
  mission_feedback_allowed=false;
  stop_status.tx_complete=stop_status.wheels_stopped=false;
  command("chassis feedback on\r");assert(strstr(output,"grab_busy") && feedback_commands==writes);
  mission_pending=false; /* Cancelled mission still completing its owned stop. */
  command("chassis feedback on\r");assert(strstr(output,"grab_busy") && feedback_commands==writes);
  grab_route_busy=false; /* Ordinary idle keeps the original preparation API. */
  command("chassis feedback 0\r");assert(feedback_commands==++writes && !strstr(output,"ERR"));
  command("chassis feedback on\r");assert(feedback_commands==++writes && !strstr(output,"ERR"));
  command("chassis feedback off\r");assert(feedback_commands==++writes && !strstr(output,"ERR"));
  assert(motor_calls==motors && stop_calls==stops);
}
static void qr_receive(const char *text) {while(*text)QR_ReceiveData((uint8_t)*text++);}
static void test_qr(void)
{
  unsigned mode;
  char code[QR_TASK_CODE_BUFFER_SIZE];
  command("qr status\r");assert(strstr(output,"OK qr status valid=0 seq=0 age_ms=NA received=0 accepted=0 rejected=0 uart_errors=0 stream=off event_dropped=0 source=none\r\n"));
  command("qr read\r");assert(strstr(output,"OK qr read valid=0 seq=0 code=NA age_ms=NA source=none\r\n"));
  qr_receive("123+231+312+321\r\n");
  tick+=10;
  command("qr read\r");assert(strstr(output,"seq=1 code=123+231+312+321 age_ms=10"));
  command("qr read\r");assert(strstr(output,"seq=1 code=123+231+312+321 age_ms=10"));
  assert(QR_TaskCodeGet(code) && !QR_TaskCodeGet(code));
  for(mode=0;mode<3;mode++)
  {
    cal_status.busy=mode==0;motion_busy=mode==1;heading_status.active=mode==2;grab_busy=mode==3;
    command("qr status\r");assert(strstr(output,"OK qr status"));
    command("qr read\r");assert(strstr(output,"OK qr read"));
    command("qr stream on\r");assert(strstr(output,"OK qr stream=on"));
    command("qr stream off\r");assert(strstr(output,"OK qr stream=off"));
    assert(!camera_calls && !motor_calls && !stop_calls);
  }
  cal_status.busy=false;motion_busy=false;heading_status.active=false;grab_busy=0;
  command("qr stream invalid\r");assert(strstr(output,"ERR qr format"));
  command("qr status extra\r");assert(strstr(output,"ERR qr format"));
  command("qr stream on\r");assert(qr_event_count==0); /* No replay of cached code. */
  qr_receive("111+222+333+444\r");ArmConsole_Process();assert(qr_event_count==0);
  tick+=100;qr_receive("444+333+222+111\r");ArmConsole_Process();assert(qr_event_count==0);
  tick+=100;ArmConsole_Process();assert(qr_event_count==1);
  assert(strstr(qr_event,"EVT qr code seq=3 code=444+333+222+111 age_ms=100 source=uart\r\n"));
  command("qr status\r");assert(strstr(output,"event_dropped=1"));
  qr_receive("111+222+333+444\r");ArmConsole_Process();assert(qr_event_count==1);
  command("qr stream off\r");assert(!qr_event[0]);
  tick+=500;ArmConsole_Process();assert(qr_event_count==1);
  command("qr status\r");assert(strstr(output,"event_dropped=3"));
  /* Sequence and elapsed arithmetic must handle the MCU tick rollover. */
  tick=UINT32_MAX-100U;command("qr stream on\r");
  qr_receive("111+222+333+444\r");tick=99U;ArmConsole_Process();
  assert(qr_event_count==2 && strstr(qr_event,"age_ms=200"));
  command("qr stream off\r");
  /* Longest counter values remain a complete machine-readable line. */
  QR_Latest.Sequence=QR_Latest.Received=QR_Latest.Accepted=QR_Latest.Rejected=QR_Latest.UartErrors=UINT32_MAX;
  QR_Latest.ReceivedTick=tick+1U;
  ArmConsole_QrSkipped=UINT32_MAX;qr_event_dropped=0;
  command("qr status\r");
  assert(strstr(output,"seq=4294967295 age_ms=4294967295") && strstr(output,"event_dropped=4294967295 source=uart\r\n"));
  ArmConsole_QrSkipped=0;
  QR_Init();tick=100;
}
int main(void)
{
  {
    char qr[]="grab mission qr 500 100 lower fixed";
    char fixed[]="grab mission fixed 156+123+516+231 500 100 upper radar";
    char raised[]="grab mission fixed 156+123+516+231 500 100 upper radar raised";
    char extra[]="grab mission fixed 156+123+516+231 500 100 upper radar raised cosine extra overflow overlimit";
    char *tokens[ARM_CONSOLE_TOKEN_COUNT];
    assert(ArmConsole_Tokenize(qr,tokens)==7);
    assert(!strcmp(tokens[5],"lower") && !strcmp(tokens[6],"fixed"));
    assert(ArmConsole_Tokenize(fixed,tokens)==8);
    assert(!strcmp(tokens[3],"156+123+516+231") && !strcmp(tokens[6],"upper") && !strcmp(tokens[7],"radar"));
    assert(ArmConsole_Tokenize(raised,tokens)==9 && !strcmp(tokens[8],"raised"));
    assert(ArmConsole_Tokenize(extra,tokens)==0);
    char asymmetric[]="chassis test 1000 0 1000 linear 500 0.02 0 1 300";
    assert(ArmConsole_Tokenize(asymmetric,tokens)==11 && !strcmp(tokens[10],"300"));
  }
  UART_HandleTypeDef uart={0};MechanicalArm_EventTypeDef event={0};unsigned before;
  uart.Instance=USART1;ArmConsole_Init(&uart);BusAutoFaultSeen=true;BusAutoFaultTick=12345; /* Existing guards use an already acknowledged fault. */
  test_paused_feedback_admission();
  snapshot.UsbConfigured=1;snapshot.RequestActive=1;
  command("grab mission fixed 156+123+516+231 5000 100 upper radar lowered\r");
  assert(mission_fields==9 && !strstr(output,"ERR")); /* Full 63-byte payload plus CR. */
  command("grab mission fixed 156+123+516+231 1000 100 upper fixed lowered cosine\r");
  assert(mission_fields==10 && !strstr(output,"ERR"));
  command("grab mission fixed 156+123+516+231 1000 100 lower fixed vhigh linear scale=1.2 smooth=1\r");
  assert(mission_fields==12 && !strstr(output,"ERR"));
  command("grab mission fixed 156+123+516+231 1000 100 lower fixed vhigh linear scale=1.2\r");
  assert(mission_fields==11 && !strstr(output,"ERR"));
  command("lidarservo status\r");assert(lidar_sets==0 && strstr(output,"enabled=0 pulse_us=0"));
  for(unsigned pulse=500;pulse<=5000;pulse+=500) {
    char wire[40];snprintf(wire,sizeof wire,"lidarservo pulse %u\r",pulse);
    command(wire);assert(lidar_pulse==pulse && strstr(output,"OK lidarservo enabled=1"));
  }
  before=lidar_sets;
  const char *invalid[]={"499","5001","0","1500.5","nan","1000 extra","9999999999999999999"};
  for(unsigned i=0;i<sizeof invalid/sizeof invalid[0];i++) {
    char wire[64];snprintf(wire,sizeof wire,"lidarservo pulse %s\r",invalid[i]);
    command(wire);assert(lidar_sets==before && lidar_pulse==5000 && strstr(output,"ERR lidarservo"));
  }
  command("lidarservo out\r");assert(lidar_pulse==2450 && strstr(output,"pulse_us=2450"));
  command("lidarservo in\r");assert(lidar_pulse==1450 && strstr(output,"pulse_us=1450"));
  before=lidar_sets;
  grab_busy=grab_route_busy=motion_busy=radar_scan_busy=bus_locked=true;
  command("lidarservo out\r");assert(lidar_sets==before && strstr(output,"task_busy"));
  command("lidarservo in\r");assert(lidar_sets==before && strstr(output,"task_busy"));
  command("lidarservo off\r");assert(lidar_sets==before && lidar_pulse==1450 && strstr(output,"format"));
  assert(grab_busy && grab_route_busy && motion_busy && radar_scan_busy && bus_locked);
  command("lidarservo status\r");assert(strstr(output,"enabled=1 pulse_us=1450"));
  grab_busy=grab_route_busy=motion_busy=radar_scan_busy=bus_locked=false;
  command("lidarservo pulse 1537\r");assert(lidar_pulse==1537 && strstr(output,"pulse_us=1537"));
  command("lidarservo off\r");assert(lidar_pulse==1537 && strstr(output,"format"));
  {
    const uint8_t frame[]={0xFF,0xB2,3,1,1,0x4E,1,0x52,0xFF,0xDA,0,7,0,0xFE};
    unsigned cameras=camera_calls,motors=motor_calls;
    command("camera log start\r");assert(pixel_observer && strstr(output,"log_begin id=1 next=0"));
    pixel_observer(frame,123,1);pixel_observer(frame,124,0);
    assert(strstr(output,"PIX id=1 i=0 t=123 c=3 v=1 m=1 x=334 y=338 dx=-38 dy=7 crc="));
    assert(strstr(output,"PIX id=1 i=1 t=124 c=3 v=1 m=0 x=334 y=338 dx=-38 dy=7 crc="));
    command("camera log start\r");assert(strstr(output,"id=1 next=2"));
    pixel_backlog=4097;pixel_observer(frame,125,1);pixel_backlog=0;
    command("camera log stop\r");assert(!pixel_observer && strstr(output,"count=3 queued=2 dropped=1"));
    assert(camera_calls==cameras && motor_calls==motors);
  }
  QR_Init();test_qr();
  {
    const char *retired[]={"vision ref\r","vision status\r","vision material 1\r","vision calib material 1\r","vision stop\r","material ref\r","material auto 3\r","material calib 3\r","material status\r","material stop\r"};
    unsigned motors=motor_calls,cameras=camera_calls,stops=stop_calls;
    for(unsigned i=0;i<sizeof(retired)/sizeof(retired[0]);i++)
    {
      command(retired[i]);assert(strstr(output,"ERR retired") && strstr(output,"grab align"));
      assert(motor_calls==motors && camera_calls==cameras && stop_calls==stops);
    }
  }
  command("grab calib ref\r");assert(calib_ref && strstr(output,"OK grab calib ref confirmed"));
  radar_scan_busy=true;
  command("grab mission 200\r");assert(strstr(output,"stationary capture active"));
  command("grab pose work\r");assert(strstr(output,"stationary capture active"));
  radar_scan_busy=false;
  motion_busy=true;
  command("grab calib 1\r");assert(strstr(output,"other_task_busy") && !grab_busy);
  command("imu cal start\r");assert(strstr(output,"chassis_task_busy"));
  command("wheel stop\r");assert(!motion_busy && strstr(output,"OK chassis stop id="));
  snapshot.RxByteCount=UINT32_MAX;snapshot.Data.CX=65535;snapshot.Data.DX=-32768;snapshot.Data.Sequence=7;
  command("help\r\n");assert(occurrences("arm> ")==1);assert(strstr(output,"camera status"));assert(!strstr(output,"vision ring <"));
  assert(!strstr(output,"vision calib") && !strstr(output,"vision material") && !strstr(output,"vision ref"));
  snapshot.Data.Target=2; snapshot.Data.Function=0xB2; snapshot.HasFrame=1; snapshot.LastFrameTick=80;
  snapshot.HasValidData=snapshot.TargetValid=1;
  command("camera status\r");assert(strstr(output,"4294967295"));assert(camera_calls==0 && motor_calls==0);
  assert(strstr(output,"cx=65535") && strstr(output,"dx=-32768") && strstr(output,"seq=7"));
  assert(strstr(output,"last fn=178 target=2") && strstr(output,"frame_age_ms=20"));
  assert(strstr(output,"protocol=B2 age_source=rx"));
  command("camera status\r");
  assert(strstr(output,"state=Ok") && strstr(output,"cx=65535") && strstr(output,"seq=7"));
  assert(strstr(output,"age_source=rx") && strstr(output,"frame_age_ms=20") && camera_calls==0 && motor_calls==0);
  tick=4980;
  command("camera status\r");
  assert(strstr(output,"state=Stale") && strstr(output,"frame_age_ms=4900"));
  assert(strstr(output,"cx=na cy=na dx=na dy=na") && !strstr(output,"cx=65535"));
  tick=100;
  snapshot.HasValidData=snapshot.TargetValid=0;
  command("camera status\r");
  assert(strstr(output,"state=Lost") && strstr(output,"cx=na cy=na") && !strstr(output,"cx=0"));
  snapshot.HasFrame=0;
  command("camera status\r");assert(strstr(output,"state=Wait") && strstr(output,"frame_age_ms=0"));
  snapshot.HasFrame=1;
  command("help\r");assert(strstr(output,"auto dwell 0ms") && !strstr(output,"after measured revolution"));
  command("camera status\r"); assert(strstr(output,"cx=na cy=na") && !strstr(output,"cx=65535"));
  command("imu status\r");assert(strstr(output,"4294967295") && strstr(output,"-12.50"));
  assert(!ArmConsole_ImuStreamEnabled() && strstr(output,"result=NONE"));
  command("imu stream on\r");assert(ArmConsole_ImuStreamEnabled() && strstr(output,"OK imu stream=on"));
  command("imu stream off\r");assert(!ArmConsole_ImuStreamEnabled() && strstr(output,"OK imu stream=off"));
  command("imu stream invalid\r");assert(!ArmConsole_ImuStreamEnabled() && strstr(output,"ERR imu format"));
  command("imu stream on\r");assert(ArmConsole_ImuStreamEnabled());
  command("imu cal start\r\n");assert(cal_starts==1 && strstr(output,"cal_ms=20000") && strstr(output,"verify_ms=30000"));
  assert(!ArmConsole_ImuStreamEnabled() && strstr(output,"stream=off"));
  assert(occurrences("arm> ")==1);
  command("imu cal start\r\n");assert(cal_starts==1 && strstr(output,"busy"));
  command("imu stream on\r");assert(!ArmConsole_ImuStreamEnabled() && strstr(output,"busy"));
  before=refresh_calls;command("imu status\r");assert(refresh_calls==before);
  assert(strstr(output,"cal_state=") && strstr(output,"result=PENDING") && strstr(output,"persistent=UNTESTED"));
  assert(!strstr(output,"corrected") && !strstr(output,"bias_dps") && !strstr(output,"source=flash"));
  before=motor_calls;command("chassis velocity 30 0 0\r");assert(motor_calls==before && strstr(output,"imu_busy"));
  command("pos x 90\r");assert(motor_calls==before && strstr(output,"imu_busy"));
  command("grab calib 1\r");assert(!grab_busy && strstr(output,"other_task_busy"));
  command("vision material 2\r");assert(strstr(output,"ERR retired"));
  command("grip open\r");assert(strstr(output,"imu_busy"));
  command("vision status\r");assert(!strstr(output,"imu_busy"));
  command("chassis stop\r");assert(!strstr(output,"imu_busy"));
  command("imu cal cancel\r");assert(cal_cancels==1 && !cal_status.busy);
  command("imu verify\r");assert(verify_duration==5000 && cal_status.busy);
  command("imu cal clear\r");assert(cal_cancels==2 && !cal_status.busy);
  command("imu verify 120\r");assert(strstr(output,"ERR imu format") && !cal_status.busy);
  command("imu cal clear\r");assert(cal_cancels==3);
  command("imu verify 5\r");assert(verify_duration==5000 && cal_status.busy);
  command("imu cal clear\r");
  command("imu verify 30\r");assert(strstr(output,"ERR imu format") && !cal_status.busy);
  command("imu verify 121\r");assert(strstr(output,"ERR imu format") && !cal_status.busy);
  chassis_busy=true;command("imu cal start\r");assert(cal_starts==1 && strstr(output,"motion_busy"));
  command("imu verify\r");assert(!cal_status.busy && strstr(output,"motion_busy"));chassis_busy=false;
  motor_busy=1;command("imu cal start\r");assert(cal_starts==1 && strstr(output,"motion_busy"));motor_busy=0;
  grab_busy=1;command("imu cal start\r");assert(cal_starts==1 && strstr(output,"grab_busy"));grab_busy=0;
  cal_status.state=HWT101_CAL_DONE;
  before=refresh_calls;command("imu status\r");assert(refresh_calls==before+1 && strstr(output,"result=PASS"));
  cal_status.state=HWT101_CAL_FAILED;cal_status.gap_ms=353;cal_status.max_gap_ms=353;
  cal_status.recovery_step="CAL_WAIT";cal_status.recovery_expected_mode=1;cal_status.recovery_mode_mask=6;
  cal_status.recovery_mode_first=cal_status.recovery_mode_last=0x1234;
  cal_status.recovery_mode_seen=cal_status.recovery_mode_valid=true;cal_status.recovery_mode_reads=2;
  command("imu status\r");assert(strstr(output,"result=FAIL") && strstr(output,"gap_ms=353 max_gap_ms=353 limit_ms=300"));
  assert(strstr(output,"recovery_step=CAL_WAIT expected_mode=0x0001 first_mode=0x1234 last_mode=0x1234"));
  assert(strstr(output,"mode_seen=1 mode_valid=1 mode_HAL=0 mode_reads=2"));
  assert(strstr(output,"allowed_mode_mask=0x0006"));
  cal_status.timing_valid=true;cal_status.peak_time_s=cal_status.bias_time_s=10;
  cal_status.native_modes_seen=7;cal_status.native_normal_ms=15800;
  command("imu status\r");assert(strstr(output,"OK imu native timing_valid=1 peak_time_s=10 bias_time_s=10"));
  assert(strstr(output,"native_modes=0x07 native_normal_ms=15800"));
  command("imu cal clear\r");assert(cal_cancels==5);
  command("imu cal start extra\r");assert(cal_starts==1 && strstr(output,"ERR imu format"));
  before=motor_calls;command("chassis hold 30 5\r");assert(motor_calls==before && strstr(output,"imu_not_verified"));
  cal_status.state=HWT101_CAL_DONE;
  command("chassis hold 101 5\r");assert(motor_calls==before && strstr(output,"ERR range"));
  command("chassis hold 30 5\r");assert(heading_status.active && strstr(output,"source=module"));
  command("chassis velocity 30 0 0\r");assert(strstr(output,"chassis_task_busy"));
  command("grab calib 1\r");assert(strstr(output,"other_task_busy"));
  command("imu cal start\r");assert(strstr(output,"chassis_task_busy") && cal_starts==1);
  command("chassis status\r");assert(strstr(output,"active=1"));
  command("chassis stop\r");assert(!heading_status.active);
  command("chassis hold 0 1\r");assert(heading_status.active);
  command("imu cal clear\r");assert(!heading_status.active && cal_cancels==6);
  motor_calls=before;stop_calls=0;
  cal_status.state=HWT101_CAL_DONE;
  command("imu cal forget\r");assert(strstr(output,"deprecated") && cal_status.state==HWT101_CAL_DONE && cal_cancels==6);
  command("screen status\r");assert(strstr(output,"4294967295") && strstr(output,"12345"));
  {
    unsigned before_camera = camera_calls, before_motor = motor_calls;
    command("vision calib 1\r");assert(strstr(output,"ERR retired"));
    command("vision ring 1\r");assert(strstr(output,"ERR retired"));
    assert(camera_calls == before_camera && motor_calls == before_motor);
  }
  command("camera material 3\r");assert(camera_calls==1 && snapshot.RequestTarget==3);
  for(unsigned ring=1;ring<=3;ring++) {
    char cmd[32];snprintf(cmd,sizeof cmd,"camera ring %u\r",ring);command(cmd);
    assert(snapshot.RequestTarget==9+ring && strstr(output,"OK camera ring"));
  }
  before=camera_calls;command("camera ring 4\r");assert(camera_calls==before);
  command("camera material 3\r");
  camera_result=HAL_BUSY;command("camera material 2\r");assert(strstr(output,"busy") && snapshot.RequestTarget==3);
  camera_result=HAL_ERROR;snapshot.UsbConfigured=0;command("camera material 2\r");assert(strstr(output,"usb") && snapshot.RequestTarget==3);
  snapshot.UsbConfigured=1;command("camera material 2\r");assert(strstr(output,"tx"));camera_result=HAL_OK;
  grab_busy=1;before=camera_calls;command("camera material 2\r");assert(strstr(output,"busy") && camera_calls==before);
  command("vision material 2\r");assert(strstr(output,"ERR retired") && grab_busy);
  command("grab stop\r");assert(!grab_busy );
  command("grip duty 7.5\r");assert(grip_duty==7.5f);command("grip duty 99\r");assert(grip_duty==7.5f && strstr(output,"ERR"));
  command("chassis velocity 10 -20 90\r");assert(forward_value==10 && left_value==-20 && fabsf(yaw_value-1.5707963f)<0.0001f);
  before=motor_calls;command("chassis\r");assert(strstr(output,"chassis_task_busy") && motor_calls==before);
  command("chassis velocity 20 0 0\r");assert(forward_value==20 && motor_calls==before+1);
  command("chassis velocity 0 0 0\r");assert(!motion_busy && !continuous_velocity);
  command("config z 100 20 0\r");assert(config.SpeedRpm==100 && config.Acceleration==20 && config.PulseLimit==0);
  command("pos x 90\r\n");assert(motor_pulses==800 && !strstr(output,"arm> "));
  event.Action=MECHANICAL_ARM_ACTION_POSITION;event.Axis=MECHANICAL_ARM_AXIS_X;event.PositionPulses=800;event.Result=MECHANICAL_ARM_RESULT_OK;
  ArmConsole_MotorEventHandle(&event);assert(strstr(output,"accepted") && occurrences("arm> ")==1);
  motor_result=MECHANICAL_ARM_RESULT_BUSY;command("enable base\r");assert(strstr(output,"ERR busy") && strstr(output,"arm> "));motor_result=MECHANICAL_ARM_RESULT_NONE;
  command("pos x xyz\r");assert(strstr(output,"ERR"));
  grab_busy=true;before=grab_stops;command("stop all\r");assert(!grab_busy && grab_stops>before);
  command("grab calib 3\r");assert(grab_busy && strstr(output,"OK grab calib accepted"));
  unsigned cameras=camera_calls,motors=motor_calls;
  command("grab calib status\r");assert(strstr(output,"OK grab calib phase=") && camera_calls==cameras && motor_calls==motors);
  command("grab calib ref\r");assert(strstr(output,"busy") && calib_ref);
  command("grab stop\r");assert(calib_ref && !grab_busy);
  /* The diagnostic is readable under each existing owner/operation gate. */
  for(unsigned owner=0;owner<9;owner++) {
    motor_busy=owner==0;chassis_busy=owner==1;motion_busy=owner==2;
    route_reserved=owner==3;cal_status.busy=owner==4;turntable_busy=owner==5;
    radar_scan_busy=owner==6;BusRecoveryPending=owner==7;ManualGuardActive=owner==8;
    command("grab calib status\r");
    assert(strstr(output,"OK grab calib phase=") && camera_calls==cameras && motor_calls==motors);
    ManualGuardActive=0;BusRecoveryPending=0;
  }
  motor_busy=0;chassis_busy=motion_busy=route_reserved=cal_status.busy=turntable_busy=radar_scan_busy=false;
  radar_scan_busy=true;command("grab calib 3\r");assert(!grab_busy && strstr(output,"stationary capture"));radar_scan_busy=false;
  route_reserved=true;
  command("enable base\r");assert(strstr(output,"chassis_task_busy"));
  command("state base\r");assert(!strstr(output,"chassis_task_busy"));
  command("position base\r");assert(!strstr(output,"chassis_task_busy"));
  command("imu cal start\r");assert(strstr(output,"chassis_task_busy"));
  command("wheel stop\r");assert(!route_reserved);
  /* Raw Ctrl+C discards a partial line, even a full ring, and cancels all producers. */
  grab_busy=grab_route_busy=false;
  before=manual_arm_stops;command("chassis stop 39\r");
  assert(manual_arm_stops==before+1); /* Manual motion survives task ACK: stop still targets arm. */
  before=motor_calls;command("chassis velocity 10");
  route_reserved=motion_busy=true;grab_busy=1;
  output[0]=0;ArmConsole_ReceiveData(0x03);ArmConsole_StopProcess();
  assert(!route_reserved && !motion_busy && !grab_busy);
  assert(strstr(output,"token=0") && strstr(output,"tx_complete=0 wheels_stopped=0"));
  command("\r");assert(motor_calls==before && !strstr(output,"ERR format"));
  for(unsigned j=0;j<200;j++)ArmConsole_ReceiveData('x');
  ArmConsole_ReceiveData(0x03);
  command("chassis stop 4294967295\r");
  assert(strstr(output,"token=4294967295") && !strstr(output,"ERR format"));
  command("chassis stop-status\r");assert(strstr(output,"token=4294967295"));
  /* Interrupt between foreground stop servicing and ring pop must preserve the
   * first byte of a following command, rather than dropping its leading 'c'. */
  ArmConsole_ReceiveData(0x03);ArmConsole_ReceiveData('c');
  {uint8_t byte=0;assert(ArmConsole_RingPop(&byte)==2 && byte==0);}
  ArmConsole_StopProcess();
  command("hassis stop 41\r");assert(strstr(output,"token=41"));
  before=stop_calls;
  command("chassis stop 0\r");assert(strstr(output,"ERR stop token"));
  command("chassis stop -1\r");assert(strstr(output,"ERR stop token"));
  command("chassis stop 4294967296\r");assert(strstr(output,"ERR stop token") && stop_calls==before);
  route_reserved=motion_busy=true;grab_busy=1;cal_status.busy=true;
  command("chassis stop 42\r");
  assert(!route_reserved && !motion_busy && !grab_busy && strstr(output,"token=42"));
  cal_status.busy=false;
  output[0]=0;memset(&event,0,sizeof(event));
  event.Action=MECHANICAL_ARM_ACTION_STOP;event.Result=MECHANICAL_ARM_RESULT_OK;
  event.Axis=MECHANICAL_ARM_AXIS_ALL;event.StopIdMask=0x60U;
  ArmConsole_MotorEventHandle(&event);
  assert(strstr(output,"OK stop z sent") && strstr(output,"OK stop x sent") && !strstr(output,"OK stop all sent"));
  output[0]=0;event.StopIdMask=0xffU;ArmConsole_MotorEventHandle(&event);
  assert(strstr(output,"OK stop all sent"));
  bus_locked=true;motor_result=MECHANICAL_ARM_RESULT_TX_ERROR;
  command("enable base\r");assert(strstr(output,"ERR motor tx bus_locked=1"));
  assert(strstr(output,"[MOTOR BUS] locked=1 reason=uart_error fault_ms=12345"));
  assert(strstr(output,"addr=7 func=0x3A active=0 wait=0 uart_error=0x00000004"));
  output[0]=0;memset(&event,0,sizeof(event));
  event.Axis=event.FailureAxis=MECHANICAL_ARM_AXIS_BASE;
  event.Action=MECHANICAL_ARM_ACTION_DISABLE;event.Result=MECHANICAL_ARM_RESULT_TX_ERROR;
  ArmConsole_MotorEventHandle(&event);
  assert(strstr(output,"ERR motor communication axis=base bus_locked=1"));
  output[0]=0;event.Result=MECHANICAL_ARM_RESULT_ACK_TIMEOUT;
  ArmConsole_MotorEventHandle(&event);
  assert(strstr(output,"ERR ack_timeout axis=base state_unknown bus_locked=1"));
  bus_locked=false;command("enable base\r");assert(strstr(output,"ERR motor tx bus_locked=0"));
  /* A damaged line cannot turn into a valid actuator command after recovery. */
  before=motor_calls;command("enable ");ArmConsole_ReceiveFault();
  command("base\r");assert(motor_calls==before);
  command("info\r");assert(strstr(output,"GRAB protocol=B2 align_color=3 pick_color=1\r\n"));assert(strstr(output,"STM32F407"));
  assert(strstr(output,"BUILD motorbus_diag=1"));
  assert(strstr(output,"arm positive_dir base=1 z=0 x=1 (0=CW 1=CCW)"));
  assert(strstr(output,"[MOTOR BUS] locked=0 reason=none"));
  for(unsigned j=0;j<600;j++)ArmConsole_ReceiveData('x');
  ArmConsole_ReceiveData(3);command("chassis stop 99\r");assert(strstr(output,"token=99"));
  chassis_busy=false; heading_status.active=false; motor_busy=grab_busy=0;
  route_reserved=motion_busy=cal_status.busy=false;
  command("imu zero extra\r"); assert(!zero_calls && strstr(output,"ERR imu format"));
  motor_busy=1; command("imu zero\r"); assert(!zero_calls && strstr(output,"motion_busy")); motor_busy=0;
  command("imu zero\r"); assert(zero_calls==1 && strstr(output,"OK imu zero started"));
  cal_status.busy=false;
  for(unsigned owner=0;owner<6;owner++) {
    chassis_busy=owner==0; motor_busy=owner==1; grab_busy=owner==2;
    route_reserved=owner==3; motion_busy=owner==4; cal_status.busy=owner==5;
    command("system reset\r"); assert(!ArmConsole_ResetPending() && strstr(output,"ERR system reset busy"));
  }
  chassis_busy=false; motor_busy=grab_busy=0; route_reserved=motion_busy=cal_status.busy=false;
  command("system reset extra\r"); assert(!ArmConsole_ResetPending());
  urgent_idle=false; command("system reset\r"); assert(ArmConsole_ResetPending() && strstr(output,"OK system reset pending"));
  command("vision stop\r");assert(ArmConsole_ResetPending() && strstr(output,"ERR retired"));
  before=motor_calls; command("enable base\r"); assert(motor_calls==before && strstr(output,"reset_pending"));
  tick+=1000; assert(ArmConsole_ResetProcess() && !reset_calls);
  urgent_idle=true; assert(ArmConsole_ResetProcess() && !reset_calls);
  tick+=199; assert(ArmConsole_ResetProcess() && !reset_calls);
  tick++; ArmConsole_ResetProcess(); assert(reset_calls==1);
  command("system reset\r"); ArmConsole_ReceiveData(3); ArmConsole_StopProcess();
  assert(!ArmConsole_ResetPending()); tick+=6000; ArmConsole_ResetProcess(); assert(reset_calls==1);
  {
    const char *stops[]={"stop all\r","stop base\r","stop z\r","stop x\r","grab stop\r","camera stop\r"};
    for(unsigned i=0;i<sizeof(stops)/sizeof(stops[0]);i++) {
      command("system reset\r"); assert(ArmConsole_ResetPending());
      command(stops[i]); assert(!ArmConsole_ResetPending() && strstr(output,"cancelled_by_stop"));
      tick+=6000; ArmConsole_ResetProcess(); assert(reset_calls==1);
    }
  }
  urgent_idle=false; command("system reset\r"); tick+=5001; ArmConsole_ResetProcess();
  assert(!ArmConsole_ResetPending() && reset_calls==1 && strstr(output,"ack_timeout"));
  command("system reset\r"); urgent_losses++; ArmConsole_ResetProcess();
  assert(!ArmConsole_ResetPending() && reset_calls==1 && strstr(output,"ack_failed"));
  command("grab stop\r");assert(grab_stops>0);grab_stops=0;
  grab_busy=true; before=motor_calls;
  command("grab route status\r");assert(strstr(output,"OK grab route query"));
  command("grab mission status\r");assert(strstr(output,"OK grab route query"));
  command("chassis task\r");assert(strstr(output,"OK chassis task query"));
  command("chassis route status\r");assert(strstr(output,"OK chassis route query"));
  assert(grab_busy && motor_calls==before);
  { unsigned stops=grab_stops;command("vision stop\r");assert(grab_busy && grab_stops==stops && strstr(output,"ERR retired")); }

  command("grab track status\r");assert(strstr(output,"track routed"));
  command("grab track get body_speed\r");assert(strstr(output,"track routed"));
  command("grab track start 3\r");assert(strstr(output,"track routed"));
  { unsigned stops=grab_stops; command("grab track stop\r");assert(!grab_busy && grab_stops==stops+1); grab_busy=true; }
  grab_stops=0;
  command("pos z 10\r");assert(motor_calls==before&&strstr(output,"grab_busy"));
  command("grip open\r");assert(strstr(output,"grab_busy"));
  ring_manual=true;
  command("grab ring take 2\r");assert(strstr(output,"ring take routed") && motor_calls==before);
  command("grip catch\r");assert(!strstr(output,"grab_busy") && grip_duty==ARM_GRIPPER_CATCH);
  command("grab stack lower -100\r");assert(strstr(output,"stack routed") && motor_calls==before);
  command("grab stack release\r");assert(strstr(output,"stack routed") && motor_calls==before);
  command("grab stack lift\r");assert(strstr(output,"stack routed") && motor_calls==before);
  command("pos z 10\r");assert(motor_calls==before && strstr(output,"grab_busy"));
  ring_manual=false;
  /* A moving Z target may be superseded through the task-owned path. The real
   * task parser decides whether lower/release is valid at its current state. */
  command("grab stack lower -90\r");assert(strstr(output,"stack routed") && motor_calls==before);
  command("imu verify\r");assert(strstr(output,"grab_busy"));
  command("state x\r");assert(motor_calls==before&&strstr(output,"grab_busy"));
  command("grab status\r");assert(strstr(output,"OK grab state=hold"));
  command("chassis stop\r");assert(!grab_busy&&grab_stops==1);
  command("grab stack align 3\r");assert(strstr(output,"stack routed") && motor_calls==before);
  grab_busy=grab_route_busy=true;command("stop all\r");assert(!grab_busy&&!grab_route_busy&&grab_stops==2);
  grab_route_busy=true;command("grab start fixed\r");assert(strstr(output,"grab_busy"));
  command("grab stack align 3\r");assert(strstr(output,"route_busy") && !strstr(output,"stack routed"));
  command("grab stop\r");assert(!grab_route_busy);
  motor_result=MECHANICAL_ARM_RESULT_NONE;
  guard_moving=true;before=motor_calls;
  command("pos base -180\r");assert(motor_calls==before && strstr(output,"axis_busy_or_fault"));
  guard_moving=false;
  command("pos base -180\r");assert(motor_calls==before+1);
  /* Two commands in the same RX burst never queue two eventual movements. */
  before=motor_calls;command("pos base -90\rpos base -90\r");
  assert(motor_calls==before+1 && strstr(output,"command_not_queued"));
  before=motor_calls;command("pos base -90\rstop all\r");assert(motor_calls==before);
  command("bus recover\r");assert(BusRecoveryPending && recovery_fake.active);
  before=motor_calls;command("pos x 90\r");assert(motor_calls==before && strstr(output,"operation_busy"));
  command("bus status\r");assert(strstr(output,"active=1"));
  command("bus recovery\r");assert(strstr(output,"OK bus attempt stage=") && strstr(output,"OK bus attempt_rx") && strstr(output,"OK bus first"));
  recovery_fake.active=false;recovery_fake.success=true;recovery_fake.reason="complete";
  output[0]=0;ArmConsole_Process();assert(!BusRecoveryPending && strstr(output,"tasks_resumed=0"));
  command("bus recover\r");ArmConsole_ReceiveData(3);ArmConsole_StopProcess();
  assert(!BusRecoveryPending && !recovery_fake.active);
  mission_pending=mission_recovery_accepted=true;unsigned saved_stops=grab_stops;
  command("bus recover\r");assert(!BusRecoveryPending && !recovery_fake.active && grab_stops==saved_stops);
  assert(strstr(output,"mission_preserved=1"));mission_pending=mission_recovery_accepted=false;
  output[0]=0;stop_status.tx_complete=true;stop_status.reason="monitoring";
  ArmConsole_StopProcess();assert(strstr(output,"tx_complete=1 wheels_stopped=0"));
  output[0]=0;ArmConsole_StopProcess();assert(!output[0]); /* no repeated report */
  stop_status.wheels_stopped=true;stop_status.reason="stopped";
  ArmConsole_StopProcess();assert(strstr(output,"wheels_stopped=1 reason=stopped"));
  /* ID 8 queries never wait for task ownership; motion cannot enter another task. */
  grab_busy=true;
  before=turntable_commands;command("turntable status\r");
  assert(turntable_commands==before+1 && strstr(output,"OK turntable"));
  command("turntable index 1\r");assert(turntable_commands==before+1 && strstr(output,"grab_busy"));
  grab_busy=false;motion_busy=true;
  command("turntable origin\r");assert(turntable_commands==before+1 && strstr(output,"other_task_busy"));
  motion_busy=false;command("turntable index 1\r");assert(turntable_commands==before+2);
  turntable_busy=true;before=motor_calls;
  command("pos x 10\r");assert(motor_calls==before && strstr(output,"turntable_busy"));
  command("system reset\r");assert(!ArmConsole_ResetPending() && strstr(output,"busy"));
  before=turntable_stops;command("stop all\r");assert(!turntable_busy && turntable_stops>before);
  turntable_busy=true;before=turntable_stops;
  ArmConsole_ReceiveData(3);ArmConsole_StopProcess();assert(!turntable_busy && turntable_stops>before);
  grab_busy=grab_route_busy=true;motor_busy=true;motion_busy=true;
  command("grab qr status\r");assert(strstr(output,"OK grab route query"));
  command("grab qr stop\r");assert(!grab_busy && !grab_route_busy && strstr(output,"stop requested"));
  motor_busy=false;motion_busy=false;
  command("bus read 5 current\r");assert(BusTest.active && strstr(output,"read_only=1"));
  for(unsigned i=0;i<10 && BusTest.active;i++){tick+=25;ArmConsole_Process();}
  assert(!BusTest.active && BusTest.received==1 && strstr(output,"phase_ma=2500"));
  command("bus read 5 voltage\r");
  for(unsigned i=0;i<10 && BusTest.active;i++){tick+=25;ArmConsole_Process();}
  assert(strstr(output,"voltage_mv=24000"));
  command("bus read 5 error\r");
  for(unsigned i=0;i<10 && BusTest.active;i++){tick+=25;ArmConsole_Process();}
  assert(strstr(output,"error_raw=-65536 error_deg=-360.000"));
  command("bus read 5 home\r");
  for(unsigned i=0;i<10 && BusTest.active;i++){tick+=25;ArmConsole_Process();}
  assert(strstr(output,"home_rpm=30 timeout_ms=10000"));
  command("bus read 5 driver\r");
  for(unsigned i=0;i<10 && BusTest.active;i++){tick+=25;ArmConsole_Process();}
  assert(strstr(output,"current_limit_ma=3000") && strstr(output,"reach_deg=0.8"));
  command("bus read 5 current\r");
  probe_event.result=MOTOR_BUS_REPLY;probe_event.length=4;
  probe_event.data[2]=0xe2;probe_event.data[3]=0x6b;
  output[0]=0;tick+=25;ArmConsole_Process();
  assert(!BusTest.active && BusTest.rejected==1 && !strstr(output,"phase_ma="));
  command("bus read 5 driver\r");
  ArmConsole_ReceiveData(3);ArmConsole_StopProcess();assert(!BusTest.active && !probe_active);
  command("bus read 5 bogus\r");assert(!BusTest.active && strstr(output,"ERR format"));
  command("bus test 1 1\r");assert(strstr(output,"OK bus test start id=1 rounds=1"));
  before=motor_calls;command("wheel fl 10\r");assert(motor_calls==before && strstr(output,"bus test active"));
  command("state x\r");assert(motor_calls==before && strstr(output,"bus test active"));
  for(unsigned i=0;i<10 && BusTest.active;i++){tick+=25;ArmConsole_Process();}
  assert(!BusTest.active && BusTest.received==3 && BusTest.sent==3 && BusTest.reply_max==2);
  bus_locked=true;command("bus test 4 1\r");assert(BusTest.active && strstr(output,"locked=1"));
  command("bus test status\r");assert(strstr(output,"id=4 rounds=1 active=1"));
  command("bus test stop\r");assert(!BusTest.active && bus_locked && strstr(output,"reason=stopped"));
  command("bus test 4 1\r");
  assert(BusTest.pending);
  probe_event.result=MOTOR_BUS_TIMEOUT;probe_event.tx_completed=false;
  output[0]=0;ArmConsole_Process();assert(strstr(output,"reason=tx_timeout"));
  tick+=25;ArmConsole_Process();assert(BusTest.pending);
  probe_event.result=MOTOR_BUS_IO_ERROR;probe_event.uart_error=4;
  output[0]=0;ArmConsole_Process();assert(strstr(output,"reason=uart_error uart_error=0x00000004"));
  tick+=25;ArmConsole_Process();assert(BusTest.pending);
  probe_event.result=MOTOR_BUS_IO_ERROR;probe_event.uart_error=0;
  output[0]=0;ArmConsole_Process();assert(strstr(output,"reason=transport_error"));
  assert(!BusTest.active && BusTest.timeouts==1 && BusTest.uart_errors==2 && BusTest.last_uart_error==0 && bus_locked);
  command("bus test 0\r");assert(!BusTest.active && strstr(output,"ERR format"));
  command("bus test 9\r");assert(!BusTest.active && strstr(output,"ERR format"));
  command("bus test 4 101\r");assert(!BusTest.active && strstr(output,"ERR format"));
  motion_busy=true;command("bus test 4\r");assert(!BusTest.active && strstr(output,"tasks_busy"));motion_busy=false;
  command("bus test 8\r");assert(BusTest.active && BusTest.rounds==10);
  ArmConsole_ReceiveData(3);ArmConsole_StopProcess();assert(!BusTest.active && !probe_active);
  bus_locked=false;
  mode_feedback_enabled=true;mode_feedback_address=3;
  command("bus mode-test receive off 3\r");assert(mode_fake.active && ArmConsole_OperationBusy());
  assert(!mode_feedback_enabled && mode_feedback_address==0);
  before=motor_calls;command("wheel fl 10\r");assert(before==motor_calls && strstr(output,"mode-test active"));
  command("bus read 1 driver\r");assert(!BusTest.active && strstr(output,"mode-test active"));
  command("bus mode-test status\r");assert(strstr(output,"gap_ms=3"));
  mode_fake.active=false;mode_fake.success=mode_fake.restored=true;mode_fake.reason="complete";
  ArmConsole_Process();assert(mode_feedback_enabled && mode_feedback_address==3 && !ArmConsole_OperationBusy());
  mode_feedback_valid=false;command("bus mode-test none on 5\r");assert(!mode_fake.active && strstr(output,"fresh_stopped_feedback_required"));mode_feedback_valid=true;
  mode_feedback_rpm=2;command("bus mode-test none on 5\r");assert(!mode_fake.active && strstr(output,"fresh_stopped_feedback_required"));mode_feedback_rpm=0;
  command("bus mode-test none on 4\r");assert(!mode_fake.active && strstr(output,"ERR format"));
  command("bus mode-test none on 5\r");assert(mode_fake.active && mode_feedback_enabled);
  ArmConsole_ReceiveData(3);ArmConsole_StopProcess();assert(!mode_fake.active && mode_fake.cancelled);
  ArmConsole_Process();
  grab_busy=grab_route_busy=true;
  mission_pending=true;recovery_fake.active=true;
  command("grab mission pause\r");assert(mission_pauses==1 && recovery_fake.active);
  command("grab mission stop\r");assert(mission_cancels==1 && !recovery_fake.active);mission_pending=false;
  /* A runtime transport fault starts recovery without a console command. */
  bus_locked=true;BusAutoFaultSeen=false;recovery_fake.active=false;recovery_fake.success=false;
  tick+=3000;output[0]=0;ArmConsole_Process();
  assert(BusRecoveryPending && recovery_fake.active);
  recovery_fake.active=false;recovery_fake.reason="reply_timeout";
  ArmConsole_Process();assert(BusRecoveryPending==3);
  tick+=1999;ArmConsole_Process();assert(BusRecoveryPending==3 && !recovery_fake.active);
  tick++;ArmConsole_Process();assert(BusRecoveryPending==2 && recovery_fake.active);
  /* Stop cancels retry ownership and the same old fault must not restart it. */
  ArmConsole_ReceiveData(3);ArmConsole_StopProcess();
  tick+=3000;ArmConsole_Process();assert(!BusRecoveryPending && !recovery_fake.active);
  bus_locked=false;
  rx_error_fake_valid=true;rx_error_fake.sequence=42;rx_error_fake.phase=1;
  rx_error_fake.tick_ms=UINT32_MAX;rx_error_fake.token=UINT32_MAX;rx_error_fake.request_seq=UINT32_MAX;
  rx_error_fake.rx_total=UINT32_MAX;rx_error_fake.tx_ms=UINT32_MAX;rx_error_fake.uart_error=HAL_UART_ERROR_NE;
  rx_error_fake.tx_length=MOTOR_BUS_FRAME_MAX;memset(rx_error_fake.tx,0xff,sizeof rx_error_fake.tx);
  memset(rx_error_fake.dma,0xab,sizeof rx_error_fake.dma);
  command("bus recovery\r");
  assert(strstr(output,"OK bus rx_error seq=42 view=first phase=irq") &&
    strstr(output,"OK bus rx_error_tx seq=42 bytes=40 hex=FFFFFFFF") &&
    strstr(output,"OK bus rx_error_data seq=42 offset=32 bytes=32 hex=ABABABAB"));
  output[0]=0;tick+=2000;ArmConsole_BusRecoveryProcess();
  assert(!strstr(output,"OK bus rx_error seq=42"));
  rx_error_fake.sequence++;output[0]=0;tick+=2000;ArmConsole_BusRecoveryProcess();
  assert(strstr(output,"OK bus rx_error seq=43 view=latest"));
  rx_error_fake_valid=false;
  /* Quick native calibration has its own command, the same motion guards and
   * a ten-second verification report; it never silently uses full mode. */
  memset(&cal_status,0,sizeof cal_status);
  chassis_busy=false;motor_busy=grab_busy=motion_busy=0;heading_status.active=false;
  command("imu cal quick\r");
  assert(quick_cal_starts==1 && strstr(output,"cal_ms=20000") && strstr(output,"verify_ms=10000"));
  command("imu cal quick\r");assert(quick_cal_starts==1 && strstr(output,"busy"));
  command("imu cal cancel\r");
  chassis_busy=true;command("imu cal quick\r");assert(quick_cal_starts==1 && strstr(output,"motion_busy"));chassis_busy=false;
  grab_busy=1;command("imu cal quick\r");assert(quick_cal_starts==1 && strstr(output,"grab_busy"));grab_busy=0;
  motor_busy=1;command("imu cal quick\r");assert(quick_cal_starts==1 && strstr(output,"motion_busy"));motor_busy=0;
  command("imu cal quick extra\r");assert(quick_cal_starts==1 && strstr(output,"ERR imu format"));
  puts("arm_console_host_test: commands, units, busy, status, CRLF and async prompts OK");return 0;
}
