/* Exercise actual byte input, dispatch and async prompts, with device boundary fakes. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../template/App/arm_console.c"
#include "../template/Hardware/QR.c"
static char output[16000];
static uint32_t tick = 100;
static unsigned reset_calls, zero_calls;
static bool urgent_idle=true;
static uint32_t urgent_losses;
void NVIC_SystemReset(void) { reset_calls++; }
bool ConsoleTx_UrgentIdle(void) { return urgent_idle; }
static unsigned camera_calls, motor_calls, stop_calls, notice_clears;
static uint8_t arm_busy, material_busy, motor_busy;
static bool grab_busy, grab_route_busy;
bool RadarConsole_Command(unsigned n,char *t[]){(void)n;(void)t;return false;}
bool RadarConsole_ScanBusy(void){return false;}
static unsigned grab_stops;
bool GrabTask_IsBusy(void){return grab_busy;}
bool GrabRoute_IsBusy(void){return grab_route_busy;}
void GrabTask_Stop(void){grab_stops++;grab_busy=false;}
void GrabRoute_Stop(void){grab_route_busy=false;if(grab_busy)GrabTask_Stop();}
bool GrabRoute_Command(unsigned n,char *t[]){(void)n;(void)t;return false;}
void GrabTask_StatusGet(GrabTask_Status_t *s){memset(s,0,sizeof(*s));}
bool GrabTask_Command(unsigned n,char *t[]){if(n==2&&!strcmp(t[0],"grab")&&!strcmp(t[1],"status")){const char*s="OK grab state=hold\r\n";return ConsoleTx_Write((const uint8_t*)s,(uint16_t)strlen(s));}return false;}
static bool motion_busy;
static bool bus_locked;
static MotorBus_Recovery_t recovery_fake={false,false,0,0,"idle"};
static MotorBus_Event_t guard_event;
static bool guard_ready, guard_moving;
bool MotorBus_RecoveryStart(void) { recovery_fake.active=true;return true; }
void MotorBus_RecoveryCancel(void) { recovery_fake.active=false; }
void MotorBus_RecoveryGet(MotorBus_Recovery_t *s) { *s=recovery_fake; }
bool Mecanum_RecoveryAfterBusCheck(void) { return true; }
void Mecanum_Feedback_Enable(bool enabled) { (void)enabled; }
void MotorBus_Cancel(MotorBus_Owner_t owner) { (void)owner;guard_ready=false; }
bool MotorBus_OwnerBusy(MotorBus_Owner_t owner) { (void)owner;return guard_ready; }
bool MotorBus_Submit(MotorBus_Owner_t owner,const uint8_t *p,uint8_t n,uint8_t reply,uint32_t token,bool priority)
{
  (void)n;(void)reply;(void)token;(void)priority;assert(owner==MOTOR_BUS_GUARD);
  memset(&guard_event,0,sizeof(guard_event));guard_event.result=MOTOR_BUS_REPLY;
  guard_event.data[0]=p[0];guard_event.data[1]=p[1];
  if(p[1]==0x35)guard_event.data[4]=guard_moving?10:0;
  else guard_event.data[2]=3;
  guard_ready=true;return true;
}
bool MotorBus_EventGet(MotorBus_Owner_t owner,MotorBus_Event_t *e)
{ (void)owner;if(!guard_ready)return false;*e=guard_event;guard_ready=false;return true; }
bool MotorBus_IsQuarantined(void) { return bus_locked; }
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *out)
{
  memset(out,0,sizeof(*out));out->locked=bus_locked;
  out->reason=bus_locked?"uart_error":"none";
  out->tick_ms=12345;out->uart_error=4;
  out->owner=MOTOR_BUS_ARM;out->address=7;out->function=0x3a;
}
static MaterialVision_StateTypeDef material_state;
static MaterialVision_ErrorTypeDef material_error;
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
void ConsoleTx_GetStats(ConsoleTx_Stats_t *out){memset(out,0,sizeof(*out));out->urgent_dropped=urgent_losses;}
void ConsoleRx_GetStats(ConsoleRx_Stats_t *out){memset(out,0,sizeof(*out));}
uint32_t ConsoleTx_Dropped(void){return 0;}
bool ConsoleTx_Event(const char *data,uint16_t length)
{ assert(length<sizeof(qr_event));if(qr_event[0])qr_event_dropped++;memcpy(qr_event,data,length);qr_event[length]=0;qr_event_count++;return true; }
void ConsoleTx_EventCancel(void) {if(qr_event[0])qr_event_dropped++;qr_event[0]=0;}
uint32_t ConsoleTx_EventDropped(void) {return qr_event_dropped;}
bool ChassisTelemetry_Command(unsigned count,char *tokens[]) {(void)count;(void)tokens;return false;}
static bool route_reserved;
bool ChassisRoute_IsBusy(void){return route_reserved;}
bool ChassisRoute_Cancel(void){route_reserved=false;motion_busy=false;return true;}
bool ChassisRoute_Command(unsigned n,char *t[]){(void)n;(void)t;return false;}
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
void ChassisMotion_StatusGet(ChassisMotion_Status_t *s){memset(s,0,sizeof(*s));}
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *out){*out=stop_status;}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length, uint32_t timeout)
{ (void)uart; assert((uint32_t)length * 10U * 1000U <= 115200U * timeout);
  assert(strlen(output)+length < sizeof(output)); strncat(output,(const char *)data,length); return HAL_OK; }
void OledUi_NoticeSet(const char *text) { if (!text) notice_clears++; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out) { *out = snapshot; }
Camera_VisualStateTypeDef Camera_VisualStateGet(const Camera_SnapshotTypeDef *data)
{ return data->HasValidData ? CAMERA_VIS_OK : CAMERA_VIS_LOST; }
const char *Camera_VisualStateNameGet(Camera_VisualStateTypeDef state) { return state == CAMERA_VIS_OK ? "Ok" : "Lost"; }
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef color)
{ camera_calls++; if (camera_result == HAL_OK) snapshot.RequestTarget = color; return camera_result; }
void Camera_RequestStop(void) { snapshot.RequestActive = 0; }
uint8_t ArmVision_IsBusy(void) { return arm_busy; }
uint8_t ArmVision_IsCalibrated(void) { return 1; }
uint8_t ArmVision_IsReferenceValid(void) { return 1; }
uint32_t ArmVision_StatusSequenceGet(void) { return 1; }
ArmVision_ResultTypeDef ArmVision_ReferenceSet(void) { return arm_busy || material_busy ? ARM_VISION_RESULT_BUSY : ARM_VISION_RESULT_OK; }
ArmVision_ResultTypeDef ArmVision_MaterialStart(Camera_ColorTypeDef color, ArmVision_JobTypeDef job)
{ (void)color; (void)job; if (arm_busy || material_busy) return ARM_VISION_RESULT_BUSY; arm_busy = 1; return ARM_VISION_RESULT_OK; }
ArmVision_ResultTypeDef ArmVision_MaterialCalibrationStart(Camera_ColorTypeDef color) { return ArmVision_MaterialStart(color, ARM_VISION_JOB_ALIGN_ONLY); }
void ArmVision_Stop(void) { if (arm_busy) stop_calls++; arm_busy = 0; }
const char *ArmVision_StateNameGet(void) { return arm_busy ? "COLLECT" : "IDLE"; }
ArmVision_ErrorTypeDef ArmVision_ErrorGet(void) { return ARM_VISION_ERROR_NONE; }
const char *ArmVision_ErrorNameGet(void) { return "none"; }
const char *ArmVision_CalibrationSourceNameGet(void) { return "material"; }
uint8_t ArmVision_CalibrationTargetGet(void) { return 3; }
uint8_t ArmVision_CalibrationResultGet(ArmVision_CalibrationDataTypeDef *out) { memset(out,0,sizeof(*out)); return 0; }
uint8_t ArmVision_CalibrationDebugGet(ArmVision_CalibrationDebugDataTypeDef *out) { memset(out,0,sizeof(*out)); return 0; }
uint8_t ArmVision_ErrorInfoGet(ArmVision_ErrorInfoTypeDef *out) { memset(out,0,sizeof(*out)); return 0; }
uint8_t ArmVision_MoveDebugGet(ArmVision_MoveDebugDataTypeDef *out) { memset(out,0,sizeof(*out)); return 0; }
uint8_t ArmVision_ProgressGet(ArmVision_ProgressTypeDef *out) { memset(out,0,sizeof(*out)); return 1; }
uint8_t MaterialVision_IsBusy(void) { return material_busy; }
uint8_t MaterialVision_IsCalibrated(void) { return 1; }
MaterialVision_ResultTypeDef MaterialVision_Start(Camera_ColorTypeDef color)
{ (void)color; if (arm_busy || material_busy) return MATERIAL_VISION_RESULT_BUSY; material_busy=1;material_state=MATERIAL_VISION_STATE_SEARCH;return MATERIAL_VISION_RESULT_OK; }
MaterialVision_ResultTypeDef MaterialVision_CalibrationStart(Camera_ColorTypeDef color) { return MaterialVision_Start(color); }
void MaterialVision_Stop(void) { if (material_busy) stop_calls++; material_busy=0;material_state=MATERIAL_VISION_STATE_IDLE;material_error=MATERIAL_VISION_ERROR_NONE; }
const char *MaterialVision_StateNameGet(void) { return material_busy ? "SEARCH" : "IDLE"; }
const char *MaterialVision_ErrorNameGet(void) { return material_error ? "motor_ack_timeout" : "none"; }
MaterialVision_StateTypeDef MaterialVision_StateGet(void) { return material_state; }
MaterialVision_ErrorTypeDef MaterialVision_ErrorGet(void) { return material_error; }
uint8_t MaterialVision_CalibrationGet(MaterialVision_CalibrationDataTypeDef *out) { memset(out,0,sizeof(*out)); return 1; }
bool HWT101_Is_Ready(void) { return true; }
bool HWT101_Angle_Get(HWT101_Angle_t *out) { memset(out,0,sizeof(*out)); out->yaw=-12.5f;out->update_count=44;return true; }
bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *out,uint32_t age) { (void)out;(void)age;return true; }
bool HWT101_Status_Get(HWT101_Status_t *out) { memset(out,0,sizeof(*out));out->valid_read_count=44;out->i2c_error_count=UINT32_MAX;return true; }
static HWT101_CalStatus_t cal_status;
static unsigned cal_starts, cal_cancels, refresh_calls;
static uint32_t verify_duration;
static bool chassis_busy;
bool HWT101_Cal_IsBusy(void) { return cal_status.busy; }
bool HWT101_Cal_ZeroStart(void) { zero_calls++; cal_status.busy=true; return true; }
bool HWT101_Cal_Start(void)
{
  assert(!chassis_busy && !arm_busy && !motor_busy && !material_busy);
  cal_starts++; cal_status.busy=true; cal_status.state=HWT101_CAL_CALIBRATING; return true;
}
bool HWT101_Cal_VerifyStart(uint32_t duration)
{
  assert(!chassis_busy && !arm_busy && !motor_busy && !material_busy);
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
bool Mecanum_Velocity_Start(float forward,float left,float yaw) { forward_value=forward;left_value=left;yaw_value=yaw;motor_calls++;return true; }
static Mecanum_HeadingTestStatus_t heading_status={false,"idle",0,0,0};
bool Mecanum_HeadingTest_Start(float speed,uint32_t duration)
{
  (void)duration;
  if(cal_status.state!=HWT101_CAL_DONE){heading_status.reason="imu_not_verified";return false;}
  forward_value=speed;motor_calls++;heading_status.active=true;heading_status.reason="running";return true;
}
bool Mecanum_IsBusy(void) { return chassis_busy || heading_status.active; }
void Mecanum_HeadingTest_StatusGet(Mecanum_HeadingTestStatus_t *s){*s=heading_status;}
bool Mecanum_Test_Stop(void) { stop_calls++;heading_status.active=false;return true; }
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
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis) { (void)axis;stop_calls++;return motor_result; }
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
  for(mode=0;mode<4;mode++)
  {
    cal_status.busy=mode==0;motion_busy=mode==1;heading_status.active=mode==2;material_busy=mode==3;
    command("qr status\r");assert(strstr(output,"OK qr status"));
    command("qr read\r");assert(strstr(output,"OK qr read"));
    command("qr stream on\r");assert(strstr(output,"OK qr stream=on"));
    command("qr stream off\r");assert(strstr(output,"OK qr stream=off"));
    assert(!camera_calls && !motor_calls && !stop_calls);
  }
  cal_status.busy=false;motion_busy=false;heading_status.active=false;material_busy=0;
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
  UART_HandleTypeDef uart={0};MechanicalArm_EventTypeDef event={0};unsigned before;
  uart.Instance=USART1;ArmConsole_Init(&uart);snapshot.UsbConfigured=1;snapshot.RequestActive=1;
  QR_Init();test_qr();
  motion_busy=true;
  command("material auto 1\r");assert(strstr(output,"chassis_task_busy") && !material_busy);
  command("imu cal start\r");assert(strstr(output,"chassis_task_busy"));
  command("wheel stop\r");assert(!motion_busy && strstr(output,"OK chassis stop id="));
  snapshot.RxByteCount=UINT32_MAX;snapshot.Data.CX=65535;snapshot.Data.DX=-32768;snapshot.Data.Sequence=7;
  command("help\r\n");assert(occurrences("arm> ")==1);assert(strstr(output,"camera status"));assert(!strstr(output,"vision ring <"));
  assert(!strstr(output,"vision calib <"));
  snapshot.Data.Target=2; snapshot.Data.Function=0xB2; snapshot.HasFrame=1; snapshot.LastFrameTick=80;
  snapshot.HasValidData=snapshot.TargetValid=1;
  command("camera status\r");assert(strstr(output,"4294967295"));assert(camera_calls==0 && motor_calls==0);
  assert(strstr(output,"cx=65535") && strstr(output,"dx=-32768") && strstr(output,"seq=7"));
  assert(strstr(output,"last fn=178 target=2") && strstr(output,"frame_age_ms=20"));
  assert(strstr(output,"protocol=B2 age_source=rx"));
  snapshot.HasValidData=snapshot.TargetValid=0;
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
  command("material auto 1\r");assert(!material_busy && strstr(output,"imu_busy"));
  command("vision material 2\r");assert(!arm_busy && strstr(output,"imu_busy"));
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
  arm_busy=1;command("imu verify\r");assert(!cal_status.busy && strstr(output,"motion_busy"));arm_busy=0;
  material_busy=1;command("imu cal start\r");assert(cal_starts==1 && strstr(output,"motion_busy"));material_busy=0;
  cal_status.state=HWT101_CAL_DONE;
  before=refresh_calls;command("imu status\r");assert(refresh_calls==before+1 && strstr(output,"result=PASS"));
  cal_status.state=HWT101_CAL_FAILED;cal_status.gap_ms=353;cal_status.max_gap_ms=353;
  command("imu status\r");assert(strstr(output,"result=FAIL") && strstr(output,"gap_ms=353 max_gap_ms=353 limit_ms=300"));
  command("imu cal clear\r");assert(cal_cancels==5);
  command("imu cal start extra\r");assert(cal_starts==1 && strstr(output,"ERR imu format"));
  before=motor_calls;command("chassis hold 30 5\r");assert(motor_calls==before && strstr(output,"imu_not_verified"));
  cal_status.state=HWT101_CAL_DONE;
  command("chassis hold 101 5\r");assert(motor_calls==before && strstr(output,"ERR range"));
  command("chassis hold 30 5\r");assert(heading_status.active && strstr(output,"source=module"));
  command("chassis velocity 30 0 0\r");assert(strstr(output,"chassis_hold_busy"));
  command("material auto 1\r");assert(strstr(output,"chassis_hold_busy"));
  command("imu cal start\r");assert(strstr(output,"motion_busy") && cal_starts==1);
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
    command("vision calib 1\r");assert(strstr(output,"ERR Unsupported B3"));
    command("vision ring 1\r");assert(strstr(output,"ERR Unsupported B3"));
    command("camera ring 1\r");assert(strstr(output,"ERR Unsupported B3"));
    assert(camera_calls == before_camera && motor_calls == before_motor);
  }
  command("camera material 3\r");assert(camera_calls==1 && snapshot.RequestTarget==3);
  camera_result=HAL_BUSY;command("camera material 2\r");assert(strstr(output,"busy") && snapshot.RequestTarget==3);
  camera_result=HAL_ERROR;snapshot.UsbConfigured=0;command("camera material 2\r");assert(strstr(output,"usb") && snapshot.RequestTarget==3);
  snapshot.UsbConfigured=1;command("camera material 2\r");assert(strstr(output,"tx"));camera_result=HAL_OK;
  material_busy=1;before=camera_calls;command("camera material 2\r");assert(strstr(output,"busy") && camera_calls==before);
  command("vision material 2\r");assert(strstr(output,"busy") && !arm_busy);
  command("vision stop\r");assert(!material_busy && notice_clears);
  command("grip duty 7.5\r");assert(grip_duty==7.5f);command("grip duty 99\r");assert(grip_duty==7.5f && strstr(output,"ERR"));
  command("chassis velocity 10 -20 90\r");assert(forward_value==10 && left_value==-20 && fabsf(yaw_value-1.5707963f)<0.0001f);
  command("config z 100 20 0\r");assert(config.SpeedRpm==100 && config.Acceleration==20 && config.PulseLimit==0);
  command("pos x 90\r\n");assert(motor_pulses==800 && !strstr(output,"arm> "));
  event.Action=MECHANICAL_ARM_ACTION_POSITION;event.Axis=MECHANICAL_ARM_AXIS_X;event.PositionPulses=800;event.Result=MECHANICAL_ARM_RESULT_OK;
  ArmConsole_MotorEventHandle(&event);assert(strstr(output,"accepted") && occurrences("arm> ")==1);
  motor_result=MECHANICAL_ARM_RESULT_BUSY;command("enable base\r");assert(strstr(output,"ERR busy") && strstr(output,"arm> "));motor_result=MECHANICAL_ARM_RESULT_NONE;
  command("pos x xyz\r");assert(strstr(output,"ERR"));
  arm_busy=1;before=stop_calls;command("stop all\r");assert(!arm_busy && stop_calls>before);
  command("material auto 3\r");assert(material_busy);
  output[0]=0;material_busy=0;material_state=MATERIAL_VISION_STATE_ERROR;material_error=MATERIAL_VISION_ERROR_MOTOR_ACK_TIMEOUT;
  ArmConsole_Process();assert(strstr(output,"ERR material reason=motor_ack_timeout") && occurrences("arm> ")==1);
  output[0]=0;ArmConsole_Process();assert(output[0]==0);
  route_reserved=true;
  command("enable base\r");assert(strstr(output,"chassis_task_busy"));
  command("state base\r");assert(!strstr(output,"chassis_task_busy"));
  command("position base\r");assert(!strstr(output,"chassis_task_busy"));
  command("imu cal start\r");assert(strstr(output,"chassis_task_busy"));
  command("wheel stop\r");assert(!route_reserved);
  /* Raw Ctrl+C discards a partial line, even a full ring, and cancels all producers. */
  before=motor_calls;command("chassis velocity 10");
  route_reserved=motion_busy=true;arm_busy=material_busy=1;
  output[0]=0;ArmConsole_ReceiveData(0x03);ArmConsole_StopProcess();
  assert(!route_reserved && !motion_busy && !arm_busy && !material_busy);
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
  route_reserved=motion_busy=true;material_busy=1;cal_status.busy=true;
  command("chassis stop 42\r");
  assert(!route_reserved && !motion_busy && !material_busy && strstr(output,"token=42"));
  cal_status.busy=false;
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
  chassis_busy=false; heading_status.active=false; motor_busy=arm_busy=material_busy=0;
  route_reserved=motion_busy=cal_status.busy=false;
  command("imu zero extra\r"); assert(!zero_calls && strstr(output,"ERR imu format"));
  motor_busy=1; command("imu zero\r"); assert(!zero_calls && strstr(output,"motion_busy")); motor_busy=0;
  command("imu zero\r"); assert(zero_calls==1 && strstr(output,"OK imu zero started"));
  cal_status.busy=false;
  for(unsigned owner=0;owner<7;owner++) {
    chassis_busy=owner==0; motor_busy=owner==1; arm_busy=owner==2; material_busy=owner==3;
    route_reserved=owner==4; motion_busy=owner==5; cal_status.busy=owner==6;
    command("system reset\r"); assert(!ArmConsole_ResetPending() && strstr(output,"ERR system reset busy"));
  }
  chassis_busy=false; motor_busy=arm_busy=material_busy=0; route_reserved=motion_busy=cal_status.busy=false;
  command("system reset extra\r"); assert(!ArmConsole_ResetPending());
  urgent_idle=false; command("system reset\r"); assert(ArmConsole_ResetPending() && strstr(output,"OK system reset pending"));
  before=motor_calls; command("enable base\r"); assert(motor_calls==before && strstr(output,"reset_pending"));
  tick+=1000; assert(ArmConsole_ResetProcess() && !reset_calls);
  urgent_idle=true; assert(ArmConsole_ResetProcess() && !reset_calls);
  tick+=199; assert(ArmConsole_ResetProcess() && !reset_calls);
  tick++; ArmConsole_ResetProcess(); assert(reset_calls==1);
  command("system reset\r"); ArmConsole_ReceiveData(3); ArmConsole_StopProcess();
  assert(!ArmConsole_ResetPending()); tick+=6000; ArmConsole_ResetProcess(); assert(reset_calls==1);
  {
    const char *stops[]={"stop all\r","stop base\r","stop z\r","stop x\r","vision stop\r","material stop\r","camera stop\r"};
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
  grab_busy=true; before=motor_calls;
  command("pos z 10\r");assert(motor_calls==before&&strstr(output,"grab_busy"));
  command("grip open\r");assert(strstr(output,"grab_busy"));
  command("imu verify\r");assert(strstr(output,"grab_busy"));
  command("state x\r");assert(motor_calls==before&&strstr(output,"grab_busy"));
  command("grab status\r");assert(strstr(output,"OK grab state=hold"));
  command("chassis stop\r");assert(!grab_busy&&grab_stops==1);
  grab_busy=grab_route_busy=true;command("stop all\r");assert(!grab_busy&&!grab_route_busy&&grab_stops==2);
  grab_route_busy=true;command("grab start fixed\r");assert(strstr(output,"grab_busy"));
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
  recovery_fake.active=false;recovery_fake.success=true;recovery_fake.reason="complete";
  output[0]=0;ArmConsole_Process();assert(!BusRecoveryPending && strstr(output,"tasks_resumed=0"));
  command("bus recover\r");ArmConsole_ReceiveData(3);ArmConsole_StopProcess();
  assert(!BusRecoveryPending && !recovery_fake.active);
  output[0]=0;stop_status.tx_complete=true;stop_status.reason="monitoring";
  ArmConsole_StopProcess();assert(strstr(output,"tx_complete=1 wheels_stopped=0"));
  output[0]=0;ArmConsole_StopProcess();assert(!output[0]); /* no repeated report */
  stop_status.wheels_stopped=true;stop_status.reason="stopped";
  ArmConsole_StopProcess();assert(strstr(output,"wheels_stopped=1 reason=stopped"));
  puts("arm_console_host_test: commands, units, busy, status, CRLF and async prompts OK");return 0;
}
