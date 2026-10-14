/* Production sampling service, with asynchronous device fixtures. */
#include <assert.h>
#include <stdio.h>
#include "../template/App/grab_calibration.c"
static uint32_t tick;
static Camera_DataTypeDef frame;
static bool commit_ok=true, chassis_ok=true, usb_configured=true;
static uint8_t motor_busy, axis_acc=37, last_acc;
static unsigned requests, moves, stops, arm_stops, task_stops, commits, velocity_requests;
static int32_t last_pulses;
static int16_t x_response_dx=2, x_response_dy=8;
static float last_forward, committed[4];
static Mecanum_Feedback_t wheels[4];
static HAL_StatusTypeDef camera_result=HAL_OK;
static MechanicalArm_ResultTypeDef motor_result=MECHANICAL_ARM_RESULT_NONE;
uint32_t HAL_GetTick(void) { return tick; }
bool GrabTask_VisualResponseSet(float a,float b,float c,float d)
{ commits++; if(!commit_ok)return false; committed[0]=a; committed[1]=b; committed[2]=c; committed[3]=d; return true; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out)
{ memset(out,0,sizeof(*out)); out->Data=frame; out->RequestFunction=0xB2; out->RequestTarget=CAMERA_COLOR_BLUE; out->RequestActive=1; out->UsbConfigured=usb_configured; out->TargetValid=out->HasValidData=frame.Sequence!=0; }
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef color) { frame.Function=0xB2; frame.Target=(uint8_t)color; requests++; return camera_result; }
void Camera_RequestStop(void) {}
uint8_t MechanicalArm_IsBusy(void) { return motor_busy; }
uint8_t MechanicalArm_ConfigGet(MechanicalArm_AxisTypeDef axis,MechanicalArm_ConfigTypeDef *out)
{ assert(axis==MECHANICAL_ARM_AXIS_X); out->Acceleration=axis_acc; return 1; }
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef axis)
{ assert(axis==MECHANICAL_ARM_AXIS_ALL); arm_stops++; return MECHANICAL_ARM_RESULT_NONE; }
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef axis)
{ assert(axis==MECHANICAL_ARM_AXIS_X); return motor_result; }
MechanicalArm_ResultTypeDef MechanicalArm_PositionRead(MechanicalArm_AxisTypeDef axis)
{ assert(axis==MECHANICAL_ARM_AXIS_X); return motor_result; }
MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef axis,int32_t pulses,uint16_t rpm,uint8_t acceleration,MechanicalArm_PositionModeTypeDef mode)
{ assert(axis==MECHANICAL_ARM_AXIS_X && rpm>0 && mode==MECHANICAL_ARM_POSITION_RELATIVE_CURRENT); moves++; last_pulses=pulses; last_acc=acceleration; return motor_result; }
bool Mecanum_FeedbackGet(unsigned wheel,Mecanum_Feedback_t *out) { *out=wheels[wheel]; return true; }
bool Mecanum_Velocity_Request(float forward,float left,float yaw,uint8_t acc)
{ assert(left==0 && yaw==0 && acc==20); velocity_requests++; last_forward=forward; return chassis_ok; }
bool Mecanum_Test_Stop(void) { stops++; return true; }
static void reset(void)
{
  tick=1; memset(&frame,0,sizeof(frame)); memset(wheels,0,sizeof(wheels));
  motor_busy=0;
  x_response_dx=2;x_response_dy=8;
  commit_ok=chassis_ok=usb_configured=true; camera_result=HAL_OK; motor_result=MECHANICAL_ARM_RESULT_NONE;
  requests=moves=stops=arm_stops=task_stops=commits=velocity_requests=0; GrabCalibration_Init();
  assert(!stops && !arm_stops);
}
static void feedback(int64_t raw)
{ unsigned i; for(i=0;i<4;i++) { wheels[i].position_valid=true; wheels[i].position_ms=tick; wheels[i].position_raw=i<2?raw:-raw; } }
static void sample(int16_t dx,int16_t dy)
{ unsigned i; for(i=0;i<5;i++) { frame.DX=dx; frame.DY=dy; frame.Tick=++tick; frame.Sequence++; GrabCalibration_Process(); } }
static void event(MechanicalArm_ActionTypeDef action,int32_t position,uint8_t flags)
{ MechanicalArm_EventTypeDef e={0}; e.Axis=MECHANICAL_ARM_AXIS_X; e.Action=action; e.Result=MECHANICAL_ARM_RESULT_OK; e.CurrentPosition=position; e.CurrentPositionRaw=position; e.StateFlags[MECHANICAL_ARM_AXIS_X]=flags; assert(GrabCalibration_MotorEventHandle(&e)); }
static void wheel_finish(int64_t raw)
{ tick+=1000; feedback(raw); GrabCalibration_Process(); assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_SETTLE); tick+=150; feedback(raw); GrabCalibration_Process(); }
static void x_finish(int32_t position)
{ event(MECHANICAL_ARM_ACTION_POSITION,0,0); tick+=50; GrabCalibration_Process(); event(MECHANICAL_ARM_ACTION_STATE,0,2); GrabCalibration_Process(); event(MECHANICAL_ARM_ACTION_POSITION_READ,position,0); tick+=150; GrabCalibration_Process(); }
static void run_calibration(void)
{
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_OK);
  feedback(0); sample(0,0); assert(last_forward==20);
  /* Feedback represents about 12 mm, deliberately different from nominal 20 mm. */
  wheel_finish(3129); frame.Sequence++; frame.DX=999; GrabCalibration_Process();
  sample(24,6); assert(last_forward==-20); wheel_finish(0);
  sample(0,0); GrabCalibration_Process(); event(MECHANICAL_ARM_ACTION_POSITION_READ,-1000,0);
  assert(last_pulses==44 && last_acc==axis_acc); x_finish(-1728);
  sample(x_response_dx,x_response_dy); assert(last_pulses==-44); x_finish(-1000);
}
static void measured_calibration_retained(void)
{
  GrabCalibration_CalibrationDataTypeDef old,current;
  reset(); run_calibration();
  assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_ALIGNED && GrabCalibration_IsCalibrated());
  assert(commits==1); assert(fabsf(committed[0]-2.0f)<0.01f && fabsf(committed[1]-0.5f)<0.01f);
  assert(fabsf(committed[2]-(2.0f/(728.0f*360.0f/65536.0f)))<0.001f);
  assert(GrabCalibration_DataGet(&old)); GrabCalibration_Cancel(); assert(GrabCalibration_DataGet(&current));
  assert(memcmp(&old,&current,sizeof(old))==0);
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_OK);
  assert(GrabCalibration_IsCalibrated()); chassis_ok=false; feedback(0); sample(0,0);
  assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_ERROR); assert(Error==GRAB_CAL_ERROR_CHASSIS_TX);
  assert(GrabCalibration_DataGet(&current) && memcmp(&old,&current,sizeof(old))==0);
  GrabCalibration_Cancel();chassis_ok=commit_ok=true;x_response_dx=24;x_response_dy=6;
  unsigned previous_commits=commits;run_calibration();
  assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_ERROR && Error==GRAB_CAL_ERROR_RESPONSE_INVALID);
  assert(commits==previous_commits);
  assert(GrabCalibration_DataGet(&current) && memcmp(&old,&current,sizeof(old))==0);
  GrabCalibration_Cancel();x_response_dx=2;x_response_dy=8;commit_ok=false;run_calibration();assert(commits==2);
  assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_ERROR);
  assert(GrabCalibration_DataGet(&current) && memcmp(&old,&current,sizeof(old))==0);
}
static void missing_feedback_and_zero_motion(void)
{
  reset();
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_OK);
  sample(0,0); assert(!velocity_requests);
  assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_COLLECT);
  tick+=60000; GrabCalibration_Process(); assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_COLLECT);
  feedback(0); GrabCalibration_Process(); assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_CHASSIS_MOVE);
  tick+=1000; feedback(0); GrabCalibration_Process(); tick+=150; feedback(0); GrabCalibration_Process();
  assert(GrabCalibration_StateGet()==GRAB_CAL_STATE_ERROR);
  assert(Error==GRAB_CAL_ERROR_RESPONSE_INVALID && !commits);
  GrabCalibration_Cancel(); assert(!service_busy());
}
static void rejected_start_and_error_scope(void)
{
  reset();unsigned before=requests;
  assert(GrabCalibration_Start((Camera_ColorTypeDef)0)==GRAB_CAL_RESULT_PARAM_ERROR && requests==before);
  camera_result=HAL_ERROR;
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_ERROR);
  assert(Error==GRAB_CAL_ERROR_CAMERA_TX && !moves && !velocity_requests);
  GrabCalibration_Cancel();usb_configured=false;
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_ERROR && Error==GRAB_CAL_ERROR_USB_OFF);
  usb_configured=true;
  GrabCalibration_Cancel();camera_result=HAL_BUSY;
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_ERROR && Error==GRAB_CAL_ERROR_CAMERA_BUSY);
  GrabCalibration_Cancel();camera_result=HAL_OK;
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_OK);
  before=requests;
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_BUSY && requests==before);
  GrabCalibration_Cancel();assert(!arm_stops && !stops);
}
static void wrong_channel_and_old_frames_ignored(void)
{
  unsigned i;
  reset();
  assert(GrabCalibration_Start(CAMERA_COLOR_BLUE)==GRAB_CAL_RESULT_OK); feedback(0);
  frame.Target=CAMERA_COLOR_RED; sample(0,0); assert(!velocity_requests);
  frame.Target=CAMERA_COLOR_BLUE; frame.Function=0xB3; sample(0,0); assert(!velocity_requests);
  frame.Function=0xB2;
  usb_configured=false;sample(0,0);assert(!velocity_requests && SampleIndex==0);
  usb_configured=true;
  for(i=0;i<5;i++)
  { frame.Tick=tick; frame.Sequence++; tick+=CAMERA_DATA_STALE_MS+1; GrabCalibration_Process(); }
  assert(!velocity_requests);
  feedback(0); sample(0,0); assert(velocity_requests==1);
  GrabCalibration_Cancel(); assert(!service_busy());
}
int main(void)
{
  measured_calibration_retained(); missing_feedback_and_zero_motion(); rejected_start_and_error_scope();
  wrong_channel_and_old_frames_ignored();
  puts("grab_calibration_test: OK"); return 0;
}
