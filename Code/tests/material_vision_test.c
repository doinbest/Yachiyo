/* Real calibration sampler and compatibility facade, with asynchronous fixtures. */
#include <assert.h>
#include <stdio.h>
#include "../template/App/MaterialVision.c"
static uint32_t tick;
static Camera_DataTypeDef frame;
static bool peer_busy, material_active, calibrating, reference_valid;
static bool start_ok=true, commit_ok=true, chassis_ok=true, route_busy, motion_busy;
static uint8_t motor_busy, axis_acc=37, last_acc;
static unsigned requests, moves, stops, arm_stops, task_stops, commits, velocity_requests;
static int32_t last_pulses;
static float last_forward, committed[4];
static Mecanum_Feedback_t wheels[4];
static GrabTask_Status_t grab_status;
static HAL_StatusTypeDef camera_result=HAL_OK;
static MechanicalArm_ResultTypeDef motor_result=MECHANICAL_ARM_RESULT_NONE;
uint32_t HAL_GetTick(void) { return tick; }
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
bool GrabTask_IsBusy(void) { return peer_busy; }
bool GrabTask_MaterialActive(void) { return material_active; }
bool GrabTask_MaterialCalibrating(void) { return material_active && peer_busy && calibrating; }
bool GrabTask_MaterialReferenceValid(void) { return reference_valid; }
bool GrabTask_MaterialReferenceSet(void)
{ if(peer_busy || motor_busy || route_busy || motion_busy)return false; reference_valid=true; return true; }
bool GrabTask_MaterialStart(uint8_t color,bool calibration)
{
  if(!start_ok)return false;
  material_active=peer_busy=true; calibrating=calibration;
  grab_status.state=calibration?GRAB_CALIBRATION:GRAB_MATERIAL_SEARCH; grab_status.reason="none";
  if(calibration)return MaterialVision_CalibrationBegin((Camera_ColorTypeDef)color)==MATERIAL_VISION_RESULT_OK;
  return true;
}
void GrabTask_Stop(void)
{ task_stops++; if(calibrating)MaterialVision_CalibrationCancel(); peer_busy=false; }
void GrabTask_StatusGet(GrabTask_Status_t *out) { *out=grab_status; }
bool GrabTask_VisualResponseSet(float a,float b,float c,float d)
{ commits++; if(!commit_ok)return false; committed[0]=a; committed[1]=b; committed[2]=c; committed[3]=d; return true; }
void Camera_SnapshotGet(Camera_SnapshotTypeDef *out)
{ memset(out,0,sizeof(*out)); out->Data=frame; out->RequestFunction=0xB2; out->RequestTarget=CAMERA_COLOR_BLUE; out->RequestActive=out->UsbConfigured=1; out->TargetValid=out->HasValidData=frame.Sequence!=0; }
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
  tick=1; memset(&frame,0,sizeof(frame)); memset(wheels,0,sizeof(wheels)); memset(&grab_status,0,sizeof(grab_status));
  peer_busy=material_active=calibrating=reference_valid=route_busy=motion_busy=false; motor_busy=0;
  start_ok=commit_ok=chassis_ok=true; camera_result=HAL_OK; motor_result=MECHANICAL_ARM_RESULT_NONE;
  requests=moves=stops=arm_stops=task_stops=commits=velocity_requests=0; MaterialVision_Init();
  assert(!stops && !arm_stops); assert(!MaterialVision_IsReferenceValid());
}
static void feedback(int64_t raw)
{ unsigned i; for(i=0;i<4;i++) { wheels[i].position_valid=true; wheels[i].position_ms=tick; wheels[i].position_raw=i<2?raw:-raw; } }
static void sample(int16_t dx,int16_t dy)
{ unsigned i; for(i=0;i<5;i++) { frame.DX=dx; frame.DY=dy; frame.Tick=++tick; frame.Sequence++; MaterialVision_CalibrationProcess(); } }
static void event(MechanicalArm_ActionTypeDef action,int32_t position,uint8_t flags)
{ MechanicalArm_EventTypeDef e={0}; e.Axis=MECHANICAL_ARM_AXIS_X; e.Action=action; e.Result=MECHANICAL_ARM_RESULT_OK; e.CurrentPosition=position; e.CurrentPositionRaw=position; e.StateFlags[MECHANICAL_ARM_AXIS_X]=flags; assert(MaterialVision_CalibrationEvent(&e)); }
static void wheel_finish(int64_t raw)
{ tick+=1000; feedback(raw); MaterialVision_CalibrationProcess(); assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_SETTLE); tick+=150; feedback(raw); MaterialVision_CalibrationProcess(); }
static void x_finish(int32_t position)
{ event(MECHANICAL_ARM_ACTION_POSITION,0,0); tick+=50; MaterialVision_CalibrationProcess(); event(MECHANICAL_ARM_ACTION_STATE,0,2); MaterialVision_CalibrationProcess(); event(MECHANICAL_ARM_ACTION_POSITION_READ,position,0); tick+=150; MaterialVision_CalibrationProcess(); }
static void run_calibration(void)
{
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK);
  feedback(0); sample(0,0); assert(last_forward==20);
  /* Feedback represents about 12 mm, deliberately different from nominal 20 mm. */
  wheel_finish(3129); frame.Sequence++; frame.DX=999; MaterialVision_CalibrationProcess();
  sample(24,6); assert(last_forward==-20); wheel_finish(0);
  sample(0,0); MaterialVision_CalibrationProcess(); event(MECHANICAL_ARM_ACTION_POSITION_READ,-1000,0);
  assert(last_pulses==44 && last_acc==axis_acc); x_finish(-1728);
  sample(2,8); assert(last_pulses==-44); x_finish(-1000);
}
static void facade(void)
{
  reset(); assert(MaterialVision_Start(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_NOT_READY);
  assert(MaterialVision_Start((Camera_ColorTypeDef)0)==MATERIAL_VISION_RESULT_PARAM_ERROR);
  assert(MaterialVision_ReferenceSet()==MATERIAL_VISION_RESULT_OK);
  peer_busy=true; assert(!MaterialVision_IsBusy()); assert(MaterialVision_Start(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_BUSY);
  assert(MaterialVision_ReferenceSet()==MATERIAL_VISION_RESULT_BUSY && reference_valid); peer_busy=false;
  assert(MaterialVision_Start(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK); assert(MaterialVision_IsBusy());
  assert(!requests && !moves); assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_SEARCH);
  grab_status.state=GRAB_ALIGN; assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_TARGET_LOCK);
  grab_status.state=GRAB_ERROR; grab_status.reason="chassis_tx"; assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_CHASSIS_TX);
  MaterialVision_Stop(); assert(task_stops==1 && reference_valid); assert(!MaterialVision_IsBusy());
}
static void measured_calibration_retained(void)
{
  MaterialVision_CalibrationDataTypeDef old,current;
  reset(); reference_valid=true; run_calibration();
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_ALIGNED && MaterialVision_IsCalibrated());
  assert(commits==1); assert(fabsf(committed[0]-2.0f)<0.01f && fabsf(committed[1]-0.5f)<0.01f);
  assert(fabsf(committed[2]-(2.0f/(728.0f*360.0f/65536.0f)))<0.001f);
  assert(MaterialVision_CalibrationGet(&old)); MaterialVision_Stop(); assert(MaterialVision_CalibrationGet(&current));
  assert(memcmp(&old,&current,sizeof(old))==0 && reference_valid);
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK);
  assert(MaterialVision_IsCalibrated()); chassis_ok=false; feedback(0); sample(0,0);
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_ERROR); assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_CHASSIS_TX);
  assert(MaterialVision_CalibrationGet(&current) && memcmp(&old,&current,sizeof(old))==0);
  MaterialVision_Stop(); chassis_ok=true; commit_ok=false; run_calibration(); assert(commits==2);
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_ERROR);
  assert(MaterialVision_CalibrationGet(&current) && memcmp(&old,&current,sizeof(old))==0);
}
static void missing_feedback_and_zero_motion(void)
{
  reset(); reference_valid=true;
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK);
  sample(0,0); assert(!velocity_requests);
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_COLLECT);
  tick+=60000; MaterialVision_CalibrationProcess(); assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_COLLECT);
  feedback(0); MaterialVision_CalibrationProcess(); assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_CHASSIS_MOVE);
  tick+=1000; feedback(0); MaterialVision_CalibrationProcess(); tick+=150; feedback(0); MaterialVision_CalibrationProcess();
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_ERROR);
  assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_RESPONSE_INVALID && !commits);
  MaterialVision_Stop(); assert(reference_valid);
}
static void rejected_start_and_error_scope(void)
{
  reset(); reference_valid=true; start_ok=false; grab_status.reason="usb_off";
  assert(MaterialVision_Start(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_ERROR);
  assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_USB_OFF && !requests && !moves);
  MaterialVision_Stop(); assert(!task_stops);
  start_ok=true; motor_busy=1;
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_BUSY);
  motor_busy=0; route_busy=true;
  assert(MaterialVision_Start(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_BUSY);
}
static void wrong_channel_and_old_frames_ignored(void)
{
  unsigned i;
  reset(); reference_valid=true;
  assert(MaterialVision_CalibrationStart(CAMERA_COLOR_BLUE)==MATERIAL_VISION_RESULT_OK); feedback(0);
  frame.Target=CAMERA_COLOR_RED; sample(0,0); assert(!velocity_requests);
  frame.Target=CAMERA_COLOR_BLUE; frame.Function=0xB3; sample(0,0); assert(!velocity_requests);
  frame.Function=0xB2;
  for(i=0;i<5;i++)
  { frame.Tick=tick; frame.Sequence++; tick+=CAMERA_DATA_STALE_MS+1; MaterialVision_CalibrationProcess(); }
  assert(!velocity_requests);
  feedback(0); sample(0,0); assert(velocity_requests==1);
  MaterialVision_Stop(); assert(reference_valid && !MaterialVision_IsBusy());
}
static void unified_failure_reason_preserved(void)
{
  reset();material_active=peer_busy=true;
  grab_status.state=GRAB_STOPPING;grab_status.result="failed";grab_status.reason="vision_usb_off";
  assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_USB_OFF);
  assert(!strcmp(MaterialVision_ErrorNameGet(),"vision_usb_off"));
  grab_status.state=GRAB_ERROR;grab_status.reason="calibration_x_limit";
  assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_X_LIMIT);
  assert(!strcmp(MaterialVision_ErrorNameGet(),"calibration_x_limit"));
  calibrating=true;grab_status.state=GRAB_PREPARE;
  State=MATERIAL_VISION_STATE_ALIGNED;Error=MATERIAL_VISION_ERROR_MOTOR_ERROR;
  assert(MaterialVision_StateGet()==MATERIAL_VISION_STATE_COLLECT);
  assert(MaterialVision_ErrorGet()==MATERIAL_VISION_ERROR_NONE);
}
int main(void)
{
  facade(); measured_calibration_retained(); missing_feedback_and_zero_motion(); rejected_start_and_error_scope();
  wrong_channel_and_old_frames_ignored();
  unified_failure_reason_preserved();
  puts("material_vision_test: OK"); return 0;
}
