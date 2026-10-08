#include "grab_calibration.h"
#include "GrabTask.h"
#include "mecanum_chassis.h"
#include "chassis_config.h"
#include "mechanical_arm_config.h"
#include <math.h>
#include <string.h>

#define SAMPLE_COUNT 5U
#define CAL_SPEED_MM_S 20.0f
#define CAL_X_DEG 5.0f
#define CAL_X_RPM 8U
#define SETTLE_MS 150U
#define POLL_MS 50U
#define X_TIMEOUT_MS 5000U
#define POSITION_FRESH_MS 1200U

typedef enum
{
  CAL_NONE, CAL_BASELINE, CAL_FORWARD_BEGIN, CAL_FORWARD_SAMPLE,
  CAL_FORWARD_RETURN, CAL_X_BASELINE, CAL_X_BEGIN, CAL_X_SAMPLE, CAL_X_RETURN
} CalibrationPhase;
static GrabCalibration_StateTypeDef State;
static GrabCalibration_ErrorTypeDef Error;
static CalibrationPhase Phase;
static uint8_t Sampling, Calibrated, SampleIndex, NeedPositionRead;
static uint32_t LastSequence, MoveStartTick, SettleTick, PollTick;
static float ForwardSpeed, ForwardDelta, XDeltaDegree;
static int16_t DxSamples[SAMPLE_COUNT], DySamples[SAMPLE_COUNT], BaselineDx, BaselineDy;
static int64_t WheelOrigin[4], XOrigin;
static uint32_t WheelOriginTick[4];
static MechanicalArm_ActionTypeDef PendingAction;
static Camera_ColorTypeDef SelectedColor;
static GrabCalibration_CalibrationDataTypeDef Calibration, Candidate;
static const char *ErrorNames[] =
{
  "none", "camera_tx", "motor_ack_timeout", "motor_arrival_timeout", "motor_error", "response_invalid", "internal", "chassis_tx", "motor_busy", "motor_param", "motor_tx", "camera_busy", "usb_off"
};
static GrabCalibration_ErrorTypeDef motor_error(MechanicalArm_ResultTypeDef result)
{
  switch(result)
  {
    case MECHANICAL_ARM_RESULT_BUSY: return GRAB_CAL_ERROR_MOTOR_BUSY;
    case MECHANICAL_ARM_RESULT_PARAM_ERROR: return GRAB_CAL_ERROR_MOTOR_PARAM;
    case MECHANICAL_ARM_RESULT_TX_ERROR: return GRAB_CAL_ERROR_MOTOR_TX;
    case MECHANICAL_ARM_RESULT_ACK_TIMEOUT: return GRAB_CAL_ERROR_MOTOR_ACK_TIMEOUT;
    default: return GRAB_CAL_ERROR_MOTOR_ERROR;
  }
}
static uint8_t service_busy(void)
{ return Sampling && State!=GRAB_CAL_STATE_IDLE && State!=GRAB_CAL_STATE_ALIGNED && State!=GRAB_CAL_STATE_ERROR; }
static void fail(GrabCalibration_ErrorTypeDef error)
{
  /* GrabTask owns error parking; the sampler only reports its cause. */
  Camera_RequestStop();
  State=GRAB_CAL_STATE_ERROR; Error=error;
}
static int16_t median(const int16_t *data)
{
  int16_t sorted[SAMPLE_COUNT],value;
  unsigned i,j;
  memcpy(sorted,data,sizeof(sorted));
  for(i=1;i<SAMPLE_COUNT;i++)
  {
    value=sorted[i]; j=i;
    while(j && sorted[j-1]>value) { sorted[j]=sorted[j-1]; j--; }
    sorted[j]=value;
  }
  return sorted[SAMPLE_COUNT/2];
}
static void collect(CalibrationPhase phase)
{
  Camera_SnapshotTypeDef snapshot;
  Camera_SnapshotGet(&snapshot); LastSequence=snapshot.Data.Sequence;
  Phase=phase; SampleIndex=0; State=GRAB_CAL_STATE_COLLECT;
}
static void settle(void)
{ SettleTick=HAL_GetTick(); State=GRAB_CAL_STATE_SETTLE; }
/* Wheel deltas use configured protocol polarity and 0x36 units, not command pulses.
 * They measure drive travel under current wheel geometry, not external car localization. */
static uint8_t wheel_baseline(void)
{
  unsigned i; Mecanum_Feedback_t feedback[4];
  for(i=0;i<4;i++)
    if(!Mecanum_FeedbackGet(i,&feedback[i]) || !feedback[i].position_valid ||
       HAL_GetTick()-feedback[i].position_ms>POSITION_FRESH_MS)return 0;
  for(i=0;i<4;i++) { WheelOrigin[i]=feedback[i].position_raw; WheelOriginTick[i]=feedback[i].position_ms; }
  return 1;
}
static uint8_t wheel_delta(float *out)
{
  static const uint8_t dirs[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,
    CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  unsigned i; Mecanum_Feedback_t f; double sum=0;
  for(i=0;i<4;i++)
  {
    if(!Mecanum_FeedbackGet(i,&f) || !f.position_valid ||
       f.position_ms==WheelOriginTick[i] || HAL_GetTick()-f.position_ms>POSITION_FRESH_MS)return 0;
    sum+=(double)(f.position_raw-WheelOrigin[i])*(dirs[i]?-1.0:1.0);
  }
  *out=(float)(sum*0.25*3.14159265358979323846*CHASSIS_WHEEL_DIAMETER_MM/
    (CHASSIS_MOTOR_TO_WHEEL_RATIO*CHASSIS_EMM_POSITION_UNITS_PER_REV));
  return 1;
}
static uint8_t chassis_start(float speed)
{
  if(!Mecanum_Velocity_Request(speed,0,0,20)) { Error=GRAB_CAL_ERROR_CHASSIS_TX; return 0; }
  ForwardSpeed=speed; MoveStartTick=HAL_GetTick(); State=GRAB_CAL_STATE_CHASSIS_MOVE;
  return 1;
}
static uint8_t x_start(float degree)
{
  int32_t pulses=(int32_t)(degree*MECHANICAL_ARM_COMMAND_PULSES_PER_REV/
    MECHANICAL_ARM_DEGREES_PER_REV+(degree<0?-0.5f:0.5f));
  MechanicalArm_ConfigTypeDef config; MechanicalArm_ResultTypeDef result;
  if(!MechanicalArm_ConfigGet(MECHANICAL_ARM_AXIS_X,&config))
  { Error=GRAB_CAL_ERROR_MOTOR_PARAM; return 0; }
  result=MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_X,pulses,CAL_X_RPM,
    config.Acceleration,MECHANICAL_ARM_POSITION_RELATIVE_CURRENT);
  if(result!=MECHANICAL_ARM_RESULT_NONE) { Error=motor_error(result); return 0; }
  MoveStartTick=HAL_GetTick(); PendingAction=MECHANICAL_ARM_ACTION_POSITION;
  State=GRAB_CAL_STATE_X_ACK; return 1;
}
static uint8_t response_valid(void)
{
  float f=sqrtf(Candidate.ForwardDxPerMm*Candidate.ForwardDxPerMm+
    Candidate.ForwardDyPerMm*Candidate.ForwardDyPerMm);
  float x=sqrtf(Candidate.XDxPerDegree*Candidate.XDxPerDegree+
    Candidate.XDyPerDegree*Candidate.XDyPerDegree);
  float cross=fabsf(Candidate.ForwardDxPerMm*Candidate.XDyPerDegree-
    Candidate.ForwardDyPerMm*Candidate.XDxPerDegree);
  return isfinite(f) && isfinite(x) && f*fabsf(ForwardDelta)>=3.0f &&
    x*fabsf(XDeltaDegree)>=3.0f && cross>=0.2f*f*x;
}
void GrabCalibration_Init(void)
{
  memset(&Calibration,0,sizeof(Calibration)); memset(&Candidate,0,sizeof(Candidate));
  State=GRAB_CAL_STATE_IDLE; Error=GRAB_CAL_ERROR_NONE;
  Phase=CAL_NONE; Sampling=Calibrated=NeedPositionRead=0; PendingAction=MECHANICAL_ARM_ACTION_NONE;
}
GrabCalibration_ResultTypeDef GrabCalibration_Start(Camera_ColorTypeDef color)
{
  HAL_StatusTypeDef result; Camera_SnapshotTypeDef snapshot;
  if(color<CAMERA_COLOR_RED || color>CAMERA_COLOR_LIGHT_BLUE)return GRAB_CAL_RESULT_PARAM_ERROR;
  if(service_busy())return GRAB_CAL_RESULT_BUSY;
  Sampling=1; NeedPositionRead=0; Error=GRAB_CAL_ERROR_NONE; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  SelectedColor=color;
  ForwardDelta=XDeltaDegree=0; memset(&Candidate,0,sizeof(Candidate));
  result=Camera_MaterialStart(color);
  if(result!=HAL_OK)
  {
    Camera_SnapshotGet(&snapshot);
    Error=result==HAL_BUSY?GRAB_CAL_ERROR_CAMERA_BUSY:
      (!snapshot.UsbConfigured?GRAB_CAL_ERROR_USB_OFF:GRAB_CAL_ERROR_CAMERA_TX);
    State=GRAB_CAL_STATE_ERROR; return GRAB_CAL_RESULT_ERROR;
  }
  collect(CAL_BASELINE); return GRAB_CAL_RESULT_OK;
}
void GrabCalibration_Process(void)
{
  uint32_t now=HAL_GetTick(); Camera_SnapshotTypeDef snapshot; int16_t dx,dy;
  MechanicalArm_ResultTypeDef result;
  if(!service_busy())return;
  if(State==GRAB_CAL_STATE_CHASSIS_MOVE)
  {
    if(now-MoveStartTick>=1000U)
    { if(!Mecanum_Test_Stop())fail(GRAB_CAL_ERROR_CHASSIS_TX); else settle(); }
    else if(!Mecanum_Velocity_Request(ForwardSpeed,0,0,20))fail(GRAB_CAL_ERROR_CHASSIS_TX);
    return;
  }
  if(State==GRAB_CAL_STATE_X_ARRIVAL)
  {
    if(now-MoveStartTick>X_TIMEOUT_MS) { fail(GRAB_CAL_ERROR_MOTOR_ARRIVAL_TIMEOUT); return; }
    if(now-PollTick>=POLL_MS && !MechanicalArm_IsBusy())
    {
      PollTick=now; result=MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_X);
      if(result==MECHANICAL_ARM_RESULT_NONE)
      { PendingAction=MECHANICAL_ARM_ACTION_STATE; State=GRAB_CAL_STATE_X_STATE; }
      else if(result!=MECHANICAL_ARM_RESULT_BUSY)fail(motor_error(result));
    }
    return;
  }
  if(State==GRAB_CAL_STATE_X_ACK || State==GRAB_CAL_STATE_X_STATE)return;
  if((Phase==CAL_X_BEGIN || NeedPositionRead) && PendingAction==MECHANICAL_ARM_ACTION_NONE)
  {
    if(MechanicalArm_IsBusy())return;
    result=MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_X);
    if(result==MECHANICAL_ARM_RESULT_NONE)
    { PendingAction=MECHANICAL_ARM_ACTION_POSITION_READ; State=GRAB_CAL_STATE_X_STATE; }
    else if(result!=MECHANICAL_ARM_RESULT_BUSY)fail(motor_error(result));
    return;
  }
  if(State==GRAB_CAL_STATE_SETTLE)
  {
    if(now-SettleTick<SETTLE_MS)return;
    if(Phase==CAL_FORWARD_SAMPLE)
    {
      if(!wheel_delta(&ForwardDelta))return;
      if(!isfinite(ForwardDelta) || fabsf(ForwardDelta)<=0.1f)
      { fail(GRAB_CAL_ERROR_RESPONSE_INVALID); return; }
      collect(CAL_FORWARD_SAMPLE);
    }
    else if(Phase==CAL_FORWARD_RETURN)collect(CAL_X_BASELINE);
    else if(Phase==CAL_X_SAMPLE)collect(CAL_X_SAMPLE);
    else if(Phase==CAL_X_RETURN)
    {
      if(!response_valid() || !GrabTask_VisualResponseSet(Candidate.ForwardDxPerMm,
        Candidate.ForwardDyPerMm,Candidate.XDxPerDegree,Candidate.XDyPerDegree))
      { fail(GRAB_CAL_ERROR_RESPONSE_INVALID); return; }
      Calibration=Candidate; Calibrated=1; Camera_RequestStop(); State=GRAB_CAL_STATE_ALIGNED;
    }
    return;
  }
  if(Phase==CAL_FORWARD_BEGIN)
  {
    if(wheel_baseline())
    { Phase=CAL_FORWARD_SAMPLE; if(!chassis_start(CAL_SPEED_MM_S))fail(Error); }
    return;
  }
  Camera_SnapshotGet(&snapshot);
  if(!snapshot.RequestActive || !snapshot.UsbConfigured || !snapshot.TargetValid ||
    !snapshot.HasValidData || snapshot.RequestFunction!=0xB2 ||
    snapshot.RequestTarget!=(uint8_t)SelectedColor || snapshot.Data.Function!=0xB2 ||
    snapshot.Data.Target!=(uint8_t)SelectedColor || now-snapshot.Data.Tick>CAMERA_DATA_STALE_MS ||
    snapshot.Data.Sequence==LastSequence)return;
  LastSequence=snapshot.Data.Sequence;
  DxSamples[SampleIndex]=snapshot.Data.DX; DySamples[SampleIndex]=snapshot.Data.DY;
  if(++SampleIndex<SAMPLE_COUNT)return;
  dx=median(DxSamples); dy=median(DySamples); SampleIndex=0;
  switch(Phase)
  {
    case CAL_BASELINE:
      BaselineDx=dx; BaselineDy=dy; Phase=CAL_FORWARD_BEGIN;
      if(wheel_baseline()) { Phase=CAL_FORWARD_SAMPLE; if(!chassis_start(CAL_SPEED_MM_S))fail(Error); }
      break;
    case CAL_FORWARD_SAMPLE:
      Candidate.ForwardDxPerMm=((float)dx-BaselineDx)/ForwardDelta;
      Candidate.ForwardDyPerMm=((float)dy-BaselineDy)/ForwardDelta;
      Phase=CAL_FORWARD_RETURN; if(!chassis_start(-CAL_SPEED_MM_S))fail(Error);
      break;
    case CAL_X_BASELINE:
      BaselineDx=dx; BaselineDy=dy; Phase=CAL_X_BEGIN; break;
    case CAL_X_SAMPLE:
      Candidate.XDxPerDegree=((float)dx-BaselineDx)/XDeltaDegree;
      Candidate.XDyPerDegree=((float)dy-BaselineDy)/XDeltaDegree;
      Phase=CAL_X_RETURN; if(!x_start(-CAL_X_DEG))fail(Error);
      break;
    default: fail(GRAB_CAL_ERROR_INTERNAL); break;
  }
}
uint8_t GrabCalibration_MotorEventHandle(const MechanicalArm_EventTypeDef *event)
{
  if(!event || !service_busy() || event->Axis!=MECHANICAL_ARM_AXIS_X || event->Action!=PendingAction)return 0;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  if(event->Result!=MECHANICAL_ARM_RESULT_OK) { fail(motor_error(event->Result)); return 1; }
  if(event->Action==MECHANICAL_ARM_ACTION_POSITION)
  { MoveStartTick=HAL_GetTick(); PollTick=MoveStartTick; State=GRAB_CAL_STATE_X_ARRIVAL; }
  else if(event->Action==MECHANICAL_ARM_ACTION_STATE)
  {
    uint8_t flags=event->StateFlags[MECHANICAL_ARM_AXIS_X];
    if(flags&0x0CU)fail(GRAB_CAL_ERROR_MOTOR_ERROR);
    else if(flags&0x02U)
    { NeedPositionRead=1; State=GRAB_CAL_STATE_COLLECT; }
    else State=GRAB_CAL_STATE_X_ARRIVAL;
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_POSITION_READ)
  {
    NeedPositionRead=0;
    if(Phase==CAL_X_BEGIN)
    { XOrigin=event->CurrentPositionRaw; Phase=CAL_X_SAMPLE; if(!x_start(CAL_X_DEG))fail(Error); }
    else
    {
      if(Phase==CAL_X_SAMPLE)
      {
        XDeltaDegree=(float)(event->CurrentPositionRaw-XOrigin)*360.0f/65536.0f*
          (MECHANICAL_ARM_X_POSITIVE_DIR?-1.0f:1.0f);
        if(!isfinite(XDeltaDegree) || fabsf(XDeltaDegree)<0.01f)
        { fail(GRAB_CAL_ERROR_RESPONSE_INVALID); return 1; }
      }
      settle();
    }
  }
  return 1;
}
void GrabCalibration_Cancel(void)
{
  if(service_busy()) Camera_RequestStop();
  Sampling=NeedPositionRead=0; Phase=CAL_NONE; PendingAction=MECHANICAL_ARM_ACTION_NONE;
  State=GRAB_CAL_STATE_IDLE; Error=GRAB_CAL_ERROR_NONE;
}
GrabCalibration_StateTypeDef GrabCalibration_StateGet(void) { return State; }
uint8_t GrabCalibration_IsCalibrated(void) { return Calibrated; }
const char *GrabCalibration_StateNameGet(void)
{
  static const char *names[]={"idle","collect","chassis_move","x_ack","x_arrival","x_state","settle","complete","error"};
  return names[(unsigned)State];
}
const char *GrabCalibration_ErrorNameGet(void)
{ return ErrorNames[(unsigned)Error]; }
uint8_t GrabCalibration_DataGet(GrabCalibration_CalibrationDataTypeDef *data)
{ if(!data || !Calibrated)return 0; *data=Calibration; return 1; }
