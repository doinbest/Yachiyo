#include "GrabTask.h"
#include "Arm.h"
#include "ArmVision.h"
#include "MaterialVision.h"
#include "Camera.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include "hwt101_calibration.h"
#include "mecanum_chassis.h"
#include "mechanical_arm_config.h"
#include "console_tx.h"
#include "turntable.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* RAM-only mechanical references are restored after boot homing.
 * Vision starts from an explicitly unmeasured trial model, adjustable with grab set. */
enum
{
  C_X_PPM, C_Z_PPM, C_X_MIN, C_X_MAX, C_Z_MIN, C_Z_MAX,
  C_X_PRE, C_Z_OBSERVE, C_Z_GRAB, C_Z_LIFT, C_POS_TOL, C_STOP_SPEED,
  C_CLOSE_MS, C_AGE_MS, C_FEEDBACK_MS, C_ACCEL, C_TRAVEL,
  C_LEAD, C_DX_TOL, C_DY_TOL, C_REF_U, C_REF_V,
  C_J00, C_J01, C_J02, C_J10, C_J11, C_J12,
  C_LAMBDA, C_BODY_SPEED, C_X_SPEED, C_Z_SPEED, C_STABLE_MS, C_FRAMES,
  C_RETRIES, C_X_WEIGHT, C_Z_PLACE, C_LOSS_GRACE_MS,
  C_Z_PROC_GRAB, C_Z_PROC_LIFT, C_Z_CAR_LIFT, C_Z_PROC_PLACE,
  C_Z_TEMP_PLACE, C_Z_STACK_PLACE, C_X_CAR, C_BASE_CAR_OFFSET, C_COUNT
};
static const char *const Keys[C_COUNT] = {
  "x_ppm","z_ppm","x_min","x_max","z_min","z_max","x_pre","z_observe",
  "z_grab","z_lift","pos_tol","stop_speed","close_ms","age_ms","feedback_ms",
  "accel","travel_mm","lead_mm","dx_tol","dy_tol","ref_u","ref_v",
  "j00","j01","j02","j10","j11","j12","lambda","body_speed","x_speed",
  "z_speed","stable_ms","frames","retries","x_weight","z_place","loss_grace_ms",
  "z_proc_grab","z_proc_lift","z_car_lift","z_proc_place","z_temp_place",
  "z_stack_place","x_car","base_car_offset"
};
static const char *const States[] = {"idle","prepare","acquire","align","settle",
  "descend","close","lift","turn","place","release","retract","hold","stopping","error","homing","vision_pause","vision_grace",
  "x_retract","index","car_extend","return_turn","external_extend","observe_z","observe_x","complete"};
typedef struct
{
  float mm, speed, target;
  int64_t raw;
  uint32_t position_tick, state_tick, quiet_since, command_tick;
  uint8_t flags;
  bool position_valid, state_valid, quiet, commanded;
} GrabAxis_t;
static float Config[C_COUNT];
static GrabTask_Status_t Status;
static GrabAxis_t Axes[3]; /* X/Z in mm, Base in motor degrees; driver signs normalized. */
static MechanicalArm_ActionTypeDef PendingAction;
static MechanicalArm_AxisTypeDef PendingAxis;
static uint32_t StartTick, StateTick, LastPoll, LastControl, RequestTick;
static uint32_t LastSequence, LastInvalidCount, StableTick, StopTick, WheelQuietTick;
static uint32_t WheelGroupTick, CaptureCount;
static int64_t WheelOrigin[4], WheelLastPosition[4];
static uint32_t WheelLastPositionTick[4];
#define GRAB_RECOVERY_WINDOW_MS 1500U
#define GRAB_CONTROL_PERIOD_MS 100U
#define GRAB_RECOVERY_OBSERVATIONS 3U
#define GRAB_RECOVERY_LIMIT 2U
static unsigned PrepareStep, PollIndex, Retries;
static uint32_t RecoveryUsed, RecoveryCount, RecoveryTick, RecoverySequence;
static bool RecoveryWaiting, RecoveryRestartPending;
static bool LossActive;
static uint32_t LossTick, LossLastMs, LossMaxMs, GraceCount, GraceSequence;
static bool HaveCapture, VisualStable, StopArmSent, StopArmTx, StopChassisSent;
static bool WheelQuiet, WheelBaseline, FinishError, FreshVision;
static bool StopEvidenceStarted;
static bool FeedbackSelected;
static float XIntegral;
static uint32_t LastReport;
static bool ReportPending;
static void status_write(bool debug);
static void align_control(uint32_t now);
static float Heading, PreviousVelocity[3];
static Camera_DataTypeDef Vision;
static Camera_ColorTypeDef ActiveColor;
static bool BootActive, BootStopping, BootFailed, BootHomeSent;
static unsigned BootAxisIndex, BootStep, BootGood;
static int64_t BootPositionRaw;
static bool BootPositionValid;
static uint8_t BootHomeFlags, BootMotorFlags;
static bool BootHomeFlagsValid, BootMotorFlagsValid;
static uint32_t BootTick, BootPoll;
static const MechanicalArm_AxisTypeDef BootAxes[3]={MECHANICAL_ARM_AXIS_Z,MECHANICAL_ARM_AXIS_X,MECHANICAL_ARM_AXIS_BASE};
static void boot_stop(const char *reason,bool failed);
static void boot_process(uint32_t now);
static uint8_t boot_event(const MechanicalArm_EventTypeDef *event);
typedef enum { OP_ALIGN, OP_STORE, OP_TAKE, OP_RETURN } GrabOperation_t;
typedef enum { SCENE_RAW, SCENE_PROC, SCENE_ROUGH, SCENE_TEMP1, SCENE_TEMP2 } GrabScene_t;
static GrabOperation_t Operation;
static GrabScene_t Scene;
static bool OutsideReferenceValid, OutsidePose, ToCar, IndexSubmitted, InventoryTouched;
static float OutsideBase, ObserveX, ObserveZ, ExternalX;
static const char *const OperationNames[]={"align","store","take","return"};
static const char *const SceneNames[]={"raw","proc","rough","temp1","temp2"};
static unsigned grab_key(void) { return Scene==SCENE_PROC?C_Z_PROC_GRAB:C_Z_GRAB; }
static unsigned lift_key(void) { return Scene==SCENE_PROC?C_Z_PROC_LIFT:C_Z_LIFT; }
static unsigned release_key(void)
{ return Scene==SCENE_TEMP1?C_Z_TEMP_PLACE:(Scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE); }
static void invalidate_inventory(void)
{
  if(InventoryTouched && Status.slot)
    (void)Turntable_InventorySet(Status.slot,TURNTABLE_UNKNOWN,0);
  InventoryTouched=false;
}

static float clamp(float value, float low, float high)
{ return value < low ? low : (value > high ? high : value); }
static bool fixed_mode(void) { return strcmp(Status.mode, "fixed") == 0; }
static Camera_ColorTypeDef target_color(void)
{ return ActiveColor; }
static Camera_ColorTypeDef default_color(const char *mode)
{ return mode && !strcmp(mode,"align") ? GRAB_ALIGN_COLOR : GRAB_PICK_COLOR; }
static bool valid_mode(const char *mode)
{ return mode && (!strcmp(mode,"fixed") || !strcmp(mode,"align") || !strcmp(mode,"pick")); }
static void enter(GrabTask_State_t state)
{ Status.state = state; StateTick = HAL_GetTick(); ReportPending=true; }

/* Weighted minimum-norm solution v = -lambda W J' (J W J')^-1 e.
 * Fade X only toward the bound it would approach; leaving a bound stays possible. */
static bool solve(float dx, float dy, float x, float result[3])
{
  float j[2][3], w[3] = {1,1,0}, a,b,d,det,s0,s1,scale=1;
  float span = Config[C_X_MAX]-Config[C_X_MIN]; unsigned i, pass;
  w[2]=Config[C_X_WEIGHT];
  for(pass=0;pass<2;pass++)
  {
    a=b=d=0;
    for(i=0;i<3;i++)
    {
      j[0][i]=Config[C_J00+i]; j[1][i]=Config[C_J10+i];
      a+=w[i]*j[0][i]*j[0][i]; b+=w[i]*j[0][i]*j[1][i]; d+=w[i]*j[1][i]*j[1][i];
    }
    det=a*d-b*b;
    if(!isfinite(det) || a<=0 || d<=0 || det<=1e-5f*a*d) return false;
    s0=(-Config[C_LAMBDA])*(d*dx-b*dy)/det;
    s1=(-Config[C_LAMBDA])*(a*dy-b*dx)/det;
    for(i=0;i<3;i++) result[i]=w[i]*(j[0][i]*s0+j[1][i]*s1);
    if(pass==0)
    {
      float room=result[2]>=0 ? Config[C_X_MAX]-x : x-Config[C_X_MIN];
      w[2]=Config[C_X_WEIGHT]*clamp(room/fmaxf(Config[C_LEAD],span*.1f),0,1);
    }
  }
  if(hypotf(result[0],result[1])>Config[C_BODY_SPEED])
    scale=Config[C_BODY_SPEED]/hypotf(result[0],result[1]);
  if(fabsf(result[2])*scale>Config[C_X_SPEED]) scale=Config[C_X_SPEED]/fabsf(result[2]);
  for(i=0;i<3;i++) result[i]*=scale;
  return true;
}

/* 2026-09-29 bounded live probes at the home observation pose:
 * forward body command increases CX; left body command decreases CY.
 * At Base home the camera faces body-left; X extension moves it toward the
 * material in the same direction as left translation (user-confirmed geometry).
 * Errors are DX=CX-ref_u and DY=ref_v-CY; both left and extension increase DY.
 * 2 px/mm remains an initial control scale, NOT a measured camera calibration. */
static bool VisualModelCustom;
static void visual_defaults_restore(void)
{
  Config[C_REF_U]=334; Config[C_REF_V]=338;
  Config[C_J00]=2; Config[C_J01]=0; Config[C_J02]=0;
  Config[C_J10]=0; Config[C_J11]=2; Config[C_J12]=2;
  VisualModelCustom=false;
}
static const char *visual_model_name(void)
{
  unsigned i;
  if(!isfinite(Config[C_REF_U]) || !isfinite(Config[C_REF_V])) return "unset";
  for(i=C_J00;i<=C_J12;i++) if(!isfinite(Config[i])) return "unset";
  return VisualModelCustom?"custom":"initial";
}

static bool configuration_required_for(unsigned key,const char *mode,GrabOperation_t op,GrabScene_t scene)
{
  if(key==C_X_PPM || key==C_X_MIN || key==C_X_MAX || key==C_Z_PPM ||
     key==C_Z_MIN || key==C_Z_MAX || key==C_POS_TOL || key==C_STOP_SPEED ||
     key==C_FEEDBACK_MS) return true;
  if(op!=OP_ALIGN)
  {
    if(key==C_Z_CAR_LIFT || (op!=OP_RETURN && (key==C_Z_PLACE || key==C_X_CAR ||
       key==C_BASE_CAR_OFFSET || key==C_CLOSE_MS))) return true;
    if(op==OP_STORE && (key==(scene==SCENE_PROC?C_Z_PROC_GRAB:C_Z_GRAB) ||
       key==(scene==SCENE_PROC?C_Z_PROC_LIFT:C_Z_LIFT))) return true;
    if(op==OP_TAKE && key==(scene==SCENE_TEMP1?C_Z_TEMP_PLACE:
       (scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE))) return true;
  }
  return strcmp(mode,"fixed") && key<=C_J12 && key!=C_Z_GRAB &&
    key!=C_Z_LIFT && key!=C_CLOSE_MS;
}
static bool configuration_required(unsigned key,const char *mode)
{ return configuration_required_for(key,mode,Operation,Scene); }
static const char *configuration_missing_for(const char *mode,GrabOperation_t op,GrabScene_t scene)
{
  unsigned i; float test[3];
  if(!valid_mode(mode)) return "mode";
  for(i=0;i<C_COUNT;i++)
    if(configuration_required_for(i,mode,op,scene) && !isfinite(Config[i])) return Keys[i];
  if(Config[C_Z_MIN]>=Config[C_Z_MAX]) return "z_range";
  if(Config[C_X_MIN]>=Config[C_X_MAX]) return "x_range";
  if(op!=OP_ALIGN)
  {
    unsigned pickup=scene==SCENE_PROC?C_Z_PROC_GRAB:C_Z_GRAB;
    unsigned lifted=scene==SCENE_PROC?C_Z_PROC_LIFT:C_Z_LIFT;
    unsigned released=scene==SCENE_TEMP1?C_Z_TEMP_PLACE:(scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE);
    if(Config[C_Z_CAR_LIFT]<Config[C_Z_MIN] || Config[C_Z_CAR_LIFT]>Config[C_Z_MAX]) return "z_car_lift";
    if(op!=OP_RETURN && (Config[C_Z_PLACE]<Config[C_Z_MIN] || Config[C_Z_PLACE]>Config[C_Z_MAX] ||
       Config[C_Z_CAR_LIFT]<=Config[C_Z_PLACE])) return "z_car_targets";
    if(op!=OP_RETURN && (Config[C_X_CAR]<Config[C_X_MIN] || Config[C_X_CAR]>Config[C_X_MAX])) return "x_car";
    if(op==OP_STORE && (Config[pickup]<Config[C_Z_MIN] || Config[pickup]>Config[C_Z_MAX] ||
       Config[lifted]<Config[C_Z_MIN] || Config[lifted]>Config[C_Z_MAX] || Config[lifted]<=Config[pickup])) return "z_targets";
    if(op==OP_TAKE && (Config[released]<Config[C_Z_MIN] || Config[released]>Config[C_Z_MAX])) return Keys[released];
  }
  if(fmaxf(fabsf(Config[C_Z_MIN]),fabsf(Config[C_Z_MAX]))*Config[C_Z_PPM]>2147483000.0f) return "z_pulses";
  if(fmaxf(fabsf(Config[C_X_MIN]),fabsf(Config[C_X_MAX]))*Config[C_X_PPM]>2147483000.0f) return "x_pulses";
  if(Config[C_Z_SPEED]*Config[C_Z_PPM]*60/3200<1 || Config[C_Z_SPEED]*Config[C_Z_PPM]*60/3200>3000) return "z_rpm";
  if(Config[C_X_SPEED]*Config[C_X_PPM]*60/3200<1 || Config[C_X_SPEED]*Config[C_X_PPM]*60/3200>3000) return "x_rpm";
  if(op!=OP_RETURN && OutsideReferenceValid && !OutsidePose) return "outside_pose";
  if(!strcmp(mode,"fixed")) return NULL;
  if(Config[C_X_PRE]<Config[C_X_MIN] || Config[C_X_PRE]>Config[C_X_MAX]) return "x_range";
  if(Config[C_Z_OBSERVE]<Config[C_Z_MIN] || Config[C_Z_OBSERVE]>Config[C_Z_MAX]) return "z_observe";
  if(Config[C_LEAD]>Config[C_X_MAX]-Config[C_X_MIN]) return "lead_mm";
  if(!solve(1,1,Config[C_X_PRE],test)) return "matrix_rank";
  return NULL;
}
static const char *configuration_missing(const char *mode)
{ return configuration_missing_for(mode,Operation,Scene); }
bool GrabTask_ConfigReady(const char *mode)
{ return configuration_missing_for(mode,!strcmp(mode,"align")?OP_ALIGN:OP_STORE,SCENE_RAW)==NULL; }
bool GrabTask_IsBusy(void)
{ return Status.state!=GRAB_IDLE && Status.state!=GRAB_ERROR && Status.state!=GRAB_COMPLETE; }
void GrabTask_Init(void)
{
  unsigned i;
  BootActive=BootStopping=false;
  RecoveryUsed=RecoveryCount=0; RecoveryWaiting=RecoveryRestartPending=false;
  LossActive=false; LossTick=LossLastMs=LossMaxMs=GraceCount=GraceSequence=0;
  for(i=0;i<C_COUNT;i++) Config[i]=NAN;
  /* 2026-09-30: trial tuning from blue dx=-95..-57; the previous profile
   * commanded at most 10 mm/s. X remains limited to 5 mm/s. */
  Config[C_LAMBDA]=.6f; Config[C_BODY_SPEED]=20; Config[C_X_SPEED]=5;
  /* Z: user-requested 250 RPM at 3200 command pulses/rev and 480 pulses/mm.
   * Keep the public z_speed setting in mm/s; home O_Vel is driver-owned. */
  Config[C_Z_SPEED]=250.0f*3200.0f/(60.0f*480.0f);
  Config[C_STABLE_MS]=200; Config[C_FRAMES]=3; Config[C_RETRIES]=2; Config[C_X_WEIGHT]=2.333333f;
  Config[C_POS_TOL]=1; Config[C_STOP_SPEED]=2; /* mm / mm/s, initial relaxed acceptance thresholds. */
  Config[C_FEEDBACK_MS]=2000; Config[C_CLOSE_MS]=500;
  Config[C_AGE_MS]=1000; Config[C_ACCEL]=20; Config[C_TRAVEL]=0; Config[C_LEAD]=2;
  Config[C_DX_TOL]=5; Config[C_DY_TOL]=5;
  Config[C_LOSS_GRACE_MS]=250;
  Config[C_Z_CAR_LIFT]=0; Config[C_BASE_CAR_OFFSET]=-180;
  visual_defaults_restore();
  memset(&Status,0,sizeof Status); memset(Axes,0,sizeof Axes);
  Status.mode="fixed"; Status.reason="idle"; Status.missing="z_ppm";
  Status.reference_cause="boot_pending";
  Operation=OP_STORE; Scene=SCENE_RAW; Status.operation="store"; Status.scene="raw";
  Status.result="none"; Status.slot=0; OutsideReferenceValid=false; OutsidePose=true;
  ToCar=IndexSubmitted=InventoryTouched=false;
  ActiveColor=GRAB_PICK_COLOR;
  Status.x_mm=Status.x_target_mm=Status.z_mm=Status.z_target_mm=NAN;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  HaveCapture=false; LastReport=HAL_GetTick(); ReportPending=false;
}

static void stable_reset(void)
{ VisualStable=false; CaptureCount=0; StableTick=HAL_GetTick(); }
/* Freeze the anchor until three new valid observations restore the episode.
 * Invalid packets or one intermittent good packet must not restart this clock. */
static void loss_finish(uint32_t now)
{
  if(LossActive) {
    LossLastMs=now-LossTick;
    if(LossLastMs>LossMaxMs) LossMaxMs=LossLastMs;
    LossActive=false;
  }
  GraceCount=0;
}
static void stop_begin(void)
{
  unsigned i;
  stable_reset();
  StopTick=HAL_GetTick(); StopArmSent=StopArmTx=StopChassisSent=false;
  Status.stop_requested=true; Status.stop_confirmed=false; WheelQuiet=false;
  StopEvidenceStarted=false;
  for(i=0;i<3;i++) { Axes[i].quiet=false; Axes[i].commanded=false; }
  memset(PreviousVelocity,0,sizeof PreviousVelocity);
  XIntegral=Axes[0].target;
  Status.forward_mm_s=Status.left_mm_s=0;
  /* Let an in-flight feedback request drain without immediately replacing it;
   * normal stopping must reach a clean shared-bus boundary. */
  Mecanum_Feedback_Enable(false);
  if(Status.state==GRAB_STOPPING) Turntable_Stop();
}
static void fault(const char *reason)
{
  if(Status.state==GRAB_STOPPING || Status.state==GRAB_ERROR) return;
  loss_finish(HAL_GetTick());
  RecoveryWaiting=RecoveryRestartPending=false; RecoveryCount=0;
  invalidate_inventory(); Status.result="failed";
  Status.reason=reason; FinishError=true; Camera_RequestStop(); HaveCapture=FreshVision=false;
  enter(GRAB_STOPPING); stop_begin();
}
void GrabTask_Stop(void)
{
  if(BootActive) { boot_stop("home_cancelled",false); return; }
  if(Status.state==GRAB_IDLE || Status.state==GRAB_COMPLETE) return;
  loss_finish(HAL_GetTick());
  RecoveryWaiting=RecoveryRestartPending=false; RecoveryCount=0;
  /* Repeated console stops must not reset fresh standstill evidence or resend
   * stops ahead of the feedback needed to finish the existing cancellation. */
  if(Status.state==GRAB_STOPPING)
  {
    if(FinishError) { Status.reason="cancelled"; FinishError=false; ReportPending=true; }
    return;
  }
  invalidate_inventory(); Status.result="cancelled";
  Status.reason="cancelled"; FinishError=false;
  Camera_RequestStop(); HaveCapture=FreshVision=false;
  enter(GRAB_STOPPING); stop_begin();
}
void GrabTask_ReferenceInvalidate(MechanicalArm_AxisTypeDef axis)
{
  unsigned i;
  if(GrabTask_IsBusy()) fault("reference_changed");
  OutsideReferenceValid=false; OutsidePose=true;
  Status.reference_cause=axis==MECHANICAL_ARM_AXIS_X?"x_reference_changed":
    axis==MECHANICAL_ARM_AXIS_Z?"z_reference_changed":
    axis==MECHANICAL_ARM_AXIS_BASE?"base_reference_changed":"all_reference_changed";
  if(axis==MECHANICAL_ARM_AXIS_X || axis==MECHANICAL_ARM_AXIS_ALL)
    Config[C_X_MIN]=Config[C_X_MAX]=Config[C_X_PRE]=Config[C_X_CAR]=NAN;
  if(axis==MECHANICAL_ARM_AXIS_Z || axis==MECHANICAL_ARM_AXIS_ALL)
  {
    Config[C_Z_MIN]=Config[C_Z_MAX]=Config[C_Z_OBSERVE]=Config[C_Z_GRAB]=Config[C_Z_LIFT]=Config[C_Z_PLACE]=NAN;
    Config[C_Z_PROC_GRAB]=Config[C_Z_PROC_LIFT]=Config[C_Z_PROC_PLACE]=Config[C_Z_TEMP_PLACE]=Config[C_Z_STACK_PLACE]=NAN;
    Config[C_Z_CAR_LIFT]=NAN;
  }
  if(axis==MECHANICAL_ARM_AXIS_BASE)
  {
    Config[C_REF_U]=Config[C_REF_V]=NAN;
    for(i=C_J00;i<=C_J12;i++) Config[i]=NAN;
  }
  Status.missing=configuration_missing(Status.mode);
  if(!Status.missing) Status.missing="none";
}
static bool start_operation(const char *mode,Camera_ColorTypeDef color,GrabOperation_t op,GrabScene_t scene,uint8_t slot)
{
  HWT101_Angle_t angle; Mecanum_Status_t bus; const char *missing;
  if(GrabTask_IsBusy()) return false;
  /* A rejected request must still identify the requested mode in its diagnostic. */
  if(!valid_mode(mode)) { Status.reason="invalid_mode"; Status.missing="mode"; return false; }
  if(color<CAMERA_COLOR_RED || color>CAMERA_COLOR_LIGHT_BLUE)
  { Status.reason="invalid_color"; Status.missing="color"; return false; }
  Status.mode=!strcmp(mode,"fixed")?"fixed":(!strcmp(mode,"align")?"align":"pick");
  ActiveColor=color; Operation=op; Scene=scene;
  Status.operation=OperationNames[op]; Status.scene=op==OP_ALIGN || op==OP_RETURN?"none":SceneNames[scene];
  Status.result="none"; Status.slot=slot;
  if(op==OP_RETURN && !OutsideReferenceValid)
  { Status.reason="outside_reference_unavailable"; Status.missing="outside_reference"; return false; }
  missing=configuration_missing(mode);
  if(missing) { Status.reason="config_missing"; Status.missing=missing; return false; }
  if(op==OP_STORE || op==OP_TAKE)
  {
    Turntable_Inventory_t inventory;
    if(!slot && op==OP_STORE) slot=Turntable_FindEmpty();
    Status.slot=slot;
    if(!slot || slot>3)
    { Status.reason=op==OP_STORE?"no_confirmed_empty_slot":"invalid_slot"; Status.missing="slot"; return false; }
    if(!Turntable_ReferenceValid() || !Turntable_SlotConfigured(slot))
    { Status.reason="turntable_reference_missing"; Status.missing="turntable_reference_or_angle"; return false; }
    inventory=Turntable_InventoryGet(slot);
    if(inventory.state==TURNTABLE_UNKNOWN)
    { Status.reason="inventory_unconfirmed"; Status.missing="slot_inventory"; return false; }
    if((op==OP_STORE && inventory.state!=TURNTABLE_EMPTY) ||
       (op==OP_TAKE && inventory.state!=TURNTABLE_OCCUPIED))
    { Status.reason=op==OP_STORE?"slot_occupied":"slot_empty"; Status.missing="slot_inventory"; return false; }
    if(op==OP_TAKE) ActiveColor=(Camera_ColorTypeDef)inventory.color;
  }
  if(ArmVision_IsBusy() || MaterialVision_IsBusy() || ChassisRoute_IsBusy() ||
     ChassisMotion_IsBusy() || MechanicalArm_IsBusy() || Mecanum_IsBusy() || HWT101_Cal_IsBusy() || Turntable_IsBusy())
  { Status.reason="busy"; return false; }
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stop_pending || bus.ack_profile!=MECANUM_ACK_RECEIVE)
  { Status.reason="bus_unavailable"; return false; }
  if(strcmp(mode,"fixed") && (!HWT101_Cal_ControlAngleGet(&angle) || !isfinite(angle.yaw) ||
     (uint32_t)(HAL_GetTick()-angle.last_update_ms)>HWT101_DATA_FRESH_MS))
  { Status.reason="imu_invalid"; return false; }
  Heading=!strcmp(mode,"fixed") ? 0 : angle.yaw*CHASSIS_HEADING_TEST_YAW_SIGN;
  Status.mode=!strcmp(mode,"fixed")?"fixed":(!strcmp(mode,"align")?"align":"pick");
  Status.reason="running"; Status.missing="none"; Status.result="pending";
  InventoryTouched=IndexSubmitted=false; ToCar=op!=OP_RETURN;
  memset(Axes,0,sizeof Axes); memset(WheelLastPositionTick,0,sizeof WheelLastPositionTick);
  Status.x_mm=Status.x_target_mm=Status.z_mm=Status.z_target_mm=NAN;
  RecoveryUsed=RecoveryCount=0; RecoveryWaiting=RecoveryRestartPending=false;
  LossActive=false; LossTick=LossLastMs=LossMaxMs=GraceCount=GraceSequence=0;
  PrepareStep=PollIndex=Retries=0; HaveCapture=FreshVision=WheelBaseline=FeedbackSelected=false;
  PendingAction=MECHANICAL_ARM_ACTION_NONE; StartTick=HAL_GetTick();
  LastControl=LastPoll=StartTick; LastSequence=LastInvalidCount=0; WheelGroupTick=0;
  enter(GRAB_PREPARE); stop_begin(); stable_reset();
  return true;
}
static bool start_with_color(const char *mode,Camera_ColorTypeDef color)
{ return start_operation(mode,color,!strcmp(mode,"align")?OP_ALIGN:OP_STORE,SCENE_RAW,0); }
bool GrabTask_Start(const char *mode)
{ return start_with_color(mode,default_color(mode)); }

static bool result_accept(MechanicalArm_ResultTypeDef result, MechanicalArm_ActionTypeDef action,
                          MechanicalArm_AxisTypeDef axis)
{
  if(result==MECHANICAL_ARM_RESULT_BUSY) return false;
  if(result!=MECHANICAL_ARM_RESULT_NONE && result!=MECHANICAL_ARM_RESULT_OK)
  { fault(result==MECHANICAL_ARM_RESULT_ACK_TIMEOUT?"motor_ack_timeout":"motor_request"); return false; }
  PendingAction=action; PendingAxis=axis; return true;
}
static bool axis_move(unsigned i,float target)
{
  MechanicalArm_AxisTypeDef axis=i==0?MECHANICAL_ARM_AXIS_X:MECHANICAL_ARM_AXIS_Z;
  float ppm=Config[i==0?C_X_PPM:C_Z_PPM], speed=Config[i==0?C_X_SPEED:C_Z_SPEED];
  float minimum=Config[i==0?C_X_MIN:C_Z_MIN],maximum=Config[i==0?C_X_MAX:C_Z_MAX];
  uint16_t rpm=(uint16_t)floorf(speed*ppm*60/3200);
  int32_t pulses;
  if(target<minimum || target>maximum || !isfinite(target)) { fault("axis_limit"); return false; }
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy()) return false;
  pulses=(int32_t)lroundf(target*ppm);
  if(!result_accept(MechanicalArm_PositionEx(axis,pulses,rpm,8,
    MECHANICAL_ARM_POSITION_ABSOLUTE),MECHANICAL_ARM_ACTION_POSITION,axis)) return false;
  target=(float)pulses/ppm;
  Axes[i].target=target; Axes[i].command_tick=HAL_GetTick(); Axes[i].commanded=true;
  Axes[i].quiet=false;
  if(i==0) Status.x_target_mm=target; else Status.z_target_mm=target;
  return true;
}
/* Use the measured outside multi-turn angle as the anchor for every owned turn.
 * A near-home origin is not necessarily numerical zero. Keep calibration values:
 * the camera model is unavailable only while its observation pose is suspended. */
static bool base_move(float target)
{
  MechanicalArm_ConfigTypeDef config;
  int32_t pulses;
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy()) return false;
  if(!OutsideReferenceValid || !Axes[2].position_valid) { fault("outside_reference_unavailable"); return false; }
  if(!MechanicalArm_ConfigGet(MECHANICAL_ARM_AXIS_BASE,&config)) { fault("base_config"); return false; }
  pulses=(int32_t)lroundf((target-Axes[2].mm)*3200.0f/360.0f);
  if(!result_accept(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_BASE,pulses,
    config.SpeedRpm,config.Acceleration,MECHANICAL_ARM_POSITION_RELATIVE_CURRENT),
    MECHANICAL_ARM_ACTION_POSITION,MECHANICAL_ARM_AXIS_BASE)) return false;
  Axes[2].target=Axes[2].mm+(float)pulses*360.0f/3200.0f; Axes[2].command_tick=HAL_GetTick();
  Axes[2].commanded=true; Axes[2].quiet=false; OutsidePose=false;
  Status.reference_cause="transfer_off_pose";
  return true;
}
static bool axis_fresh(unsigned i,uint32_t now)
{ return Axes[i].position_valid && Axes[i].state_valid &&
    (uint32_t)(now-Axes[i].position_tick)<=Config[C_FEEDBACK_MS] &&
    (uint32_t)(now-Axes[i].state_tick)<=Config[C_FEEDBACK_MS]; }
static bool axis_quiet(unsigned i,uint32_t now,uint32_t after)
{
  return axis_fresh(i,now) && Axes[i].quiet && (int32_t)(Axes[i].position_tick-after)>0 &&
    (int32_t)(Axes[i].state_tick-after)>0 &&
    (uint32_t)(now-Axes[i].quiet_since)>=Config[C_STABLE_MS];
}
static bool arrived(unsigned i,uint32_t now)
{
  return Axes[i].commanded && axis_quiet(i,now,Axes[i].command_tick) &&
    (Axes[i].flags&3U)==3U && fabsf(Axes[i].mm-Axes[i].target)<=(i==2?1.0f:Config[C_POS_TOL]);
}
static void poll_axes(uint32_t now)
{
  unsigned i; MechanicalArm_AxisTypeDef axis; MechanicalArm_ActionTypeDef action;
  if(((Status.state==GRAB_PREPARE && PrepareStep==0) || Status.state==GRAB_SETTLE ||
      Status.state==GRAB_VISION_PAUSE || Status.state==GRAB_STOPPING) && !StopEvidenceStarted) return;
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy() || now-LastPoll<25) return;
  i=(PollIndex/2)%3; axis=i==2?MECHANICAL_ARM_AXIS_BASE:(i==0?MECHANICAL_ARM_AXIS_X:MECHANICAL_ARM_AXIS_Z);
  action=(PollIndex%2)==0?MECHANICAL_ARM_ACTION_POSITION_READ:MECHANICAL_ARM_ACTION_STATE;
  if(result_accept(action==MECHANICAL_ARM_ACTION_POSITION_READ?MechanicalArm_PositionRead(axis):
    MechanicalArm_StateRead(axis),action,axis)) { PollIndex=(PollIndex+1)%6; LastPoll=now; }
}
uint8_t GrabTask_MotorEventHandle(const MechanicalArm_EventTypeDef *event)
{
  unsigned i; GrabAxis_t *axis; uint32_t now=HAL_GetTick();
  if(!event) return 0;
  if(BootActive) return boot_event(event);
  if(PendingAction==MECHANICAL_ARM_ACTION_NONE || event->Action!=PendingAction || event->Axis!=PendingAxis)
  {
    /* Observe reference changes from console AND legacy tasks without consuming
     * their events. Lost ACK after TX cannot preserve a trusted old coordinate. */
    if(event->Transmitted)
    {
      if((event->Action==MECHANICAL_ARM_ACTION_ZERO || event->Action==MECHANICAL_ARM_ACTION_HOME) &&
         event->Axis!=MECHANICAL_ARM_AXIS_BASE) GrabTask_ReferenceInvalidate(event->Axis);
      if(((event->Action==MECHANICAL_ARM_ACTION_HOME || event->Action==MECHANICAL_ARM_ACTION_ZERO) &&
          (event->Axis==MECHANICAL_ARM_AXIS_BASE || event->Axis==MECHANICAL_ARM_AXIS_ALL)) ||
         (event->Action==MECHANICAL_ARM_ACTION_POSITION && event->Axis==MECHANICAL_ARM_AXIS_BASE))
        GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
      if(event->Action==MECHANICAL_ARM_ACTION_HOME)
        Status.reference_cause=event->Axis==MECHANICAL_ARM_AXIS_ALL?"manual_home_all":
          event->Axis==MECHANICAL_ARM_AXIS_X?"manual_home_x":
          event->Axis==MECHANICAL_ARM_AXIS_Z?"manual_home_z":"manual_home_base";
      else if(event->Action==MECHANICAL_ARM_ACTION_ZERO)
        Status.reference_cause=event->Axis==MECHANICAL_ARM_AXIS_ALL?"manual_zero_all":
          event->Axis==MECHANICAL_ARM_AXIS_X?"manual_zero_x":
          event->Axis==MECHANICAL_ARM_AXIS_Z?"manual_zero_z":"manual_zero_base";
      else if(event->Action==MECHANICAL_ARM_ACTION_POSITION && event->Axis==MECHANICAL_ARM_AXIS_BASE)
        Status.reference_cause="manual_base_move";
    }
    return 0;
  }
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  if(event->Result!=MECHANICAL_ARM_RESULT_OK)
  { if(Status.state!=GRAB_STOPPING) fault("motor_feedback"); return 1; }
  if(event->Action==MECHANICAL_ARM_ACTION_STOP) { StopArmTx=event->Transmitted!=0; return 1; }
  i=event->Axis==MECHANICAL_ARM_AXIS_BASE?2:(event->Axis==MECHANICAL_ARM_AXIS_X?0:1); axis=&Axes[i];
  if(event->Action==MECHANICAL_ARM_ACTION_POSITION_READ)
  {
    float ppm=i==2?3200.0f/360:Config[i==0?C_X_PPM:C_Z_PPM]; int64_t raw=event->CurrentPositionRaw;
    if((i==2?MECHANICAL_ARM_BASE_POSITIVE_DIR:(i==0?MECHANICAL_ARM_X_POSITIVE_DIR:MECHANICAL_ARM_Z_POSITIVE_DIR))!=0) raw=-raw;
    if(axis->position_valid && now!=axis->position_tick)
    {
      /* Fixed mode does not move X: exact raw-position stability needs no invented X scale. */
      axis->speed=isfinite(ppm)?fabsf((float)(raw-axis->raw)*3200/65536/ppm)*1000/(now-axis->position_tick):
        (raw==axis->raw?0:INFINITY);
      if(axis->speed<=(i==2?1.0f:Config[C_STOP_SPEED]))
      { if(!axis->quiet) axis->quiet_since=now; axis->quiet=true; }
      else axis->quiet=false;
    }
    axis->raw=raw; axis->position_tick=now; axis->position_valid=true;
    axis->mm=isfinite(ppm)?(float)raw*3200/65536/ppm:NAN;
    if(i==0) Status.x_mm=axis->mm; else if(i==1) Status.z_mm=axis->mm;
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_STATE)
  {
    axis->flags=event->StateFlags[event->Axis]; axis->state_tick=now; axis->state_valid=true;
    if((axis->flags&0x0CU)!=0 || !(axis->flags&1U)) fault("axis_disabled_or_stall");
  }
  return 1;
}

/* Every wheel must have fresh speed, position and enabled/non-stall state.
 * Standstill requires new position groups AFTER stop TX as well as low speed. */
static bool wheels(uint32_t now)
{
  unsigned i; bool quiet=true, group_new=true; uint32_t group=now;
  const float mm_per_unit=CHASSIS_WHEEL_DIAMETER_MM*3.14159265358979323846f/
    (65536.0f*CHASSIS_MOTOR_TO_WHEEL_RATIO);
  for(i=0;i<4;i++)
  {
    Mecanum_Feedback_t f;
    if(!Mecanum_FeedbackGet(i,&f) || !f.speed_valid || !f.position_valid || !f.state_valid ||
      now-f.speed_ms>Config[C_FEEDBACK_MS] || now-f.position_ms>Config[C_FEEDBACK_MS] ||
      now-f.state_ms>Config[C_FEEDBACK_MS]) { WheelQuiet=false; return false; }
    if((f.state_flags&0x0CU) || !(f.state_flags&1U)) { WheelQuiet=false; fault("wheel_disabled_or_stall"); return false; }
    if(!WheelBaseline) WheelOrigin[i]=f.position_raw;
    /* A past travel violation must not block fresh standstill evidence. Keep all
     * speed/position/state checks below active while completing the stop. */
    if(Status.state!=GRAB_STOPPING && !fixed_mode() && Config[C_TRAVEL]>0 &&
       fabsf((float)(f.position_raw-WheelOrigin[i])*mm_per_unit)>Config[C_TRAVEL])
    { fault("travel_limit"); return false; }
    if(fabsf((float)f.speed_rpm)*65536*mm_per_unit/60>Config[C_STOP_SPEED] ||
       (int32_t)(f.speed_ms-StopTick)<=0 || (int32_t)(f.position_ms-StopTick)<=0 ||
       (int32_t)(f.state_ms-StopTick)<=0) quiet=false;
    if(f.position_ms==WheelLastPositionTick[i]) group_new=false;
    if(WheelLastPositionTick[i] && f.position_ms!=WheelLastPositionTick[i] &&
       fabsf((float)(f.position_raw-WheelLastPosition[i])*mm_per_unit)*1000/
       (uint32_t)(f.position_ms-WheelLastPositionTick[i])>Config[C_STOP_SPEED]) quiet=false;
    if((int32_t)(f.position_ms-group)<0) group=f.position_ms;
  }
  if(group_new && group!=WheelGroupTick)
  {
    for(i=0;i<4;i++)
    { Mecanum_Feedback_t f; (void)Mecanum_FeedbackGet(i,&f); WheelLastPosition[i]=f.position_raw; WheelLastPositionTick[i]=f.position_ms; }
    WheelGroupTick=group;
    if(quiet) { if(!WheelQuiet) WheelQuietTick=now; WheelQuiet=true; } else WheelQuiet=false;
  }
  if(!quiet) WheelQuiet=false;
  WheelBaseline=true; return true;
}
static void stop_process(uint32_t now,bool emergency)
{
  Mecanum_Status_t bus;
  /* Normal stops drain the arm request as well: a priority chassis stop would
   * otherwise cancel its ACK/read and quarantine the shared UART. */
  if(!StopChassisSent && (emergency || (PendingAction==MECHANICAL_ARM_ACTION_NONE &&
      !MechanicalArm_IsBusy() && Mecanum_CanStopCleanly())))
  { if(Mecanum_Test_Stop()) StopChassisSent=true; }
  Mecanum_StatusGet(&bus);
  if(!StopArmSent && (emergency || (StopChassisSent && bus.stage==MECANUM_STAGE_STOPPED &&
    bus.tx_complete && PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy())))
  {
    /* Emergency ALL cancels the outstanding X/Z request but never disables a motor.
     * Stopping Base leaves its established work angle unchanged. */
    MechanicalArm_AxisTypeDef axis=emergency?MECHANICAL_ARM_AXIS_ALL:MECHANICAL_ARM_AXIS_X;
    if(result_accept(MechanicalArm_Stop(axis),MECHANICAL_ARM_ACTION_STOP,axis)) StopArmSent=true;
  }
  Mecanum_StatusGet(&bus);
  if(StopChassisSent && bus.stage==MECANUM_STAGE_STOPPED && bus.tx_complete && StopArmTx && !StopEvidenceStarted)
  {
    /* Only replies requested AFTER all stop transmissions can establish rest. */
    StopEvidenceStarted=true; StopTick=now; WheelQuiet=false;
    Axes[0].quiet=Axes[1].quiet=Axes[2].quiet=false;
    Mecanum_Feedback_Enable(true);
  }
  if(StopEvidenceStarted &&
     WheelQuiet && now-WheelQuietTick>=Config[C_STABLE_MS] &&
     axis_quiet(0,now,StopTick) && axis_quiet(1,now,StopTick) && axis_quiet(2,now,StopTick)) Status.stop_confirmed=true;
}

/* A legal no-coordinate report can pause alignment; link/feedback faults cannot. */
static bool vision_request_valid(const Camera_SnapshotTypeDef *s)
{
  if(!s->UsbConfigured) { fault("vision_usb_off"); return false; }
  if(!s->RequestActive || s->RequestFunction!=0xB2 || s->RequestTarget!=target_color())
  { fault("vision_request_lost"); return false; }
  return true;
}
static void vision_pause_begin(void)
{
  GraceCount=0;
  if(RecoveryUsed>=GRAB_RECOVERY_LIMIT) { fault("vision_recover_limit"); return; }
  RecoveryWaiting=false; RecoveryCount=0; HaveCapture=FreshVision=false;
  /* Alignment can reacquire through the existing B2 command, without changing Pi code. */
  RecoveryRestartPending=true;
  Status.reason="vision_pause_stopping"; enter(GRAB_VISION_PAUSE); stop_begin();
}
static bool vision_near_window(int dx,int dy)
{ return abs(dx)<=2*Config[C_DX_TOL] && abs(dy)<=2*Config[C_DY_TOL]; }
static void vision_loss_begin(uint32_t now)
{
  if(!LossActive) { LossActive=true; LossTick=Vision.Tick; }
  stable_reset(); FreshVision=false;
  /* Alignment in both align and pick can decelerate through a brief gap.
   * Settle and near-window motion still stop; old data never confirms arrival. */
  if(Status.state!=GRAB_ALIGN ||
     Config[C_LOSS_GRACE_MS]<=0 || now-LossTick>=Config[C_LOSS_GRACE_MS] ||
     vision_near_window(Status.dx,Status.dy)) { vision_pause_begin(); return; }
  GraceCount=0; GraceSequence=LastSequence;
  Status.reason="vision_gap_decelerating"; enter(GRAB_VISION_GRACE);
}
static void vision_grace_process(uint32_t now)
{
  Camera_SnapshotTypeDef s; Mecanum_Status_t bus; bool valid; int dx,dy;
  Camera_SnapshotGet(&s);
  if(!vision_request_valid(&s)) return;
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stage==MECANUM_STAGE_FAULT) { fault("chassis_tx"); return; }
  if(now-LossTick>=Config[C_LOSS_GRACE_MS] || now-LossTick>Config[C_AGE_MS])
  { vision_pause_begin(); return; }
  if(s.InvalidCount!=LastInvalidCount) { GraceCount=0; LastInvalidCount=s.InvalidCount; }
  valid=s.HasValidData && s.TargetValid && s.Data.Function==0xB2 &&
    s.Data.Target==target_color() && (int32_t)(s.Data.Tick-LossTick)>0 &&
    (int32_t)(s.Data.Tick-RequestTick)>=0 && now-s.Data.Tick<=Config[C_AGE_MS];
  if(!valid) GraceCount=0;
  else if(s.Data.Sequence!=GraceSequence)
  {
    GraceSequence=s.Data.Sequence;
    dx=(int)s.Data.CX-(int)Config[C_REF_U]; dy=(int)Config[C_REF_V]-(int)s.Data.CY;
    /* A new near-window or reversed error makes the previous direction unsafe
     * to reuse even before three observations have confirmed visual recovery. */
    if(vision_near_window(dx,dy) ||
       (dx*(int)Status.dx<0 && abs(dx)>Config[C_DX_TOL] && abs(Status.dx)>Config[C_DX_TOL]) ||
       (dy*(int)Status.dy<0 && abs(dy)>Config[C_DY_TOL] && abs(Status.dy)>Config[C_DY_TOL]))
    { vision_pause_begin(); return; }
    if(GraceCount<GRAB_RECOVERY_OBSERVATIONS) GraceCount++;
    if(GraceCount>=GRAB_RECOVERY_OBSERVATIONS)
    {
      Vision=s.Data; LastSequence=s.Data.Sequence;
      Status.dx=(int16_t)dx; Status.dy=(int16_t)dy;
      HaveCapture=true; FreshVision=false; loss_finish(now); stable_reset(); LastControl=now;
      Status.reason="vision_gap_recovered"; enter(GRAB_ALIGN);
      return; /* Only the next new coordinate may contribute to alignment. */
    }
  }
  align_control(now); /* Decelerate previous commands, never solve stale errors. */
}
static void vision_pause_process(uint32_t now)
{
  Camera_SnapshotTypeDef s; Mecanum_Status_t bus; uint32_t elapsed;
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stage==MECANUM_STAGE_FAULT) { fault("chassis_tx"); return; }
  Camera_SnapshotGet(&s);
  if(!vision_request_valid(&s)) return;
  Status.stop_confirmed=false; /* Recheck current feedback; do not reuse a latched confirmation. */
  stop_process(now,false);
  if(Status.state!=GRAB_VISION_PAUSE) return;
  if(!RecoveryWaiting)
  {
    if(!Status.stop_confirmed) return;
    RecoveryWaiting=true; RecoveryTick=now; RecoveryCount=0;
    RecoverySequence=s.Data.Sequence; LastInvalidCount=s.InvalidCount;
    Status.reason="vision_recover_wait"; ReportPending=true;
    return; /* No packet seen before standstill can count. */
  }
  elapsed=now-RecoveryTick;
  if(elapsed>GRAB_RECOVERY_WINDOW_MS) { fault("vision_recover_timeout"); return; }
  if(RecoveryRestartPending)
  {
    HAL_StatusTypeDef result;
    if(!Status.stop_confirmed) return;
    result=Camera_MaterialStart(target_color());
    if(result==HAL_BUSY) return; /* Retries share the existing bounded recovery window. */
    if(result!=HAL_OK) { fault("camera_tx"); return; }
    RecoveryRestartPending=false; RequestTick=now;
    RecoverySequence=0; LastInvalidCount=0; RecoveryCount=0;
    Status.reason="vision_reacquire_wait"; ReportPending=true;
    return; /* Camera_MaterialStart clears the old snapshot; never count the pre-send copy. */
  }
  if(s.InvalidCount!=LastInvalidCount)
  { RecoveryCount=0; LastInvalidCount=s.InvalidCount; }
  if(!s.HasValidData || !s.TargetValid || s.Data.Function!=0xB2 || s.Data.Target!=target_color() ||
     (uint32_t)(now-s.Data.Tick)>Config[C_AGE_MS])
  { RecoveryCount=0; return; }
  if(s.Data.Sequence!=RecoverySequence)
  {
    RecoverySequence=s.Data.Sequence;
    if((int32_t)(s.Data.Tick-RecoveryTick)<0 || (int32_t)(s.Data.Tick-RequestTick)<0) return;
    if(RecoveryCount<GRAB_RECOVERY_OBSERVATIONS) RecoveryCount++;
  }
  /* Freshness and rest can become true independently of a new coordinate packet. */
  if(RecoveryCount<GRAB_RECOVERY_OBSERVATIONS || !Status.stop_confirmed) return;
  RecoveryUsed++; RecoveryWaiting=false; RecoveryCount=0; loss_finish(now);
  LastSequence=s.Data.Sequence; Vision=s.Data; HaveCapture=true; FreshVision=false;
  Status.dx=(int16_t)((int)Vision.CX-(int)Config[C_REF_U]);
  Status.dy=(int16_t)((int)Config[C_REF_V]-(int)Vision.CY);
  Axes[0].target=XIntegral=Axes[0].mm; Axes[0].commanded=false;
  Status.x_target_mm=Axes[0].mm; memset(PreviousVelocity,0,sizeof PreviousVelocity);
  LastControl=now; stable_reset(); Status.stop_requested=Status.stop_confirmed=false;
  Status.reason="vision_recovered"; enter(GRAB_ALIGN);
}

static bool vision_read(uint32_t now)
{
  Camera_SnapshotTypeDef s; Camera_SnapshotGet(&s); FreshVision=false;
  if(!vision_request_valid(&s)) return false;
  /* Preserve invalid events even when the latest packet in a USB batch is valid. */
  if(s.InvalidCount!=LastInvalidCount)
  {
    LastInvalidCount=s.InvalidCount; stable_reset();
    if(HaveCapture) { vision_loss_begin(now); return false; }
  }
  if(!s.HasValidData || !s.TargetValid || !s.RequestActive ||
     s.RequestFunction!=0xB2 || s.RequestTarget!=target_color() ||
     s.Data.Function!=0xB2 || s.Data.Target!=target_color())
  { stable_reset(); if(HaveCapture) { if(s.HasFrame && !s.TargetValid) vision_loss_begin(now); else fault("vision_lost"); } return false; }
  if((int32_t)(s.Data.Tick-RequestTick)<0) return false;
  if((uint32_t)(now-s.Data.Tick)>Config[C_AGE_MS])
  { stable_reset(); if(HaveCapture) fault("vision_stale"); return false; }
  if(HaveCapture && s.Data.Sequence==LastSequence)
  {
    /* Silence is also a gap: after one control period without a new sample,
     * enter tolerance using the old RX timestamp, not the polling time. */
    if(Config[C_LOSS_GRACE_MS]>0 && now-Vision.Tick>=GRAB_CONTROL_PERIOD_MS)
    { vision_loss_begin(now); return false; }
    return true;
  }
  if(!HaveCapture) stable_reset();
  LastSequence=s.Data.Sequence; Vision=s.Data;
  HaveCapture=FreshVision=true;
  Status.dx=(int16_t)((int)Vision.CX-(int)Config[C_REF_U]);
  Status.dy=(int16_t)((int)Config[C_REF_V]-(int)Vision.CY);
  return true;
}
static bool in_window(void)
{ return abs(Status.dx)<=Config[C_DX_TOL] && abs(Status.dy)<=Config[C_DY_TOL]; }
static void visual_stability(uint32_t now)
{
  if(!FreshVision) return;
  if(!in_window() || !Axes[0].quiet || Axes[0].speed>Config[C_STOP_SPEED] || !WheelQuiet)
  { stable_reset(); return; }
  if(CaptureCount==0) StableTick=now;
  CaptureCount++;
  VisualStable=CaptureCount>=Config[C_FRAMES] && now-StableTick>=Config[C_STABLE_MS];
}
static void align_control(uint32_t now)
{
  float v[3], dt, delta, gain=1, xnext, error, yaw; unsigned i; HWT101_Angle_t angle;
  Mecanum_Status_t bus;
  bool grace=Status.state==GRAB_VISION_GRACE;
  if(now-LastControl<GRAB_CONTROL_PERIOD_MS || (!FreshVision && !grace)) return;
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stage==MECANUM_STAGE_FAULT) { fault("chassis_tx"); return; }
  /* Leave at least 50ms after the complete wheel batch for arm and wheel reads.
   * Skip this busy interval's integration but retain the existing sub-pulse remainder. */
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy() ||
    bus.stage==MECANUM_STAGE_WHEELS || bus.stage==MECANUM_STAGE_SYNC ||
    (bus.stage==MECANUM_STAGE_SENT && now-bus.sent_ms<50))
  { LastControl=now; return; }
  dt=fminf((now-LastControl)/1000.0f,.2f); LastControl=now;
  if(!HWT101_Cal_ControlAngleGet(&angle) || !isfinite(angle.yaw) || now-angle.last_update_ms>HWT101_DATA_FRESH_MS)
  { fault("imu_invalid"); return; }
  if(grace)
  {
    float speed=sqrtf(PreviousVelocity[0]*PreviousVelocity[0]+
      PreviousVelocity[1]*PreviousVelocity[1]+PreviousVelocity[2]*PreviousVelocity[2]);
    float scale=speed>0?fmaxf(0,1-Config[C_ACCEL]*dt/speed):0;
    for(i=0;i<3;i++) v[i]=PreviousVelocity[i]*scale;
  }
  else
  {
    if(!solve((float)Status.dx,(float)Status.dy,Axes[0].mm,v)) { fault("matrix_rank"); return; }
    if(in_window()) v[0]=v[1]=v[2]=0;
  }
  delta=sqrtf((v[0]-PreviousVelocity[0])*(v[0]-PreviousVelocity[0])+
    (v[1]-PreviousVelocity[1])*(v[1]-PreviousVelocity[1])+
    (v[2]-PreviousVelocity[2])*(v[2]-PreviousVelocity[2]));
  if(delta>Config[C_ACCEL]*dt) gain=Config[C_ACCEL]*dt/delta;
  for(i=0;i<3;i++) v[i]=PreviousVelocity[i]+gain*(v[i]-PreviousVelocity[i]);
  xnext=clamp(XIntegral+v[2]*dt,Config[C_X_MIN],Config[C_X_MAX]);
  xnext=clamp(xnext,Axes[0].mm-Config[C_LEAD],Axes[0].mm+Config[C_LEAD]);
  error=fmodf(Heading-angle.yaw*CHASSIS_HEADING_TEST_YAW_SIGN+540,360)-180;
  yaw=clamp(error*CHASSIS_HEADING_TEST_KP,-CHASSIS_HEADING_TEST_OUTPUT_LIMIT,CHASSIS_HEADING_TEST_OUTPUT_LIMIT);
  if(!Mecanum_Velocity_Request(v[0],v[1],yaw,CHASSIS_VELOCITY_ACCELERATION))
  { Mecanum_Status_t bus; Mecanum_StatusGet(&bus); if(bus.locked || bus.stage==MECANUM_STAGE_FAULT) fault("chassis_tx"); return; }
  Status.forward_mm_s=v[0]; Status.left_mm_s=v[1]; memcpy(PreviousVelocity,v,sizeof v);
  /* Accumulate fractional travel only on free control opportunities, bounded by
   * real feedback. A rejected position request rebases to the last accepted target. */
  XIntegral=xnext;
  if(fabsf(xnext-Axes[0].target)*Config[C_X_PPM]>=.5f && !axis_move(0,xnext))
    XIntegral=Axes[0].target;
}

static void enter_axis(GrabTask_State_t state,unsigned axis)
{ Axes[axis].commanded=false; enter(state); }
static void transfer_complete(void)
{
  OutsidePose=true;
  Status.reference_cause="transfer_return_verified";
  Status.missing=configuration_missing(Status.mode);
  if(!Status.missing) Status.missing="none";
  Status.result=Operation==OP_STORE?"stored":(Operation==OP_TAKE?"released":"returned");
  Status.reason=Status.result; Status.stop_requested=false;
  enter(GRAB_COMPLETE);
}
static void transfer_process(uint32_t now)
{
  if(Status.state==GRAB_DESCEND || Status.state==GRAB_LIFT ||
     Status.state==GRAB_PLACE || Status.state==GRAB_RETRACT || Status.state==GRAB_OBSERVE_Z)
  {
    float target;
    if(Status.state==GRAB_DESCEND) target=Config[Operation==OP_TAKE?release_key():grab_key()];
    else if(Status.state==GRAB_LIFT) target=Config[Operation==OP_TAKE?C_Z_CAR_LIFT:lift_key()];
    else if(Status.state==GRAB_PLACE) target=Config[C_Z_PLACE];
    else if(Status.state==GRAB_OBSERVE_Z) target=ObserveZ;
    else if(Operation==OP_TAKE && !ToCar) target=Config[C_Z_MAX]; /* Leave the external release vertically. */
    else if(Operation==OP_RETURN || (Operation==OP_TAKE && ToCar))
      target=fmaxf(Axes[1].mm,Config[C_Z_CAR_LIFT]);
    else target=Config[C_Z_CAR_LIFT];
    if(!Axes[1].commanded)
    {
      if(axis_move(1,target) && Status.state==GRAB_PLACE) InventoryTouched=true;
    }
    else if(arrived(1,now))
    {
      if(Status.state==GRAB_DESCEND)
      {
        if(Operation==OP_TAKE) { Arm_GripperSet(ARM_GRIPPER_OPEN); enter(GRAB_RELEASE); }
        else { Arm_GripperSet(ARM_GRIPPER_CATCH); enter(GRAB_CLOSE); }
      }
      else if(Status.state==GRAB_LIFT)
      {
        Camera_RequestStop();
        if(Operation==OP_TAKE)
        {
          if(!Turntable_InventorySet(Status.slot,TURNTABLE_EMPTY,0)) { fault("inventory_record_failed"); return; }
          InventoryTouched=false; ToCar=false;
        }
        else ToCar=true;
        enter_axis(GRAB_X_RETRACT,0);
      }
      else if(Status.state==GRAB_PLACE)
      {
        if(Operation==OP_TAKE) { Arm_GripperSet(ARM_GRIPPER_CATCH); enter(GRAB_CLOSE); }
        else { Arm_GripperSet(ARM_GRIPPER_OPEN); enter(GRAB_RELEASE); }
      }
      else if(Status.state==GRAB_OBSERVE_Z) enter_axis(GRAB_OBSERVE_X,0);
      else if(Operation==OP_TAKE && !ToCar) enter_axis(GRAB_OBSERVE_Z,1);
      else enter_axis(GRAB_X_RETRACT,0);
    }
  }
  else if(Status.state==GRAB_CLOSE && now-StateTick>=Config[C_CLOSE_MS]) enter_axis(GRAB_LIFT,1);
  else if(Status.state==GRAB_RELEASE && now-StateTick>=Config[C_CLOSE_MS])
  {
    if(Operation==OP_STORE)
    {
      if(!Turntable_InventorySet(Status.slot,TURNTABLE_OCCUPIED,(uint8_t)ActiveColor)) { fault("inventory_record_failed"); return; }
      InventoryTouched=false; ToCar=false;
    }
    enter_axis(GRAB_RETRACT,1);
  }
  else if(Status.state==GRAB_X_RETRACT || Status.state==GRAB_CAR_EXTEND ||
          Status.state==GRAB_EXTERNAL_EXTEND || Status.state==GRAB_OBSERVE_X)
  {
    float target=Status.state==GRAB_X_RETRACT?Config[C_X_MIN]:
      (Status.state==GRAB_CAR_EXTEND?Config[C_X_CAR]:(Status.state==GRAB_EXTERNAL_EXTEND?ExternalX:ObserveX));
    if(!Axes[0].commanded) (void)axis_move(0,target);
    else if(arrived(0,now))
    {
      if(Status.state==GRAB_X_RETRACT)
      {
        if(ToCar) { IndexSubmitted=false; enter(GRAB_INDEX); }
        else enter_axis(GRAB_RETURN_TURN,2);
      }
      else if(Status.state==GRAB_CAR_EXTEND) enter_axis(GRAB_PLACE,1);
      else if(Status.state==GRAB_EXTERNAL_EXTEND) enter_axis(GRAB_DESCEND,1);
      else transfer_complete();
    }
  }
  else if(Status.state==GRAB_INDEX)
  {
    Turntable_Status_t turntable;
    if(!IndexSubmitted)
    {
      if(PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy())
      {
        if(Turntable_Index(Status.slot)) IndexSubmitted=true;
        else
        {
          Turntable_StatusGet(&turntable);
          if(strcmp(turntable.reason,"busy") && strcmp(turntable.reason,"bus_busy")) fault(turntable.reason);
        }
      }
    }
    else if(!Turntable_IsBusy())
    {
      Turntable_StatusGet(&turntable);
      if(turntable.arrived && turntable.feedback_valid && !turntable.moving && turntable.slot==Status.slot) enter_axis(GRAB_TURN,2);
      else if(turntable.arrived && (!turntable.feedback_valid || turntable.moving)) fault("turntable_feedback_stale");
      else fault(turntable.reason);
    }
  }
  else if(Status.state==GRAB_TURN || Status.state==GRAB_RETURN_TURN)
  {
    float target=OutsideBase+(Status.state==GRAB_TURN?Config[C_BASE_CAR_OFFSET]:0);
    if(!Axes[2].commanded) (void)base_move(target);
    else if(arrived(2,now))
    {
      if(Status.state==GRAB_TURN) enter_axis(GRAB_CAR_EXTEND,0);
      else if(Operation==OP_TAKE) enter_axis(GRAB_EXTERNAL_EXTEND,0);
      else enter_axis(GRAB_OBSERVE_Z,1);
    }
  }
}

void GrabTask_Process(void)
{
  uint32_t now=HAL_GetTick(); bool feedback;
  if(ReportPending)
  { status_write(false); LastReport=now; ReportPending=false; }
  else if(GrabTask_IsBusy() && now-LastReport>=1000)
  { status_write(true); LastReport=now; }
  if(!GrabTask_IsBusy()) return;
  Status.elapsed_ms=now-StartTick;
  if(BootActive) { boot_process(now); return; }
  /* No elapsed-time cutoff for task stages. Advance on feedback or explicit stop. */
  if(Status.state==GRAB_PREPARE && !FeedbackSelected)
  {
    /* A diagnostic single-wheel selection persists in Mecanum; retry while its
     * outstanding read drains, then deliberately restore all four wheels. */
    Mecanum_Feedback_Enable(false);
    if(!Mecanum_Feedback_Select(0)) return;
    FeedbackSelected=true;
  }
  feedback=wheels(now);
  if(Status.state!=GRAB_PREPARE && Status.state!=GRAB_STOPPING &&
     (!feedback || !axis_fresh(0,now) || !axis_fresh(1,now) || !axis_fresh(2,now))) fault("feedback_stale");
  if(Status.state!=GRAB_STOPPING && Status.state!=GRAB_PREPARE &&
     (Axes[1].mm<Config[C_Z_MIN]-Config[C_POS_TOL] || Axes[1].mm>Config[C_Z_MAX]+Config[C_POS_TOL] ||
      (Axes[0].mm<Config[C_X_MIN]-Config[C_POS_TOL] ||
       Axes[0].mm>Config[C_X_MAX]+Config[C_POS_TOL]))) fault("axis_limit");
  if(Status.state==GRAB_STOPPING)
  {
    Mecanum_Status_t bus; Turntable_Status_t turntable; bool tray_stopped;
    stop_process(now,true);
    Mecanum_StatusGet(&bus); Turntable_StatusGet(&turntable);
    tray_stopped=!Turntable_IsBusy() && !turntable.moving && turntable.feedback_valid &&
      strcmp(turntable.reason,"stopped_unverified");
    if(Status.stop_confirmed && tray_stopped && PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy())
      enter(FinishError?GRAB_ERROR:GRAB_IDLE);
    else if((bus.locked && !MechanicalArm_IsBusy() && !Turntable_IsBusy()) ||
      (now-StopTick>(uint32_t)fmaxf(5000,Config[C_FEEDBACK_MS]+Config[C_STABLE_MS]+1000) &&
       (!feedback || !axis_fresh(0,now) || !axis_fresh(1,now) || !axis_fresh(2,now) || !tray_stopped)))
    {
      /* Missing fresh standstill evidence cannot indefinitely block explicit
       * bus recovery. The task ends unconfirmed and never resumes itself. */
      Status.stop_confirmed=false; Status.reason="stop_unconfirmed"; Status.result="failed";
      PendingAction=MECHANICAL_ARM_ACTION_NONE; enter(GRAB_ERROR);
    }
    else poll_axes(now);
    return;
  }
  if(Status.state==GRAB_PREPARE)
  {
    if(PrepareStep==0)
    {
      stop_process(now,false);
      if(Status.stop_confirmed && feedback)
      {
        if(Axes[1].mm<Config[C_Z_MIN]-Config[C_POS_TOL] || Axes[1].mm>Config[C_Z_MAX]+Config[C_POS_TOL] ||
          (Axes[0].mm<Config[C_X_MIN]-Config[C_POS_TOL] || Axes[0].mm>Config[C_X_MAX]+Config[C_POS_TOL]))
        { fault("initial_axis_limit"); return; }
        if(Operation!=OP_RETURN)
        {
          OutsideBase=Axes[2].mm; ObserveX=ExternalX=Axes[0].mm; ObserveZ=Axes[1].mm;
          OutsideReferenceValid=true; OutsidePose=true;
        }
        if(Operation==OP_TAKE || Operation==OP_RETURN) enter_axis(GRAB_RETRACT,1);
        else
        {
          Arm_GripperSet(ARM_GRIPPER_OPEN);
          if(fixed_mode()) enter_axis(GRAB_DESCEND,1);
          else PrepareStep=1;
        }
      }
    }
    else if(PrepareStep==1) { if(axis_move(1,Config[C_Z_OBSERVE])) PrepareStep=2; }
    else if(PrepareStep==2 && arrived(1,now)) PrepareStep=3;
    else if(PrepareStep==3) { if(axis_move(0,Config[C_X_PRE])) PrepareStep=4; }
    else if(PrepareStep==4 && arrived(0,now))
    {
      HAL_StatusTypeDef result;
      ObserveX=Axes[0].mm; ObserveZ=Axes[1].mm;
      result=Camera_MaterialStart(target_color());
      if(result==HAL_OK) { RequestTick=now; LastControl=now; LastInvalidCount=0; stable_reset(); enter(GRAB_ACQUIRE); }
      else if(result!=HAL_BUSY) fault("camera_tx");
    }
  }
  else if(Status.state==GRAB_VISION_PAUSE) vision_pause_process(now);
  else if(Status.state==GRAB_VISION_GRACE) vision_grace_process(now);
  else if(Status.state==GRAB_ACQUIRE || Status.state==GRAB_ALIGN || Status.state==GRAB_SETTLE)
  {
    bool usable=vision_read(now);
    if(HaveCapture && (uint32_t)(now-Vision.Tick)>Config[C_AGE_MS]) fault("vision_stale");
    if(Status.state==GRAB_ACQUIRE && usable && FreshVision)
    { enter(GRAB_ALIGN); XIntegral=Axes[0].target; Status.stop_requested=Status.stop_confirmed=false; }
    if(Status.state==GRAB_ALIGN && usable)
    {
      visual_stability(now);
      if(VisualStable) { enter(GRAB_SETTLE); stop_begin(); stable_reset(); }
      else align_control(now);
    }
    else if(Status.state==GRAB_SETTLE)
    {
      stop_process(now,false); visual_stability(now);
      if(FreshVision && !in_window() && Status.stop_confirmed)
      {
        if(Retries >= (unsigned)Config[C_RETRIES]) fault("alignment_drift");
        else
        { Retries++; enter(GRAB_ALIGN); Status.stop_requested=Status.stop_confirmed=false; stable_reset(); LastControl=now; }
      }
      if(Status.state==GRAB_SETTLE && Status.stop_confirmed && VisualStable)
      {
        if(!strcmp(Status.mode,"align")) { Status.reason="aligned_wait_check"; enter(GRAB_HOLD); }
        else { ExternalX=Axes[0].mm; enter_axis(GRAB_DESCEND,1); }
      }
    }
  }
  else transfer_process(now);
  if(GrabTask_IsBusy()) poll_axes(now);
}
void GrabTask_StatusGet(GrabTask_Status_t *out)
{
  if(!out) return;
  Status.state_name=States[Status.state]; Status.busy=GrabTask_IsBusy();
  {
    Camera_SnapshotTypeDef camera; Camera_SnapshotGet(&camera);
    Status.age_ms=camera.HasValidData?(uint32_t)(HAL_GetTick()-camera.Data.Tick):0;
    Status.rx_seq=camera.HasValidData?camera.Data.Sequence:0;
    Status.vision_state=!camera.RequestActive?"Idle":(!camera.UsbConfigured?"Off":
      (!camera.HasFrame?"Wait":((uint32_t)(HAL_GetTick()-camera.LastFrameTick)>Config[C_AGE_MS]?"Stale":
      (!camera.HasValidData || !camera.TargetValid?"Lost":"Ok"))));
  }
  Status.recovery_used=RecoveryUsed; Status.recovery_count=RecoveryCount;
  Status.loss_ms=LossActive?(uint32_t)(HAL_GetTick()-LossTick):LossLastMs;
  Status.loss_max_ms=Status.loss_ms>LossMaxMs?Status.loss_ms:LossMaxMs;
  Status.grace_count=GraceCount;
  Status.recovery_left_ms=-1;
  if(Status.state==GRAB_VISION_PAUSE && RecoveryWaiting)
  {
    uint32_t elapsed=HAL_GetTick()-RecoveryTick;
    Status.recovery_left_ms=elapsed>=GRAB_RECOVERY_WINDOW_MS?0:(int32_t)(GRAB_RECOVERY_WINDOW_MS-elapsed);
  }
  *out=Status;
}
/* Boot homing uses its own time limits: calibration is not required to establish
 * the initial reference. Emm manual pp62-63: 0x3B bit2 running, bit3 failure. */
void GrabTask_BootHomeStart(void)
{
  if(GrabTask_IsBusy()) return;
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_ALL);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  Status.reference_cause="home_verifying";
  BootActive=true; BootStopping=BootFailed=BootHomeSent=false;
  BootAxisIndex=BootStep=BootGood=0; BootTick=BootPoll=StartTick=HAL_GetTick();
  BootPositionValid=BootHomeFlagsValid=BootMotorFlagsValid=false;
  Status.reason="home_z"; Status.stop_requested=Status.stop_confirmed=false;
  enter(GRAB_HOMING);
}
static void boot_stop(const char *reason,bool failed)
{
  if(BootStopping) return;
  BootStopping=true; BootFailed=failed; BootStep=10; BootTick=HAL_GetTick();
  Status.reason=reason; Status.stop_requested=true; Status.stop_confirmed=false;
  ReportPending=true;
}
static bool boot_request(MechanicalArm_ResultTypeDef result,MechanicalArm_ActionTypeDef action,
                         MechanicalArm_AxisTypeDef axis)
{
  if(result==MECHANICAL_ARM_RESULT_BUSY) return false;
  if(result!=MECHANICAL_ARM_RESULT_NONE && result!=MECHANICAL_ARM_RESULT_OK)
  {
    if(!BootStopping) boot_stop("home_request_failed",true);
    return false;
  }
  PendingAction=action; PendingAxis=axis; return true;
}
static void boot_process(uint32_t now)
{
  MechanicalArm_AxisTypeDef axis=BootAxes[BootAxisIndex];
  if(now-BootTick>(BootStopping?5000U:60000U))
  {
    if(!BootStopping) boot_stop("home_timeout",true);
    else { BootActive=false; Status.reason="home_stop_unconfirmed"; enter(GRAB_ERROR); return; }
  }
  if(BootStep==10) /* Drain ordinary reads first; external priority stop may already have cancelled them. */
  {
    if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy()) return;
    if(boot_request(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL),MECHANICAL_ARM_ACTION_STOP,
                    MECHANICAL_ARM_AXIS_ALL)) BootStep=11;
    return;
  }
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy() || now-BootPoll<100) return;
  BootPoll=now;
  if(BootStep==0 && now-BootTick>=1000)
  {
    if(boot_request(MechanicalArm_Home(axis,axis==MECHANICAL_ARM_AXIS_BASE?0:2),MECHANICAL_ARM_ACTION_HOME,axis))
    { BootHomeSent=true; BootStep=1; }
  }
  else if(BootStep==2 || BootStep==14)
    (void)boot_request(MechanicalArm_HomeStateRead(axis),MECHANICAL_ARM_ACTION_HOME_STATE,axis);
  else if(BootStep==3)
    (void)boot_request(MechanicalArm_PositionRead(axis),MECHANICAL_ARM_ACTION_POSITION_READ,axis);
  else if(BootStep==4)
    (void)boot_request(MechanicalArm_StateRead(axis),MECHANICAL_ARM_ACTION_STATE,axis);
  else if(BootStep==12)
  {
    if(boot_request(MechanicalArm_HomeAbort(axis),MECHANICAL_ARM_ACTION_HOME_ABORT,axis)) BootStep=13;
  }
}
static uint8_t boot_event(const MechanicalArm_EventTypeDef *event)
{
  MechanicalArm_AxisTypeDef axis=BootAxes[BootAxisIndex];
  if(PendingAction==MECHANICAL_ARM_ACTION_NONE || event->Action!=PendingAction || event->Axis!=PendingAxis) return 0;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  if(event->Result!=MECHANICAL_ARM_RESULT_OK)
  {
    if(!BootStopping) boot_stop("home_feedback_failed",true);
    else if(BootStep==10 || BootStep==11) BootStep=BootStep==11?12:10;
    else { BootActive=false; Status.reason="home_stop_unconfirmed"; enter(GRAB_ERROR); }
    return 1;
  }
  if(BootStopping)
  {
    if(event->Action==MECHANICAL_ARM_ACTION_STOP)
    {
      if(!BootHomeSent) { BootActive=false; enter(BootFailed?GRAB_ERROR:GRAB_IDLE); }
      else BootStep=12;
    }
    else if(event->Action==MECHANICAL_ARM_ACTION_HOME_ABORT) BootStep=14;
    else if(event->Action==MECHANICAL_ARM_ACTION_HOME_STATE && !(event->StateFlags[axis]&4U))
    { BootActive=false; enter(BootFailed?GRAB_ERROR:GRAB_IDLE); }
    return 1; /* stop_confirmed stays false: abort flag is not a standstill measurement. */
  }
  if(event->Action==MECHANICAL_ARM_ACTION_HOME) { BootStep=2; BootPoll=HAL_GetTick(); }
  else if(event->Action==MECHANICAL_ARM_ACTION_HOME_STATE)
  {
    uint8_t flags=event->StateFlags[axis];
    BootHomeFlags=flags; BootHomeFlagsValid=true;
    if((flags&0x38U) || (flags&3U)!=3U) boot_stop("home_driver_fault",true);
    else if(!(flags&4U)) BootStep=3;
    else { BootGood=0; BootPositionValid=false; }
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_POSITION_READ)
  {
    int64_t raw=event->CurrentPositionRaw;
    bool stable=BootPositionValid && llabs(raw-BootPositionRaw)<=182;
    BootPositionRaw=raw; BootPositionValid=true;
    /* Base near-home targets a saved single-turn mechanical origin, not the
     * multi-turn position counter's numerical zero. Do not clear that counter. */
    if(axis==MECHANICAL_ARM_AXIS_BASE ? !stable : (raw < -182 || raw > 182))
    { BootGood=0; BootStep=2; }
    else BootStep=4;
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_STATE)
  {
    uint8_t flags=event->StateFlags[axis];
    BootMotorFlags=flags; BootMotorFlagsValid=true;
    if((flags&0x0cU) || !(flags&1U)) boot_stop("home_axis_fault",true);
    /* 0x3B already confirms homing finished; Base need not assert the separate
     * position-move reached flag after near-home. Enabled/non-stall is required. */
    else if(axis!=MECHANICAL_ARM_AXIS_BASE && (flags&3U)!=3U) { BootGood=0; BootStep=2; }
    else if(++BootGood<2) BootStep=2;
    else
    {
      /* Restore known defaults only after this axis has passed homing feedback. */
      if(axis==MECHANICAL_ARM_AXIS_Z)
      {
        Config[C_Z_PPM]=480; Config[C_Z_MIN]=-80; Config[C_Z_MAX]=0;
        Config[C_Z_GRAB]=-80; Config[C_Z_LIFT]=-40; Config[C_Z_PLACE]=-60; Config[C_Z_CAR_LIFT]=0;
        Config[C_Z_OBSERVE]=0;
      }
      else if(axis==MECHANICAL_ARM_AXIS_X)
      {
        /* User measurement: 100 motor degrees -> 28 mm; 3200 command pulses/rev. */
        Config[C_X_PPM]=3200.0f*100.0f/(360.0f*28.0f);
        Config[C_X_MIN]=0; Config[C_X_MAX]=100; Config[C_X_PRE]=0;
      }
      else if(axis==MECHANICAL_ARM_AXIS_BASE) visual_defaults_restore();
      if(axis==MECHANICAL_ARM_AXIS_BASE) Status.reference_cause="home_verified";
      Status.missing=configuration_missing(Status.mode);
      if(!Status.missing) Status.missing="none";
      BootAxisIndex++; BootGood=BootStep=0; BootHomeSent=false; BootTick=HAL_GetTick();
      BootPositionValid=BootHomeFlagsValid=BootMotorFlagsValid=false;
      if(BootAxisIndex==3)
      { BootActive=false; Status.reason="home_complete"; Status.missing=configuration_missing(Status.mode);
        if(!Status.missing) Status.missing="none";
        enter(GRAB_IDLE);
        /* Start the IMU's fresh-data timeout and 5s verification only now. */
        HWT101_Cal_BootVerifyArm(); }
      else { Status.reason=BootAxisIndex==1?"home_x":"home_base"; ReportPending=true; }
    }
  }
  return 1;
}
static bool config_value_valid(unsigned key,float value)
{
  if(!isfinite(value)) return false;
  if(key==C_X_PPM || key==C_Z_PPM) return value>0 && value<=100000;
  if(key==C_Z_PLACE || (key>=C_X_MIN && key<=C_Z_LIFT) ||
     (key>=C_Z_PROC_GRAB && key<=C_X_CAR)) return fabsf(value)<=10000;
  if(key==C_BASE_CAR_OFFSET) return fabsf(value)<=360;
  if(key>=C_J00 && key<=C_J12) return fabsf(value)<=10000;
  if(key==C_REF_U || key==C_REF_V) return value>=0 && value<= (key==C_REF_U?639:479) && floorf(value)==value;
  if(key==C_RETRIES) return value>=0 && value<=10 && floorf(value)==value;
  if(key==C_LOSS_GRACE_MS) return value>=0 && value<=300 && floorf(value)==value;
  if(key==C_FRAMES) return value>=3 && value<=100 && floorf(value)==value;
  if(key==C_CLOSE_MS || key==C_STABLE_MS || key==C_AGE_MS || key==C_FEEDBACK_MS)
  {
    float min=key==C_STABLE_MS?200:((key==C_AGE_MS || key==C_FEEDBACK_MS)?50:1);
    float max=(key==C_AGE_MS || key==C_FEEDBACK_MS)?5000:120000;
    return value>=min && value<=max && floorf(value)==value;
  }
  if(key==C_ACCEL) return value>0 && value<=1000;
  if(key==C_LAMBDA) return value>=.001f && value<=10;
  if(key==C_X_WEIGHT) return value>=.01f && value<=100;
  if(key==C_TRAVEL) return value>=0 && value<=10000;
  if(key==C_LEAD) return value>0 && value<=10000;
  return value>0 && value<=100;
}
static void reply(const char *text)
{ (void)ConsoleTx_Write((const uint8_t *)text,(uint16_t)strlen(text)); }
/* Independent latest-only slot: raw Pi pixels must not overwrite grab progress.
 * Invalid snapshots retain old coordinates in Camera, so never print those as current. */
static void camera_status_write(void)
{
  Camera_SnapshotTypeDef snapshot; char line[224]; uint32_t age; bool valid;
  if(fixed_mode() || (Status.state!=GRAB_ACQUIRE && Status.state!=GRAB_ALIGN &&
    Status.state!=GRAB_SETTLE && Status.state!=GRAB_VISION_PAUSE && Status.state!=GRAB_VISION_GRACE && !(Status.state==GRAB_HOLD && !strcmp(Status.mode,"align"))))
  { ConsoleTx_DebugCancel(CONSOLE_DEBUG_CAMERA); return; }
  Camera_SnapshotGet(&snapshot);
  valid=snapshot.RequestActive && snapshot.RequestFunction==0xB2 &&
    snapshot.RequestTarget==target_color() && snapshot.HasFrame && snapshot.HasValidData &&
    snapshot.TargetValid && snapshot.Data.Function==0xB2 && snapshot.Data.Target==target_color();
  if(valid)
  {
    age=(uint32_t)(HAL_GetTick()-snapshot.Data.Tick);
    (void)snprintf(line,sizeof line,
      "VISION RX fn=B2 color=%u valid=1 fresh=%u cx=%u cy=%u dx=%d dy=%d rx_seq=%lu age_source=rx age=%lu\r\n",
      (unsigned)target_color(),age<=Config[C_AGE_MS]?1U:0U,snapshot.Data.CX,snapshot.Data.CY,
      snapshot.Data.DX,snapshot.Data.DY,
      (unsigned long)snapshot.Data.Sequence,(unsigned long)age);
  }
  else
    (void)snprintf(line,sizeof line,"VISION RX fn=B2 color=%u valid=0 fresh=0 cx=na cy=na dx=na dy=na rx_seq=na age_source=rx age=na\r\n",(unsigned)target_color());
  (void)ConsoleTx_Debug(CONSOLE_DEBUG_CAMERA,line,(uint16_t)strlen(line));
}
static void status_write(bool debug)
{
  char line[640], dx[8]="na", dy[8]="na", remaining[12]="na"; GrabTask_Status_t s; GrabTask_StatusGet(&s);
  if(s.recovery_left_ms>=0) (void)snprintf(remaining,sizeof remaining,"%ld",(long)s.recovery_left_ms);
  if(HaveCapture && s.state!=GRAB_VISION_GRACE && !strcmp(s.vision_state,"Ok"))
  {
    (void)snprintf(dx,sizeof dx,"%d",s.dx);
    (void)snprintf(dy,sizeof dy,"%d",s.dy);
  }
  if(!debug && !strcmp(s.reason,"config_missing"))
  {
    char missing[256]=""; size_t used=0; unsigned i;
    /* Report every unknown input in one ordered reply, without debug truncation. */
    for(i=0;i<C_COUNT;i++)
      if(configuration_required(i,s.mode) && !isfinite(Config[i]))
      {
        size_t length=strlen(Keys[i]);
        if(used+length+2>=sizeof missing) break;
        if(used) missing[used++]=',';
        memcpy(missing+used,Keys[i],length+1); used+=length;
      }
    if(!used)
    {
      const char *invalid=configuration_missing(s.mode);
      (void)snprintf(missing,sizeof missing,"%s",invalid?invalid:"none");
    }
    (void)snprintf(line,sizeof line,
      "OK grab state=%s mode=%s reason=%s missing=%s ref_cause=%s operation=%s scene=%s slot=%u result=%s stop_requested=%u stop_confirmed=%u elapsed=%lu\r\n",
      s.state_name,s.mode,s.reason,missing,s.reference_cause,s.operation,s.scene,(unsigned)s.slot,s.result,s.stop_requested?1U:0U,s.stop_confirmed?1U:0U,(unsigned long)s.elapsed_ms);
    reply(line); return;
  }
  if(s.state==GRAB_HOMING)
  {
    char raw[32]="na", home[8]="na", motor[8]="na";
    static const char *const names[]={"z","x","base"};
    if(BootPositionValid) (void)snprintf(raw,sizeof raw,"%lld",(long long)BootPositionRaw);
    if(BootHomeFlagsValid) (void)snprintf(home,sizeof home,"0x%02X",BootHomeFlags);
    if(BootMotorFlagsValid) (void)snprintf(motor,sizeof motor,"0x%02X",BootMotorFlags);
    (void)snprintf(line,sizeof line,
      "OK grab state=homing mode=%s reason=%s axis=%s step=%u home_flags=%s raw=%s motor_flags=%s stable=%u stop_requested=%u stop_confirmed=0 elapsed=%lu\r\n",
      s.mode,s.reason,names[BootAxisIndex],BootStep,home,raw,motor,BootGood,
      s.stop_requested?1U:0U,(unsigned long)s.elapsed_ms);
    if(debug) (void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION,line,(uint16_t)strlen(line));
    else reply(line);
    return;
  }
  (void)snprintf(line,sizeof line,
    "OK grab state=%s mode=%s reason=%s missing=%s ref_cause=%s operation=%s scene=%s slot=%u result=%s x=%.3f xt=%.3f z=%.3f zt=%.3f b=%.2f bt=%.2f dx=%s dy=%s protocol=B2 age_source=rx rx_seq=%lu vision=%s color=%u model=%s age=%lu vf=%.3f vl=%.3f stop_requested=%u stop_confirmed=%u elapsed=%lu recovery_used=%lu recovery_count=%lu recovery_left_ms=%s loss_ms=%lu loss_max_ms=%lu grace_count=%lu\r\n",
    s.state_name,s.mode,s.reason,s.missing,s.reference_cause,s.operation,s.scene,(unsigned)s.slot,s.result,s.x_mm,s.x_target_mm,s.z_mm,s.z_target_mm,
    Axes[2].position_valid?Axes[2].mm:NAN,Axes[2].commanded?Axes[2].target:NAN,
    dx,dy,(unsigned long)s.rx_seq,s.vision_state,(unsigned)target_color(),visual_model_name(),(unsigned long)s.age_ms,s.forward_mm_s,s.left_mm_s,
    s.stop_requested?1U:0U,s.stop_confirmed?1U:0U,(unsigned long)s.elapsed_ms,
    (unsigned long)s.recovery_used,(unsigned long)s.recovery_count,remaining,
    (unsigned long)s.loss_ms,(unsigned long)s.loss_max_ms,(unsigned long)s.grace_count);
  if(debug)
  {
    camera_status_write();
    /* Latest-only bounded telemetry never occupies the ordered command reply FIFO. */
    size_t length=strlen(line);
    if(length>319)
    {
      /* Preserve complete key/value tokens when the full snapshot exceeds the debug slot. */
      if(Operation==OP_ALIGN || Status.state==GRAB_ACQUIRE || Status.state==GRAB_ALIGN ||
         Status.state==GRAB_SETTLE || Status.state==GRAB_VISION_PAUSE || Status.state==GRAB_VISION_GRACE)
        (void)snprintf(line,sizeof line,"OK grab state=%s mode=%s reason=%s protocol=B2 age_source=rx rx_seq=%lu vision=%s dx=%s dy=%s color=%u stop_requested=%u stop_confirmed=%u recovery_used=%lu recovery_count=%lu recovery_left_ms=%s loss_ms=%lu loss_max_ms=%lu grace_count=%lu\r\n",
          s.state_name,s.mode,s.reason,(unsigned long)s.rx_seq,s.vision_state,dx,dy,(unsigned)target_color(),
          s.stop_requested?1U:0U,s.stop_confirmed?1U:0U,(unsigned long)s.recovery_used,(unsigned long)s.recovery_count,remaining,
          (unsigned long)s.loss_ms,(unsigned long)s.loss_max_ms,(unsigned long)s.grace_count);
      else
        (void)snprintf(line,sizeof line,"OK grab state=%s mode=%s reason=%s operation=%s scene=%s slot=%u result=%s x=%.3f xt=%.3f z=%.3f zt=%.3f ref_cause=%s color=%u stop_requested=%u stop_confirmed=%u\r\n",
          s.state_name,s.mode,s.reason,s.operation,s.scene,(unsigned)s.slot,s.result,s.x_mm,s.x_target_mm,
          s.z_mm,s.z_target_mm,s.reference_cause,(unsigned)target_color(),s.stop_requested?1U:0U,s.stop_confirmed?1U:0U);
      length=strlen(line);
    }
    (void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION,line,(uint16_t)length);
  }
  else reply(line);
}
bool GrabTask_Command(unsigned count,char *tokens[])
{
  unsigned i; size_t length=0;
  if(!tokens || !count || strcmp(tokens[0],"grab")) return false;
  for(i=0;i<count;i++) { if(!tokens[i]) { reply("ERR grab syntax\r\n"); return true; } length+=strlen(tokens[i])+(i?1:0); }
  if(count>6 || length>63) { reply("ERR grab syntax\r\n"); return true; }
  if(count==2 && !strcmp(tokens[1],"status")) status_write(false);
  else if(count==3 && !strcmp(tokens[1],"get"))
  {
    char line[96];
    for(i=0;i<C_COUNT;i++) if(!strcmp(tokens[2],Keys[i])) break;
    if(i==C_COUNT) reply("ERR grab config\r\n");
    else
    {
      if(isfinite(Config[i])) (void)snprintf(line,sizeof line,"OK grab config key=%s value=%.6g\r\n",Keys[i],Config[i]);
      else (void)snprintf(line,sizeof line,"OK grab config key=%s value=unset\r\n",Keys[i]);
      reply(line);
    }
  }
  else if(count==2 && !strcmp(tokens[1],"stop")) { GrabTask_Stop(); status_write(false); }
  else if(count==2 && !strcmp(tokens[1],"rehome"))
  {
    Mecanum_Status_t bus;
    Mecanum_StatusGet(&bus);
    if((Status.state!=GRAB_IDLE && Status.state!=GRAB_COMPLETE) || MechanicalArm_IsBusy() || ArmVision_IsBusy() ||
       MaterialVision_IsBusy() || ChassisRoute_IsBusy() ||
       ChassisMotion_IsBusy() || Mecanum_IsBusy() || HWT101_Cal_IsBusy() || Turntable_IsBusy() ||
       bus.locked || bus.stop_pending || bus.ack_profile!=MECANUM_ACK_RECEIVE)
      reply("ERR grab rehome rejected; stop tasks and inspect bus status\r\n");
    else { GrabTask_BootHomeStart(); reply("OK grab rehome accepted; Z/X/Base verification pending\r\n"); status_write(false); }
  }
  else if(count>=3 && count<=5 && !strcmp(tokens[1],"start"))
  {
    Camera_ColorTypeDef color=default_color(tokens[2]); uint8_t slot=0;
    bool fixed=!strcmp(tokens[2],"fixed"), align=!strcmp(tokens[2],"align");
    if((fixed && count!=3 && count!=5) || (align && count>4))
    { reply("ERR grab start color; fixed <slot> <color>, align [color], pick [color] [slot]\r\n"); return true; }
    if(count>=4)
    {
      const char *chosen=tokens[fixed?4:3];
      if(strlen(chosen)!=1 || chosen[0]<'1' || chosen[0]>'6')
      { reply("ERR grab start color; use 1..6\r\n"); return true; }
      color=(Camera_ColorTypeDef)(chosen[0]-'0');
    }
    if(count==5)
    {
      const char *chosen=tokens[fixed?3:4];
      if(strlen(chosen)!=1 || chosen[0]<'1' || chosen[0]>'3')
      { reply("ERR grab slot; use 1..3\r\n"); return true; }
      slot=(uint8_t)(chosen[0]-'0');
    }
    if(!start_operation(tokens[2],color,align?OP_ALIGN:OP_STORE,SCENE_RAW,slot))
      reply("ERR grab start rejected; inspect grab status\r\n");
    else
    {
      char line[128];
      (void)snprintf(line,sizeof line,"OK grab start %s accepted color=%u protocol=B2\r\n",
        Status.mode,(unsigned)target_color());
      reply(line);
    }
    status_write(false);
  }
  else if(count==5 && !strcmp(tokens[1],"store") && !strcmp(tokens[2],"proc"))
  {
    if(strlen(tokens[3])!=1 || tokens[3][0]<'1' || tokens[3][0]>'3' ||
       strlen(tokens[4])!=1 || tokens[4][0]<'1' || tokens[4][0]>'6')
      reply("ERR grab store; use proc <slot 1..3> <color 1..6>\r\n");
    else if(!start_operation("fixed",(Camera_ColorTypeDef)(tokens[4][0]-'0'),OP_STORE,SCENE_PROC,(uint8_t)(tokens[3][0]-'0')))
      reply("ERR grab store rejected; inspect grab status\r\n");
    else reply("OK grab store proc accepted\r\n");
    status_write(false);
  }
  else if(count==4 && !strcmp(tokens[1],"take"))
  {
    GrabScene_t scene;
    if(!strcmp(tokens[3],"rough")) scene=SCENE_ROUGH;
    else if(!strcmp(tokens[3],"temp1")) scene=SCENE_TEMP1;
    else if(!strcmp(tokens[3],"temp2")) scene=SCENE_TEMP2;
    else { reply("ERR grab take scene; use rough|temp1|temp2\r\n"); return true; }
    if(strlen(tokens[2])!=1 || tokens[2][0]<'1' || tokens[2][0]>'3')
      reply("ERR grab slot; use 1..3\r\n");
    else if(!start_operation("fixed",GRAB_PICK_COLOR,OP_TAKE,scene,(uint8_t)(tokens[2][0]-'0')))
      reply("ERR grab take rejected; inspect grab status\r\n");
    else reply("OK grab take accepted\r\n");
    status_write(false);
  }
  else if(count==2 && !strcmp(tokens[1],"return"))
  {
    if(!start_operation("fixed",GRAB_PICK_COLOR,OP_RETURN,SCENE_RAW,0))
      reply("ERR grab return rejected; inspect grab status\r\n");
    else reply("OK grab return accepted\r\n");
    status_write(false);
  }
  else if(count==4 && !strcmp(tokens[1],"set"))
  {
    char *end; float value=strtof(tokens[3],&end);
    if(GrabTask_IsBusy()) { reply("ERR grab busy\r\n"); return true; }
    for(i=0;i<C_COUNT;i++) if(!strcmp(tokens[2],Keys[i])) break;
    if(i==C_COUNT || end==tokens[3] || *end || !config_value_valid(i,value)) reply("ERR grab config\r\n");
    else
    { Config[i]=value; if(i>=C_J00 && i<=C_J12) VisualModelCustom=true; Status.missing=configuration_missing(Status.mode); if(!Status.missing) Status.missing="none"; reply("OK grab set\r\n"); }
  }
  else reply("ERR grab syntax\r\n");
  return true;
}
