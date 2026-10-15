#include "GrabTask.h"
#include "QR.h"
#include "Arm.h"
#include "grab_calibration.h"
#include "Camera.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include "hwt101_calibration.h"
#include "mecanum_chassis.h"
#include "mechanical_arm_config.h"
#include "console_tx.h"
#include "motor_bus.h"
#include "motor_id_config.h"
#include "turntable.h"
#include "grab_tracking.h"
#include "chassis_observer.h"
#include <stddef.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Temporary Base angle tolerance until the mechanical structure is adjusted. */
#define GRAB_BASE_POSITION_TOL_DEG 2.0f

/* RAM-only mechanical references are restored after boot homing.
 * Vision starts from an explicitly unmeasured trial model, adjustable with grab set. */
enum
{
#define PARAM(id,key,initial,min,max,flags,phase) id,
#include "grab_parameters.def"
#undef PARAM
  C_COUNT
};
static const char *const Keys[C_COUNT] = {
#define PARAM(id,key,initial,min,max,flags,phase) key,
#include "grab_parameters.def"
#undef PARAM
};
typedef struct
{
  float initial, minimum, maximum;
  unsigned char validation, phase;
} GrabParameter_t;
static const GrabParameter_t Parameters[C_COUNT] = {
#define PARAM(id,key,initial,min,max,flags,phase) {initial,min,max,flags,phase},
#include "grab_parameters.def"
#undef PARAM
};
enum { DEFAULT_INIT=1, DEFAULT_X_HOME=2, DEFAULT_Z_HOME=4, DEFAULT_VISUAL=8 };
static const char *const States[] = {"idle","prepare","acquire","align","settle",
  "descend","close","lift","turn","place","release","retract","hold","stopping","error","homing","vision_pause","vision_grace",
  "x_retract","index","car_extend","return_turn","external_extend","observe_z","observe_x","complete",
  "track_observe","track_match","track_follow","track_pause","route_pose","manual_z","calibration","qr_scan"};
typedef struct
{
  float mm, speed, target, signed_speed;
  int64_t raw;
  uint32_t position_tick, state_tick, quiet_since, command_tick;
  uint8_t flags;
  bool position_valid, state_valid, quiet, commanded;
  bool arrival_waiting, arrival_reported, arrival_accepted;
  bool target_reused;
  float arrival_anchor;
  uint32_t arrival_since, arrival_position_tick, arrival_state_tick;
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
#define GRAB_RECOVERY_WINDOW_MS ((uint32_t)Config[C_VISION_WAIT_MS])
#define GRAB_CONTROL_PERIOD_MS 100U
#define GRAB_RECOVERY_OBSERVATIONS 3U
/* RecoveryUsed is diagnostic only; successful reacquisitions do not exhaust a quota. */
static unsigned PrepareStep, PollIndex, Retries;
static uint32_t RecoveryUsed, RecoveryCount, RecoveryTick, RecoverySequence;
static bool RecoveryWaiting, RecoveryRestartPending, RecoveryPassive;
static uint32_t RecoveryValidTick;
static bool LossActive;
static uint32_t LossTick, LossLastMs, LossMaxMs, GraceCount, GraceSequence;
static bool HaveCapture, VisualStable, StopArmSent, StopArmTx, StopChassisSent;
static bool WheelQuiet, WheelBaseline, FinishError, FreshVision;
static bool StopEvidenceStarted;
static bool FeedbackSelected;
static float XIntegral;
static uint32_t LastReport;
static bool ReportPending;
static uint32_t QrInitialSequence;
static unsigned QrZStep;
static float QrStartZ;
static uint32_t QrZWaitTick;
static bool QrCompetition;
static void status_write(bool debug);
static void reply(const char *text);
static void align_control(uint32_t now);
static float Heading, PreviousVelocity[3];
static Camera_DataTypeDef Vision;
static Camera_ColorTypeDef ActiveColor;
static bool BootActive, BootStopping, BootFailed, BootHomeSent;
static bool BootLaunching, BootStopUnconfirmed, BootWaitTurntable;
static bool ManualBaseHomePending, BootBaseOnly, BootResumePose, BootResumePoseWork;
static uint8_t BootStartedMask, BootVerifiedMask, BootAbortMask, BootAbortSentMask;
static uint32_t BootHomeTick[3];
static unsigned BootAxisIndex, BootStep, BootGood;
static int64_t BootPositionRaw;
static bool BootPositionValid, BootPositionStable;
static uint8_t BootHomeFlags, BootMotorFlags;
static bool BootHomeFlagsValid, BootMotorFlagsValid;
static uint32_t BootTick, BootPoll, BootEnableTick, BootDisabledTick;
static bool BootDisabledWaiting, BootEnableRecovery;
static bool BootBaseDegraded, BootAcceptBase, BaseHomeAcceptAvailable;
static uint8_t BaseHomeFailureFlags;
static int64_t BootRecoveryRaw;
/* A full home/position/state round takes at least 300 ms at the 100 ms poll.
 * Allow two rounds for transient disable, then one bounded enable verification. */
#define BOOT_DISABLE_GRACE_MS 600U
#define BOOT_ENABLE_VERIFY_MS 1500U
static const MechanicalArm_AxisTypeDef BootAxes[3]={MECHANICAL_ARM_AXIS_Z,MECHANICAL_ARM_AXIS_X,MECHANICAL_ARM_AXIS_BASE};
static void boot_stop(const char *reason,bool failed);
static void boot_process(uint32_t now);
static uint8_t boot_event(const MechanicalArm_EventTypeDef *event);
static void base_reference_verify_start(bool work);
typedef enum { OP_ALIGN, OP_STORE, OP_TAKE, OP_RETURN, OP_CAR_PLACE, OP_TRACK_PICK, OP_CAR_PICK } GrabOperation_t;
typedef enum { SCENE_RAW, SCENE_PROC, SCENE_ROUGH, SCENE_TEMP1, SCENE_TEMP2 } GrabScene_t;
static GrabOperation_t Operation;
static GrabScene_t Scene;
static bool OutsideReferenceValid, OutsidePose, ToCar, IndexSubmitted, InventoryTouched;
static float OutsideBase, ObserveX, ObserveZ, ExternalX;
static float RearBase;
/* Transfer context survives an ordinary stop. Held/released are software
 * observations of servo commands, not a physical object sensor. */
static bool TrackStoreActive,TrackStoreReleased,TrackStoreRecorded,TrackHeld;
static bool StoreFinishTravel;
static struct {
  bool pending,automatic,pick,observe,finish_travel;
  Camera_ColorTypeDef color; uint8_t slot; GrabTracking_Mode_t mode;
  float pickup,speed,budget;
} NextTrack;
static bool tracking_start(Camera_ColorTypeDef color,uint8_t slot,GrabTracking_Mode_t mode,
 bool automatic,bool pick,bool observe,float pickup,float speed,float budget);
static bool track_store_can_replace(void)
{ return TrackStoreActive && TrackStoreReleased && TrackStoreRecorded && !TrackHeld; }
static uint8_t TrackStoreSlot;
static float TrackStoreLiftTarget,TrackStorePlaceTarget;
static uint32_t TrackStoreReleaseTick;
static bool CarPickActive,CarPickClosed,CarPickWaiting;
static uint8_t CarPickSlot,CarPickColor;
static float CarPickLiftTarget,CarPickTarget,CarPickClearanceX;
/* Only processed material returning to tray 3 uses this clearance sequence.
 * Saved reference-relative angles survive stop/recovery; never accumulate jogs. */
static struct {
  bool active,returning,submitted,verified,recheck;
  float outside_angle,return_angle,lift_z;
} ProcClearance;
static uint32_t CarPickWaitTick,CarPickCloseTick;
static bool RearReferenceValid, RearPose, PoseWork;
static unsigned PoseStep;
static uint32_t PoseEvidenceTick;
static bool PoseQr;
static bool QrDeparting;
static uint8_t NextRawColor;
static bool NextRawRequested;
static const char *const OperationNames[]={"align","store","take","return","car_place","track_pick","car_pick"};
static const char *const SceneNames[]={"raw","proc","rough","temp1","temp2"};
static unsigned grab_key(void) { return Scene==SCENE_PROC?C_Z_PROC_PLACE:C_Z_GRAB; }
static unsigned lift_key(void) { return Scene==SCENE_PROC?C_Z_CAR_LIFT:C_Z_LIFT; }
static unsigned release_key(void)
{ return Scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE; }
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
/* CPU-only estimator/trace data; no peripheral DMA accesses this state. */
static GrabTracking_t Tracking __attribute__((section("CCM_TRACKING"), zero_init));
#else
static GrabTracking_t Tracking;
#endif
static GrabTracking_Config_t TrackConfig;
static GrabTracking_Config_t TrackComparison[2];
static GrabTracking_Mode_t TrackMode;
static unsigned TrackEntrySamples;
static GrabTracking_Config_t *track_config(GrabTracking_Mode_t mode)
{ return mode==GRAB_TRACK_PREDICT?&TrackConfig:&TrackComparison[(unsigned)mode-1U]; }
static const char *track_mode_name(GrabTracking_Mode_t mode)
{ return mode==GRAB_TRACK_VISUAL?"visual":mode==GRAB_TRACK_FEEDBACK?"feedback":"predict"; }
static bool track_mode_parse(const char *text,GrabTracking_Mode_t *mode)
{
  for(unsigned i=0;i<=GRAB_TRACK_FEEDBACK;i++)if(!strcmp(text,track_mode_name((GrabTracking_Mode_t)i)))
  {*mode=(GrabTracking_Mode_t)i;return true;}
  return false;
}
static GrabTracking_Input_t TrackInput;
static GrabTracking_Output_t TrackOutput;
static ChassisObserver_t TrackObserver;
#if defined(__CC_ARM)
static ChassisObserver_History_t TrackObserverHistory __attribute__((section("CCM_TRACKING"), zero_init));
#else
static ChassisObserver_History_t TrackObserverHistory;
#endif
static ChassisModel_Velocity_t TrackPositionVelocity;
static struct {
  int64_t position; uint32_t tick; float units_s,offset,target_offset;
  bool valid,anchored;
} TrackWheel[4];
static struct { uint32_t tick; float units[4]; } TrackCommandHistory[32];
static float TrackCommandUnitsS[4];
static unsigned TrackCommandCount;
static uint32_t TrackCommandApplied;
static bool TrackCommandActive;
static uint32_t TrackObserverTick, TrackLastBatch, TrackBatchInterval;
static uint32_t TrackLastStateRead, TrackLastRequest, TrackTraceTick;
static unsigned TrackSlowPoll, TrackTraceIndex, TrackIoSlot;
static struct { uint8_t address; const char *kind; uint32_t age_ms, limit_ms; bool valid; } TrackFeedbackFault;
static struct { uint8_t address,flags; const char *kind; uint32_t received_ms,detected_ms; } TrackMotorFault;
static bool TrackReady, TrackMoving, TrackVisualPause, TrackTraceSending, TrackXZeroValid, TrackTraceSingle;
static float TrackHeading, TrackXZero, TrackPulseRemainder;
static int32_t TrackLastPulse;
static uint16_t TrackLastRpm;
static bool TrackPulseValid;
/* One-button entry observes before motion. Existing references retain their budget. */
static bool TrackAutoStart, TrackPickArmed, TrackPickLowering;
static bool TrackPickHoldActive;
static bool TrackCloseStopping, TrackCloseZeroRequested, TrackCloseXRequested;
static uint32_t TrackPickHoldStart, TrackPickHoldLast;
static uint32_t TrackZPollTick;
static bool TrackZPollState;
static float TrackEntryBudget;
static bool TrackEntryHaveSample;
static uint32_t TrackEntryFirstSample;
static uint32_t TrackControlSequence, TrackXControlSequence;
static struct {
  int32_t requested_pulses,sent_pulses;
  uint16_t requested_rpm,sent_rpm;
  uint32_t request_ms,sent_ms;
  uint8_t action;
  bool requested,transmitted,acknowledged;
} TrackXDispatch;
/* One passive IO snapshot, not 128 copies in the frozen control ring.
 * Wheel raw position/RPM retain protocol signs; sent RPM uses logical signs.
 * valid bits: 0..3 wheel position,4 sent wheels,5 X TX,6 X ACK,7 X position,
 * 8 wheel ACK,9 complete batch,10 X accepted target,11..14 wheel speed. */
typedef struct {
  int32_t sent_rpm[4],feedback_rpm[4],x_requested_pulses,x_sent_pulses;
  int64_t wheel_raw[4],x_raw;
  uint16_t wheel_age_ms[4],wheel_speed_age_ms[4],x_requested_rpm,x_sent_rpm;
  uint16_t x_sent_age_ms,x_request_age_ms,sent_age_ms,io_valid;
  uint8_t x_sent_action,bus_stage,bus_error,bus_locked;
  uint32_t motion_sequence,stop_sequence;
} TrackIo_t;
static TrackIo_t TrackIo;
static bool TrackLogEnabled,TrackLogHaveTick;
static uint32_t TrackLogId,TrackLogIndex,TrackLogPeriod,TrackLogTick;
static const ChassisModel_Geometry_t TrackGeometry={CHASSIS_WHEELBASE_MM,
  CHASSIS_TRACK_WIDTH_MM,CHASSIS_WHEEL_DIAMETER_MM,CHASSIS_MOTOR_TO_WHEEL_RATIO};
static void track_process(uint32_t now);
static void track_status_write(bool debug);
static void track_observer(uint32_t now);
static void track_close_stop(void);
static void track_trace_process(void);
static void track_record_io(TrackIo_t *record,uint32_t now);
static void track_log_record(const GrabTracking_Record_t *record);
static bool track_command(unsigned count,char *tokens[]);
static bool tracking_mode(void) { return !strcmp(Status.mode,"track"); }
static void track_motor_fault(uint8_t address,const char *kind,uint8_t flags,uint32_t received,uint32_t now)
{
  if(!tracking_mode() || TrackMotorFault.address)return;
  TrackMotorFault.address=address;TrackMotorFault.kind=kind;TrackMotorFault.flags=flags;
  TrackMotorFault.received_ms=received;TrackMotorFault.detected_ms=now;
}
static void invalidate_inventory(void)
{
  if(InventoryTouched && Status.slot)
    (void)Turntable_InventorySet(Status.slot,TURNTABLE_UNKNOWN,0);
  InventoryTouched=false;
}

static float clamp(float value, float low, float high)
{ return value < low ? low : (value > high ? high : value); }
static bool fixed_mode(void) { return strcmp(Status.mode, "fixed") == 0; }
static bool RingTest, RingPlaceActive;
static bool StaticAlignTest;
static bool HeldPlaceActive,ProcessedStoreActive,FinalAlignmentRetry,StationAlignRaised;
static bool EmptyPlaceAlignActive,PrealignedPlaceActive;
static struct {bool valid,picked;uint8_t ring;GrabScene_t scene;float x,z,release_z;} AlignedPlace;
static uint8_t HeldMaterialColor;
static bool ResumePending;
static GrabTask_State_t ResumeState;
static unsigned ResumePrepare;
static uint32_t ResumeElapsed;
static const char *ResumeReason,*ResumeResult;
static bool CalibrationTask, CalibrationReferenceConfirmed;
/* Stacking compensation is already included in the Pi B2 errors. */
static bool StackTest;
static float ManualZTarget;
static float RingReleaseZ, RingReturnZ;
static uint8_t RingNumber;
static float RingAlignZ;
static bool RingLowerSent, RingRaise, RingStayLow;
static bool RingApproachStarted;
static uint32_t RingApproachSequence, RingLowerTick;
/* One static profile for color/ring/pick; dynamic tracking keeps its own RAM settings.
 * Preserve the proven ring speeds as the shared static defaults. */
static float align_parameter(unsigned key)
{
  return Config[key];
}
/* Every B2 target supplies signed errors, including real zero. The camera
 * owns its per-target crosshair; CX/CY are only for field-of-view diagnostics. */
static int alignment_dx(const Camera_DataTypeDef *data)
{ return data->DX; }
static int alignment_dy(const Camera_DataTypeDef *data)
{ return data->DY; }
static Camera_ColorTypeDef target_color(void)
{ return RingTest ? (Camera_ColorTypeDef)(0x09U+RingNumber) : ActiveColor; }
static HAL_StatusTypeDef request_target(void)
{ return RingTest ? Camera_RingStart(RingNumber) : Camera_MaterialStart(ActiveColor); }
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
  w[2]=align_parameter(C_X_WEIGHT);
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
    s0=(-align_parameter(C_LAMBDA))*(d*dx-b*dy)/det;
    s1=(-align_parameter(C_LAMBDA))*(a*dy-b*dx)/det;
    for(i=0;i<3;i++) result[i]=w[i]*(j[0][i]*s0+j[1][i]*s1);
    if(pass==0)
    {
      float room=result[2]>=0 ? Config[C_X_MAX]-x : x-Config[C_X_MIN];
      w[2]=align_parameter(C_X_WEIGHT)*clamp(room/fmaxf(align_parameter(C_LEAD),span*.1f),0,1);
    }
  }
  if(hypotf(result[0],result[1])>align_parameter(C_BODY_SPEED))
    scale=align_parameter(C_BODY_SPEED)/hypotf(result[0],result[1]);
  if(fabsf(result[2])*scale>align_parameter(C_X_SPEED)) scale=align_parameter(C_X_SPEED)/fabsf(result[2]);
  for(i=0;i<3;i++) result[i]*=scale;
  return true;
}

/* 2026-09-29 bounded live probes at the home observation pose:
 * forward body command increases CX; left body command decreases CY.
 * At Base home the camera faces body-left; X extension moves it toward the
 * material in the same direction as left translation (user-confirmed geometry).
 * Camera errors are positive right/up; both left and extension increase DY.
 * 2 px/mm remains an initial control scale, NOT a measured camera calibration. */
static bool VisualModelCustom;
static void defaults_restore(unsigned phase)
{
  unsigned i;
  for(i=0;i<C_COUNT;i++) if(Parameters[i].phase & phase) Config[i]=Parameters[i].initial;
}
static void visual_defaults_restore(void)
{
  defaults_restore(DEFAULT_VISUAL);
  VisualModelCustom=false;
}
static const char *visual_model_name(void)
{
  unsigned i;
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
    if((op!=OP_RETURN && op!=OP_CAR_PICK && key==C_RELEASE_MS) ||
       (op==OP_CAR_PICK && key==C_CAR_PICK_WAIT_MS))return true;
    if(key==C_X_TRANSFER_SPEED || key==C_Z_CAR_LIFT || (op!=OP_RETURN && (key==C_CLOSE_MS ||
       (op!=OP_CAR_PICK && (key==C_Z_PLACE || key==C_BASE_CAR_OFFSET))))) return true;
    if(op==OP_STORE && (key==(scene==SCENE_PROC?C_Z_PROC_PLACE:C_Z_GRAB) ||
       key==(scene==SCENE_PROC?C_Z_CAR_LIFT:C_Z_LIFT))) return true;
    if(op==OP_TAKE && key==(scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE)) return true;
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
    unsigned pickup=scene==SCENE_PROC?C_Z_PROC_PLACE:C_Z_GRAB;
    unsigned lifted=scene==SCENE_PROC?C_Z_CAR_LIFT:C_Z_LIFT;
    unsigned released=scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE;
    if(Config[C_Z_CAR_LIFT]<Config[C_Z_MIN] || Config[C_Z_CAR_LIFT]>Config[C_Z_MAX]) return "z_car_lift";
    if(op!=OP_RETURN && op!=OP_CAR_PICK && (Config[C_Z_PLACE]<Config[C_Z_MIN] || Config[C_Z_PLACE]>Config[C_Z_MAX] ||
       Config[C_Z_CAR_LIFT]<=Config[C_Z_PLACE])) return "z_car_targets";
    if(op==OP_STORE && (Config[pickup]<Config[C_Z_MIN] || Config[pickup]>Config[C_Z_MAX] ||
       Config[lifted]<Config[C_Z_MIN] || Config[lifted]>Config[C_Z_MAX] || Config[lifted]<=Config[pickup])) return "z_targets";
    if(op==OP_TAKE && (Config[released]<Config[C_Z_MIN] || Config[released]>Config[C_Z_MAX])) return Keys[released];
  }
  if(fmaxf(fabsf(Config[C_Z_MIN]),fabsf(Config[C_Z_MAX]))*Config[C_Z_PPM]>2147483000.0f) return "z_pulses";
  if(fmaxf(fabsf(Config[C_X_MIN]),fabsf(Config[C_X_MAX]))*Config[C_X_PPM]>2147483000.0f) return "x_pulses";
  if(Config[C_Z_SPEED]*Config[C_Z_PPM]*60/3200<1 || Config[C_Z_SPEED]*Config[C_Z_PPM]*60/3200>3000) return "z_rpm";
  if(op==OP_STORE && (!isfinite(Config[C_Z_PICK_SPEED]) ||
     Config[C_Z_PICK_SPEED]*Config[C_Z_PPM]*60/3200<1 ||
     Config[C_Z_PICK_SPEED]*Config[C_Z_PPM]*60/3200>3000)) return "z_pick_speed";
  if(Config[C_X_SPEED]*Config[C_X_PPM]*60/3200<1 || Config[C_X_SPEED]*Config[C_X_PPM]*60/3200>3000) return "x_rpm";
  if(op!=OP_ALIGN && (Config[C_X_TRANSFER_SPEED]*Config[C_X_PPM]*60/3200<1 ||
     Config[C_X_TRANSFER_SPEED]*Config[C_X_PPM]*60/3200>3000)) return "x_transfer_rpm";
  if(op!=OP_RETURN && op!=OP_CAR_PLACE && op!=OP_CAR_PICK && OutsideReferenceValid && !OutsidePose && !RearPose) return "outside_pose";
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
  NextTrack.pending=false;QrDeparting=false;NextRawColor=0;NextRawRequested=false;
  TrackCloseStopping=TrackCloseZeroRequested=TrackCloseXRequested=false;
  memset(&ProcClearance,0,sizeof ProcClearance);
  StoreFinishTravel=false;
  TrackStoreActive=TrackStoreReleased=TrackStoreRecorded=TrackHeld=false;TrackStoreSlot=0;
  CarPickActive=CarPickClosed=CarPickWaiting=false;CarPickSlot=CarPickColor=0;
  EmptyPlaceAlignActive=PrealignedPlaceActive=AlignedPlace.valid=false;
  BootActive=BootStopping=false;
  BootBaseDegraded=BootAcceptBase=BaseHomeAcceptAvailable=false;
  ManualBaseHomePending=BootBaseOnly=BootResumePose=false;
  RecoveryUsed=RecoveryCount=0; RecoveryWaiting=RecoveryRestartPending=RecoveryPassive=false;
  LossActive=false; LossTick=LossLastMs=LossMaxMs=GraceCount=GraceSequence=0;
  for(i=0;i<C_COUNT;i++) Config[i]=NAN;
  defaults_restore(DEFAULT_INIT);
  visual_defaults_restore();
  memset(&Status,0,sizeof Status); memset(Axes,0,sizeof Axes);
  Status.mode="fixed"; Status.reason="idle"; Status.missing="z_ppm";
  Status.reference_cause="boot_pending";
  RingTest=RingPlaceActive=RingLowerSent=RingRaise=RingApproachStarted=false;
  StaticAlignTest=false;HeldPlaceActive=ProcessedStoreActive=ResumePending=FinalAlignmentRetry=StationAlignRaised=false;
  CalibrationTask=CalibrationReferenceConfirmed=false;GrabCalibration_Init();
  StackTest=false;
  RingStayLow=false;
  RingApproachSequence=RingLowerTick=0;
  Operation=OP_STORE; Scene=SCENE_RAW; Status.operation="store"; Status.scene="raw";
  Status.result="none"; Status.slot=0; OutsideReferenceValid=false; OutsidePose=true;
  ToCar=IndexSubmitted=InventoryTouched=false;
  RearReferenceValid=RearPose=false;RearBase=0;
  ActiveColor=GRAB_PICK_COLOR;
  Status.x_mm=Status.x_target_mm=Status.z_mm=Status.z_target_mm=NAN;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  HaveCapture=false; LastReport=HAL_GetTick(); ReportPending=false;
  GrabTracking_Init(&Tracking); GrabTracking_Defaults(&TrackConfig);
  TrackMode=GRAB_TRACK_PREDICT;
  for(unsigned i=0;i<2;i++)GrabTracking_DefaultsForMode(&TrackComparison[i],(GrabTracking_Mode_t)(i+1U));
  ChassisObserver_Init(&TrackObserver,0,0,0);
  memset(TrackWheel,0,sizeof TrackWheel); memset(&TrackPositionVelocity,0,sizeof TrackPositionVelocity);
  TrackCommandCount=0; TrackCommandApplied=0; TrackCommandActive=false;
  memset(TrackCommandUnitsS,0,sizeof TrackCommandUnitsS);
  memset(&TrackInput,0,sizeof TrackInput); memset(&TrackOutput,0,sizeof TrackOutput);
  TrackReady=TrackMoving=TrackVisualPause=TrackTraceSending=TrackPulseValid=TrackXZeroValid=false;
  TrackAutoStart=TrackPickArmed=TrackPickLowering=TrackEntryHaveSample=false;TrackEntryBudget=50;
  TrackZPollTick=0;TrackZPollState=false;
  TrackPickHoldActive=false;TrackPickHoldStart=TrackPickHoldLast=0;
  TrackObserverTick=TrackLastBatch=TrackBatchInterval=0;
  TrackSlowPoll=TrackTraceIndex=TrackIoSlot=0; memset(&TrackFeedbackFault,0,sizeof TrackFeedbackFault); memset(&TrackMotorFault,0,sizeof TrackMotorFault); TrackXZero=TrackPulseRemainder=0;
  memset(&TrackXDispatch,0,sizeof TrackXDispatch);
  TrackLogEnabled=TrackLogHaveTick=false;
  TrackLogId=TrackLogIndex=TrackLogTick=0;TrackLogPeriod=50;
  ConsoleTx_TrackingCancel();
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
static void arrival_reset(unsigned i)
{ Axes[i].arrival_waiting=Axes[i].arrival_reported=Axes[i].arrival_accepted=Axes[i].target_reused=false; }
static void stop_begin(void)
{
  for(unsigned i=0;i<3;i++)arrival_reset(i);
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
  if(Status.state==GRAB_STOPPING) {
    if(ProcClearance.active && (ProcClearance.submitted || ProcClearance.verified)) {
      ProcClearance.submitted=false;ProcClearance.recheck=true;
    }
    IndexSubmitted=false;Turntable_Stop();
  }
}
static void fault(const char *reason)
{
  TrackCloseStopping=false;
  NextRawColor=0;NextRawRequested=false;
  NextTrack.pending=false;
  if(Status.state==GRAB_STOPPING || Status.state==GRAB_ERROR) return;
  MotorBus_Diagnostic_t bus_fault;MotorBus_DiagnosticGet(&bus_fault);
  bool transport=!strcmp(reason,"motor_feedback") || !strcmp(reason,"motor_request") ||
    !strcmp(reason,"motor_ack_timeout") || !strcmp(reason,"chassis_tx") ||
    !strcmp(reason,"feedback_stale") || !strcmp(reason,"transport_error") ||
    !strcmp(reason,"bus_locked") || !strcmp(reason,"turntable_feedback_stale");
  /* Keep stage/inventory before the asynchronous error destroys the result.
   * Only transport faults are resumable here; real drive protections still fault. */
  if(transport && !CalibrationTask && !BootActive && MotorBus_TransientFault(&bus_fault) && GrabTask_Pause())
  {Status.reason="bus_recovery_wait";ReportPending=true;return;}
  if(CalibrationTask) GrabCalibration_Cancel();
  loss_finish(HAL_GetTick());
  RecoveryWaiting=RecoveryRestartPending=RecoveryPassive=false; RecoveryCount=0;
  invalidate_inventory(); Status.result="failed";
  Status.reason=reason; FinishError=true; Camera_RequestStop(); HaveCapture=FreshVision=false;
  enter(GRAB_STOPPING); stop_begin();
}
void GrabTask_Stop(void)
{
  TrackCloseStopping=false;
  NextTrack.pending=false;
  BaseHomeAcceptAvailable=false;
  if(BootActive) { boot_stop("home_cancelled",false); return; }
  if(CalibrationTask) GrabCalibration_Cancel();
  if(Status.state==GRAB_IDLE || Status.state==GRAB_COMPLETE) return;
  loss_finish(HAL_GetTick());
  RecoveryWaiting=RecoveryRestartPending=RecoveryPassive=false; RecoveryCount=0;
  /* Repeated console stops must not reset fresh standstill evidence or resend
   * stops ahead of the feedback needed to finish the existing cancellation. */
  if(Status.state==GRAB_STOPPING)
  {
    if(FinishError) { Status.reason="cancelled"; FinishError=false; ReportPending=true; }
    return;
  }
  if(!ResumePending)invalidate_inventory();
  Status.result="cancelled";
  Status.reason="cancelled"; FinishError=false;
  Camera_RequestStop(); HaveCapture=FreshVision=false;
  enter(GRAB_STOPPING); stop_begin();
}
void GrabTask_ReferenceInvalidate(MechanicalArm_AxisTypeDef axis)
{
  AlignedPlace.valid=false;
  Config[C_HEAD_PROC_GAIN]=Config[C_HEAD_TEMP_GAIN]=NAN;
  Config[C_HEAD_LINE_GAIN]=NAN;
  unsigned i;
  if(GrabTask_IsBusy()) fault("reference_changed");
  OutsideReferenceValid=false; OutsidePose=true;
  BaseHomeAcceptAvailable=false; /* Any changed origin invalidates the saved X/Z proof. */
  if(axis==MECHANICAL_ARM_AXIS_ALL)Status.base_home_degraded=false;
  RearPose=false;
  if(axis==MECHANICAL_ARM_AXIS_BASE) {
    RearReferenceValid=false;ManualBaseHomePending=false;
    Status.base_home_degraded=false;BaseHomeAcceptAvailable=false;
  }
  Tracking.reference_valid=false; Tracking.samples=Tracking.motions=0; Tracking.have_sample=false;
  Status.reference_cause=axis==MECHANICAL_ARM_AXIS_X?"x_reference_changed":
    axis==MECHANICAL_ARM_AXIS_Z?"z_reference_changed":
    axis==MECHANICAL_ARM_AXIS_BASE?"base_reference_changed":"all_reference_changed";
  if(axis==MECHANICAL_ARM_AXIS_X || axis==MECHANICAL_ARM_AXIS_ALL)
    Config[C_X_MIN]=Config[C_X_MAX]=Config[C_X_PRE]=NAN;
  if(axis==MECHANICAL_ARM_AXIS_Z || axis==MECHANICAL_ARM_AXIS_ALL)
  {
    Config[C_Z_MIN]=Config[C_Z_MAX]=Config[C_Z_OBSERVE]=Config[C_Z_GRAB]=Config[C_Z_LIFT]=Config[C_Z_PLACE]=NAN;
    Config[C_Z_PROC_PLACE]=Config[C_Z_STACK_PLACE]=NAN;
    Config[C_Z_CAR_LIFT]=Config[C_Z_ROUGH_ALIGN]=Config[C_Z_TEMP_ALIGN]=NAN;
  }
  if(axis==MECHANICAL_ARM_AXIS_BASE)
  {
    for(i=C_J00;i<=C_J12;i++) Config[i]=NAN;
  }
  Status.missing=configuration_missing(Status.mode);
  if(!Status.missing) Status.missing="none";
}
static bool start_operation(const char *mode,Camera_ColorTypeDef color,GrabOperation_t op,GrabScene_t scene,uint8_t slot)
{
  HWT101_Angle_t angle; Mecanum_Status_t bus; const char *missing;
  if(GrabTask_IsBusy()) {Status.reason="busy";return false;}
  memset(&ProcClearance,0,sizeof ProcClearance);
  CalibrationTask=false;HeldPlaceActive=ProcessedStoreActive=ResumePending=FinalAlignmentRetry=StationAlignRaised=false;
  EmptyPlaceAlignActive=PrealignedPlaceActive=false;
  Mecanum_TrackingTiming(false,0); TrackMoving=false;
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
  if((op==OP_CAR_PLACE || op==OP_CAR_PICK) && !RearReferenceValid)
  { Status.reason="rear_reference_missing"; Status.missing="rear_reference"; return false; }
  missing=configuration_missing(mode);
  if(missing) { Status.reason="config_missing"; Status.missing=missing; return false; }
  if(op==OP_STORE || op==OP_TAKE || op==OP_CAR_PICK)
  {
    Turntable_Inventory_t inventory;
    if(!slot && op==OP_STORE) slot=Turntable_FindEmpty();
    Status.slot=slot;
    if(!slot || slot>3)
    { Status.reason=op==OP_STORE?"no_confirmed_empty_slot":"invalid_slot"; Status.missing="slot"; return false; }
    if(!Turntable_ReferenceValid() || !Turntable_SlotConfigured(slot))
    { Status.reason="turntable_reference_missing"; Status.missing="turntable_reference_or_angle"; return false; }
    inventory=Turntable_InventoryGet(slot);
    bool resume=op==OP_CAR_PICK && CarPickActive && slot==CarPickSlot;
    /* An explicit storage tray is the operator's choice of an available tray.
     * Automatic storage still resolves only an EMPTY tray via FindEmpty above. */
    if(op==OP_TAKE && inventory.state==TURNTABLE_UNKNOWN)
    { Status.reason="inventory_unconfirmed"; Status.missing="slot_inventory"; return false; }
    if(op==OP_TAKE && inventory.state!=TURNTABLE_OCCUPIED)
    { Status.reason="slot_empty"; Status.missing="slot_inventory"; return false; }
    if(op==OP_TAKE)ActiveColor=(Camera_ColorTypeDef)inventory.color;
    /* Manual work pickup trusts the selected physical tray, not the stock ledger.
     * Keep a known color if available; zero represents unspecified material. */
    if(op==OP_CAR_PICK)ActiveColor=(Camera_ColorTypeDef)(resume?CarPickColor:
      inventory.state==TURNTABLE_OCCUPIED?inventory.color:0);
  }
  if(ChassisRoute_IsBusy() ||
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
  RingTest=RingPlaceActive=RingLowerSent=RingRaise=RingApproachStarted=false;
  StaticAlignTest=false;
  StackTest=false;
  RingStayLow=false;
  RingApproachSequence=RingLowerTick=0;
  InventoryTouched=IndexSubmitted=false; ToCar=op!=OP_RETURN;
  StoreFinishTravel=false;
  memset(Axes,0,sizeof Axes); memset(WheelLastPositionTick,0,sizeof WheelLastPositionTick);
  Status.x_mm=Status.x_target_mm=Status.z_mm=Status.z_target_mm=NAN;
  RecoveryUsed=RecoveryCount=0; RecoveryWaiting=RecoveryRestartPending=RecoveryPassive=false;
  LossActive=false; LossTick=LossLastMs=LossMaxMs=GraceCount=GraceSequence=0;
  PrepareStep=PollIndex=Retries=0; HaveCapture=FreshVision=WheelBaseline=FeedbackSelected=false;
  PendingAction=MECHANICAL_ARM_ACTION_NONE; StartTick=HAL_GetTick();
  LastControl=LastPoll=StartTick; LastSequence=LastInvalidCount=0; WheelGroupTick=0;
  enter(GRAB_PREPARE); stop_begin(); stable_reset();
  if(op!=OP_CAR_PLACE){TrackStoreActive=TrackStoreReleased=TrackStoreRecorded=TrackHeld=false;TrackStoreSlot=0;}
  if(op!=OP_CAR_PICK)CarPickActive=CarPickClosed=CarPickWaiting=false;
  return true;
}
bool GrabTask_AlignStart(uint8_t target,float z,bool stay)
{
  bool ring=target>=10 && target<=12;
  if((!ring && (target<1 || target>6)) || !isfinite(z))
  { Status.reason="invalid_align_target_or_height";return false; }
  if(isfinite(Config[C_Z_MIN]) && isfinite(Config[C_Z_MAX]) &&
     (z<Config[C_Z_MIN] || z>Config[C_Z_MAX]))
  { Status.reason="align_height_outside_limits";return false; }
  if(!start_operation("align",ring?CAMERA_COLOR_BLACK:(Camera_ColorTypeDef)target,OP_ALIGN,SCENE_RAW,0))return false;
  StaticAlignTest=true;RingTest=ring;RingNumber=ring?(uint8_t)(target-9):0;
  RingAlignZ=z;RingStayLow=stay;
  Status.operation=ring?"ring_align":"color_align";Status.scene=ring?"rough":"none";
  return true;
}
static bool station_height(unsigned key)
{
  if(!isfinite(Config[key])){Status.reason="config_missing";Status.missing=Keys[key];return false;}
  if(Config[key]<Config[C_Z_MIN] || Config[key]>Config[C_Z_MAX]){Status.reason="config_missing";Status.missing=Keys[key];return false;}
  return true;
}
static bool station_alignment_height(unsigned key,bool raised,float *height)
{
  if(!raised && !station_height(key))return false;
  *height=raised?0:Config[key];
  if(*height<Config[C_Z_MIN] || *height>Config[C_Z_MAX])
  {Status.reason="align_height_outside_limits";return false;}
  return true;
}
bool GrabTask_HeldPlaceStartEx(const char *scene,uint8_t target,uint8_t material_color,bool raised)
{
  bool rough=scene && !strcmp(scene,"rough"),stack=scene && !strcmp(scene,"temp2");
  if(!scene || (!rough && !stack && strcmp(scene,"temp1")) || material_color<1 || material_color>6 ||
     target<1 || target>(stack?6:3)){Status.reason="invalid_place_target";return false;}
  unsigned align=rough?C_Z_ROUGH_ALIGN:C_Z_TEMP_ALIGN;
  unsigned release=stack?C_Z_STACK_PLACE:C_Z_PROC_PLACE;
  float align_z;
  if(!station_alignment_height(align,raised,&align_z) || !station_height(release))return false;
  if(Config[release]>align_z){Status.reason="config_missing";Status.missing=Keys[release];return false;}
  if(!GrabTask_AlignStart(stack?target:(uint8_t)(target+9),align_z,true))return false;
  StationAlignRaised=raised;
  HeldPlaceActive=true;HeldMaterialColor=material_color;TrackHeld=true;
  StackTest=stack;Scene=rough?SCENE_ROUGH:stack?SCENE_TEMP2:SCENE_TEMP1;
  RingReleaseZ=Config[release];RingReturnZ=Config[C_Z_MAX];
  Status.operation="held_place";Status.scene=rough?"rough":stack?"temp2":"temp1";Status.reason="held_alignment";
  return true;
}
bool GrabTask_AlignPlaceStartEx(const char *scene,uint8_t target,bool raised)
{
  bool rough=scene && !strcmp(scene,"rough"),stack=scene && !strcmp(scene,"temp2");
  if(!scene || (!rough && !stack && strcmp(scene,"temp1")) || target<1 || target>(stack?6:3))return false;
  unsigned release=stack?C_Z_STACK_PLACE:C_Z_PROC_PLACE;
  float align_z;
  if(!station_alignment_height(rough?C_Z_ROUGH_ALIGN:C_Z_TEMP_ALIGN,raised,&align_z) || !station_height(release))return false;
  if(Config[release]>align_z)
  {Status.reason="config_missing";Status.missing=Keys[release];return false;}
  if(TrackHeld){Status.reason="empty_claw_required";return false;}
  if(!GrabTask_AlignStart(stack?target:(uint8_t)(target+9),align_z,true))return false;
  StationAlignRaised=raised;StackTest=stack;
  AlignedPlace.valid=false;EmptyPlaceAlignActive=true;
  Scene=rough?SCENE_ROUGH:stack?SCENE_TEMP2:SCENE_TEMP1;RingReleaseZ=Config[release];
  Status.operation=stack?"empty_material_align":"empty_ring_align";Status.scene=SceneNames[Scene];
  Status.reason=stack?"empty_material_align":"empty_ring_align";
  return true;
}
bool GrabTask_AlignedPlaceStart(const char *scene,uint8_t target,uint8_t material_color)
{
  if(!scene || (strcmp(scene,"rough") && strcmp(scene,"temp1") && strcmp(scene,"temp2")) || !AlignedPlace.valid || !AlignedPlace.picked ||
     strcmp(scene,SceneNames[AlignedPlace.scene]) ||
     AlignedPlace.ring!=target || !TrackHeld)
  {Status.reason="aligned_place_reference_missing";return false;}
  if(!station_height(AlignedPlace.scene==SCENE_TEMP2?C_Z_STACK_PLACE:C_Z_PROC_PLACE))return false;
  if(!isfinite(AlignedPlace.x) || !isfinite(AlignedPlace.z) || AlignedPlace.x<Config[C_X_MIN] ||
     AlignedPlace.x>Config[C_X_MAX] || AlignedPlace.z<Config[C_Z_MIN] ||
     AlignedPlace.z>Config[C_Z_MAX] || !isfinite(AlignedPlace.release_z) ||
     AlignedPlace.release_z<Config[C_Z_MIN] || AlignedPlace.release_z>AlignedPlace.z)
  {Status.reason="aligned_place_targets";return false;}
  if(material_color>6)return false;
  /* Fixed placement does not request a color. Preserve an unknown selected-tray
   * material as zero after admitting the shared fixed-action API. */
  if(!start_operation("fixed",material_color?(Camera_ColorTypeDef)material_color:CAMERA_COLOR_BLACK,OP_ALIGN,AlignedPlace.scene,0))return false;
  ActiveColor=(Camera_ColorTypeDef)material_color;
  PrealignedPlaceActive=HeldPlaceActive=TrackHeld=true;
  HeldMaterialColor=material_color;ExternalX=AlignedPlace.x;
  /* Restore the saved horizontal position, then descend in one complete move. */
  RingPlaceActive=true;RingAlignZ=AlignedPlace.z;RingReleaseZ=AlignedPlace.release_z;
  RingReturnZ=Config[C_Z_MAX];
  Status.operation="aligned_place";Status.scene=SceneNames[Scene];Status.reason="restore_aligned_position";
  return true;
}
bool GrabTask_ProcessedStoreStartEx(uint8_t color,uint8_t ring,uint8_t slot,bool raised)
{
  if(ring<1 || ring>3){Status.reason="invalid_ring";return false;}
  float align_z;
  if(!station_alignment_height(C_Z_ROUGH_ALIGN,raised,&align_z) || !station_height(C_Z_PROC_PLACE))return false;
  if(Config[C_Z_PROC_PLACE]>align_z)
  {Status.reason="config_missing";Status.missing="z_proc_place";return false;}
  if(!start_operation("pick",(Camera_ColorTypeDef)color,OP_STORE,SCENE_PROC,slot))return false;
  ProcessedStoreActive=StaticAlignTest=true;
  if(Status.slot==3 && Turntable_SlotAngleGet(3,&ProcClearance.return_angle)) {
    ProcClearance.active=true;
    ProcClearance.outside_angle=fmodf(ProcClearance.return_angle+90.0f,360.0f);
    ProcClearance.lift_z=Config[C_Z_CAR_LIFT];
  }
  StationAlignRaised=raised;
  /* Align material DX/DY at the processing observation plane. Once settled,
   * commit the remaining vertical pickup travel without image corrections. */
  RingAlignZ=align_z;RingStayLow=true;
  /* Fully open before visual acquisition so the fingers do not hide material.
   * Keep the narrower opening exclusive to pickup inside the car trays. */
  Arm_GripperDutySet((float)ARM_GRIPPER_OPEN_PULSE_US/200.0f);
  Status.reason="processed_alignment";return true;
}
bool GrabTask_HeldPlaceStart(const char *scene,uint8_t target,uint8_t material_color)
{return GrabTask_HeldPlaceStartEx(scene,target,material_color,true);}
bool GrabTask_AlignPlaceStart(const char *scene,uint8_t ring)
{return GrabTask_AlignPlaceStartEx(scene,ring,true);}
bool GrabTask_ProcessedStoreStart(uint8_t color,uint8_t ring,uint8_t slot)
{return GrabTask_ProcessedStoreStartEx(color,ring,slot,true);}
bool GrabTask_Pause(void)
{
  if(ResumePending){if(Status.state==GRAB_ERROR)GrabTask_Stop();return true;}
  if(!GrabTask_IsBusy() || Status.state==GRAB_STOPPING || Status.state==GRAB_HOMING)return false;
  ResumeState=Status.state;ResumePrepare=PrepareStep;ResumeElapsed=HAL_GetTick()-StateTick;
  ResumeReason=Status.reason;ResumeResult=Status.result;ResumePending=true;
  GrabTask_Stop();return true;
}
void GrabTask_Cancel(void)
{
  NextRawColor=0;NextRawRequested=false;
  AlignedPlace.valid=false;
  ProcClearance.active=false;
  ResumePending=false;
  GrabTask_Stop();
  /* End pickup ownership without changing the claw output or held-material fact. */
  if(CarPickClosed)TrackHeld=true;
  CarPickActive=CarPickClosed=CarPickWaiting=false;
}
bool GrabTask_BusRecoveryVerify(void)
{
  if(!ResumePending || GrabTask_IsBusy() || MechanicalArm_IsBusy())return false;
  FinishError=false;Camera_RequestStop();HaveCapture=FreshVision=false;
  Status.reason="bus_recovery_verify";enter(GRAB_STOPPING);stop_begin();return true;
}
bool GrabTask_Continue(void)
{
  Mecanum_Status_t bus;Mecanum_StatusGet(&bus);
  if(!ResumePending || Status.state!=GRAB_IDLE || !Status.stop_confirmed || bus.locked || bus.stop_pending ||
     MechanicalArm_IsBusy() || Turntable_IsBusy() || ChassisMotion_IsBusy())return false;
  if(!RearReferenceValid){Status.reason="rear_reference_missing";return false;}
  /* An operator may inspect the car for minutes. IDLE stops axis polling;
   * rebuild stop evidence before restoring a stage with motion/servo effects. */
  uint32_t now=HAL_GetTick();bool fresh=true;
  for(unsigned i=0;i<3;i++)
    if(!Axes[i].position_valid || !Axes[i].state_valid ||
       now-Axes[i].position_tick>Config[C_FEEDBACK_MS] || now-Axes[i].state_tick>Config[C_FEEDBACK_MS])fresh=false;
  for(unsigned i=0;i<4;i++)
  {
    Mecanum_Feedback_t f;
    if(!Mecanum_FeedbackGet(i,&f) || !f.position_valid || !f.speed_valid || !f.state_valid ||
       now-f.position_ms>Config[C_FEEDBACK_MS] || now-f.speed_ms>Config[C_FEEDBACK_MS] || now-f.state_ms>Config[C_FEEDBACK_MS])fresh=false;
  }
  if(!fresh){FinishError=false;Status.reason="resume_feedback_wait";enter(GRAB_STOPPING);stop_begin();return false;}
  GrabTask_State_t state=ResumeState;
  Status.reason=ResumeReason;Status.result=ResumeResult;
  HaveCapture=FreshVision=false;stable_reset();
  for(unsigned i=0;i<3;i++)Axes[i].commanded=false;
  if(state==GRAB_ROUTE_POSE){PoseStep=0;PoseEvidenceTick=now;}
  if(state==GRAB_INDEX)IndexSubmitted=false;
  if(state==GRAB_QR_SCAN){if(QrZStep==1)QrZStep=0;else if(QrZStep==3)QrZStep=2;}
  if(state==GRAB_PREPARE)
  {PrepareStep=ResumePrepare;if(PrepareStep==2 || PrepareStep==4 || PrepareStep==6 || PrepareStep==8)PrepareStep--;else if(PrepareStep==9)PrepareStep=7;}
  if(tracking_mode() && (state==GRAB_TRACK_OBSERVE || state==GRAB_TRACK_MATCH || state==GRAB_TRACK_FOLLOW || state==GRAB_DESCEND || state==GRAB_SETTLE || state==GRAB_TRACK_PAUSE))
  {
    if(request_target()!=HAL_OK){ResumePending=true;return false;}
    TrackReady=false;StartTick=HAL_GetTick();TrackPulseValid=false;
    TrackMoving=!TrackAutoStart;state=TrackAutoStart?GRAB_TRACK_OBSERVE:TrackPickLowering?GRAB_DESCEND:GRAB_TRACK_MATCH;
    Mecanum_TrackingTiming(TrackMoving,track_config(TrackMode)->batch_ms);
  }
  else if(!fixed_mode() && (state==GRAB_ACQUIRE || state==GRAB_ALIGN || state==GRAB_SETTLE || state==GRAB_VISION_PAUSE || state==GRAB_VISION_GRACE || (EmptyPlaceAlignActive && state==GRAB_HOLD) || (HeldPlaceActive && !StationAlignRaised && state==GRAB_DESCEND)))
  {
    if(request_target()!=HAL_OK){ResumePending=true;return false;}
    RequestTick=HAL_GetTick();LastSequence=0;
    if(state!=GRAB_DESCEND && state!=GRAB_RELEASE && state!=GRAB_RETRACT)state=GRAB_ACQUIRE;
  }
  ResumePending=false;Status.stop_requested=Status.stop_confirmed=false;
  enter(state);if(state==GRAB_CLOSE || state==GRAB_RELEASE)StateTick-=ResumeElapsed;
  return true;
}
bool GrabTask_CalibrationReferenceValid(void)
{ return CalibrationReferenceConfirmed; }
bool GrabTask_CalibrationActive(void)
{ return CalibrationTask && GrabTask_IsBusy(); }
bool GrabTask_CalibrationReferenceSet(void)
{
  if(GrabTask_IsBusy() || MechanicalArm_IsBusy() || ChassisRoute_IsBusy() ||
     ChassisMotion_IsBusy() || Mecanum_IsBusy() || Turntable_IsBusy()) return false;
  CalibrationReferenceConfirmed=true;
  /* Manual confirmation restores only a missing observation model, never an
   * existing measured/custom model. Mechanical scales/bounds remain untouched. */
  if(!isfinite(Config[C_J00])) visual_defaults_restore();
  Status.reference_cause="observation_pose_confirmed";
  return true;
}
bool GrabTask_VisualResponseSet(float fdx,float fdy,float xdx,float xdy)
{
  float degree_per_mm=Config[C_X_PPM]*360.0f/3200.0f;
  float j02=xdx*degree_per_mm,j12=xdy*degree_per_mm;
  if(!isfinite(fdx) || !isfinite(fdy) || !isfinite(j02) || !isfinite(j12) ||
     degree_per_mm<=0 || fabsf(fdx)>10000 || fabsf(fdy)>10000 ||
     fabsf(j02)>10000 || fabsf(j12)>10000 ||
     fabsf(fdx*j12-fdy*j02)<.0001f) return false;
  Config[C_J00]=fdx;Config[C_J10]=fdy;Config[C_J02]=j02;Config[C_J12]=j12;
  VisualModelCustom=true;
  return true;
}
bool GrabTask_CalibrationStart(uint8_t color)
{
  if(!GrabTask_CalibrationReferenceValid()) { Status.reason="ref_required";return false; }
  if(!start_operation("align",(Camera_ColorTypeDef)color,OP_ALIGN,SCENE_RAW,0)) return false;
  CalibrationTask=true;
  Status.operation="calibration";
  /* The confirmed observation pose is already in place. PREPARE only verifies
   * standstill/feedback; it does not open the claw or reposition any arm axis. */
  return true;
}
static bool start_with_color(const char *mode,Camera_ColorTypeDef color)
{
  if(!strcmp(mode,"align"))return GrabTask_AlignStart((uint8_t)color,Config[C_Z_OBSERVE],true);
  return start_operation(mode,color,OP_STORE,SCENE_RAW,0);
}
bool GrabTask_Start(const char *mode)
{ return start_with_color(mode,default_color(mode)); }

bool GrabTask_TransferStart(const char *operation,const char *scene,uint8_t slot,uint8_t color)
{
  if(!operation || !scene) return false;
  if(!strcmp(operation,"store") && (!strcmp(scene,"raw") || !strcmp(scene,"proc")))
    return start_operation(!strcmp(scene,"raw")?"pick":"fixed",(Camera_ColorTypeDef)color,
      OP_STORE,!strcmp(scene,"raw")?SCENE_RAW:SCENE_PROC,slot);
  if(!strcmp(operation,"take"))
  {
    GrabScene_t target;
    if(!strcmp(scene,"rough")) target=SCENE_ROUGH;
    else if(!strcmp(scene,"temp1")) target=SCENE_TEMP1;
    else if(!strcmp(scene,"temp2")) target=SCENE_TEMP2;
    else return false;
    return start_operation("fixed",(Camera_ColorTypeDef)color,OP_TAKE,target,slot);
  }
  return false;
}

static bool held_feedback(unsigned i,uint32_t now);
static bool pose_start(bool work,bool qr)
{
  unsigned i;
  uint32_t now=HAL_GetTick();
  bool reuse=Status.state==GRAB_COMPLETE && !Status.stop_requested;
  Mecanum_Status_t bus;
  if(GrabTask_IsBusy() || MechanicalArm_IsBusy() || Turntable_IsBusy() ||
     HWT101_Cal_IsBusy())
  { Status.reason="busy";return false; }
  if(!RearReferenceValid && !ManualBaseHomePending) { Status.reason="rear_reference_missing";return false; }
  for(i=C_X_PPM;i<=C_Z_MAX;i++) if(!isfinite(Config[i]))
  { Status.reason="config_missing";Status.missing=Keys[i];return false; }
  if(!isfinite(Config[C_Z_CAR_LIFT]) || Config[C_Z_CAR_LIFT]<Config[C_Z_MIN] ||
     Config[C_Z_CAR_LIFT]>Config[C_Z_MAX]) { Status.reason="config_missing";Status.missing="z_car_lift";return false; }
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stop_pending || bus.stage==MECANUM_STAGE_FAULT)
  { Status.reason="bus_unavailable";return false; }
  /* Concurrent work preparation is Base-only from a certified retracted rear
   * pose. A travel preparation itself waits for the chassis to finish. */
  if(ChassisMotion_IsBusy() && ((!work && !qr) || !RearPose))
  { Status.reason="busy";return false; }
  if(!RearReferenceValid) { base_reference_verify_start(work);return true; }
  for(i=0;i<3;i++)if(!held_feedback(i,now))reuse=false;
  PoseQr=qr;PoseWork=work;PoseStep=0;PollIndex=0;
  CalibrationTask=false;
  Status.mode="fixed";Status.operation="pose";Status.scene="none";
  Status.result="pending";Status.reason=work?"preparing_work":"preparing_travel";
  Status.missing="none";
  Status.stop_requested=false;Status.stop_confirmed=false;Status.slot=0;
  if(reuse) {
    for(i=0;i<3;i++){arrival_reset(i);Axes[i].commanded=false;}
  } else memset(Axes,0,sizeof Axes);
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  StartTick=LastPoll=now;OutsideBase=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
  /* Completed owned actions may hand over their still-fresh stationary evidence.
   * Paused/stale actions must acquire feedback after this new preparation. */
  PoseEvidenceTick=reuse?now-(uint32_t)Config[C_FEEDBACK_MS]-1U:now;
  OutsideReferenceValid=true;enter(GRAB_ROUTE_POSE);
  return true;
}

bool GrabTask_PoseStart(bool work){return pose_start(work,false);}
bool GrabTask_QrPoseStart(void)
{
  if(!RearReferenceValid) { Status.reason="rear_reference_missing";return false; }
  if(!pose_start(false,true))return false;
  PoseQr=true;Status.reason="preparing_qr_pose";
  return true;
}

static bool result_accept(MechanicalArm_ResultTypeDef result, MechanicalArm_ActionTypeDef action,
                          MechanicalArm_AxisTypeDef axis)
{
  if(result==MECHANICAL_ARM_RESULT_BUSY) return false;
  if(result!=MECHANICAL_ARM_RESULT_NONE && result!=MECHANICAL_ARM_RESULT_OK)
  { fault(result==MECHANICAL_ARM_RESULT_ACK_TIMEOUT?"motor_ack_timeout":"motor_request"); return false; }
  PendingAction=action; PendingAxis=axis; return true;
}
/* Feedback may lie just outside a mechanical endpoint. Normalize only
 * measured return positions within the existing tolerance, never commands. */
static float observation_target(unsigned i,float measured)
{
  float lo=Config[i==0?C_X_MIN:C_Z_MIN],hi=Config[i==0?C_X_MAX:C_Z_MAX];
  float tolerance=Config[C_POS_TOL];
  if(isfinite(measured) && isfinite(lo) && isfinite(hi) && lo<=hi &&
     isfinite(tolerance) && tolerance>=0 && measured>=lo-tolerance && measured<=hi+tolerance)
    return fminf(hi,fmaxf(lo,measured));
  return measured; /* Real violations still reach the existing limit checks. */
}
static bool axis_move(unsigned i,float target)
{
  MechanicalArm_AxisTypeDef axis=i==0?MECHANICAL_ARM_AXIS_X:MECHANICAL_ARM_AXIS_Z;
  MechanicalArm_ConfigTypeDef config;
  float ppm=Config[i==0?C_X_PPM:C_Z_PPM], speed=i==0?align_parameter(C_X_SPEED):Config[C_Z_SPEED];
  /* Fast material descent is shared by formal store and tracking pickup;
   * numbered-ring placement and observation keep their original Z speed. */
  if(i==1 && Status.state==GRAB_DESCEND && !RingPlaceActive &&
     (Operation==OP_STORE || Operation==OP_TRACK_PICK)) speed=Config[C_Z_PICK_SPEED];
  /* Visual correction keeps x_speed; all fixed transfers and pose changes
   * share x_transfer_speed, including the standalone car-place test. */
  if(i==0 && Status.state!=GRAB_ALIGN && Status.state!=GRAB_VISION_GRACE) speed=Config[C_X_TRANSFER_SPEED];
  if(i==1 && Status.state==GRAB_QR_SCAN) speed=Config[C_QR_Z_SPEED];
  float minimum=Config[i==0?C_X_MIN:C_Z_MIN],maximum=Config[i==0?C_X_MAX:C_Z_MAX];
  if(!isfinite(speed) || speed*ppm*60/3200<1 || speed*ppm*60/3200>3000)
  { fault("axis_rpm");return false; }
  uint16_t rpm=(uint16_t)floorf(speed*ppm*60/3200);
  int32_t pulses;
  if(target<minimum || target>maximum || !isfinite(target))
  {
    char line[160];snprintf(line,sizeof line,"ERR grab axis_limit axis=%s target=%.4f min=%.4f max=%.4f current=%.4f\r\n",
      i==0?"x":"z",target,minimum,maximum,Axes[i].mm);reply(line);
    fault("axis_limit");return false;
  }
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy()) return false;
  pulses=(int32_t)lroundf(target*ppm);
  target=(float)pulses/ppm;
  if((Status.state==GRAB_PREPARE || Status.state==GRAB_LIFT || Status.state==GRAB_RETRACT ||
      Status.state==GRAB_X_RETRACT || Status.state==GRAB_CAR_EXTEND || Status.state==GRAB_EXTERNAL_EXTEND ||
      Status.state==GRAB_OBSERVE_Z || Status.state==GRAB_OBSERVE_X || Status.state==GRAB_ROUTE_POSE) &&
     held_feedback(i,HAL_GetTick()) && fabsf(Axes[i].mm-target)<=Config[C_POS_TOL] &&
     ((Axes[i].flags&2U) || (i==1 && target==Config[C_Z_MAX]) || Axes[i].arrival_accepted)) {
    arrival_reset(i);Axes[i].target=target;Axes[i].command_tick=HAL_GetTick();
    Axes[i].commanded=Axes[i].target_reused=true;
    if(i==0)Status.x_target_mm=target;else Status.z_target_mm=target;
    return true;
  }
  if(!MechanicalArm_ConfigGet(axis,&config)) { fault("axis_config"); return false; }
  uint32_t accepted_ms=HAL_GetTick();
  if(!result_accept(MechanicalArm_PositionEx(axis,pulses,rpm,config.Acceleration,
    MECHANICAL_ARM_POSITION_ABSOLUTE),MECHANICAL_ARM_ACTION_POSITION,axis)) return false;
  target=(float)pulses/ppm;
  arrival_reset(i);
  Axes[i].target=target; Axes[i].command_tick=accepted_ms; Axes[i].commanded=true;
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
  /* Use the measured multi-turn absolute anchor. On this car FD mode 2
   * returned +90 for a -90 -> 0 request; avoid that firmware interpretation. */
  pulses=(int32_t)lroundf(target*3200.0f/360.0f);
  uint32_t accepted_ms=HAL_GetTick();
  if(!result_accept(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_BASE,pulses,
    config.SpeedRpm,config.Acceleration,MECHANICAL_ARM_POSITION_ABSOLUTE),
    MECHANICAL_ARM_ACTION_POSITION,MECHANICAL_ARM_AXIS_BASE)) return false;
  arrival_reset(2);
  Axes[2].target=(float)pulses*360.0f/3200.0f; Axes[2].command_tick=accepted_ms;
  Axes[2].commanded=true; Axes[2].quiet=false; OutsidePose=RearPose=false;
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
static bool held_feedback(unsigned i,uint32_t now)
{
  return axis_quiet(i,now,now-(uint32_t)Config[C_FEEDBACK_MS]-1U) &&
    (Axes[i].flags&0x0dU)==1U && isfinite(Axes[i].mm);
}
/* Fallback for missing driver reached, never for missing communication.
 * Each axis owns its timer; only new position AND state samples advance it. */
static bool axis_stable_arrival(unsigned i,uint32_t now,uint32_t hold_ms,float tolerance)
{
  GrabAxis_t *a=&Axes[i];
  if(!a->commanded || !axis_fresh(i,now) || !a->quiet ||
     (int32_t)(a->position_tick-a->command_tick)<=0 || (int32_t)(a->state_tick-a->command_tick)<=0 ||
     (a->flags&0x0dU)!=1U || !isfinite(a->mm) || !isfinite(a->target)) {
    a->arrival_waiting=a->arrival_accepted=false;return false;
  }
  if(a->arrival_waiting &&
     ((uint32_t)(a->position_tick-a->arrival_position_tick)>(uint32_t)Config[C_FEEDBACK_MS] ||
      (uint32_t)(a->state_tick-a->arrival_state_tick)>(uint32_t)Config[C_FEEDBACK_MS]))
    a->arrival_waiting=false;
  /* A tenth of the position tolerance bounds total drift, rather than allowing
   * a slow creep to pass by accumulating tiny adjacent differences. */
  if(!a->arrival_waiting || fabsf(a->mm-a->arrival_anchor)>tolerance*.1f) {
    a->arrival_waiting=true;a->arrival_accepted=false;a->arrival_anchor=a->mm;
    a->arrival_since=now;
    a->arrival_position_tick=a->position_tick;a->arrival_state_tick=a->state_tick;
    return false;
  }
  if(a->position_tick==a->arrival_position_tick || a->state_tick==a->arrival_state_tick)return a->arrival_accepted && fabsf(a->mm-a->target)<=tolerance;
  a->arrival_position_tick=a->position_tick;a->arrival_state_tick=a->state_tick;
  /* Both observed timestamps must span the hold interval; wall time with an
   * unchanged cached position is not evidence of standstill. */
  if((int32_t)(a->position_tick-a->arrival_since)<(int32_t)hold_ms ||
     (int32_t)(a->state_tick-a->arrival_since)<(int32_t)hold_ms ||
     fabsf(a->mm-a->target)>tolerance)return false;
  a->arrival_accepted=true;
  if(!a->arrival_reported) {
    char line[160];
    (void)snprintf(line,sizeof line,"OK grab arrival axis=%s source=stable_feedback error=%.4f tolerance=%.4f hold_ms=%lu\r\n",
      i==0?"x":i==1?"z":"base",(double)fabsf(a->mm-a->target),(double)tolerance,(unsigned long)hold_ms);
    reply(line);a->arrival_reported=true;
  }
  return true;
}
static bool arrived(unsigned i,uint32_t now)
{
  float tolerance=i==2?GRAB_BASE_POSITION_TOL_DEG:Config[C_POS_TOL];
  const uint8_t ids[]={MOTOR_ID_ARM_X,MOTOR_ID_ARM_Z,MOTOR_ID_ARM_BASE};
  if(Axes[i].commanded && !Axes[i].target_reused && axis_fresh(i,now) &&
     (Axes[i].flags&0x09U)==1U &&
     MotorBus_PositionReached(ids[i],Axes[i].command_tick))return true;
  if(Axes[i].commanded && Axes[i].target_reused) {
    if(!held_feedback(i,now))return false;
    if(fabsf(Axes[i].mm-Axes[i].target)<=tolerance)return true;
    /* A held axis that drifts away needs the original absolute target submitted. */
    Axes[i].target_reused=Axes[i].commanded=false;return false;
  }
  if(!Axes[i].commanded || !axis_fresh(i,now) ||
     (Axes[i].flags&0x0dU)!=1U) {
    Axes[i].arrival_waiting=false;return false;
  }
  /* A missing 9F uses measured rest over arrival_hold_ms; Receive/02 alone
   * cannot finish a movement and normal completion has no added dwell. */
  return axis_stable_arrival(i,now,(uint32_t)Config[C_ARRIVAL_HOLD_MS],tolerance);
}
/* A stopped coarse turn can leave encoder error beyond the temporary Base business
 * tolerance. Reissue the same absolute target at low speed without changing
 * the rear anchor/model. This car's mode 2 has returned to the small correction
 * value in live tests, so owned Base turns must never use it. */
static bool base_arrival(uint32_t now)
{
  MechanicalArm_ConfigTypeDef config;
  if(arrived(2,now))return true;
  if(!Axes[2].commanded || !axis_quiet(2,now,Axes[2].command_tick) ||
     (Axes[2].flags&0x0dU)!=1U || PendingAction!=MECHANICAL_ARM_ACTION_NONE ||
     MechanicalArm_IsBusy() || !MechanicalArm_ConfigGet(MECHANICAL_ARM_AXIS_BASE,&config))return false;
  float error=Axes[2].target-Axes[2].mm;
  if(fabsf(error)<=GRAB_BASE_POSITION_TOL_DEG || fabsf(error)>5)return false; /* Fine correction, not a second coarse turn. */
  int32_t pulses=(int32_t)lroundf(Axes[2].target*3200.f/360.f);
  if(result_accept(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_BASE,pulses,
     config.SpeedRpm>15?15:config.SpeedRpm,config.Acceleration,
     MECHANICAL_ARM_POSITION_ABSOLUTE),MECHANICAL_ARM_ACTION_POSITION,MECHANICAL_ARM_AXIS_BASE))
  { arrival_reset(2);Axes[2].command_tick=now;Axes[2].quiet=false; }
  return false;
}
static void poll_axes(uint32_t now)
{
  unsigned i; MechanicalArm_AxisTypeDef axis; MechanicalArm_ActionTypeDef action;
  if(((Status.state==GRAB_PREPARE && PrepareStep==0) || Status.state==GRAB_SETTLE ||
      Status.state==GRAB_VISION_PAUSE || Status.state==GRAB_STOPPING) && !StopEvidenceStarted) return;
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy())return;
  if(now-LastPoll<25)return;
  i=(PollIndex/2)%3; axis=i==2?MECHANICAL_ARM_AXIS_BASE:(i==0?MECHANICAL_ARM_AXIS_X:MECHANICAL_ARM_AXIS_Z);
  action=(PollIndex%2)==0?MECHANICAL_ARM_ACTION_POSITION_READ:MECHANICAL_ARM_ACTION_STATE;
  if(result_accept(action==MECHANICAL_ARM_ACTION_POSITION_READ?MechanicalArm_PositionRead(axis):
    MechanicalArm_StateRead(axis),action,axis)) { PollIndex=(PollIndex+1)%6; LastPoll=now; }
}
uint8_t GrabTask_MotorEventHandle(const MechanicalArm_EventTypeDef *event)
{
  unsigned i; GrabAxis_t *axis; uint32_t now=HAL_GetTick();
  if(!event) return 0;
  /* Passive evidence survives ordinary stop/start. A queued target alone is
   * not a sent command; failed ACK after completed TX remains visible. */
  if(event->Axis==MECHANICAL_ARM_AXIS_X && event->Transmitted &&
     (event->Action==MECHANICAL_ARM_ACTION_POSITION || event->Action==MECHANICAL_ARM_ACTION_STOP))
  {
    TrackXDispatch.sent_ms=event->TxDoneMs;TrackXDispatch.action=(uint8_t)event->Action;
    TrackXDispatch.sent_pulses=event->PositionPulses;
    TrackXDispatch.sent_rpm=event->Action==MECHANICAL_ARM_ACTION_POSITION?event->SpeedRpm:0;
    TrackXDispatch.transmitted=true;TrackXDispatch.acknowledged=event->Acknowledged!=0;
  }
  if(BootActive) return boot_event(event);
  if(Status.state==GRAB_CALIBRATION && GrabCalibration_MotorEventHandle(event)) return 1;
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
      {
        ManualBaseHomePending=event->Axis==MECHANICAL_ARM_AXIS_BASE && event->HomeMode==0 &&
          event->Result==MECHANICAL_ARM_RESULT_OK;
        Status.reference_cause=event->Axis==MECHANICAL_ARM_AXIS_ALL?"manual_home_all":
          event->Axis==MECHANICAL_ARM_AXIS_X?"manual_home_x":
          event->Axis==MECHANICAL_ARM_AXIS_Z?"manual_home_z":"manual_home_base";
      }
      else if(event->Action==MECHANICAL_ARM_ACTION_ZERO)
        Status.reference_cause=event->Axis==MECHANICAL_ARM_AXIS_ALL?"manual_zero_all":
          event->Axis==MECHANICAL_ARM_AXIS_X?"manual_zero_x":
          event->Axis==MECHANICAL_ARM_AXIS_Z?"manual_zero_z":"manual_zero_base";
      else if(event->Action==MECHANICAL_ARM_ACTION_POSITION && event->Axis==MECHANICAL_ARM_AXIS_BASE)
        Status.reference_cause="manual_base_move";
      else if((event->Action==MECHANICAL_ARM_ACTION_STOP || event->Action==MECHANICAL_ARM_ACTION_HOME_ABORT) &&
              (event->Axis==MECHANICAL_ARM_AXIS_BASE || event->Axis==MECHANICAL_ARM_AXIS_ALL))
        ManualBaseHomePending=false;
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
      axis->signed_speed=isfinite(ppm)?(float)(raw-axis->raw)*3200/65536/ppm*1000/(now-axis->position_tick):0;
      if(axis->speed<=(i==2?1.0f:Config[C_STOP_SPEED]))
      { if(!axis->quiet) axis->quiet_since=now; axis->quiet=true; }
      else axis->quiet=false;
    }
    if(!axis->quiet || !axis->position_valid || now-axis->position_tick>(uint32_t)Config[C_FEEDBACK_MS])
      axis->arrival_waiting=axis->arrival_accepted=false;
    axis->raw=raw; axis->position_tick=now; axis->position_valid=true;
    axis->mm=isfinite(ppm)?(float)raw*3200/65536/ppm:NAN;
    if(i==0) Status.x_mm=axis->mm; else if(i==1) Status.z_mm=axis->mm;
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_STATE)
  {
    /* X42 0x3A bit2 is instantaneous stall; bit3 is timed stall protection.
     * Keep the raw flags, but only protection or disabled state aborts motion. */
    if(!axis->state_valid || now-axis->state_tick>(uint32_t)Config[C_FEEDBACK_MS] ||
       (event->StateFlags[event->Axis]&0x0dU)!=1U)
      axis->arrival_waiting=axis->arrival_accepted=false;
    axis->flags=event->StateFlags[event->Axis]; axis->state_tick=now; axis->state_valid=true;
    if((axis->flags&0x08U)!=0 || !(axis->flags&1U))
    {
      static const uint8_t ids[3]={MOTOR_ID_ARM_X,MOTOR_ID_ARM_Z,MOTOR_ID_ARM_BASE};
      track_motor_fault(ids[i],"axis",axis->flags,now,now);fault("axis_disabled_or_stall");
    }
  }
  return 1;
}

/* Every wheel must have fresh speed, position and enabled/non-protected state.
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
    if((f.state_flags&0x08U) || !(f.state_flags&1U))
    { track_motor_fault((uint8_t)(i+1),"wheel",f.state_flags,f.state_ms,now);WheelQuiet=false;fault("wheel_disabled_or_stall");return false; }
    if(!WheelBaseline) WheelOrigin[i]=f.position_raw;
    /* A past travel violation must not block fresh standstill evidence. Keep all
     * speed/position/state checks below active while completing the stop. */
    if(Status.state!=GRAB_STOPPING && !fixed_mode() && !tracking_mode() && Config[C_TRAVEL]>0 &&
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
    MechanicalArm_AxisTypeDef axis=(emergency || (tracking_mode() && TrackPickLowering))?
      MECHANICAL_ARM_AXIS_ALL:MECHANICAL_ARM_AXIS_X;
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
static void vision_pause_begin(bool keep_stream)
{
  GraceCount=0;
  RecoveryWaiting=false; RecoveryCount=0; HaveCapture=FreshVision=false;
  /* Alignment can reacquire through the existing B2 command, without changing Pi code. */
  RecoveryPassive=keep_stream;RecoveryRestartPending=!keep_stream;RecoveryValidTick=Vision.Tick;
  Status.reason="vision_pause_stopping"; enter(GRAB_VISION_PAUSE); stop_begin();
}
static bool vision_near_window(int dx,int dy)
{ return abs(dx)<=2*align_parameter(C_DX_TOL) && abs(dy)<=2*align_parameter(C_DY_TOL); }
static void vision_loss_begin(uint32_t now)
{
  if(!LossActive) { LossActive=true; LossTick=Vision.Tick; }
  stable_reset(); FreshVision=false;
  /* Alignment in both align and pick can decelerate through a brief gap.
   * Settle and near-window motion still stop; old data never confirms arrival. */
  if(Status.state!=GRAB_ALIGN ||
     Config[C_LOSS_GRACE_MS]<=0 || now-LossTick>=Config[C_LOSS_GRACE_MS] ||
     vision_near_window(Status.dx,Status.dy)) {
    bool keep_stream=Config[C_LOSS_GRACE_MS]>0 && now-LossTick<Config[C_LOSS_GRACE_MS] &&
      (Status.state==GRAB_SETTLE || vision_near_window(Status.dx,Status.dy));
    vision_pause_begin(keep_stream); return;
  }
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
  { vision_pause_begin(false); return; }
  if(s.InvalidCount!=LastInvalidCount) { GraceCount=0; LastInvalidCount=s.InvalidCount; }
  valid=s.HasValidData && s.TargetValid && s.Data.Function==0xB2 &&
    s.Data.Target==target_color() && (int32_t)(s.Data.Tick-LossTick)>0 &&
    (int32_t)(s.Data.Tick-RequestTick)>=0 && now-s.Data.Tick<=Config[C_AGE_MS];
  if(!valid) GraceCount=0;
  else if(s.Data.Sequence!=GraceSequence)
  {
    GraceSequence=s.Data.Sequence;
    dx=alignment_dx(&s.Data);dy=alignment_dy(&s.Data);
    /* A new near-window or reversed error makes the previous direction unsafe
     * to reuse even before three observations have confirmed visual recovery. */
    if(vision_near_window(dx,dy) ||
       (dx*(int)Status.dx<0 && abs(dx)>align_parameter(C_DX_TOL) && abs(Status.dx)>align_parameter(C_DX_TOL)) ||
       (dy*(int)Status.dy<0 && abs(dy)>align_parameter(C_DY_TOL) && abs(Status.dy)>align_parameter(C_DY_TOL)))
    { vision_pause_begin(vision_near_window(dx,dy)); return; }
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
  if(RecoveryPassive && s.HasValidData && s.TargetValid && s.Data.Function==0xB2 &&
     s.Data.Target==target_color() && (int32_t)(s.Data.Tick-RequestTick)>=0)
    RecoveryValidTick=s.Data.Tick;
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
  /* Brief near-target misses keep the live stream while physically stopped.
   * Only a sustained absence of valid coordinates restarts recognition. */
  if(RecoveryPassive && (uint32_t)(now-RecoveryValidTick)>=Config[C_LOSS_GRACE_MS])
  {RecoveryPassive=false;RecoveryRestartPending=true;}
  if(RecoveryRestartPending)
  {
    HAL_StatusTypeDef result;
    if(!Status.stop_confirmed) return;
    result=request_target();
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
  RecoveryUsed++; RecoveryWaiting=RecoveryPassive=false; RecoveryCount=0; loss_finish(now);
  LastSequence=s.Data.Sequence; Vision=s.Data; HaveCapture=true; FreshVision=false;
  Status.dx=(int16_t)alignment_dx(&Vision);
  Status.dy=(int16_t)alignment_dy(&Vision);
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
  Status.dx=(int16_t)alignment_dx(&Vision);
  Status.dy=(int16_t)alignment_dy(&Vision);
  return true;
}
/* All stages consume camera-supplied signed errors without MCU offsets. */
static int align_error_x(void)
{ return Status.dx; }
static int align_error_y(void)
{ return Status.dy; }
static bool in_window(void)
{ return abs(align_error_x())<=align_parameter(C_DX_TOL) && abs(align_error_y())<=align_parameter(C_DY_TOL); }
static void visual_stability(uint32_t now);
bool GrabTask_AlignPlaceCapture(void)
{
  uint32_t now=HAL_GetTick();
  if(!EmptyPlaceAlignActive || Status.state!=GRAB_HOLD || strcmp(Status.result,"aligned"))return false;
  /* The parent runs before GrabTask_Process: refresh wheel evidence here too. */
  if(!wheels(now) || !Status.stop_confirmed || !WheelQuiet || now-WheelQuietTick<Config[C_STABLE_MS] ||
     !axis_quiet(0,now,StopTick) || !axis_quiet(1,now,StopTick) ||
     !axis_quiet(2,now,StopTick)){stable_reset();return false;}
  for(unsigned i=0;i<3;i++)if((Axes[i].flags&0x0dU)!=1U){stable_reset();return false;}
  if(!vision_read(now) || !FreshVision)return false;
  if(!in_window())
  {
    Status.result="pending";Status.reason=StackTest?"empty_material_realign":"empty_ring_realign";
    Status.stop_requested=Status.stop_confirmed=false;stable_reset();enter(GRAB_ALIGN);return false;
  }
  visual_stability(now);
  if(!VisualStable)return false;
  AlignedPlace.x=observation_target(0,Axes[0].mm);
  AlignedPlace.z=observation_target(1,Axes[1].mm);
  AlignedPlace.scene=Scene;AlignedPlace.release_z=RingReleaseZ;
  AlignedPlace.ring=StackTest?(uint8_t)ActiveColor:RingNumber;AlignedPlace.valid=true;AlignedPlace.picked=false;
  EmptyPlaceAlignActive=false;Camera_RequestStop();
  Status.reason="aligned_position_saved";enter(GRAB_COMPLETE);return true;
}
static void visual_stability(uint32_t now)
{
  if(!FreshVision) return;
  if(!in_window() || !Axes[0].quiet || Axes[0].speed>Config[C_STOP_SPEED] || !WheelQuiet)
  { stable_reset(); return; }
  if(CaptureCount==0) StableTick=now;
  CaptureCount++;
  VisualStable=CaptureCount>=Config[C_FRAMES] && now-StableTick>=Config[C_STABLE_MS];
}
/* Static alignment and prediction tracking share the same physical output:
 * software profiles are already computed, so no second task/ramp is inserted. */
static bool visual_velocity_request(float forward,float left,float heading,float current)
{
  float error=fmodf(heading-current+540,360)-180;
  float yaw=clamp(error*CHASSIS_HEADING_TEST_KP,
    -CHASSIS_HEADING_TEST_OUTPUT_LIMIT,CHASSIS_HEADING_TEST_OUTPUT_LIMIT);
  return Mecanum_Velocity_Request(forward,left,yaw,CHASSIS_VELOCITY_ACCELERATION);
}
static void align_control(uint32_t now)
{
  float v[3], dt, delta, gain=1, xnext; unsigned i; HWT101_Angle_t angle;
  Mecanum_Status_t bus;
  bool grace=Status.state==GRAB_VISION_GRACE;
  if(now-LastControl<GRAB_CONTROL_PERIOD_MS || (!FreshVision && !grace)) return;
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stage==MECANUM_STAGE_FAULT) { fault("chassis_tx"); return; }
  /* Shared static controller follows the ACK-paced service without a second batch delay. */
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy() ||
    bus.stage==MECANUM_STAGE_WHEELS || bus.stage==MECANUM_STAGE_SYNC)
  { LastControl=now; return; }
  dt=fminf((now-LastControl)/1000.0f,.2f); LastControl=now;
  if(!HWT101_Cal_ControlAngleGet(&angle) || !isfinite(angle.yaw) || now-angle.last_update_ms>HWT101_DATA_FRESH_MS)
  { fault("imu_invalid"); return; }
  if(grace)
  {
    float speed=sqrtf(PreviousVelocity[0]*PreviousVelocity[0]+
      PreviousVelocity[1]*PreviousVelocity[1]+PreviousVelocity[2]*PreviousVelocity[2]);
    float scale=speed>0?fmaxf(0,1-align_parameter(C_ACCEL)*dt/speed):0;
    for(i=0;i<3;i++) v[i]=PreviousVelocity[i]*scale;
  }
  else
  {
    if(!solve((float)align_error_x(),(float)align_error_y(),Axes[0].mm,v)) { fault("matrix_rank"); return; }
    if(in_window()) v[0]=v[1]=v[2]=0;
  }
  delta=sqrtf((v[0]-PreviousVelocity[0])*(v[0]-PreviousVelocity[0])+
    (v[1]-PreviousVelocity[1])*(v[1]-PreviousVelocity[1])+
    (v[2]-PreviousVelocity[2])*(v[2]-PreviousVelocity[2]));
  if(delta>align_parameter(C_ACCEL)*dt) gain=align_parameter(C_ACCEL)*dt/delta;
  for(i=0;i<3;i++) v[i]=PreviousVelocity[i]+gain*(v[i]-PreviousVelocity[i]);
  xnext=clamp(XIntegral+v[2]*dt,Config[C_X_MIN],Config[C_X_MAX]);
  xnext=clamp(xnext,Axes[0].mm-align_parameter(C_LEAD),Axes[0].mm+align_parameter(C_LEAD));
  if(!visual_velocity_request(v[0],v[1],Heading,angle.yaw*CHASSIS_HEADING_TEST_YAW_SIGN))
  { Mecanum_Status_t bus; Mecanum_StatusGet(&bus); if(bus.locked || bus.stage==MECANUM_STAGE_FAULT) fault("chassis_tx"); return; }
  Status.forward_mm_s=v[0]; Status.left_mm_s=v[1]; memcpy(PreviousVelocity,v,sizeof v);
  if(StaticAlignTest && !grace && !RingApproachStarted)
  { RingApproachStarted=true; RingApproachSequence=Vision.Sequence; }
  /* Accumulate fractional travel only on free control opportunities, bounded by
   * real feedback. A rejected position request rebases to the last accepted target. */
  XIntegral=xnext;
  /* Compare executable pulses, not a floating half-pulse residual: resending
   * the same pulse target continually clears quiet evidence and prevents HOLD. */
  if(lroundf(xnext*Config[C_X_PPM])!=lroundf(Axes[0].target*Config[C_X_PPM]) && !axis_move(0,xnext))
    XIntegral=Axes[0].target;
}

/* The current Pi image is 640x480. Hold height while the ring is in its outer
 * 10%; in the central area, approach progress permits progressively lower
 * targets. Small feedback-bounded segments leave room to hold Z on a new edge
 * observation rather than finishing an entire descent after losing the ring. */
static void static_lower_control(uint32_t now)
{
  float xspan,yspan,remaining,progress,allowed,current,target;
  const float left=64,right=576,top=48,bottom=432,lead=5;
  if(!RingApproachStarted || !FreshVision || Vision.Sequence==RingApproachSequence ||
     now-RingLowerTick<GRAB_CONTROL_PERIOD_MS || !axis_fresh(1,now)) return;
  /* The empty claw can approach the known processing-ring plane continuously.
   * Keep correcting XY while the drive executes one complete Z move; fresh
   * Z arrival and visual stability still gate completion. Stops clear commanded. */
  if(EmptyPlaceAlignActive || StationAlignRaised)
  {
    if(!Axes[1].commanded && axis_move(1,RingAlignZ))
    {RingLowerTick=now;RingLowerSent=true;}
    return;
  }
  current=clamp(Axes[1].mm,RingAlignZ,Config[C_Z_MAX]);
  target=current;
  if(Vision.CX>left && Vision.CX<right && Vision.CY>top && Vision.CY<bottom)
  {
    /* Infer the camera crosshair from this same packet only for FOV scaling.
     * Never substitute these coordinates for the reported control errors. */
    float center_x=(float)Vision.CX-Vision.DX,center_y=(float)Vision.CY+Vision.DY;
    xspan=Status.dx<0?center_x-left:right-center_x;
    yspan=Status.dy>0?center_y-top:bottom-center_y;
    if(xspan>0 && yspan>0)
    {
      remaining=fmaxf(abs(Status.dx)<=align_parameter(C_DX_TOL)?0:fabsf((float)Status.dx)/xspan,
                      abs(Status.dy)<=align_parameter(C_DY_TOL)?0:fabsf((float)Status.dy)/yspan);
      progress=clamp(1-remaining,0,1);
      allowed=Config[C_Z_MAX]+(RingAlignZ-Config[C_Z_MAX])*progress;
      target=clamp(fmaxf(allowed,current-lead),RingAlignZ,current);
    }
  }
  /* Re-aiming higher only holds the current height; raising belongs to the
   * completed alignment stage, with visual correction already stopped. */
  if(Axes[1].commanded && fabsf(target-Axes[1].target)<=.5f/Config[C_Z_PPM]) return;
  if(axis_move(1,target))
  {
    RingLowerTick=now;
    RingLowerSent=fabsf(Axes[1].target-RingAlignZ)<=.5f/Config[C_Z_PPM];
  }
}

static void enter_axis(GrabTask_State_t state,unsigned axis)
{ Axes[axis].commanded=false; enter(state); }
static const char *track_store_missing(uint8_t slot,bool resume)
{
  if(!RearReferenceValid)return "rear_reference_missing";
  if(!slot || slot>3)return "no_confirmed_empty_slot";
  if(!Turntable_ReferenceValid() || !Turntable_SlotConfigured(slot))return "turntable_reference_missing";
  /* Callers resolve slot 0 through FindEmpty; explicit 1..3 needs no separate
   * stock confirmation. Record the new contents only after release completes. */
  float lift=resume && TrackStoreActive?TrackStoreLiftTarget:Config[C_Z_LIFT];
  float place=resume && TrackStoreActive?TrackStorePlaceTarget:lift-Config[C_TRACK_STORE_DROP_MM];
  if(!isfinite(lift) || lift<Config[C_Z_MIN] || lift>Config[C_Z_MAX] ||
     !isfinite(place) || place<Config[C_Z_MIN] || place>=lift)return "track_store_z_targets";
  return configuration_missing_for("fixed",OP_CAR_PLACE,SCENE_RAW);
}
static bool track_store_start(uint8_t requested)
{
  uint8_t slot=requested?requested:(TrackStoreActive?TrackStoreSlot:Turntable_FindEmpty());
  if(GrabTask_IsBusy())return false;
  if(TrackStoreActive && slot!=TrackStoreSlot){Status.reason="track_store_slot_mismatch";return false;}
  if((!TrackHeld && !TrackStoreReleased) || !OutsideReferenceValid)
  {Status.reason="track_material_not_held";return false;}
  const char *missing=track_store_missing(slot,true);
  if(missing){Status.reason=missing;Status.missing=missing;return false;}
  /* Reuse the existing accepted stop/preparation chain without opening or
   * lowering for a second pickup. Keep the saved outside pose and color. */
  if(!start_operation("fixed",ActiveColor,OP_CAR_PLACE,SCENE_RAW,slot))return false;
  Operation=OP_STORE;Status.operation="store";Status.scene="raw";
  if(!TrackStoreActive)
  {
    TrackStoreLiftTarget=Config[C_Z_LIFT];
    TrackStorePlaceTarget=TrackStoreLiftTarget-Config[C_TRACK_STORE_DROP_MM];
  }
  TrackStoreActive=true;TrackStoreSlot=slot;
  ToCar=!TrackStoreReleased;Status.reason="track_store_prepare";
  return true;
}
static void transfer_complete(void)
{
  ProcClearance.active=false;
  OutsidePose=!StoreFinishTravel;
  if(StoreFinishTravel)RearPose=true;
  Status.reference_cause=StoreFinishTravel?"travel_pose_verified":"transfer_return_verified";
  Status.missing=configuration_missing(Status.mode);
  if(!Status.missing) Status.missing="none";
  Status.result=Operation==OP_STORE?"stored":(Operation==OP_TAKE?"released":"returned");
  Status.reason=Status.result; Status.stop_requested=false;
  if(TrackStoreActive){TrackStoreActive=TrackStoreReleased=TrackStoreRecorded=TrackHeld=false;}
  StoreFinishTravel=false;
  enter(GRAB_COMPLETE);
}
static bool car_pick_start(uint8_t slot,float depth)
{
  if(GrabTask_IsBusy())return false;
  if(CarPickActive && slot!=CarPickSlot){Status.reason="car_pick_slot_mismatch";return false;}
  float top=CarPickActive?CarPickLiftTarget:Config[C_Z_MAX];
  float target=CarPickActive?CarPickTarget:top-depth;
  /* Current match order picks trays 1,2,3. The first two need clearance from
   * the remaining materials; tray3 is last and keeps X retracted. */
  float clearance=CarPickActive?CarPickClearanceX:
    Config[C_X_MIN]+(slot<=2U?GRAB_CAR_PICK_CLEARANCE_MM:0.0f);
  if(!isfinite(depth) || depth<=0 || depth>150 || !isfinite(top) ||
     !isfinite(target) || target<Config[C_Z_MIN] || target>=top)
  {Status.reason="car_pick_z_targets";return false;}
  if(!isfinite(clearance) || !isfinite(Config[C_X_MIN]) || !isfinite(Config[C_X_MAX]) ||
     clearance<Config[C_X_MIN] || clearance>Config[C_X_MAX])
  {Status.reason="car_pick_clearance_limits";return false;}
  if(!start_operation("fixed",GRAB_PICK_COLOR,OP_CAR_PICK,SCENE_RAW,slot))return false;
  if(!CarPickActive)
  {
    CarPickLiftTarget=top;CarPickTarget=target;CarPickSlot=slot;
    CarPickClearanceX=clearance;
    CarPickColor=(uint8_t)ActiveColor;CarPickClosed=false;
  }
  CarPickActive=true;CarPickWaiting=false;
  Status.reason=CarPickClosed?"car_pick_continue":"car_pick_prepare";
  return true;
}
bool GrabTask_CarPickStart(uint8_t slot)
{ return car_pick_start(slot,Config[C_CAR_PICK_DROP_MM]); }
static void car_pick_complete(void)
{
  OutsideBase=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
  ObserveX=observation_target(0,Axes[0].mm);ObserveZ=observation_target(1,Axes[1].mm);
  OutsideReferenceValid=OutsidePose=true;RearPose=false;
  CarPickActive=CarPickWaiting=false;TrackHeld=true;
  if(AlignedPlace.valid)AlignedPlace.picked=true;
  Status.result="picked";Status.reason="car_pick_complete";
  Status.reference_cause="work_pose_verified";Status.missing="none";
  Status.stop_requested=false;enter(GRAB_COMPLETE);
}
/* Wait for the saved clearance/return target. A completed target is only
 * re-read after pause or stale feedback, so a low arm never causes a new turn. */
static bool processing_clearance_ready(uint32_t now)
{
  Turntable_Status_t tray;
  float target=ProcClearance.returning?ProcClearance.return_angle:ProcClearance.outside_angle;
  uint8_t slot=ProcClearance.returning?3U:0U;
  if(!ProcClearance.submitted) {
    if(Turntable_IsBusy())return false;
    if(!ProcClearance.verified) {
      if(!axis_quiet(1,now,0))return false;
      if(Axes[1].mm<ProcClearance.lift_z-Config[C_POS_TOL]) {
        if(!Axes[1].commanded) (void)axis_move(1,ProcClearance.lift_z);
        return false;
      }
      if(!ProcClearance.returning &&
         (!axis_quiet(2,now,0) || (Axes[2].flags&0x0dU)!=1U ||
          fabsf(Axes[2].mm-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))>GRAB_BASE_POSITION_TOL_DEG))return false;
    }
    if(!ProcClearance.verified) {
      Status.reason=ProcClearance.returning?"processed_clearance_return":"processed_clearance_out";
      ReportPending=true;
    }
    bool accepted=ProcClearance.verified?Turntable_AngleVerify(target,slot):
      Turntable_AngleStart(target,slot,!ProcClearance.returning);
    if(!accepted) {
      Turntable_StatusGet(&tray);
      if(strcmp(tray.reason,"busy") && strcmp(tray.reason,"bus_busy"))fault(tray.reason);
      return false;
    }
    ProcClearance.submitted=true;return false;
  }
  if(Turntable_IsBusy())return false;
  Turntable_StatusGet(&tray);
  if(!tray.arrived){fault(tray.reason);return false;}
  ProcClearance.verified=true;
  if(!tray.feedback_valid){ProcClearance.submitted=false;return false;}
  return !tray.moving && tray.slot==slot;
}
static bool transfer_preindex_enabled(void)
{ return Operation==OP_CAR_PICK || ProcessedStoreActive || TrackStoreActive; }
/* Raw pickup owns an established outside pose until a Base move invalidates
 * it. Precision arrival tolerance is not the tray's clearance boundary. */
static bool raw_preindex_ready(uint32_t now)
{
  return RearReferenceValid && OutsidePose && !RearPose &&
    axis_quiet(2,now,0) && (Axes[2].flags&0x0dU)==1U;
}
static void transfer_preindex(uint32_t now)
{
  if(ProcClearance.active)return;
  bool taking=Operation==OP_CAR_PICK && !CarPickClosed &&
    (Status.state==GRAB_RETRACT || Status.state==GRAB_X_RETRACT);
  bool storing=(ProcessedStoreActive || TrackStoreActive) && ToCar &&
    (Status.state==GRAB_LIFT || Status.state==GRAB_X_RETRACT);
  if(!taking && !storing)return;
  if(TrackStoreActive) { if(!raw_preindex_ready(now))return; }
  else if(!RearReferenceValid || !axis_quiet(2,now,0) ||
          (Axes[2].flags&0x0dU)!=1U ||
          fabsf(Axes[2].mm-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))>GRAB_BASE_POSITION_TOL_DEG)return;
  Turntable_Status_t tray;
  if(!IndexSubmitted)
  {
    if(Turntable_Index(Status.slot))IndexSubmitted=true;
    else {
      Turntable_StatusGet(&tray);
      if(strcmp(tray.reason,"busy") && strcmp(tray.reason,"bus_busy"))fault(tray.reason);
    }
  }
  else if(!Turntable_IsBusy())
  {
    Turntable_StatusGet(&tray);
    if(!tray.arrived)fault(tray.reason);
  }
}
static void transfer_process(uint32_t now)
{
  if(TrackStoreActive && TrackStoreReleased && TrackStoreRecorded && NextRawColor &&
     !StoreFinishTravel &&
     (Status.state==GRAB_RETURN_TURN || Status.state==GRAB_OBSERVE_Z || Status.state==GRAB_OBSERVE_X)) {
    Camera_SnapshotTypeDef next_camera;Camera_SnapshotGet(&next_camera);
    if((!NextRawRequested || !next_camera.RequestActive || next_camera.RequestTarget!=NextRawColor) &&
       Camera_MaterialStart((Camera_ColorTypeDef)NextRawColor)==HAL_OK)NextRawRequested=true;
  }
  /* With Base outside, ID8 can turn while Z lifts and X retracts. The INDEX
   * stage still joins these independent moves before Base may enter the car. */
  transfer_preindex(now);
  if(ProcClearance.active && ProcClearance.returning &&
     (Status.state==GRAB_X_RETRACT || Status.state==GRAB_TURN))
    (void)processing_clearance_ready(now);
  if(Status.state==GRAB_STOPPING || Status.state==GRAB_ERROR)return;
  if(Operation==OP_CAR_PICK && Status.state==GRAB_LIFT)
  {
    if(strcmp(Status.reason,"car_pick_lift_clearance"))
    {Status.reason="car_pick_lift_clearance";ReportPending=true;}
    /* UART requests drain one at a time; both independent drives run in this
     * same stage. Absolute saved targets avoid adding another 30 mm on resume. */
    if(!Axes[1].commanded && !axis_move(1,CarPickLiftTarget))return;
    if(!Axes[0].commanded || fabsf(Axes[0].target-CarPickClearanceX)>0.5f/Config[C_X_PPM])
      if(!axis_move(0,CarPickClearanceX))return;
    if(!arrived(1,now) || !arrived(0,now))return;
    if(!Turntable_InventorySet(Status.slot,TURNTABLE_EMPTY,0))
    {fault("inventory_record_failed");return;}
    InventoryTouched=false;ToCar=false;TrackHeld=true;
    /* Keep X extended through the Base turn to clear the other tray contents. */
    enter_axis(GRAB_RETURN_TURN,2);return;
  }
  if(RingRaise)
  {
    if(!Axes[1].commanded) axis_move(1,Config[C_Z_MAX]);
    else if(arrived(1,now))
    {
      RingRaise=false;Status.reason=StackTest?"stack_aligned_ready":(RingTest?"ring_aligned_ready":"color_aligned_ready");Status.result="aligned";
      enter(GRAB_HOLD);
    }
    return;
  }
  if(Status.state==GRAB_DESCEND || Status.state==GRAB_LIFT ||
     Status.state==GRAB_PLACE || Status.state==GRAB_RETRACT || Status.state==GRAB_OBSERVE_Z)
  {
    float target;
    if(Status.state==GRAB_DESCEND) target=FinalAlignmentRetry?RingAlignZ:RingPlaceActive?RingReleaseZ:Config[Operation==OP_TAKE?release_key():grab_key()];
    else if(Status.state==GRAB_LIFT) target=Operation==OP_CAR_PICK?CarPickLiftTarget:
      TrackStoreActive?fmaxf(Axes[1].mm,TrackStoreLiftTarget):ProcClearance.active?ProcClearance.lift_z:Config[Operation==OP_TAKE?C_Z_CAR_LIFT:lift_key()];
    else if(Status.state==GRAB_PLACE) target=Operation==OP_CAR_PICK?CarPickTarget:TrackStoreActive?TrackStorePlaceTarget:Config[C_Z_PLACE];
    else if(Status.state==GRAB_OBSERVE_Z) target=ObserveZ;
    else if(RingPlaceActive) target=RingReturnZ;
    else if(Operation==OP_TAKE && !ToCar) target=Config[C_Z_MAX]; /* Leave the external release vertically. */
    else if(Operation==OP_CAR_PICK) target=fmaxf(Axes[1].mm,CarPickLiftTarget);
    else if(Operation==OP_RETURN || (Operation==OP_TAKE && ToCar))
      target=fmaxf(Axes[1].mm,Config[C_Z_CAR_LIFT]);
    else if(StoreFinishTravel && Operation==OP_STORE && !ToCar)
      target=fmaxf(Axes[1].mm,Config[C_Z_CAR_LIFT]);
    else target=TrackStoreActive?TrackStoreLiftTarget:Config[C_Z_CAR_LIFT];
    if(!Axes[1].commanded)
    {
      if(axis_move(1,target) && Status.state==GRAB_PLACE && Operation!=OP_CAR_PLACE) InventoryTouched=true;
    }
    else if(arrived(1,now))
    {
      if(Status.state==GRAB_DESCEND)
      {
        if(FinalAlignmentRetry)
        {
          FinalAlignmentRetry=false;RingPlaceActive=false;RingLowerSent=RingApproachStarted=false;
          Axes[0].target=XIntegral=Axes[0].mm;memset(PreviousVelocity,0,sizeof PreviousVelocity);
          HaveCapture=FreshVision=false;Status.stop_requested=Status.stop_confirmed=false;
          stable_reset();RequestTick=LastControl=now;enter(GRAB_ACQUIRE);return;
        }
        if(HeldPlaceActive && !PrealignedPlaceActive && !StationAlignRaised)
        {
          if(!vision_read(now) || !FreshVision)
          {Status.reason="release_alignment_wait";return;}
          if(!in_window())
          {
            FinalAlignmentRetry=true;Axes[1].commanded=false;
            Status.reason="final_alignment_reacquire";return;
          }
        }
        if(HeldPlaceActive)
        {
          Arm_GripperSet(ARM_GRIPPER_OPEN);TrackHeld=false;enter(GRAB_RELEASE);
        }
        else if(Operation==OP_TAKE || RingPlaceActive) { Arm_GripperSet(ARM_GRIPPER_OPEN); enter(GRAB_RELEASE); }
        else
        {
          Arm_GripperSet(ARM_GRIPPER_CATCH);enter(GRAB_CLOSE);
        }
      }
      else if(Status.state==GRAB_LIFT)
      {
        Camera_RequestStop();
        if(Operation==OP_TRACK_PICK)
        {
          TrackPickArmed=false;Tracking.frozen=true;
          if(!TrackStoreSlot)
          {Status.result="picked";Status.reason="track_pick_complete";enter(GRAB_COMPLETE);return;}
          TrackStoreActive=true;TrackStoreReleased=TrackStoreRecorded=false;
          TrackStoreLiftTarget=Config[C_Z_LIFT];
          TrackStorePlaceTarget=TrackStoreLiftTarget-Config[C_TRACK_STORE_DROP_MM];
          Operation=OP_STORE;Status.mode="fixed";Status.operation="store";Status.scene="raw";
          Status.slot=TrackStoreSlot;Status.reason="track_store_retract";
          Heading=0;Mecanum_TrackingTiming(false,0);TrackMoving=false;
        }
        if(Operation==OP_TAKE)
        {
          if(!Turntable_InventorySet(Status.slot,TURNTABLE_EMPTY,0)) { fault("inventory_record_failed"); return; }
      InventoryTouched=false; ToCar=false;
        }
        else ToCar=true;
        if(ProcClearance.active) {
          /* This branch is reached only after fresh Z lift arrival. */
          ProcClearance.returning=true;
          ProcClearance.submitted=ProcClearance.verified=false;
        }
        enter_axis(GRAB_X_RETRACT,0);
      }
      else if(Status.state==GRAB_PLACE)
      {
        if(Operation==OP_TAKE || Operation==OP_CAR_PICK)
        {
          Arm_GripperSet(ARM_GRIPPER_CATCH);
          if(Operation==OP_CAR_PICK){CarPickClosed=true;CarPickCloseTick=now;}
          enter(GRAB_CLOSE);
        }
        else { Arm_GripperSet(ARM_GRIPPER_OPEN);if(TrackStoreActive){TrackHeld=false;TrackStoreReleased=true;TrackStoreReleaseTick=now;}enter(GRAB_RELEASE); }
      }
      else if(Status.state==GRAB_OBSERVE_Z) enter_axis(GRAB_OBSERVE_X,0);
      else if(RingPlaceActive)
      {
        RingPlaceActive=false; Status.mode="align";Status.result="released";
        Status.reason="ring_place_complete";
        if(HeldPlaceActive){HeldPlaceActive=PrealignedPlaceActive=AlignedPlace.valid=false;ActiveColor=(Camera_ColorTypeDef)HeldMaterialColor;Camera_RequestStop();enter(GRAB_COMPLETE);}
        else enter(GRAB_HOLD);
      }
      else if(Operation==OP_TAKE && !ToCar) enter_axis(GRAB_OBSERVE_Z,1);
      else enter_axis(GRAB_X_RETRACT,0);
    }
  }
  else if(Status.state==GRAB_CLOSE && now-StateTick>=Config[C_CLOSE_MS]) enter_axis(GRAB_LIFT,1);
  else if(Status.state==GRAB_RELEASE && now-StateTick>=Config[C_RELEASE_MS])
  {
    if(Operation==OP_CAR_PLACE)
    {
      Status.result="released"; Status.reason="car_place_complete"; Status.missing="none";
      Status.reference_cause="car_place_verified"; enter(GRAB_COMPLETE); return;
    }
    if(Operation==OP_STORE)
    {
      if(!Turntable_InventorySet(Status.slot,TURNTABLE_OCCUPIED,(uint8_t)ActiveColor)) { fault("inventory_record_failed"); return; }
      if(TrackStoreActive)TrackStoreRecorded=true;
      InventoryTouched=false; ToCar=false;
    }
    enter_axis(GRAB_RETRACT,1);
  }
  else if(Status.state==GRAB_X_RETRACT || Status.state==GRAB_CAR_EXTEND ||
          Status.state==GRAB_EXTERNAL_EXTEND || Status.state==GRAB_OBSERVE_X)
  {
    float target=Status.state==GRAB_X_RETRACT?Config[C_X_MIN]:
      (Status.state==GRAB_CAR_EXTEND?Config[C_X_MIN]:(Status.state==GRAB_EXTERNAL_EXTEND?ExternalX:ObserveX));
    if(!Axes[0].commanded) (void)axis_move(0,target);
    else if(arrived(0,now))
    {
      if(Status.state==GRAB_X_RETRACT)
      {
        if(Operation==OP_CAR_PLACE) enter_axis(GRAB_TURN,2);
        else if(ToCar && ProcClearance.active) enter_axis(GRAB_TURN,2);
        else if(ToCar) enter(GRAB_INDEX);
        else enter_axis(GRAB_RETURN_TURN,2);
      }
      else if(Status.state==GRAB_CAR_EXTEND)enter_axis(GRAB_PLACE,1);
      else if(Status.state==GRAB_EXTERNAL_EXTEND)
      {
        if(PrealignedPlaceActive)Status.reason="aligned_place_descending";
        enter_axis(GRAB_DESCEND,1);
      }
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
      else if(turntable.arrived && !turntable.feedback_valid && transfer_preindex_enabled())
      { if(!Turntable_VerifySlot(Status.slot)){
          Turntable_StatusGet(&turntable);
          if(strcmp(turntable.reason,"busy") && strcmp(turntable.reason,"bus_busy"))fault(turntable.reason);
        } }
      else if(transfer_preindex_enabled() && !strcmp(turntable.reason,"position_changed"))IndexSubmitted=false;
      else if(turntable.arrived && (!turntable.feedback_valid || turntable.moving)) fault("turntable_feedback_stale");
      else fault(turntable.reason);
    }
  }
  else if(Status.state==GRAB_TURN || Status.state==GRAB_RETURN_TURN)
  {
    float target=StoreFinishTravel && Status.state==GRAB_RETURN_TURN?RearBase:
      Operation==OP_CAR_PICK?RearBase+(Status.state==GRAB_RETURN_TURN?GRAB_BASE_WORK_OFFSET_DEG:0):
      TrackStoreActive?RearBase+(Status.state==GRAB_RETURN_TURN?GRAB_BASE_WORK_OFFSET_DEG:0):
      OutsideBase+(Status.state==GRAB_TURN?Config[C_BASE_CAR_OFFSET]:0);
    if(!Axes[2].commanded) (void)base_move(target);
    else if(base_arrival(now))
    {
      if(Status.state==GRAB_TURN && Operation==OP_CAR_PICK)
      {
        /* The homed/retracted X pose already aligns with the selected tray.
         * Start the requested dwell after ID8 and Base have both arrived. */
        if(!CarPickWaiting)
        {
          /* Limited opening clears adjacent trays; closed pickup resumes via LIFT. */
          Arm_GripperDutySet((float)ARM_GRIPPER_OPEN_PULSE_US/200.0f);
          CarPickWaiting=true;CarPickWaitTick=now;Status.reason="car_pick_wait";ReportPending=true;
        }
        else if(now-CarPickWaitTick>=Config[C_CAR_PICK_WAIT_MS])enter_axis(GRAB_PLACE,1);
      }
      else if(Status.state==GRAB_TURN) {
        if(ProcClearance.active && !processing_clearance_ready(now))return;
        enter_axis(GRAB_PLACE,1);
      }
      else if(Operation==OP_CAR_PICK)car_pick_complete();
      else if(StoreFinishTravel)transfer_complete();
      else if(Operation==OP_TAKE) enter_axis(GRAB_EXTERNAL_EXTEND,0);
      else enter_axis(GRAB_OBSERVE_Z,1);
    }
  }
}

static void pose_complete(void)
{
  QrDeparting=false;
  if(PoseWork)Arm_GripperSet(ARM_GRIPPER_OPEN);
  RearPose=!PoseWork && !PoseQr;OutsidePose=PoseWork;
  Status.result=PoseQr?"qr_pose_ready":PoseWork?"work_ready":"travel_ready";Status.reason=Status.result;
  Status.reference_cause=PoseWork?"work_pose_verified":"travel_pose_verified";
  Status.missing="none";Status.stop_confirmed=true;enter(GRAB_COMPLETE);
  if(NextTrack.pending)
  {
    bool finish_travel=NextTrack.finish_travel;
    NextTrack.pending=false;
    TrackStoreActive=TrackStoreReleased=TrackStoreRecorded=TrackHeld=false;
    if(!tracking_start(NextTrack.color,NextTrack.slot,NextTrack.mode,NextTrack.automatic,
         NextTrack.pick,NextTrack.observe,NextTrack.pickup,NextTrack.speed,NextTrack.budget))
      fault(Status.reason);
    else StoreFinishTravel=finish_travel;
  }
}
static bool pose_axis_held(unsigned i,uint32_t now)
{
  return axis_quiet(i,now,PoseEvidenceTick) && (Axes[i].flags&0x0dU)==1U;
}
void GrabTask_HeadingConfigGet(unsigned station,StationHeading_Config_t *out)
{
  if(!out)return;
  *out=(StationHeading_Config_t){Config[station==2?C_HEAD_PROC_ZERO:C_HEAD_TEMP_ZERO],
    Config[station==2?C_HEAD_PROC_GAIN:C_HEAD_TEMP_GAIN],Config[C_HEAD_TOL],Config[C_HEAD_SPREAD],
    Config[C_HEAD_SETTLE],Config[station==2?C_HEAD_PROC_TURN:C_HEAD_TEMP_TURN],STATION_HEADING_WAIT_MS,
    Axes[1].mm,isfinite(Axes[1].mm) && fabsf(Axes[1].mm-Config[C_Z_CAR_LIFT])<=Config[C_POS_TOL]};
  if(Status.base_home_degraded)out->gain=NAN;
}
void GrabTask_LineHeadingConfigGet(StationHeading_Config_t *out)
{
  if(!out)return;
  GrabTask_HeadingConfigGet(3,out);
  out->zero_deg=Config[C_HEAD_LINE_ZERO];
  out->gain=Status.base_home_degraded?NAN:Config[C_HEAD_LINE_GAIN];
}
bool GrabTask_HeadingReady(void)
{
  uint32_t now=HAL_GetTick();
  if(Status.state!=GRAB_COMPLETE || !RearReferenceValid ||
     strcmp(Status.result,"work_ready"))return false;
  poll_axes(now);
  return pose_axis_held(0,now) && pose_axis_held(1,now) && pose_axis_held(2,now) &&
    fabsf(Axes[0].mm-Config[C_X_MIN])<=Config[C_POS_TOL] &&
    /* Match pose preparation: already higher is ready, without rebasing Axes[].
     * HeadingConfigGet separately checks whether calibration fits this actual Z. */
    Axes[1].mm>=Config[C_Z_CAR_LIFT]-Config[C_POS_TOL] &&
    Axes[1].mm<=Config[C_Z_MAX]+Config[C_POS_TOL] &&
    fabsf(Axes[2].mm-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<=GRAB_BASE_POSITION_TOL_DEG;
}
bool GrabTask_PoseClearanceReady(void)
{
  uint32_t now=HAL_GetTick();
  return Status.state==GRAB_ROUTE_POSE && !ResumePending &&
    !Status.stop_requested && PoseStep>=4 && RearReferenceValid &&
    pose_axis_held(0,now) && pose_axis_held(1,now) && axis_fresh(2,now) &&
    (Axes[2].flags&0x0dU)==1U &&
    fabsf(Axes[0].mm-Config[C_X_MIN])<=Config[C_POS_TOL] &&
    Axes[1].mm>=Config[C_Z_CAR_LIFT]-Config[C_POS_TOL];
}
static void pose_process(uint32_t now)
{
  unsigned i;
  if(PoseStep==0)
  {
    /* Fresh stationary measurements select the remaining moves after a stop;
     * a cached rear-pose flag is not enough to skip a moving X/Z axis. */
    for(i=0;i<3;i++) if(QrDeparting?!axis_fresh(i,now):!pose_axis_held(i,now)) { poll_axes(now);return; }
    if(Axes[0].mm<Config[C_X_MIN]-Config[C_POS_TOL] || Axes[0].mm>Config[C_X_MAX]+Config[C_POS_TOL] ||
       Axes[1].mm<Config[C_Z_MIN]-Config[C_POS_TOL] || Axes[1].mm>Config[C_Z_MAX]+Config[C_POS_TOL])
    { fault("initial_axis_limit");return; }
    if(fabsf(Axes[0].mm-Config[C_X_MIN])<=Config[C_POS_TOL] &&
       Axes[1].mm>=Config[C_Z_CAR_LIFT]-Config[C_POS_TOL])
      PoseStep=4;
    else if(!QrDeparting && (PoseWork || PoseQr) && ChassisMotion_IsBusy())
    { fault("travel_pose_changed");return; }
    else PoseStep=Axes[1].mm>=Config[C_Z_CAR_LIFT]-Config[C_POS_TOL]?3:1;
  }
  for(i=0;i<3;i++) if(!axis_fresh(i,now)) { fault("feedback_stale");return; }
  if(PoseStep==1) { if(axis_move(1,fmaxf(Axes[1].mm,Config[C_Z_CAR_LIFT]))) PoseStep=2; }
  else if(PoseStep==2 && arrived(1,now)) PoseStep=3;
  else if(PoseStep==3) {
    if(pose_axis_held(0,now) && fabsf(Axes[0].mm-Config[C_X_MIN])<=Config[C_POS_TOL])PoseStep=4;
    else if(axis_move(0,Config[C_X_MIN]))PoseStep=4;
  }
  else if(PoseStep==4 && (!Axes[0].commanded || arrived(0,now)))
  {
    float target=RearBase+(PoseQr?35.0f:PoseWork?GRAB_BASE_WORK_OFFSET_DEG:0.0f);
    if(pose_axis_held(2,now) && fabsf(Axes[2].mm-target)<=GRAB_BASE_POSITION_TOL_DEG &&
       PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy())
    { pose_complete();return; }
    if(base_move(target))PoseStep=5;
  }
  else if(PoseStep==5 && base_arrival(now) && PendingAction==MECHANICAL_ARM_ACTION_NONE &&
          !MechanicalArm_IsBusy())
  {
    pose_complete();return;
  }
  poll_axes(now);
}

bool GrabTask_QrReady(float *scan_y)
{
  if(GrabTask_IsBusy() || MechanicalArm_IsBusy() || Turntable_IsBusy() ||
     HWT101_Cal_IsBusy() || !RearReferenceValid || !isfinite(Config[C_Z_PPM]) ||
     !isfinite(Config[C_Z_MIN]) || !isfinite(Config[C_Z_MAX]) ||
     Config[C_QR_Z_MIN]<Config[C_Z_MIN] || Config[C_QR_Z_MIN]>Config[C_Z_MAX] ||
     Config[C_QR_Z_SPEED]*Config[C_Z_PPM]*60/3200<1) return false;
  if(scan_y)*scan_y=Config[C_QR_SCAN_Y];
  return true;
}
bool GrabTask_QrScanStart(uint32_t initial_sequence)
{
  if(!GrabTask_QrReady(NULL) || ChassisMotion_IsBusy())return false;
  memset(Axes,0,sizeof Axes);PendingAction=MECHANICAL_ARM_ACTION_NONE;
  QrInitialSequence=initial_sequence;PollIndex=0;LastPoll=StartTick=HAL_GetTick();
  QrZStep=0;QrZWaitTick=StartTick;QrCompetition=false;QrDeparting=false;
  Operation=OP_ALIGN;InventoryTouched=false;Status.slot=0;
  Status.mode="fixed";Status.operation="qr_scan";Status.scene="none";
  Status.result="pending";Status.reason="qr_wait";Status.missing="none";
  Status.stop_requested=false;Status.stop_confirmed=false;FinishError=false;
  CalibrationTask=false;WheelBaseline=false;Mecanum_Feedback_Enable(true);
  enter(GRAB_QR_SCAN);return true;
}
bool GrabTask_QrDepartStart(void)
{
  if(Status.state==GRAB_COMPLETE && !strcmp(Status.result,"travel_ready"))return true;
  if(QrDeparting)return true;
  if(Status.state!=GRAB_QR_SCAN || ResumePending || !RearReferenceValid)return false;
  QrDeparting=true;PoseQr=PoseWork=false;PoseStep=0;
  /* Keep an in-flight query/ACK; the first lift target replaces the search Z
   * target through ordinary bus scheduling, without a global stop chain. */
  for(unsigned i=0;i<3;i++){arrival_reset(i);Axes[i].commanded=false;}
  Status.operation="pose";Status.result="pending";Status.reason="qr_departure_restore";
  Status.stop_requested=Status.stop_confirmed=false;
  PoseEvidenceTick=HAL_GetTick()-(uint32_t)Config[C_FEEDBACK_MS]-1U;
  enter(GRAB_ROUTE_POSE);return true;
}
bool GrabTask_QrDepartureActive(void)
{return QrDeparting && Status.state==GRAB_ROUTE_POSE && !ResumePending && !Status.stop_requested;}
bool GrabTask_QrScanCompetitionStart(uint32_t initial_sequence)
{
  if(Status.state==GRAB_QR_SCAN && QrCompetition) {
    QrInitialSequence=initial_sequence;Status.result="pending";return true;
  }
  if(!GrabTask_QrScanStart(initial_sequence))return false;
  QrCompetition=true;return true;
}
void GrabTask_NextRawColorSet(uint8_t color)
{NextRawColor=color>=1 && color<=6?color:0;NextRawRequested=false;}
static void qr_scan_process(uint32_t now)
{
  QR_SnapshotTypeDef qr;QR_SnapshotGet(&qr);
  if(qr.Valid && qr.Source==QR_SOURCE_UART && qr.Sequence!=QrInitialSequence)
  {
    Status.reason="qr_scanned";Status.result="qr_scanned";FinishError=false;
    if(QrCompetition)return; /* Mission owns immediate departure, no global STOP. */
    enter(GRAB_STOPPING);stop_begin();return;
  }
  bool fresh=axis_fresh(0,now)&&axis_fresh(1,now)&&axis_fresh(2,now);
  if(!fresh)
  {
    if(Axes[1].commanded)fault("feedback_stale");
    else poll_axes(now);
    return;
  }
  (void)wheels(now);if(Status.state!=GRAB_QR_SCAN)return;
  if(QrZStep==0)
  {
    if(now-StartTick<(uint32_t)Config[C_QR_WAIT_MS]) { poll_axes(now);return; }
    QrStartZ=Axes[1].mm;
    if(QrStartZ<Config[C_Z_MIN]-Config[C_POS_TOL] || QrStartZ>Config[C_Z_MAX]+Config[C_POS_TOL])
    { fault("axis_limit");return; }
    QrStartZ=fminf(Config[C_Z_MAX],fmaxf(Config[C_Z_MIN],QrStartZ));
    if(QrStartZ<=Config[C_QR_Z_MIN]+Config[C_POS_TOL])
    { QrZStep=4;Status.reason="qr_not_found"; }
    else if(axis_move(1,Config[C_QR_Z_MIN]))
    { QrZStep=1;Status.reason="qr_descending"; }
  }
  else if(QrZStep==1 && arrived(1,now))
  { QrZStep=2;QrZWaitTick=now;Status.reason="qr_lower_wait"; }
  else if(QrZStep==2 && now-QrZWaitTick>=(uint32_t)Config[C_QR_WAIT_MS])
  {
    if(axis_move(1,QrStartZ)) { QrZStep=3;Status.reason="qr_ascending"; }
  }
  else if(QrZStep==3 && arrived(1,now))
  { QrZStep=4;Status.reason="qr_not_found"; }
  poll_axes(now);
}

void GrabTask_Process(void)
{
  uint32_t now=HAL_GetTick(); bool feedback;
  if(TrackCloseStopping)track_close_stop();
  if(Tracking.reference_valid || tracking_mode()) track_observer(now);
  if(TrackTraceSending) track_trace_process();
  if(tracking_mode() && !BootActive)
  {
    if(ReportPending || (GrabTask_IsBusy() && now-LastReport>=1000))
    { track_status_write(!ReportPending); LastReport=now; ReportPending=false; }
    if(GrabTask_IsBusy()) track_process(now);
    return;
  }
  if(ReportPending)
  { status_write(false); LastReport=now; ReportPending=false; }
  else if(GrabTask_IsBusy() && now-LastReport>=1000)
  { status_write(true); LastReport=now; }
  if(!GrabTask_IsBusy()) return;
  Status.elapsed_ms=now-StartTick;
  if(BootActive) { boot_process(now); return; }
  if(Status.state==GRAB_CALIBRATION)
  {
    (void)wheels(now);
    if(Status.state!=GRAB_CALIBRATION) return;
    GrabCalibration_Process();
    if(GrabCalibration_StateGet()==GRAB_CAL_STATE_ERROR)
      fault(GrabCalibration_ErrorNameGet());
    else if(GrabCalibration_StateGet()==GRAB_CAL_STATE_ALIGNED)
    { Status.reason="calibrated";Status.result="calibrated";enter(GRAB_COMPLETE); }
    return;
  }
  if(Status.state==GRAB_QR_SCAN) { qr_scan_process(now);return; }
  if(Status.state==GRAB_ROUTE_POSE) { pose_process(now);return; }
  /* No elapsed-time cutoff for task stages. Advance on feedback or explicit stop. */
  if(Status.state==GRAB_PREPARE && !FeedbackSelected)
  {
    /* Restore all-wheel polling after a diagnostic single-wheel selection.
     * Selection retains the outstanding read/reply, then uses the new schedule. */
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
  if(ProcClearance.active && ProcClearance.recheck) {
    if(!processing_clearance_ready(now)){poll_axes(now);return;}
    ProcClearance.recheck=false;
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
        if(CalibrationTask)
        {
          Axes[0].target=XIntegral=Axes[0].mm;ObserveX=observation_target(0,Axes[0].mm);ObserveZ=observation_target(1,Axes[1].mm);
          if(Axes[0].mm+5*3200/(360*Config[C_X_PPM])>Config[C_X_MAX])
          { fault("calibration_x_limit");return; }
          GrabCalibration_ResultTypeDef begin=GrabCalibration_Start(ActiveColor);
          if(begin==GRAB_CAL_RESULT_OK) enter(GRAB_CALIBRATION);
          else if(begin!=GRAB_CAL_RESULT_BUSY) fault(GrabCalibration_ErrorNameGet());
          return;
        }
        if(PrealignedPlaceActive)
        {ExternalX=AlignedPlace.x;enter_axis(GRAB_EXTERNAL_EXTEND,0);return;}
        if(Operation==OP_CAR_PICK)
        {
          OutsideBase=RearBase+GRAB_BASE_WORK_OFFSET_DEG;OutsideReferenceValid=true;
          if(CarPickClosed && now-CarPickCloseTick<Config[C_CLOSE_MS]){poll_axes(now);return;}
          ToCar=!CarPickClosed;
          enter_axis(CarPickClosed?GRAB_LIFT:GRAB_RETRACT,1);return;
        }
        if(Operation==OP_CAR_PLACE)
        {
          OutsideBase=RearBase+GRAB_BASE_WORK_OFFSET_DEG; OutsideReferenceValid=true;
          enter_axis(GRAB_RETRACT,1);
          return;
        }
        if(TrackStoreActive)
        {
          if(TrackStoreReleased)
          {
            if(now-TrackStoreReleaseTick<Config[C_RELEASE_MS]){poll_axes(now);return;}
            if(!Turntable_InventorySet(Status.slot,TURNTABLE_OCCUPIED,(uint8_t)ActiveColor))
            {fault("inventory_record_failed");return;}
            InventoryTouched=false;TrackStoreRecorded=true;
          }
          ToCar=!TrackStoreReleased;enter_axis(TrackStoreReleased?GRAB_RETRACT:GRAB_LIFT,1);return;
        }
        if(Operation!=OP_RETURN)
        {
          OutsideBase=RearReferenceValid && RearPose?RearBase+GRAB_BASE_WORK_OFFSET_DEG:Axes[2].mm;
          ObserveX=ExternalX=observation_target(0,Axes[0].mm); ObserveZ=observation_target(1,Axes[1].mm);
          OutsideReferenceValid=true; OutsidePose=true;
        }
        if(RearReferenceValid && RearPose && Operation!=OP_RETURN) PrepareStep=5;
        else if(Operation==OP_TAKE || Operation==OP_RETURN) enter_axis(GRAB_RETRACT,1);
        else
        {
          if(!HeldPlaceActive && !ProcessedStoreActive)Arm_GripperSet(ARM_GRIPPER_OPEN);
          if(fixed_mode()) enter_axis(GRAB_DESCEND,1);
          else PrepareStep=1;
        }
      }
    }
    else if(PrepareStep==5) { if(axis_move(1,fmaxf(Axes[1].mm,Config[C_Z_CAR_LIFT]))) PrepareStep=6; }
    else if(PrepareStep==6 && arrived(1,now)) PrepareStep=7;
    else if(PrepareStep==7) { if(axis_move(0,Config[C_X_MIN])) PrepareStep=8; }
    else if(PrepareStep==8 && arrived(0,now))
    { if(base_move(OutsideBase)) PrepareStep=9; }
    else if(PrepareStep==9 && base_arrival(now) && (!StaticAlignTest || arrived(0,now)))
    {
      RearPose=false;OutsidePose=true;
      if(Operation==OP_TAKE) enter_axis(GRAB_RETRACT,1);
      else { if(!HeldPlaceActive && !ProcessedStoreActive)Arm_GripperSet(ARM_GRIPPER_OPEN);if(fixed_mode()) enter_axis(GRAB_DESCEND,1);else PrepareStep=1; }
    }
    else if(PrepareStep==1) {
      if(ProcClearance.active && !processing_clearance_ready(now)){poll_axes(now);return;}
      if(ProcClearance.active)Status.reason="processed_alignment";
      if(axis_move(1,StationAlignRaised?0:ProcessedStoreActive?RingAlignZ:StaticAlignTest?Config[C_Z_MAX]:Config[C_Z_OBSERVE])) PrepareStep=2;
    }
    else if(PrepareStep==2 && arrived(1,now)) PrepareStep=3;
    else if(PrepareStep==3)
    {
      /* Numbered rings start vision at the current X pose; only visual alignment extends X. */
      if(StaticAlignTest) { Axes[0].target=Axes[0].mm;PrepareStep=4; }
      else if(axis_move(0,Config[C_X_PRE])) PrepareStep=4;
    }
    else if(PrepareStep==4 && (StaticAlignTest || arrived(0,now)))
    {
      HAL_StatusTypeDef result;
      ObserveX=observation_target(0,Axes[0].mm);
      /* Processing observation does not change the raised posture after storage. */
      ObserveZ=ProcessedStoreActive?Config[C_Z_MAX]:observation_target(1,Axes[1].mm);
      result=request_target();
      if(result==HAL_OK) { RequestTick=now; LastControl=now; LastInvalidCount=0; stable_reset(); enter(GRAB_ACQUIRE); }
      else if(result!=HAL_BUSY) fault("camera_tx");
    }
  }
  else if(Status.state==GRAB_VISION_PAUSE) vision_pause_process(now);
  else if(Status.state==GRAB_VISION_GRACE) vision_grace_process(now);
  else if(Status.state==GRAB_ACQUIRE || Status.state==GRAB_ALIGN || Status.state==GRAB_SETTLE)
  {
    bool usable=vision_read(now);
    if(Status.state==GRAB_ACQUIRE && !usable && now-RequestTick>Config[C_VISION_WAIT_MS])
    { fault("vision_acquire_timeout"); return; }
    if(HaveCapture && (uint32_t)(now-Vision.Tick)>Config[C_AGE_MS]) fault("vision_stale");
    if(Status.state==GRAB_ACQUIRE && usable && FreshVision)
    {
      enter(GRAB_ALIGN);XIntegral=Axes[0].target;Status.stop_requested=Status.stop_confirmed=false;
      if(StaticAlignTest) { if(!ProcessedStoreActive)Axes[1].commanded=false;LastControl=now-GRAB_CONTROL_PERIOD_MS; }
    }
    if(Status.state==GRAB_ALIGN && usable)
    {
      /* Pause/vision-stop clears command evidence. Re-establish this same Z
       * target before horizontal correction or counting stable image frames. */
      if(ProcessedStoreActive && !arrived(1,now))
      {
        if(!Axes[1].commanded)axis_move(1,RingAlignZ);
        stable_reset();poll_axes(now);return;
      }
      visual_stability(now);
      if(StaticAlignTest && !ProcessedStoreActive && (!RingLowerSent || !arrived(1,now))) stable_reset();
      if(VisualStable) { enter(GRAB_SETTLE); stop_begin(); stable_reset(); }
      else
      {
        align_control(now);
        if(StaticAlignTest && !ProcessedStoreActive && Status.state==GRAB_ALIGN) static_lower_control(now);
      }
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
        if(!strcmp(Status.mode,"align"))
        {
          if(HeldPlaceActive)
          {
            ExternalX=Axes[0].mm;RingPlaceActive=true;Status.reason="held_descending";
            if(StationAlignRaised){Camera_RequestStop();HaveCapture=FreshVision=false;}
            enter_axis(GRAB_DESCEND,1);return;
          }
          Status.reason=RingTest?"ring_aligned":"aligned_wait_check";
          if(StaticAlignTest)
          {
            HaveCapture=FreshVision=false;
            if(RingStayLow)
            {
              /* Keep recognition for observation, with no motion correction in HOLD. */
              Status.reason=RingTest?"ring_aligned_low":"color_aligned_low";Status.result="aligned";
              enter(GRAB_HOLD);
            }
            else
            {
              Camera_RequestStop();
              if(RingAlignZ==Config[C_Z_MAX] && axis_quiet(1,now,StopTick) &&
                 (Axes[1].flags&0x0dU)==1U && fabsf(Axes[1].mm-Config[C_Z_MAX])<=Config[C_POS_TOL])
              {
                /* The verified alignment height is already the requested top. */
                RingRaise=false;Status.reason=StackTest?"stack_aligned_ready":(RingTest?"ring_aligned_ready":"color_aligned_ready");
                Status.result="aligned";enter(GRAB_HOLD);
              }
              else
              {
                RingRaise=true;Status.reason=RingTest?"ring_raise_no_align":"color_raise_no_align";Status.result="pending";
                enter_axis(GRAB_OBSERVE_Z,1);
              }
            }
          }
          else enter(GRAB_HOLD);
        }
        else
        {
          ExternalX=Axes[0].mm;
          if(ProcessedStoreActive)
          {
            /* Stable fresh alignment and physical stop were established above.
             * Keep XY fixed; changes caused by the final Z travel are expected. */
            Camera_RequestStop();HaveCapture=FreshVision=false;
            Status.reason="processed_descending";
          }
          enter_axis(GRAB_DESCEND,1);
        }
      }
    }
  }
  else if(Status.state==GRAB_MANUAL_Z)
  {
    /* Latest absolute target wins; keep owned ACK/read tokens and never release the claw. */
    if(!Axes[1].commanded || fabsf(Axes[1].target-ManualZTarget)*Config[C_Z_PPM]>=.5f)
      (void)axis_move(1,ManualZTarget);
    else if(arrived(1,now))
    { Status.reason="stack_height_ready";Status.result="height_ready";enter(GRAB_HOLD); }
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
    if(StaticAlignTest && RingStayLow && Status.state==GRAB_HOLD && !strcmp(Status.mode,"align") &&
       !strcmp(Status.vision_state,"Ok") && camera.RequestFunction==0xB2 &&
       camera.RequestTarget==target_color() && camera.Data.Target==target_color())
    {
      /* Read-only low-view diagnostics must not re-enter the motion controller. */
      Status.dx=(int16_t)alignment_dx(&camera.Data);
      Status.dy=(int16_t)alignment_dy(&camera.Data);
    }
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
static void boot_home_start(bool wait_turntable)
{
  if(GrabTask_IsBusy()) return;
  CalibrationTask=false;
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_ALL);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  Status.reference_cause="home_verifying";
  Status.missing="home_pending";
  BootActive=true; BootStopping=BootFailed=BootHomeSent=false;
  BootBaseDegraded=BootAcceptBase=BaseHomeAcceptAvailable=false;
  BootBaseOnly=BootResumePose=false;
  BootLaunching=true;BootStopUnconfirmed=false;
  BootWaitTurntable=wait_turntable;
  BootStartedMask=BootVerifiedMask=BootAbortMask=BootAbortSentMask=0;
  memset(BootHomeTick,0,sizeof BootHomeTick);
  BootAxisIndex=BootStep=BootGood=0; BootTick=BootPoll=StartTick=HAL_GetTick();
  BootPositionValid=BootPositionStable=BootHomeFlagsValid=BootMotorFlagsValid=false;
  BootDisabledWaiting=BootEnableRecovery=false;
  Status.reason="home_z"; Status.stop_requested=Status.stop_confirmed=false;
  enter(GRAB_HOMING);
}
void GrabTask_BootHomeStart(void) { boot_home_start(true); }
/* Verify the already requested manual near-home, without repeating HOME on
 * Base or disturbing the established X/Z origins and settings. */
static void base_reference_verify_start(bool work)
{
  CalibrationTask=false;ManualBaseHomePending=false;
  BootActive=BootBaseOnly=BootResumePose=true;BootResumePoseWork=work;
  BootBaseDegraded=BootAcceptBase=BaseHomeAcceptAvailable=false;
  BootStopping=BootFailed=BootLaunching=BootStopUnconfirmed=BootWaitTurntable=false;
  BootStartedMask=4U;BootVerifiedMask=3U;BootAbortMask=BootAbortSentMask=0;
  BootAxisIndex=2;BootStep=2;BootGood=0;BootHomeSent=true;
  BootTick=StartTick=BootHomeTick[2]=HAL_GetTick();BootPoll=BootTick-100U;
  BootPositionValid=BootPositionStable=BootHomeFlagsValid=BootMotorFlagsValid=false;
  BootDisabledWaiting=BootEnableRecovery=false;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  Status.mode="fixed";Status.operation="pose";Status.scene="none";Status.slot=0;
  Status.result="pending";Status.reason=Status.reference_cause="base_reference_verifying";
  Status.missing="rear_reference";Status.stop_requested=Status.stop_confirmed=false;
  enter(GRAB_HOMING);
}
static void reply(const char *text);
/* Z/X HOME run together; verify both before starting the loaded Base.
 * The startup tray must also finish before Base receives ENABLE/HOME. */
static void boot_axis_select(unsigned index,uint32_t now,bool verify)
{
  BootAxisIndex=index;BootGood=0;BootStep=verify?2:5;
  BootBaseDegraded=false;
  BootHomeSent=(BootStartedMask&(1U<<index))!=0;
  BootTick=verify?BootHomeTick[index]:now;BootPoll=now-100U;
  BootPositionValid=BootPositionStable=BootHomeFlagsValid=BootMotorFlagsValid=false;
  BootDisabledWaiting=BootEnableRecovery=false;
  Status.reason=index==0?"home_z":index==1?"home_x":"home_base";
  ReportPending=true;
}
static void boot_abort_next(void)
{
  uint8_t unsent=BootAbortMask&(uint8_t)~BootAbortSentMask;
  uint8_t remaining=unsent?unsent:BootAbortMask;
  if(!remaining) {
    BootActive=false;
    if(BootStopUnconfirmed)Status.reason="home_stop_unconfirmed";
    enter(BootFailed||BootStopUnconfirmed?GRAB_ERROR:GRAB_IDLE);
    return;
  }
  for(unsigned i=0;i<3;i++)if(remaining&(1U<<i)) {
    BootAxisIndex=i;BootHomeSent=true;BootStep=unsent?12:14;
    BootPoll=HAL_GetTick()-100U;return;
  }
}
static void boot_stop(const char *reason,bool failed)
{
  if(BootStopping) return;
  BootStopping=true; BootFailed=failed; BootStep=10; BootTick=HAL_GetTick();
  BootResumePose=false;
  BootAbortMask=BootStartedMask&(uint8_t)~BootVerifiedMask;BootAbortSentMask=0;
  Turntable_Stop(); /* Cancel pending boot HOME as well as an active ID8 HOME. */
  Status.reason=reason; Status.stop_requested=true; Status.stop_confirmed=false;
  if(failed)
  {
    static const char *const names[]={"z","x","base"}; char line[160];
    (void)snprintf(line,sizeof line,"ERR grab home axis=%s reason=%s home_flags=0x%02X home_valid=%u motor_flags=0x%02X motor_valid=%u\r\n",
      names[BootAxisIndex],reason,BootHomeFlags,BootHomeFlagsValid?1U:0U,
      BootMotorFlags,BootMotorFlagsValid?1U:0U);
    reply(line);
  }
  ReportPending=true;
}
static bool boot_request(MechanicalArm_ResultTypeDef result,MechanicalArm_ActionTypeDef action,
                         MechanicalArm_AxisTypeDef axis)
{
  if(result==MECHANICAL_ARM_RESULT_BUSY) return false;
  if(result!=MECHANICAL_ARM_RESULT_NONE && result!=MECHANICAL_ARM_RESULT_OK)
  {
    if(!BootStopping)
    {
      MotorBus_Diagnostic_t diagnostic;
      char message[256];
      const char *result_name=result==MECHANICAL_ARM_RESULT_PARAM_ERROR?"param_error":
        result==MECHANICAL_ARM_RESULT_TX_ERROR?"tx_error":"request_error";
      const char *action_name=action==MECHANICAL_ARM_ACTION_ENABLE?"enable":
        action==MECHANICAL_ARM_ACTION_HOME?"home":
        action==MECHANICAL_ARM_ACTION_HOME_STATE?"home_state":
        action==MECHANICAL_ARM_ACTION_POSITION_READ?"position_read":
        action==MECHANICAL_ARM_ACTION_SPEED_READ?"speed_read":"state";
      MotorBus_DiagnosticGet(&diagnostic);
      (void)snprintf(message,sizeof message,
        "ERR grab home_request axis=%s action=%s result=%s bus_locked=%u bus_reason=%s uart_error=0x%08lX fault_ms=%lu\r\n",
        BootAxisIndex==0?"z":BootAxisIndex==1?"x":"base",action_name,result_name,
        diagnostic.locked?1U:0U,diagnostic.reason?diagnostic.reason:"none",
        (unsigned long)diagnostic.uart_error,(unsigned long)diagnostic.tick_ms);
      reply(message);
      boot_stop(diagnostic.locked?"home_bus_fault":"home_request_failed",true);
    }
    return false;
  }
  PendingAction=action; PendingAxis=axis; return true;
}
static void boot_process(uint32_t now)
{
  MechanicalArm_AxisTypeDef axis=BootAxes[BootAxisIndex];
  if(!BootStopping) {
    for(unsigned i=0;i<3;i++)if((BootStartedMask&(1U<<i)) && !(BootVerifiedMask&(1U<<i)) &&
                              now-BootHomeTick[i]>60000U) {
      if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy())return;
      if(BootAxisIndex!=i) {
        BootHomeFlags=BootMotorFlags=0;
        BootPositionValid=BootHomeFlagsValid=BootMotorFlagsValid=false;
      }
      BootAxisIndex=i;boot_stop("home_timeout",true);axis=BootAxes[i];break;
    }
    if(!BootStopping && BootVerifiedMask==7U) {
      if(Turntable_IsBusy()) { Status.reason="home_wait_turntable";return; }
      if(BootWaitTurntable && !Turntable_ReferenceValid()) { boot_stop("home_turntable_failed",true); }
      else {
        BootActive=false;
        Status.reason=Status.base_home_degraded?"home_complete_degraded":"home_complete";
        Status.reference_cause=Status.base_home_degraded?"base_home_degraded":"home_verified";
        Status.missing=configuration_missing(Status.mode);
        if(!Status.missing)Status.missing="none";
        enter(GRAB_IDLE);
        if(BootBaseOnly) {
          bool resume=BootResumePose,work=BootResumePoseWork;
          BootBaseOnly=BootResumePose=false;
          Status.reference_cause=Status.base_home_degraded?"base_home_degraded":"base_home_verified";
          if(resume && !GrabTask_PoseStart(work)) { Status.result="rejected";enter(GRAB_ERROR); }
          return;
        }
        /* Arm only when all four drives have finished; motion-busy would fail IMU boot verification. */
        HWT101_Cal_BootVerifyArm();return;
      }
    }
  }
  if(now-BootTick>(BootStopping?5000U:60000U))
  {
    if(!BootStopping) boot_stop("home_timeout",true);
    else { BootActive=false; Status.reason="home_stop_unconfirmed"; enter(GRAB_ERROR); return; }
  }
  if(!BootStopping && (BootStep==7 || BootEnableRecovery) &&
     now-BootEnableTick>=BOOT_ENABLE_VERIFY_MS)
    boot_stop("home_enable_failed",true);
  if(!BootStopping && BootStep==16) /* Z/X verified; wait before launching Base. */
  {
    if(Turntable_IsBusy()) {
      if(strcmp(Status.reason,"home_wait_turntable")) {
        Status.reason="home_wait_turntable";ReportPending=true;
      }
      return;
    }
    if(BootWaitTurntable && !Turntable_ReferenceValid())
      boot_stop("home_turntable_failed",true);
    else boot_axis_select(2,now,false); /* Base gets its own enable/motion window. */
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
    if(boot_request(MechanicalArm_HomeStateRead(axis),MECHANICAL_ARM_ACTION_HOME_STATE,axis))BootStep=5;
  }
  else if(BootStep==5)
    (void)boot_request(MechanicalArm_HomeStateRead(axis),MECHANICAL_ARM_ACTION_HOME_STATE,axis);
  else if(BootStep==6 || BootStep==9)
  {
    if(boot_request(MechanicalArm_Enable(axis),MECHANICAL_ARM_ACTION_ENABLE,axis))
    {
      BootEnableTick=now;
      if(BootHomeSent) {
        BootEnableRecovery=true;BootRecoveryRaw=BootPositionRaw;
        BootPositionValid=BootPositionStable=false;BootGood=0;
      }
      BootStep=1;
    }
  }
  else if(BootStep==7)
    (void)boot_request(MechanicalArm_StateRead(axis),MECHANICAL_ARM_ACTION_STATE,axis);
  else if(BootStep==8)
  {
    if(boot_request(MechanicalArm_Home(axis,axis==MECHANICAL_ARM_AXIS_BASE?1:2),MECHANICAL_ARM_ACTION_HOME,axis))
    { BootHomeSent=true;BootStartedMask|=(uint8_t)(1U<<BootAxisIndex);
      BootHomeTick[BootAxisIndex]=now;
      if(axis==MECHANICAL_ARM_AXIS_BASE)BootTick=now;
      BootStep=1; }
  }
  else if(BootStep==2 || BootStep==14)
    (void)boot_request(MechanicalArm_HomeStateRead(axis),MECHANICAL_ARM_ACTION_HOME_STATE,axis);
  else if(BootStep==3)
    (void)boot_request(MechanicalArm_PositionRead(axis),MECHANICAL_ARM_ACTION_POSITION_READ,axis);
  else if(BootStep==4)
    (void)boot_request(MechanicalArm_StateRead(axis),MECHANICAL_ARM_ACTION_STATE,axis);
  else if(BootStep==17 || BootStep==18)
    (void)boot_request(MechanicalArm_SpeedRead(axis),MECHANICAL_ARM_ACTION_SPEED_READ,axis);
  else if(BootStep==12)
  {
    if(boot_request(MechanicalArm_HomeAbort(axis),MECHANICAL_ARM_ACTION_HOME_ABORT,axis)) BootStep=13;
  }
}
static void boot_axis_verified(MechanicalArm_AxisTypeDef axis)
{
  if(axis==MECHANICAL_ARM_AXIS_Z) defaults_restore(DEFAULT_Z_HOME);
  else if(axis==MECHANICAL_ARM_AXIS_X) defaults_restore(DEFAULT_X_HOME);
  else {
    visual_defaults_restore();
    RearBase=(float)BootPositionRaw*360.0f/65536.0f;
    if(MECHANICAL_ARM_BASE_POSITIVE_DIR) RearBase=-RearBase;
    RearReferenceValid=RearPose=true;
    Status.base_home_degraded=BootBaseDegraded;
    BaseHomeAcceptAvailable=false;
    if(BootBaseDegraded) {
      char line[160];
      (void)snprintf(line,sizeof line,
        "OK grab home_degraded axis=base failed_home_flags=0x%02X rear_deg=%.2f reference=approximate motion=quiet\r\n",
        BaseHomeFailureFlags,RearBase);
      reply(line);
      Status.reference_cause="base_home_degraded";
    }
  }
  Status.missing="home_pending";
  BootVerifiedMask|=(uint8_t)(1U<<BootAxisIndex);
  if(BootVerifiedMask==7U) {
    BootStep=15;Status.reason=Status.base_home_degraded?"home_complete_degraded":
      BootBaseOnly?"base_home_verified":"home_wait_turntable";ReportPending=true;
  }
  else if(BootAxisIndex==1) {
    /* Z and X are verified retracted before opening; Base has not started.
     * Preserve the existing ID8 gate and Base startup settling interval. */
    Arm_GripperSet(ARM_GRIPPER_OPEN);
    boot_axis_select(2,HAL_GetTick(),false);BootStep=16;
  }
  else boot_axis_select(BootAxisIndex+1,HAL_GetTick(),true);
}
static uint8_t boot_event(const MechanicalArm_EventTypeDef *event)
{
  MechanicalArm_AxisTypeDef axis=BootAxes[BootAxisIndex];
  if(PendingAction==MECHANICAL_ARM_ACTION_NONE || event->Action!=PendingAction || event->Axis!=PendingAxis) return 0;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  if(event->Result!=MECHANICAL_ARM_RESULT_OK)
  {
    if(!BootStopping) boot_stop("home_feedback_failed",true);
    else if(BootStep==10) BootStep=10;
    else {
      BootStopUnconfirmed=true;
      if(BootStep==13)BootAbortSentMask|=(uint8_t)(1U<<BootAxisIndex);
      else if(BootStep==14)BootAbortMask&=(uint8_t)~(1U<<BootAxisIndex);
      boot_abort_next();
    }
    return 1;
  }
  if(BootStopping)
  {
    if(event->Action==MECHANICAL_ARM_ACTION_STOP)
      boot_abort_next();
    else if(event->Action==MECHANICAL_ARM_ACTION_HOME_ABORT) {
      BootAbortSentMask|=(uint8_t)(1U<<BootAxisIndex);boot_abort_next();
    }
    else if(event->Action==MECHANICAL_ARM_ACTION_HOME_STATE && !(event->StateFlags[axis]&4U))
    { BootAbortMask&=(uint8_t)~(1U<<BootAxisIndex);boot_abort_next(); }
    return 1; /* stop_confirmed stays false: abort flag is not a standstill measurement. */
  }
  if(event->Action==MECHANICAL_ARM_ACTION_ENABLE)
  { BootStep=BootHomeSent?2:7;BootPoll=HAL_GetTick(); }
  else if(event->Action==MECHANICAL_ARM_ACTION_HOME) {
    if(BootLaunching && BootAxisIndex==0)boot_axis_select(1,HAL_GetTick(),false);
    else if(BootLaunching) { BootLaunching=false;boot_axis_select(0,HAL_GetTick(),true); }
    else { BootStep=2;BootPoll=HAL_GetTick(); }
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_HOME_STATE)
  {
    uint8_t flags=event->StateFlags[axis];
    BootHomeFlags=flags; BootHomeFlagsValid=true;
    /* Bit3 before HOME describes the previous attempt, which a new HOME may
     * clear. Overtemperature/current and missing calibration remain faults. */
    if((flags&0x30U) || (flags&3U)!=3U) boot_stop("home_driver_fault",true);
    else if(BootHomeSent && (flags&8U)) {
      if(axis!=MECHANICAL_ARM_AXIS_BASE) boot_stop("home_driver_fault",true);
      else {
        BaseHomeAcceptAvailable=true;
        BaseHomeFailureFlags=flags;
        if(!BootAcceptBase && Config[C_BASE_HOME_DEGRADE]<.5f) boot_stop("home_driver_fault",true);
        else if(!(flags&4U)) {
          BootBaseDegraded=true;BootStep=3;
          Status.reason="home_degraded_check";ReportPending=true;
        }
        else { BootGood=0;BootPositionValid=BootPositionStable=false; }
      }
    }
    else if(!(flags&4U)) BootStep=BootHomeSent?3:6;
    else { BootGood=0; BootPositionValid=BootPositionStable=false; }
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_POSITION_READ)
  {
    int64_t raw=event->CurrentPositionRaw;
    int64_t stable_tol=axis==MECHANICAL_ARM_AXIS_BASE?
      (int64_t)(GRAB_BASE_HOME_STABLE_TOL_DEG*65536.0f/360.0f):182;
    BootPositionStable=BootPositionValid && llabs(raw-BootPositionRaw)<=stable_tol;
    BootPositionRaw=raw; BootPositionValid=true;
    if(BootEnableRecovery && llabs(raw-BootRecoveryRaw)>stable_tol)
    { boot_stop("home_position_changed",true);return 1; }
    /* Base near-home targets a saved single-turn mechanical origin, not the
     * multi-turn position counter's numerical zero. Do not clear that counter. */
    BootStep=4;
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_STATE)
  {
    uint8_t flags=event->StateFlags[axis];
    BootMotorFlags=flags; BootMotorFlagsValid=true;
    if(flags&0x08U) boot_stop("home_axis_fault",true);
    else if(!BootHomeSent) {
      if(flags&1U)BootStep=8;
      /* Keep reading actual enable state; the ENABLE ACK alone is insufficient. */
    }
    else if(!(flags&1U)) {
      BootGood=0;BootStep=2;
      if(!BootDisabledWaiting) {
        BootDisabledWaiting=true;BootDisabledTick=HAL_GetTick();
        Status.reason="home_enable_wait";ReportPending=true;
      }
      if(!BootEnableRecovery && HAL_GetTick()-BootDisabledTick>=BOOT_DISABLE_GRACE_MS &&
         !(flags&4U) && BootPositionStable &&
         (axis==MECHANICAL_ARM_AXIS_BASE || (BootPositionRaw>=-182 && BootPositionRaw<=182)))
      {
        char line[144];
        (void)snprintf(line,sizeof line,
          "OK grab home_enable_recovery axis=%s home_flags=0x%02X motor_flags=0x%02X raw=%lld\r\n",
          axis==MECHANICAL_ARM_AXIS_Z?"z":axis==MECHANICAL_ARM_AXIS_X?"x":"base",
          BootHomeFlags,BootMotorFlags,(long long)BootPositionRaw);
        /* A failed HOME needs measured zero speed before re-enabling too. */
        reply(line);BootStep=BootBaseDegraded?18:9;
        Status.reason="home_enable_recovery";ReportPending=true;
      }
    }
    else if(!BootPositionStable || (flags&4U) ||
            (axis!=MECHANICAL_ARM_AXIS_BASE && (BootPositionRaw < -182 || BootPositionRaw > 182)))
    { BootGood=0;BootStep=2; }
    /* 0x3B already confirms homing finished; Base need not assert the separate
     * position-move reached flag after near-home. Enabled/non-protected is required. */
    else if(axis!=MECHANICAL_ARM_AXIS_BASE && (flags&3U)!=3U) { BootGood=0; BootStep=2; }
    else if(BootBaseDegraded) BootStep=17;
    else if(++BootGood<2) {
      BootDisabledWaiting=false;BootStep=2;
      Status.reason=BootEnableRecovery?"home_enable_recovery":
        axis==MECHANICAL_ARM_AXIS_Z?"home_z":axis==MECHANICAL_ARM_AXIS_X?"home_x":"home_base";
    }
    else boot_axis_verified(axis);
  }
  else if(event->Action==MECHANICAL_ARM_ACTION_SPEED_READ) {
    /* Two separate fresh HOME/position/state/speed rounds are required.
     * The 5-degree position window alone never proves standstill. */
    if(event->CurrentSpeedRpm!=0) { BootGood=0;BootStep=2; }
    else if(BootStep==18) BootStep=9;
    else if(++BootGood<2) { BootDisabledWaiting=false;BootStep=2; }
    else boot_axis_verified(axis);
  }
  return 1;
}
static bool config_value_valid(unsigned key,float value)
{
  const GrabParameter_t *p=&Parameters[key];
  return isfinite(value) && value<=p->maximum &&
    ((p->validation & 2)?value>p->minimum:value>=p->minimum) &&
    (!(p->validation & 1) || floorf(value)==value);
}
static void reply(const char *text)
{ (void)ConsoleTx_Write((const uint8_t *)text,(uint16_t)strlen(text)); }

/**********************************************************
*** 动态追踪：局部反馈、最新目标发送与独立命令
**********************************************************/
/* Only completed synchronous wheel batches fill the short encoder read gaps.
 * Keep their integral at each real encoder timestamp, so a reversal is not
 * extrapolated using a 300-500 ms old average wheel speed. */
static void track_command_motion(uint32_t now,const Mecanum_Status_t *tx)
{
  float units[4]={0}; uint32_t dt=0,after=0;
  bool valid=tx->sent_velocity_valid && !tx->locked;
  if(TrackCommandCount)
  {
    unsigned last=TrackCommandCount-1;
    memcpy(units,TrackCommandHistory[last].units,sizeof units);
    dt=now-TrackCommandHistory[last].tick;
    if(dt>100)dt=0;
    if(valid && tx->sent_ms!=TrackCommandApplied)
      after=now-tx->sent_ms<=dt?now-tx->sent_ms:dt;
    else if(!valid)after=dt;
  }
  for(unsigned i=0;i<4;i++)
  {
    float next=valid?tx->sent_rpm[i]*65536.f/60.f:0;
    units[i]+=TrackCommandUnitsS[i]*(dt-after)/1000.f+next*after/1000.f;
    TrackCommandUnitsS[i]=next;
  }
  if(valid)TrackCommandActive=true;
  TrackCommandApplied=tx->sent_ms;
  if(TrackCommandCount==32)
  { memmove(TrackCommandHistory,TrackCommandHistory+1,31*sizeof(*TrackCommandHistory)); TrackCommandCount--; }
  TrackCommandHistory[TrackCommandCount].tick=now;
  memcpy(TrackCommandHistory[TrackCommandCount].units,units,sizeof units); TrackCommandCount++;
}
static bool track_command_at(unsigned wheel,uint32_t stamp,float *units)
{
  if(!TrackCommandCount || (int32_t)(stamp-TrackCommandHistory[0].tick)<0)return false;
  for(unsigned i=1;i<TrackCommandCount;i++)
  {
    if((int32_t)(stamp-TrackCommandHistory[i].tick)<=0)
    {
      float fraction=(float)(stamp-TrackCommandHistory[i-1].tick)/
        (float)(TrackCommandHistory[i].tick-TrackCommandHistory[i-1].tick);
      *units=TrackCommandHistory[i-1].units[wheel]+fraction*
        (TrackCommandHistory[i].units[wheel]-TrackCommandHistory[i-1].units[wheel]);
      return true;
    }
  }
  *units=TrackCommandHistory[TrackCommandCount-1].units[wheel]; return true;
}
static void track_observer(uint32_t now)
{
  static const uint8_t dirs[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  ChassisObserver_Feedback_t sample; Mecanum_Status_t tx; HWT101_Angle_t angle={0};
  ChassisModel_Velocity_t actual; float rpm[4],sent[4];
  float innovation_blend=clamp((now-TrackObserverTick)/100.f,0,1);
  unsigned i;
  if(now-TrackObserverTick<20)return;
  TrackObserverTick=now; memset(&sample,0,sizeof sample);
  Mecanum_StatusGet(&tx); track_command_motion(now,&tx);
  sample.now_ms=now; sample.units_per_rev=65536;
  sample.yaw_valid=HWT101_Cal_ControlAngleGet(&angle) && now-angle.last_update_ms<=HWT101_DATA_FRESH_MS;
  sample.yaw_rad=(angle.yaw*CHASSIS_HEADING_TEST_YAW_SIGN-TrackHeading)*CHASSIS_MODEL_PI/180;
  for(i=0;i<4;i++)
  {
    Mecanum_Feedback_t f; memset(&f,0,sizeof f); (void)Mecanum_FeedbackGet(i,&f);
    int64_t position=dirs[i]?-f.position_raw:f.position_raw;
    uint32_t age=now-f.position_ms;
    float command_at_read=0;
    bool predicted=TrackCommandActive && track_command_at(i,f.position_ms,&command_at_read);
    sample.valid[i]=f.position_valid && age<=600;
    if(sample.valid[i] && (!TrackWheel[i].valid || f.position_ms!=TrackWheel[i].tick))
    {
      uint32_t dt=f.position_ms-TrackWheel[i].tick;
      TrackWheel[i].units_s=TrackWheel[i].valid && dt && dt<=600?
        (float)(position-TrackWheel[i].position)*1000.f/dt:0;
      TrackWheel[i].position=position; TrackWheel[i].tick=f.position_ms; TrackWheel[i].valid=true;
      if(predicted)
      {
        TrackWheel[i].target_offset=(float)position-command_at_read;
        if(!TrackWheel[i].anchored)TrackWheel[i].offset=TrackWheel[i].target_offset;
        TrackWheel[i].anchored=true;
      }
    }
    if(predicted && TrackWheel[i].anchored)
    {
      /* Encoder innovations correct the predictor continuously over 100 ms;
       * a single returning wheel must not create an instantaneous camera step. */
      TrackWheel[i].offset+=(TrackWheel[i].target_offset-TrackWheel[i].offset)*innovation_blend;
      sample.position[i]=(int64_t)llroundf(TrackCommandHistory[TrackCommandCount-1].units[i]+TrackWheel[i].offset);
    }
    else sample.position[i]=position+(int64_t)llroundf(TrackWheel[i].units_s*age/1000.f);
    sample.time_ms[i]=now;
    rpm[i]=TrackWheel[i].units_s*60.f/65536.f;
  }
  for(i=0;i<4;i++)sent[i]=(float)tx.sent_rpm[i]; /* already logical forward signs */
  ChassisObserver_Command(&TrackObserver,&TrackGeometry,sent,now,tx.sent_ms,
    tx.sent_velocity_valid && !tx.locked);
  if(ChassisModel_Forward(&TrackGeometry,rpm,&actual))TrackPositionVelocity=actual;
  ChassisObserver_Feedback(&TrackObserver,&TrackGeometry,&sample,&TrackObserverHistory);
  actual=TrackPositionVelocity;
  TrackInput.motion_mm[0]=TrackObserver.feedback.x_mm;
  TrackInput.motion_mm[1]=TrackObserver.feedback.y_mm;
  /* A cached X read belongs to position_tick, not now. Extrapolate from its
   * measured signed slope before saving/interpolating camera motion history. */
  TrackInput.motion_mm[2]=Axes[0].position_valid?
    clamp(Axes[0].mm+Axes[0].signed_speed*fminf((now-Axes[0].position_tick)/1000.f,.2f),Config[C_X_MIN],Config[C_X_MAX])-TrackXZero:0;
  TrackInput.actual_mm_s[0]=actual.vx_mm_s; TrackInput.actual_mm_s[1]=actual.vy_mm_s;
  TrackInput.actual_mm_s[2]=Axes[0].signed_speed;
  /* A prior manual move/stop is not this segment's first wheel batch. Including
   * idle time corrupts the braking delay as well as the timing statistics. */
  if(!TrackMoving || !tx.sent_velocity_valid || tx.stage==MECANUM_STAGE_STOPPED ||
     (int32_t)(tx.sent_ms-StartTick)<0)
    TrackLastBatch=TrackBatchInterval=0;
  else if(tx.sent_ms!=TrackLastBatch)
  {
    if(TrackLastBatch)TrackBatchInterval=tx.sent_ms-TrackLastBatch;
    TrackLastBatch=tx.sent_ms;
  }
  TrackInput.batch_interval_ms=TrackBatchInterval;
  TrackInput.sent_ms=tx.sent_velocity_valid?tx.sent_ms:0;
}

static bool track_feedback_item(uint32_t now,uint8_t address,const char *kind,
                                bool valid,uint32_t tick,uint32_t limit)
{
  uint32_t age=now-tick;
  if(valid && age<=limit)return true;
  if(TrackReady && !TrackFeedbackFault.address)
  {
    TrackFeedbackFault.address=address; TrackFeedbackFault.kind=kind;
    TrackFeedbackFault.age_ms=age; TrackFeedbackFault.limit_ms=limit; TrackFeedbackFault.valid=valid;
  }
  return false;
}
static bool track_feedback(uint32_t now)
{
  static const uint8_t ids[3]={MOTOR_ID_ARM_X,MOTOR_ID_ARM_Z,MOTOR_ID_ARM_BASE};
  bool fresh=true;
  for(unsigned i=0;i<3;i++)
  {
    if(!track_feedback_item(now,ids[i],"position",Axes[i].position_valid,Axes[i].position_tick,(uint32_t)Config[C_FEEDBACK_MS]))fresh=false;
    if(!track_feedback_item(now,ids[i],"state",Axes[i].state_valid,Axes[i].state_tick,(uint32_t)Config[C_FEEDBACK_MS]))fresh=false;
  }
  for(unsigned i=0;i<4;i++)
  {
    Mecanum_Feedback_t f={0}; (void)Mecanum_FeedbackGet(i,&f);
    if(!track_feedback_item(now,(uint8_t)(i+1),"position",f.position_valid,f.position_ms,600))fresh=false;
    if(!track_feedback_item(now,(uint8_t)(i+1),"speed",f.speed_valid,f.speed_ms,(uint32_t)Config[C_FEEDBACK_MS]))fresh=false;
    if(!track_feedback_item(now,(uint8_t)(i+1),"state",f.state_valid,f.state_ms,(uint32_t)Config[C_FEEDBACK_MS]))fresh=false;
    if(f.state_valid && ((f.state_flags&0x08U)||!(f.state_flags&1U)))
    { track_motor_fault((uint8_t)(i+1),"wheel",f.state_flags,f.state_ms,now);fault("wheel_disabled_or_stall"); }
  }
  return fresh;
}

/* Each accepted arm transaction advances one of the round-robin opportunities. A changing
 * X target cannot continually overtake the slow state/pose reads. Missing work
 * is skipped, so quiet tracking does not add fixed waits or empty bus slots. */
static bool track_pick_window(float tolerance)
{ return fabsf(TrackInput.error_px[0])<=tolerance && fabsf(TrackInput.error_px[1])<=tolerance; }
static bool track_pick_hold(uint32_t now,bool new_measurement)
{
  const uint32_t grace=track_config(TrackMode)->grace_ms;
  if(!TrackPickArmed || TrackPickLowering || !TrackInput.valid || now-TrackInput.sample_ms>grace ||
     (new_measurement && !track_pick_window(Config[C_TRACK_PICK_TOL_PX])))
  { TrackPickHoldActive=false;return false; }
  if(new_measurement)
  {
    if(!TrackPickHoldActive || TrackInput.sample_ms-TrackPickHoldLast>grace)
    { TrackPickHoldActive=true;TrackPickHoldStart=TrackInput.sample_ms; }
    TrackPickHoldLast=TrackInput.sample_ms;
  }
  /* Timer/duplicate snapshots cannot extend a visual hold. */
  return TrackPickHoldActive && TrackPickHoldLast-TrackPickHoldStart>=(uint32_t)Config[C_TRACK_PICK_HOLD_MS];
}
static void track_io(uint32_t now,bool moving)
{
  MechanicalArm_ResultTypeDef result; MechanicalArm_AxisTypeDef axis;
  MechanicalArm_ActionTypeDef action;
  if(PendingAction!=MECHANICAL_ARM_ACTION_NONE || MechanicalArm_IsBusy())return;
  if(TrackPickLowering && Status.state==GRAB_DESCEND)
  {
    /* High-level admission owns one continuous Z descent. XY still follows;
     * low-plane pixel changes do not split or restart this fixed movement. */
    if(!Axes[1].commanded)
    {
      if(axis_move(1,Config[C_Z_GRAB]))Status.reason="track_pick_descend";
      return;
    }
  }
  unsigned slots=TrackPickLowering?4U:3U;
  for(unsigned attempt=0;attempt<slots;attempt++)
  {
    unsigned slot=(TrackIoSlot+attempt)%slots;
    if(slot==3)
    {
      /* Active descent needs timely Z arrival evidence without starving X. */
      if(now-TrackZPollTick<50U)continue;
      action=TrackZPollState?MECHANICAL_ARM_ACTION_STATE:MECHANICAL_ARM_ACTION_POSITION_READ;
      result=TrackZPollState?MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_Z):MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_Z);
      if(result_accept(result,action,MECHANICAL_ARM_AXIS_Z))
      {TrackZPollState=!TrackZPollState;TrackZPollTick=now;TrackIoSlot=0;}
      return;
    }
    if(slot==0)
    {
      if(!moving || TrackXControlSequence==TrackControlSequence)continue;
      if(fabsf(TrackOutput.velocity_mm_s[2])<=.1f)
      {
        if(!TrackPulseValid) { TrackXControlSequence=TrackControlSequence; continue; }
        if(result_accept(MechanicalArm_StopScheduled(MECHANICAL_ARM_AXIS_X),MECHANICAL_ARM_ACTION_STOP,MECHANICAL_ARM_AXIS_X))
        { arrival_reset(0);TrackPulseValid=false; TrackXControlSequence=TrackControlSequence; Axes[0].commanded=false; TrackIoSlot=1; }
        return;
      }
      float ppm=Config[C_X_PPM],exact=TrackOutput.x_target_mm*ppm+TrackPulseRemainder;
      float rpm=fabsf(TrackOutput.velocity_mm_s[2])*ppm*60/3200;
      uint16_t speed=(uint16_t)clamp(roundf(rpm),1,3000);
      int32_t pulse=(int32_t)lroundf(exact);
      pulse=(int32_t)clamp((float)pulse,ceilf(Config[C_X_MIN]*ppm),floorf(Config[C_X_MAX]*ppm));
      if(TrackPulseValid && pulse==TrackLastPulse && speed==TrackLastRpm)
      { TrackXControlSequence=TrackControlSequence; continue; }
      {
        MechanicalArm_ConfigTypeDef config;
        if(!MechanicalArm_ConfigGet(MECHANICAL_ARM_AXIS_X,&config)) { fault("axis_config"); return; }
        result=MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_X,pulse,speed,config.Acceleration,MECHANICAL_ARM_POSITION_ABSOLUTE);
      }
      if(result_accept(result,MECHANICAL_ARM_ACTION_POSITION,MECHANICAL_ARM_AXIS_X))
      {
        TrackXDispatch.requested_pulses=pulse; TrackXDispatch.requested_rpm=speed;
        TrackXDispatch.request_ms=now; TrackXDispatch.requested=true;
        TrackPulseRemainder=exact-pulse; TrackLastPulse=pulse; TrackLastRpm=speed;
        TrackPulseValid=true; TrackXControlSequence=TrackControlSequence; TrackIoSlot=1;
        arrival_reset(0);Axes[0].target=pulse/ppm; Axes[0].command_tick=now; Axes[0].commanded=true; Axes[0].quiet=false;
        Status.x_target_mm=Axes[0].target;
      }
      return;
    }
    if(slot==1)
    {
      if(Axes[0].position_valid && now-Axes[0].position_tick<50)continue;
      if(result_accept(MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_X),MECHANICAL_ARM_ACTION_POSITION_READ,MECHANICAL_ARM_AXIS_X))
      { LastPoll=now; TrackIoSlot=2; }
      return;
    }
    if(now-TrackLastStateRead<100)continue;
    /* Initial pose reads first; then low-rate Z/Base and drive-enabled evidence. */
    unsigned index;
    for(index=0;index<3;index++)if(!Axes[index].position_valid || !Axes[index].state_valid)break;
    bool initial=index<3;
    if(initial)action=!Axes[index].position_valid?MECHANICAL_ARM_ACTION_POSITION_READ:MECHANICAL_ARM_ACTION_STATE;
    else
    {
      static const unsigned order[5]={0,1,2,1,2};
      index=order[TrackSlowPoll%5];
      action=TrackSlowPoll%5<3?MECHANICAL_ARM_ACTION_STATE:MECHANICAL_ARM_ACTION_POSITION_READ;
    }
    axis=index==0?MECHANICAL_ARM_AXIS_X:(index==1?MECHANICAL_ARM_AXIS_Z:MECHANICAL_ARM_AXIS_BASE);
    result=action==MECHANICAL_ARM_ACTION_POSITION_READ?MechanicalArm_PositionRead(axis):MechanicalArm_StateRead(axis);
    if(result_accept(result,action,axis))
    { if(!initial)TrackSlowPoll=(TrackSlowPoll+1)%5; TrackLastStateRead=now; TrackIoSlot=TrackPickLowering?3U:0U; }
    return;
  }
}

static void track_pause(bool visual)
{
  TrackPickHoldActive=false;
  TrackVisualPause=visual; Status.reason=visual?"track_visual_wait":"track_window_end";
  enter(GRAB_TRACK_PAUSE); stop_begin(); TrackMoving=false;
}
/* Only pre-index while the arm is confirmed at the external work direction.
 * Returning from the previous tray must finish its empty work pose first. */
static void track_preindex(uint32_t now)
{
  if(!TrackPickArmed || !TrackStoreSlot || !TrackReady || !raw_preindex_ready(now))return;
  Turntable_Status_t tray;
  if(!IndexSubmitted){
    if(Turntable_Index(TrackStoreSlot))IndexSubmitted=true;
    else {Turntable_StatusGet(&tray);
      if(strcmp(tray.reason,"busy") && strcmp(tray.reason,"bus_busy"))fault(tray.reason);
    }
  }else if(!Turntable_IsBusy()){
    Turntable_StatusGet(&tray);if(!tray.arrived)fault(tray.reason);
  }
}
static void track_close_stop(void)
{
  /* Close the gripper first; zero velocity and scheduled X hold drain normal
   * transactions without cancelling ID8 or blocking on all-motor stability. */
  if(!TrackCloseZeroRequested && Mecanum_Velocity_Request(0,0,0,CHASSIS_VELOCITY_ACCELERATION))
    TrackCloseZeroRequested=true;
  if(!TrackCloseXRequested && PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy() &&
     result_accept(MechanicalArm_StopScheduled(MECHANICAL_ARM_AXIS_X),MECHANICAL_ARM_ACTION_STOP,MECHANICAL_ARM_AXIS_X)) {
    TrackCloseXRequested=true;Axes[0].commanded=false;TrackPulseValid=false;
  }
  Mecanum_Status_t tx;Mecanum_StatusGet(&tx);
  if(TrackCloseZeroRequested && TrackCloseXRequested && PendingAction==MECHANICAL_ARM_ACTION_NONE &&
     tx.tx_complete && tx.sent_velocity_valid && !tx.sent_rpm[0] && !tx.sent_rpm[1] && !tx.sent_rpm[2] && !tx.sent_rpm[3])
    TrackCloseStopping=false;
}
static void track_pick_close(void)
{
  TrackPickLowering=TrackPickArmed=false;TrackMoving=false;
  Tracking.frozen=true;TrackHeld=true;Status.reason="track_pick_close";
  Arm_GripperSet(ARM_GRIPPER_CATCH);enter(GRAB_CLOSE);
  TrackCloseStopping=true;TrackCloseZeroRequested=TrackCloseXRequested=false;
  Mecanum_TrackingTiming(false,0);
}
static void track_process(uint32_t now)
{
  const GrabTracking_Config_t *profile=track_config(TrackMode);
  Camera_SnapshotTypeDef camera; Mecanum_Status_t tx; bool fresh,window_wait=false;
  Status.elapsed_ms=now-StartTick;
  if(Status.state==GRAB_STOPPING)
  {
    (void)wheels(now); stop_process(now,true);
    if(Status.stop_confirmed && PendingAction==MECHANICAL_ARM_ACTION_NONE)
    { Tracking.frozen=true; TrackMoving=false; Mecanum_TrackingTiming(false,0); enter(FinishError?GRAB_ERROR:GRAB_IDLE); }
    else if(now-StopTick>5000)
    { Tracking.frozen=true; TrackMoving=false; Status.reason="stop_unconfirmed"; Mecanum_TrackingTiming(false,0); enter(GRAB_ERROR); }
    else poll_axes(now);
    return;
  }
  if(!FeedbackSelected)
  {
    if(!Mecanum_Feedback_Select(0))return;
    FeedbackSelected=true;
  }
  Mecanum_Feedback_Enable(true);
  Camera_SnapshotGet(&camera); Mecanum_StatusGet(&tx);
  if(tx.locked || tx.stage==MECANUM_STAGE_FAULT) { fault("chassis_tx"); return; }
  fresh=track_feedback(now);
  if(fresh)track_preindex(now);
  if(Status.state==GRAB_STOPPING)return;
  if(Operation==OP_TRACK_PICK && (Status.state==GRAB_CLOSE || Status.state==GRAB_LIFT))
  {
    if(!fresh) { fault("feedback_stale");return; }
    transfer_process(now);poll_axes(now);return;
  }
  if(fresh && TrackPickLowering && Status.state==GRAB_DESCEND && arrived(1,now))
  {track_pick_close();return;}
  if(!TrackReady)
  {
    /* Continuous handoffs retain fresh measured feedback. Read only what is
     * absent/expired instead of demanding 18 timestamps after this entry. */
    track_io(now,false);
    if(fresh && TrackObserver.feedback_valid)
    {
      TrackReady=true;
      if(!TrackXZeroValid) { TrackXZero=Axes[0].mm; TrackXZeroValid=true; TrackInput.motion_mm[2]=0; }
    }
    else return;
  }
  else if(!fresh) { fault("feedback_stale"); return; }
  if(Axes[0].mm<Config[C_X_MIN]-Config[C_POS_TOL] || Axes[0].mm>Config[C_X_MAX]+Config[C_POS_TOL])
  { fault("axis_limit"); return; }
  TrackInput.valid=camera.UsbConfigured && camera.RequestActive && camera.RequestFunction==0xB2 &&
    camera.RequestTarget==target_color() && camera.HasValidData && camera.TargetValid &&
    camera.Data.Function==0xB2 && camera.Data.Target==target_color();
  TrackInput.now_ms=now; TrackInput.sample_ms=camera.Data.Tick; TrackInput.sequence=camera.Data.Sequence;
  TrackInput.received_ms=camera.Data.ReceivedTick; TrackInput.receive_time_valid=camera.Data.HasReceiveTime!=0;
  /* Keep the camera's adjustable material reference authoritative. CX/CY
   * remain diagnostics, not a second controller-side target definition. */
  TrackInput.error_px[0]=(float)camera.Data.DX;
  TrackInput.error_px[1]=(float)camera.Data.DY;
  TrackInput.x_mm=Axes[0].mm; TrackInput.x_min_mm=Config[C_X_MIN]; TrackInput.x_max_mm=Config[C_X_MAX];
  TrackInput.feedback_age_ms=now-Axes[0].position_tick;
  if(TrackMode==GRAB_TRACK_FEEDBACK)
    for(unsigned i=0;i<4;i++){Mecanum_Feedback_t wheel={0};if(Mecanum_FeedbackGet(i,&wheel) && wheel.position_valid && now-wheel.position_ms>TrackInput.feedback_age_ms)TrackInput.feedback_age_ms=now-wheel.position_ms;}
  for(unsigned i=0;i<3;i++) { TrackInput.j[0][i]=Config[C_J00+i]; TrackInput.j[1][i]=Config[C_J10+i]; }
  if(TrackAutoStart)
  {
    /* Observe first using fresh pixels and measured platform motion. This is
     * not an additional all-motor standstill gate between consecutive items. */
    if(!TrackInput.valid || now-camera.Data.Tick>profile->grace_ms ||
       (int32_t)(camera.Data.Tick-StartTick)<0) { track_io(now,false);return; }
    if(!Tracking.have_sample || (camera.Data.Sequence!=Tracking.last_sequence &&
       camera.Data.Tick!=Tracking.last_sample_ms))
    {
      if(!TrackEntryHaveSample) { TrackEntryHaveSample=true;TrackEntryFirstSample=camera.Data.Tick; }
      TrackEntrySamples++;
      TrackInput.actuating=false;GrabTracking_Update(&Tracking,profile,&TrackInput,&TrackOutput);
      /* Observation estimates material speed but sends no candidate velocity. */
      memset(Tracking.previous_mm_s,0,sizeof Tracking.previous_mm_s);
      Status.dx=(int16_t)TrackInput.error_px[0];Status.dy=(int16_t)TrackInput.error_px[1];
      if(!TrackOutput.model_valid) { fault("track_model");return; }
    }
    if(!TrackEntryHaveSample || (TrackMode==GRAB_TRACK_PREDICT && camera.Data.Tick-TrackEntryFirstSample<profile->fit_ms) ||
       (TrackMode==GRAB_TRACK_PREDICT?Tracking.samples:TrackEntrySamples)<2) { track_io(now,false);return; }
    if(!Tracking.reference_valid)
      GrabTracking_Reference(&Tracking,TrackInput.motion_mm[1]-(50-TrackEntryBudget));
    TrackAutoStart=false;TrackMoving=true;
    GrabTracking_Begin(&Tracking);Mecanum_TrackingTiming(true,profile->batch_ms);
    Status.reason="track_matching";enter(GRAB_TRACK_MATCH);
    if(TrackPickArmed)Arm_GripperSet(ARM_GRIPPER_OPEN);
  }
  bool new_measurement=TrackInput.valid && (!Tracking.have_sample ||
    (TrackInput.sequence!=Tracking.last_sequence && TrackInput.sample_ms!=Tracking.last_sample_ms));
  bool pick_hold_ready=track_pick_hold(now,new_measurement);
  if(Status.state==GRAB_TRACK_PAUSE)
  {
    (void)wheels(now); stop_process(now,false);
    if(!StopEvidenceStarted || !Status.stop_confirmed) { poll_axes(now); return; }
    if(!TrackVisualPause)
    {
      /* A travel edge is a temporary allocation constraint. Keep observing
       * from the stopped pose; a returning target may be reachable by moving
       * away from the edge, without changing its reference or budget. */
      window_wait=true;Tracking.frozen=false;
    }
    else if(TrackInput.valid && now-camera.Data.Tick<=profile->control_ms*2 && camera.Data.Sequence!=Tracking.last_sequence)
    {
      RecoveryUsed++; Status.stop_requested=Status.stop_confirmed=false;
      TrackMoving=true; TrackPulseValid=false; Mecanum_TrackingTiming(true,profile->batch_ms);
      memset(Tracking.previous_mm_s,0,sizeof Tracking.previous_mm_s);
      Status.reason="track_recovered"; enter(TrackPickLowering?GRAB_DESCEND:GRAB_TRACK_MATCH);
    }
    else { track_io(now,false); return; }
  }
  if(new_measurement || now-LastControl>=profile->control_ms)
  {
    TrackInput.actuating=TrackMoving;
    TrackInput.request_ms=TrackLastRequest;
    /* Candidate corrections while waiting were never sent. Resume with
     * acceleration measured from rest, not accumulated unsent predictions. */
    if(window_wait)memset(Tracking.previous_mm_s,0,sizeof Tracking.previous_mm_s);
    uint32_t calculation_started=HAL_GetTick();
    LastControl=now;TrackControlSequence++; GrabTracking_Update(&Tracking,profile,&TrackInput,&TrackOutput);
    uint32_t calculation_ms=HAL_GetTick()-calculation_started;
    Tracking.computation_timing.count++;
    Tracking.computation_timing.bins[calculation_ms<200?calculation_ms:200]++;
    if(calculation_ms>Tracking.computation_timing.maximum_ms)Tracking.computation_timing.maximum_ms=calculation_ms;
    GrabTracking_Record_t *record=Tracking.record_count?
      &Tracking.records[(Tracking.record_head+GRAB_TRACKING_TRACE-1U)%GRAB_TRACKING_TRACE]:NULL;
    if(record){track_record_io(&TrackIo,now);record->sent_age_ms=TrackIo.sent_age_ms;}
    if(TrackInput.valid)
    { Status.dx=(int16_t)TrackInput.error_px[0]; Status.dy=(int16_t)TrackInput.error_px[1]; Vision=camera.Data; HaveCapture=true; }
    if(Status.state!=GRAB_TRACK_OBSERVE)
    {
      HWT101_Angle_t angle;
      if(!window_wait && TrackOutput.visual_pause)
      { if(record)track_log_record(record);track_pause(true);return; }
      if(!TrackOutput.model_valid) { if(record)track_log_record(record);fault("track_model"); return; }
      if(window_wait)
      {
        /* Timers and repeated/invalid coordinates update diagnostics only.
         * Only a new usable observation may restart physical tracking. */
        if(!new_measurement || TrackOutput.visual_pause || TrackOutput.window_end)
        { if(record)track_log_record(record);track_io(now,false);return; }
        Status.stop_requested=Status.stop_confirmed=false;
        TrackMoving=true;TrackPulseValid=false;
        Mecanum_TrackingTiming(true,profile->batch_ms);
        Status.reason="track_window_recovered";enter(TrackPickLowering?GRAB_DESCEND:GRAB_TRACK_MATCH);
      }
      else
      {
        if(TrackOutput.window_end) { if(record)track_log_record(record);track_pause(false); return; }
      }
      if(!HWT101_Cal_ControlAngleGet(&angle) || now-angle.last_update_ms>HWT101_DATA_FRESH_MS)
      { if(record)track_log_record(record);fault("imu_invalid"); return; }
      Status.forward_mm_s=TrackOutput.velocity_mm_s[0]; Status.left_mm_s=TrackOutput.velocity_mm_s[1];
      /* Drive grade20 differs from shared software300mm/s². */
      if(visual_velocity_request(Status.forward_mm_s,Status.left_mm_s,TrackHeading,angle.yaw*CHASSIS_HEADING_TEST_YAW_SIGN))
      {
        TrackLastRequest=now;
        if(Tracking.record_count)Tracking.records[(Tracking.record_head+GRAB_TRACKING_TRACE-1U)%GRAB_TRACKING_TRACE].request_age_ms=0;
      }
      if(TrackPickArmed && new_measurement && !TrackPickLowering &&
         (pick_hold_ready || track_pick_hold(now,true)) && track_pick_window(Config[C_TRACK_PICK_TOL_PX]))
      {
        OutsideBase=Axes[2].mm;ObserveX=observation_target(0,Axes[0].mm);ObserveZ=observation_target(1,Axes[1].mm);
        OutsideReferenceValid=OutsidePose=true;
        TrackPickLowering=true;Operation=OP_TRACK_PICK;Scene=SCENE_RAW;
        Status.operation="track_pick";Status.scene="raw";Status.reason="track_pick_descend";
        enter_axis(GRAB_DESCEND,1);
      }
      /* Z completion, not an all-motor standstill or low-plane visual dwell,
       * triggers closure. Descent still requires actual drive completion. */
      if(TrackPickLowering && arrived(1,now))
      {
        if(record)track_log_record(record);
        track_pick_close();
        return;
      }
      if(!TrackPickLowering)
      {
        if(TrackOutput.stable && Status.state!=GRAB_TRACK_FOLLOW) { Status.reason="track_following"; enter(GRAB_TRACK_FOLLOW); }
        else if(!TrackOutput.stable && Status.state==GRAB_TRACK_FOLLOW) { Status.reason="track_matching"; enter(GRAB_TRACK_MATCH); }
      }
    }
    if(record)track_log_record(record);
  }
  track_io(now,TrackMoving);
}

static unsigned track_percentile(const uint32_t bins[101],uint32_t count)
{
  uint32_t sum=0; unsigned i;
  for(i=0;i<101;i++) { sum+=bins[i]; if(sum>=count*.95f)return i; }
  return 100;
}
static void track_status_write(bool debug)
{
  char line[320]; Camera_SnapshotTypeDef camera; Camera_SnapshotGet(&camera);
  (void)snprintf(line,sizeof line,
    "OK grab track state=%s reason=%s mode=%s base_home_degraded=%u color=%u ref=%u valid=%u cx=%u cy=%u dx=%d dy=%d vf=%.2f vl=%.2f vx=%.2f x=%.2f room=%.2f xr=%.2f stop_requested=%u stop_confirmed=%u seq=%lu\r\n",
    States[Status.state],Status.reason,track_mode_name(TrackMode),Status.base_home_degraded?1U:0U,(unsigned)ActiveColor,Tracking.reference_valid?1U:0U,
    (TrackInput.valid && HAL_GetTick()-camera.Data.Tick<=track_config(TrackMode)->grace_ms)?1U:0U,camera.Data.CX,camera.Data.CY,Status.dx,Status.dy,
    Status.forward_mm_s,Status.left_mm_s,TrackMoving?TrackOutput.velocity_mm_s[2]:0,
    Axes[0].position_valid?Axes[0].mm:NAN,TrackOutput.body_room_mm,TrackOutput.x_room_mm,
    Status.stop_requested?1U:0U,Status.stop_confirmed?1U:0U,(unsigned long)Tracking.last_sequence);
  if(debug)(void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION,line,(uint16_t)strlen(line)); else reply(line);
  (void)snprintf(line,sizeof line,
    "OK grab track diag du=%.2f dv=%.2f au=%.2f av=%.2f ff=%.2f,%.2f,%.2f correction=%.2f,%.2f,%.2f latency=%lu batch=%lu feedback_age=%lu forward=%.2f left=%.2f records=%u model=%s\r\n",
    TrackOutput.target_px_s[0],TrackOutput.target_px_s[1],TrackOutput.target_px_s2[0],TrackOutput.target_px_s2[1],
    TrackOutput.feedforward_mm_s[0],TrackOutput.feedforward_mm_s[1],TrackOutput.feedforward_mm_s[2],
    TrackOutput.correction_mm_s[0],TrackOutput.correction_mm_s[1],TrackOutput.correction_mm_s[2],
    (unsigned long)track_config(TrackMode)->latency_ms,(unsigned long)TrackBatchInterval,
    (unsigned long)(Axes[0].position_valid?HAL_GetTick()-Axes[0].position_tick:0),TrackInput.motion_mm[0],TrackInput.motion_mm[1],Tracking.record_count,visual_model_name());
  if(debug)(void)ConsoleTx_Debug(CONSOLE_DEBUG_CAMERA,line,(uint16_t)strlen(line)); else reply(line);
  if(TrackFeedbackFault.address)
  {
    (void)snprintf(line,sizeof line,"OK grab track feedback_fault id=%u kind=%s age_ms=%lu limit_ms=%lu valid=%u\r\n",
      TrackFeedbackFault.address,TrackFeedbackFault.kind,(unsigned long)TrackFeedbackFault.age_ms,
      (unsigned long)TrackFeedbackFault.limit_ms,TrackFeedbackFault.valid?1U:0U);
    if(debug)(void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION,line,(uint16_t)strlen(line)); else reply(line);
  }
  if(!debug)
  {
    if(TrackMotorFault.address)
    {
      (void)snprintf(line,sizeof line,"OK grab track motor_fault id=%u kind=%s flags=0x%02X received_ms=%lu detected_ms=%lu\r\n",
        TrackMotorFault.address,TrackMotorFault.kind,TrackMotorFault.flags,
        (unsigned long)TrackMotorFault.received_ms,(unsigned long)TrackMotorFault.detected_ms);
      reply(line);
    }
    if(TrackMode==GRAB_TRACK_PREDICT)
    {
      (void)snprintf(line,sizeof line,
        "OK grab track estimate fit=%.2f,%.2f,%.2f,%.2f comp=%.2f,%.2f confidence=%.3f,%.3f\r\n",
        Tracking.raw_velocity_px_s[0],Tracking.raw_velocity_px_s[1],Tracking.raw_accel_px_s2[0],Tracking.raw_accel_px_s2[1],
        Tracking.last_compensated_px[0],Tracking.last_compensated_px[1],Tracking.velocity_confidence[0],Tracking.velocity_confidence[1]);
      reply(line);
    }
    float count=(float)Tracking.error_count;
    (void)snprintf(line,sizeof line,"OK grab track stats samples=%lu rms_u=%.3f rms_v=%.3f p95_u=%u p95_v=%u residual_0=%.3f residual_50=%.3f residual_100=%.3f frozen=%u\r\n",
      (unsigned long)Tracking.error_count,count?sqrt(Tracking.error_square[0]/count):0,count?sqrt(Tracking.error_square[1]/count):0,
      track_percentile(Tracking.error_bins[0],Tracking.error_count),track_percentile(Tracking.error_bins[1],Tracking.error_count),
      Tracking.latency_count?sqrt(Tracking.latency_square[0]/Tracking.latency_count):0,
      Tracking.latency_count?sqrt(Tracking.latency_square[1]/Tracking.latency_count):0,
      Tracking.latency_count?sqrt(Tracking.latency_square[2]/Tracking.latency_count):0,Tracking.frozen?1U:0U);
    reply(line);
  }
}
static void track_timing_write(void)
{
  char line[640];Mecanum_Timing_t batches;
  Mecanum_TimingGet(&batches);
  uint32_t total=0,p50=0,p95=0;
  for(unsigned i=0;i<=200;i++) {
    total+=batches.interval_bins[i];
    if(total>=(batches.intervals+1)/2 && !p50)p50=i;
    if(total>=(uint32_t)(((uint64_t)batches.intervals*95+99)/100)){p95=i;break;}
  }
  (void)snprintf(line,sizeof line,
    "OK grab track timing visual=%lu timer=%lu seq=%lu rx=%lu control=%lu parse=%lu rx_samples=%lu clock_ms=1 updates=%lu control_p50=%u control_p95=%u control_max=%lu prediction_p95=%u visual_delay_p95=%u visual_delay_max=%lu computation_p95=%u computation_max=%lu batch_samples=%lu batch_p50=%lu batch_p95=%lu batch_max=%lu batch_duration_max=%lu\r\n",
    (unsigned long)Tracking.visual_updates,(unsigned long)Tracking.timer_updates,(unsigned long)Tracking.control_sequence,
    (unsigned long)Tracking.last_rx_ms,(unsigned long)Tracking.last_control_ms,
    (unsigned long)Tracking.last_sample_ms,(unsigned long)Tracking.visual_delay.count,
    (unsigned long)Tracking.control_timing.count,GrabTracking_Percentile(&Tracking.control_timing,50),
    GrabTracking_Percentile(&Tracking.control_timing,95),(unsigned long)Tracking.control_timing.maximum_ms,
    GrabTracking_Percentile(&Tracking.prediction_timing,95),GrabTracking_Percentile(&Tracking.visual_delay,95),
    (unsigned long)Tracking.visual_delay.maximum_ms,GrabTracking_Percentile(&Tracking.computation_timing,95),
    (unsigned long)Tracking.computation_timing.maximum_ms,(unsigned long)batches.intervals,
    (unsigned long)p50,(unsigned long)p95,(unsigned long)batches.interval_max_ms,(unsigned long)batches.duration_max_ms);
  reply(line);
}

static uint32_t track_crc(const char *text)
{
  uint32_t crc=0xFFFFFFFFUL;
  while(*text)
  {
    crc^=(uint8_t)*text++;
    for(unsigned i=0;i<8;i++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);
  }
  return ~crc;
}
static uint16_t track_age(uint32_t now,uint32_t stamp,bool valid)
{ return valid?(uint16_t)fminf((float)(now-stamp),65534):65535; }
static void track_record_io(TrackIo_t *r,uint32_t now)
{
  Mecanum_Status_t tx;Mecanum_StatusGet(&tx);r->io_valid=0;
  for(unsigned i=0;i<4;i++)
  {
    Mecanum_Feedback_t f={0};(void)Mecanum_FeedbackGet(i,&f);
    r->sent_rpm[i]=tx.sent_rpm[i];r->wheel_raw[i]=f.position_raw;r->feedback_rpm[i]=f.speed_rpm;
    r->wheel_age_ms[i]=track_age(now,f.position_ms,f.position_valid);
    r->wheel_speed_age_ms[i]=track_age(now,f.speed_ms,f.speed_valid);
    if(f.position_valid)r->io_valid|=(uint16_t)(1U<<i);
    if(f.speed_valid)r->io_valid|=(uint16_t)(1U<<(11+i));
  }
  if(tx.sent_velocity_valid)r->io_valid|=16;
  r->sent_age_ms=track_age(now,tx.sent_ms,tx.sent_velocity_valid);
  if(TrackXDispatch.transmitted)r->io_valid|=32;
  if(TrackXDispatch.acknowledged)r->io_valid|=64;
  if(Axes[0].position_valid)r->io_valid|=128;
  if(tx.acknowledged)r->io_valid|=256;
  if(tx.tx_complete)r->io_valid|=512;
  if(TrackXDispatch.requested)r->io_valid|=1024;
  r->x_sent_pulses=TrackXDispatch.sent_pulses;r->x_sent_rpm=TrackXDispatch.sent_rpm;
  r->x_sent_age_ms=track_age(now,TrackXDispatch.sent_ms,TrackXDispatch.transmitted);
  r->x_requested_pulses=TrackXDispatch.requested_pulses;r->x_requested_rpm=TrackXDispatch.requested_rpm;
  r->x_request_age_ms=track_age(now,TrackXDispatch.request_ms,TrackXDispatch.requested);
  r->x_raw=Axes[0].raw;r->x_sent_action=TrackXDispatch.action;
  r->bus_stage=(uint8_t)tx.stage;r->bus_error=(uint8_t)tx.error;r->bus_locked=(uint8_t)tx.locked;
  r->motion_sequence=tx.motion_sequence;r->stop_sequence=tx.stop_sequence;
}
/* Live CRC binds core and contemporaneous IO. Frozen rows retain only the
 * original core: never attach today's IO snapshot to an older ring entry.
 * 640-byte bounded latest-only slot, never the control/stop reply queue. */
static unsigned track_record_format(char *line,unsigned capacity,const GrabTracking_Record_t *r,
                                    const TrackIo_t *io,bool live,uint32_t index)
{
  int n=live?snprintf(line,capacity,"TRACK live id=%lu i=%lu",(unsigned long)TrackLogId,(unsigned long)index):
    snprintf(line,capacity,"TRACK row i=%lu",(unsigned long)index);
  if(n<0 || (unsigned)n>=capacity)return 0;
  unsigned used=(unsigned)n;
  n=snprintf(line+used,capacity-used,
    " t=%lu seq=%lu rx=%lu mode=%s data=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%u",
    (unsigned long)r->time_ms,(unsigned long)r->sequence,(unsigned long)r->sample_ms,track_mode_name((GrabTracking_Mode_t)r->mode),r->error_px[0],r->error_px[1],
    r->target_px_s[0],r->target_px_s[1],r->actual_mm_s[0],r->actual_mm_s[1],r->actual_mm_s[2],
    r->command_mm_s[0],r->command_mm_s[1],r->command_mm_s[2],r->x_mm,r->forward_mm,r->left_mm,r->body_room_mm,
    r->batch_interval_ms,r->feedback_age_ms,r->request_age_ms,r->sent_age_ms,r->flags);
  if(n<0 || (unsigned)n>=capacity-used)return 0;
  used+=(unsigned)n;
  if(live && io)
  {
    n=snprintf(line+used,capacity-used,
      " io=%ld,%ld,%ld,%ld,%lld,%lld,%lld,%lld,%u,%u,%u,%u,%ld,%u,%u,%lld,%u,%u,%u,%u,%u,%lu,%lu,%ld,%u,%u,%ld,%ld,%ld,%ld,%u,%u,%u,%u",
      (long)io->sent_rpm[0],(long)io->sent_rpm[1],(long)io->sent_rpm[2],(long)io->sent_rpm[3],
      (long long)io->wheel_raw[0],(long long)io->wheel_raw[1],(long long)io->wheel_raw[2],(long long)io->wheel_raw[3],
      io->wheel_age_ms[0],io->wheel_age_ms[1],io->wheel_age_ms[2],io->wheel_age_ms[3],
      (long)io->x_sent_pulses,io->x_sent_rpm,io->x_sent_age_ms,(long long)io->x_raw,io->x_sent_action,
      io->bus_stage,io->bus_error,io->bus_locked,io->io_valid,(unsigned long)io->motion_sequence,(unsigned long)io->stop_sequence,
      (long)io->x_requested_pulses,io->x_requested_rpm,io->x_request_age_ms,
      (long)io->feedback_rpm[0],(long)io->feedback_rpm[1],(long)io->feedback_rpm[2],(long)io->feedback_rpm[3],
      io->wheel_speed_age_ms[0],io->wheel_speed_age_ms[1],io->wheel_speed_age_ms[2],io->wheel_speed_age_ms[3]);
    if(n<0 || (unsigned)n>=capacity-used)return 0;
    used+=(unsigned)n;
  }
  n=snprintf(line+used,capacity-used," crc=%08lX\r\n",(unsigned long)track_crc(line));
  if(n<0 || (unsigned)n>=capacity-used)return 0;
  return used+(unsigned)n;
}
static void track_log_record(const GrabTracking_Record_t *r)
{
  char line[640];unsigned length;
  if(!TrackLogEnabled || (TrackLogHaveTick && r->time_ms-TrackLogTick<TrackLogPeriod))return;
  TrackLogHaveTick=true;TrackLogTick=r->time_ms;
  length=track_record_format(line,sizeof line,r,&TrackIo,true,TrackLogIndex++);
  if(length)(void)ConsoleTx_Tracking(line,(uint16_t)length);
}
static void track_trace_process(void)
{
  char line[640]; const GrabTracking_Record_t *r=GrabTracking_Record(&Tracking,TrackTraceIndex);
  if(!TrackTraceSingle && HAL_GetTick()-TrackTraceTick<200)return;
  if(!r)
  {
    static const char end[]="OK grab track trace_end\r\n";
    if(ConsoleTx_Write((const uint8_t *)end,sizeof end-1U))TrackTraceSending=false;
    return;
  }
  unsigned length=track_record_format(line,sizeof line,r,NULL,false,TrackTraceIndex);
  if(length && ConsoleTx_Write((const uint8_t *)line,(uint16_t)length))
  { TrackTraceIndex++; TrackTraceTick=HAL_GetTick(); if(TrackTraceSingle)TrackTraceSending=false; }
}

typedef struct { const char *key; size_t offset; float low,high; bool integer; } TrackKey_t;
#define TRACK_KEY(key,field,low,high,integer) {key,offsetof(GrabTracking_Config_t,field),low,high,integer}
static const TrackKey_t TrackKeys[]={
#define PARAM(id,key,initial,minimum,maximum,flags,phase)
#define TRACK_PARAM(field,key,initial,minimum,maximum,integer) TRACK_KEY(key,field,minimum,maximum,integer),
#include "grab_parameters.def"
#undef TRACK_PARAM
#undef PARAM
};
#undef TRACK_KEY

static bool tracking_start(Camera_ColorTypeDef color,uint8_t slot,GrabTracking_Mode_t mode,
 bool automatic,bool pick,bool observe,float pickup,float speed,float budget)
{
 HWT101_Angle_t angle;Mecanum_Status_t tx;Camera_SnapshotTypeDef camera;
 GrabTracking_Config_t *profile=track_config(mode);
    if((GrabTask_IsBusy() && Status.state!=GRAB_HOLD && Status.state!=GRAB_TRACK_OBSERVE &&
       !(Status.state==GRAB_TRACK_PAUSE && Status.stop_confirmed)) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() || HWT101_Cal_IsBusy() || Turntable_IsBusy())
    { Status.reason="busy";return false; }
    if(!isfinite(Config[C_X_PPM]) || !isfinite(Config[C_X_MIN]) || !isfinite(Config[C_X_MAX]) || !isfinite(Config[C_Z_PPM]))
    { Status.reason="config_missing";return false; }
    if(!observe && !Tracking.reference_valid) { Status.reason="reference_missing";return false; }
    if(!HWT101_Cal_ControlAngleGet(&angle) || HAL_GetTick()-angle.last_update_ms>HWT101_DATA_FRESH_MS)
    { Status.reason="imu_invalid";return false; }
    Mecanum_StatusGet(&tx);
    if(mode!=TrackMode && tracking_mode() && !Status.stop_confirmed)
    {reply("ERR grab track mode_requires_stop; stop and wait for feedback\r\n");return false;}
    if(tx.locked || tx.stop_pending || tx.ack_profile!=MECANUM_ACK_RECEIVE ||
       (tx.sent_velocity_valid && (tx.sent_rpm[0] || tx.sent_rpm[1] || tx.sent_rpm[2] || tx.sent_rpm[3])) ||
       (!tracking_mode() && (MechanicalArm_IsBusy() || Mecanum_IsBusy())))
    { Status.reason="bus_unavailable";return false; }
    if(track_store_can_replace())
    {
      if(!Status.stop_confirmed){Status.reason="stop_unconfirmed";return false;}
      if(!GrabTask_PoseStart(true))return false;
      NextTrack.pending=true;NextTrack.color=color;NextTrack.slot=slot;NextTrack.mode=mode;
      NextTrack.finish_travel=false;
      NextTrack.automatic=automatic;NextTrack.pick=pick;NextTrack.observe=observe;
      NextTrack.pickup=pickup;NextTrack.speed=speed;NextTrack.budget=budget;
      Status.reason="preparing_next_pick";return true;
    }
    Camera_SnapshotGet(&camera);
    /* Explicit observe also re-arms a Pi program restarted after the last B2 request. */
    bool preloaded=NextRawRequested && NextRawColor==(uint8_t)color && camera.RequestActive &&
      camera.RequestFunction==0xB2 && camera.RequestTarget==color;
    if((observe && !preloaded) || !camera.RequestActive || camera.RequestFunction!=0xB2 || camera.RequestTarget!=color)
      if(Camera_MaterialStart(color)!=HAL_OK) { Status.reason="camera_tx";return false; }
    if(!Tracking.reference_valid && !tracking_mode())
    {
      TrackHeading=angle.yaw*CHASSIS_HEADING_TEST_YAW_SIGN;
      ChassisObserver_Init(&TrackObserver,0,0,0); memset(TrackWheel,0,sizeof TrackWheel);
      memset(&TrackPositionVelocity,0,sizeof TrackPositionVelocity);
      TrackCommandCount=0; TrackCommandApplied=0; TrackCommandActive=false;
      memset(TrackCommandUnitsS,0,sizeof TrackCommandUnitsS);
      TrackReady=false; TrackXZeroValid=false;
    }
    if(pick) { Config[C_Z_GRAB]=pickup;Config[C_Z_PICK_SPEED]=speed; }
    ProcClearance.active=false;
    HeldPlaceActive=ProcessedStoreActive=ResumePending=FinalAlignmentRetry=StationAlignRaised=false;
    if(mode!=TrackMode || color!=ActiveColor)GrabTracking_ResetModel(&Tracking);
    TrackMode=mode;
    Operation=OP_ALIGN;RingTest=RingPlaceActive=RingRaise=false;
    CalibrationTask=false;
    StackTest=false;
    TrackAutoStart=automatic;TrackPickArmed=pick;TrackPickLowering=false;TrackEntryBudget=budget;
    TrackStoreActive=TrackStoreReleased=TrackStoreRecorded=TrackHeld=false;TrackStoreSlot=slot;IndexSubmitted=false;
    StoreFinishTravel=false;
    CarPickActive=CarPickClosed=CarPickWaiting=false;
    TrackZPollTick=0;TrackZPollState=false;
    TrackPickHoldActive=false;TrackPickHoldStart=TrackPickHoldLast=0;
    TrackCloseStopping=false;NextRawColor=0;NextRawRequested=false;
    TrackEntryHaveSample=false;TrackEntrySamples=0;
    Status.mode="track"; Status.operation="track"; Status.scene="none"; Status.slot=slot;
    Status.reason=observe?"track_observing":"track_matching"; Status.missing="none"; Status.result="pending";
    Status.stop_requested=Status.stop_confirmed=false; ActiveColor=color;
    /* Idle stops polling: reacquire feedback, retaining the local motion origin
     * and table budget. A cached idle age is not an active tracking failure. */
    TrackReady=false;
    TrackMoving=!observe; TrackVisualPause=false; TrackTraceSending=false;
    TrackPulseValid=false; TrackIoSlot=0; memset(&TrackFeedbackFault,0,sizeof TrackFeedbackFault); memset(&TrackMotorFault,0,sizeof TrackMotorFault);TrackControlSequence=TrackXControlSequence=0; TrackPulseRemainder=0; TrackLastRequest=0; TrackLastStateRead=LastControl=StartTick=HAL_GetTick();
    FeedbackSelected=false; FinishError=false;
    if(!observe) { GrabTracking_Begin(&Tracking); Mecanum_TrackingTiming(true,profile->batch_ms); }
    else { Tracking.frozen=false; Mecanum_TrackingTiming(false,0); }
    enter(observe?GRAB_TRACK_OBSERVE:GRAB_TRACK_MATCH);
    return true;
}
bool GrabTask_TrackedStoreStart(uint8_t color,uint8_t slot,GrabTracking_Mode_t mode)
{
 if((TrackHeld || TrackStoreActive) && !track_store_can_replace()){Status.reason="track_material_not_held";return false;}
 if(color<1 || color>6 || mode>GRAB_TRACK_FEEDBACK){Status.reason="invalid_color_or_mode";return false;}
 if(!slot)slot=Turntable_FindEmpty();
 const char *missing=track_store_missing(slot,false);
 if(missing){Status.reason="config_missing";Status.missing=missing;return false;}
 if(!isfinite(Config[C_Z_GRAB]) || Config[C_Z_GRAB]<Config[C_Z_MIN] || Config[C_Z_GRAB]>Config[C_Z_MAX] ||
    Config[C_Z_LIFT]<=Config[C_Z_GRAB] || Config[C_Z_LIFT]>Config[C_Z_MAX])
 {Status.reason="config_missing";Status.missing="z_grab";return false;}
 return tracking_start((Camera_ColorTypeDef)color,slot,mode,true,true,true,Config[C_Z_GRAB],Config[C_Z_PICK_SPEED],50);
}
bool GrabTask_StoreFinishTravel(void)
{
  if(!RearReferenceValid || !GrabTask_IsBusy() || ResumePending)return false;
  if(NextTrack.pending && NextTrack.pick && NextTrack.slot)
  {NextTrack.finish_travel=true;return true;}
  if((Operation!=OP_STORE && !(TrackPickArmed && TrackStoreSlot)) ||
     Status.state==GRAB_STOPPING)return false;
  StoreFinishTravel=true;return true;
}
static bool track_command(unsigned count,char *tokens[])
{
  char line[160]; unsigned i;
  if(count>=3 && !strcmp(tokens[2],"log"))
  {
    if(count==4 && !strcmp(tokens[3],"stop"))
    {
      TrackLogEnabled=false; /* Started line drains; unstarted diagnostics yield to this reply. */
      (void)snprintf(line,sizeof line,"OK grab track log_end id=%lu count=%lu\r\n",(unsigned long)TrackLogId,(unsigned long)TrackLogIndex);reply(line);
    }
    else if((count==4 || count==5) && !strcmp(tokens[3],"start"))
    {
      char *end=NULL;unsigned long period=count==5?strtoul(tokens[4],&end,10):50;
      if(period<50 || period>1000 || (count==5 && (!*tokens[4] || *end)))
      { reply("ERR grab track log period: 50..1000 ms\r\n");return true; }
      if(!TrackLogEnabled)
      {
        if(++TrackLogId==0)TrackLogId=1;
        TrackLogIndex=0;TrackLogHaveTick=false;ConsoleTx_TrackingCancel();
      }
      TrackLogPeriod=(uint32_t)period;TrackLogEnabled=true;
      (void)snprintf(line,sizeof line,"OK grab track log_begin id=%lu period_ms=%lu next=%lu\r\n",(unsigned long)TrackLogId,(unsigned long)TrackLogPeriod,(unsigned long)TrackLogIndex);reply(line);
    }
    else if(count==3 || (count==4 && !strcmp(tokens[3],"status")))
    {
      (void)snprintf(line,sizeof line,"OK grab track log_status enabled=%u id=%lu period_ms=%lu count=%lu\r\n",TrackLogEnabled,(unsigned long)TrackLogId,(unsigned long)TrackLogPeriod,(unsigned long)TrackLogIndex);reply(line);
    }
    else reply("ERR grab track log start [50..1000] | stop | status\r\n");
    return true;
  }
  if(count==3 && !strcmp(tokens[2],"status")) { track_status_write(false);track_timing_write(); return true; }
  if(count==3 && !strcmp(tokens[2],"stop")) { GrabTask_Stop(); track_status_write(false); return true; }
  if((count==3 || count==4) && !strcmp(tokens[2],"trace"))
  {
    if(!Tracking.frozen || TrackMoving)reply("ERR grab track trace_running; stop first\r\n");
    else
    {
      char *end; unsigned long index=count==4?strtoul(tokens[3],&end,10):0;
      if(count==4 && (*end || !*tokens[3] || index>=Tracking.record_count))reply("ERR grab track trace_index\r\n");
      else
      {
        TrackTraceIndex=(unsigned)index; TrackTraceSingle=count==4;
        TrackTraceSending=true; TrackTraceTick=HAL_GetTick()-200U;
        if(count==3)
        { (void)snprintf(line,sizeof line,"OK grab track trace_begin count=%u\r\n",Tracking.record_count); reply(line); }
      }
    }
    return true;
  }
  if(((count==4 || count==5) && !strcmp(tokens[2],"get")) || ((count==5 || count==6) && !strcmp(tokens[2],"set")))
  {
    void *value;
    bool setting=!strcmp(tokens[2],"set");
    GrabTracking_Mode_t mode=GRAB_TRACK_PREDICT;
    if(count==(setting?6U:5U) && !track_mode_parse(tokens[count-1],&mode))
    {reply("ERR grab track profile: predict|visual|feedback\r\n");return true;}
    for(i=0;i<sizeof TrackKeys/sizeof TrackKeys[0];i++)if(!strcmp(tokens[3],TrackKeys[i].key))break;
    if(i==sizeof TrackKeys/sizeof TrackKeys[0]) { reply("ERR grab track key\r\n"); return true; }
    if((TrackKeys[i].offset==offsetof(GrabTracking_Config_t,body_accel_mm_s2) ||
        TrackKeys[i].offset==offsetof(GrabTracking_Config_t,x_accel_mm_s2)) && mode!=GRAB_TRACK_FEEDBACK)
    {reply("ERR grab track key: feedback profile only\r\n");return true;}
    value=(char *)track_config(mode)+TrackKeys[i].offset;
    if(setting)
    {
      char *end; float parsed=strtof(tokens[4],&end);
      if(TrackMoving || (GrabTask_IsBusy() && Status.state!=GRAB_TRACK_OBSERVE)) { reply("ERR grab track busy\r\n"); return true; }
      if(end==tokens[4] || *end || !isfinite(parsed) || parsed<TrackKeys[i].low || parsed>TrackKeys[i].high ||
         (TrackKeys[i].integer && floorf(parsed)!=parsed)) { reply("ERR grab track value\r\n"); return true; }
      if(TrackKeys[i].integer)*(uint32_t *)value=(uint32_t)parsed; else *(float *)value=parsed;
    }
    (void)snprintf(line,sizeof line,"OK grab track config key=%s value=%.6g profile=%s\r\n",TrackKeys[i].key,
      TrackKeys[i].integer?(float)*(uint32_t *)value:*(float *)value,track_mode_name(mode)); reply(line); return true;
  }
  if((count==3 || count==4) && !strcmp(tokens[2],"reference"))
  {
    float budget=50;
    if(count==4)
    {
      char *end; budget=strtof(tokens[3],&end);
      if(*end || !isfinite(budget) || budget<0 || budget>50)
      { reply("ERR grab track reference budget: 0..50 mm\r\n"); return true; }
    }
    if(TrackMoving || (GrabTask_IsBusy() && Status.state!=GRAB_TRACK_OBSERVE && Status.state!=GRAB_HOLD &&
       !(Status.state==GRAB_TRACK_PAUSE && Status.stop_confirmed)) || !axis_fresh(0,HAL_GetTick()) || !TrackObserver.feedback_valid ||
       fabsf(TrackInput.actual_mm_s[0])>2 || fabsf(TrackInput.actual_mm_s[1])>2 || fabsf(Axes[0].signed_speed)>2)
    { reply("ERR grab track reference; observe at stopped pose first\r\n"); return true; }
    /* A reset clears local coordinates, not the physical table clearance.
     * Offset the original 50 mm anchor by already-consumed approach travel. */
    GrabTracking_Reference(&Tracking,TrackInput.motion_mm[1]-(50-budget));
    (void)snprintf(line,sizeof line,"OK grab track reference accepted budget_mm=%.3g\r\n",(double)budget);
    reply(line); track_status_write(false); return true;
  }
  if(count==4 && !strcmp(tokens[2],"store"))
  {
    if(strlen(tokens[3])!=1 || tokens[3][0]<'0' || tokens[3][0]>'3' ||
       !track_store_start((uint8_t)(tokens[3][0]-'0')))
      reply("ERR grab track store rejected; inspect grab status\r\n");
    else reply("OK grab track store accepted\r\n");
    status_write(false);return true;
  }
  if(count>=3 && count<=6 && (!strcmp(tokens[2],"observe") || !strcmp(tokens[2],"start") ||
     !strcmp(tokens[2],"begin") || !strcmp(tokens[2],"pick") || !strcmp(tokens[2],"pickstore") || !strcmp(tokens[2],"visual") || !strcmp(tokens[2],"feedback")))
  {
    bool store=!strcmp(tokens[2],"pickstore");uint8_t slot=0;
    if((TrackHeld || TrackStoreActive) && !track_store_can_replace())
    {reply("ERR grab track material_held; continue with grab track store <slot>\r\n");return true;}
    GrabTracking_Mode_t mode=!strcmp(tokens[2],"visual")?GRAB_TRACK_VISUAL:
      !strcmp(tokens[2],"feedback")?GRAB_TRACK_FEEDBACK:GRAB_TRACK_PREDICT;
    if(store)
    {
      if(count<5 || strlen(tokens[4])!=1 || tokens[4][0]<'0' || tokens[4][0]>'3')
      {reply("ERR grab track pickstore <color> <slot 0..3> [predict|visual|feedback]\r\n");return true;}
      slot=(uint8_t)(tokens[4][0]-'0');if(!slot)slot=Turntable_FindEmpty();
      mode=GRAB_TRACK_FEEDBACK;
      if(count==6 && !track_mode_parse(tokens[5],&mode)){reply("ERR grab track profile\r\n");return true;}
      const char *missing=track_store_missing(slot,false);
      if(missing){(void)snprintf(line,sizeof line,"ERR grab track store %s\r\n",missing);reply(line);return true;}
    }
    bool automatic=store || !strcmp(tokens[2],"begin") || !strcmp(tokens[2],"pick") || mode!=GRAB_TRACK_PREDICT;
    bool pick=store || !strcmp(tokens[2],"pick"), observe=!strcmp(tokens[2],"observe") || automatic;
    float pickup=Config[C_Z_GRAB],speed=Config[C_Z_PICK_SPEED],budget=50;
    if((!automatic && count>4) || (!pick && count>5))
    { reply("ERR grab track syntax\r\n");return true; }
    if(count>=5 && !store)
    {
      char *end;float value=strtof(tokens[4],&end);
      if(end==tokens[4] || *end || !isfinite(value)) { reply("ERR grab track value\r\n");return true; }
      if(pick)pickup=value;else budget=value;
    }
    if(pick && count==6 && !store)
    { char *end;speed=strtof(tokens[5],&end);if(end==tokens[5] || *end) { reply("ERR grab track value\r\n");return true; } }
    if(budget<0 || budget>50) { reply("ERR grab track reference budget: 0..50 mm\r\n");return true; }
    if(pick && (!isfinite(Config[C_Z_MIN]) || !isfinite(Config[C_Z_MAX]) ||
       !isfinite(pickup) || pickup<Config[C_Z_MIN] || pickup>Config[C_Z_MAX] ||
       !isfinite(Config[C_Z_LIFT]) || Config[C_Z_LIFT]<=pickup || Config[C_Z_LIFT]>Config[C_Z_MAX] ||
       !config_value_valid(C_Z_PICK_SPEED,speed) || speed*Config[C_Z_PPM]*60/3200<1 || speed*Config[C_Z_PPM]*60/3200>3000))
    { reply("ERR grab track pickup height/speed; inspect z_grab z_lift z_pick_speed\r\n");return true; }
    Camera_ColorTypeDef color=tracking_mode()?ActiveColor:CAMERA_COLOR_BLUE;
    if(count>=4)
    {if(strlen(tokens[3])!=1 || tokens[3][0]<'1' || tokens[3][0]>'6'){reply("ERR grab track color\r\n");return true;}color=(Camera_ColorTypeDef)(tokens[3][0]-'0');}
    if(tracking_start(color,slot,mode,automatic,pick,observe,pickup,speed,budget))
    {if(automatic){(void)snprintf(line,sizeof line,"OK grab track %s accepted phase=%s mode=%s\r\n",tokens[2],NextTrack.pending?"prepare":"observe",track_mode_name(mode));reply(line);}else reply(observe?"OK grab track observe accepted motion=none\r\n":"OK grab track start accepted\r\n");}
    else {snprintf(line,sizeof line,"ERR grab track %s\r\n",Status.reason);reply(line);}
    return true;
  }
  reply("ERR grab track syntax\r\n"); return true;
}
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
  if((HaveCapture || (RingTest && RingStayLow && s.state==GRAB_HOLD && !strcmp(s.mode,"align"))) &&
     s.state!=GRAB_VISION_GRACE && !strcmp(s.vision_state,"Ok"))
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
      "OK grab state=%s mode=%s reason=%s missing=%s ref_cause=%s base_home_degraded=%u operation=%s scene=%s slot=%u result=%s stop_requested=%u stop_confirmed=%u elapsed=%lu\r\n",
      s.state_name,s.mode,s.reason,missing,s.reference_cause,s.base_home_degraded?1U:0U,s.operation,s.scene,(unsigned)s.slot,s.result,s.stop_requested?1U:0U,s.stop_confirmed?1U:0U,(unsigned long)s.elapsed_ms);
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
      "OK grab state=homing mode=%s operation=%s reason=%s ref_cause=%s base_home_degraded=%u home_scope=%s axis=%s step=%u home_flags=%s raw=%s motor_flags=%s stable=%u enable_recovery=%u home_started=%u home_verified=%u stop_requested=%u stop_confirmed=0 elapsed=%lu\r\n",
      s.mode,BootBaseOnly?"pose":"home",s.reason,s.reference_cause,s.base_home_degraded?1U:0U,BootBaseOnly?"base":"all",names[BootAxisIndex],BootStep,home,raw,motor,BootGood,
      BootEnableRecovery?1U:0U,(unsigned)BootStartedMask,(unsigned)(BootBaseOnly?(BootVerifiedMask&4U):BootVerifiedMask),
      s.stop_requested?1U:0U,(unsigned long)s.elapsed_ms);
    if(debug && strlen(line)>319U)
      (void)snprintf(line,sizeof line,
        "OK grab state=homing mode=%s operation=%s reason=%s base_home_degraded=%u home_scope=%s axis=%s home_flags=%s raw=%s motor_flags=%s stable=%u stop_requested=%u stop_confirmed=0 elapsed=%lu\r\n",
        s.mode,BootBaseOnly?"pose":"home",s.reason,s.base_home_degraded?1U:0U,
        BootBaseOnly?"base":"all",names[BootAxisIndex],home,raw,motor,BootGood,
        s.stop_requested?1U:0U,(unsigned long)s.elapsed_ms);
    if(debug) (void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION,line,(uint16_t)strlen(line));
    else reply(line);
    return;
  }
  (void)snprintf(line,sizeof line,
    "OK grab state=%s mode=%s reason=%s missing=%s ref_cause=%s base_home_degraded=%u operation=%s scene=%s slot=%u result=%s x=%.3f xt=%.3f z=%.3f zt=%.3f b=%.2f bt=%.2f dx=%s dy=%s protocol=B2 age_source=rx rx_seq=%lu vision=%s color=%u model=%s age=%lu vf=%.3f vl=%.3f stop_requested=%u stop_confirmed=%u elapsed=%lu recovery_used=%lu recovery_count=%lu recovery_left_ms=%s loss_ms=%lu loss_max_ms=%lu grace_count=%lu\r\n",
    s.state_name,s.mode,s.reason,s.missing,s.reference_cause,s.base_home_degraded?1U:0U,s.operation,s.scene,(unsigned)s.slot,s.result,s.x_mm,s.x_target_mm,s.z_mm,s.z_target_mm,
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
        (void)snprintf(line,sizeof line,"OK grab state=%s mode=%s reason=%s base_home_degraded=%u protocol=B2 age_source=rx rx_seq=%lu vision=%s dx=%s dy=%s color=%u stop_requested=%u stop_confirmed=%u recovery_used=%lu recovery_count=%lu recovery_left_ms=%s loss_ms=%lu loss_max_ms=%lu grace_count=%lu\r\n",
          s.state_name,s.mode,s.reason,s.base_home_degraded?1U:0U,(unsigned long)s.rx_seq,s.vision_state,dx,dy,(unsigned)target_color(),
          s.stop_requested?1U:0U,s.stop_confirmed?1U:0U,(unsigned long)s.recovery_used,(unsigned long)s.recovery_count,remaining,
          (unsigned long)s.loss_ms,(unsigned long)s.loss_max_ms,(unsigned long)s.grace_count);
      else
        (void)snprintf(line,sizeof line,"OK grab state=%s mode=%s reason=%s operation=%s scene=%s slot=%u result=%s x=%.3f xt=%.3f z=%.3f zt=%.3f ref_cause=%s base_home_degraded=%u color=%u stop_requested=%u stop_confirmed=%u\r\n",
          s.state_name,s.mode,s.reason,s.operation,s.scene,(unsigned)s.slot,s.result,s.x_mm,s.x_target_mm,
          s.z_mm,s.z_target_mm,s.reference_cause,s.base_home_degraded?1U:0U,(unsigned)target_color(),s.stop_requested?1U:0U,s.stop_confirmed?1U:0U);
      length=strlen(line);
    }
    (void)ConsoleTx_Debug(CONSOLE_DEBUG_VISION,line,(uint16_t)length);
  }
  else reply(line);
}
bool GrabTask_RingManualAllowed(void)
{ return (RingTest || Operation==OP_ALIGN) && Status.state==GRAB_HOLD && Status.stop_confirmed; }
bool GrabTask_RingTakePrepare(uint8_t *ring)
{
  if(!ring || !RingTest || !GrabTask_RingManualAllowed() || strcmp(Status.result,"aligned") || TrackHeld)
  {Status.reason="ring_alignment_required";return false;}
  if(!RearReferenceValid){Status.reason="rear_reference_missing";return false;}
  if(!station_height(C_Z_PROC_PLACE))return false;
  if(Config[C_Z_PROC_PLACE]>=Axes[1].mm){Status.reason="ring_release_height";return false;}
  /* A completed test may have stopped recognition or raised Z. Revalidate at
   * the actual held height; do not replay its former lowering target. */
  if(request_target()!=HAL_OK){Status.reason="camera_tx";return false;}
  RingAlignZ=observation_target(1,Axes[1].mm);RingStayLow=true;
  StationAlignRaised=fabsf(RingAlignZ-Config[C_Z_MAX])<=Config[C_POS_TOL];
  RingReleaseZ=Config[C_Z_PROC_PLACE];Scene=SCENE_ROUGH;
  AlignedPlace.valid=false;EmptyPlaceAlignActive=true;StackTest=false;
  HaveCapture=FreshVision=false;RequestTick=LastControl=HAL_GetTick();LastSequence=0;
  RingLowerSent=RingApproachStarted=false;stable_reset();
  Status.operation="empty_ring_align";Status.scene="rough";
  Status.reason="ring_pickplace_verify";Status.result="pending";
  *ring=RingNumber;enter(GRAB_ACQUIRE);return true;
}
static bool ring_place_start(float z)
{
  if(!RingTest || !GrabTask_RingManualAllowed()) { Status.reason="ring_alignment_required";return false; }
  if(!isfinite(z) || z<Config[C_Z_MIN] || z>Config[C_Z_MAX] || z>=Axes[1].mm)
  { Status.reason="ring_release_height";return false; }
  RingReleaseZ=z;RingReturnZ=Axes[1].mm;
  /* Encoder noise near the verified upper stop must not turn a return-to-top
   * into a distinct release target requiring the driver's reached bit. */
  if(fabsf(RingReturnZ-Config[C_Z_MAX])<=Config[C_POS_TOL]) RingReturnZ=Config[C_Z_MAX];
  RingPlaceActive=true;
  Status.operation="ring_place";Status.mode="fixed";Status.result="pending";
  Status.reason="ring_place_running";StartTick=HAL_GetTick();
  enter_axis(GRAB_DESCEND,1);return true;
}
static bool manual_height_start(float z)
{
  if(!GrabTask_RingManualAllowed() &&
     !(Operation==OP_ALIGN && Status.state==GRAB_MANUAL_Z && Status.stop_confirmed))
  { Status.reason="static_alignment_required";return false; }
  if(!isfinite(z) || z<Config[C_Z_MIN] || z>Config[C_Z_MAX])
  { Status.reason="stack_height_outside_limits";return false; }
  Camera_RequestStop();HaveCapture=FreshVision=false;
  ManualZTarget=z;Status.operation="stack_height";Status.mode="align";
  Status.reason="stack_height_running";Status.result="pending";StartTick=HAL_GetTick();
  enter(GRAB_MANUAL_Z);return true;
}
static void calibration_status_write(void)
{
  GrabCalibration_CalibrationDataTypeDef data; char line[224];
  const char *phase=CalibrationTask && Status.state!=GRAB_CALIBRATION?States[Status.state]:GrabCalibration_StateNameGet();
  if(GrabCalibration_DataGet(&data))
    (void)snprintf(line,sizeof line,"OK grab calib phase=%s ref=%u calibrated=1 reason=%s fdx=%.6f fdy=%.6f xdx=%.6f xdy=%.6f\r\n",
      phase,(unsigned)CalibrationReferenceConfirmed,CalibrationTask?Status.reason:GrabCalibration_ErrorNameGet(),
      (double)data.ForwardDxPerMm,(double)data.ForwardDyPerMm,(double)data.XDxPerDegree,(double)data.XDyPerDegree);
  else
    (void)snprintf(line,sizeof line,"OK grab calib phase=%s ref=%u calibrated=0 reason=%s fdx=na fdy=na xdx=na xdy=na\r\n",
      phase,(unsigned)CalibrationReferenceConfirmed,CalibrationTask?Status.reason:GrabCalibration_ErrorNameGet());
  reply(line);
}

bool GrabTask_Command(unsigned count,char *tokens[])
{
  unsigned i; size_t length=0;
  if(!tokens || !count || strcmp(tokens[0],"grab")) return false;
  for(i=0;i<count;i++) { if(!tokens[i]) { reply("ERR grab syntax\r\n"); return true; } length+=strlen(tokens[i])+(i?1:0); }
  if(count>6 || length>63) { reply("ERR grab syntax\r\n"); return true; }
  if(count>=2 && !strcmp(tokens[1],"track")) return track_command(count,tokens);
  if(count==2 && !strcmp(tokens[1],"status")) status_write(false);
  else if(count==3 && !strcmp(tokens[1],"calib"))
  {
    if(!strcmp(tokens[2],"status")) calibration_status_write();
    else if(!strcmp(tokens[2],"ref"))
      reply(GrabTask_CalibrationReferenceSet()?"OK grab calib ref confirmed\r\n":"ERR grab calib busy\r\n");
    else if(strlen(tokens[2])==1 && tokens[2][0]>='1' && tokens[2][0]<='6')
    {
      if(GrabTask_CalibrationStart((uint8_t)(tokens[2][0]-'0')))reply("OK grab calib accepted\r\n");
      else reply("ERR grab calib rejected; inspect grab status\r\n");
      status_write(false);
    }
    else reply("ERR format: grab calib ref|status|<1..6>\r\n");
  }
  else if(count==3 && !strcmp(tokens[1],"status"))
  {
    unsigned axis; uint32_t now=HAL_GetTick(); char line[224];
    if(!strcmp(tokens[2],"base")) axis=2;
    else if(!strcmp(tokens[2],"x")) axis=0;
    else if(!strcmp(tokens[2],"z")) axis=1;
    else { reply("ERR grab status base|x|z\r\n"); return true; }
    (void)snprintf(line,sizeof line,
      "OK grab axis=%s state=%s p=%.3f target=%.3f flags=0x%02X fresh=%u quiet=%u reached=%u speed=%.3f commanded=%u stable_ms=%lu\r\n",
      tokens[2],States[Status.state],Axes[axis].position_valid?Axes[axis].mm:NAN,
      Axes[axis].commanded?Axes[axis].target:NAN,(unsigned)Axes[axis].flags,
      axis_fresh(axis,now)?1U:0U,Axes[axis].quiet?1U:0U,arrived(axis,now)?1U:0U,
      Axes[axis].speed,Axes[axis].commanded?1U:0U,
      Axes[axis].quiet?(unsigned long)(now-Axes[axis].quiet_since):0UL);
    reply(line);
  }
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
  else if(count>=4 && count<=6 && !strcmp(tokens[1],"align"))
  {
    bool ring=!strcmp(tokens[2],"ring"),color=!strcmp(tokens[2],"color");
    uint8_t target;float z=0;char line[192];
    bool stay=count==6 || (ring && count==4);
    if((!ring && !color) || strlen(tokens[3])!=1 || tokens[3][0]<'1' ||
       tokens[3][0]>(ring?'3':'6') || (count==6 && strcmp(tokens[5],"stay")))
    { reply("ERR format: grab align color <1..6>|ring <1..3> [z_mm] [stay]\r\n");return true; }
    target=(uint8_t)(tokens[3][0]-'0'+(ring?9:0));
    if(count>=5)
    {
      char *end;z=strtof(tokens[4],&end);
      if(end==tokens[4] || *end || !isfinite(z)) { reply("ERR align height\r\n");return true; }
    }
    if(GrabTask_AlignStart(target,z,stay))
    {
      (void)snprintf(line,sizeof line,"OK grab %s accepted target=%s id=%u align_z=%.1f return_top=%u error_source=dx_dy\r\n",
        Status.operation,ring?"digit_ring":"color",(unsigned)(target-(ring?9:0)),z,stay?0U:1U);reply(line);
    }
    else reply("ERR grab align rejected; inspect grab status\r\n");
    status_write(false);
  }
  else if(count>=3 && !strcmp(tokens[1],"stack"))
  {
    bool accepted=false;
    if(count==4 && !strcmp(tokens[2],"align"))
    {
      if(strlen(tokens[3])==1 && tokens[3][0]>='1' && tokens[3][0]<='6')
      {
        accepted=GrabTask_AlignStart((uint8_t)(tokens[3][0]-'0'),Config[C_Z_MAX],false);
        if(accepted)
        {
          StackTest=true;Status.operation="stack_align";Status.reason="stack_center_align";
          reply("OK grab stack_align accepted error_source=dx_dy compensation=vision\r\n");
          status_write(false);return true;
        }
      }
      else Status.reason="invalid_color";
    }
    else if(count==4 && !strcmp(tokens[2],"lower"))
    {
      char *end;float z=strtof(tokens[3],&end);
      if(end!=tokens[3] && !*end)accepted=manual_height_start(z);
      else Status.reason="invalid_stack_height";
    }
    else if(count==3 && !strcmp(tokens[2],"lift"))accepted=manual_height_start(Config[C_Z_MAX]);
    else if(count==3 && !strcmp(tokens[2],"release"))
    {
      if(GrabTask_RingManualAllowed())
      {
        Camera_RequestStop();HaveCapture=FreshVision=false;
        Arm_GripperSet(ARM_GRIPPER_OPEN);Status.operation="stack_release";
        Status.reason="stack_released";Status.result="released";accepted=true;
      }
      else Status.reason="static_alignment_required_or_height_moving";
    }
    else { reply("ERR format: grab stack align <color> | lower <z_mm> | release | lift\r\n");return true; }
    reply(accepted?"OK grab stack accepted\r\n":"ERR grab stack rejected; inspect grab status\r\n");
    status_write(false);
  }
  else if(count>=3 && !strcmp(tokens[1],"ring"))
  {
    if(count>=3 && count<=6 && !strcmp(tokens[2],"align"))
    {
      uint8_t ring=1;char line[160];float z=0;
      if(count>=4) {
        if(tokens[3][0]<'1' || tokens[3][0]>'3' || tokens[3][1]) {
          reply("ERR format: grab ring align <1|2|3>\r\n");return true;
        }
        ring=(uint8_t)(tokens[3][0]-'0');
      }
      if(count==6 && strcmp(tokens[5],"stay")) {
        reply("ERR format: grab ring align <1|2|3> <z_mm> stay\r\n");return true;
      }
      if(count>=5) {
        char *end;z=strtof(tokens[4],&end);
        if(end==tokens[4] || *end || !isfinite(z)) { reply("ERR ring align_z\r\n");return true; }
      }
      if(isfinite(Config[C_Z_MIN]) && isfinite(Config[C_Z_MAX]) &&
         (z<Config[C_Z_MIN] || z>Config[C_Z_MAX])) { reply("ERR ring align_z outside z limits\r\n");return true; }
      if(GrabTask_AlignStart((uint8_t)(9+ring),z,count<=4 || count==6))
      {
        (void)snprintf(line,sizeof line,"OK grab ring_align accepted target=digit_ring ring=%u align_z=%.1f number_recognition=1 return_top=%u error_source=dx_dy\r\n",ring,z,RingStayLow?0U:1U);reply(line);
      }
      else reply("ERR grab ring_align rejected; inspect grab status\r\n");
    }
    else if(count==4 && !strcmp(tokens[2],"place"))
    {
      char *end;float z=strtof(tokens[3],&end);
      if(end==tokens[3] || *end || !ring_place_start(z))reply("ERR grab ring_place rejected; inspect grab status\r\n");
      else reply("OK grab ring_place accepted\r\n");
    }
    else reply("ERR format: grab ring align [1|2|3] [z_mm] [stay] | grab ring place <z_mm>\r\n");
    status_write(false);
  }
  else if(count==3 && !strcmp(tokens[1],"place") && !strcmp(tokens[2],"car"))
  {
    if(start_operation("fixed",CAMERA_COLOR_RED,OP_CAR_PLACE,SCENE_RAW,0))
    { Status.scene="car"; reply("OK grab car_place accepted\r\n"); }
    else reply("ERR grab car_place rejected; inspect grab status\r\n");
    status_write(false);
  }
  else if(count==3 && !strcmp(tokens[1],"pose"))
  {
    if(strcmp(tokens[2],"rear") && strcmp(tokens[2],"work")) reply("ERR grab pose rear|work\r\n");
    else if(!GrabTask_PoseStart(!strcmp(tokens[2],"work"))) reply("ERR grab pose rejected; inspect grab status\r\n");
    else reply("OK grab pose accepted\r\n");
    status_write(false);
  }
  else if(count==2 && !strcmp(tokens[1],"rehome"))
  {
    Mecanum_Status_t bus;
    Mecanum_StatusGet(&bus);
    if((Status.state!=GRAB_IDLE && Status.state!=GRAB_COMPLETE && Status.state!=GRAB_ERROR) || MechanicalArm_IsBusy() ||
       ChassisRoute_IsBusy() ||
       ChassisMotion_IsBusy() || Mecanum_IsBusy() || HWT101_Cal_IsBusy() || Turntable_IsBusy() ||
       bus.locked || bus.stop_pending || bus.ack_profile!=MECANUM_ACK_RECEIVE)
      reply("ERR grab rehome rejected; stop tasks and inspect bus status\r\n");
    else { boot_home_start(false); reply("OK grab rehome accepted; Z/X/Base verification pending\r\n"); status_write(false); }
  }
  else if(count==3 && !strcmp(tokens[1],"home") && !strcmp(tokens[2],"accept"))
  {
    Mecanum_Status_t bus;Mecanum_StatusGet(&bus);
    if(Status.state!=GRAB_ERROR || !BaseHomeAcceptAvailable || (BootVerifiedMask&3U)!=3U ||
       MechanicalArm_IsBusy() || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() ||
       Mecanum_IsBusy() || HWT101_Cal_IsBusy() || Turntable_IsBusy() || bus.locked ||
       bus.stop_pending || bus.ack_profile!=MECANUM_ACK_RECEIVE)
      reply("ERR grab home accept rejected; inspect grab status\r\n");
    else {
      /* Explicit acceptance retries fresh feedback, never HOME or X/Z motion.
       * Preserve startup/manual scope so startup still arms IMU verification. */
      BootActive=BootAcceptBase=true;BootStopping=BootFailed=BootLaunching=false;
      BootResumePose=false;BootStopUnconfirmed=false;BootAbortMask=BootAbortSentMask=0;
      BootHomeTick[2]=HAL_GetTick();boot_axis_select(2,HAL_GetTick(),true);
      BootBaseDegraded=true; /* Abort may clear bit3; this remains an approximate anchor. */
      Status.stop_requested=Status.stop_confirmed=false;
      Status.reason="home_degraded_check";StartTick=HAL_GetTick();enter(GRAB_HOMING);
      reply("OK grab home accept accepted; fresh Base feedback pending\r\n");
    }
    status_write(false);
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
    if(!(align?GrabTask_AlignStart((uint8_t)color,Config[C_Z_OBSERVE],true):
         start_operation(tokens[2],color,OP_STORE,SCENE_RAW,slot)))
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
  else if((count==4 || count==5) && !strcmp(tokens[1],"take") && !strcmp(tokens[3],"work"))
  {
    float depth=Config[C_CAR_PICK_DROP_MM];
    if(count==5){char *end;depth=strtof(tokens[4],&end);if(end==tokens[4] || *end){reply("ERR grab take depth\r\n");return true;}}
    if(strlen(tokens[2])!=1 || tokens[2][0]<'1' || tokens[2][0]>'3')reply("ERR grab slot; use 1..3\r\n");
    else if(!car_pick_start((uint8_t)(tokens[2][0]-'0'),depth))reply("ERR grab take work rejected; inspect grab status\r\n");
    else reply("OK grab take work accepted\r\n");
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
    { if(Config[i]!=value && (i==C_X_MIN || i==C_Z_CAR_LIFT || i==C_X_PPM || i==C_Z_PPM))
        Config[C_HEAD_PROC_GAIN]=Config[C_HEAD_TEMP_GAIN]=Config[C_HEAD_LINE_GAIN]=NAN;
      Config[i]=value; if(i>=C_J00 && i<=C_J12) VisualModelCustom=true; Status.missing=configuration_missing(Status.mode); if(!Status.missing) Status.missing="none"; reply("OK grab set\r\n"); }
  }
  else reply("ERR grab syntax\r\n");
  return true;
}
