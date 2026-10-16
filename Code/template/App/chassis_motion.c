#include "chassis_motion.h"
#include "chassis_config.h"
#include "chassis_localization.h"
#include "chassis_route.h"
#include "mecanum_chassis.h"
#include "hwt101_calibration.h"
#include "mechanical_arm.h"
#include "GrabTask.h"
#include <math.h>
#include <string.h>

static ChassisMotion_Status_t Motion;
static ChassisMotion_Profile_t SelectedProfile;
bool ChassisMotion_ProfileSet(ChassisMotion_Profile_t profile)
{
  if(ChassisMotion_IsBusy() || (profile!=CHASSIS_PROFILE_LINEAR && profile!=CHASSIS_PROFILE_COSINE))return false;
  SelectedProfile=profile;return true;
}
ChassisMotion_Profile_t ChassisMotion_ProfileGet(void){return SelectedProfile;}
/* Straight heading PD outputs rad/s. D is measured yaw rate, not target error
 * differences; pure turns keep their independent proportional law. */
static struct {
  float target_yaw_deg, output_limit, output_rad_s;
  float rate_yaw_deg, rate_deg_s;
  uint32_t rate_tick;
  bool rate_seeded;
} HeadingPid;
static uint8_t DriverAcceleration;
static uint32_t TaskStopSequence;
static bool HoldTask;
static ChassisMotion_HoldStatus_t HoldStatus;
static bool request_stop(const char *reason, bool error);
static bool in_target(void);
static void hold_status_refresh(void);
static void heading_rate_reset(void)
{
  HeadingPid.rate_yaw_deg=Motion.current_deg;
  HeadingPid.rate_tick=Motion.heading_sample_tick;
  HeadingPid.rate_deg_s=0;
  HeadingPid.rate_seeded=Motion.heading_valid;
}
static float heading_update(float current)
{
  float error = fmodf(HeadingPid.target_yaw_deg-current,360.0f);
  if (error>180.0f) error-=360.0f;
  if (error<-180.0f) error+=360.0f;
  float output;
  if(Motion.turn_only) output=CHASSIS_TURN_KP*error;
  else
  {
    uint32_t dt=Motion.heading_sample_tick-HeadingPid.rate_tick;
    if(!HeadingPid.rate_seeded || dt>CHASSIS_MOTION_MAX_GAP_MS) heading_rate_reset();
    else if(dt)
    {
      float delta=remainderf(current-HeadingPid.rate_yaw_deg,360.0f);
      float measured_rate=delta*1000.0f/(float)dt;
      float alpha=(float)dt/(CHASSIS_HEADING_RATE_FILTER_MS+(float)dt);
      HeadingPid.rate_deg_s+=alpha*(measured_rate-HeadingPid.rate_deg_s);
      HeadingPid.rate_yaw_deg=current;
      HeadingPid.rate_tick=Motion.heading_sample_tick;
    }
    /* Suppress small heading noise in ordinary translation/heading hold.
     * turn_only above, including visual precision correction, has no deadband.
     * Reset measured-rate history so old D damping cannot leak across the zone. */
    /* Degree/radian conversion and accumulated float deltas can put nominal
     * +/-0.1 just outside the inclusive boundary; this is below IMU resolution. */
    if(fabsf(error)<=CHASSIS_HEADING_DEADBAND_DEG+0.0001f)
    {
      heading_rate_reset();
      output=0;
    }
    else output=Motion.heading_kp*error-Motion.heading_kd*HeadingPid.rate_deg_s;
  }
  return fmaxf(-HeadingPid.output_limit, fminf(HeadingPid.output_limit,
                                           output));
}
static bool WaitingForStop, FinishAsError, ManualRamp, HaveYaw, DistanceBraking;
static uint32_t LastControlTick, StopRampTick, StopRampMs, LastYawCount, PidSampleCount,
    PidSampleTick, TurnRampTick;
static float PreviousYaw, AnchorYaw, AnchorMapYaw;
static ChassisModel_Velocity_t StopFrom;
static uint32_t DistanceGeneration, DistanceTimeout, DistanceStopTick, StopSequence,
                StableGroups;
static uint32_t StopSpeedTime[4];
static uint32_t StopPositionTime[4];
static unsigned ArrivalGroups;
static struct {
  bool armed,passed,lateral;
  float next_x,next_y,speed,accel;
} Corner;
static int64_t StopPositions[4];
static float DistanceMaxSpeed, MapVx, MapVy;
static float DistancePeakSpeed, DistanceRampFrom;
static uint32_t DistanceRampTick, CurveUpdateTick;
static uint32_t DistanceFeedbackAge;
static ChassisPath FollowPath;
static bool FollowingPath;
static float PathProgress,PathTurnSpeed;
static float HeadingTolerance = CHASSIS_DISTANCE_HEADING_DEG;
static unsigned DistanceCorrections;
static ChassisLocalization_Status_t Position; /* Main-loop snapshot; avoid large stack use. */
static ChassisStop_Status_t StopStatus;
static bool StopMonitoring, StopHaveGroup;
static uint32_t StopMotionSequence, StopDispatchSequence, StopActionId, StopTxTick;
static uint32_t StopQuietGroups;
static Mecanum_Feedback_t StopPrevious[4];
static void stop_monitor_process(void);
static const ChassisModel_Geometry_t Geometry = {CHASSIS_WHEELBASE_MM, CHASSIS_TRACK_WIDTH_MM,
                                                 CHASSIS_WHEEL_DIAMETER_MM,
                                                 CHASSIS_MOTOR_TO_WHEEL_RATIO};

static float half_cosine(float phase)
{
  if (phase <= 0)
    return 0;
  if (phase >= 1)
    return 1;
  return .5f - .5f * cosf(CHASSIS_MODEL_PI * phase);
}
static float distance_ramp_seconds(void)
{ return Motion.cosine_k * .002f; }
static void distance_ramp_start(uint32_t now)
{
  float distance=hypotf(Motion.error_x_mm,Motion.error_y_mm);
  float available=fmaxf(0,distance-CHASSIS_DISTANCE_TOLERANCE_MM*.5f);
  /* Two half-cosine ramps cover peak*T. Also reserve drive response travel.
   * Short legs keep the ramp duration and lower their peak instead of issuing
   * a full-speed command that cannot stop within the available distance. */
  DistancePeakSpeed=fminf(DistanceMaxSpeed,
      available/(distance_ramp_seconds()+CHASSIS_DISTANCE_RESPONSE_S));
  if(DistanceCorrections)
    DistancePeakSpeed=fminf(DistancePeakSpeed,CHASSIS_DISTANCE_CORRECTION_SPEED_MM_S);
  DistanceRampFrom=hypotf(MapVx,MapVy);
  DistanceRampTick=CurveUpdateTick=now;
  DistanceFeedbackAge=0;
}
static float distance_brake_speed(float available,float delay)
{
  float low=0,high=DistancePeakSpeed;
  if(high<=0 || available<=0)return 0;
  if(available>=high*(distance_ramp_seconds()*.5f+delay))return high;
  /* Invert the remaining integral of v=peak*(1+cos(pi*u))/2.
   * The resulting envelope follows the same cosine brake shape under ideal
   * feedback; actual remaining distance and feedback age keep it closed loop.
   * No source code from the reference project is copied here. */
  for(unsigned i=0;i<16;i++){
    float v=(low+high)*.5f;
    float u=acosf(fmaxf(-1,fminf(1,2*v/DistancePeakSpeed-1)))/CHASSIS_MODEL_PI;
    float travel=DistancePeakSpeed*distance_ramp_seconds()*.5f*
        (1-u-sinf(CHASSIS_MODEL_PI*u)/CHASSIS_MODEL_PI)+v*delay;
    if(travel>available)high=v;else low=v;
  }
  return low;
}
static float wrap_deg(float angle)
{
  float result = fmodf(angle + 180.0f, 360.0f);
  if (result < 0)
    result += 360.0f;
  return result - 180.0f;
}
void ChassisMotion_AnchorInvalidate(void)
{
  Motion.map_anchor_valid = false;
  Motion.heading_valid = false;
  HaveYaw = false;
  HeadingPid.rate_seeded=false;
}
/* Every read passes through the calibration module's qualification and age gate. */
static bool heading_read(uint32_t now)
{
  HWT101_Angle_t angle;
  /* These explicit workflows include yaw zeroing. Their old mapping must not
   * survive even if no intermediate angle was consumed during the workflow. */
  if(HWT101_Cal_IsBusy())
  {
    ChassisMotion_AnchorInvalidate();
    return false;
  }
  if (!HWT101_Cal_ControlAngleGet(&angle) || !isfinite(angle.yaw) ||
      (uint32_t)(now - angle.last_update_ms) > HWT101_DATA_FRESH_MS)
  {
    /* A temporary qualification/read gap is not a new coordinate origin.
     * Keep the unwrap baseline and map offset; qualification owns recovery.
     * Localization will withhold feedback and rebuild measured baselines. */
    Motion.heading_valid=false;
    HeadingPid.rate_seeded=false;
    return false;
  }
  if (HaveYaw && angle.update_count < LastYawCount)
  {
    ChassisMotion_AnchorInvalidate(); /* Module stream restarted: caller must re-anchor. */
    return false;
  }
  if (!HaveYaw)
  {
    PreviousYaw = angle.yaw;
    Motion.unwrapped_yaw_deg = angle.yaw;
    HaveYaw = true;
  }
  else if (angle.update_count != LastYawCount)
  {
    Motion.unwrapped_yaw_deg += wrap_deg(angle.yaw - PreviousYaw);
    PreviousYaw = angle.yaw;
  }
  LastYawCount = angle.update_count;
  Motion.current_deg = Motion.unwrapped_yaw_deg * CHASSIS_HEADING_TEST_YAW_SIGN;
  Motion.heading_sample_tick = angle.last_update_ms;
  Motion.heading_valid = true;
  if (Motion.map_anchor_valid)
    Motion.map_yaw_rad = AnchorMapYaw + (Motion.unwrapped_yaw_deg - AnchorYaw) *
                                            CHASSIS_HEADING_TEST_YAW_SIGN * CHASSIS_MODEL_PI /
                                            180.0f;
  return true;
}
void ChassisMotion_HeadingProcess(void)
{
  (void)heading_read(HAL_GetTick());
}
void ChassisMotion_Init(void)
{
  memset(&Motion, 0, sizeof(Motion));
  SelectedProfile=CHASSIS_PROFILE_LINEAR;
  memset(&StopStatus, 0, sizeof(StopStatus));
  StopStatus.reason = "idle";
  StopMonitoring = StopHaveGroup = false;
  StopQuietGroups = 0;
  memset(&HeadingPid, 0, sizeof(HeadingPid));
  Motion.reason = "idle";
  HoldTask=false;DriverAcceleration=0;
  memset(&HoldStatus,0,sizeof HoldStatus);HoldStatus.reason="idle";
  Motion.rpm_scale = 1;
  MapVx = MapVy = 0;
  FollowingPath=false;PathProgress=0;
  StableGroups = 0;
  ArrivalGroups=CHASSIS_DISTANCE_STOP_GROUPS;memset(&Corner,0,sizeof Corner);
  WaitingForStop = FinishAsError = ManualRamp = HaveYaw = DistanceBraking = false;
  LastControlTick = HAL_GetTick();
  LastYawCount = PidSampleCount = PidSampleTick = 0;
}
void ChassisMotion_TargetDefaults(ChassisMotion_Target_t *t)
{
  if (t == NULL)
    return;
  memset(t, 0, sizeof(*t));
  t->transition_ms = CHASSIS_MOTION_DEFAULT_RAMP_MS;
  t->hold_ms = 1000;
  t->stop_ms = CHASSIS_MOTION_DEFAULT_RAMP_MS;
}
bool ChassisMotion_IsBusy(void)
{
  return Motion.state == CHASSIS_MOTION_RUNNING || Motion.state == CHASSIS_MOTION_STOPPING;
}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *s)
{
  if (s != NULL)
    *s = Motion;
}
static bool target_valid(const ChassisMotion_Target_t *t, bool indefinite)
{
  uint32_t total;
  if (t == NULL || !isfinite(t->vx_mm_s) || !isfinite(t->vy_mm_s) || !isfinite(t->omega_rad_s) ||
      !isfinite(t->heading_deg))
    return false;
  if (hypotf(t->vx_mm_s, t->vy_mm_s) > CHASSIS_MOTION_MAX_LINEAR_MM_S ||
      fabsf(t->omega_rad_s) > (indefinite ? 2.0f*CHASSIS_MODEL_PI : CHASSIS_MOTION_MAX_OMEGA_RAD_S))
    return false;
  if (t->mode != CHASSIS_MOTION_ANGULAR_VELOCITY && t->mode != CHASSIS_MOTION_HEADING &&
      t->mode != CHASSIS_MOTION_HOLD_CURRENT)
    return false;
  if (t->mode != CHASSIS_MOTION_ANGULAR_VELOCITY && t->omega_rad_s != 0)
    return false;
  if (t->transition_ms > CHASSIS_MOTION_MAX_DURATION_MS ||
      t->hold_ms > CHASSIS_MOTION_MAX_DURATION_MS || t->stop_ms > CHASSIS_MOTION_MAX_DURATION_MS)
    return false;
  total = t->transition_ms + t->hold_ms + t->stop_ms;
  return indefinite || (total >= CHASSIS_MOTION_MIN_DURATION_MS && total <= CHASSIS_MOTION_MAX_DURATION_MS);
}
static bool start(const ChassisMotion_Target_t *t, bool indefinite, bool feedback_handoff)
{
  uint32_t now = HAL_GetTick();
  Mecanum_Status_t bus;
  if (ChassisMotion_IsBusy())
    return false; /* Keep the running task's diagnostic intact. */
  if (ChassisRoute_IsBusy() && !ChassisRoute_IsSubmitting())
  {
    Motion.reason = "route_reserved";
    return false;
  }
  if (!target_valid(t, indefinite))
  {
    Motion.reason = "invalid_target";
    return false;
  }
  if (HWT101_Cal_IsBusy() || GrabTask_CalibrationActive() || MechanicalArm_IsBusy())
  {
    Motion.reason = "busy";
    return false;
  }
  if(Mecanum_IsBusy())
  {
    /* Ordinary map motion can adopt a retired producer's committed zero.
     * Keep the explicit-stop latch itself intact: timed/manual velocity and
     * IMU ownership still require their original stop path. */
    if(!feedback_handoff){Motion.reason="busy";return false;}
    Mecanum_StatusGet(&bus);
    if(bus.locked || bus.stage==MECANUM_STAGE_FAULT || bus.error!=MECANUM_ERROR_NONE)
    {Motion.reason="bus_unavailable";return false;}
    if(GrabTask_IsBusy() && !GrabTask_PoseClearanceReady() && !GrabTask_QrDepartureActive())
    {Motion.reason="grab_task_busy";return false;}
    if(!Mecanum_VelocityHandoffReady())
    {Motion.reason="chassis_handoff_wait";return false;}
    for(unsigned i=0;i<4;i++){
      const Mecanum_Feedback_t *wheel=&Position.wheels[i];
      if(!wheel->speed_valid || !wheel->position_valid || !wheel->state_valid ||
         now-wheel->speed_ms>600U || now-wheel->position_ms>600U || now-wheel->state_ms>600U)
      {Motion.reason="feedback_invalid";return false;}
      if((wheel->state_flags&0x0cU) || !(wheel->state_flags&1U))
      {Motion.reason=(wheel->state_flags&0x0cU)?"resume_wheel_fault":"resume_wheels_disabled";return false;}
    }
  }
  Mecanum_StatusGet(&bus);
  if (bus.ack_profile == MECANUM_ACK_UNKNOWN)
  {
    Motion.reason = "ack_profile_unknown";
    return false;
  }
  if (bus.locked || bus.stop_pending || bus.stage == MECANUM_STAGE_FAULT)
  {
    Motion.reason = "bus_unavailable";
    return false;
  }
  if (t->mode != CHASSIS_MOTION_ANGULAR_VELOCITY && !heading_read(now))
  {
    Motion.reason = "imu_invalid";
    return false;
  }
  if (t->mode == CHASSIS_MOTION_HEADING && !Motion.map_anchor_valid)
  {
    Motion.reason = "anchor_invalid";
    return false;
  }
  Motion.distance_mode = Motion.turn_only = Motion.stop_confirmed = false;
  ArrivalGroups=CHASSIS_DISTANCE_STOP_GROUPS;memset(&Corner,0,sizeof Corner);
  FollowingPath=false;
  Motion.indefinite_velocity = indefinite;
  hold_status_refresh();
  DriverAcceleration=0;HoldTask=false;
  TaskStopSequence=bus.stop_sequence;
  Motion.requested = *t;
  Motion.requested.heading_deg = wrap_deg(t->heading_deg);
  if (t->mode == CHASSIS_MOTION_HOLD_CURRENT)
    Motion.requested.heading_deg = wrap_deg(Motion.current_deg);
  Motion.state = CHASSIS_MOTION_RUNNING;
  Motion.reason = "running";
  Motion.action_id++;
  Motion.start_tick = Motion.sample_tick = LastControlTick = now;
  Motion.elapsed_ms = Motion.control_dt_ms = 0;
  Motion.profile=SelectedProfile;Motion.peak_mm_s=0;Motion.stop_ms=0;
  Motion.linear_accel_mm_s2=CHASSIS_DISTANCE_ACCEL_MM_S2;
  Motion.cosine_k=CHASSIS_DISTANCE_COSINE_K;
  Motion.heading_kp=CHASSIS_HEADING_TEST_KP;Motion.heading_kd=CHASSIS_HEADING_TEST_KD;
  memset(&Motion.body_target, 0, sizeof(Motion.body_target));
  memset(Motion.rpm_unquantized, 0, sizeof(Motion.rpm_unquantized));
  memset(Motion.rpm_command, 0, sizeof(Motion.rpm_command));
  Motion.rpm_scale = 1;
  WaitingForStop = FinishAsError = ManualRamp = DistanceBraking = false;
  if (t->mode != CHASSIS_MOTION_ANGULAR_VELOCITY)
  {
    HeadingPid.output_limit=CHASSIS_HEADING_TEST_OUTPUT_LIMIT;
    HeadingPid.output_rad_s=0;
    HeadingPid.target_yaw_deg=Motion.requested.heading_deg;
    heading_rate_reset();
    PidSampleTick = Motion.heading_sample_tick;
    PidSampleCount = LastYawCount;
  }
  return true;
}
bool ChassisMotion_Start(const ChassisMotion_Target_t *t) { return start(t,false,false); }
/* This sends one initial snapshot, then the shared task process refreshes it. */
static bool immediate_command(void)
{
  ChassisModel_Velocity_t body = {Motion.requested.vx_mm_s,Motion.requested.vy_mm_s,
                                  Motion.requested.omega_rad_s};
  ChassisModel_Wheels_t wheels;
  unsigned i;
  if (!ChassisModel_Inverse(&Geometry,&body,CHASSIS_MOTION_MAX_WHEEL_RPM,&wheels) ||
      !Mecanum_Velocity_Request(body.vx_mm_s*wheels.scale,body.vy_mm_s*wheels.scale,
                               body.omega_rad_s*wheels.scale,DriverAcceleration))
  { request_stop("velocity_rejected",true);return false; }
  Motion.body_target=body;
  Motion.body_target.vx_mm_s*=wheels.scale;
  Motion.body_target.vy_mm_s*=wheels.scale;
  Motion.body_target.omega_rad_s*=wheels.scale;
  Motion.rpm_scale=wheels.scale;
  for(i=0;i<4;i++) {Motion.rpm_unquantized[i]=wheels.rpm[i];Motion.rpm_command[i]=wheels.command[i];}
  return true;
}
bool ChassisMotion_VelocityStart(float vx,float vy,float omega)
{
  ChassisMotion_Target_t target;
  memset(&target,0,sizeof target);
  target.vx_mm_s=vx;target.vy_mm_s=vy;target.omega_rad_s=omega;
  if(!start(&target,true,false)) return false;
  DriverAcceleration=CHASSIS_VELOCITY_ACCELERATION;
  return immediate_command();
}
bool ChassisMotion_VelocityUpdate(float vx,float vy,float omega)
{
  ChassisMotion_Target_t target=Motion.requested;
  Mecanum_Status_t bus;
  Mecanum_StatusGet(&bus);
  if(bus.stop_pending || bus.stop_sequence!=TaskStopSequence) return false;
  if(!Motion.indefinite_velocity || Motion.state!=CHASSIS_MOTION_RUNNING || WaitingForStop)
    return false;
  target.vx_mm_s=vx;target.vy_mm_s=vy;target.omega_rad_s=omega;
  if(!target_valid(&target,true)) return false;
  Motion.requested=target;
  return immediate_command();
}
bool ChassisMotion_HoldStart(float forward,uint32_t duration)
{
  ChassisMotion_Target_t target;
  if(ChassisMotion_IsBusy())
  { if(!HoldTask) HoldStatus.reason="chassis_busy";return false; }
  hold_status_refresh();HoldTask=false;
  if(!isfinite(forward) || fabsf(forward)>100 || duration<1000 || duration>10000)
  { HoldStatus.reason="range";return false; }
  memset(&target,0,sizeof target);
  target.vx_mm_s=forward;target.mode=CHASSIS_MOTION_HOLD_CURRENT;target.hold_ms=duration;
  if(!start(&target,false,false))
  {
    HoldStatus.reason=!strcmp(Motion.reason,"imu_invalid")?"imu_not_verified":
        (!strcmp(Motion.reason,"busy")?(HWT101_Cal_IsBusy()?"imu_busy":"chassis_busy"):Motion.reason);
    return false;
  }
  DriverAcceleration=CHASSIS_VELOCITY_ACCELERATION;HoldTask=true;
  return immediate_command();
}
static void hold_status_refresh(void)
{
  if(HoldTask)
  {
    HoldStatus.active=ChassisMotion_IsBusy() && !WaitingForStop;
    HoldStatus.reason=!strcmp(Motion.reason,"timed_complete")?"complete":
        (!strcmp(Motion.reason,"cancelled")?"stopped":Motion.reason);
    HoldStatus.target_deg=Motion.requested.heading_deg;
    HoldStatus.current_deg=Motion.current_deg;
    HoldStatus.output_rad_s=Motion.body_target.omega_rad_s;
    if(!ChassisMotion_IsBusy()) HoldTask=false;
  }
}
void ChassisMotion_HoldStatusGet(ChassisMotion_HoldStatus_t *out)
{
  hold_status_refresh();
  if(out) *out=HoldStatus;
}
bool ChassisMotion_MoveTo(float x, float y, float yaw, float speed, uint32_t timeout)
{ return ChassisMotion_MoveToAccel(x,y,yaw,speed,CHASSIS_DISTANCE_ACCEL_MM_S2,timeout); }
void ChassisMotion_TestParametersDefaults(ChassisMotion_TestParameters_t *p)
{
  if(!p)return;
  p->curve_period_ms=CHASSIS_CURVE_PERIOD_MS;
  p->linear_decel_mm_s2=CHASSIS_DISTANCE_ACCEL_MM_S2;
  p->linear_accel_mm_s2=CHASSIS_DISTANCE_ACCEL_MM_S2;p->cosine_k=CHASSIS_DISTANCE_COSINE_K;
  p->heading_kp=CHASSIS_HEADING_TEST_KP;p->heading_kd=CHASSIS_HEADING_TEST_KD;
}
bool ChassisMotion_MoveToAccel(float x, float y, float yaw, float speed, float accel, uint32_t timeout)
{
  ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);p.linear_accel_mm_s2=p.linear_decel_mm_s2=accel;
  return ChassisMotion_MoveToTuned(x,y,yaw,speed,&p,timeout);
}
bool ChassisMotion_MoveToTuned(float x, float y, float yaw, float speed,
                              const ChassisMotion_TestParameters_t *p, uint32_t timeout)
{
  ChassisMotion_Target_t t;
  if (ChassisMotion_IsBusy()) return false;
  ChassisLocalization_Get(&Position);
  if(!p){Motion.reason="invalid_target";return false;}
  if (!isfinite(p->linear_accel_mm_s2) || p->linear_accel_mm_s2<CHASSIS_TEST_ACCEL_MIN_MM_S2 || p->linear_accel_mm_s2>CHASSIS_TEST_ACCEL_MAX_MM_S2)
  { Motion.reason="accel_range";return false; }
  if(!isfinite(p->linear_decel_mm_s2) || p->linear_decel_mm_s2<CHASSIS_TEST_ACCEL_MIN_MM_S2 || p->linear_decel_mm_s2>CHASSIS_TEST_ACCEL_MAX_MM_S2)
  {Motion.reason="decel_range";return false;}
  if(p->curve_period_ms<1 || p->curve_period_ms>CHASSIS_CURVE_PERIOD_MAX_MS)
  {Motion.reason="curve_period_range";return false;}
  if(p->cosine_k<CHASSIS_TEST_COSINE_K_MIN || p->cosine_k>CHASSIS_TEST_COSINE_K_MAX)
  {Motion.reason="cosine_k_range";return false;}
  if(!isfinite(p->heading_kp) || !isfinite(p->heading_kd) || p->heading_kp<0 || p->heading_kd<0 ||
     p->heading_kp>CHASSIS_TEST_HEADING_GAIN_MAX || p->heading_kd>CHASSIS_TEST_HEADING_GAIN_MAX)
  {Motion.reason="heading_gain_range";return false;}
  if (!isfinite(x) || !isfinite(y) || !isfinite(yaw) || !isfinite(speed) ||
      fabsf(x) > 10000 || fabsf(y) > 10000 || speed <= 0 ||
      speed > CHASSIS_MOTION_MAX_LINEAR_MM_S || timeout < CHASSIS_MOTION_MIN_DURATION_MS ||
      timeout > CHASSIS_DISTANCE_MAX_TIMEOUT_MS)
  { Motion.reason = "invalid_target"; return false; }
  if (!Position.origin_valid || !Position.feedback_valid || !Position.speed_valid ||
      !Position.units_per_rev || (uint32_t)(HAL_GetTick()-Position.feedback_tick)>600U)
  { Motion.reason = "feedback_invalid"; return false; }
  {
    unsigned i;
    for(i=0;i<4;i++) if(Position.wheels[i].speed_rpm>CHASSIS_DISTANCE_STOP_RPM ||
                        Position.wheels[i].speed_rpm < -CHASSIS_DISTANCE_STOP_RPM)
    { Motion.reason="feedback_moving";return false; }
  }
  if (!heading_read(HAL_GetTick()) || !Motion.map_anchor_valid)
  { Motion.reason = "imu_invalid"; return false; }
  ChassisMotion_TargetDefaults(&t);
  t.mode = CHASSIS_MOTION_HEADING;
  /* Convert map target to the signed module frame once; no implicit module zero. */
  t.heading_deg = Motion.current_deg + wrap_deg(yaw - Motion.map_yaw_rad*180.0f/CHASSIS_MODEL_PI);
  if (!start(&t,false,true)) return false;
  Motion.distance_mode = true;
  Motion.curve_period_ms=p->curve_period_ms;
  Motion.linear_decel_mm_s2=p->linear_decel_mm_s2;
  Motion.linear_accel_mm_s2=p->linear_accel_mm_s2;Motion.cosine_k=p->cosine_k;
  Motion.heading_kp=p->heading_kp;Motion.heading_kd=p->heading_kd;
  HeadingTolerance = CHASSIS_DISTANCE_HEADING_DEG;
  Motion.target_x_mm = x; Motion.target_y_mm = y; Motion.target_map_yaw_deg = wrap_deg(yaw);
  Motion.error_x_mm = x - Position.observer.feedback.x_mm;
  Motion.error_y_mm = y - Position.observer.feedback.y_mm;
  Motion.error_heading_deg = wrap_deg(yaw - Motion.map_yaw_rad*180.0f/CHASSIS_MODEL_PI);
  DistanceGeneration = Position.generation; DistanceTimeout = timeout; DistanceMaxSpeed = speed;
  MapVx = MapVy = 0; StableGroups = 0; DistanceCorrections = 0;
  distance_ramp_start(HAL_GetTick());
  return true;
}
bool ChassisMotion_TurnTo(float yaw, uint32_t timeout)
{ return ChassisMotion_TurnToSpeed(yaw,CHASSIS_MOTION_MAX_OMEGA_DEG_S,timeout); }

void ChassisMotion_UseRouteArrival(void)
{if(Motion.state==CHASSIS_MOTION_RUNNING && Motion.distance_mode)ArrivalGroups=CHASSIS_ROUTE_STOP_GROUPS;}
bool ChassisMotion_PrepareCorner(float x,float y,float speed,float accel,uint32_t timeout)
{
  if(Motion.state!=CHASSIS_MOTION_RUNNING || !Motion.distance_mode || Motion.turn_only ||
     WaitingForStop || FollowingPath || Motion.profile!=CHASSIS_PROFILE_LINEAR ||
     Corner.armed || Corner.passed || !isfinite(x) || !isfinite(y) || !isfinite(speed) || !isfinite(accel) ||
     fabsf(Motion.target_x_mm-2100)>.01f || fabsf(Motion.target_y_mm-2100)>.01f ||
     fabsf(wrap_deg(Motion.target_map_yaw_deg-90))>.01f || x>=Motion.target_x_mm ||
     fabsf(y-Motion.target_y_mm)>.01f || speed<=0 || speed>CHASSIS_MOTION_MAX_LINEAR_MM_S ||
     accel<CHASSIS_TEST_ACCEL_MIN_MM_S2 || accel>CHASSIS_TEST_ACCEL_MAX_MM_S2 ||
     timeout<DistanceTimeout || timeout>CHASSIS_DISTANCE_MAX_TIMEOUT_MS)return false;
  Corner.armed=true;Corner.next_x=x;Corner.next_y=y;Corner.speed=speed;Corner.accel=accel;
  DistanceTimeout=timeout;return true;
}
bool ChassisMotion_CornerPassed(void){return Corner.passed;}

static void corner_handoff(uint32_t now)
{
  if(!Corner.armed || !in_target())return;
  Corner.armed=false;Corner.passed=Corner.lateral=true;
  Motion.target_x_mm=Corner.next_x;Motion.target_y_mm=Corner.next_y;
  DistanceMaxSpeed=Corner.speed;Motion.linear_accel_mm_s2=Corner.accel;
  Motion.error_x_mm=Corner.next_x-Position.observer.feedback.x_mm;
  Motion.error_y_mm=Corner.next_y-Position.observer.feedback.y_mm;
  /* Preserve MapVx/MapVy, heading history, origin, action ID and deadline. */
  distance_ramp_start(now);
}
float ChassisMotion_PathProgress(void){return PathProgress;}
bool ChassisMotion_PathStart(const ChassisPath *path,float speed,float turn_deg_s,
                             uint32_t timeout,float resume_mm)
{
  if(!path || path->count<2 || path->count>CHASSIS_PATH_CAPACITY ||
     !isfinite(resume_mm) || resume_mm<0 || resume_mm>path->length ||
     !isfinite(turn_deg_s) || turn_deg_s<5 || turn_deg_s>CHASSIS_MOTION_MAX_OMEGA_DEG_S)return false;
  for(unsigned i=0;i<path->count;i++){
    ChassisPath_Point p=path->point[i];
    if(!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.yaw) || !isfinite(p.s) ||
       !isfinite(p.limit) || p.limit<0 || (i && p.s<=path->point[i-1].s))return false;
  }
  ChassisPath_Point end=path->point[path->count-1];
  if(!ChassisMotion_MoveTo(end.x,end.y,end.yaw,speed,timeout))return false;
  FollowPath=*path;FollowingPath=true;PathProgress=resume_mm;PathTurnSpeed=turn_deg_s;
  return true;
}
bool ChassisMotion_TurnToSpeed(float yaw,float max_deg_s,uint32_t timeout)
{
  if(!isfinite(max_deg_s) || max_deg_s<5 || max_deg_s>CHASSIS_MOTION_MAX_OMEGA_DEG_S)
  { Motion.reason="turn_speed_range";return false; }
  if (ChassisMotion_IsBusy()) return false;
  if (!isfinite(yaw) || yaw < -180.0f || yaw > 180.0f)
  { Motion.reason="invalid_target";return false; }
  if (GrabTask_IsBusy())
  { Motion.reason="grab_task_busy";return false; }
  ChassisLocalization_Get(&Position);
  if (!ChassisMotion_MoveTo(Position.observer.feedback.x_mm,
      Position.observer.feedback.y_mm,yaw,CHASSIS_DISTANCE_SPEED_MM_S,timeout)) return false;
  Motion.turn_only=true;
  HeadingPid.output_limit=max_deg_s*CHASSIS_MODEL_PI/180.0f;
  TurnRampTick=Motion.start_tick;
  return true;
}
bool ChassisMotion_TurnToPrecision(float yaw,float max_deg_s,float tolerance_deg,uint32_t timeout)
{
  if(!isfinite(tolerance_deg) || tolerance_deg<=0 || tolerance_deg>CHASSIS_DISTANCE_HEADING_DEG)return false;
  if(!ChassisMotion_TurnToSpeed(yaw,max_deg_s,timeout))return false;
  HeadingTolerance=tolerance_deg;return true;
}
static bool request_stop(const char *reason, bool error)
{
  Mecanum_Feedback_Arrival(false);
  Corner.armed=false;
  Motion.reason = reason;
  Motion.stop_confirmed = false;
  FinishAsError = error;
  Motion.state = CHASSIS_MOTION_STOPPING;
  WaitingForStop = true;
  DistanceStopTick = HAL_GetTick();
  Motion.stop_ms=0;
  StableGroups = 0;
  if (Motion.distance_mode)
  {
    unsigned i;
    ChassisLocalization_Get(&Position);
    StopSequence = Position.feedback_sequence;
    for (i=0;i<4;i++) { StopPositions[i]=Position.wheels[i].position_raw; StopSpeedTime[i]=Position.wheels[i].speed_ms;StopPositionTime[i]=Position.wheels[i].position_ms; }
  }
  Motion.sample_tick = HAL_GetTick();
  Motion.elapsed_ms = (uint32_t)(Motion.sample_tick - Motion.start_tick);
  memset(&Motion.body_target, 0, sizeof(Motion.body_target));
  memset(Motion.rpm_unquantized, 0, sizeof(Motion.rpm_unquantized));
  memset(Motion.rpm_command, 0, sizeof(Motion.rpm_command));
  DistanceBraking = Motion.distance_mode && !error && !strcmp(reason,"position_reached");
  /* A normal stop finishes the synchronized zero snapshot before issuing stop frames.
     Emergency/cancel paths still interrupt immediately and retain cache-fault checks. */
  if (DistanceBraking && Mecanum_Velocity_Request(0,0,0,0)) return true;
  DistanceBraking=false;
  if (!Mecanum_Test_Stop())
  {
    Motion.state = CHASSIS_MOTION_ERROR;
    Motion.reason = "stop_rejected";
    WaitingForStop = false;
    return false;
  }
  return true;
}
bool ChassisMotion_Stop(uint32_t stop_ms)
{
  if (stop_ms > CHASSIS_MOTION_MAX_DURATION_MS)
    return false;
  if (WaitingForStop)
  {
    if (Motion.distance_mode && !FinishAsError)
    {
      Motion.reason="cancelled";
      if(DistanceBraking) { DistanceBraking=false;return Mecanum_Test_Stop(); }
    }
    return true;
  }
  if (Motion.distance_mode) stop_ms = 0;
  if (!ChassisMotion_IsBusy() || stop_ms == 0)
    return request_stop("cancelled", false);
  ManualRamp = true;
  StopRampTick = HAL_GetTick();
  StopRampMs = stop_ms;
  StopFrom = Motion.body_target;
  Motion.state = CHASSIS_MOTION_STOPPING;
  Motion.reason = "stopping";
  return true;
}

bool ChassisMotion_StopRequest(uint32_t token)
{
  Mecanum_Status_t bus;
  bool ok;
  stop_monitor_process();
  Mecanum_StatusGet(&bus);
  if (token && StopStatus.id && token == StopStatus.token &&
      bus.motion_sequence == StopMotionSequence && Motion.action_id == StopActionId)
    return true;
  StopStatus.id++;
  if (!StopStatus.id) StopStatus.id = 1;
  StopStatus.token = token;
  StopStatus.requested_ms = HAL_GetTick();
  StopStatus.tx_complete = StopStatus.wheels_stopped = false;
  StopStatus.reason = "pending_tx";
  StopHaveGroup = false;
  StopQuietGroups = 0;
  /* Stop() cancels scheduled/ramped motion; Test_Stop also covers already-finished
   * tasks and joins the sole UART5 stop transaction if cancellation started it. */
  ok = ChassisMotion_Stop(0);
  Mecanum_StatusGet(&bus);
  if (!bus.stop_pending && !Mecanum_Test_Stop()) ok = false;
  Mecanum_StatusGet(&bus);
  StopMotionSequence = bus.motion_sequence;
  StopDispatchSequence = bus.stop_sequence;
  StopActionId = Motion.action_id;
  StopMonitoring = ok;
  Mecanum_Feedback_StopMonitor(ok);
  if (!ok) StopStatus.reason = "tx_failed";
  return ok;
}

static void stop_monitor_process(void)
{
  Mecanum_Status_t bus;
  uint32_t now = HAL_GetTick(), oldest = 0, newest = UINT32_MAX;
  bool fresh = true, changed = true, quiet = true;
  unsigned i;
  if (!StopStatus.id) return;
  Mecanum_StatusGet(&bus);
  if (bus.motion_sequence != StopMotionSequence || Motion.action_id != StopActionId)
  {
    StopStatus.wheels_stopped = false;
    StopStatus.reason = "new_motion";
    StopMonitoring = false;
    Mecanum_Feedback_StopMonitor(false);
    return;
  }
  if (!StopMonitoring) return;
  if (bus.stage == MECANUM_STAGE_FAULT || bus.stop_sequence != StopDispatchSequence)
  {
    StopStatus.wheels_stopped = false;
    StopStatus.reason = "tx_failed";
    StopMonitoring = false;
    Mecanum_Feedback_StopMonitor(false);
    return;
  }
  if (!StopStatus.tx_complete)
  {
    if (bus.stage != MECANUM_STAGE_STOPPED || bus.stop_pending || !bus.tx_complete)
    {
      if ((uint32_t)(now - StopStatus.requested_ms) >= 6000U)
      { StopStatus.reason = "tx_failed"; StopMonitoring = false; Mecanum_Feedback_StopMonitor(false); }
      return;
    }
    StopStatus.tx_complete = true;
    StopTxTick = bus.sent_ms;
    StopStatus.reason = "monitoring";
  }
  /* Stop evidence is independent of localization setup and its cached snapshot. */
  Mecanum_Feedback_t wheels[4];
  for (i = 0; i < 4; i++)
    if (!Mecanum_FeedbackGet(i, &wheels[i])) { memset(&wheels[i], 0, sizeof wheels[i]); fresh = false; }
  for (i = 0; i < 4; i++)
  {
    const Mecanum_Feedback_t *w = &wheels[i];
    uint32_t speed_age = now - w->speed_ms, position_age = now - w->position_ms;
    if (!w->speed_valid || !w->position_valid || speed_age > 600U || position_age > 600U ||
        !((uint32_t)(w->speed_ms - StopTxTick) > 0U &&
          (uint32_t)(w->speed_ms - StopTxTick) < 0x80000000U) ||
        !((uint32_t)(w->position_ms - StopTxTick) > 0U &&
          (uint32_t)(w->position_ms - StopTxTick) < 0x80000000U)) fresh = false;
    if (speed_age > oldest) oldest = speed_age;
    if (position_age > oldest) oldest = position_age;
    if (speed_age < newest) newest = speed_age;
    if (position_age < newest) newest = position_age;
    if (StopHaveGroup && (w->speed_ms == StopPrevious[i].speed_ms ||
                          w->position_ms == StopPrevious[i].position_ms)) changed = false;
    if (w->speed_rpm > CHASSIS_DISTANCE_STOP_RPM || w->speed_rpm < -CHASSIS_DISTANCE_STOP_RPM)
      quiet = false;
    if (StopHaveGroup)
    {
      float delta = fabsf((float)(w->position_raw - StopPrevious[i].position_raw)) *
                    CHASSIS_MODEL_PI * Geometry.wheel_diameter_mm /
                    ((float)CHASSIS_EMM_POSITION_UNITS_PER_REV * Geometry.gear_ratio);
      if (delta > CHASSIS_DISTANCE_STOP_DELTA_MM) quiet = false;
    }
  }
  /* Bound all eight receive timestamps, including speed-to-position separation. */
  if (oldest - newest > 250U) fresh = false;
  if (!fresh)
  {
    StopStatus.wheels_stopped = false;
    StopStatus.reason = "missing_feedback";
    StopQuietGroups = 0;
    StopHaveGroup = false;
  }
  else
  {
    if (!quiet)
    {
      StopQuietGroups = 0;
      StopStatus.wheels_stopped = false;
      StopStatus.reason = "monitoring";
    }
    if (changed)
    {
      if (quiet)
      {
        /* First complete post-TX group is a position baseline, not a delta. */
        StopQuietGroups++;
        if (StopQuietGroups >= CHASSIS_DISTANCE_STOP_GROUPS)
        { StopStatus.wheels_stopped = true; StopStatus.reason = "stopped"; }
        else StopStatus.reason = "monitoring";
      }
      memcpy(StopPrevious, wheels, sizeof(StopPrevious));
      StopHaveGroup = true;
    }
  }
  if (StopStatus.wheels_stopped)
  { StopMonitoring = false; Mecanum_Feedback_StopMonitor(false); }
  if (!StopStatus.wheels_stopped && (uint32_t)(now - StopStatus.requested_ms) >= 6000U)
  {
    if (fresh) StopStatus.reason = "timeout";
    StopMonitoring = false;
    Mecanum_Feedback_StopMonitor(false);
  }
}

bool ChassisMotion_StopPending(void) { return StopMonitoring; }

void ChassisMotion_StopStatusGet(ChassisStop_Status_t *out)
{
  stop_monitor_process();
  if (out) *out = StopStatus;
}
bool ChassisMotion_AnchorSet(float map_yaw_rad)
{
  if (!isfinite(map_yaw_rad) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() || Mecanum_IsBusy() || HWT101_Cal_IsBusy() ||
      GrabTask_CalibrationActive() || MechanicalArm_IsBusy())
    return false;
  ChassisMotion_AnchorInvalidate();
  if (!heading_read(HAL_GetTick()))
    return false;
  AnchorYaw = Motion.unwrapped_yaw_deg;
  AnchorMapYaw = map_yaw_rad;
  Motion.map_yaw_rad = map_yaw_rad;
  Motion.map_anchor_valid = true;
  return true;
}
bool ChassisMotion_AnchorCorrect(float yaw)
{
  if(!isfinite(yaw) || ChassisMotion_IsBusy() || ChassisRoute_IsBusy() ||
     Mecanum_IsBusy() || MechanicalArm_IsBusy() || GrabTask_IsBusy() ||
     HWT101_Cal_IsBusy() || !heading_read(HAL_GetTick()) || !Motion.map_anchor_valid)return false;
  AnchorYaw=Motion.unwrapped_yaw_deg;AnchorMapYaw=yaw;Motion.map_yaw_rad=yaw;
  Motion.heading_revision++;
  Motion.state=CHASSIS_MOTION_IDLE;Motion.reason="heading_corrected";
  Motion.distance_mode=Motion.turn_only=false;
  Motion.error_heading_deg=0;Motion.body_target.omega_rad_s=0;
  HeadingPid.output_rad_s=0;MapVx=MapVy=0;
  return true;
}
static bool distance_feedback_check(uint32_t now)
{
  ChassisLocalization_Get(&Position);
  Motion.error_x_mm = Motion.target_x_mm - Position.observer.feedback.x_mm;
  Motion.error_y_mm = Motion.target_y_mm - Position.observer.feedback.y_mm;
  Motion.error_heading_deg = wrap_deg(Motion.target_map_yaw_deg - Motion.map_yaw_rad*180.0f/CHASSIS_MODEL_PI);
  return Position.feedback_valid && Position.speed_valid && Position.origin_valid &&
         Position.generation == DistanceGeneration && (uint32_t)(now-Position.feedback_tick)<=600U &&
         Motion.heading_valid && Motion.map_anchor_valid;
}
static bool in_target(void)
{
  return (!FollowingPath || PathProgress>=FollowPath.turn_end) &&
         (Motion.turn_only || hypotf(Motion.error_x_mm,Motion.error_y_mm)<=CHASSIS_DISTANCE_TOLERANCE_MM) &&
         fabsf(Motion.error_heading_deg)<=HeadingTolerance;
}
/* Only complete groups received after the stop request count as stop evidence. */
static bool stopped_feedback(uint32_t now)
{
  unsigned i;
  bool quiet = true; /* Physical stop evidence is independent of target accuracy. */
  bool route_arrival=ArrivalGroups==CHASSIS_ROUTE_STOP_GROUPS;
  Mecanum_Status_t bus;
  Mecanum_StatusGet(&bus);
  if(!bus.tx_complete)return false;
  if (Position.feedback_sequence == StopSequence) return false;
  /* Position groups and speed reads are independent UART transactions. Wait
   * for all four new speeds before consuming evidence; a reused fresh sample
   * is not movement and must not reset an already quiet interval. */
  for(i=0;i<4;i++)
    if(Position.wheels[i].speed_ms==StopSpeedTime[i] ||
       (route_arrival && Position.wheels[i].position_ms==StopPositionTime[i])) return false;
  StopSequence = Position.feedback_sequence;
  for(i=0;i<4;i++)
  {
    const Mecanum_Feedback_t *w = &Position.wheels[i];
    float delta = fabsf((float)(w->position_raw - StopPositions[i])) * CHASSIS_MODEL_PI *
                  Geometry.wheel_diameter_mm / ((float)Position.units_per_rev * Geometry.gear_ratio);
    if (!w->speed_valid || (uint32_t)(now-w->speed_ms)>600U ||
        (route_arrival && (!w->position_valid || (uint32_t)(now-w->position_ms)>600U)) ||
        w->speed_ms == StopSpeedTime[i] || w->speed_ms==bus.sent_ms ||
        (uint32_t)(w->speed_ms-bus.sent_ms)>0x7FFFFFFFU ||
        w->position_ms==bus.sent_ms || (uint32_t)(w->position_ms-bus.sent_ms)>0x7FFFFFFFU ||
        w->speed_rpm > CHASSIS_DISTANCE_STOP_RPM || w->speed_rpm < -CHASSIS_DISTANCE_STOP_RPM ||
        delta > CHASSIS_DISTANCE_STOP_DELTA_MM) quiet = false;
    StopPositions[i]=w->position_raw; StopSpeedTime[i]=w->speed_ms;StopPositionTime[i]=w->position_ms;
  }
  if(!quiet) { StableGroups=0; return false; }
  StableGroups++;
  return StableGroups>=ArrivalGroups;
}
static bool distance_heading_command(ChassisModel_Velocity_t *command)
{
  if(LastYawCount!=PidSampleCount)
  {
    uint32_t elapsed=Motion.heading_sample_tick-PidSampleTick;
    if(elapsed>CHASSIS_MOTION_MAX_GAP_MS) return false;
    if(elapsed)
    {
      HeadingPid.output_rad_s=heading_update(Motion.current_deg);
      PidSampleTick=Motion.heading_sample_tick;PidSampleCount=LastYawCount;
    }
  }
  command->omega_rad_s=HeadingPid.output_rad_s;
  return isfinite(command->omega_rad_s);
}
static bool path_command(uint32_t dt,ChassisModel_Velocity_t *command)
{
  float px=Position.observer.feedback.x_mm,py=Position.observer.feedback.y_mm;
  PathProgress=ChassisPath_Project(&FollowPath,PathProgress,px,py);
  ChassisPath_Point at=ChassisPath_Sample(&FollowPath,PathProgress);
  ChassisPath_Point ahead=ChassisPath_Sample(&FollowPath,PathProgress+8);
  float dx=ahead.x-at.x,dy=ahead.y-at.y,len=hypotf(dx,dy);
  float slope=len>.01f?(ahead.yaw-at.yaw)/(ahead.s-at.s):0;
  float speed=DistanceMaxSpeed;
  /* Brake before the curved section, not after reaching its first point.
   * Reserve angular authority for correcting measured heading error. */
  for(unsigned i=1;i<FollowPath.count;i++){
    ChassisPath_Point a=FollowPath.point[i-1],b=FollowPath.point[i];
    if(b.s<PathProgress)continue;
    float gradient=fabsf(b.yaw-a.yaw)/(b.s-a.s);
    float cap=b.limit;
    if(gradient>.001f)cap=fminf(cap,.65f*PathTurnSpeed/gradient);
    float distance=fmaxf(0,a.s-PathProgress);
    /* Final target braking uses the existing feedback-aware distance law. */
    if(i+1==FollowPath.count)break;
    speed=fminf(speed,sqrtf(cap*cap+2*150*distance));
  }
  float yaw_error=wrap_deg(at.yaw-Motion.map_yaw_rad*180/CHASSIS_MODEL_PI);
  speed*=fminf(1,5/fmaxf(5,fabsf(yaw_error)));
  float vx=(len>.01f?dx/len*speed:0)+2*(at.x-px);
  float vy=(len>.01f?dy/len*speed:0)+2*(at.y-py);
  float norm=hypotf(vx,vy),limit=fminf(DistanceMaxSpeed,speed+20);
  if(norm>limit){vx*=limit/norm;vy*=limit/norm;}
  dx=vx-MapVx;dy=vy-MapVy;norm=hypotf(dx,dy);
  float step=CHASSIS_PATH_ACCEL_MM_S2*dt/1000.0f;
  if(norm>step){dx*=step/norm;dy*=step/norm;}
  MapVx+=dx;MapVy+=dy;
  command->vx_mm_s=cosf(Motion.map_yaw_rad)*MapVx+sinf(Motion.map_yaw_rad)*MapVy;
  command->vy_mm_s=-sinf(Motion.map_yaw_rad)*MapVx+cosf(Motion.map_yaw_rad)*MapVy;
  float omega=slope*hypotf(MapVx,MapVy)+2*yaw_error;
  command->omega_rad_s=fmaxf(-PathTurnSpeed,fminf(PathTurnSpeed,omega))*CHASSIS_MODEL_PI/180;
  PidSampleTick=Motion.heading_sample_tick;PidSampleCount=LastYawCount;
  return true;
}
static bool distance_command(uint32_t dt, ChassisModel_Velocity_t *command)
{
  if(FollowingPath){
    PathProgress=ChassisPath_Project(&FollowPath,PathProgress,
      Position.observer.feedback.x_mm,Position.observer.feedback.y_mm);
    if(PathProgress<FollowPath.turn_end+5)return path_command(dt,command);
    FollowingPath=false;
    distance_ramp_start(HAL_GetTick()); /* Continue from the curved exit speed. */
    HeadingPid.output_rad_s=heading_update(Motion.current_deg);
  }
  float vx=Motion.error_x_mm*CHASSIS_DISTANCE_KP, vy=Motion.error_y_mm*CHASSIS_DISTANCE_KP;
  float norm=hypotf(vx,vy);
  static const uint8_t directions[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,
      CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,
      CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  float distance=hypotf(Motion.error_x_mm,Motion.error_y_mm);
  float delay=CHASSIS_DISTANCE_RESPONSE_S;
  float rpm[4], measured=0, available, brake_speed, limit;
  ChassisModel_Velocity_t feedback;
  unsigned i;
  uint32_t now=HAL_GetTick(), age=(uint32_t)(now-Position.feedback_tick);
  if(Motion.turn_only)
  {
    command->vx_mm_s=command->vy_mm_s=0;
    if(!distance_heading_command(command)) return false;
    command->omega_rad_s*=half_cosine((float)(now-TurnRampTick)/CHASSIS_MOTION_DEFAULT_RAMP_MS);
    if(HeadingTolerance<CHASSIS_DISTANCE_HEADING_DEG && now-TurnRampTick>=CHASSIS_MOTION_DEFAULT_RAMP_MS) {
      /* A proportional sub-RPM command rounds to zero before sub-degree arrival.
       * Only the precision turn uses the smallest realizable wheel speed. */
      float minimum=CHASSIS_MODEL_PI*Geometry.wheel_diameter_mm /
        (60*Geometry.gear_ratio*(Geometry.wheelbase_mm+Geometry.track_mm)*.5f);
      if(fabsf(command->omega_rad_s)<minimum)
        command->omega_rad_s=copysignf(fminf(minimum,HeadingPid.output_limit),Motion.error_heading_deg);
    }
    return true;
  }
  for(i=0;i<4;i++)
  {
    uint32_t speed_age=(uint32_t)(now-Position.wheels[i].speed_ms);
    uint32_t position_age=(uint32_t)(now-Position.wheels[i].position_ms);
    if(speed_age>age) age=speed_age;
    if(position_age>age) age=position_age;
    rpm[i]=(directions[i]?-1.0f:1.0f)*(float)Position.wheels[i].speed_rpm;
  }
  if(Motion.profile==CHASSIS_PROFILE_COSINE){
    /* A fresh group must not suddenly release the braking budget. Retain this
     * leg's observed age bound; the next action/curve exit starts a new bound. */
    if(age>DistanceFeedbackAge)DistanceFeedbackAge=age;
    age=DistanceFeedbackAge;
  }
  uint32_t curve_dt=now-CurveUpdateTick;
  if(curve_dt>=Motion.curve_period_ms)
  {
    CurveUpdateTick=now;dt=curve_dt;
    delay+=age/1000.0f;
    if(ChassisModel_Forward(&Geometry,rpm,&feedback))
      measured=hypotf(feedback.vx_mm_s,feedback.vy_mm_s);
    /* The cosine envelope already reserves response + feedback age. Subtracting
     * (measured - previous command) again feeds each deceleration back into its
     * own distance budget: old wheel speed can force zero far before the target.
     * Keep the original linear budget; cosine must not depend on its last output. */
    available=fmaxf(0,distance-CHASSIS_DISTANCE_TOLERANCE_MM*0.5f);
    if(Motion.profile==CHASSIS_PROFILE_LINEAR)
      available=fmaxf(0,available-fmaxf(0,measured-hypotf(MapVx,MapVy))*delay);
    if(Motion.profile==CHASSIS_PROFILE_COSINE){
      brake_speed=distance_brake_speed(available,delay);
      float ramp=DistanceRampFrom+(DistancePeakSpeed-DistanceRampFrom)*
          half_cosine((uint32_t)(now-DistanceRampTick)/(distance_ramp_seconds()*1000));
      /* Allow both feedback and the drive to settle before asking P to close
       * another error: two response/age horizons plus the cosine transition.
       * than racing a delayed endpoint with the full 2.5 gain. The distance-aware
       * reference stays nonzero before arrival; no latched zero or long crawl. */
      float gain=fminf(CHASSIS_DISTANCE_KP,1.0f/(2.0f*delay+distance_ramp_seconds()));
      limit=fminf(DistanceMaxSpeed,fminf(ramp,fminf(brake_speed,distance*gain)));
    }else{
      float a=Motion.linear_decel_mm_s2;
      brake_speed=sqrtf(a*a*delay*delay+2*a*available)-a*delay;
      limit=fminf(DistanceMaxSpeed,brake_speed);
    }
    if(DistanceCorrections)
      limit=fminf(limit,CHASSIS_DISTANCE_CORRECTION_SPEED_MM_S);
    else if(distance<=CHASSIS_DISTANCE_CREEP_MM)
      limit=fminf(limit,Motion.profile==CHASSIS_PROFILE_COSINE?
          fminf(CHASSIS_DISTANCE_CREEP_MM_S,CHASSIS_DISTANCE_TOLERANCE_MM/delay):
          CHASSIS_DISTANCE_CREEP_MM_S);
    if(Corner.lateral){
      /* Keep Y=2100 correction independent of the long westward distance. */
      vx=fmaxf(-limit,fminf(limit,vx));
      float cross=fabsf(Motion.error_y_mm),a=Motion.linear_decel_mm_s2;
      float cross_limit=fminf(CHASSIS_DISTANCE_CREEP_MM_S,sqrtf(a*a*delay*delay+2*a*cross)-a*delay);
      vy=fmaxf(-cross_limit,fminf(cross_limit,vy));
      norm=hypotf(vx,vy);
      if(norm>limit){vx*=limit/norm;vy*=limit/norm;}
    }else if(norm>limit) { vx*=limit/norm; vy*=limit/norm; }
    if(hypotf(Motion.error_x_mm,Motion.error_y_mm)<=CHASSIS_DISTANCE_TOLERANCE_MM) vx=vy=0;
    if(Motion.profile==CHASSIS_PROFILE_LINEAR){
      float dx=vx-MapVx,dy=vy-MapVy,change=hypotf(dx,dy);
      /* A negative projection change removes current translation momentum.
       * It includes deceleration and reversal, in either travel direction. */
      float rate=(dx*MapVx+dy*MapVy<0)?Motion.linear_decel_mm_s2:Motion.linear_accel_mm_s2;
      float step=rate*dt/1000.0f;
      if(Corner.lateral){
        /* Reserve a Y braking/correction share before accelerating west;
         * joint normalization would otherwise dilute Y by the 900 mm X error. */
        float yr=dy*MapVy<0?Motion.linear_decel_mm_s2:Motion.linear_accel_mm_s2;
        float ys=fminf(step*.70710678f,yr*dt/1000.0f);
        dy=fmaxf(-ys,fminf(ys,dy));
        float xr=dx*MapVx<0?Motion.linear_decel_mm_s2:Motion.linear_accel_mm_s2;
        float xs=fminf(xr*dt/1000.0f,sqrtf(fmaxf(0,step*step-dy*dy)));
        dx=fmaxf(-xs,fminf(xs,dx));
      }else if(change>step){dx*=step/change;dy*=step/change;}
      MapVx+=dx;MapVy+=dy;
    }else{
      /* Bound feedback-driven vector changes by this ramp's peak acceleration;
       * retain the requested half-cosine rather than imposing linear's 300 cap. */
      float dx=vx-MapVx,dy=vy-MapVy,change=hypotf(dx,dy);
      float step=CHASSIS_MODEL_PI*fmaxf(DistancePeakSpeed,DistanceRampFrom)*dt/
          (2.0f*distance_ramp_seconds()*1000.0f);
      if(change>step){dx*=step/change;dy*=step/change;}
      MapVx+=dx;MapVy+=dy;
    }
  }
  command->vx_mm_s=cosf(Motion.map_yaw_rad)*MapVx+sinf(Motion.map_yaw_rad)*MapVy;
  command->vy_mm_s=-sinf(Motion.map_yaw_rad)*MapVx+cosf(Motion.map_yaw_rad)*MapVy;
  return distance_heading_command(command);
}
void ChassisMotion_Process(void)
{
  uint32_t now = HAL_GetTick(), dt, elapsed, stop_start, total;
  float factor = 1, heading_output;
  bool heading_ok = heading_read(now);
  ChassisModel_Velocity_t command;
  ChassisModel_Wheels_t wheels;
  Mecanum_Status_t bus;
  unsigned i;
  stop_monitor_process(); /* Runs while idle too; independent of at-target checks. */
  Mecanum_Feedback_Arrival(ChassisMotion_IsBusy() && WaitingForStop && !FinishAsError &&
    ArrivalGroups==CHASSIS_ROUTE_STOP_GROUPS && !strcmp(Motion.reason,"position_reached"));
  if (!ChassisMotion_IsBusy())
    return;
  Motion.elapsed_ms=now-Motion.start_tick;
  if(WaitingForStop)Motion.stop_ms=now-DistanceStopTick;
  Mecanum_StatusGet(&bus);
  /* A bus-level emergency stop cancels the owning task as well. It must never
   * be followed by another refresh of the previous indefinite target. */
  if (!WaitingForStop && bus.stop_sequence!=TaskStopSequence)
  { request_stop("cancelled",false);return; }
  if (bus.stage == MECANUM_STAGE_FAULT)
  {
    if (WaitingForStop)
    {
      Motion.state = CHASSIS_MOTION_ERROR;
      Motion.reason = "stop_failed";
      WaitingForStop = false;
    }
    else
      request_stop("bus_fault", true);
    return;
  }
  if (Motion.distance_mode)
  {
    bool feedback_ok = distance_feedback_check(now);
    if(feedback_ok && Position.speed_valid){
      static const uint8_t dirs[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,
        CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,
        CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
      float rpm[4];bool fresh=true;ChassisModel_Velocity_t measured;
      for(unsigned w=0;w<4;w++){
        if(!Position.wheels[w].speed_valid || now-Position.wheels[w].speed_ms>600U)fresh=false;
        rpm[w]=(dirs[w]?-1.0f:1.0f)*Position.wheels[w].speed_rpm;
      }
      if(fresh && ChassisModel_Forward(&Geometry,rpm,&measured))
        Motion.peak_mm_s=fmaxf(Motion.peak_mm_s,hypotf(measured.vx_mm_s,measured.vy_mm_s));
    }
    /* A cancelled task is proving parking, not arrival at its old map target.
     * Recovery intentionally rebuilds localization after the stop; a transient
     * gap here must not relatch a motion error after fresh stop proof succeeds.
     * Active motion and normal position-arrival stops still require feedback. */
    if (!feedback_ok && !FinishAsError &&
        (!WaitingForStop || !strcmp(Motion.reason,"position_reached")))
    {
      if (!WaitingForStop) { request_stop("feedback_invalid", true); return; }
      FinishAsError=true;Motion.reason="feedback_invalid";
      if(DistanceBraking) { DistanceBraking=false;(void)Mecanum_Test_Stop(); }
    }
    if (WaitingForStop && (uint32_t)(now-DistanceStopTick)>=CHASSIS_DISTANCE_STOP_TIMEOUT_MS)
    {
      if(DistanceBraking) { DistanceBraking=false;(void)Mecanum_Test_Stop(); }
      Motion.state=CHASSIS_MOTION_ERROR;Motion.reason="stop_unconfirmed";
      WaitingForStop=false;return;
    }
  }
  if (WaitingForStop)
  {
    if(DistanceBraking)
    {
      if(bus.locked || bus.error!=MECANUM_ERROR_NONE)
      {
        DistanceBraking=false;FinishAsError=true;Motion.reason="bus_fault";
        (void)Mecanum_Test_Stop();return;
      }
      if(bus.stage!=MECANUM_STAGE_SENT || !bus.tx_complete || !bus.sent_velocity_valid ||
         bus.sent_rpm[0] || bus.sent_rpm[1] || bus.sent_rpm[2] || bus.sent_rpm[3]) return;
      if((uint32_t)(bus.sent_ms-DistanceStopTick)>0x7FFFFFFFU)
      {
        /* A just-sent identical zero may have been deduplicated. The request
         * API owns its refresh interval: retry until a post-stop TX completes. */
        (void)Mecanum_Velocity_Request(0,0,0,0);
        return;
      }
      DistanceBraking=false;
      if(!Mecanum_RequestPlannedStop())
      { Motion.state=CHASSIS_MOTION_ERROR;Motion.reason="stop_rejected";WaitingForStop=false; }
      return;
    }
    if (bus.stage == MECANUM_STAGE_STOPPED && !bus.stop_pending)
    {
      if (bus.error != MECANUM_ERROR_NONE)
      {
        FinishAsError = true;
        if (bus.error == MECANUM_ERROR_CANCELLED)
          Motion.reason = "sync_cache_uncertain";
        else if (!strcmp(Motion.reason, "timed_complete") || !strcmp(Motion.reason, "cancelled"))
          Motion.reason = "bus_fault";
      }
      if (Motion.distance_mode && !FinishAsError && !strcmp(Motion.reason,"position_reached"))
      {
        if (!stopped_feedback(now)) return;
        Motion.stop_confirmed = true;
        if(!in_target())
        {
          /* Keep the action ID, original deadline and normal motion guards.
             Corrections begin only after fresh physical stop evidence. */
          if(Motion.turn_only && (uint32_t)(now-Motion.start_tick)<DistanceTimeout && !bus.locked)
          {
            MapVx=MapVy=0;HeadingPid.output_rad_s=0;
            WaitingForStop=false;Motion.stop_confirmed=false;TaskStopSequence=bus.stop_sequence;
            LastControlTick=TurnRampTick=now;
            PidSampleTick=Motion.heading_sample_tick;PidSampleCount=LastYawCount;
            Motion.state=CHASSIS_MOTION_RUNNING;Motion.reason="heading_correcting";
            return;
          }
          if(!Motion.turn_only && DistanceCorrections<CHASSIS_DISTANCE_CORRECTIONS &&
             hypotf(Motion.error_x_mm,Motion.error_y_mm)<=CHASSIS_DISTANCE_CORRECTION_MM &&
             fabsf(Motion.error_heading_deg)<=CHASSIS_DISTANCE_HEADING_DEG &&
             (uint32_t)(now-Motion.start_tick)<DistanceTimeout && !bus.locked)
          {
            DistanceCorrections++;MapVx=MapVy=0;HeadingPid.output_rad_s=0;
            heading_rate_reset();
            distance_ramp_start(now);
            WaitingForStop=false;Motion.stop_confirmed=false;TaskStopSequence=bus.stop_sequence;
            LastControlTick=now;PidSampleTick=Motion.heading_sample_tick;PidSampleCount=LastYawCount;
            Motion.state=CHASSIS_MOTION_RUNNING;Motion.reason="position_correcting";
            return;
          }
          Motion.state=CHASSIS_MOTION_ERROR;Motion.reason=Motion.turn_only?"turn_not_reached":"position_not_reached";
          WaitingForStop=false;return;
        }
        Motion.reason = "feedback_arrived";
      }
      Motion.state = FinishAsError ? CHASSIS_MOTION_ERROR : CHASSIS_MOTION_DONE;
      WaitingForStop = false;
      hold_status_refresh();
    }
    return;
  }
  if (bus.locked && bus.error != MECANUM_ERROR_NONE)
  {
    request_stop("bus_fault", true);
    return;
  }
  if (HWT101_Cal_IsBusy())
  {
    request_stop("calibration_busy", true);
    return;
  }
  if (Motion.requested.mode != CHASSIS_MOTION_ANGULAR_VELOCITY && !heading_ok)
  {
    request_stop("imu_invalid", true);
    return;
  }
  if (Motion.requested.mode == CHASSIS_MOTION_HEADING && !Motion.map_anchor_valid)
  {
    request_stop("anchor_invalid", true);
    return;
  }
  dt = (uint32_t)(now - LastControlTick);
  /* Legacy direct velocity/hold do not integrate a motion ramp. Preserve their
   * continuity through a short scheduling gap; hold still checks fresh IMU and
   * its real elapsed duration. Planned trajectories retain their existing gap check. */
  if (dt > CHASSIS_MOTION_MAX_GAP_MS && !Motion.indefinite_velocity && !HoldTask)
  {
    Motion.control_dt_ms = dt;
    request_stop("control_gap", true);
    return;
  }
  if (dt < CHASSIS_MOTION_PERIOD_MS)
    return;
  Motion.control_dt_ms = dt;
  Motion.sample_tick = LastControlTick = now;
  elapsed = (uint32_t)(now - Motion.start_tick);
  Motion.elapsed_ms = elapsed;
  if (Motion.distance_mode)
  {
    if(elapsed>=DistanceTimeout) { request_stop(Motion.turn_only?"turn_timeout":"distance_timeout",true);return; }
    corner_handoff(now); /* Before the endpoint's zero-reference/stop branch. */
    /* Normal linear arrival finishes its translational ramp before zero TX.
     * Emergency/cancel and independent turn/path stops keep their own rules. */
    if(in_target() && (Motion.turn_only || FollowingPath || Motion.profile!=CHASSIS_PROFILE_LINEAR ||
                       (MapVx==0 && MapVy==0)))
    { request_stop("position_reached",false);return; }
    if(!distance_command(dt,&command)) { request_stop("heading_invalid",true);return; }
  }
  else if (ManualRamp)
  {
    if ((uint32_t)(now - StopRampTick) >= StopRampMs)
    {
      request_stop("cancelled", false);
      return;
    }
    factor = 1 - half_cosine((float)(uint32_t)(now - StopRampTick) / StopRampMs);
    command = StopFrom;
    command.vx_mm_s *= factor;
    command.vy_mm_s *= factor;
    command.omega_rad_s *= factor;
  }
  else
  {
    stop_start = Motion.requested.transition_ms + Motion.requested.hold_ms;
    total = stop_start + Motion.requested.stop_ms;
    if (!Motion.indefinite_velocity && elapsed >= total)
    {
      request_stop("timed_complete", false);
      return;
    }
    if (elapsed < Motion.requested.transition_ms)
      factor = half_cosine((float)elapsed / Motion.requested.transition_ms);
    else if (!Motion.indefinite_velocity && elapsed >= stop_start)
    {
      factor = 1 - half_cosine((float)(elapsed - stop_start) / Motion.requested.stop_ms);
      Motion.state = CHASSIS_MOTION_STOPPING;
    }
    command.vx_mm_s = Motion.requested.vx_mm_s * factor;
    command.vy_mm_s = Motion.requested.vy_mm_s * factor;
    command.omega_rad_s = Motion.requested.omega_rad_s * factor;
    if (Motion.requested.mode != CHASSIS_MOTION_ANGULAR_VELOCITY)
    {
      if (LastYawCount != PidSampleCount)
      {
        uint32_t sample_dt = (uint32_t)(Motion.heading_sample_tick - PidSampleTick);
        if (sample_dt > CHASSIS_MOTION_MAX_GAP_MS && !HoldTask)
        {
          request_stop("heading_gap", true);
          return;
        }
        if (sample_dt > 0)
        {
          heading_output = heading_update(Motion.current_deg);
          if (!isfinite(heading_output))
          {
            request_stop("heading_invalid", true);
            return;
          }
          HeadingPid.output_rad_s = heading_output;
          PidSampleTick = Motion.heading_sample_tick;
          PidSampleCount = LastYawCount;
        }
      }
      command.omega_rad_s = HeadingPid.output_rad_s * factor;
    }
  }
  if (!ChassisModel_Inverse(&Geometry, &command, CHASSIS_MOTION_MAX_WHEEL_RPM, &wheels))
  {
    request_stop("model_invalid", true);
    return;
  }
  command.vx_mm_s *= wheels.scale;
  command.vy_mm_s *= wheels.scale;
  command.omega_rad_s *= wheels.scale;
  Motion.body_target = command;
  Motion.rpm_scale = wheels.scale;
  for (i = 0; i < 4; i++)
  {
    Motion.rpm_unquantized[i] = wheels.rpm[i];
    Motion.rpm_command[i] = wheels.command[i];
  }
  if (!Mecanum_Velocity_Request(command.vx_mm_s, command.vy_mm_s, command.omega_rad_s, DriverAcceleration))
  {
    request_stop("velocity_rejected", true);
    return;
  }
}
