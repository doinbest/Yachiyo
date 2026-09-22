#include "chassis_motion.h"
#include "chassis_config.h"
#include "chassis_localization.h"
#include "chassis_route.h"
#include "mecanum_chassis.h"
#include "hwt101_calibration.h"
#include "ArmVision.h"
#include "MaterialVision.h"
#include "mechanical_arm.h"
#include <math.h>
#include <string.h>

static ChassisMotion_Status_t Motion;
static Mecanum_HeadingPid_t HeadingPid;
static bool WaitingForStop, FinishAsError, ManualRamp, HaveYaw, DistanceBraking;
static uint32_t LastControlTick, StopRampTick, StopRampMs, LastYawCount, PidSampleCount,
    PidSampleTick;
static float PreviousYaw, AnchorYaw, AnchorMapYaw;
static ChassisModel_Velocity_t StopFrom;
static uint32_t DistanceGeneration, DistanceTimeout, DistanceStopTick, StopSequence,
                StableSince, StableGroups;
static uint32_t StopSpeedTime[4];
static int64_t StopPositions[4];
static float DistanceMaxSpeed, MapVx, MapVy;
static unsigned DistanceCorrections;
static ChassisLocalization_Status_t Position; /* Main-loop snapshot; avoid large stack use. */
static ChassisStop_Status_t StopStatus;
static bool StopMonitoring, StopHaveGroup;
static uint32_t StopMotionSequence, StopDispatchSequence, StopActionId, StopTxTick;
static uint32_t StopQuietSince, StopQuietGroups, StopUnits;
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
}
/* Every read passes through the calibration module's qualification and age gate. */
static bool heading_read(uint32_t now)
{
  HWT101_Angle_t angle;
  if (!HWT101_Cal_ControlAngleGet(&angle) || !isfinite(angle.yaw) ||
      (uint32_t)(now - angle.last_update_ms) > HWT101_DATA_FRESH_MS)
  {
    ChassisMotion_AnchorInvalidate();
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
  memset(&StopStatus, 0, sizeof(StopStatus));
  StopStatus.reason = "idle";
  StopMonitoring = StopHaveGroup = false;
  StopQuietGroups = 0;
  memset(&HeadingPid, 0, sizeof(HeadingPid));
  Motion.reason = "idle";
  Motion.rpm_scale = 1;
  MapVx = MapVy = 0;
  StableGroups = 0;
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
static bool target_valid(const ChassisMotion_Target_t *t)
{
  uint32_t total;
  if (t == NULL || !isfinite(t->vx_mm_s) || !isfinite(t->vy_mm_s) || !isfinite(t->omega_rad_s) ||
      !isfinite(t->heading_deg))
    return false;
  if (hypotf(t->vx_mm_s, t->vy_mm_s) > CHASSIS_MOTION_MAX_LINEAR_MM_S ||
      fabsf(t->omega_rad_s) > CHASSIS_MOTION_MAX_OMEGA_RAD_S)
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
  return total >= CHASSIS_MOTION_PERIOD_MS && total <= CHASSIS_MOTION_MAX_DURATION_MS;
}
bool ChassisMotion_Start(const ChassisMotion_Target_t *t)
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
  if (!target_valid(t))
  {
    Motion.reason = "invalid_target";
    return false;
  }
  if (HWT101_Cal_IsBusy() || ArmVision_IsBusy() || MaterialVision_IsBusy() ||
      MechanicalArm_IsBusy() || Mecanum_IsBusy())
  {
    Motion.reason = "busy";
    return false;
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
  Motion.distance_mode = Motion.stop_confirmed = false;
  Motion.requested = *t;
  Motion.requested.heading_deg = wrap_deg(t->heading_deg);
  if (t->mode == CHASSIS_MOTION_HOLD_CURRENT)
    Motion.requested.heading_deg = wrap_deg(Motion.current_deg);
  Motion.state = CHASSIS_MOTION_RUNNING;
  Motion.reason = "running";
  Motion.action_id++;
  Motion.start_tick = Motion.sample_tick = LastControlTick = now;
  Motion.elapsed_ms = Motion.control_dt_ms = 0;
  memset(&Motion.body_target, 0, sizeof(Motion.body_target));
  memset(Motion.rpm_unquantized, 0, sizeof(Motion.rpm_unquantized));
  memset(Motion.rpm_command, 0, sizeof(Motion.rpm_command));
  Motion.rpm_scale = 1;
  WaitingForStop = FinishAsError = ManualRamp = DistanceBraking = false;
  if (t->mode != CHASSIS_MOTION_ANGULAR_VELOCITY)
  {
    Mecanum_HeadingPid_Init(&HeadingPid, CHASSIS_HEADING_TEST_KP, 0, 0,
                            CHASSIS_MOTION_PERIOD_MS / 1000.0f, 0, CHASSIS_MOTION_MAX_OMEGA_RAD_S);
    Mecanum_HeadingPid_Set_Target(&HeadingPid, Motion.requested.heading_deg);
    PidSampleTick = Motion.heading_sample_tick;
    PidSampleCount = LastYawCount;
  }
  return true;
}
bool ChassisMotion_MoveTo(float x, float y, float yaw, float speed, uint32_t timeout)
{
  ChassisMotion_Target_t t;
  if (ChassisMotion_IsBusy()) return false;
  ChassisLocalization_Get(&Position);
  if (!isfinite(x) || !isfinite(y) || !isfinite(yaw) || !isfinite(speed) ||
      fabsf(x) > 10000 || fabsf(y) > 10000 || speed <= 0 ||
      speed > CHASSIS_MOTION_MAX_LINEAR_MM_S || timeout < CHASSIS_MOTION_PERIOD_MS ||
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
  if (!ChassisMotion_Start(&t)) return false;
  Motion.distance_mode = true;
  Motion.target_x_mm = x; Motion.target_y_mm = y; Motion.target_map_yaw_deg = wrap_deg(yaw);
  Motion.error_x_mm = x - Position.observer.feedback.x_mm;
  Motion.error_y_mm = y - Position.observer.feedback.y_mm;
  Motion.error_heading_deg = wrap_deg(yaw - Motion.map_yaw_rad*180.0f/CHASSIS_MODEL_PI);
  DistanceGeneration = Position.generation; DistanceTimeout = timeout; DistanceMaxSpeed = speed;
  MapVx = MapVy = 0; StableGroups = 0; DistanceCorrections = 0;
  return true;
}
static bool request_stop(const char *reason, bool error)
{
  Motion.reason = reason;
  Motion.stop_confirmed = false;
  FinishAsError = error;
  Motion.state = CHASSIS_MOTION_STOPPING;
  WaitingForStop = true;
  DistanceStopTick = HAL_GetTick();
  StableGroups = 0;
  if (Motion.distance_mode)
  {
    unsigned i;
    ChassisLocalization_Get(&Position);
    StopSequence = Position.feedback_sequence;
    for (i=0;i<4;i++) { StopPositions[i]=Position.wheels[i].position_raw; StopSpeedTime[i]=Position.wheels[i].speed_ms; }
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
    return;
  }
  if (!StopMonitoring) return;
  if (bus.stage == MECANUM_STAGE_FAULT || bus.stop_sequence != StopDispatchSequence)
  {
    StopStatus.wheels_stopped = false;
    StopStatus.reason = "tx_failed";
    StopMonitoring = false;
    return;
  }
  if (!StopStatus.tx_complete)
  {
    if (bus.stage != MECANUM_STAGE_STOPPED || bus.stop_pending || !bus.tx_complete)
    {
      if ((uint32_t)(now - StopStatus.requested_ms) >= 6000U)
      { StopStatus.reason = "tx_failed"; StopMonitoring = false; }
      return;
    }
    StopStatus.tx_complete = true;
    StopTxTick = bus.sent_ms;
    StopStatus.reason = "monitoring";
  }
  ChassisLocalization_Get(&Position);
  if (Position.units_per_rev != CHASSIS_EMM_POSITION_UNITS_PER_REV) fresh = false;
  if (StopHaveGroup && StopUnits != Position.units_per_rev)
  { StopHaveGroup = false; StopQuietGroups = 0; }
  for (i = 0; i < 4; i++)
  {
    const Mecanum_Feedback_t *w = &Position.wheels[i];
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
    if (StopHaveGroup && Position.units_per_rev)
    {
      float delta = fabsf((float)(w->position_raw - StopPrevious[i].position_raw)) *
                    CHASSIS_MODEL_PI * Geometry.wheel_diameter_mm /
                    ((float)Position.units_per_rev * Geometry.gear_ratio);
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
        if (!StopQuietGroups) StopQuietSince = now - newest;
        StopQuietGroups++;
        if (StopQuietGroups >= CHASSIS_DISTANCE_STOP_GROUPS &&
            (uint32_t)(now - oldest - StopQuietSince) >= CHASSIS_DISTANCE_STOP_STABLE_MS &&
            (uint32_t)(now - oldest - StopQuietSince) < 0x80000000U)
        { StopStatus.wheels_stopped = true; StopStatus.reason = "stopped"; }
        else StopStatus.reason = "monitoring";
      }
      memcpy(StopPrevious, Position.wheels, sizeof(StopPrevious));
      StopUnits = Position.units_per_rev;
      StopHaveGroup = true;
    }
  }
  if (!StopStatus.wheels_stopped && (uint32_t)(now - StopStatus.requested_ms) >= 6000U)
  {
    if (fresh) StopStatus.reason = "timeout";
    StopMonitoring = false;
  }
}

void ChassisMotion_StopStatusGet(ChassisStop_Status_t *out)
{
  stop_monitor_process();
  if (out) *out = StopStatus;
}
bool ChassisMotion_AnchorSet(float map_yaw_rad)
{
  if (!isfinite(map_yaw_rad) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy() || Mecanum_IsBusy() || HWT101_Cal_IsBusy() ||
      ArmVision_IsBusy() || MaterialVision_IsBusy() || MechanicalArm_IsBusy())
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
  return hypotf(Motion.error_x_mm,Motion.error_y_mm)<=CHASSIS_DISTANCE_TOLERANCE_MM &&
         fabsf(Motion.error_heading_deg)<=CHASSIS_DISTANCE_HEADING_DEG;
}
/* Only complete groups received after the stop request count as stop evidence. */
static bool stopped_feedback(uint32_t now)
{
  unsigned i;
  bool quiet = true; /* Physical stop evidence is independent of target accuracy. */
  if (Position.feedback_sequence == StopSequence) return false;
  StopSequence = Position.feedback_sequence;
  for(i=0;i<4;i++)
  {
    const Mecanum_Feedback_t *w = &Position.wheels[i];
    float delta = fabsf((float)(w->position_raw - StopPositions[i])) * CHASSIS_MODEL_PI *
                  Geometry.wheel_diameter_mm / ((float)Position.units_per_rev * Geometry.gear_ratio);
    if (!w->speed_valid || (uint32_t)(now-w->speed_ms)>600U ||
        w->speed_ms == StopSpeedTime[i] || (uint32_t)(w->speed_ms-DistanceStopTick)>0x7FFFFFFFU ||
        (uint32_t)(w->position_ms-DistanceStopTick)>0x7FFFFFFFU ||
        w->speed_rpm > CHASSIS_DISTANCE_STOP_RPM || w->speed_rpm < -CHASSIS_DISTANCE_STOP_RPM ||
        delta > CHASSIS_DISTANCE_STOP_DELTA_MM) quiet = false;
    StopPositions[i]=w->position_raw; StopSpeedTime[i]=w->speed_ms;
  }
  if(!quiet) { StableGroups=0; return false; }
  if(!StableGroups) StableSince=now;
  StableGroups++;
  return StableGroups>=CHASSIS_DISTANCE_STOP_GROUPS && (uint32_t)(now-StableSince)>=CHASSIS_DISTANCE_STOP_STABLE_MS;
}
static bool distance_command(uint32_t dt, ChassisModel_Velocity_t *command)
{
  float vx=Motion.error_x_mm*CHASSIS_DISTANCE_KP, vy=Motion.error_y_mm*CHASSIS_DISTANCE_KP;
  float norm=hypotf(vx,vy), dx,dy,step=CHASSIS_DISTANCE_ACCEL_MM_S2*dt/1000.0f;
  static const uint8_t directions[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,
      CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,
      CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  float distance=hypotf(Motion.error_x_mm,Motion.error_y_mm);
  float delay=CHASSIS_DISTANCE_RESPONSE_S;
  float rpm[4], measured=0, available, brake_speed, limit;
  ChassisModel_Velocity_t feedback;
  unsigned i;
  uint32_t age=0, now=HAL_GetTick();
  for(i=0;i<4;i++)
  {
    uint32_t speed_age=(uint32_t)(now-Position.wheels[i].speed_ms);
    uint32_t position_age=(uint32_t)(now-Position.wheels[i].position_ms);
    if(speed_age>age) age=speed_age;
    if(position_age>age) age=position_age;
    rpm[i]=(directions[i]?-1.0f:1.0f)*(float)Position.wheels[i].speed_rpm;
  }
  delay+=age/1000.0f;
  if(ChassisModel_Forward(&Geometry,rpm,&feedback))
    measured=hypotf(feedback.vx_mm_s,feedback.vy_mm_s);
  /* v*delay + v^2/(2*a) <= remaining distance. Account for drive lag
     when measured speed exceeds our commanded ramp state. */
  available=fmaxf(0,distance-CHASSIS_DISTANCE_TOLERANCE_MM*0.5f-
      fmaxf(0,measured-hypotf(MapVx,MapVy))*delay);
  brake_speed=sqrtf(CHASSIS_DISTANCE_ACCEL_MM_S2*CHASSIS_DISTANCE_ACCEL_MM_S2*delay*delay+
      2*CHASSIS_DISTANCE_ACCEL_MM_S2*available)-CHASSIS_DISTANCE_ACCEL_MM_S2*delay;
  limit=fminf(DistanceMaxSpeed,brake_speed);
  if(distance<=CHASSIS_DISTANCE_CREEP_MM || DistanceCorrections)
    limit=fminf(limit,CHASSIS_DISTANCE_CREEP_MM_S);
  if(norm>limit) { vx*=limit/norm; vy*=limit/norm; }
  if(hypotf(Motion.error_x_mm,Motion.error_y_mm)<=CHASSIS_DISTANCE_TOLERANCE_MM) vx=vy=0;
  dx=vx-MapVx; dy=vy-MapVy; norm=hypotf(dx,dy);
  if(norm>step) { dx*=step/norm;dy*=step/norm; }
  MapVx+=dx;MapVy+=dy;
  command->vx_mm_s=cosf(Motion.map_yaw_rad)*MapVx+sinf(Motion.map_yaw_rad)*MapVy;
  command->vy_mm_s=-sinf(Motion.map_yaw_rad)*MapVx+cosf(Motion.map_yaw_rad)*MapVy;
  if(LastYawCount!=PidSampleCount)
  {
    uint32_t elapsed=Motion.heading_sample_tick-PidSampleTick;
    if(elapsed>CHASSIS_MOTION_MAX_GAP_MS) return false;
    if(elapsed)
    {
      HeadingPid.sample_time_s=elapsed/1000.0f;
      HeadingPid.output_rad_s=Mecanum_HeadingPid_Update(&HeadingPid,Motion.current_deg);
      PidSampleTick=Motion.heading_sample_tick;PidSampleCount=LastYawCount;
    }
  }
  command->omega_rad_s=HeadingPid.output_rad_s;
  return isfinite(command->omega_rad_s);
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
  if (!ChassisMotion_IsBusy())
    return;
  Mecanum_StatusGet(&bus);
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
    if (!feedback_ok && !FinishAsError)
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
      if(!Mecanum_CanStopCleanly() || bus.stage!=MECANUM_STAGE_SENT || !bus.tx_complete || !bus.sent_velocity_valid ||
         (uint32_t)(bus.sent_ms-DistanceStopTick)>0x7FFFFFFFU ||
         bus.sent_rpm[0] || bus.sent_rpm[1] || bus.sent_rpm[2] || bus.sent_rpm[3]) return;
      DistanceBraking=false;
      if(!Mecanum_Test_Stop())
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
          if(DistanceCorrections<CHASSIS_DISTANCE_CORRECTIONS &&
             hypotf(Motion.error_x_mm,Motion.error_y_mm)<=CHASSIS_DISTANCE_CORRECTION_MM &&
             fabsf(Motion.error_heading_deg)<=CHASSIS_DISTANCE_HEADING_DEG &&
             (uint32_t)(now-Motion.start_tick)<DistanceTimeout && !bus.locked)
          {
            DistanceCorrections++;MapVx=MapVy=0;HeadingPid.output_rad_s=0;
            WaitingForStop=false;Motion.stop_confirmed=false;
            LastControlTick=now;PidSampleTick=Motion.heading_sample_tick;PidSampleCount=LastYawCount;
            Motion.state=CHASSIS_MOTION_RUNNING;Motion.reason="position_correcting";
            return;
          }
          Motion.state=CHASSIS_MOTION_ERROR;Motion.reason="position_not_reached";
          WaitingForStop=false;return;
        }
        Motion.reason = "feedback_arrived";
      }
      Motion.state = FinishAsError ? CHASSIS_MOTION_ERROR : CHASSIS_MOTION_DONE;
      WaitingForStop = false;
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
  if (dt > CHASSIS_MOTION_MAX_GAP_MS)
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
    if(elapsed>=DistanceTimeout) { request_stop("distance_timeout",true);return; }
    if(in_target()) { request_stop("position_reached",false);return; }
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
    if (elapsed >= total)
    {
      request_stop("timed_complete", false);
      return;
    }
    if (elapsed < Motion.requested.transition_ms)
      factor = half_cosine((float)elapsed / Motion.requested.transition_ms);
    else if (elapsed >= stop_start)
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
        if (sample_dt > CHASSIS_MOTION_MAX_GAP_MS)
        {
          request_stop("heading_gap", true);
          return;
        }
        if (sample_dt > 0)
        {
          HeadingPid.sample_time_s = sample_dt / 1000.0f;
          heading_output = Mecanum_HeadingPid_Update(&HeadingPid, Motion.current_deg);
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
  if (!Mecanum_Velocity_Request(command.vx_mm_s, command.vy_mm_s, command.omega_rad_s, 0))
  {
    request_stop("velocity_rejected", true);
    return;
  }
}
