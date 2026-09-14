#include "chassis_localization.h"
#include "chassis_config.h"
#include "chassis_motion.h"
#include "chassis_route.h"
#include <math.h>
#include <string.h>

static const ChassisModel_Geometry_t geometry = {
    CHASSIS_WHEELBASE_MM, CHASSIS_TRACK_WIDTH_MM,
    CHASSIS_WHEEL_DIAMETER_MM, CHASSIS_MOTOR_TO_WHEEL_RATIO};
static const uint8_t dirs[4] = {
    CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR, CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR, CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
static ChassisLocalization_Status_t status;
static uint32_t last_observe;

void ChassisLocalization_Init(void)
{
  memset(&status, 0, sizeof(status));
  last_observe = HAL_GetTick() - 20U;
  ChassisObserver_Init(&status.observer, 0, 0, 0);
}

void ChassisLocalization_Get(ChassisLocalization_Status_t *out)
{
  if (out)
    *out = status;
}

bool ChassisLocalization_Origin(float x, float y, float yaw)
{
  if (!isfinite(x) || !isfinite(y) || !isfinite(yaw) || fabsf(x) > 10000 || fabsf(y) > 10000 ||
      ChassisMotion_IsBusy() || Mecanum_IsBusy() || ChassisRoute_IsBusy() ||
      !ChassisMotion_AnchorSet(yaw))
    return false;
  ChassisObserver_Init(&status.observer, x, y, yaw);
  status.origin = status.observer.feedback;
  status.origin_valid = true;
  status.feedback_valid = false;
  status.generation++;
  return true;
}

bool ChassisLocalization_ConfirmPositionUnits(uint32_t value)
{
  if (ChassisMotion_IsBusy() || Mecanum_IsBusy() || ChassisRoute_IsBusy() ||
      (value != 0U && value != CHASSIS_EMM_POSITION_UNITS_PER_REV))
    return false;
  if (status.units_per_rev != value)
  {
    status.units_per_rev = value;
    status.observer.feedback_baseline = status.observer.feedback_valid = false;
    status.feedback_valid = false;
    status.generation++;
  }
  return true;
}

void ChassisLocalization_Process(void)
{
  static ChassisMotion_Status_t motion;
  static Mecanum_Status_t tx;
  static ChassisObserver_Feedback_t sample;
  uint32_t now = HAL_GetTick();
  bool fresh = true, changed = true, lost;
  bool previously_valid = status.feedback_valid;
  float rpm[4];
  unsigned i;
  ChassisMotion_StatusGet(&motion);
  sample.yaw_valid = motion.heading_valid && motion.map_anchor_valid && status.origin_valid;
  sample.yaw_rad = motion.map_yaw_rad;
  lost = status.origin_valid && !motion.map_anchor_valid;
  if (lost)
    status.origin_valid = false;
  status.speed_valid = true;
  for (i = 0; i < 4; i++)
  {
    uint32_t age;
    memset(&status.wheels[i], 0, sizeof(status.wheels[i]));
    (void)Mecanum_FeedbackGet(i, &status.wheels[i]);
    sample.position[i] = dirs[i] ? -status.wheels[i].position_raw : status.wheels[i].position_raw;
    sample.time_ms[i] = status.wheels[i].position_ms;
    sample.valid[i] = status.wheels[i].position_valid;
    age = now - sample.time_ms[i];
    if (!sample.valid[i] || age > 600U)
      fresh = false;
    if (sample.time_ms[i] == status.observer.previous_ms[i])
      changed = false;
    if (!status.wheels[i].speed_valid || now - status.wheels[i].speed_ms > 600U)
      status.speed_valid = false;
  }
  fresh = fresh && sample.yaw_valid &&
          isfinite(sample.yaw_rad) && status.units_per_rev != 0;
  /* Check loss on every loop, even when no complete new wheel group arrives. */
  if (!fresh)
  {
    status.observer.feedback_valid = false;
    status.observer.feedback_baseline = false;
  }
  if ((uint32_t)(now - last_observe) >= 20U)
  {
    bool had_baseline = status.observer.feedback_baseline;
    last_observe = now;
    Mecanum_StatusGet(&tx);
    for (i = 0; i < 4; i++)
      rpm[i] = (float)tx.sent_rpm[i];
    sample.now_ms = now;
    sample.units_per_rev = status.units_per_rev;
    ChassisObserver_Command(&status.observer, &geometry, rpm, now, tx.sent_ms,
        !tx.locked && tx.sent_velocity_valid &&
        (tx.stage == MECANUM_STAGE_STOPPED || (tx.sent_ms && now - tx.sent_ms <= 600U)));
    /* A partial replacement mixes two polling rounds. Validate group skew only
     * when all four positions are new; individual age limits still apply above. */
    if (!had_baseline || changed || !fresh)
      ChassisObserver_Feedback(&status.observer, &geometry, &sample);
    if (had_baseline && changed && status.observer.feedback_valid)
    {
      status.feedback_sequence++;
      status.feedback_tick = now;
    }
  }
  status.feedback_valid = fresh && status.observer.feedback_valid;
  if (lost || (previously_valid && !status.feedback_valid))
    status.generation++;
}
