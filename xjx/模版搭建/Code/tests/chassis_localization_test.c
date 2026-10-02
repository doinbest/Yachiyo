#include "chassis_localization.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static bool motion_busy, bus_busy, route_busy;
static Mecanum_Feedback_t wheels[4];
static ChassisMotion_Status_t motion;
uint32_t HAL_GetTick(void) { return tick; }
bool Mecanum_IsBusy(void) { return bus_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
bool ChassisRoute_IsBusy(void) { return route_busy; }
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out) { *out = motion; }
bool ChassisMotion_AnchorSet(float yaw)
{
  if (!motion.heading_valid) return false;
  motion.map_anchor_valid = true;
  motion.map_yaw_rad = yaw;
  return true;
}
void Mecanum_StatusGet(Mecanum_Status_t *out) { memset(out, 0, sizeof(*out)); }
bool Mecanum_FeedbackGet(unsigned i, Mecanum_Feedback_t *out) { *out = wheels[i]; return true; }
static void group(uint32_t time, int64_t position)
{
  const uint8_t dirs[4] = {
      CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR, CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
      CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR, CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  unsigned i;
  tick = time;
  for (i = 0; i < 4; i++)
  {
    wheels[i].position_raw = dirs[i] ? -position : position;
    wheels[i].position_ms = wheels[i].speed_ms = time;
    wheels[i].position_valid = wheels[i].speed_valid = true;
  }
  ChassisLocalization_Process();
}
int main(void)
{
  ChassisLocalization_Status_t s;
  uint32_t generation;
  ChassisLocalization_Init();
  motion.heading_valid = true;
  assert(!ChassisLocalization_ConfirmPositionUnits(16384));
  assert(ChassisLocalization_ConfirmPositionUnits(65536));
  assert(ChassisLocalization_Origin(100, 200, 0));
  group(100, 0);
  ChassisLocalization_Get(&s);
  assert(!s.feedback_valid && s.feedback_sequence == 0 && s.feedback_tick == 0);
  group(200, 655);
  ChassisLocalization_Get(&s);
  assert(s.feedback_valid && s.speed_valid && s.feedback_sequence == 1 && s.feedback_tick == 200);
  assert(s.observer.feedback.x_mm > 102 && s.origin.x_mm == 100 && s.origin.y_mm == 200);
  generation = s.generation;
  tick = 220; ChassisLocalization_Process(); ChassisLocalization_Get(&s);
  assert(s.feedback_sequence == 1 && s.feedback_tick == 200);
  wheels[0].position_ms = 240; tick = 240;
  ChassisLocalization_Process(); ChassisLocalization_Get(&s);
  assert(s.feedback_sequence == 1 && s.wheels[0].position_ms == 240);
  /* Staleness invalidates on an ordinary loop, not only complete groups. */
  tick = 800; ChassisLocalization_Process();
  tick = 801; ChassisLocalization_Process(); ChassisLocalization_Get(&s);
  assert(!s.feedback_valid && !s.speed_valid && s.generation == generation + 1);
  assert(s.feedback_sequence == 1);
  tick = 802; ChassisLocalization_Process(); ChassisLocalization_Get(&s);
  assert(s.generation == generation + 1);
  group(900, 1000); ChassisLocalization_Get(&s);
  assert(!s.feedback_valid && s.feedback_sequence == 1);
  group(1000, 1100); ChassisLocalization_Get(&s);
  assert(s.feedback_valid && s.feedback_sequence == 2 && s.generation == generation + 1);
  motion.map_anchor_valid = false; tick = 1001;
  ChassisLocalization_Process(); ChassisLocalization_Get(&s);
  assert(!s.origin_valid && !s.feedback_valid && s.generation == generation + 2);
  assert(ChassisLocalization_Origin(0, 0, 0));
  ChassisLocalization_Get(&s); generation = s.generation;
  route_busy = true;
  assert(!ChassisLocalization_Origin(0, 0, 0));
  assert(!ChassisLocalization_ConfirmPositionUnits(0));
  route_busy = false; motion_busy = true;
  assert(!ChassisLocalization_Origin(0, 0, 0));
  assert(!ChassisLocalization_ConfirmPositionUnits(0));
  motion_busy = false; bus_busy = true;
  assert(!ChassisLocalization_Origin(0, 0, 0));
  bus_busy = false;
  assert(!ChassisLocalization_Origin(NAN, 0, 0));
  assert(ChassisLocalization_ConfirmPositionUnits(65536));
  ChassisLocalization_Get(&s); assert(s.generation == generation);
  assert(ChassisLocalization_ConfirmPositionUnits(0));
  ChassisLocalization_Get(&s); assert(s.generation == generation + 1);
  assert(!s.feedback_valid);
  puts("localization: baseline, complete groups, freshness, generation, origin and busy guards PASS");
  return 0;
}
