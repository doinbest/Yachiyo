#include "chassis_observer.h"
#include <string.h>
#include <math.h>
void ChassisObserver_Init(ChassisObserver_t *o, float x, float y, float yaw)
{
  memset(o, 0, sizeof(*o));
  o->command.x_mm = o->feedback.x_mm = x;
  o->command.y_mm = o->feedback.y_mm = y;
  o->command.yaw_rad = o->feedback.yaw_rad = yaw;
}
void ChassisObserver_Command(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                             const float rpm[4], uint32_t now, uint32_t applied_ms, bool valid)
{
  ChassisModel_Velocity_t v;
  uint32_t dt = now - o->command_ms, after = 0;
  if (!valid || !ChassisModel_Forward(g, rpm, &v))
  {
    o->command_valid = false;
    o->command_baseline = false;
    return;
  }
  if (o->command_baseline && dt <= 100U)
  {
    if (applied_ms != o->applied_ms)
    {
      after = now - applied_ms;
      if (after > dt)
      {
        o->command_valid = false;
        goto baseline;
      }
    }
    (void)ChassisModel_Forward(g, o->previous_rpm, &v);
    o->command_valid = ChassisModel_IntegrateMidpoint(&o->command, &v, (dt - after) / 1000.0f);
    if (after)
    {
      (void)ChassisModel_Forward(g, rpm, &v);
      o->command_valid = ChassisModel_IntegrateMidpoint(&o->command, &v, after / 1000.0f);
    }
  }
  else
    o->command_valid = false;
baseline:
  memcpy(o->previous_rpm, rpm, sizeof(o->previous_rpm));
  o->applied_ms = applied_ms;
  o->command_ms = now;
  o->command_baseline = true;
}
void ChassisObserver_Feedback(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                              const ChassisObserver_Feedback_t *f)
{
  uint32_t oldest = 0, newest = 0xFFFFFFFFU, age, i;
  float rpm[4], dt_sum = 0, dyaw;
  bool new_group = true;
  ChassisModel_Velocity_t v;
  if (!f->units_per_rev || !f->yaw_valid || !isfinite(f->yaw_rad))
    goto invalid;
  for (i = 0; i < 4; i++)
  {
    age = f->now_ms - f->time_ms[i];
    if (!f->valid[i] || age > 600U)
      goto invalid;
    if (age > oldest)
      oldest = age;
    if (age < newest)
      newest = age;
    if (f->time_ms[i] == o->previous_ms[i])
      new_group = false;
  }
  o->feedback_span_ms = oldest - newest;
  if (o->feedback_span_ms > 250U)
    goto invalid;
  if (o->feedback_baseline && !new_group)
    return;
  if (o->feedback_baseline)
  {
    for (i = 0; i < 4; i++)
    {
      uint32_t dt = f->time_ms[i] - o->previous_ms[i];
      if (!dt || dt > 600U)
        goto invalid;
      rpm[i] = (float)(f->position[i] - o->previous_position[i]) * 60000.0f /
               ((float)f->units_per_rev * dt);
      dt_sum += dt / 4000.0f;
    }
    if (!ChassisModel_Forward(g, rpm, &v))
      goto invalid;
    dyaw = f->yaw_rad - o->previous_yaw;
    while (dyaw > CHASSIS_MODEL_PI)
      dyaw -= 2 * CHASSIS_MODEL_PI;
    while (dyaw < -CHASSIS_MODEL_PI)
      dyaw += 2 * CHASSIS_MODEL_PI;
    o->feedback.yaw_rad = o->previous_yaw;
    v.omega_rad_s = dyaw / dt_sum;
    o->feedback_valid = ChassisModel_IntegrateMidpoint(&o->feedback, &v, dt_sum);
    o->feedback.yaw_rad = f->yaw_rad;
  }
  else
    o->feedback_valid = false;
  for (i = 0; i < 4; i++)
  {
    o->previous_position[i] = f->position[i];
    o->previous_ms[i] = f->time_ms[i];
  }
  o->previous_yaw = f->yaw_rad;
  o->feedback_baseline = true;
  return;
invalid:
  o->feedback_valid = false;
  o->feedback_baseline = false;
}
