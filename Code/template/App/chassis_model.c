#include "chassis_model.h"
#include <math.h>
#include <stddef.h>
static bool geometry_valid(const ChassisModel_Geometry_t *g)
{
  return g != NULL && isfinite(g->wheelbase_mm) && g->wheelbase_mm > 0 && isfinite(g->track_mm) &&
         g->track_mm > 0 && isfinite(g->wheel_diameter_mm) && g->wheel_diameter_mm > 0 &&
         isfinite(g->gear_ratio) && g->gear_ratio > 0;
}
static bool velocity_valid(const ChassisModel_Velocity_t *v)
{
  return v != NULL && isfinite(v->vx_mm_s) && isfinite(v->vy_mm_s) && isfinite(v->omega_rad_s);
}
bool ChassisModel_Inverse(const ChassisModel_Geometry_t *g, const ChassisModel_Velocity_t *v,
                          float max_rpm, ChassisModel_Wheels_t *out)
{
  ChassisModel_Wheels_t result;
  float k, conversion, rotation, largest = 0;
  unsigned i;
  if (!geometry_valid(g) || !velocity_valid(v) || out == NULL || !isfinite(max_rpm) ||
      max_rpm < 1 || max_rpm > 32767)
    return false;
  k = (g->wheelbase_mm + g->track_mm) * .5f;
  conversion = 60 * g->gear_ratio / (CHASSIS_MODEL_PI * g->wheel_diameter_mm);
  rotation = k * v->omega_rad_s;
  if (!isfinite(k) || !isfinite(conversion) || !isfinite(rotation) || conversion <= 0)
    return false;
  result.rpm[0] = (v->vx_mm_s - v->vy_mm_s - rotation) * conversion;
  result.rpm[1] = (v->vx_mm_s + v->vy_mm_s - rotation) * conversion;
  result.rpm[2] = (v->vx_mm_s - v->vy_mm_s + rotation) * conversion;
  result.rpm[3] = (v->vx_mm_s + v->vy_mm_s + rotation) * conversion;
  for (i = 0; i < 4; i++)
  {
    if (!isfinite(result.rpm[i]))
      return false;
    if (fabsf(result.rpm[i]) > largest)
      largest = fabsf(result.rpm[i]);
  }
  result.scale = largest > max_rpm ? max_rpm / largest : 1;
  for (i = 0; i < 4; i++)
  {
    result.rpm[i] *= result.scale;
    result.command[i] = (int16_t)roundf(result.rpm[i]);
  }
  *out = result;
  return true;
}
bool ChassisModel_Forward(const ChassisModel_Geometry_t *g, const float rpm[4],
                          ChassisModel_Velocity_t *out)
{
  ChassisModel_Velocity_t result;
  float v[4], conversion, k;
  unsigned i;
  if (!geometry_valid(g) || rpm == NULL || out == NULL)
    return false;
  k = (g->wheelbase_mm + g->track_mm) * .5f;
  conversion = CHASSIS_MODEL_PI * g->wheel_diameter_mm / (60 * g->gear_ratio);
  if (!isfinite(k) || !isfinite(conversion) || conversion <= 0)
    return false;
  for (i = 0; i < 4; i++)
  {
    v[i] = rpm[i] * conversion;
    if (!isfinite(v[i]))
      return false;
  }
  result.vx_mm_s = (v[0] + v[1] + v[2] + v[3]) * .25f;
  result.vy_mm_s = (-v[0] + v[1] - v[2] + v[3]) * .25f;
  result.omega_rad_s = (-v[0] - v[1] + v[2] + v[3]) / (4 * k);
  if (!velocity_valid(&result))
    return false;
  *out = result;
  return true;
}
bool ChassisModel_BodyToMap(float vx, float vy, float yaw, float *x, float *y)
{
  float a, b;
  if (!isfinite(vx) || !isfinite(vy) || !isfinite(yaw) || x == NULL || y == NULL)
    return false;
  a = cosf(yaw) * vx - sinf(yaw) * vy;
  b = sinf(yaw) * vx + cosf(yaw) * vy;
  if (!isfinite(a) || !isfinite(b))
    return false;
  *x = a;
  *y = b;
  return true;
}
bool ChassisModel_MapToBody(float x, float y, float yaw, float *vx, float *vy)
{
  return ChassisModel_BodyToMap(x, y, -yaw, vx, vy);
}
bool ChassisModel_IntegrateMidpoint(ChassisModel_Pose_t *pose, const ChassisModel_Velocity_t *v,
                                    float dt)
{
  ChassisModel_Pose_t next;
  float x, y, mid;
  if (pose == NULL || !velocity_valid(v) || !isfinite(dt) || dt < 0 || !isfinite(pose->x_mm) ||
      !isfinite(pose->y_mm) || !isfinite(pose->yaw_rad))
    return false;
  mid = pose->yaw_rad + v->omega_rad_s * dt * .5f;
  if (!ChassisModel_BodyToMap(v->vx_mm_s, v->vy_mm_s, mid, &x, &y))
    return false;
  next.x_mm = pose->x_mm + x * dt;
  next.y_mm = pose->y_mm + y * dt;
  next.yaw_rad = pose->yaw_rad + v->omega_rad_s * dt;
  if (!isfinite(next.x_mm) || !isfinite(next.y_mm) || !isfinite(next.yaw_rad))
    return false;
  *pose = next;
  return true;
}
