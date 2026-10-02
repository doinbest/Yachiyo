#ifndef CHASSIS_OBSERVER_H
#define CHASSIS_OBSERVER_H
#include "chassis_model.h"
typedef struct
{
  int64_t position[4]; /* logical forward sign, protocol units */
  uint32_t time_ms[4], now_ms, units_per_rev;
  bool valid[4], yaw_valid;
  float yaw_rad;
} ChassisObserver_Feedback_t;
typedef struct
{
  ChassisModel_Pose_t command, feedback;
  bool command_valid, feedback_valid, command_baseline, feedback_baseline;
  uint32_t command_ms, previous_ms[4], feedback_span_ms;
  int64_t previous_position[4];
  float previous_yaw;
  float previous_rpm[4];
  uint32_t applied_ms;
} ChassisObserver_t;
void ChassisObserver_Init(ChassisObserver_t *o, float x, float y, float yaw);
void ChassisObserver_Command(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                             const float rpm[4], uint32_t now, uint32_t applied_ms, bool valid);
void ChassisObserver_Feedback(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                              const ChassisObserver_Feedback_t *f);
#endif
