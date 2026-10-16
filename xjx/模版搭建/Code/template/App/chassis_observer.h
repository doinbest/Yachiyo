#ifndef CHASSIS_OBSERVER_H
#define CHASSIS_OBSERVER_H
#include "chassis_model.h"
#define CHASSIS_OBSERVER_YAW_HISTORY 32U /* 20 ms localization cadence, covers 600 ms. */
#define CHASSIS_OBSERVER_POSITION_HISTORY 8U
typedef struct
{
  struct { uint32_t time_ms; float rad; } yaw_history[CHASSIS_OBSERVER_YAW_HISTORY];
  unsigned yaw_count, yaw_next;
  struct { int64_t position; uint32_t time_ms; }
      position_history[4][CHASSIS_OBSERVER_POSITION_HISTORY];
  unsigned position_count[4], position_next[4];
} ChassisObserver_History_t; /* Private producer storage; never copied into telemetry/status. */
typedef struct
{
  int64_t position[4]; /* logical forward sign, protocol units */
  uint32_t time_ms[4], now_ms, units_per_rev;
  bool valid[4], yaw_valid;
  float yaw_rad;
  uint32_t yaw_ms;
  bool yaw_time_valid; /* Legacy host callers may use now_ms when not provided. */
  uint32_t max_span_ms; /* Zero keeps the ordinary 250 ms limit; tracking accepts its 600 ms read window. */
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
  double aligned_position[4];
  uint32_t feedback_ms; /* Common position/yaw time, not foreground consumption time. */
  bool aligned_baseline;
} ChassisObserver_t;
void ChassisObserver_Init(ChassisObserver_t *o, float x, float y, float yaw);
void ChassisObserver_Command(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                             const float rpm[4], uint32_t now, uint32_t applied_ms, bool valid);
void ChassisObserver_Feedback(ChassisObserver_t *o, const ChassisModel_Geometry_t *g,
                              const ChassisObserver_Feedback_t *f, ChassisObserver_History_t *history);
/** Drop interpolation baselines without changing the retained pose or command observer. */
void ChassisObserver_FeedbackReset(ChassisObserver_t *o);
#endif
