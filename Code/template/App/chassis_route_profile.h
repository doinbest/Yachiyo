/** @file chassis_route_profile.h
 * @brief Pure map route phases shared by firmware and the offline planner.
 */
#ifndef CHASSIS_ROUTE_PROFILE_H
#define CHASSIS_ROUTE_PROFILE_H

#include <stdbool.h>
#include "radar_map.h"

#define CHASSIS_ROUTE_TARGET_COUNT 15U
#define CHASSIS_ROUTE_PROFILE_MAX_PHASES 6U
#define CHASSIS_TASK_PROFILE 1

typedef struct {
  bool turn;
  const char *phase, *direction;
  float x_mm, y_mm, yaw_deg, speed_mm_s;
} ChassisRoute_Phase_t;

typedef struct {
  ChassisRoute_Phase_t phases[CHASSIS_ROUTE_PROFILE_MAX_PHASES];
  unsigned count;
  float final_yaw_deg, nominal_time_s;
} ChassisRoute_Profile_t;

/** @brief Read origin at index0 or the fifteen targets at index1..15.
 * @return false for NULL output or an index beyond the final target.
 */
bool ChassisRouteProfile_FixedPoint(unsigned index, RadarPlan_Point_t *out);
/** @brief Build nominal absolute phases, in map mm/deg and mm/s, without HAL.
 * @param previous_yaw_deg Accepted final heading of the preceding route leg.
 * @param first_scan_leg Keep heading90 for the first task leg, including detours.
 * @return true when endpoints, heading and positive speed are valid.
 */
bool ChassisRouteProfile_Build(const RadarPlan_Point_t *from,
                              const RadarPlan_Point_t *to,
                              float previous_yaw_deg, float speed_mm_s,
                              bool first_scan_leg,
                              ChassisRoute_Profile_t *out);
/** @brief 机械臂实车任务路线；车体作业航向按原料0°、加工180°、暂存90°分别管理。
 * @note 加工区Y=300、直线进站末段125mm；暂存末段75mm，保留规划绕行路点。
 */
bool ChassisRouteProfile_BuildTask(const RadarPlan_Point_t *from,
                                 const RadarPlan_Point_t *to,
                                 float previous_yaw_deg, float speed_mm_s,
                                 bool first_scan_leg, ChassisRoute_Profile_t *out);

/** Select only the standard lower-start entrance. Other stations/detours retain
 * their existing task phases. false restores translation, stopped turn, straight. */
bool ChassisRouteProfile_BuildTaskEntry(const RadarPlan_Point_t *from,
                                      const RadarPlan_Point_t *to,
                                      float previous_yaw_deg,float speed_mm_s,
                                      bool first_scan_leg,bool continuous_entry,
                                      ChassisRoute_Profile_t *out);

/** Apply formal linear speed caps and per-phase acceleration; shared by preview.
 * Passing false preserves the generated speeds and selects default300 rates. */
void ChassisRouteProfile_ApplyTaskLinear(const RadarPlan_Point_t *from,
                                       const RadarPlan_Point_t *to,
                                       ChassisRoute_Profile_t *profile,bool linear,
                                       float *accels);

#endif
