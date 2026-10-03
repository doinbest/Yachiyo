/** @file chassis_route_profile.h
 * @brief Pure map route phases shared by firmware and the offline planner.
 */
#ifndef CHASSIS_ROUTE_PROFILE_H
#define CHASSIS_ROUTE_PROFILE_H

#include <stdbool.h>
#include "radar_map.h"

#define CHASSIS_ROUTE_TARGET_COUNT 15U
#define CHASSIS_ROUTE_PROFILE_MAX_PHASES 6U

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

#endif
