/** @file chassis_localization.h
 * @brief Main-loop localization shared by control and optional telemetry.
 */
#ifndef CHASSIS_LOCALIZATION_H
#define CHASSIS_LOCALIZATION_H
#include "chassis_observer.h"
#include "mecanum_chassis.h"
typedef struct
{
  ChassisObserver_t observer;
  Mecanum_Feedback_t wheels[4];
  ChassisModel_Pose_t origin;
  bool origin_valid;
  uint32_t units_per_rev, generation, feedback_sequence, feedback_tick;
  bool feedback_valid, speed_valid;
} ChassisLocalization_Status_t;
/** @brief Initialize RAM before accepting commands; does not move hardware. */
void ChassisLocalization_Init(void);
/** @brief Sample every main-loop pass; observer integration runs at most every 20ms. */
void ChassisLocalization_Process(void);
/** @brief Copy the last main-loop snapshot. Wheel timestamps remain receive times. */
void ChassisLocalization_Get(ChassisLocalization_Status_t *status);
/** @brief Set map origin while idle, requiring a qualified fresh heading anchor. */
bool ChassisLocalization_Origin(float x_mm, float y_mm, float yaw_rad);
/** @brief Confirm protocol units (65536), or revoke with zero, while idle. */
bool ChassisLocalization_ConfirmPositionUnits(uint32_t units_per_rev);
#endif
