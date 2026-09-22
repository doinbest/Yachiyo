/** @file chassis_route.h
 * @brief Sixteen fixed map stops, each advanced by an explicit operator request.
 */
#ifndef CHASSIS_ROUTE_H
#define CHASSIS_ROUTE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
  const char *state, *reason;
  uint32_t segment, action_id;
  float target_x_mm, target_y_mm, target_yaw_deg;
  float error_x_mm, error_y_mm, error_heading_deg;
  bool feedback_valid, stop_confirmed;
} ChassisRoute_Status_t;
/** @brief Initialize RAM before commands are accepted; never stops motors. */
void ChassisRoute_Init(void);
/** @brief Includes manual waiting and stopping; excludes terminal results. */
bool ChassisRoute_IsBusy(void);
/** @brief Internal admission token for the route's own distance submission. */
bool ChassisRoute_IsSubmitting(void);
/** @return Request accepted, not physical movement or arrival. */
bool ChassisRoute_Start(void);
bool ChassisRoute_Next(void);
bool ChassisRoute_Cancel(void);
/** @brief Main-loop only, after motion processing. */
void ChassisRoute_Process(void);
void ChassisRoute_StatusGet(ChassisRoute_Status_t *status);
/** @brief Console command handler; returns false for unrelated commands. */
bool ChassisRoute_Command(unsigned count, char *tokens[]);
#endif
