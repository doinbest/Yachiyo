/** @file chassis_route.h
 * @brief Fifteen map targets with internally sequenced travel and docking phases.
 */
#ifndef CHASSIS_ROUTE_H
#define CHASSIS_ROUTE_H
#include <stdbool.h>
#include <stdint.h>
#include "radar_map.h"
typedef struct {
  const char *state, *reason;
  const char *phase, *direction;
  uint32_t segment, action_id;
  float target_x_mm, target_y_mm, target_yaw_deg;
  float error_x_mm, error_y_mm, error_heading_deg;
  bool feedback_valid, stop_confirmed;
  bool planned;
  uint32_t total, station, visit;
  float speed_mm_s, phase_speed_mm_s;
  float phase_target_x_mm, phase_target_y_mm, phase_target_yaw_deg;
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
/** @brief 扫码visit1/原料visit2到点姿态及停车确认后交接；speed为mm/s。 */
bool ChassisRoute_StationStart(float speed);
/** @brief Copy a completed radar plan and execute it explicitly; station_mode
 * pauses at QR visit1 and RAW visit2 for the existing first-pick test. */
bool ChassisRoute_PlanStart(const RadarPlan_t *plan, float speed, bool station_mode);
/** @brief 仅扫码station5/visit1允许续跑；重新检查到点与反馈资格。 */
bool ChassisRoute_StationResume(void);
/** @brief 原料station4/visit2结束路线所有权，不提交下一个目标。 */
bool ChassisRoute_StationFinish(void);
/** @brief 组合路线是否仍保留站点任务所有权。 */
bool ChassisRoute_StationReserved(void);
#define CHASSIS_TASK_ROUTE 1
/** @brief 新后向机构任务：NULL为固定路线，否则使用已完成的右下雷达计划；不扫码。 */
bool ChassisRoute_TaskStart(const RadarPlan_t *plan,float speed);
/** @brief 工位任务接管局部移动与取放；保留路线与下一工位，不要求重设原点。 */
bool ChassisRoute_WorkBegin(void);
/** @brief 局部动作反馈完成后，按当前航向继续下一目标。 */
bool ChassisRoute_WorkResume(void);
/** @brief Main-loop only, after motion processing. */
void ChassisRoute_Process(void);
void ChassisRoute_StatusGet(ChassisRoute_Status_t *status);
/** @brief Console command handler; returns false for unrelated commands. */
bool ChassisRoute_Command(unsigned count, char *tokens[]);
#endif
