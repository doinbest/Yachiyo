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
  float base_speed_mm_s, motion_scale;
  float phase_target_x_mm, phase_target_y_mm, phase_target_yaw_deg;
} ChassisRoute_Status_t;
/** @brief Initialize RAM before commands are accepted; never stops motors. */
void ChassisRoute_Init(void);
/** QR route from the explicit map origin: upper (2250,2250,-90), or lower
 * (2250,150,180) diagonal -> clockwise90 -> forward; both turn180 at the
 * scan point and confirm stopping before reporting route completion. */
bool ChassisRoute_QrTestStart(float speed,float scan_y);
/** @brief Includes manual waiting and stopping; excludes terminal results. */
bool ChassisRoute_IsBusy(void);
/** @brief Internal admission token for the route's own distance submission. */
bool ChassisRoute_IsSubmitting(void);
bool ChassisRoute_Cancel(void);
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
typedef struct { bool scan_qr; float qr_scan_y_mm, turn_speed_deg_s; RadarStart_Id_t start_id; } ChassisRoute_TaskOptions_t;
bool ChassisRoute_TaskStartEx(const RadarPlan_t *plan,float speed,const ChassisRoute_TaskOptions_t *options);
bool ChassisRoute_TaskStartScaled(const RadarPlan_t *plan,float base_speed,
                                 const ChassisRoute_TaskOptions_t *options,float scale);
/* Optional smooth entrance affects only standard lower linear QR geometry. */
bool ChassisRoute_TaskStartConfigured(const RadarPlan_t *plan,float base_speed,
  const ChassisRoute_TaskOptions_t *options,float scale,bool smooth);
bool ChassisRoute_TaskStart(const RadarPlan_t *plan,float speed);
/** Pause only owned active/reserved work or an existing valid pause; terminal
 * status from an earlier match never establishes pause ownership. */
bool ChassisRoute_TaskPause(void);
/** Mission-only adoption of a temporary feedback loss, including the current
 * action already stopping/failed with feedback_invalid. Keeps the original
 * reference, phase/target and path progress, then proves a fresh monitored stop.
 * Caller qualifies the retained map/IMU recovery and healthy motor bus. Travel
 * requires the same nonzero action; mission-owned work may use a local action.
 * False retains targets/phase and exposes the rejection through route reason. */
bool ChassisRoute_TaskPauseFeedbackLoss(void);
/** Suspend pause-monitor producers while the bus recovery owner verifies motors. */
void ChassisRoute_TaskRecoveryHold(bool hold);
bool ChassisRoute_TaskContinue(void);
/** Prepare/check resume feedback without releasing paused route or issuing motion. */
bool ChassisRoute_TaskResumeReady(void);
/** @brief 工位任务接管局部移动与取放；保留路线与下一工位，不要求重设原点。 */
bool ChassisRoute_WorkBegin(void);
/** @brief 局部动作反馈完成后，按当前航向继续下一目标。
 * 加工离站核验(X观察中心,425)；其他工位保留原中心交接。 */
bool ChassisRoute_WorkResume(void);
/** @brief Main-loop only, after motion processing. */
void ChassisRoute_Process(void);
void ChassisRoute_StatusGet(ChassisRoute_Status_t *status);
/** @brief Console command handler; returns false for unrelated commands. */
bool ChassisRoute_Command(unsigned count, char *tokens[]);
#endif
