/** @file chassis_motion.h
 * @brief 主循环20ms有限时长低速任务。完成仅表示时间结束且停车发送完成。
 */
#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H
#include "chassis_model.h"

typedef enum
{
  CHASSIS_MOTION_IDLE,
  CHASSIS_MOTION_RUNNING,
  CHASSIS_MOTION_STOPPING,
  CHASSIS_MOTION_DONE,
  CHASSIS_MOTION_ERROR
} ChassisMotion_State_t;
typedef enum
{
  CHASSIS_MOTION_ANGULAR_VELOCITY,
  CHASSIS_MOTION_HEADING,
  CHASSIS_MOTION_HOLD_CURRENT /* Capture qualified module yaw once at start. */
} ChassisMotion_Mode_t;
typedef struct
{
  float vx_mm_s, vy_mm_s, omega_rad_s;
  float heading_deg; /* 乘过CHASSIS_HEADING_TEST_YAW_SIGN的HWT模块角度，deg。 */
  ChassisMotion_Mode_t mode;
  uint32_t transition_ms, hold_ms, stop_ms;
} ChassisMotion_Target_t;
typedef struct
{
  ChassisMotion_State_t state;
  const char *reason;
  uint32_t action_id, start_tick, sample_tick, control_dt_ms, elapsed_ms;
  ChassisMotion_Target_t requested;
  ChassisModel_Velocity_t body_target;
  float rpm_unquantized[4];
  int16_t rpm_command[4];
  float rpm_scale;
  bool distance_mode, stop_confirmed;
  float target_x_mm, target_y_mm, target_map_yaw_deg;
  float error_x_mm, error_y_mm, error_heading_deg;
  bool heading_valid, map_anchor_valid;
  float current_deg, unwrapped_yaw_deg, map_yaw_rad;
  uint32_t heading_sample_tick;
} ChassisMotion_Status_t;

/** @brief 启动时初始化RAM；不得用它终止运行中的电机。 */
void ChassisMotion_Init(void);
/** @brief 0目标、角速度模式、500ms过渡/1000ms保持/500ms减速。 */
void ChassisMotion_TargetDefaults(ChassisMotion_Target_t *target);
/** @brief 仅空闲且视觉/机械臂/校准/底盘无冲突时接受任务；不阻塞。
 * @param target 车体mm/s、rad/s；线速度模长<=100、角速度<=0.15。
 * @return true为RAM目标接受，非发送完成或到位；失败原因见状态。
 * @pre 主循环调用；heading模式要求本次静止验证通过、角度新鲜且已AnchorSet。
 * HOLD_CURRENT只要求验证后的新鲜角度，不建立或修改地图锚点。
 */
bool ChassisMotion_Start(const ChassisMotion_Target_t *target);
/** @brief Feedback-position task in map mm/deg; main-loop, qualified fresh IMU,
 * valid origin and four-wheel feedback required. True means accepted only.
 */
bool ChassisMotion_MoveTo(float x_mm, float y_mm, float map_yaw_deg,
                          float max_speed_mm_s, uint32_t timeout_ms);
/** @brief Refresh heading before localization sampling, without sending targets. */
void ChassisMotion_HeadingProcess(void);
/** @brief 半余弦减速后请求停车；stop_ms=0立即取消并请求异步停车。 */
bool ChassisMotion_Stop(uint32_t stop_ms);
/** @brief HWT101/校准处理之后、Mecanum_Velocity_Process之前循环调用。
 * 超过100ms调度间隔直接停车报错，不补跑积压周期。
 */
void ChassisMotion_Process(void);
bool ChassisMotion_IsBusy(void);
void ChassisMotion_StatusGet(ChassisMotion_Status_t *status);
typedef struct
{
  uint32_t id, token, requested_ms;
  bool tx_complete, wheels_stopped;
  const char *reason;
} ChassisStop_Status_t;
/** @brief Main-loop immediate stop; nonzero token retries reuse the current request
 * only while no later motion was accepted. Zero always requests another stop.
 * @return true means accepted, not physical standstill. */
bool ChassisMotion_StopRequest(uint32_t client_token);
/** @brief Fresh stop evidence independent of map/IMU and target arrival.
 * Uses Emm 65536 units/rev; needs fresh post-TX groups. Confirmed result is retained
 * after temporary polling ends, until a new motion/request invalidates it. */
/** @brief Main-loop: true while waiting for stop TX / four-wheel evidence. */
bool ChassisMotion_StopPending(void);
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *status);
/** @brief 静止且具有新鲜已验证角度时，将当前模块航向对应到map_yaw_rad。
 * @return 成功重建展开角和地图锚点；不修改模块归零或Flash。
 */
bool ChassisMotion_AnchorSet(float map_yaw_rad);
/** @brief 归零/重校准/重启通知；角度失效也会自动撤销锚点。 */
void ChassisMotion_AnchorInvalidate(void);
#endif
