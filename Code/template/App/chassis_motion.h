/** @file chassis_motion.h
 * @brief 主循环运动任务（控制目标1ms）；距离及转向完成核验实际反馈。
 */
#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H
#include "chassis_path.h"
#include "chassis_model.h"

typedef enum { CHASSIS_PROFILE_LINEAR, CHASSIS_PROFILE_COSINE } ChassisMotion_Profile_t;
/** Select the next distance task while idle. The accepted action snapshots it. */
bool ChassisMotion_ProfileSet(ChassisMotion_Profile_t profile);
ChassisMotion_Profile_t ChassisMotion_ProfileGet(void);
static inline const char *ChassisMotion_ProfileName(ChassisMotion_Profile_t profile)
{ return profile==CHASSIS_PROFILE_COSINE?"cosine":"linear"; }

/** Continuous path retains its final target and only stops at that target.
 * resume_mm is the saved path progress, zero for a new leg. */
bool ChassisMotion_PathStart(const ChassisPath *path,float speed,float turn_deg_s,
                             uint32_t timeout,float resume_mm);
/* speed is already effective; only path-point caps and acceleration scale here. */
bool ChassisMotion_PathStartScaled(const ChassisPath *path,float speed,float turn_deg_s,
                                   uint32_t timeout,float resume_mm,float scale);
float ChassisMotion_PathProgress(void);

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
  bool distance_mode, turn_only, stop_confirmed, indefinite_velocity;
  float target_x_mm, target_y_mm, target_map_yaw_deg;
  float error_x_mm, error_y_mm, error_heading_deg;
  bool heading_valid, map_anchor_valid;
  float current_deg, unwrapped_yaw_deg, map_yaw_rad;
  uint32_t heading_sample_tick;
  uint32_t heading_revision; /* Map correction only, independent of origin generation. */
  ChassisMotion_Profile_t profile;
  float linear_accel_mm_s2; /* Per-action translation acceleration. */
  float linear_decel_mm_s2; /* Independent braking envelope and slowing slew. */
  uint16_t curve_period_ms; /* Translation reference update; control remains 1ms. */
  uint16_t cosine_k; /* Half-cosine transition duration = K * 2 ms. */
  float heading_kp, heading_kd; /* rad/s per deg, and per (deg/s). */
  float peak_mm_s; /* Fresh wheel-feedback planar speed, never commanded speed. */
  uint32_t stop_ms; /* Final stop request to fresh standstill confirmation. */
} ChassisMotion_Status_t;

/** @brief 启动时初始化RAM；不得用它终止运行中的电机。 */
void ChassisMotion_Init(void);
/** @brief 0目标、角速度模式、500ms过渡/1000ms保持/500ms减速。 */
void ChassisMotion_TargetDefaults(ChassisMotion_Target_t *target);
/** @brief 仅空闲且视觉/机械臂/校准/底盘无冲突时接受任务；不阻塞。
 * @param target 车体mm/s、rad/s；线速度模长<=5000、角速度<=100°/s（约1.745329 rad/s）。
 * @return true为RAM目标接受，非发送完成或到位；失败原因见状态。
 * @pre 主循环调用；heading模式要求本次静止验证通过、角度新鲜且已AnchorSet。
 * HOLD_CURRENT只要求验证后的新鲜角度，不建立或修改地图锚点。
 */
bool ChassisMotion_Start(const ChassisMotion_Target_t *target);
/** @brief Indefinite body velocity until explicit stop; immediate target, drive grade20.
 * Linear speed <=5000 mm/s; legacy debug yaw <=360 deg/s (2*PI rad/s).
 * Same admission gates as timed tasks, no duration surrogate.
 */
bool ChassisMotion_VelocityStart(float vx_mm_s, float vy_mm_s, float omega_rad_s);
/** @brief Replace only a running indefinite target; preserve action/start tick. */
bool ChassisMotion_VelocityUpdate(float vx_mm_s, float vy_mm_s, float omega_rad_s);
/** @brief Legacy module-yaw hold: capture qualified fresh yaw, no map origin.
 * forward=-100..100 mm/s, duration=1000..10000 ms, immediate target/drive grade20.
 */
bool ChassisMotion_HoldStart(float forward_mm_s, uint32_t duration_ms);
typedef struct
{
  bool active;
  const char *reason;
  float target_deg, current_deg, output_rad_s;
} ChassisMotion_HoldStatus_t;
void ChassisMotion_HoldStatusGet(ChassisMotion_HoldStatus_t *status);
/** @brief Feedback-position task in map mm/deg; main-loop, qualified fresh IMU,
 * valid origin and four-wheel feedback required. A retired producer's fully
 * committed zero may transfer into map motion (including turn/path) without
 * FE or clearing the explicit-stop latch. Active grab control, queued motion,
 * stale/unhealthy wheels and stop/fault work still block the handoff.
 * True means accepted only.
 */
bool ChassisMotion_MoveTo(float x_mm, float y_mm, float map_yaw_deg,
                          float max_speed_mm_s, uint32_t timeout_ms);
typedef struct {
  float linear_accel_mm_s2;
  float linear_decel_mm_s2;
  uint16_t cosine_k;
  uint16_t curve_period_ms;
  float heading_kp, heading_kd;
} ChassisMotion_TestParameters_t;
void ChassisMotion_TestParametersDefaults(ChassisMotion_TestParameters_t *parameters);
/** @brief Per-action independent-test tuning; defaults do not persist to later tasks. */
bool ChassisMotion_MoveToTuned(float x_mm, float y_mm, float map_yaw_deg, float max_speed_mm_s,
                              const ChassisMotion_TestParameters_t *parameters, uint32_t timeout_ms);
/** Formal route only: normal position arrivals use two fresh stop groups.
 * Explicit stop monitoring, cancellation and faults retain their own rules. */
void ChassisMotion_UseRouteArrival(void);
/** Arm the standard north-to-west connection before its current endpoint.
 * The linear task keeps yaw, action and velocity; total timeout is frozen here. */
bool ChassisMotion_PrepareCorner(float next_x_mm,float next_y_mm,float speed_mm_s,
                                 float accel_mm_s2,uint32_t total_timeout_ms);
/** Sticky until the next motion starts, including across an explicit stop. */
bool ChassisMotion_CornerPassed(void);
/** @brief Legacy symmetric test override, 10..5000 mm/s^2; next MoveTo uses default.
 * Applies to linear translation; cosine retains its own transition model.
 */
bool ChassisMotion_MoveToAccel(float x_mm, float y_mm, float map_yaw_deg,
                              float max_speed_mm_s, float accel_mm_s2, uint32_t timeout_ms);
/** @brief 原地转至绝对地图航向，只输出角速度，不修正XY漂移。
 * @param map_yaw_deg 地图角度，-180..180 deg；+X为0，逆时针为正。
 * @param timeout_ms 同一动作及角度修正的总时限，20..400000 ms。
 * @return true仅表示受理；完成需要IMU到角及新鲜四轮停车反馈。
 * @pre 有效地图原点、已验证IMU、四轮新鲜静止反馈；抓取任务空闲。
 */
bool ChassisMotion_TurnTo(float map_yaw_deg, uint32_t timeout_ms);
bool ChassisMotion_TurnToSpeed(float map_yaw_deg,float max_deg_s,uint32_t timeout_ms);
/** @brief Refresh heading before localization sampling, without sending targets. */
void ChassisMotion_HeadingProcess(void);
/** @brief 半余弦减速后请求停车；stop_ms=0立即取消并请求异步停车。 */
bool ChassisMotion_Stop(uint32_t stop_ms);
/** @brief HWT101/校准处理之后、Mecanum_Process之前循环调用。
 * 规划曲线超过100ms调度间隔直接停车报错，不补跑积压周期。
 * 兼容持续速度/hold无位移积分，短调度间隔恢复后刷新原目标；hold仍检查IMU新鲜度与时长。
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
/** @brief 归零/重校准或角度流重启时撤销锚点。
 * 短暂缺数据只撤销当前航向有效性，保留映射；恢复须重新取得有效反馈。 */
void ChassisMotion_AnchorInvalidate(void);
/** Internal half of ChassisHeading_Apply; preserves raw IMU unwrap. */
bool ChassisMotion_AnchorCorrect(float map_yaw_rad);
/** Station/visual precision: bypasses translation heading deadband;
 * retains requested tolerance and fresh standstill confirmation. */
bool ChassisMotion_TurnToPrecision(float yaw,float max_deg_s,float tolerance_deg,uint32_t timeout);
#endif
