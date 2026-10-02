/** @file GrabTask.h
 * @brief 蓝色物料仅协同对准、静止红色物料单件抓取；主循环独占底盘/Base/X/Z，抓取后转向放置。
 */
#ifndef GRAB_TASK_H
#define GRAB_TASK_H
#include "mechanical_arm.h"
#include "Camera.h"
#include <stdbool.h>

/* Shared by camera requests and the read-only firmware capability report. */
#define GRAB_ALIGN_COLOR CAMERA_COLOR_BLUE
#define GRAB_PICK_COLOR CAMERA_COLOR_RED

typedef enum
{
  GRAB_IDLE, GRAB_PREPARE, GRAB_ACQUIRE, GRAB_ALIGN, GRAB_SETTLE,
  GRAB_DESCEND, GRAB_CLOSE, GRAB_LIFT, GRAB_TURN, GRAB_PLACE, GRAB_RELEASE, GRAB_RETRACT, GRAB_HOLD, GRAB_STOPPING, GRAB_ERROR, GRAB_HOMING, GRAB_VISION_PAUSE, GRAB_VISION_GRACE
} GrabTask_State_t;
typedef struct
{
  GrabTask_State_t state;
  const char *state_name, *mode, *reason, *missing;
  const char *reference_cause; /**< Latest cause that invalidated or verified pickup references. */
  bool busy, stop_requested, stop_confirmed;
  float x_mm, x_target_mm, z_mm, z_target_mm, forward_mm_s, left_mm_s;
  int16_t dx, dy;
  uint32_t rx_seq; /**< MCU receive sequence, not camera capture sequence. */
  const char *vision_state;
  uint32_t age_ms, elapsed_ms;
  uint32_t recovery_used; /**< Successful automatic recoveries in this task, 0..2. */
  uint32_t recovery_count; /**< Consecutive new valid observations in the recovery window, 0..3. */
  int32_t recovery_left_ms; /**< -1 before standstill/outside recovery; MCU time only. */
  uint32_t loss_ms; /**< Active interruption or last completed one, measured from last good RX; ms. */
  uint32_t loss_max_ms; /**< Longest interruption in this task, including recovery confirmation; ms. */
  uint32_t grace_count; /**< New valid observations during short-loss tolerance, 0..3. */
} GrabTask_Status_t;

/** @brief 初始化RAM标定缺项；仅启动时调用，不用于停车。 */
void GrabTask_Init(void);
/** @brief Arm one Z/X collision then Base near homing sequence after bus init or explicit grab rehome.
 * Completion requires 0x3B flags and feedback: X/Z zero + reached, Base stable + enabled.
 * Base near-home does not imply numerical zero. Success arms IMU boot verification once.
 */
void GrabTask_BootHomeStart(void);
/** @brief 持续主循环调用；蓝色align短漏检默认容忍250 ms，停稳后恢复等待1500 ms。
 * 容忍期间只按上次速度减速，不沿旧误差加速或判定对准；仅align停稳后重发一次B2。
 */
void GrabTask_Process(void);
/** @brief fixed/align/pick；返回受理，不等于到位或抓持确认。必须先排除其他任务。 */
bool GrabTask_Start(const char *mode);
/** @brief 只读检查所选模式标定完整性和数值/机械边界，不访问硬件。 */
bool GrabTask_ConfigReady(const char *mode);
/** @brief 已受理的参考变化通知：X/Z清除绝对边界/位置，BASE清除准心/J；
 * ALL仅清除X/Z，all home调用者另通知BASE；不自动重标定或改变硬件。
 */
void GrabTask_ReferenceInvalidate(MechanicalArm_AxisTypeDef axis);
/** @brief HOLD仍拥有机构；等待用户检查，不能据此自动计数。 */
bool GrabTask_IsBusy(void);
/** @brief 取消自动推进并请求停车；重复请求保留停稳证据，保持夹爪输出与Z使能，检查状态确认停车。 */
void GrabTask_Stop(void);
/** @brief 消费本任务发起的机械臂事件；非本任务返回0。 */
uint8_t GrabTask_MotorEventHandle(const MechanicalArm_EventTypeDef *event);
/** @brief 主循环只读状态快照；距离mm、速度mm/s、年龄/耗时ms。 */
void GrabTask_StatusGet(GrabTask_Status_t *status);
/** @brief 空格ASCII grab命令，最多6字段/63字节；返回是否属于本模块。 */
bool GrabTask_Command(unsigned count, char *tokens[]);
#endif
