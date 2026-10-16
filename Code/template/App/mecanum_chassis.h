/**
 * @file    mecanum_chassis.h
 * @brief   四轮麦克纳姆底盘运动学和步进电机同步控制。
 */
#ifndef MECANUM_CHASSIS_H
#define MECANUM_CHASSIS_H

#include "main.h"
#include <stdbool.h>

#define MECANUM_WHEEL_COUNT 4U

#define MECANUM_VELOCITY_REFRESH_MS 100U
typedef enum
{
  MECANUM_ACK_UNKNOWN = 0,
  MECANUM_ACK_NONE,
  MECANUM_ACK_RECEIVE
} Mecanum_AckProfile_t;
typedef enum
{
  MECANUM_STAGE_IDLE = 0,
  MECANUM_STAGE_WHEELS,
  MECANUM_STAGE_SYNC,
  MECANUM_STAGE_SENT,
  MECANUM_STAGE_STOPPING,
  MECANUM_STAGE_STOPPED,
  MECANUM_STAGE_FAULT
} Mecanum_Stage_t;
typedef enum
{
  MECANUM_ERROR_NONE = 0,
  MECANUM_ERROR_PROFILE,
  MECANUM_ERROR_TX,
  MECANUM_ERROR_TIMEOUT,
  MECANUM_ERROR_ACK,
  MECANUM_ERROR_CANCELLED
} Mecanum_Error_t;
typedef struct
{
  uint32_t sequence;
  uint32_t motion_sequence, stop_sequence; /* Accepted motion and actual stop dispatch epochs. */
  Mecanum_Stage_t stage;
  Mecanum_Error_t error;
  Mecanum_AckProfile_t ack_profile;
  bool locked, stop_pending;
  int32_t sent_rpm[4];
  uint32_t sent_ms;
  uint32_t sent_sequence; /* Sequence of the committed batch, not the pending plan. */
  bool sent_velocity_valid;
  bool tx_complete, acknowledged;
  uint8_t acknowledged_wheels;
} Mecanum_Status_t;
typedef struct
{
  int32_t speed_rpm;    /* protocol motor sign; not corrected to chassis forward polarity */
  int64_t position_raw; /* signed magnitude 32-bit payload, no truncation */
  uint8_t state_flags;
  uint32_t speed_ms, position_ms, state_ms;
  uint32_t speed_tx_ms, position_tx_ms;
  uint32_t speed_target_ms, speed_target_sequence;
  int32_t speed_target_rpm; /* Logical chassis-forward target active at speed query TX. */
  bool speed_target_valid;
  bool speed_valid, position_valid, state_valid;
} Mecanum_Feedback_t;
/** Accept a latest target snapshot; main loop sends asynchronously. acc is drive grade. */
bool Mecanum_Velocity_Request(float forward_mm_s, float left_mm_s, float yaw_rad_s, uint8_t acc);
/** @brief 设置追踪模式与用户显式批次限频。
 * @param enabled 是否追踪；interval_ms为0..200ms，0按应答推进。
 * @retval 无
 * @note 保留Receive应答、四轮原子批次及批间公平调度。
 */
void Mecanum_TrackingTiming(bool enabled, uint32_t interval_ms);
void Mecanum_Process(void);
void Mecanum_StatusGet(Mecanum_Status_t *status);
typedef struct
{
  uint32_t batches, intervals, last_ms, interval_max_ms, duration_max_ms;
  uint32_t interval_bins[201]; /* Last bin includes intervals >=200ms. */
} Mecanum_Timing_t;
/** @brief Copy cumulative actual batch timing in MCU milliseconds, without traffic. */
void Mecanum_TimingGet(Mecanum_Timing_t *out);
/** Local profile only; caller must first confirm all four physical drive ACK settings. */
bool Mecanum_AckProfile_Set(Mecanum_AckProfile_t profile);
/** Caller confirms physical reset of drivers; Stop alone is insufficient for recovery. */
bool Mecanum_RecoveryAfterReset(void);
/** @brief Unlock after explicit bus recovery verifies zero cache writes,
 * addressed wheel sync and stopped feedback. Never starts motion or resets pose. */
bool Mecanum_RecoveryAfterBusCheck(void);
/** Poll speed 1..4, position 1..4, then state 1..4 to bound complete group skew. */
/** @brief Main-loop temporary all-wheel stop feedback, independent of user selection.
 * Release retains the most recent user/task enable and selection policy. */
void Mecanum_Feedback_StopMonitor(bool active);
/** Normal route arrival: refresh drive states once, then four speeds/positions.
 * Does not alter user selection; independent stop monitoring takes precedence. */
void Mecanum_Feedback_Arrival(bool active);
void Mecanum_Feedback_Enable(bool enabled);
/** @brief Select the next poll's motor address 1..4, or 0 for all.
 * Disables new polling until Enable(true). An outstanding read/reply completes
 * normally with its original identity; selection does not cancel it or move motors.
 * @return true for a valid selection, including while a read/reply is pending.
 */
bool Mecanum_Feedback_Select(uint8_t address);
bool Mecanum_FeedbackGet(unsigned wheel, Mecanum_Feedback_t *feedback);
/* 整车速度模式的重复发送周期，防止驱动器速度命令超时。 */
/* HWT101原生标定/验证忙时，运动启动接口返回false；停车接口仍可调用。 */

/**
 * @brief 麦克纳姆轮编号
 *
 * 轮号与电机 ID 的固定顺序为：左上、左下、右下、右上，对应
 * 数组下标 0、1、2、3 和默认电机地址 1、2、3、4。
 */
typedef enum
{
  MECANUM_WHEEL_FRONT_LEFT = 0,
  MECANUM_WHEEL_REAR_LEFT,
  MECANUM_WHEEL_REAR_RIGHT,
  MECANUM_WHEEL_FRONT_RIGHT
} MecanumWheel_t;

/**********************************************************
*** 四电机同步位置控制
**********************************************************/
/**
 * @brief    控制四轮按给定车体相对位移同步运动
 * @param    forward_mm ：车体前后位移，向前为正，单位 mm
 * @param    left_mm    ：车体左右位移，向左为正，单位 mm
 * @param    yaw_rad    ：车体旋转角度，逆时针为正，单位 rad
 * @retval   true  ：请求已接受；完成状态请读取 Mecanum_StatusGet
 * @retval   false ：标定忙、运动学参数无效或电机命令发送失败
 * @note     每台电机位置命令的 snF=1，最后用广播地址0统一触发
 */
bool Mecanum_Move_Control(float forward_mm, float left_mm, float yaw_rad);

/**
 * @brief    低速控制底盘前进或后退，用于检查四轮极性。
 * @param    forward_mm ：前进为正、后退为负，单位mm
 * @retval   true请求已接受；false参数或UART5发送失败
 * @note     仅允许前后运动，速度和最大距离由chassis_config.h限制
 */
bool Mecanum_Polarity_Move(float forward_mm);

/**
 * @brief    让指定车轮按低速连续转动。
 * @param    wheel ：车轮枚举
 * @param    rpm   ：正数表示该轮向车体前方滚动，负数表示反向
 * @retval   true请求已接受；false参数或UART5发送失败
 */
bool Mecanum_Wheel_Test(MecanumWheel_t wheel, int16_t rpm);

/**
 * @brief    依次立即停止底盘四个车轮。
 * @retval   true停车请求已接受；四条TX完成后才解除运动记录
 * @note     发送失败保留忙状态；发送成功不代表已取得电机实际静止反馈。
 */
bool Mecanum_Test_Stop(void);
/** @brief Read-only gate: the wheel batch is complete and no feedback is on wire.
 * @note Unsent queries may be cancelled. Planned stops use RequestPlannedStop
 * to drain the current reply; emergency stops do not wait on this gate.
 */
bool Mecanum_CanStopCleanly(void);
/** @brief 零速批次确认后，受理普通停车并等待当前反馈应答结束。
 * @return 已受理；实际停止与静止确认仍由状态/轮反馈判定。
 * @note 暂不排下一条反馈，保持当前应答完整；紧急停车仍用Test_Stop。 */
bool Mecanum_RequestPlannedStop(void);

/** @brief 是否存在待发送批次或尚未显式停车的运动命令。
 * @return true时不得开始IMU标定或静止验证。
 * @note 只记录本模块发出的运动命令，不检测编码器或实际静止。
 * 位置、极性和单轮运动也会保持忙；即使估算时间已到，仍须调用
 * Mecanum_Test_Stop()且四条停止命令均发送成功，才能解除记录。
 */
bool Mecanum_IsBusy(void);
/** Read-only velocity handoff gate: committed four-wheel zero, no queued motion
 * or stop/fault work. Does not clear IsBusy's explicit-stop latch or claim
 * measured standstill; the receiving task still validates fresh feedback. */
bool Mecanum_VelocityHandoffReady(void);

#endif /* MECANUM_CHASSIS_H */
