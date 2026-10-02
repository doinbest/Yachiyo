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
  bool speed_valid, position_valid, state_valid;
} Mecanum_Feedback_t;
/** Accept a latest target snapshot; main loop sends asynchronously. acc is drive grade. */
bool Mecanum_Velocity_Request(float forward_mm_s, float left_mm_s, float yaw_rad_s, uint8_t acc);
void Mecanum_Process(void);
void Mecanum_StatusGet(Mecanum_Status_t *status);
/** Local profile only; caller must first confirm all four physical drive ACK settings. */
bool Mecanum_AckProfile_Set(Mecanum_AckProfile_t profile);
/** Caller confirms physical reset of drivers; Stop alone is insufficient for recovery. */
bool Mecanum_RecoveryAfterReset(void);
/** @brief Clear chassis transport fault after verified bus recovery only when
 * no untriggered synchronous cache remains. Never starts motion. */
bool Mecanum_RecoveryAfterBusCheck(void);
/** Poll speed 1..4, position 1..4, then state 1..4 to bound complete group skew. */
/** @brief Main-loop temporary all-wheel stop feedback, independent of user selection.
 * Release retains the most recent user/task enable and selection policy. */
void Mecanum_Feedback_StopMonitor(bool active);
void Mecanum_Feedback_Enable(bool enabled);
/** Select motor address 1..4, or 0 for all; disables polling until Enable(true). */
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
*** 麦克纳姆轮逆运动学解算
**********************************************************/
/**
 * @brief    将车体前向、横向和旋转速度分解为四个车轮速度
 * @param    forward_mm_s ：车体前向速度，向前为正，单位 mm/s
 * @param    left_mm_s    ：车体横向速度，向左为正，单位 mm/s
 * @param    yaw_rad_s    ：车体角速度，逆时针为正，单位 rad/s
 * @param    wheel_mm_s   ：四轮输出数组，顺序为左上、左下、右下、右上
 * @retval   true  ：输入有效，四轮速度解算完成
 * @retval   false ：输出指针为空，或旋转时尚未配置轴距/轮距
 */
bool Mecanum_Wheel_Speed_Calc(float forward_mm_s, float left_mm_s, float yaw_rad_s,
                              float wheel_mm_s[MECANUM_WHEEL_COUNT]);

/**********************************************************
*** 麦克纳姆轮正运动学解算
**********************************************************/
/**
 * @brief    根据四个车轮速度计算车体前向、横向和旋转速度
 * @param    wheel_mm_s   ：四轮速度数组，顺序为左上、左下、右下、右上
 * @param    forward_mm_s ：返回车体前向速度，单位 mm/s
 * @param    left_mm_s    ：返回车体向左速度，单位 mm/s
 * @param    yaw_rad_s    ：返回车体逆时针角速度，单位 rad/s
 * @retval   true  ：正运动学解算完成
 * @retval   false ：参数指针为空，或轴距/轮距未配置导致无法计算角速度
 */
bool Mecanum_Body_Speed_Calc(const float wheel_mm_s[MECANUM_WHEEL_COUNT], float *forward_mm_s,
                             float *left_mm_s, float *yaw_rad_s);

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
/** @brief Read-only main-loop gate for planned stops: no chassis or feedback
 * transaction remains queued, on wire, or awaiting event consumption.
 * @note Call immediately before Test_Stop. Emergency stops do not wait on this gate.
 */
bool Mecanum_CanStopCleanly(void);

/** @brief 是否存在未结束的速度/航向任务或尚未显式停车的运动命令。
 * @return true时不得开始IMU标定或静止验证。
 * @note 只记录本模块发出的运动命令，不检测编码器或实际静止。
 * 位置、极性和单轮运动也会保持忙；即使估算时间已到，仍须调用
 * Mecanum_Test_Stop()且四条停止命令均发送成功，才能解除记录。
 */
bool Mecanum_IsBusy(void);

/**********************************************************
*** 连续速度控制与航向角 PID
**********************************************************/
/**
 * @brief    按车体速度发送四轮速度命令。
 * @param    forward_mm_s ：车体前向速度，向前为正，单位 mm/s
 * @param    left_mm_s    ：车体横向速度，向左为正，单位 mm/s
 * @param    yaw_rad_s    ：车体角速度，逆时针为正，单位 rad/s
 * @retval   true  ：四轮目标快照已接受
 * @retval   false ：运动学参数无效，未发送命令
 * @note     仅保存下一组快照；主循环异步发送四轮缓存命令和同步帧。
 */
bool Mecanum_Velocity_Control(float forward_mm_s, float left_mm_s, float yaw_rad_s);

/**
 * @brief    启动整车持续速度控制。
 * @param    forward_mm_s ：车体前后速度，前进为正，单位mm/s
 * @param    left_mm_s    ：车体左右速度，向左为正，单位mm/s
 * @param    yaw_rad_s    ：车体旋转速度，逆时针为正，单位rad/s
 * @retval   true  ：首组速度请求已接受
 * @retval   false ：运动学参数无效或UART5发送失败
 * @note     启动后由Mecanum_Velocity_Process()周期刷新，直到显式停车
 */
bool Mecanum_Velocity_Start(float forward_mm_s, float left_mm_s, float yaw_rad_s);

/**
 * @brief    刷新整车持续速度控制。
 * @param    无
 * @retval   无
 * @note     应在主循环中调用；速度模式不会自动结束
 */
void Mecanum_Velocity_Process(void);

/**
 * @brief    取消整车持续速度控制。
 * @param    无
 * @retval   无
 * @note     仅清除周期刷新状态，实际停车由Mecanum_Test_Stop()完成
 */
void Mecanum_VelocityRefresh_Stop(void);

/**
 * @brief    离散位置式航向角 PID 状态。
 *
 * PID 输入为角度误差（deg），输出为车体角速度（rad/s）。
 * ki、kd 使用显式采样周期计算，便于在不同任务周期下重新整定。
 */
typedef struct
{
  float kp;
  float ki;
  float kd;
  float sample_time_s;
  float integral_limit;
  float output_limit;
  float target_yaw_deg;
  float integral;
  float previous_error_deg;
  float output_rad_s;
  bool initialized;
} Mecanum_HeadingPid_t;

typedef struct
{
  bool active;
  const char *reason;
  float target_deg, current_deg, output_rad_s;
} Mecanum_HeadingTestStatus_t;

/** @brief 独立模块航向测试；锁定当前航向，以指定前向速度运行指定时长。
 * @param forward_mm_s -100..100 mm/s，负值后退，0为原地保持航向。
 * @param duration_ms 1000..10000 ms，超时自动发送停车帧。
 * @return false表示参数、忙、模块角度无效或发送失败，具体原因见状态。
 * @pre 本次启动必须通过IMU静止验证；主循环持续调用Mecanum_Velocity_Process。
 * @note 使用模块最新角度，不进行STM32外部漂移扣除；原生标定/验证期间禁止启动。
 */
bool Mecanum_HeadingTest_Start(float forward_mm_s, uint32_t duration_ms);
/** @brief 查询当前/最后一次独立测试状态；只读，status须非空。 */
void Mecanum_HeadingTest_StatusGet(Mecanum_HeadingTestStatus_t *status);

/**
 * @brief    初始化航向角 PID。
 * @param    pid               ：PID 状态结构体
 * @param    kp/ki/kd          ：PID 参数，输入误差单位为度，输出单位为rad/s
 * @param    sample_time_s     ：固定调用周期，单位s，例如50ms填写0.05f
 * @param    integral_limit    ：积分项输入限幅，单位deg*s
 * @param    output_limit      ：输出角速度限幅，单位rad/s
 * @retval   无
 */
void Mecanum_HeadingPid_Init(Mecanum_HeadingPid_t *pid, float kp, float ki, float kd,
                             float sample_time_s, float integral_limit, float output_limit);

/**
 * @brief    设置目标航向并清除上一段运动的积分状态。
 * @param    pid       ：PID 状态结构体
 * @param    target_deg：目标航向角，单位deg
 * @retval   无
 */
void Mecanum_HeadingPid_Set_Target(Mecanum_HeadingPid_t *pid, float target_deg);

/**
 * @brief    根据当前Yaw计算一次离散位置式PID输出。
 * @param    pid             ：PID状态结构体
 * @param    current_yaw_deg ：IMU当前Yaw，单位deg
 * @retval   车体角速度修正量，单位rad/s
 * @note     目标值和当前值的误差会归一化到[-180,180]。
 */
float Mecanum_HeadingPid_Update(Mecanum_HeadingPid_t *pid, float current_yaw_deg);

/**
 * @brief    计算航向PID并发送一次带航向保持的车体速度命令。
 * @param    pid             ：PID状态结构体
 * @param    current_yaw_deg ：IMU当前Yaw，单位deg
 * @param    forward_mm_s    ：期望前向速度，单位mm/s
 * @param    left_mm_s       ：期望左向速度，单位mm/s
 * @retval   true  ：速度请求已接受
 * @retval   false ：参数或运动学配置无效
 * @note     应在固定周期的主循环/任务中调用，禁止在中断中调用。
 */
bool Mecanum_Heading_Hold_Step(Mecanum_HeadingPid_t *pid, float current_yaw_deg, float forward_mm_s,
                               float left_mm_s);

/**********************************************************
*** 距离与步进电机计数换算
**********************************************************/
/**
 * @brief    将车轮行驶距离换算成 Emm 位置控制命令脉冲数
 * @param    distance_mm ：车轮行驶距离，单位 mm，可为负数
 * @retval   Emm_V5_Pos_Control() 的 clk 参数绝对值
 * @note     方向不包含在返回值中，由调用函数根据距离正负单独设置 dir
 */
uint32_t Mecanum_Distance_To_Pulse(float distance_mm);

/**
 * @brief    将车轮行驶距离换算成理论编码器计数
 * @param    distance_mm ：车轮行驶距离，单位 mm，可为负数
 * @retval   对应的理论编码器计数绝对值
 * @note     该结果用于反馈校验，不直接作为 Emm 位置命令的 clk 参数
 */
uint32_t Mecanum_Distance_To_Encoder(float distance_mm);

/**********************************************************
*** 运动时间估算
**********************************************************/
/**
 * @brief    根据测试转速估算车轮完成指定路程需要的时间
 * @param    wheel_distance_mm ：车轮行驶距离，单位 mm，可为负数
 * @retval   理论运动时间，单位 ms；参数无效时返回0
 * @note     仅用于按键测试互锁，不代替电机到位反馈
 */
uint32_t Mecanum_Move_Time_Calc(float wheel_distance_mm);

#endif /* MECANUM_CHASSIS_H */
