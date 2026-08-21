/**
 * @file    mecanum_chassis.h
 * @brief   四轮麦克纳姆底盘运动学和步进电机同步控制。
 */
#ifndef MECANUM_CHASSIS_H
#define MECANUM_CHASSIS_H

#include "main.h"
#include <stdbool.h>

#define MECANUM_WHEEL_COUNT 4U

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
bool Mecanum_Wheel_Speed_Calc(float forward_mm_s,
                              float left_mm_s,
                              float yaw_rad_s,
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
bool Mecanum_Body_Speed_Calc(
  const float wheel_mm_s[MECANUM_WHEEL_COUNT],
  float *forward_mm_s,
  float *left_mm_s,
  float *yaw_rad_s);

/**********************************************************
*** 四电机同步位置控制
**********************************************************/
/**
  * @brief    控制四轮按给定车体相对位移同步运动
  * @param    forward_mm ：车体前后位移，向前为正，单位 mm
  * @param    left_mm    ：车体左右位移，向左为正，单位 mm
  * @param    yaw_rad    ：车体旋转角度，逆时针为正，单位 rad
  * @retval   true  ：四台电机缓存命令及广播同步触发命令已经发出
  * @retval   false ：运动学参数无效，未发送电机运动命令
  * @note     每台电机位置命令的 snF=1，最后用广播地址0统一触发
  */
bool Mecanum_Move_Control(float forward_mm,
                           float left_mm,
                           float yaw_rad);

/**********************************************************
*** 连续速度控制与航向角 PID
**********************************************************/
/**
  * @brief    按车体速度发送四轮速度命令。
  * @param    forward_mm_s ：车体前向速度，向前为正，单位 mm/s
  * @param    left_mm_s    ：车体横向速度，向左为正，单位 mm/s
  * @param    yaw_rad_s    ：车体角速度，逆时针为正，单位 rad/s
  * @retval   true  ：四轮速度命令已经发送
  * @retval   false ：运动学参数无效，未发送命令
  * @note     该函数会依次发送四个 UART5 DMA 命令，禁止在中断中调用。
  */
bool Mecanum_Velocity_Control(float forward_mm_s,
                              float left_mm_s,
                              float yaw_rad_s);

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

/**
  * @brief    初始化航向角 PID。
  * @param    pid               ：PID 状态结构体
  * @param    kp/ki/kd          ：PID 参数，输入误差单位为度，输出单位为rad/s
  * @param    sample_time_s     ：固定调用周期，单位s，例如50ms填写0.05f
  * @param    integral_limit    ：积分项输入限幅，单位deg*s
  * @param    output_limit      ：输出角速度限幅，单位rad/s
  * @retval   无
  */
void Mecanum_HeadingPid_Init(Mecanum_HeadingPid_t *pid,
                             float kp,
                             float ki,
                             float kd,
                             float sample_time_s,
                             float integral_limit,
                             float output_limit);

/**
  * @brief    设置目标航向并清除上一段运动的积分状态。
  * @param    pid       ：PID 状态结构体
  * @param    target_deg：目标航向角，单位deg
  * @retval   无
  */
void Mecanum_HeadingPid_Set_Target(Mecanum_HeadingPid_t *pid,
                                   float target_deg);

/**
  * @brief    根据当前Yaw计算一次离散位置式PID输出。
  * @param    pid             ：PID状态结构体
  * @param    current_yaw_deg ：IMU当前Yaw，单位deg
  * @retval   车体角速度修正量，单位rad/s
  * @note     目标值和当前值的误差会归一化到[-180,180]。
  */
float Mecanum_HeadingPid_Update(Mecanum_HeadingPid_t *pid,
                                float current_yaw_deg);

/**
  * @brief    计算航向PID并发送一次带航向保持的车体速度命令。
  * @param    pid             ：PID状态结构体
  * @param    current_yaw_deg ：IMU当前Yaw，单位deg
  * @param    forward_mm_s    ：期望前向速度，单位mm/s
  * @param    left_mm_s       ：期望左向速度，单位mm/s
  * @retval   true  ：速度命令已发送
  * @retval   false ：参数或运动学配置无效
  * @note     应在固定周期的主循环/任务中调用，禁止在中断中调用。
  */
bool Mecanum_Heading_Hold_Step(Mecanum_HeadingPid_t *pid,
                               float current_yaw_deg,
                               float forward_mm_s,
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
