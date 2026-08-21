/**
 * @file    robot_safety.h
 * @brief   智能搬运车统一安全停止状态。
 */
#ifndef ROBOT_SAFETY_H
#define ROBOT_SAFETY_H

#include "main.h"
#include <stdbool.h>

/** 第一版能够区分的安全停止原因。 */
typedef enum
{
  ROBOT_STOP_NONE = 0,
  ROBOT_STOP_REMOTE_REQUEST,
  ROBOT_STOP_IMU_TIMEOUT,
  ROBOT_STOP_CHASSIS_COMMAND_FAILED,
  ROBOT_STOP_INTERNAL_ERROR
} RobotStopReason_t;

/** 安全停止锁存状态。 */
typedef struct
{
  RobotStopReason_t reason;
  uint32_t stop_time_ms;
  uint32_t stop_request_count;
  bool stopped;
} RobotSafetyStatus_t;

/**
 * @brief    初始化安全停止状态
 * @param    无
 * @retval   无
 * @note     本函数不使能电机，只清除上电后的软件状态
 */
void RobotSafety_Init(void);

/**
 * @brief    锁存停止原因并向四轮发送广播立即停止命令
 * @param    reason ：停止原因，ROBOT_STOP_NONE按内部错误处理
 * @retval   无
 * @note     首个停止原因会被保留，重复调用不会自动恢复车辆
 */
void RobotSafety_Stop(RobotStopReason_t reason);

/**
 * @brief    判断车辆是否处于锁存安全停止状态
 * @retval   true  已停止，必须复位后才能重新进行按键运动
 * @retval   false 尚未触发安全停止
 */
bool RobotSafety_Is_Stopped(void);

/**
 * @brief    读取安全停止状态
 * @param    status ：调用者提供的输出结构体
 * @retval   true  读取成功
 * @retval   false 参数为空
 */
bool RobotSafety_Status_Get(RobotSafetyStatus_t *status);

#endif /* ROBOT_SAFETY_H */
