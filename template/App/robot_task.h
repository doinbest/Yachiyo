/**
 * @file    robot_task.h
 * @brief   第一版智能搬运任务入口和可观察状态。
 *
 * 当前只完成香橙派链路、任务码确认和安全停止门禁，不自动执行比赛
 * 路线。START消息会被记录但不会驱动电机，避免未验证流程误启动。
 */
#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "competition_task_code.h"
#include <stdbool.h>

/** 第一版任务入口状态。 */
typedef enum
{
  ROBOT_TASK_WAIT_LINK = 0,
  ROBOT_TASK_WAIT_CODE,
  ROBOT_TASK_READY,
  ROBOT_TASK_SAFE_STOP
} RobotTaskState_t;

/** 任务入口状态和通信统计。 */
typedef struct
{
  RobotTaskState_t state;
  CompetitionTaskCode_t task_code;
  uint32_t state_enter_ms;
  uint32_t received_frame_count;
  uint32_t heartbeat_count;
  uint32_t rejected_task_count;
  uint32_t unsupported_message_count;
  uint32_t blocked_start_count;
  uint16_t last_frame_seq;
} RobotTaskStatus_t;

/**
 * @brief    初始化任务码和任务入口状态
 * @param    无
 * @retval   无
 */
void RobotTask_Init(void);

/**
 * @brief    处理香橙派输入并更新WAIT_LINK/WAIT_CODE/READY/SAFE_STOP
 * @param    无
 * @retval   无
 * @note     应在while(1)中持续调用，不包含自动路线动作
 */
void RobotTask_Process(void);

/**
 * @brief    读取当前任务入口状态
 * @param    status ：调用者提供的输出结构体
 * @retval   true  读取成功
 * @retval   false 参数为空
 */
bool RobotTask_Status_Get(RobotTaskStatus_t *status);

#endif /* ROBOT_TASK_H */
