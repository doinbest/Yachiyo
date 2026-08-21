/**
 * @file    robot_safety.c
 * @brief   智能搬运车统一安全停止实现。
 */
#include "robot_safety.h"

#include "mecanum_chassis.h"

static RobotSafetyStatus_t robot_safety_status;
static bool robot_safety_initialized = false;

void RobotSafety_Init(void)
{
  robot_safety_status.reason = ROBOT_STOP_NONE;
  robot_safety_status.stop_time_ms = 0U;
  robot_safety_status.stop_request_count = 0U;
  robot_safety_status.stopped = false;
  robot_safety_initialized = true;
}

void RobotSafety_Stop(RobotStopReason_t reason)
{
  ++robot_safety_status.stop_request_count;

  if (robot_safety_status.stopped)
  {
    return;
  }
  if (reason == ROBOT_STOP_NONE)
  {
    reason = ROBOT_STOP_INTERNAL_ERROR;
  }

  robot_safety_status.reason = reason;
  robot_safety_status.stop_time_ms = HAL_GetTick();
  robot_safety_status.stopped = true;

  if (robot_safety_initialized)
  {
    Mecanum_Stop();
  }
}

bool RobotSafety_Is_Stopped(void)
{
  return robot_safety_status.stopped;
}

bool RobotSafety_Status_Get(RobotSafetyStatus_t *status)
{
  if (status == NULL)
  {
    return false;
  }
  *status = robot_safety_status;
  return true;
}
