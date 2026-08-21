/**
 * @file    robot_task.c
 * @brief   第一版智能搬运任务入口实现。
 */
#include "robot_task.h"

#include "orange_pi_link.h"
#include "robot_safety.h"

#include <string.h>

#define ROBOT_TASK_CODE_CONFIRM_COUNT  3U
#define ROBOT_TASK_LINK_TIMEOUT_MS      1500U

static RobotTaskStatus_t robot_task_status;

static void RobotTask_State_Set(RobotTaskState_t state)
{
  if (robot_task_status.state == state)
  {
    return;
  }
  robot_task_status.state = state;
  robot_task_status.state_enter_ms = HAL_GetTick();
}

static void RobotTask_Frame_Process(const OrangePiFrame_t *frame)
{
  TaskCodeSubmitResult_t submit_result;

  if (frame == NULL)
  {
    return;
  }

  ++robot_task_status.received_frame_count;
  robot_task_status.last_frame_seq = frame->seq;

  switch (frame->type)
  {
    case ORANGE_PI_MSG_HEARTBEAT:
      ++robot_task_status.heartbeat_count;
      break;

    case ORANGE_PI_MSG_TASK_CODE:
      submit_result = TaskCode_Binary_Submit(frame->payload,
                                             frame->payload_length,
                                             frame->seq,
                                             HAL_GetTick());
      if (submit_result == TASK_CODE_SUBMIT_REJECTED)
      {
        ++robot_task_status.rejected_task_count;
      }
      else if ((submit_result == TASK_CODE_SUBMIT_CONFIRMED) ||
               (submit_result == TASK_CODE_SUBMIT_UNCHANGED))
      {
        (void)TaskCode_Get(&robot_task_status.task_code);
      }
      break;

    case ORANGE_PI_MSG_START:
      /* 自动路线尚未实现，第一版只记录请求，绝不从串口直接启动底盘。 */
      ++robot_task_status.blocked_start_count;
      break;

    case ORANGE_PI_MSG_STOP:
      RobotSafety_Stop(ROBOT_STOP_REMOTE_REQUEST);
      break;

    case ORANGE_PI_MSG_TARGET_OBSERVATION:
    case ORANGE_PI_MSG_OBSTACLE_OBSERVATION:
    default:
      ++robot_task_status.unsupported_message_count;
      break;
  }
}

void RobotTask_Init(void)
{
  (void)memset(&robot_task_status, 0, sizeof(robot_task_status));
  TaskCode_Init(ROBOT_TASK_CODE_CONFIRM_COUNT);
  robot_task_status.state = ROBOT_TASK_WAIT_LINK;
  robot_task_status.state_enter_ms = HAL_GetTick();
}

void RobotTask_Process(void)
{
  OrangePiFrame_t frame;

  OrangePi_Link_Process();
  while (OrangePi_Link_Frame_Get(&frame))
  {
    RobotTask_Frame_Process(&frame);
  }

  if (RobotSafety_Is_Stopped())
  {
    RobotTask_State_Set(ROBOT_TASK_SAFE_STOP);
  }
  else if (!OrangePi_Link_Is_Alive(ROBOT_TASK_LINK_TIMEOUT_MS))
  {
    RobotTask_State_Set(ROBOT_TASK_WAIT_LINK);
  }
  else if (robot_task_status.task_code.valid)
  {
    RobotTask_State_Set(ROBOT_TASK_READY);
  }
  else
  {
    RobotTask_State_Set(ROBOT_TASK_WAIT_CODE);
  }
}

bool RobotTask_Status_Get(RobotTaskStatus_t *status)
{
  if (status == NULL)
  {
    return false;
  }
  *status = robot_task_status;
  return true;
}
