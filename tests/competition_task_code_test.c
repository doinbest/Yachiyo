/**
 * @file  competition_task_code_test.c
 * @brief PC端验证任务码格式和三帧一致确认。
 */
#include "competition_task_code.h"

#include <assert.h>
#include <stdio.h>

static void TaskCode_Assert_First_Task(const CompetitionTaskCode_t *task)
{
  assert(task != NULL);
  assert(task->valid);
  assert(task->color[0][0] == 4U);
  assert(task->color[0][1] == 5U);
  assert(task->color[0][2] == 2U);
  assert(task->color[3][0] == 3U);
  assert(task->color[3][1] == 1U);
  assert(task->color[3][2] == 2U);
}

int main(void)
{
  const uint8_t valid_binary[TASK_CODE_BINARY_LENGTH] =
  {
    4U, 5U, 2U, 3U, 2U, 1U,
    2U, 5U, 4U, 3U, 1U, 2U
  };
  const uint8_t invalid_binary[TASK_CODE_BINARY_LENGTH] =
  {
    4U, 5U, 7U, 3U, 2U, 1U,
    2U, 5U, 4U, 3U, 1U, 2U
  };
  CompetitionTaskCode_t task;

  TaskCode_Init(3U);
  assert(TaskCode_Binary_Submit(invalid_binary, sizeof(invalid_binary),
                                1U, 10U) ==
         TASK_CODE_SUBMIT_REJECTED);
  assert(TaskCode_Binary_Submit(valid_binary, sizeof(valid_binary),
                                2U, 20U) ==
         TASK_CODE_SUBMIT_CANDIDATE);
  assert(TaskCode_Binary_Submit(valid_binary, sizeof(valid_binary),
                                3U, 30U) ==
         TASK_CODE_SUBMIT_CANDIDATE);
  assert(!TaskCode_Get(&task));
  assert(TaskCode_Binary_Submit(valid_binary, sizeof(valid_binary),
                                4U, 40U) ==
         TASK_CODE_SUBMIT_CONFIRMED);
  assert(TaskCode_Get(&task));
  TaskCode_Assert_First_Task(&task);
  assert(task.source_seq == 4U);
  assert(task.update_ms == 40U);

  assert(TaskCode_Ascii_Submit("452+321+254+312", 5U, 50U) ==
         TASK_CODE_SUBMIT_UNCHANGED);
  assert(TaskCode_Ascii_Submit("452-321+254+312", 6U, 60U) ==
         TASK_CODE_SUBMIT_REJECTED);
  assert(TaskCode_Ascii_Submit("452+321+254+31", 7U, 70U) ==
         TASK_CODE_SUBMIT_REJECTED);

  TaskCode_Reset();
  assert(!TaskCode_Get(&task));
  puts("competition_task_code_test: PASS");
  return 0;
}
