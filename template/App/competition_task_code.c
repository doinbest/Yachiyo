/**
 * @file    competition_task_code.c
 * @brief   2027智能搬运赛题任务码解析与确认实现。
 */
#include "competition_task_code.h"

#include <string.h>

static CompetitionTaskCode_t task_code_candidate;
static CompetitionTaskCode_t task_code_confirmed;
static uint8_t task_code_required_match_count = 1U;
static uint8_t task_code_current_match_count = 0U;

/** 判断任务码中的12个颜色编号是否完全一致。 */
static bool TaskCode_Color_Is_Equal(const CompetitionTaskCode_t *left,
                                    const CompetitionTaskCode_t *right)
{
  uint8_t stage;
  uint8_t item;

  if ((left == NULL) || (right == NULL))
  {
    return false;
  }

  for (stage = 0U; stage < TASK_CODE_STAGE_COUNT; ++stage)
  {
    for (item = 0U; item < TASK_CODE_ITEM_COUNT; ++item)
    {
      if (left->color[stage][item] != right->color[stage][item])
      {
        return false;
      }
    }
  }
  return true;
}

/** 检查单个颜色编号是否在当前赛题资料给出的1~6范围内。 */
static bool TaskCode_Color_Is_Valid(uint8_t color)
{
  return (color >= TASK_CODE_COLOR_MIN) &&
         (color <= TASK_CODE_COLOR_MAX);
}

/** 将已经完成格式检查的任务码提交给连续一致性确认器。 */
static TaskCodeSubmitResult_t TaskCode_Parsed_Submit(
  const CompetitionTaskCode_t *parsed,
  uint16_t seq,
  uint32_t now_ms)
{
  if (parsed == NULL)
  {
    return TASK_CODE_SUBMIT_REJECTED;
  }

  if (task_code_confirmed.valid &&
      TaskCode_Color_Is_Equal(parsed, &task_code_confirmed))
  {
    task_code_confirmed.source_seq = seq;
    task_code_confirmed.update_ms = now_ms;
    return TASK_CODE_SUBMIT_UNCHANGED;
  }

  if ((task_code_current_match_count > 0U) &&
      TaskCode_Color_Is_Equal(parsed, &task_code_candidate))
  {
    if (task_code_current_match_count < 255U)
    {
      ++task_code_current_match_count;
    }
  }
  else
  {
    task_code_candidate = *parsed;
    task_code_current_match_count = 1U;
  }

  task_code_candidate.source_seq = seq;
  task_code_candidate.update_ms = now_ms;
  task_code_candidate.valid = false;

  if (task_code_current_match_count < task_code_required_match_count)
  {
    return TASK_CODE_SUBMIT_CANDIDATE;
  }

  task_code_confirmed = task_code_candidate;
  task_code_confirmed.valid = true;
  return TASK_CODE_SUBMIT_CONFIRMED;
}

void TaskCode_Init(uint8_t required_match_count)
{
  task_code_required_match_count = required_match_count;
  if (task_code_required_match_count == 0U)
  {
    task_code_required_match_count = 1U;
  }
  TaskCode_Reset();
}

TaskCodeSubmitResult_t TaskCode_Binary_Submit(const uint8_t *data,
                                               uint16_t length,
                                               uint16_t seq,
                                               uint32_t now_ms)
{
  CompetitionTaskCode_t parsed;
  uint8_t stage;
  uint8_t item;
  uint16_t index = 0U;

  if ((data == NULL) || (length != TASK_CODE_BINARY_LENGTH))
  {
    return TASK_CODE_SUBMIT_REJECTED;
  }

  (void)memset(&parsed, 0, sizeof(parsed));
  for (stage = 0U; stage < TASK_CODE_STAGE_COUNT; ++stage)
  {
    for (item = 0U; item < TASK_CODE_ITEM_COUNT; ++item)
    {
      if (!TaskCode_Color_Is_Valid(data[index]))
      {
        return TASK_CODE_SUBMIT_REJECTED;
      }
      parsed.color[stage][item] = data[index];
      ++index;
    }
  }

  return TaskCode_Parsed_Submit(&parsed, seq, now_ms);
}

TaskCodeSubmitResult_t TaskCode_Ascii_Submit(const char *text,
                                              uint16_t seq,
                                              uint32_t now_ms)
{
  uint8_t binary[TASK_CODE_BINARY_LENGTH];
  uint8_t stage;
  uint8_t item;
  uint16_t text_index = 0U;
  uint16_t binary_index = 0U;

  if ((text == NULL) || (strlen(text) != TASK_CODE_ASCII_LENGTH))
  {
    return TASK_CODE_SUBMIT_REJECTED;
  }

  for (stage = 0U; stage < TASK_CODE_STAGE_COUNT; ++stage)
  {
    for (item = 0U; item < TASK_CODE_ITEM_COUNT; ++item)
    {
      if ((text[text_index] < '1') || (text[text_index] > '6'))
      {
        return TASK_CODE_SUBMIT_REJECTED;
      }
      binary[binary_index] = (uint8_t)(text[text_index] - '0');
      ++binary_index;
      ++text_index;
    }

    if (stage < (TASK_CODE_STAGE_COUNT - 1U))
    {
      if (text[text_index] != '+')
      {
        return TASK_CODE_SUBMIT_REJECTED;
      }
      ++text_index;
    }
  }

  return TaskCode_Binary_Submit(binary, sizeof(binary), seq, now_ms);
}

bool TaskCode_Get(CompetitionTaskCode_t *task)
{
  if ((task == NULL) || (!task_code_confirmed.valid))
  {
    return false;
  }

  *task = task_code_confirmed;
  return true;
}

void TaskCode_Reset(void)
{
  (void)memset(&task_code_candidate, 0, sizeof(task_code_candidate));
  (void)memset(&task_code_confirmed, 0, sizeof(task_code_confirmed));
  task_code_current_match_count = 0U;
}
