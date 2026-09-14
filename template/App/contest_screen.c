/**
 * @file    contest_screen.c
 * @brief   工训赛任务码与搬运进度显示模块实现。
 */
#include "contest_screen.h"

#include "tjc_screen.h"

#include <string.h>

#define CONTEST_SCREEN_BOOT_DELAY_MS       800U  /* 串口屏上电稳定等待时间。 */
#define CONTEST_SCREEN_COMMAND_INTERVAL_MS 100U  /* 相邻串口屏命令最小间隔。 */
#define CONTEST_SCREEN_TASK_CODE_LENGTH    15U   /* 有效任务码字符数量。 */
#define CONTEST_SCREEN_DIRTY_TASK          0x01U /* 任务码待刷新标志。 */
#define CONTEST_SCREEN_DIRTY_GRAB          0x02U /* 抓取数待刷新标志。 */
#define CONTEST_SCREEN_DIRTY_PLACE         0x04U /* 放置数待刷新标志。 */
#define CONTEST_SCREEN_DIRTY_STATE         0x08U /* 运行状态待刷新标志。 */
#define CONTEST_SCREEN_DIRTY_ALL           0x0FU /* 所有动态字段待刷新标志。 */

typedef enum
{
  CONTEST_SCREEN_START_WAIT = 0,
  CONTEST_SCREEN_START_PAGE_FIRST,
  CONTEST_SCREEN_START_PAGE_SECOND,
  CONTEST_SCREEN_START_READY
} ContestScreen_StartTypeDef;

static ContestScreen_StartTypeDef ContestScreen_StartState;
static ContestScreen_StateTypeDef ContestScreen_State;
static uint32_t ContestScreen_LastTick;
static char ContestScreen_TaskCode[CONTEST_SCREEN_TASK_CODE_LENGTH + 1U];
static char ContestScreen_GrabText[4];
static char ContestScreen_PlaceText[4];
static uint8_t ContestScreen_GrabCount;
static uint8_t ContestScreen_PlaceCount;
static uint8_t ContestScreen_DirtyFlags;
static uint8_t ContestScreen_NextField;

/** 函数：判断时间是否到达；参数：起始时刻和等待时间；返回值：1到达，0未到。 */
static uint8_t ContestScreen_TimeIsUp(uint32_t StartTick, uint32_t DelayMs)
{
  return ((HAL_GetTick() - StartTick) >= DelayMs) ? 1U : 0U;
}

/** 函数：校验任务码格式；参数：TaskCode字符串；返回值：1正确，0错误。 */
static uint8_t ContestScreen_TaskCodeIsValid(const char *TaskCode)
{
  uint8_t Index;

  if ((TaskCode == NULL) ||
      (strlen(TaskCode) != CONTEST_SCREEN_TASK_CODE_LENGTH))
  {
    return 0U;
  }

  for (Index = 0U; Index < CONTEST_SCREEN_TASK_CODE_LENGTH; ++Index)
  {
    if ((Index == 3U) || (Index == 7U) || (Index == 11U))
    {
      if (TaskCode[Index] != '+')
      {
        return 0U;
      }
    }
    else if ((TaskCode[Index] < '0') || (TaskCode[Index] > '9'))
    {
      return 0U;
    }
  }
  return 1U;
}

/** 函数：取得状态文本；参数：State状态枚举；返回值：UTF-8状态字符串。 */
static const char *ContestScreen_StateTextGet(ContestScreen_StateTypeDef State)
{
  switch (State)
  {
    case CONTEST_SCREEN_WAIT_QR:
      return "\xE7\xAD\x89\xE5\xBE\x85\xE4\xBA\x8C\xE7\xBB\xB4\xE7\xA0\x81";
    case CONTEST_SCREEN_RUNNING:
      return "\xE8\xBF\x90\xE8\xA1\x8C\xE4\xB8\xAD";
    case CONTEST_SCREEN_COMPLETE:
      return "\xE4\xBB\xBB\xE5\x8A\xA1\xE5\xAE\x8C\xE6\x88\x90";
    case CONTEST_SCREEN_ERROR:
    default:
      return "\xE6\x98\xBE\xE7\xA4\xBA\xE6\x95\x85\xE9\x9A\x9C";
  }
}

/** 函数：发送一个脏字段；参数：无；返回值：无；说明：发送失败时保留标志。 */
static void ContestScreen_DirtyFieldSend(void)
{
  uint8_t Attempt;
  uint8_t Field;
  uint8_t DirtyMask;
  HAL_StatusTypeDef Status;

  for (Attempt = 0U; Attempt < 4U; ++Attempt)
  {
    Field = (uint8_t)((ContestScreen_NextField + Attempt) % 4U);
    DirtyMask = (uint8_t)(1U << Field);
    if ((ContestScreen_DirtyFlags & DirtyMask) == 0U)
    {
      continue;
    }

    switch (Field)
    {
      case 0U:
        Status = TJC_Text_Set("main.tTaskCode", ContestScreen_TaskCode);
        break;
      case 1U:
        Status = TJC_Text_Set("main.tGrab", ContestScreen_GrabText);
        break;
      case 2U:
        Status = TJC_Text_Set("main.tPlace", ContestScreen_PlaceText);
        break;
      case 3U:
      default:
        Status = TJC_Text_Set("main.tState",
                             ContestScreen_StateTextGet(ContestScreen_State));
        break;
    }

    if (Status == HAL_OK)
    {
      ContestScreen_DirtyFlags &= (uint8_t)(~DirtyMask);
      ContestScreen_NextField = (uint8_t)((Field + 1U) % 4U);
    }
    break;
  }
}

/** 函数：初始化比赛显示；参数：无；返回值：无；说明：设置等待二维码初值。 */
void ContestScreen_Init(void)
{
  (void)memcpy(ContestScreen_TaskCode, "---+---+---+---",
               CONTEST_SCREEN_TASK_CODE_LENGTH + 1U);
  ContestScreen_GrabText[0] = '0';
  ContestScreen_GrabText[1] = '/';
  ContestScreen_GrabText[2] = '6';
  ContestScreen_GrabText[3] = '\0';
  (void)memcpy(ContestScreen_PlaceText, ContestScreen_GrabText,
               sizeof(ContestScreen_PlaceText));
  ContestScreen_GrabCount = 0U;
  ContestScreen_PlaceCount = 0U;
  ContestScreen_State = CONTEST_SCREEN_WAIT_QR;
  ContestScreen_DirtyFlags = 0U;
  ContestScreen_NextField = 0U;
  ContestScreen_StartState = CONTEST_SCREEN_START_WAIT;
  ContestScreen_LastTick = HAL_GetTick();
}

/** 函数：设置任务码；参数：TaskCode字符串；返回值：1成功，0格式错误。 */
uint8_t ContestScreen_TaskCodeSet(const char *TaskCode)
{
  if (ContestScreen_TaskCodeIsValid(TaskCode) == 0U)
  {
    return 0U;
  }
  if (strcmp(ContestScreen_TaskCode, TaskCode) != 0)
  {
    (void)memcpy(ContestScreen_TaskCode, TaskCode,
                 CONTEST_SCREEN_TASK_CODE_LENGTH + 1U);
    ContestScreen_DirtyFlags |= CONTEST_SCREEN_DIRTY_TASK;
  }
  return 1U;
}

/** 函数：设置比赛进度；参数：抓取数和放置数；返回值：1成功，0越界。 */
uint8_t ContestScreen_ProgressSet(uint8_t GrabCount, uint8_t PlaceCount)
{
  if ((GrabCount > 6U) || (PlaceCount > 6U))
  {
    return 0U;
  }
  if (ContestScreen_GrabCount != GrabCount)
  {
    ContestScreen_GrabCount = GrabCount;
    ContestScreen_GrabText[0] = (char)('0' + GrabCount);
    ContestScreen_DirtyFlags |= CONTEST_SCREEN_DIRTY_GRAB;
  }
  if (ContestScreen_PlaceCount != PlaceCount)
  {
    ContestScreen_PlaceCount = PlaceCount;
    ContestScreen_PlaceText[0] = (char)('0' + PlaceCount);
    ContestScreen_DirtyFlags |= CONTEST_SCREEN_DIRTY_PLACE;
  }
  return 1U;
}

/** 函数：设置比赛状态；参数：State状态枚举；返回值：无。 */
void ContestScreen_StateSet(ContestScreen_StateTypeDef State)
{
  if (State > CONTEST_SCREEN_ERROR)
  {
    State = CONTEST_SCREEN_ERROR;
  }
  if (ContestScreen_State != State)
  {
    ContestScreen_State = State;
    ContestScreen_DirtyFlags |= CONTEST_SCREEN_DIRTY_STATE;
  }
}

/** 函数：处理比赛显示；参数：无；返回值：无；说明：每100ms最多发送一条命令。 */
void ContestScreen_Process(void)
{
  switch (ContestScreen_StartState)
  {
    case CONTEST_SCREEN_START_WAIT:
      if (ContestScreen_TimeIsUp(ContestScreen_LastTick,
                                 CONTEST_SCREEN_BOOT_DELAY_MS) != 0U)
      {
        ContestScreen_StartState = CONTEST_SCREEN_START_PAGE_FIRST;
      }
      break;
    case CONTEST_SCREEN_START_PAGE_FIRST:
      if (TJC_Command_Send("page main") == HAL_OK)
      {
        ContestScreen_LastTick = HAL_GetTick();
        ContestScreen_StartState = CONTEST_SCREEN_START_PAGE_SECOND;
      }
      break;
    case CONTEST_SCREEN_START_PAGE_SECOND:
      if (ContestScreen_TimeIsUp(ContestScreen_LastTick,
                                 CONTEST_SCREEN_COMMAND_INTERVAL_MS) != 0U)
      {
        if (TJC_Command_Send("page main") == HAL_OK)
        {
          ContestScreen_LastTick = HAL_GetTick();
          ContestScreen_DirtyFlags = CONTEST_SCREEN_DIRTY_ALL;
          ContestScreen_StartState = CONTEST_SCREEN_START_READY;
        }
      }
      break;
    case CONTEST_SCREEN_START_READY:
      if ((ContestScreen_DirtyFlags != 0U) &&
          (ContestScreen_TimeIsUp(ContestScreen_LastTick,
                                  CONTEST_SCREEN_COMMAND_INTERVAL_MS) != 0U))
      {
        ContestScreen_DirtyFieldSend();
        ContestScreen_LastTick = HAL_GetTick();
      }
      break;
    default:
      ContestScreen_Init();
      break;
  }
}
