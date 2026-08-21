/**
 * @file    competition_task_code.h
 * @brief   2027智能搬运赛题四组三位任务码解析与确认。
 *
 * 任务码包含4个阶段，每阶段3个颜色编号。当前赛题资料给出的
 * 颜色范围为1~6；连续收到若干帧完全一致的数据后才对外确认。
 */
#ifndef COMPETITION_TASK_CODE_H
#define COMPETITION_TASK_CODE_H

#include <stdbool.h>
#include <stdint.h>

#define TASK_CODE_STAGE_COUNT          4U
#define TASK_CODE_ITEM_COUNT           3U
#define TASK_CODE_BINARY_LENGTH        12U
#define TASK_CODE_ASCII_LENGTH         15U
#define TASK_CODE_COLOR_MIN            1U
#define TASK_CODE_COLOR_MAX            6U

/** 赛题资料中给出的颜色编号。 */
typedef enum
{
  TASK_COLOR_RED = 1,
  TASK_COLOR_YELLOW = 2,
  TASK_COLOR_BLUE = 3,
  TASK_COLOR_GREEN = 4,
  TASK_COLOR_BLACK = 5,
  TASK_COLOR_LIGHT_BLUE = 6
} TaskColor_t;

/** 已解析的四组三位任务码。 */
typedef struct
{
  uint8_t color[TASK_CODE_STAGE_COUNT][TASK_CODE_ITEM_COUNT];
  uint16_t source_seq;      /**< 产生本任务码的香橙派帧序号。 */
  uint32_t update_ms;       /**< 最近确认时间，来自HAL_GetTick()。 */
  bool valid;               /**< true表示任务码已经通过一致性确认。 */
} CompetitionTaskCode_t;

/** 一次候选任务码提交的结果。 */
typedef enum
{
  TASK_CODE_SUBMIT_REJECTED = 0,
  TASK_CODE_SUBMIT_CANDIDATE,
  TASK_CODE_SUBMIT_CONFIRMED,
  TASK_CODE_SUBMIT_UNCHANGED
} TaskCodeSubmitResult_t;

/**
 * @brief    初始化任务码确认状态
 * @param    required_match_count ：确认前需要连续一致的帧数，0按1处理
 * @retval   无
 * @note     本函数不会生成默认任务码，初始化后任务保持无效
 */
void TaskCode_Init(uint8_t required_match_count);

/**
 * @brief    提交12字节二进制任务码
 * @param    data   ：按阶段0~3、每阶段项目0~2排列的12个颜色编号
 * @param    length ：必须等于TASK_CODE_BINARY_LENGTH
 * @param    seq    ：来源通信帧序号
 * @param    now_ms ：当前HAL毫秒时间
 * @retval   TASK_CODE_SUBMIT_REJECTED  数据长度或颜色编号非法
 * @retval   TASK_CODE_SUBMIT_CANDIDATE 合法但一致帧数尚不足
 * @retval   TASK_CODE_SUBMIT_CONFIRMED 新任务码达到确认次数
 * @retval   TASK_CODE_SUBMIT_UNCHANGED 与已经确认的任务码相同
 */
TaskCodeSubmitResult_t TaskCode_Binary_Submit(const uint8_t *data,
                                               uint16_t length,
                                               uint16_t seq,
                                               uint32_t now_ms);

/**
 * @brief    提交形如452+321+254+312的ASCII任务码
 * @param    text   ：以\0结束、长度恰好15字节的字符串
 * @param    seq    ：来源通信帧序号
 * @param    now_ms ：当前HAL毫秒时间
 * @retval   含义与TaskCode_Binary_Submit()一致
 */
TaskCodeSubmitResult_t TaskCode_Ascii_Submit(const char *text,
                                              uint16_t seq,
                                              uint32_t now_ms);

/**
 * @brief    读取最近一次已经确认的任务码
 * @param    task ：调用者提供的输出结构体
 * @retval   true  已复制一份有效任务码
 * @retval   false 参数为空或尚无确认任务码
 */
bool TaskCode_Get(CompetitionTaskCode_t *task);

/**
 * @brief    清除候选和已确认任务码
 * @param    无
 * @retval   无
 * @warning  运行中的比赛任务不能静默调用本函数
 */
void TaskCode_Reset(void);

#endif /* COMPETITION_TASK_CODE_H */
