/**
 * @file    contest_screen.h
 * @brief   工训赛任务码与搬运进度显示模块。
 */
#ifndef CONTEST_SCREEN_H
#define CONTEST_SCREEN_H

#include "main.h"

/** 比赛串口屏底部状态。 */
typedef enum
{
  CONTEST_SCREEN_WAIT_QR = 0,
  CONTEST_SCREEN_RUNNING,
  CONTEST_SCREEN_COMPLETE,
  CONTEST_SCREEN_ERROR
} ContestScreen_StateTypeDef;

/** 函数：初始化比赛串口屏显示状态；参数：无；返回值：无；说明：先等待屏幕启动800ms。 */
void ContestScreen_Init(void);

/** 函数：设置比赛任务码；参数：DDD+DDD+DDD+DDD字符串；返回值：1成功，0格式错误。 */
uint8_t ContestScreen_TaskCodeSet(const char *TaskCode);

/** 函数：设置抓取数和放置数；参数：两个0至6计数；返回值：1成功，0越界。 */
uint8_t ContestScreen_ProgressSet(uint8_t GrabCount, uint8_t PlaceCount);

/** 函数：设置比赛运行状态；参数：State状态枚举；返回值：无；说明：无效值显示故障。 */
void ContestScreen_StateSet(ContestScreen_StateTypeDef State);

/** 函数：处理屏幕启动和字段刷新；参数：无；返回值：无；说明：主循环持续调用。 */
void ContestScreen_Process(void);

#endif /* CONTEST_SCREEN_H */
