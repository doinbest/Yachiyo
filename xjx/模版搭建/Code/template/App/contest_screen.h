/**
 * @file    contest_screen.h
 * @brief   工训赛任务码与搬运进度显示模块。
 */
#ifndef CONTEST_SCREEN_H
#define CONTEST_SCREEN_H

#include "main.h"

/** 函数：初始化比赛串口屏显示状态；参数：无；返回值：无；说明：先等待屏幕启动800ms。 */
void ContestScreen_Init(void);

/** 函数：设置比赛任务码；参数：DDD+DDD+DDD+DDD字符串；返回值：1成功，0格式错误。 */
uint8_t ContestScreen_TaskCodeSet(const char *TaskCode);
/** @brief 站点强制重发两行与计数；清除旧发送故障，返回格式校验结果。 */
uint8_t ContestScreen_TaskCodeRefresh(const char *TaskCode);
/** @brief 所有待刷新字段均完成DMA；不代表屏幕渲染已人工验收。 */
uint8_t ContestScreen_TaskCodeSent(void);
/** @brief 本轮刷新有发送错误或超过1秒未完成DMA。 */
uint8_t ContestScreen_HasError(void);

/** 函数：设置抓取数和放置数；参数：两个0至6计数；返回值：1成功，0越界。 */
uint8_t ContestScreen_ProgressSet(uint8_t GrabCount, uint8_t PlaceCount);

/** 函数：处理屏幕启动和字段刷新；参数：无；返回值：无；说明：主循环持续调用。 */
void ContestScreen_Process(void);

#endif /* CONTEST_SCREEN_H */
