/**
 * @file    delay.h
 * @brief   基于 STM32 HAL 时基的系统延时接口。
 *
 * 不重新配置 SysTick，避免破坏 HAL_GetTick()、HAL_Delay() 和超时判断。
 */
#ifndef SYSTEM_DELAY_H
#define SYSTEM_DELAY_H

#include "main.h"
#include <stdbool.h>

/**********************************************************
*** 毫秒阻塞延时
**********************************************************/
/**
  * @brief    使用 HAL 时基阻塞等待指定的毫秒数
  * @param    milliseconds ：等待时间，单位 ms
  * @retval   无
  * @note     延时期间 CPU 不执行主循环任务，不适合长时间等待
  */
void Delay_Milliseconds(uint32_t milliseconds);

/**********************************************************
*** 非阻塞时间判断
**********************************************************/
/**
  * @brief    判断从指定时刻起是否已经经过目标时间
  * @param    start_tick   ：起始时刻，由 HAL_GetTick() 获取
  * @param    interval_ms  ：目标时间间隔，单位 ms
  * @retval   true  ：已经达到或超过目标时间
  * @retval   false ：尚未达到目标时间
  * @note     使用无符号减法，可正确处理 HAL Tick 的自然溢出
  */
bool Delay_Time_Is_Up(uint32_t start_tick, uint32_t interval_ms);

#endif /* SYSTEM_DELAY_H */
