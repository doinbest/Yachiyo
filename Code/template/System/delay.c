/**
 * @file    delay.c
 * @brief   基于 STM32 HAL 时基的系统延时接口实现。
 */
#include "delay.h"

/**********************************************************
*** 毫秒阻塞延时
**********************************************************/
/**
  * @brief    使用 STM32 HAL 时基进行毫秒阻塞延时
  * @param    milliseconds ：需要延时的时间，单位 ms
  * @retval   无
  * @note     内部直接调用 HAL_Delay()，不会重新配置 SysTick
  */
void Delay_Milliseconds(uint32_t milliseconds)
{
  HAL_Delay(milliseconds);
}

/**********************************************************
*** 非阻塞时间判断
**********************************************************/
/**
  * @brief    判断从起始时刻开始是否已经经过指定时间
  * @param    start_tick   ：由 HAL_GetTick() 获取的起始时刻
  * @param    interval_ms ：需要等待的时间间隔，单位 ms
  * @retval   true  ：指定时间已经到达
  * @retval   false ：指定时间尚未到达
  * @note     使用无符号减法，可正确处理32位 HAL Tick 回绕
  */
bool Delay_Time_Is_Up(uint32_t start_tick, uint32_t interval_ms)
{
  /* 无符号减法可以自然兼容 HAL Tick 的32位回绕。 */
  return (HAL_GetTick() - start_tick) >= interval_ms;
}
