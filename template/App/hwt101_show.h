/**
 * @file    hwt101_show.h
 * @brief   在 SSD1306 OLED 上显示 HWT101 Z 轴角度。
 */
#ifndef HWT101_SHOW_H
#define HWT101_SHOW_H

#include "hwt101_i2c.h"

/**********************************************************
*** HWT101角度显示初始化
**********************************************************/
/**
  * @brief    初始化 OLED，并显示 HWT101 等待界面
  * @param    hi2c ：OLED 使用的 I2C 句柄，当前工程传入 &hi2c1
  * @retval   HAL状态
  */
HAL_StatusTypeDef HWT101_Show_Init(I2C_HandleTypeDef *hi2c);

/**********************************************************
*** HWT101角度显示刷新
**********************************************************/
/**
  * @brief    将最新的 Yaw 和有效帧计数刷新到 OLED
  * @param    angle ：HWT101 最新 Z 轴角度
  * @retval   HAL状态
  * @note     内部限制为每100ms最多刷新一次，避免频繁占用 I2C 总线
  */
HAL_StatusTypeDef HWT101_Show_Process(const HWT101_Angle_t *angle);

#endif /* HWT101_SHOW_H */
