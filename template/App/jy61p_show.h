/**
 * @file    jy61p_show.h
 * @brief   在 SSD1306 OLED 上显示 JY61P 三轴角度。
 */
#ifndef JY61P_SHOW_H
#define JY61P_SHOW_H

#include "jy61p.h"

/**********************************************************
*** JY61P角度显示初始化
**********************************************************/
/**
  * @brief    初始化 OLED，并显示 JY61P 等待界面
  * @param    hi2c ：OLED 使用的 I2C 句柄，当前工程传入 &hi2c1
  * @retval   HAL状态
  */
HAL_StatusTypeDef JY61P_Show_Init(I2C_HandleTypeDef *hi2c);

/**********************************************************
*** JY61P角度显示刷新
**********************************************************/
/**
  * @brief    将最新的 Roll、Pitch、Yaw 刷新到 OLED
  * @param    angle ：JY61P 最新三轴角度
  * @retval   HAL状态
  * @note     内部限制为每100ms最多刷新一次，避免频繁占用 I2C 总线
  */
HAL_StatusTypeDef JY61P_Show_Process(const JY61P_Angle_t *angle);

#endif /* JY61P_SHOW_H */
