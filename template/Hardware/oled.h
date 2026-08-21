/**
 * @file    oled.h
 * @brief   SSD1306 128x64 OLED 硬件 I2C 驱动。
 *
 * 接线：PB6=I2C1_SCL，PB7=I2C1_SDA，模块地址默认0x3C。
 */
#ifndef OLED_H
#define OLED_H

#include "main.h"

#define OLED_WIDTH             128U
#define OLED_PAGE_COUNT          8U
#define OLED_I2C_ADDRESS_7BIT  0x3CU

/**********************************************************
*** OLED初始化
**********************************************************/
/**
  * @brief    初始化 SSD1306 OLED 并清空屏幕
  * @param    hi2c ：OLED 使用的 I2C 句柄，当前工程传入 &hi2c1
  * @retval   HAL_OK      ：OLED 应答正常且初始化完成
  * @retval   其他HAL状态 ：句柄无效、设备无应答或 I2C 发送失败
  * @note     必须在 MX_I2C1_Init() 之后调用
  */
HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *hi2c);

/**********************************************************
*** OLED清屏
**********************************************************/
/**
  * @brief    清空 SSD1306 的8个显示页
  * @param    无
  * @retval   HAL状态
  */
HAL_StatusTypeDef OLED_Clear(void);

/**********************************************************
*** OLED单页清除
**********************************************************/
/**
  * @brief    清空指定的一页显示内容
  * @param    page ：页号，范围0~7，每页高度8像素
  * @retval   HAL状态
  */
HAL_StatusTypeDef OLED_Page_Clear(uint8_t page);

/**********************************************************
*** OLED字符串显示
**********************************************************/
/**
  * @brief    使用6x8字库在指定位置显示 ASCII 字符串
  * @param    x    ：起始横坐标，范围0~127
  * @param    page ：起始页号，范围0~7
  * @param    text ：以 '\0' 结尾的 ASCII 字符串
  * @retval   HAL状态
  * @note     超出屏幕右边界的字符会被停止显示，不自动换行
  */
HAL_StatusTypeDef OLED_String_Show(uint8_t x, uint8_t page,
                                   const char *text);

/**********************************************************
*** OLED整行无闪烁刷新
**********************************************************/
/**
  * @brief    在内存中生成完整页数据，再通过一次 I2C 传输刷新整行
  * @param    page ：页号，范围0~7
  * @param    text ：以 '\0' 结尾的 ASCII 字符串
  * @retval   HAL状态
  * @note     未使用区域自动填0，动态显示时不会出现先清屏再重画的闪烁
  */
HAL_StatusTypeDef OLED_Line_Show(uint8_t page, const char *text);

/**********************************************************
*** OLED显示开关
**********************************************************/
HAL_StatusTypeDef OLED_Display_On(void);
HAL_StatusTypeDef OLED_Display_Off(void);

#endif /* OLED_H */
