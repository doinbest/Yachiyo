/**
 * @file    key.h
 * @brief   板载四按键扫描驱动。
 *
 * 按键使用 CubeMX 已配置的 PE2、PE3、PE4、PE5 输入引脚。
 * 本模块只负责消抖和生成一次性“按下事件”，不直接控制电机。
 */
#ifndef KEY_H
#define KEY_H

#include "main.h"

/**
 * @brief 按键按下事件
 *
 * 枚举顺序与板载 PE2、PE3、PE4、PE5 的物理顺序一致，表示物理键编号；具体动作由当前应用分派。
 */
typedef enum
{
  KEY_EVENT_NONE = 0,
  KEY_EVENT_PE2,
  KEY_EVENT_PE3,
  KEY_EVENT_PE4,
  KEY_EVENT_PE5
} KeyEvent_t;

/**********************************************************
*** 按键初始化
**********************************************************/
/**
  * @brief    初始化四个板载按键的原始状态和稳定状态
  * @param    无
  * @retval   无
  * @note     必须在 MX_GPIO_Init() 之后调用；初始化不会产生按键事件
  */
void Key_Init(void);

/**********************************************************
*** 按键扫描与消抖
**********************************************************/
/**
  * @brief    扫描 PE2、PE3、PE4、PE5，并完成20ms非阻塞消抖
  * @param    无
  * @retval   无
  * @note     应在 while(1) 中持续调用；长按只产生一次按下事件
  */
void Key_Scan(void);

/**********************************************************
*** 按键事件读取
**********************************************************/
/**
  * @brief    读取并清除一个已经消抖完成的按键按下事件
  * @param    无
  * @retval   KEY_EVENT_NONE      ：当前没有新的按键事件
  * @retval   其他 KeyEvent_t 值  ：对应物理按键被按下
  */
KeyEvent_t Key_Get_Press_Event(void);

#endif /* KEY_H */
