#ifndef __LED_H
#define __LED_H

#include "main.h"

/**
 * @brief 板载 LED 的亮灭状态。
 *
 * 当前目标板 LED 沿用参考工程的 PB2 连接，引脚输出高电平时点亮。
 */
typedef enum
{
    LED_OFF = 0,
    LED_ON
} led_state_t;

/**
 * @brief 将板载 LED 设置为默认熄灭状态。
 * @note  请在 MX_GPIO_Init() 之后调用。
 */
void led_init(void);

/**
 * @brief 点亮板载 LED。
 */
void led_on(void);

/**
 * @brief 熄灭板载 LED。
 */
void led_off(void);

/**
 * @brief 翻转板载 LED 的当前状态。
 */
void led_toggle(void);

/**
 * @brief 设置板载 LED 状态。
 * @param state LED_ON 表示点亮，LED_OFF 表示熄灭。
 */
void led_set(led_state_t state);

#endif
