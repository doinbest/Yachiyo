#ifndef TURNTABLE_KEY_H
#define TURNTABLE_KEY_H
#include <stdint.h>
#define KEY_START 1U
#define KEY_STEP  2U
/** @brief 初始化 PA15/PB3 低有效按键；上电按住时必须先松开再按。 */
void Key_Init(void);
/** @brief 主循环扫描，返回消抖后的按下事件位图；无长按、不连发。 */
uint8_t Key_Scan(void);
#endif
