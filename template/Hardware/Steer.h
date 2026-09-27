#ifndef __STEER_H
#define __STEER_H

#include "main.h"

HAL_StatusTypeDef Steer_Init(TIM_HandleTypeDef *htim);
void Steer_SetDuty(float Duty);
/** @brief 将舵机控制脉宽清零；不切断舵机电源，失去信号后的行为取决于舵机。 */
void Steer_SignalOff(void);

#endif /* __STEER_H */
