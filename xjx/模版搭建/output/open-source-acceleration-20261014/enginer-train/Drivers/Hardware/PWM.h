#ifndef __PWM_H
#define __PWM_H
#include "tim.h"

void PWM_Init(TIM_HandleTypeDef *htim, uint32_t TIM_Channel);
void PWM_SetComper(TIM_HandleTypeDef *htim, uint32_t Channel,uint16_t comper);
void Claw_SetAngle(float Angle);
void zhuanpan_SetAngle(float Angle);

#endif
