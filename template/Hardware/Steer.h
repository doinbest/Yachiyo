#ifndef __STEER_H
#define __STEER_H

#include "main.h"

HAL_StatusTypeDef Steer_Init(TIM_HandleTypeDef *htim);
void Steer_SetDuty(float Duty);

#endif /* __STEER_H */
