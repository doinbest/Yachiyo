#include "main.h"
#include "tim.h"
void PWM_Init(TIM_HandleTypeDef *htim, uint32_t TIM_Channel)
{
  HAL_TIM_PWM_Start(htim,TIM_Channel);
}

void PWM_SetComper(TIM_HandleTypeDef *htim, uint32_t Channel,uint16_t comper)
{
	__HAL_TIM_SetCompare(htim, TIM_CHANNEL_1,comper);
}

void Claw_SetAngle(float Angle)
{
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,Angle / 180 * 2000 + 500);
}

void zhuanpan_SetAngle(float Angle)
{
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,Angle / 180 * 2000 + 500);
}