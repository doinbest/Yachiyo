#include "main.h"

void Tim_enable(TIM_HandleTypeDef *htim,uint8_t flag)
{
	switch(flag)
	{
		case 1:
			HAL_TIM_Base_Start_IT(htim);
			break;
		case 0:
			HAL_TIM_Base_Stop(htim);
			break;
		default:
			break;
	}
}
uint16_t timer_count;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM6)
	{
		timer_count++;
	}
		
}

