#include "main.h"
#include "Delay.h"

uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;

	if (HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin) == 0)
	{
		HAL_Delay(20);
		while (HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin) == 0);
		HAL_Delay(20);
		KeyNum = 1;
	}
	if (HAL_GPIO_ReadPin(key2_GPIO_Port,key2_Pin) == 0)
	{
		HAL_Delay(20);
		while (HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin) == 0);
		HAL_Delay(20);
		KeyNum = 2;
	}
	if (HAL_GPIO_ReadPin(key3_GPIO_Port,key3_Pin) == 0)
	{
		HAL_Delay(20);
		while (HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin) == 0);
		HAL_Delay(20);
		KeyNum = 3;
	}
	return KeyNum;
}

////按键中断函数
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin==KEY1_Pin)
//	{
//		if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==0)
//		{
//			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//		}
//		__HAL_GPIO_EXTI_CLEAR_IT(KEY1_Pin);
//	}
//	else if(GPIO_Pin==KEY3_Pin)
//	{
//		if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==0)
//		{
//			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
//		}
//		__HAL_GPIO_EXTI_CLEAR_IT(KEY3_Pin);
//	}
//}

