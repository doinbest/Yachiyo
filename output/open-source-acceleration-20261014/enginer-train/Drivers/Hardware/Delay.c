#include "Delay.h"
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 5000000 * us);
    while (delay--)
	{
		;
	}
}

void delay_ms(uint32_t us)
{
   delay_us(1000*us);
}
