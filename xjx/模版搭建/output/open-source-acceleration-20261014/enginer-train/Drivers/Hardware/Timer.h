#ifndef __Timer_H
#define __Timer_H
#include "timer.h"

void Tim_enable(TIM_HandleTypeDef *htim,uint8_t flag);
extern uint16_t timer_count;

#endif
