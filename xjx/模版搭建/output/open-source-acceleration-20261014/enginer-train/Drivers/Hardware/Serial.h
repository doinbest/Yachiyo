#ifndef __Serial_H
#define __Serial_H
#include "main.h"
#include "usart.h"


void Serial_Init_IT(UART_HandleTypeDef *huart,uint8_t * Data_Buf);
void Serial_Printf(UART_HandleTypeDef *huart,const char *format, ...);

#endif
