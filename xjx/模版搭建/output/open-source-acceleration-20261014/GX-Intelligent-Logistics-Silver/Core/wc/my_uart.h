#ifndef _MY_UART_
#define _MY_UART_

#include "stm32f1xx_hal.h"
#include "stdio.h"	
#include "usart.h"
extern float pos_x;//绝对坐标X轴
extern float pos_y;//绝对坐标Y轴
extern float zangle;
void Data_Analyse(uint8_t rec);
void Data_stop(double*pos_X,double*pos_Y);

void Gyro_Init(void);//陀螺仪Z轴角度初始化函数
void rwm_dq(void);//二维码读取函数
#endif
