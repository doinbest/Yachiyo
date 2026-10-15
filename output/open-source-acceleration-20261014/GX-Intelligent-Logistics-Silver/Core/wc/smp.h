#ifndef __smp_H__
#define __smp_H__


#include <stdint.h>
#include "stm32f1xx_hal.h"  // HAL
#include "stdio.h"	
#include  <stdlib.h> 
#include <stdbool.h>
extern double smp_num[3];//【0，1】：偏差值(X轴Y轴值)，【2】:颜色ID（0：红色，1：绿色，2：蓝色）
extern uint8_t smp_wk[3];////0:红色，1：绿色，2：蓝色（这个是数组的下标），里面存放的是物块的位置ID（0：左上，1：右上，2：正下）---------这个每抓一次物块，就会增加1（余3后的值）
extern bool smp_bool; //标志正常接收到数据--1：接受到，0：未接受到

int smp_por(uint8_t byte);
void smp_fs(int n);
void smp_2m_dq(void);
#endif


