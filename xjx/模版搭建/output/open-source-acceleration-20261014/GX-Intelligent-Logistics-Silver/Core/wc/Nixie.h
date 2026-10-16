#ifndef __NIXIE_H
#define __NIXIE_H

#include   "stm32f1xx_hal.h"
// void GPIOMax7219_Init(void);


extern int rwm_ys[6];


void Write_Max7219_byte(unsigned char DATA);
void Write_Max7219(unsigned char address, unsigned char dat);
void MAX7219_Init(void);
void Nixie_showNum(unsigned char index, unsigned char num, unsigned char dot);
void Nixie_showPlus(unsigned char index);
void Nixie_reset(void);
void Nixie_rwm(char*c);

#define NIXIE_LIGHT(light) Write_Max7219(0x03, light) // 修改亮度
#define NIXIE_SHUTDOWN     Write_Max7219(0x0C, 0x00)  // 掉电
#define NIXIE_UP           Write_Max7219(0x0C, 0x01)  // 上电

#define Max7219_pinCLK_GPIO_Port GPIOE
#define Max7219_pinDIN_GPIO_Port GPIOE
#define Max7219_pinCS_GPIO_Port  GPIOE

#define Max7219_pinCLK_Pin GPIO_PIN_9
#define Max7219_pinDIN_Pin GPIO_PIN_7
#define Max7219_pinCS_Pin GPIO_PIN_8



#endif

