#ifndef __HWT101_H
#define __HWT101_H
#include "main.h"

extern uint8_t HWT101_rxbuf[255];
extern uint8_t HWT101_rx_temp[255];
extern float global_angle,global_angle_all;
extern float angular_velocity_z;
extern uint8_t HWT101_rxdata;


void HWT101_Init(void);
void HW101_adjust(void);
void HWT101_Receive_Task(void);
void set_angle_zero(void);

void ParseAndPrintData(uint8_t *data, uint16_t length);

#endif
