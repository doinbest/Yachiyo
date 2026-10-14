#ifndef LIDAR_SERVO_H
#define LIDAR_SERVO_H

#include "main.h"

/* PD12 / TIM4_CH1: 50 Hz, one timer count = 1 us. */
/* TD-8125MG, verified on the installed radar mechanism on 2026-10-12. */
#define LIDAR_SERVO_SCAN_PULSE_US 2450U
#define LIDAR_SERVO_STOW_PULSE_US 1450U
HAL_StatusTypeDef LidarServo_Init(void);
HAL_StatusTypeDef LidarServo_PulseSet(uint16_t pulse_us);
uint16_t LidarServo_PulseGet(void);

#endif
