#include "main.h"
#include <assert.h>
#include <stdio.h>
typedef struct { void *Instance; uint32_t compare; } TIM_HandleTypeDef;
#define __TIM_H__
#define TIM4 ((void *)4)
#define TIM_CHANNEL_1 1U
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((void)(channel), (timer)->compare = (value))
TIM_HandleTypeDef htim4 = {TIM4, 123};
static unsigned starts;
static HAL_StatusTypeDef start_result = HAL_OK;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ assert(timer == &htim4 && channel == TIM_CHANNEL_1); starts++; return start_result; }
#include "../template/Hardware/lidar_servo.c"
int main(void)
{
  assert(LidarServo_PulseSet(1500) == HAL_ERROR);
  assert(LidarServo_Init() == HAL_OK && starts == 1 && htim4.compare == 0);
  assert(LidarServo_PulseGet() == 0);
  const uint16_t values[] = {500, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 1537};
  for (unsigned i=0; i<sizeof(values)/sizeof(values[0]); i++) {
    assert(LidarServo_PulseSet(values[i]) == HAL_OK);
    assert(htim4.compare == values[i] && LidarServo_PulseGet() == values[i]);
  }
  assert(LidarServo_PulseSet(499) == HAL_ERROR && htim4.compare == 1537);
  assert(LidarServo_PulseSet(5001) == HAL_ERROR && htim4.compare == 1537);
  assert(LidarServo_PulseSet(0) == HAL_ERROR && htim4.compare == 1537);
  assert(LIDAR_SERVO_SCAN_PULSE_US == 2450U && LIDAR_SERVO_STOW_PULSE_US == 1450U);
  assert(LidarServo_PulseSet(LIDAR_SERVO_SCAN_PULSE_US) == HAL_OK && htim4.compare == 2450);
  assert(LidarServo_PulseSet(LIDAR_SERVO_STOW_PULSE_US) == HAL_OK && htim4.compare == 1450);
  start_result=HAL_ERROR;
  assert(LidarServo_Init() == HAL_ERROR && htim4.compare == 0);
  assert(LidarServo_PulseSet(1500) == HAL_ERROR && LidarServo_PulseGet() == 0);
  puts("lidar_servo_test: OK"); return 0;
}
