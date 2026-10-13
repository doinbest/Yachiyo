#include "lidar_servo.h"
#include "tim.h"

static uint8_t Ready;
static uint16_t PulseUs;

HAL_StatusTypeDef LidarServo_Init(void)
{
  HAL_StatusTypeDef result;
  Ready = 0U;
  PulseUs = 0U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
  result = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  Ready = (result == HAL_OK) ? 1U : 0U;
  return result;
}

HAL_StatusTypeDef LidarServo_PulseSet(uint16_t pulse_us)
{
  if (!Ready || pulse_us < 500U || pulse_us > 5000U) return HAL_ERROR;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
  PulseUs = pulse_us;
  return HAL_OK;
}

uint16_t LidarServo_PulseGet(void)
{
  return PulseUs;
}

