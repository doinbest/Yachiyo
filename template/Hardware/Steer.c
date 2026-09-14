#include "Steer.h"
#include "chassis_route.h"
#include "chassis_motion.h"

#define STEER_PWM_PERIOD_COUNT 20000U  /* 50Hz PWM周期对应的计数值。 */

static TIM_HandleTypeDef *Steer_Timer;

/*
 *函数简介:初始化夹爪舵机PWM
 *参数说明:htim  TIM1句柄
 *返回类型:HAL执行状态
 *备注:启动PE9的TIM1_CH1，CCR保持为0，不主动移动夹爪
 */
HAL_StatusTypeDef Steer_Init(TIM_HandleTypeDef *htim)
{
  if ((htim == NULL) || (htim->Instance != TIM1))
  {
    return HAL_ERROR;
  }

  Steer_Timer = htim;
  __HAL_TIM_SET_COMPARE(Steer_Timer, TIM_CHANNEL_1, 0U);
  return HAL_TIM_PWM_Start(Steer_Timer, TIM_CHANNEL_1);
}

/*
 *函数简介:舵机设置占空比
 *参数说明:Duty  舵机占空比，允许范围2.5f～12.5f
 *返回类型:无
 *备注:TIM1已经配置为20ms周期，直接将占空比换算为CCR值
 */
void Steer_SetDuty(float Duty)
{
  uint32_t Compare;

  if (ChassisRoute_IsBusy() || ChassisMotion_IsBusy() ||
      (Steer_Timer == NULL) || (Duty < 2.5f) || (Duty > 12.5f))
  {
    return;
  }

  Compare = (uint32_t)((float)STEER_PWM_PERIOD_COUNT * Duty / 100.0f);
  __HAL_TIM_SET_COMPARE(Steer_Timer, TIM_CHANNEL_1, Compare);
}
