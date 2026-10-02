#include "main.h"
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
typedef struct { void *Instance; uint32_t compare; } TIM_HandleTypeDef;
#define TIM1 ((void *)1)
#define TIM_CHANNEL_1 1U
#define __HAL_TIM_SET_COMPARE(timer, channel, value) ((void)(channel), (timer)->compare = (value))
static unsigned starts;
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; starts++; return HAL_OK; }
static bool route_busy, motion_busy;
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
#include "../template/Hardware/Steer.c"
#include "../template/App/Arm.c"
int main(void)
{
  TIM_HandleTypeDef timer = {TIM1, 42};
  assert(Steer_Init(&timer) == HAL_OK && starts == 1 && timer.compare == 0);
  Steer_SetDuty(7.5f); assert(timer.compare == 1500);
  route_busy = true; Steer_SetDuty(8.0f); assert(timer.compare == 1500);
  route_busy = false; motion_busy = true; Steer_SetDuty(8.0f); assert(timer.compare == 1500);
  motion_busy = false; Steer_SetDuty(8.0f); assert(timer.compare == 1600);
  Steer_SignalOff(); assert(timer.compare == 0);
  Arm_GripperSet(ARM_GRIPPER_OPEN); assert(timer.compare == 500);
  Arm_GripperSet(ARM_GRIPPER_CATCH); assert(timer.compare == 700);
  Arm_GripperSet(ARM_GRIPPER_IDLE); assert(timer.compare == 500);
  puts("steer_route_guard_test: OK"); return 0;
}
