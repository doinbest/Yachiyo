#ifndef __ARM_H
#define __ARM_H

#include "main.h"

/* 用户2026-10-09反装舵机实测；50Hz PWM，单位us。 */
#define ARM_GRIPPER_OPEN_PULSE_US 2500U
#define ARM_GRIPPER_CATCH_PULSE_US 1920U

typedef enum
{
  ARM_GRIPPER_IDLE = 0,
  ARM_GRIPPER_OPEN,
  ARM_GRIPPER_CATCH
} Arm_GripperStatusTypeDef;

void Arm_GripperSet(Arm_GripperStatusTypeDef Status);
void Arm_GripperDutySet(float Duty);
/** @brief 清零夹爪舵机控制脉宽，不切断舵机供电。 */
void Arm_GripperSignalOff(void);

#endif /* __ARM_H */
