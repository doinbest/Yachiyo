#ifndef __ARM_H
#define __ARM_H

#include "main.h"

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
