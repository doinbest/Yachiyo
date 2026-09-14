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

#endif /* __ARM_H */
