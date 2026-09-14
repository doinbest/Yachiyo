#include "Arm.h"

#include "Steer.h"

static const float Arm_GripperIdleDuty = 7.50f;
static const float Arm_GripperOpenDuty = 7.50f;
static const float Arm_GripperCatchDuty = 7.50f;

/*
 *函数简介:夹爪状态设置
 *参数说明:Status  夹爪目标状态
 *返回类型:无
 *备注:三个安全占空比当前均为7.5%，实物标定后分别修改上方常量
 */
void Arm_GripperSet(Arm_GripperStatusTypeDef Status)
{
  switch (Status)
  {
    case ARM_GRIPPER_IDLE:
      Steer_SetDuty(Arm_GripperIdleDuty);
      break;

    case ARM_GRIPPER_OPEN:
      Steer_SetDuty(Arm_GripperOpenDuty);
      break;

    case ARM_GRIPPER_CATCH:
      Steer_SetDuty(Arm_GripperCatchDuty);
      break;

    default:
      break;
  }
}

/*
 *函数简介:直接设置夹爪标定占空比
 *参数说明:Duty  允许范围2.5f～12.5f
 *返回类型:无
 *备注:供grip duty命令小步标定，实际限幅由Steer模块完成
 */
void Arm_GripperDutySet(float Duty)
{
  Steer_SetDuty(Duty);
}
