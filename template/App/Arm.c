#include "Arm.h"

#include "Steer.h"

static const float Arm_GripperIdleDuty = 2.50f;  /* 500 us：待机时松开。 */
static const float Arm_GripperOpenDuty = 2.50f;  /* 500 us：完全松开。 */
static const float Arm_GripperCatchDuty = 4.00f; /* 800 us：完全抓紧。 */

/*
 *函数简介:夹爪状态设置
 *参数说明:Status  夹爪目标状态
 *返回类型:无
 *备注:50Hz PWM下，待机/松开为500us，抓紧为800us
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

/*
 *函数简介:停止夹爪舵机的有效控制脉冲
 *参数说明:无
 *返回类型:无
 *备注:用于退出OLED舵机测试页，不切断舵机电源
 */
void Arm_GripperSignalOff(void)
{
  Steer_SignalOff();
}
