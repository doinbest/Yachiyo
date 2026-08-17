/**
 * @file    chassis_key_test.c
 * @brief   四按键麦克纳姆底盘1米平移测试程序实现。
 */
#include "chassis_key_test.h"

#include "chassis_config.h"
#include "delay.h"
#include "key.h"
#include "mecanum_chassis.h"

static bool chassis_is_moving = false;
static uint32_t chassis_move_start_tick = 0U;
static uint32_t chassis_move_lock_time_ms = 0U;

/**********************************************************
*** 启动一次底盘平移测试
**********************************************************/
/**
  * @brief    向麦轮控制层发送一次平移命令，并启动重复按键互锁计时
  * @param    forward_mm ：车体前后位移，向前为正，单位 mm
  * @param    left_mm    ：车体左右位移，向左为正，单位 mm
  * @retval   无
  * @note     本测试不包含旋转，因此 yaw 参数固定为0
  */
static void Chassis_Key_Move_Start(float forward_mm, float left_mm)
{
  if (Mecanum_Move_Control(forward_mm, left_mm, 0.0f))
  {
    chassis_move_start_tick = HAL_GetTick();
    chassis_move_lock_time_ms =
      Mecanum_Move_Time_Calc(CHASSIS_KEY_TEST_DISTANCE_MM) +
      CHASSIS_MOVE_FINISH_MARGIN_MS;
    chassis_is_moving = true;
  }
}

/**********************************************************
*** 按键底盘测试初始化
**********************************************************/
/**
  * @brief    初始化四按键状态和运动互锁状态
  * @param    无
  * @retval   无
  * @note     必须在 GPIO、UART5 DMA 和电机上电等待完成后调用
  */
void Chassis_Key_Init(void)
{
  Key_Init();
  chassis_is_moving = false;
  chassis_move_start_tick = HAL_GetTick();
  chassis_move_lock_time_ms = 0U;
}

/**********************************************************
*** 按键底盘测试主循环
**********************************************************/
/**
  * @brief    扫描方向按键，并控制底盘前后左右移动1米
  * @param    无
  * @retval   无
  * @note     应在 while(1) 中持续调用；每次有效按下只发送一组同步命令
  */
void Chassis_Key_Process(void)
{
  KeyEvent_t key_event;

  Key_Scan();
  key_event = Key_Get_Press_Event();

  /*
   * 没有接入四轮到位反馈前，使用理论运动时间加1秒余量作为测试互锁。
   * 运动期间仍会读取并丢弃新按键事件，避免旧事件在停车后突然执行。
   */
  if (chassis_is_moving)
  {
    if (!Delay_Time_Is_Up(chassis_move_start_tick,
                          chassis_move_lock_time_ms))
    {
      return;
    }
    chassis_is_moving = false;
  }

  switch (key_event)
  {
    case KEY_EVENT_FORWARD:
      Chassis_Key_Move_Start(CHASSIS_KEY_TEST_DISTANCE_MM, 0.0f);
      break;

    case KEY_EVENT_BACKWARD:
      Chassis_Key_Move_Start(-CHASSIS_KEY_TEST_DISTANCE_MM, 0.0f);
      break;

    case KEY_EVENT_LEFT:
      Chassis_Key_Move_Start(0.0f, CHASSIS_KEY_TEST_DISTANCE_MM);
      break;

    case KEY_EVENT_RIGHT:
      Chassis_Key_Move_Start(0.0f, -CHASSIS_KEY_TEST_DISTANCE_MM);
      break;

    case KEY_EVENT_NONE:
    default:
      break;
  }
}
