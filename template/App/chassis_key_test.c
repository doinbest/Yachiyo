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
static uint32_t chassis_last_control_tick = 0U;
static float chassis_forward_mm_s = 0.0f;
static float chassis_left_mm_s = 0.0f;
static Mecanum_HeadingPid_t chassis_heading_pid;

/**********************************************************
*** 启动一次底盘平移测试
**********************************************************/
/**
  * @brief    向麦轮控制层发送一次平移命令，并启动重复按键互锁计时
  * @param    forward_mm      ：车体前后位移，向前为正，单位 mm
  * @param    left_mm         ：车体左右位移，向左为正，单位 mm
  * @param    current_yaw_deg ：启动时的IMU航向角，单位 deg
  * @retval   无
  * @note     本测试的期望运动只有平移，yaw由IMU航向PID实时修正
  */
static void Chassis_Key_Move_Start(float forward_mm,
                                   float left_mm,
                                   float current_yaw_deg)
{
  float geometry_check[MECANUM_WHEEL_COUNT];

  /* 旋转修正需要轴距和轮距，未实测配置时不启动按键运动。 */
  if (!Mecanum_Wheel_Speed_Calc(0.0f, 0.0f, 1.0f, geometry_check))
  {
    return;
  }

  chassis_forward_mm_s = (forward_mm >= 0.0f) ?
                         CHASSIS_KEY_TEST_SPEED_MM_S :
                         -CHASSIS_KEY_TEST_SPEED_MM_S;
  chassis_left_mm_s = (left_mm >= 0.0f) ?
                      CHASSIS_KEY_TEST_SPEED_MM_S :
                      -CHASSIS_KEY_TEST_SPEED_MM_S;

  if (forward_mm == 0.0f)
  {
    chassis_forward_mm_s = 0.0f;
  }
  if (left_mm == 0.0f)
  {
    chassis_left_mm_s = 0.0f;
  }

  Mecanum_HeadingPid_Set_Target(&chassis_heading_pid,
                                current_yaw_deg);

  if (Mecanum_Heading_Hold_Step(&chassis_heading_pid,
                                current_yaw_deg,
                                chassis_forward_mm_s,
                                chassis_left_mm_s))
  {
    chassis_move_start_tick = HAL_GetTick();
    chassis_last_control_tick = chassis_move_start_tick;
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
  chassis_last_control_tick = chassis_move_start_tick;
  chassis_move_lock_time_ms = 0U;
  chassis_forward_mm_s = 0.0f;
  chassis_left_mm_s = 0.0f;
  Mecanum_HeadingPid_Init(&chassis_heading_pid,
                          CHASSIS_KEY_HEADING_KP,
                          CHASSIS_KEY_HEADING_KI,
                          CHASSIS_KEY_HEADING_KD,
                          (float)CHASSIS_KEY_HEADING_PERIOD_MS / 1000.0f,
                          CHASSIS_KEY_HEADING_INTEGRAL_LIMIT,
                          CHASSIS_KEY_HEADING_OUTPUT_LIMIT);
}

/**********************************************************
*** 按键底盘测试主循环
**********************************************************/
/**
  * @brief    扫描方向按键，并控制底盘前后左右移动1米
  * @param    imu_angle ：最新的JY61P角度数据
  * @retval   无
  * @note     应在 while(1) 中持续调用；每次有效按下只发送一组同步命令
  */
void Chassis_Key_Process(const JY61P_Angle_t *imu_angle)
{
  KeyEvent_t key_event;
  uint32_t current_tick;

  if (imu_angle == NULL)
  {
    return;
  }

  Key_Scan();
  key_event = Key_Get_Press_Event();
  current_tick = HAL_GetTick();

  /*
   * 没有接入四轮到位反馈前，使用理论运动时间加1秒余量作为测试互锁。
   * 运动期间仍会读取并丢弃新按键事件，避免旧事件在停车后突然执行。
   */
  if (chassis_is_moving)
  {
    if (!Delay_Time_Is_Up(chassis_move_start_tick,
                          chassis_move_lock_time_ms))
    {
      if (Delay_Time_Is_Up(chassis_last_control_tick,
                           CHASSIS_KEY_HEADING_PERIOD_MS))
      {
        chassis_last_control_tick = current_tick;
        if (!Mecanum_Heading_Hold_Step(&chassis_heading_pid,
                                       imu_angle->yaw,
                                       chassis_forward_mm_s,
                                       chassis_left_mm_s))
        {
          (void)Mecanum_Velocity_Control(0.0f, 0.0f, 0.0f);
          chassis_is_moving = false;
        }
      }
      return;
    }

    /* 理论距离时间到达后，明确发送四轮速度0，结束本次按键测试。 */
    (void)Mecanum_Velocity_Control(0.0f, 0.0f, 0.0f);
    chassis_is_moving = false;
    chassis_forward_mm_s = 0.0f;
    chassis_left_mm_s = 0.0f;
  }

  if (imu_angle->update_count == 0U)
  {
    /* 尚未收到有效IMU帧，不允许用默认0度作为航向基准启动。 */
    return;
  }

  switch (key_event)
  {
    case KEY_EVENT_FORWARD:
      Chassis_Key_Move_Start(CHASSIS_KEY_TEST_DISTANCE_MM, 0.0f,
                             imu_angle->yaw);
      break;

    case KEY_EVENT_BACKWARD:
      Chassis_Key_Move_Start(-CHASSIS_KEY_TEST_DISTANCE_MM, 0.0f,
                             imu_angle->yaw);
      break;

    case KEY_EVENT_LEFT:
      Chassis_Key_Move_Start(0.0f, CHASSIS_KEY_TEST_DISTANCE_MM,
                             imu_angle->yaw);
      break;

    case KEY_EVENT_RIGHT:
      Chassis_Key_Move_Start(0.0f, -CHASSIS_KEY_TEST_DISTANCE_MM,
                             imu_angle->yaw);
      break;

    case KEY_EVENT_NONE:
    default:
      break;
  }
}
