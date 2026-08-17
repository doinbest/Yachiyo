/**
 * @file    mecanum_chassis.c
 * @brief   四轮麦克纳姆底盘运动学和步进电机同步控制实现。
 */
#include "mecanum_chassis.h"

#include "chassis_config.h"
#include "delay.h"
#include "Emm_V5.h"

#define MECANUM_PI 3.14159265358979323846f

/* 轮号0~3依次映射电机地址1~4。 */
static const uint8_t motor_address[MECANUM_WHEEL_COUNT] =
{
  CHASSIS_MOTOR_ID_FRONT_LEFT,
  CHASSIS_MOTOR_ID_REAR_LEFT,
  CHASSIS_MOTOR_ID_REAR_RIGHT,
  CHASSIS_MOTOR_ID_FRONT_RIGHT
};

/* 各轮“向车体前方滚动”时对应的 Emm dir 参数。 */
static const uint8_t motor_forward_direction[MECANUM_WHEEL_COUNT] =
{
  CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,
  CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
  CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,
  CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR
};

/**********************************************************
*** 基础数值换算
**********************************************************/
/**
  * @brief    计算浮点数的绝对值
  * @param    value ：输入值
  * @retval   value 的非负绝对值
  */
static float Mecanum_Get_Abs(float value)
{
  return (value >= 0.0f) ? value : -value;
}

/**
  * @brief    将车轮行驶距离换算成电机侧计数值
  * @param    distance_mm          ：车轮行驶距离，单位 mm，可为负数
  * @param    counts_per_motor_rev ：电机每转一圈对应的计数值
  * @retval   距离对应的非负计数值，结果四舍五入到整数
  * @note     计算公式：距离/(PI*轮径)*减速比*每圈计数
  */
static uint32_t Mecanum_Distance_To_Count(float distance_mm,
                                          uint32_t counts_per_motor_rev)
{
  float wheel_circumference_mm;
  float motor_revolutions;
  float count;

  wheel_circumference_mm = MECANUM_PI * CHASSIS_WHEEL_DIAMETER_MM;
  if ((wheel_circumference_mm <= 0.0f) ||
      (CHASSIS_MOTOR_TO_WHEEL_RATIO <= 0.0f))
  {
    return 0U;
  }

  motor_revolutions = Mecanum_Get_Abs(distance_mm) /
                      wheel_circumference_mm;
  motor_revolutions *= CHASSIS_MOTOR_TO_WHEEL_RATIO;
  count = motor_revolutions * (float)counts_per_motor_rev;

  /* 加0.5后取整，减小浮点数转换成整数产生的累计误差。 */
  return (uint32_t)(count + 0.5f);
}

/**********************************************************
*** 麦克纳姆轮逆运动学解算
**********************************************************/
/**
  * @brief    将车体前向、横向和旋转速度分解为四个车轮速度
  * @param    forward_mm_s ：车体前向速度，向前为正，单位 mm/s
  * @param    left_mm_s    ：车体横向速度，向左为正，单位 mm/s
  * @param    yaw_rad_s    ：车体角速度，逆时针为正，单位 rad/s
  * @param    wheel_mm_s   ：四轮输出数组，顺序为左上、左下、右下、右上
  * @retval   true  ：输入有效，四轮速度解算完成
  * @retval   false ：输出指针为空，或旋转时尚未配置轴距/轮距
  */
bool Mecanum_Wheel_Speed_Calc(float forward_mm_s,
                              float left_mm_s,
                              float yaw_rad_s,
                              float wheel_mm_s[MECANUM_WHEEL_COUNT])
{
  float rotation_arm_mm;

  if (wheel_mm_s == NULL)
  {
    return false;
  }

  rotation_arm_mm = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_WIDTH_MM) * 0.5f;
  if ((yaw_rad_s != 0.0f) && (rotation_arm_mm <= 0.0f))
  {
    return false;
  }

  /*
   * X型麦轮逆解：轮缘正速度统一定义为“该轮向车体前方滚动”。
   * 0 左上 = Vx - Vy - (L+W)Wz
   * 1 左下 = Vx + Vy - (L+W)Wz
   * 2 右下 = Vx - Vy + (L+W)Wz
   * 3 右上 = Vx + Vy + (L+W)Wz
   */
  wheel_mm_s[MECANUM_WHEEL_FRONT_LEFT] =
    forward_mm_s - left_mm_s - rotation_arm_mm * yaw_rad_s;
  wheel_mm_s[MECANUM_WHEEL_REAR_LEFT] =
    forward_mm_s + left_mm_s - rotation_arm_mm * yaw_rad_s;
  wheel_mm_s[MECANUM_WHEEL_REAR_RIGHT] =
    forward_mm_s - left_mm_s + rotation_arm_mm * yaw_rad_s;
  wheel_mm_s[MECANUM_WHEEL_FRONT_RIGHT] =
    forward_mm_s + left_mm_s + rotation_arm_mm * yaw_rad_s;

  return true;
}

/**********************************************************
*** 麦克纳姆轮正运动学解算
**********************************************************/
/**
  * @brief    根据四个车轮速度计算车体前向、横向和旋转速度
  * @param    wheel_mm_s   ：四轮速度数组，顺序为左上、左下、右下、右上
  * @param    forward_mm_s ：返回车体前向速度，单位 mm/s
  * @param    left_mm_s    ：返回车体向左速度，单位 mm/s
  * @param    yaw_rad_s    ：返回车体逆时针角速度，单位 rad/s
  * @retval   true  ：正运动学解算完成
  * @retval   false ：参数指针为空，或轴距/轮距未配置导致无法计算角速度
  */
bool Mecanum_Body_Speed_Calc(
  const float wheel_mm_s[MECANUM_WHEEL_COUNT],
  float *forward_mm_s,
  float *left_mm_s,
  float *yaw_rad_s)
{
  float rotation_arm_mm;

  if ((wheel_mm_s == NULL) || (forward_mm_s == NULL) ||
      (left_mm_s == NULL) || (yaw_rad_s == NULL))
  {
    return false;
  }

  *forward_mm_s = (wheel_mm_s[0] + wheel_mm_s[1] +
                   wheel_mm_s[2] + wheel_mm_s[3]) * 0.25f;
  *left_mm_s = (-wheel_mm_s[0] + wheel_mm_s[1] -
                wheel_mm_s[2] + wheel_mm_s[3]) * 0.25f;

  rotation_arm_mm = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_WIDTH_MM) * 0.5f;
  if (rotation_arm_mm <= 0.0f)
  {
    *yaw_rad_s = 0.0f;
    return false;
  }

  *yaw_rad_s = (-wheel_mm_s[0] - wheel_mm_s[1] +
                wheel_mm_s[2] + wheel_mm_s[3]) /
               (4.0f * rotation_arm_mm);
  return true;
}

/**********************************************************
*** 距离与步进电机计数换算
**********************************************************/
/**
  * @brief    将车轮行驶距离换算成 Emm 位置控制命令脉冲数
  * @param    distance_mm ：车轮行驶距离，单位 mm，可为负数
  * @retval   Emm_V5_Pos_Control() 的 clk 参数绝对值
  * @note     方向不包含在返回值中，由调用函数根据距离正负单独设置 dir
  */
uint32_t Mecanum_Distance_To_Pulse(float distance_mm)
{
  return Mecanum_Distance_To_Count(distance_mm,
                                   CHASSIS_COMMAND_PULSES_PER_REV);
}

/**
  * @brief    将车轮行驶距离换算成理论编码器计数
  * @param    distance_mm ：车轮行驶距离，单位 mm，可为负数
  * @retval   对应的理论编码器计数绝对值
  * @note     该结果用于反馈校验，不直接作为 Emm 位置命令的 clk 参数
  */
uint32_t Mecanum_Distance_To_Encoder(float distance_mm)
{
  return Mecanum_Distance_To_Count(distance_mm,
                                   CHASSIS_ENCODER_COUNTS_PER_REV);
}

/**********************************************************
*** 运动时间估算
**********************************************************/
/**
  * @brief    根据测试转速估算车轮完成指定路程需要的时间
  * @param    wheel_distance_mm ：车轮行驶距离，单位 mm，可为负数
  * @retval   理论运动时间，单位 ms；参数无效时返回0
  * @note     仅用于按键测试互锁，不代替电机到位反馈
  */
uint32_t Mecanum_Move_Time_Calc(float wheel_distance_mm)
{
  float wheel_circumference_mm;
  float motor_revolutions;
  float move_time_ms;

  wheel_circumference_mm = MECANUM_PI * CHASSIS_WHEEL_DIAMETER_MM;
  if ((wheel_circumference_mm <= 0.0f) ||
      (CHASSIS_TEST_SPEED_RPM == 0U))
  {
    return 0U;
  }

  motor_revolutions = Mecanum_Get_Abs(wheel_distance_mm) /
                      wheel_circumference_mm;
  motor_revolutions *= CHASSIS_MOTOR_TO_WHEEL_RATIO;
  move_time_ms = motor_revolutions * 60000.0f /
                 (float)CHASSIS_TEST_SPEED_RPM;

  return (uint32_t)(move_time_ms + 0.5f);
}

/**********************************************************
*** 四电机同步位置控制
**********************************************************/
/**
  * @brief    控制四轮按给定车体相对位移同步运动
  * @param    forward_mm ：车体前后位移，向前为正，单位 mm
  * @param    left_mm    ：车体左右位移，向左为正，单位 mm
  * @param    yaw_rad    ：车体旋转角度，逆时针为正，单位 rad
  * @retval   true  ：四台电机缓存命令及广播同步触发命令已经发出
  * @retval   false ：运动学参数无效，未发送电机运动命令
  * @note     每台电机位置命令的 snF=1，最后用广播地址0统一触发
  */
bool Mecanum_Move_Control(float forward_mm,
                          float left_mm,
                          float yaw_rad)
{
  float wheel_distance_mm[MECANUM_WHEEL_COUNT];
  uint32_t command_pulses;
  uint8_t direction;
  uint8_t wheel;

  /* 位移与速度使用同一线性变换，单位分别变成 mm 和 rad。 */
  if (!Mecanum_Wheel_Speed_Calc(forward_mm, left_mm, yaw_rad,
                                wheel_distance_mm))
  {
    return false;
  }

  for (wheel = 0U; wheel < MECANUM_WHEEL_COUNT; ++wheel)
  {
    command_pulses = Mecanum_Distance_To_Pulse(wheel_distance_mm[wheel]);

    /* 距离为正时使用该轮的“向前”方向，距离为负时翻转方向。 */
    direction = motor_forward_direction[wheel];
    if (wheel_distance_mm[wheel] < 0.0f)
    {
      direction = (direction == 0U) ? 1U : 0U;
    }

    /*
     * snF=1：先把位置命令缓存到对应电机，不立即运动。
     * raF=0：相对上一目标位置运动，适合每次按键移动固定距离。
     */
    Emm_V5_Pos_Control(motor_address[wheel], direction,
                       CHASSIS_TEST_SPEED_RPM,
                       CHASSIS_TEST_ACCELERATION,
                       command_pulses, 0U, true);

    /* 官方例程在相邻命令之间延时10ms，确保DMA发送结束且避免粘包。 */
    Delay_Milliseconds(CHASSIS_UART_COMMAND_INTERVAL_MS);
  }

  /* 地址0是广播地址，统一触发四台已经缓存命令的电机。 */
  Emm_V5_Synchronous_motion(0U);
  Delay_Milliseconds(CHASSIS_UART_COMMAND_INTERVAL_MS);

  return true;
}
