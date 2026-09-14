#include "chassis_route.h"
#include "chassis_motion.h"
/**
 * @file    mecanum_chassis.c
 * @brief   四轮麦克纳姆底盘运动学和步进电机同步控制实现。
 */
#include "mecanum_chassis.h"

#include "chassis_config.h"
#include "motor_bus.h"
#include <string.h>
#include "hwt101_calibration.h"
#include <math.h>

#define MECANUM_PI 3.14159265358979323846f
#define MECANUM_MAX_MOTOR_RPM 5000U

/* 轮号0~3依次映射电机地址1~4。 */
static const uint8_t motor_address[MECANUM_WHEEL_COUNT] = {
    CHASSIS_MOTOR_ID_FRONT_LEFT, CHASSIS_MOTOR_ID_REAR_LEFT, CHASSIS_MOTOR_ID_REAR_RIGHT,
    CHASSIS_MOTOR_ID_FRONT_RIGHT};

/* 各轮“向车体前方滚动”时对应的 Emm dir 参数。 */
static const uint8_t motor_forward_direction[MECANUM_WHEEL_COUNT] = {
    CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR, CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR, CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};

static bool MecanumVelocityActive;
/* 无到位反馈：发过运动命令后，直到四轮停止命令均成功发送才解除。 */
static bool MecanumMotionPending;
static float MecanumVelocityForward;
static float MecanumVelocityLeft;
static float MecanumVelocityYaw;
static uint32_t MecanumVelocityLastSendTick;
static Mecanum_HeadingTestStatus_t HeadingTest = {false, "idle", 0, 0, 0};
static Mecanum_HeadingPid_t HeadingTestPid;
static float HeadingTestSpeed;
static uint32_t HeadingTestStartTick, HeadingTestDuration, HeadingTestLastTick;
static uint32_t HeadingTestSampleTick, HeadingTestSampleCount;
static void Mecanum_HeadingTest_Process(void);

typedef struct
{
  uint8_t frame[4][13], length[4];
  int32_t rpm[4];
  bool sync;
  uint8_t count;
} Mecanum_Batch_t;
static Mecanum_Batch_t Batch, NextBatch;
static bool BatchActive, NextValid, BatchWaiting, BatchStarted, LastBatchValid;
static uint8_t BatchIndex, StopIndex;
static uint32_t LastBatchMs;
static Mecanum_Status_t BusStatus;
static Mecanum_Feedback_t Feedback[4];
static bool FeedbackEnabled;
static uint8_t FeedbackIndex;
static uint8_t FeedbackAddress;
static uint32_t FeedbackLastMs;
static bool FaultStopPending;
static bool StopFailed;

static bool Mecanum_Ready(void)
{
  /* Preserve the communication fault when a new request is rejected. */
  if (BusStatus.locked || BusStatus.stop_pending || MotorBus_IsQuarantined())
    return false;
  if (BusStatus.ack_profile == MECANUM_ACK_UNKNOWN)
  {
    BusStatus.error = MECANUM_ERROR_PROFILE;
    return false;
  }
  return !HWT101_Cal_IsBusy() && !BusStatus.locked && !BusStatus.stop_pending &&
         !MotorBus_IsQuarantined();
}
static void VelocityFrame(uint8_t *p, unsigned wheel, int32_t rpm, uint8_t acc, bool sync)
{
  uint16_t magnitude = (uint16_t)(rpm < 0 ? -rpm : rpm);
  p[0] = motor_address[wheel];
  p[1] = 0xf6;
  p[2] = (uint8_t)(motor_forward_direction[wheel] ^ (rpm < 0));
  p[3] = (uint8_t)(magnitude >> 8);
  p[4] = (uint8_t)magnitude;
  p[5] = acc;
  p[6] = (uint8_t)sync;
  p[7] = 0x6b;
}
static bool QueuePosition(float forward, float left, float yaw, uint16_t speed, uint8_t acc,
                          uint8_t relative)
{
  float distance[4];
  unsigned i;
  uint32_t pulses;
  Mecanum_Batch_t next;
  if (!Mecanum_Ready() || BatchActive || NextValid || !isfinite(forward) || !isfinite(left) ||
      !isfinite(yaw) || !Mecanum_Wheel_Speed_Calc(forward, left, yaw, distance))
    return false;
  memset(&next, 0, sizeof(next));
  next.count = 4;
  next.sync = true;
  for (i = 0; i < 4; i++)
  {
    if (!isfinite(distance[i]) || fabsf(distance[i]) > 1000000.0f)
      return false;
    pulses = Mecanum_Distance_To_Pulse(distance[i]);
    VelocityFrame(next.frame[i], i, distance[i] < 0 ? -(int32_t)speed : speed, acc, true);
    next.frame[i][1] = 0xfd;
    next.frame[i][6] = (uint8_t)(pulses >> 24);
    next.frame[i][7] = (uint8_t)(pulses >> 16);
    next.frame[i][8] = (uint8_t)(pulses >> 8);
    next.frame[i][9] = (uint8_t)pulses;
    next.frame[i][10] = relative;
    next.frame[i][11] = 1;
    next.frame[i][12] = 0x6b;
    next.length[i] = 13;
  }
  Mecanum_VelocityRefresh_Stop();
  NextBatch = next;
  NextValid = true;
  BusStatus.motion_sequence++;
  MecanumMotionPending = true;
  return true;
}
static void BatchFault(Mecanum_Error_t error)
{
  FaultStopPending = !BusStatus.stop_pending;
  BusStatus.error = error;
  BusStatus.stage = MECANUM_STAGE_FAULT;
  BusStatus.locked = true;
  BatchActive = false;
  BatchWaiting = false;
  NextValid = false;
  MecanumMotionPending = true;
  Mecanum_VelocityRefresh_Stop();
  MotorBus_Release(MOTOR_BUS_CHASSIS);
  MotorBus_Release(MOTOR_BUS_STOP);
}
static Mecanum_Error_t EventError(const MotorBus_Event_t *e)
{
  if (e->result == MOTOR_BUS_TIMEOUT)
    return MECANUM_ERROR_TIMEOUT;
  if (e->result == MOTOR_BUS_CANCELLED)
    return MECANUM_ERROR_CANCELLED;
  if (e->result != MOTOR_BUS_TX_DONE && e->result != MOTOR_BUS_REPLY)
    return MECANUM_ERROR_TX;
  if (e->result == MOTOR_BUS_REPLY && e->data[2] != 2)
    return MECANUM_ERROR_ACK;
  return MECANUM_ERROR_NONE;
}
static void FeedbackProcess(void)
{
  MotorBus_Event_t e;
  Mecanum_Feedback_t *f;
  uint32_t magnitude;
  static const uint8_t function[3] = {0x35, 0x36, 0x3a};
  static const uint8_t length[3] = {6, 8, 4};
  uint8_t cmd[3], wheel, kind;
  if (MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e))
  {
    wheel = (uint8_t)(e.token / 3);
    kind = (uint8_t)(e.token % 3);
    f = &Feedback[wheel];
    if (e.result == MOTOR_BUS_REPLY)
    {
      if (kind == 0)
      {
        f->speed_rpm = (int32_t)(((uint32_t)e.data[3] << 8) | e.data[4]);
        if (e.data[2])
          f->speed_rpm = -f->speed_rpm;
        f->speed_ms = e.tick_ms;
        f->speed_valid = true;
      }
      else if (kind == 1)
      {
        magnitude = ((uint32_t)e.data[3] << 24) | ((uint32_t)e.data[4] << 16) |
                    ((uint32_t)e.data[5] << 8) | e.data[6];
        f->position_raw = e.data[2] ? -(int64_t)magnitude : (int64_t)magnitude;
        f->position_ms = e.tick_ms;
        f->position_valid = true;
      }
      else
      {
        f->state_flags = e.data[2];
        f->state_ms = e.tick_ms;
        f->state_valid = true;
      }
    }
    else
    {
      if (kind == 0)
        f->speed_valid = false;
      else if (kind == 1)
        f->position_valid = false;
      else
        f->state_valid = false;
      if (MotorBus_IsQuarantined() &&
          !(e.result == MOTOR_BUS_CANCELLED && MotorBus_FeedbackAllowed()))
      {
        Mecanum_Error_t error = EventError(&e);
        if (BusStatus.error != MECANUM_ERROR_NONE && BusStatus.error != MECANUM_ERROR_PROFILE &&
            BusStatus.error != MECANUM_ERROR_CANCELLED)
          error = BusStatus.error;
        if (!BusStatus.stop_pending && (BatchActive || NextValid || MecanumMotionPending ||
                                        MecanumVelocityActive || HeadingTest.active))
        {
          MotorBus_Cancel(MOTOR_BUS_CHASSIS);
          BatchFault(error);
        }
        else
        {
          BusStatus.error = error;
          BusStatus.locked = true;
          if (!BusStatus.stop_pending)
            BusStatus.stage = MECANUM_STAGE_FAULT;
        }
      }
    }
  }
  if (!FeedbackEnabled || BusStatus.stop_pending ||
      (BusStatus.locked && BusStatus.error != MECANUM_ERROR_CANCELLED) ||
      !MotorBus_FeedbackAllowed() ||
      MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK) || (uint32_t)(HAL_GetTick() - FeedbackLastMs) < 16U)
    return;
  wheel = (uint8_t)(FeedbackIndex / 3);
  kind = (uint8_t)(FeedbackIndex % 3);
  cmd[0] = motor_address[wheel];
  cmd[1] = function[kind];
  cmd[2] = 0x6b;
  if (MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, length[kind], FeedbackIndex, false))
  {
    if (FeedbackAddress)
      FeedbackIndex = (uint8_t)((FeedbackAddress - 1) * 3 + (FeedbackIndex + 1) % 3);
    else
      /* Keep each four-wheel measurement group together across velocity batches.
       * Speed 1..4, position 1..4, state 1..4; tokens still encode wheel*3+kind. */
      FeedbackIndex = (uint8_t)(((wheel + 1U) % 4U) * 3U +
                                 (wheel == 3U ? (kind + 1U) % 3U : kind));
    FeedbackLastMs = HAL_GetTick();
  }
}
void Mecanum_Process(void)
{
  MotorBus_Event_t e;
  uint8_t stop[5], sync[4] = {0, 0xff, 0x66, 0x6b};
  Mecanum_Error_t error;
  if (FaultStopPending)
  {
    FaultStopPending = false;
    (void)Mecanum_Test_Stop();
  }
  if (BusStatus.locked || BusStatus.stop_pending)
    FeedbackProcess();
  if (MotorBus_EventGet(MOTOR_BUS_CHASSIS, &e))
  {
    BatchWaiting = false;
    if (!BusStatus.stop_pending)
    {
      error = EventError(&e);
      if (error != MECANUM_ERROR_NONE)
      {
        BatchFault(error);
        return;
      }
      if (e.result == MOTOR_BUS_REPLY && BatchIndex < Batch.count)
        BusStatus.acknowledged_wheels++;
      BatchIndex++;
      if (BatchIndex > Batch.count || (!Batch.sync && BatchIndex == Batch.count))
      {
        BatchActive = false;
        BusStatus.stage = MECANUM_STAGE_SENT;
        MotorBus_Release(MOTOR_BUS_CHASSIS);
        memcpy(BusStatus.sent_rpm, Batch.rpm, sizeof(Batch.rpm));
        BusStatus.sent_ms = e.tick_ms;
        BusStatus.sent_velocity_valid = Batch.length[0] == 8;
        BusStatus.tx_complete = true;
        BusStatus.acknowledged = BusStatus.ack_profile == MECANUM_ACK_RECEIVE &&
                                 BusStatus.acknowledged_wheels == Batch.count;
      }
    }
  }
  if (BusStatus.stop_pending)
  {
    if (MotorBus_EventGet(MOTOR_BUS_STOP, &e))
    {
      if (e.result != MOTOR_BUS_TX_DONE)
      {
        StopFailed = true;
        BusStatus.locked = true;
        if (BusStatus.error == MECANUM_ERROR_NONE)
          BusStatus.error = EventError(&e);
      }
      StopIndex++;
      if (StopIndex == 4)
      {
        BusStatus.stop_pending = false;
        BusStatus.stage = MECANUM_STAGE_STOPPED;
        MotorBus_Release(MOTOR_BUS_STOP);
        if (StopFailed)
        {
          BusStatus.stage = MECANUM_STAGE_FAULT;
          MecanumMotionPending = true;
          return;
        }
        MecanumMotionPending = false;
        memset(BusStatus.sent_rpm, 0, sizeof(BusStatus.sent_rpm));
        BusStatus.sent_ms = e.tick_ms;
        BusStatus.sent_velocity_valid = true;
        BusStatus.tx_complete = true;
        return;
      }
    }
    if (!MotorBus_OwnerBusy(MOTOR_BUS_STOP))
    {
      stop[0] = motor_address[StopIndex];
      stop[1] = 0xfe;
      stop[2] = 0x98;
      stop[3] = 0;
      stop[4] = 0x6b;
      if (!MotorBus_Submit(MOTOR_BUS_STOP, stop, 5, 0, BusStatus.sequence, true))
      {
        StopFailed = true;
        BusStatus.locked = true;
        BusStatus.error = MECANUM_ERROR_TX;
        StopIndex++;
        if (StopIndex == 4)
        {
          BusStatus.stop_pending = false;
          BusStatus.stage = MECANUM_STAGE_FAULT;
          MotorBus_Release(MOTOR_BUS_STOP);
        }
      }
    }
    return;
  }
  if (BusStatus.locked)
    return;
  if (!BatchActive && NextValid &&
      (!LastBatchValid || (uint32_t)(HAL_GetTick() - LastBatchMs) >= 100U))
  {
    if (!MotorBus_Reserve(MOTOR_BUS_CHASSIS))
      return;
    Batch = NextBatch;
    NextValid = false;
    BatchActive = true;
    BatchIndex = 0;
    BatchStarted = false;
    LastBatchMs = HAL_GetTick();
    LastBatchValid = true;
    BusStatus.sequence++;
    BusStatus.stage = MECANUM_STAGE_WHEELS;
    BusStatus.error = MECANUM_ERROR_NONE;
    BusStatus.tx_complete = false;
    BusStatus.acknowledged = false;
    BusStatus.acknowledged_wheels = 0;
  }
  if (BatchActive && !BatchWaiting && !MotorBus_OwnerBusy(MOTOR_BUS_CHASSIS))
  {
    if (BatchIndex < Batch.count)
    {
      if (MotorBus_Submit(MOTOR_BUS_CHASSIS, Batch.frame[BatchIndex], Batch.length[BatchIndex],
                          BusStatus.ack_profile == MECANUM_ACK_RECEIVE ? 4 : 0, BusStatus.sequence,
                          false))
      {
        BatchWaiting = true;
        BatchStarted = true;
      }
      else
        BatchFault(MECANUM_ERROR_TX);
    }
    else
    {
      BusStatus.stage = MECANUM_STAGE_SYNC;
      if (MotorBus_Submit(MOTOR_BUS_CHASSIS, sync, 4, 0, BusStatus.sequence, false))
        BatchWaiting = true;
      else
        BatchFault(MECANUM_ERROR_TX);
    }
  }
  FeedbackProcess();
}
void Mecanum_StatusGet(Mecanum_Status_t *status)
{
  if (status)
  {
    *status = BusStatus;
    /* A shared-bus fault can precede consumption of the owner's terminal event. */
    if (MotorBus_IsQuarantined())
    {
      status->locked = true;
      if (status->error == MECANUM_ERROR_NONE || status->error == MECANUM_ERROR_PROFILE)
        status->error = MECANUM_ERROR_TX;
    }
  }
}
bool Mecanum_AckProfile_Set(Mecanum_AckProfile_t profile)
{
  if (profile > MECANUM_ACK_RECEIVE || Mecanum_IsBusy() || BatchActive || NextValid)
    return false;
  BusStatus.ack_profile = profile;
  BusStatus.error = MECANUM_ERROR_NONE;
  return true;
}
bool Mecanum_RecoveryAfterReset(void)
{
  if (BusStatus.stop_pending || BatchActive || NextValid || !MotorBus_RecoverAfterReset())
    return false;
  memset(Feedback, 0, sizeof(Feedback));
  BusStatus.locked = false;
  BusStatus.error = MECANUM_ERROR_NONE;
  BusStatus.stage = MECANUM_STAGE_IDLE;
  MecanumMotionPending = false;
  BatchWaiting = false;
  LastBatchValid = false;
  FaultStopPending = false;
  memset(BusStatus.sent_rpm, 0, sizeof(BusStatus.sent_rpm));
  BusStatus.sent_ms = HAL_GetTick();
  BusStatus.sent_velocity_valid = false;
  return true;
}
void Mecanum_Feedback_Enable(bool enabled)
{
  FeedbackEnabled = enabled;
}
bool Mecanum_Feedback_Select(uint8_t address)
{
  if (address > 4 || MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK))
    return false;
  FeedbackEnabled = false;
  FeedbackAddress = address;
  FeedbackIndex = address ? (uint8_t)((address - 1) * 3) : 0;
  return true;
}
bool Mecanum_FeedbackGet(unsigned wheel, Mecanum_Feedback_t *feedback)
{
  if (wheel >= 4 || !feedback)
    return false;
  *feedback = Feedback[wheel];
  return true;
}

static float Mecanum_Clamp(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float Mecanum_Wrap_Angle_Error(float error_deg)
{
  while (error_deg > 180.0f)
  {
    error_deg -= 360.0f;
  }
  while (error_deg < -180.0f)
  {
    error_deg += 360.0f;
  }
  return error_deg;
}

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
static uint32_t Mecanum_Distance_To_Count(float distance_mm, uint32_t counts_per_motor_rev)
{
  float wheel_circumference_mm;
  float motor_revolutions;
  float count;

  wheel_circumference_mm = MECANUM_PI * CHASSIS_WHEEL_DIAMETER_MM;
  if ((wheel_circumference_mm <= 0.0f) || (CHASSIS_MOTOR_TO_WHEEL_RATIO <= 0.0f))
  {
    return 0U;
  }

  motor_revolutions = Mecanum_Get_Abs(distance_mm) / wheel_circumference_mm;
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
bool Mecanum_Wheel_Speed_Calc(float forward_mm_s, float left_mm_s, float yaw_rad_s,
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
  wheel_mm_s[MECANUM_WHEEL_FRONT_LEFT] = forward_mm_s - left_mm_s - rotation_arm_mm * yaw_rad_s;
  wheel_mm_s[MECANUM_WHEEL_REAR_LEFT] = forward_mm_s + left_mm_s - rotation_arm_mm * yaw_rad_s;
  wheel_mm_s[MECANUM_WHEEL_REAR_RIGHT] = forward_mm_s - left_mm_s + rotation_arm_mm * yaw_rad_s;
  wheel_mm_s[MECANUM_WHEEL_FRONT_RIGHT] = forward_mm_s + left_mm_s + rotation_arm_mm * yaw_rad_s;

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
bool Mecanum_Body_Speed_Calc(const float wheel_mm_s[MECANUM_WHEEL_COUNT], float *forward_mm_s,
                             float *left_mm_s, float *yaw_rad_s)
{
  float rotation_arm_mm;

  if ((wheel_mm_s == NULL) || (forward_mm_s == NULL) || (left_mm_s == NULL) || (yaw_rad_s == NULL))
  {
    return false;
  }

  *forward_mm_s = (wheel_mm_s[0] + wheel_mm_s[1] + wheel_mm_s[2] + wheel_mm_s[3]) * 0.25f;
  *left_mm_s = (-wheel_mm_s[0] + wheel_mm_s[1] - wheel_mm_s[2] + wheel_mm_s[3]) * 0.25f;

  rotation_arm_mm = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_WIDTH_MM) * 0.5f;
  if (rotation_arm_mm <= 0.0f)
  {
    *yaw_rad_s = 0.0f;
    return false;
  }

  *yaw_rad_s =
      (-wheel_mm_s[0] - wheel_mm_s[1] + wheel_mm_s[2] + wheel_mm_s[3]) / (4.0f * rotation_arm_mm);
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
  return Mecanum_Distance_To_Count(distance_mm, CHASSIS_COMMAND_PULSES_PER_REV);
}

/**
 * @brief    将车轮行驶距离换算成理论编码器计数
 * @param    distance_mm ：车轮行驶距离，单位 mm，可为负数
 * @retval   对应的理论编码器计数绝对值
 * @note     该结果用于反馈校验，不直接作为 Emm 位置命令的 clk 参数
 */
uint32_t Mecanum_Distance_To_Encoder(float distance_mm)
{
  return Mecanum_Distance_To_Count(distance_mm, CHASSIS_ENCODER_COUNTS_PER_REV);
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
  if ((wheel_circumference_mm <= 0.0f) || (CHASSIS_TEST_SPEED_RPM == 0U))
  {
    return 0U;
  }

  motor_revolutions = Mecanum_Get_Abs(wheel_distance_mm) / wheel_circumference_mm;
  motor_revolutions *= CHASSIS_MOTOR_TO_WHEEL_RATIO;
  move_time_ms = motor_revolutions * 60000.0f / (float)CHASSIS_TEST_SPEED_RPM;

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
 * @retval   true  ：请求已接受，发送和应答状态由 Mecanum_StatusGet 查询
 * @retval   false ：运动学参数无效，未发送电机运动命令
 * @note     每台电机位置命令的 snF=1，最后用广播地址0统一触发
 */
bool Mecanum_Move_Control(float forward_mm, float left_mm, float yaw_rad)
{
  if (ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return false;
  return QueuePosition(forward_mm, left_mm, yaw_rad, CHASSIS_TEST_SPEED_RPM,
                       CHASSIS_TEST_ACCELERATION, 0);
}

/**
 * @brief    低速控制底盘前进或后退，用于检查四轮极性。
 * @param    forward_mm ：前进为正、后退为负，单位mm
 * @retval   true请求已接受；false参数、距离或UART5发送失败
 * @note     四轮先缓存位置命令，再由广播命令同步启动
 */
bool Mecanum_Polarity_Move(float forward_mm)
{
  if (ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return false;
  if (!isfinite(forward_mm) || forward_mm == 0 || fabsf(forward_mm) > CHASSIS_POLARITY_TEST_MAX_MM)
    return false;
  return QueuePosition(forward_mm, 0, 0, CHASSIS_POLARITY_TEST_SPEED_RPM,
                       CHASSIS_POLARITY_TEST_ACCELERATION, 2);
}

/**
 * @brief    让指定车轮按低速连续转动。
 * @param    wheel ：车轮枚举
 * @param    rpm   ：正数向车体前方滚动，负数反向
 * @retval   true请求已接受；false参数或UART5发送失败
 * @note     连续速度命令必须使用wheel stop或chassis stop结束
 */
bool Mecanum_Wheel_Test(MecanumWheel_t wheel, int16_t rpm)
{
  if (ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return false;
  Mecanum_Batch_t next;
  if (!Mecanum_Ready() || BatchActive || NextValid || (unsigned)wheel >= 4 || rpm == 0 ||
      rpm > (int)CHASSIS_WHEEL_TEST_MAX_RPM || rpm < -(int)CHASSIS_WHEEL_TEST_MAX_RPM)
    return false;
  memset(&next, 0, sizeof(next));
  next.count = 1;
  next.length[0] = 8;
  next.rpm[(unsigned)wheel] = rpm;
  VelocityFrame(next.frame[0], (unsigned)wheel, rpm, CHASSIS_POLARITY_TEST_ACCELERATION, false);
  Mecanum_VelocityRefresh_Stop();
  NextBatch = next;
  NextValid = true;
  BusStatus.motion_sequence++;
  MecanumMotionPending = true;
  return true;
}

/**
 * @brief    依次立即停止底盘四个车轮。
 * @retval   true停车请求已接受；false至少一条发送失败
 * @note     使用单地址停止，不影响ID5至ID7机械臂电机
 */
bool Mecanum_CanStopCleanly(void)
{
  return !BatchActive && !BatchWaiting && !BusStatus.locked && !BusStatus.stop_pending &&
         !MotorBus_IsQuarantined() && !MotorBus_OwnerBusy(MOTOR_BUS_CHASSIS) &&
         !MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK);
}

bool Mecanum_Test_Stop(void)
{
  if (BusStatus.stop_pending)
    return true;
  if (BatchActive && BatchStarted)
  {
    BusStatus.locked = true;
    BusStatus.error = MECANUM_ERROR_CANCELLED;
  }
  Mecanum_VelocityRefresh_Stop();
  NextValid = false;
  BatchActive = false;
  BatchWaiting = false;
  MotorBus_Cancel(MOTOR_BUS_CHASSIS);
  MotorBus_Cancel(MOTOR_BUS_FEEDBACK);
  MotorBus_Release(MOTOR_BUS_CHASSIS);
  (void)MotorBus_Reserve(MOTOR_BUS_STOP);
  BusStatus.stop_pending = true;
  BusStatus.stop_sequence++;
  BusStatus.stage = MECANUM_STAGE_STOPPING;
  StopIndex = 0;
  BusStatus.tx_complete = false;
  BusStatus.acknowledged = false;
  BusStatus.acknowledged_wheels = 0;
  StopFailed = false;
  MecanumMotionPending = true;
  return true;
}

bool Mecanum_IsBusy(void)
{
  return MecanumVelocityActive || HeadingTest.active || MecanumMotionPending ||
         BusStatus.stop_pending || BatchActive || NextValid ||
         BusStatus.stage == MECANUM_STAGE_FAULT;
}

/**********************************************************
*** 连续速度控制
**********************************************************/
bool Mecanum_Velocity_Control(float forward_mm_s, float left_mm_s, float yaw_rad_s)
{
  if (ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return false;
  return Mecanum_Velocity_Request(forward_mm_s, left_mm_s, yaw_rad_s,
                                  CHASSIS_VELOCITY_ACCELERATION);
}

bool Mecanum_Velocity_Request(float forward_mm_s, float left_mm_s, float yaw_rad_s, uint8_t acc)
{
  Mecanum_Batch_t next;
  float speed[4], rpm[4], largest = 0, scale = 1;
  unsigned i;
  if (!Mecanum_Ready() || !isfinite(forward_mm_s) || !isfinite(left_mm_s) || !isfinite(yaw_rad_s) ||
      !Mecanum_Wheel_Speed_Calc(forward_mm_s, left_mm_s, yaw_rad_s, speed))
    return false;
  if (CHASSIS_WHEEL_DIAMETER_MM <= 0 || CHASSIS_MOTOR_TO_WHEEL_RATIO <= 0)
    return false;
  /* Position/one-wheel work is not replaceable by a velocity streaming target. */
  if ((BatchActive && (Batch.length[0] != 8 || !Batch.sync)) ||
      (NextValid && (NextBatch.length[0] != 8 || !NextBatch.sync)))
    return false;
  memset(&next, 0, sizeof(next));
  next.count = 4;
  next.sync = true;
  for (i = 0; i < 4; i++)
  {
    rpm[i] =
        speed[i] * 60.0f / (MECANUM_PI * CHASSIS_WHEEL_DIAMETER_MM) * CHASSIS_MOTOR_TO_WHEEL_RATIO;
    if (!isfinite(rpm[i]))
      return false;
    if (fabsf(rpm[i]) > largest)
      largest = fabsf(rpm[i]);
  }
  if (largest > MECANUM_MAX_MOTOR_RPM)
    scale = MECANUM_MAX_MOTOR_RPM / largest;
  for (i = 0; i < 4; i++)
  {
    float value = rpm[i] * scale;
    next.rpm[i] = (int32_t)(value >= 0 ? value + 0.5f : value - 0.5f);
    VelocityFrame(next.frame[i], i, next.rpm[i], acc, true);
    next.length[i] = 8;
  }
  NextBatch = next;
  NextValid = true;
  BusStatus.motion_sequence++;
  MecanumMotionPending = true;
  return true;
}

/**
 * @brief    启动整车持续速度控制。
 * @param    forward_mm_s ：车体前后速度，前进为正，单位mm/s
 * @param    left_mm_s    ：车体左右速度，向左为正，单位mm/s
 * @param    yaw_rad_s    ：车体旋转速度，逆时针为正，单位rad/s
 * @retval   true首帧请求已接受；false运动学参数或UART5发送失败
 * @note     后续由主循环调用Mecanum_Velocity_Process()自动刷新
 */
bool Mecanum_Velocity_Start(float forward_mm_s, float left_mm_s, float yaw_rad_s)
{
  if (HWT101_Cal_IsBusy() || HeadingTest.active)
    return false;
  if (!Mecanum_Velocity_Control(forward_mm_s, left_mm_s, yaw_rad_s))
  {
    return false;
  }

  MecanumVelocityForward = forward_mm_s;
  MecanumVelocityLeft = left_mm_s;
  MecanumVelocityYaw = yaw_rad_s;
  MecanumVelocityActive = true;
  MecanumVelocityLastSendTick = HAL_GetTick();
  return true;
}

/**
 * @brief    刷新整车持续速度控制。
 * @param    无
 * @retval   无
 * @note     速度模式持续运行，直到调用停车接口
 */
void Mecanum_Velocity_Process(void)
{
  uint32_t now;

  Mecanum_Process();
  if (HeadingTest.active)
  {
    Mecanum_HeadingTest_Process();
    return;
  }

  if (!MecanumVelocityActive)
  {
    return;
  }

  now = HAL_GetTick();
  if ((now - MecanumVelocityLastSendTick) >= MECANUM_VELOCITY_REFRESH_MS)
  {
    if (!Mecanum_Velocity_Control(MecanumVelocityForward, MecanumVelocityLeft, MecanumVelocityYaw))
    {
      MecanumVelocityActive = false;
      (void)Mecanum_Test_Stop();
      return;
    }
    MecanumVelocityLastSendTick = now;
  }
}

/**
 * @brief    取消整车持续速度控制。
 * @param    无
 * @retval   无
 * @note     不主动发送停车帧，停车由调用者按需执行
 */
void Mecanum_VelocityRefresh_Stop(void)
{
  MecanumVelocityActive = false;
  if (HeadingTest.active)
  {
    HeadingTest.active = false;
    HeadingTest.reason = "stopped";
    HeadingTest.output_rad_s = 0.0f;
  }
}

static void Mecanum_HeadingTest_Finish(const char *reason)
{
  bool stopped = Mecanum_Test_Stop();
  HeadingTest.reason = stopped ? reason : "stop_tx_error";
}

bool Mecanum_HeadingTest_Start(float forward_mm_s, uint32_t duration_ms)
{
  if (ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return false;
  HWT101_Angle_t angle;
  if (HWT101_Cal_IsBusy())
  {
    if (!HeadingTest.active)
      HeadingTest.reason = "imu_busy";
    return false;
  }
  if (Mecanum_IsBusy())
  {
    if (!HeadingTest.active)
      HeadingTest.reason = "chassis_busy";
    return false;
  }
  HeadingTest.reason = "range";
  if (!isfinite(forward_mm_s) || forward_mm_s < -100.0f || forward_mm_s > 100.0f ||
      duration_ms < 1000U || duration_ms > 10000U)
    return false;
  HeadingTest.reason = "imu_not_verified";
  if (!HWT101_Cal_ControlAngleGet(&angle))
    return false;

  HeadingTest.current_deg = angle.yaw * CHASSIS_HEADING_TEST_YAW_SIGN;
  HeadingTest.target_deg = HeadingTest.current_deg;
  HeadingTest.output_rad_s = 0.0f;
  Mecanum_HeadingPid_Init(&HeadingTestPid, CHASSIS_HEADING_TEST_KP, 0.0f, 0.0f, 0.1f, 0.0f,
                          CHASSIS_HEADING_TEST_OUTPUT_LIMIT);
  Mecanum_HeadingPid_Set_Target(&HeadingTestPid, HeadingTest.target_deg);
  HeadingTestSpeed = forward_mm_s;
  HeadingTestDuration = duration_ms;
  HeadingTestStartTick = HeadingTestLastTick = HAL_GetTick();
  HeadingTestSampleTick = angle.last_update_ms;
  HeadingTestSampleCount = angle.update_count;
  HeadingTest.active = true;
  HeadingTest.reason = "running";
  if (!Mecanum_Velocity_Control(forward_mm_s, 0.0f, 0.0f))
  {
    Mecanum_HeadingTest_Finish("motor_tx_error");
    return false;
  }
  return true;
}

static void Mecanum_HeadingTest_Process(void)
{
  HWT101_Angle_t angle;
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - HeadingTestStartTick) >= HeadingTestDuration)
  {
    Mecanum_HeadingTest_Finish("complete");
    return;
  }
  if (!HWT101_Cal_ControlAngleGet(&angle))
  {
    Mecanum_HeadingTest_Finish("imu_invalid");
    return;
  }
  if ((uint32_t)(now - HeadingTestLastTick) < MECANUM_VELOCITY_REFRESH_MS ||
      angle.update_count == HeadingTestSampleCount)
    return;
  HeadingTest.current_deg = angle.yaw * CHASSIS_HEADING_TEST_YAW_SIGN;
  HeadingTestPid.sample_time_s = (uint32_t)(angle.last_update_ms - HeadingTestSampleTick) / 1000.0f;
  HeadingTest.output_rad_s = Mecanum_HeadingPid_Update(&HeadingTestPid, HeadingTest.current_deg);
  HeadingTestLastTick = now;
  HeadingTestSampleTick = angle.last_update_ms;
  HeadingTestSampleCount = angle.update_count;
  if (!Mecanum_Velocity_Control(HeadingTestSpeed, 0.0f, HeadingTest.output_rad_s))
    Mecanum_HeadingTest_Finish("motor_tx_error");
}

void Mecanum_HeadingTest_StatusGet(Mecanum_HeadingTestStatus_t *status)
{
  if (status != NULL)
    *status = HeadingTest;
}

/**********************************************************
*** 离散位置式航向角 PID
**********************************************************/
void Mecanum_HeadingPid_Init(Mecanum_HeadingPid_t *pid, float kp, float ki, float kd,
                             float sample_time_s, float integral_limit, float output_limit)
{
  if (pid == NULL)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->sample_time_s = sample_time_s;
  pid->integral_limit = Mecanum_Get_Abs(integral_limit);
  pid->output_limit = Mecanum_Get_Abs(output_limit);
  pid->target_yaw_deg = 0.0f;
  pid->integral = 0.0f;
  pid->previous_error_deg = 0.0f;
  pid->output_rad_s = 0.0f;
  pid->initialized = false;
}

void Mecanum_HeadingPid_Set_Target(Mecanum_HeadingPid_t *pid, float target_deg)
{
  if (pid == NULL)
  {
    return;
  }

  pid->target_yaw_deg = Mecanum_Wrap_Angle_Error(target_deg);
  pid->integral = 0.0f;
  pid->previous_error_deg = 0.0f;
  pid->output_rad_s = 0.0f;
  pid->initialized = false;
}

float Mecanum_HeadingPid_Update(Mecanum_HeadingPid_t *pid, float current_yaw_deg)
{
  float error_deg;
  float derivative_deg_s;
  float integral_term;
  float output_rad_s;

  if ((pid == NULL) || (pid->sample_time_s <= 0.0f))
  {
    return 0.0f;
  }

  error_deg = Mecanum_Wrap_Angle_Error(pid->target_yaw_deg - current_yaw_deg);

  if (!pid->initialized)
  {
    pid->previous_error_deg = error_deg;
    pid->initialized = true;
  }

  pid->integral += error_deg * pid->sample_time_s;
  pid->integral = Mecanum_Clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
  derivative_deg_s = (error_deg - pid->previous_error_deg) / pid->sample_time_s;
  integral_term = pid->ki * pid->integral;
  output_rad_s = pid->kp * error_deg + integral_term + pid->kd * derivative_deg_s;
  pid->output_rad_s = Mecanum_Clamp(output_rad_s, -pid->output_limit, pid->output_limit);
  pid->previous_error_deg = error_deg;

  return pid->output_rad_s;
}

bool Mecanum_Heading_Hold_Step(Mecanum_HeadingPid_t *pid, float current_yaw_deg, float forward_mm_s,
                               float left_mm_s)
{
  float yaw_rad_s;

  if (pid == NULL || HWT101_Cal_IsBusy())
  {
    return false;
  }

  yaw_rad_s = Mecanum_HeadingPid_Update(pid, current_yaw_deg);
  return Mecanum_Velocity_Control(forward_mm_s, left_mm_s, yaw_rad_s);
}
