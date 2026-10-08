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

/* 轮号0~3依次映射电机地址1~4。 */
static const uint8_t motor_address[MECANUM_WHEEL_COUNT] = {
    CHASSIS_MOTOR_ID_FRONT_LEFT, CHASSIS_MOTOR_ID_REAR_LEFT, CHASSIS_MOTOR_ID_REAR_RIGHT,
    CHASSIS_MOTOR_ID_FRONT_RIGHT};

/* 各轮“向车体前方滚动”时对应的 Emm dir 参数。 */
static const uint8_t motor_forward_direction[MECANUM_WHEEL_COUNT] = {
    CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR, CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR, CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};

/* Motion remains busy until all four stop frames have completed. */
static bool MecanumMotionPending;
static const ChassisModel_Geometry_t Geometry = {
    CHASSIS_WHEELBASE_MM, CHASSIS_TRACK_WIDTH_MM,
    CHASSIS_WHEEL_DIAMETER_MM, CHASSIS_MOTOR_TO_WHEEL_RATIO};
static bool wheel_linear(float forward, float left, float yaw, float out[4])
{
  ChassisModel_Velocity_t body = {forward,left,yaw};
  return ChassisModel_WheelLinearInverse(&Geometry,&body,out);
}
static uint32_t Mecanum_Distance_To_Pulse(float distance_mm);

typedef struct
{
  uint8_t frame[4][13], length[4];
  int32_t rpm[4];
  bool sync;
  uint8_t count;
} Mecanum_Batch_t;
static Mecanum_Batch_t Batch, NextBatch;
static Mecanum_Timing_t BatchTiming;
static uint32_t BatchStartedMs;
void Mecanum_TimingGet(Mecanum_Timing_t *out) { if(out)*out=BatchTiming; }
static bool BatchActive, NextValid, BatchWaiting, BatchStarted, LastBatchValid;
static uint8_t BatchIndex, StopIndex;
static uint32_t LastBatchMs;
static uint32_t TrackingBatchMs;
static bool TrackingEnabled;

static unsigned TrackingFeedbackTurn, TrackingSlowWheel;
void Mecanum_TrackingTiming(bool enabled,uint32_t interval_ms)
{ if(enabled && !TrackingEnabled)memset(&BatchTiming,0,sizeof BatchTiming);
  TrackingEnabled=enabled;TrackingBatchMs=interval_ms<=200?interval_ms:0; }
/* Confirmed Emm42 factory configuration: receive acknowledgement. */
static Mecanum_Status_t BusStatus = {.ack_profile = MECANUM_ACK_RECEIVE};
static Mecanum_Feedback_t Feedback[4];
static bool FeedbackEnabled;
static bool StopFeedbackDemand;
static bool PlannedStopPending;
static uint8_t FeedbackIndex;
static uint8_t FeedbackAddress;

static bool FaultStopPending;
static bool StopFailed;
/* A stopped motor can still contain an untriggered nonzero synchronous command.
 * Zero F6 entries cannot restart motion; later motion overwrites all four. */
static bool SyncCacheUncertain;

static bool BatchZeroVelocity(void)
{
  if (!Batch.sync || Batch.count != MECANUM_WHEEL_COUNT) return false;
  for (unsigned i = 0; i < MECANUM_WHEEL_COUNT; i++)
    if (Batch.length[i] != 8 || Batch.frame[i][1] != 0xf6 || Batch.rpm[i]) return false;
  return true;
}

static bool Mecanum_Ready(void)
{
  /* Preserve the communication fault when a new request is rejected. */
  if (BusStatus.locked || BusStatus.stop_pending || PlannedStopPending || MotorBus_IsQuarantined())
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
      !isfinite(yaw) || !wheel_linear(forward, left, yaw, distance))
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
  if(LastBatchValid && next.length[0]==8 &&
     (uint32_t)(HAL_GetTick()-LastBatchMs)<MECANUM_VELOCITY_REFRESH_MS) {
    const Mecanum_Batch_t *previous=NextValid?&NextBatch:&Batch;
    if(previous->length[0]==8 && !memcmp(next.frame,previous->frame,sizeof next.frame))return true;
  }
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
    if (e.result == MOTOR_BUS_REPLY && e.length==length[kind] &&
        !(e.length==4 && (e.data[2]==0xe2 || e.data[2]==0xee)))
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
      /* Stop cancels unsent queries. No new sample does not invalidate the
       * previous measurement; its unchanged timestamp still expires normally. */
      if (e.result != MOTOR_BUS_CANCELLED)
      {
        if (kind == 0)
          f->speed_valid = false;
        else if (kind == 1)
          f->position_valid = false;
        else
          f->state_valid = false;
      }
      if (MotorBus_IsQuarantined() &&
          !(e.result == MOTOR_BUS_CANCELLED && MotorBus_FeedbackAllowed()))
      {
        Mecanum_Error_t error = EventError(&e);
        if (BusStatus.error != MECANUM_ERROR_NONE && BusStatus.error != MECANUM_ERROR_PROFILE &&
            BusStatus.error != MECANUM_ERROR_CANCELLED)
          error = BusStatus.error;
        if (!BusStatus.stop_pending && (BatchActive || NextValid || MecanumMotionPending))
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
  if ((!FeedbackEnabled && !StopFeedbackDemand) || PlannedStopPending || BusStatus.stop_pending ||
      (BusStatus.locked && BusStatus.error != MECANUM_ERROR_CANCELLED) ||
      !MotorBus_FeedbackAllowed() ||
      MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK))
    return;
  wheel = (uint8_t)(FeedbackIndex / 3);
  kind = (uint8_t)(FeedbackIndex % 3);
  cmd[0] = motor_address[wheel];
  cmd[1] = function[kind];
  cmd[2] = 0x6b;
  if (MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, length[kind], FeedbackIndex, false))
  {
    if(TrackingEnabled && !StopFeedbackDemand)
    {
      /* Four positions, then one wheel's speed/state pair. With X writes and
       * reads sharing UART5, the former 40-read slow cycle exceeded 2 seconds. */
      TrackingFeedbackTurn=(TrackingFeedbackTurn+1U)%6U;
      if(TrackingFeedbackTurn==0)TrackingSlowWheel=(TrackingSlowWheel+1U)%4U;
      if(TrackingFeedbackTurn<4)FeedbackIndex=(uint8_t)(TrackingFeedbackTurn*3+1);
      else if(TrackingFeedbackTurn==4)FeedbackIndex=(uint8_t)(TrackingSlowWheel*3);
      else FeedbackIndex=(uint8_t)(TrackingSlowWheel*3+2);
    }
    else if (FeedbackAddress && !StopFeedbackDemand)
      FeedbackIndex = (uint8_t)((FeedbackAddress - 1) * 3 + (FeedbackIndex + 1) % 3);
    else
      /* Keep each four-wheel measurement group together across velocity batches.
       * Speed 1..4, position 1..4, state 1..4; tokens still encode wheel*3+kind. */
      FeedbackIndex = (uint8_t)(((wheel + 1U) % 4U) * 3U +
                                 (wheel == 3U ? (kind + 1U) % 3U : kind));

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
        if(BatchTiming.batches && LastBatchValid) {
          uint32_t interval=e.tick_ms-BatchTiming.last_ms;
          BatchTiming.intervals++;BatchTiming.interval_bins[interval<200?interval:200]++;
          if(interval>BatchTiming.interval_max_ms)BatchTiming.interval_max_ms=interval;
        }
        BatchTiming.batches++;BatchTiming.last_ms=e.tick_ms;
        if(e.tick_ms-BatchStartedMs>BatchTiming.duration_max_ms)BatchTiming.duration_max_ms=e.tick_ms-BatchStartedMs;
        if (Batch.sync) SyncCacheUncertain = false;
        BatchActive = false;
        BusStatus.stage = MECANUM_STAGE_SENT;
        MotorBus_Release(MOTOR_BUS_CHASSIS);
        memcpy(BusStatus.sent_rpm, Batch.rpm, sizeof(Batch.rpm));
        BusStatus.sent_ms = e.tx_ms; /* FF Receive time is not the DMA send time. */
        BusStatus.sent_velocity_valid = Batch.length[0] == 8;
        BusStatus.tx_complete = true;
        BusStatus.acknowledged = BusStatus.ack_profile == MECANUM_ACK_RECEIVE &&
                                 BusStatus.acknowledged_wheels == Batch.count;

      }
    }
  }
  if (PlannedStopPending)
  {
    /* Consume the current query's matched reply, without enqueueing another.
     * The next main-loop gap then belongs to the planned stop. */
    FeedbackProcess();
    if (Mecanum_CanStopCleanly()) (void)Mecanum_Test_Stop();
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
      (!TrackingEnabled || !TrackingBatchMs || !LastBatchValid ||
       (uint32_t)(HAL_GetTick()-LastBatchMs)>=TrackingBatchMs))
  {
    if (!MotorBus_Reserve(MOTOR_BUS_CHASSIS))
    {
      FeedbackProcess(); /* Consume/queue feedback even while yielding a batch. */
      return;
    }
    Batch = NextBatch;
    NextValid = false;
    BatchActive = true;
    BatchIndex = 0;
    BatchStarted = false;
    LastBatchMs = HAL_GetTick();BatchStartedMs=LastBatchMs;
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
        if (Batch.sync && !BatchZeroVelocity()) SyncCacheUncertain = true;
      }
      else
        BatchFault(MECANUM_ERROR_TX);
    }
    else
    {
      BusStatus.stage = MECANUM_STAGE_SYNC;
      if (MotorBus_SubmitReplyTo(MOTOR_BUS_CHASSIS, sync, 4, 1, 4, BusStatus.sequence, false))
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
/** @brief Clear local stopped-batch state after physical reset or verified
 * zero-cache recovery. Keeps user settings and wheel position snapshots.
 * @retval None. @note Caller must validate the corresponding recovery proof. */
static void RecoveryClear(void)
{
  SyncCacheUncertain = false;
  BusStatus.locked = false;
  BusStatus.error = MECANUM_ERROR_NONE;
  BusStatus.stage = MECANUM_STAGE_IDLE;
  MecanumMotionPending = false;
  BatchWaiting = false;
  LastBatchValid = false;
  FaultStopPending = PlannedStopPending = false;
  memset(BusStatus.sent_rpm, 0, sizeof(BusStatus.sent_rpm));
  BusStatus.sent_ms = HAL_GetTick();
  BusStatus.sent_velocity_valid = false;
}
bool Mecanum_RecoveryAfterReset(void)
{
  if (BusStatus.stop_pending || BatchActive || NextValid || !MotorBus_RecoverAfterReset())
    return false;
  memset(Feedback, 0, sizeof(Feedback));
  RecoveryClear();
  return true;
}
bool Mecanum_RecoveryAfterBusCheck(void)
{
  MotorBus_Recovery_t status;
  MotorBus_RecoveryGet(&status);
  /* Recovery success now includes ACK-confirmed zero overwrite, addressed sync
   * and fresh stopped feedback. Ordinary stopping alone still cannot unlock. */
  if (!status.success || status.active || BusStatus.stop_pending || BatchActive || NextValid ||
      MotorBus_IsQuarantined()) return false;
  /* The recovery owner already drained/recovered the transport. Fresh read-only
   * stop polling may now be queued; do not mistake it for an unfinished batch. */
  RecoveryClear();
  return true;
}
void Mecanum_Feedback_StopMonitor(bool active)
{
  if (StopFeedbackDemand == active) return;
  StopFeedbackDemand = active;
  /* Keep the user's enable/selection policy; only the effective scan changes. */
  FeedbackIndex = active ? 0 : (FeedbackAddress ? (uint8_t)((FeedbackAddress - 1) * 3) : 0);
}
void Mecanum_Feedback_Enable(bool enabled)
{
  FeedbackEnabled = enabled;
}
bool Mecanum_Feedback_Select(uint8_t address)
{
  if (address > 4)
    return false;
  /* Only the next poll's schedule changes. The queued/in-flight read and its
   * reply retain their MotorBus token; FeedbackProcess consumes that reply
   * before submitting another. Waiting for OwnerBusy to clear here deadlocks
   * foreground callers against a continuously replenished feedback stream. */
  FeedbackEnabled = false;
  FeedbackAddress = address;
  FeedbackIndex = (!StopFeedbackDemand && address) ? (uint8_t)((address - 1) * 3) : 0;
  return true;
}
bool Mecanum_FeedbackGet(unsigned wheel, Mecanum_Feedback_t *feedback)
{
  if (wheel >= 4 || !feedback)
    return false;
  *feedback = Feedback[wheel];
  return true;
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
*** 距离与步进电机计数换算
**********************************************************/
/**
 * @brief    将车轮行驶距离换算成 Emm 位置控制命令脉冲数
 * @param    distance_mm ：车轮行驶距离，单位 mm，可为负数
 * @retval   Emm_V5_Pos_Control() 的 clk 参数绝对值
 * @note     方向不包含在返回值中，由调用函数根据距离正负单独设置 dir
 */
static uint32_t Mecanum_Distance_To_Pulse(float distance_mm)
{
  return Mecanum_Distance_To_Count(distance_mm, CHASSIS_COMMAND_PULSES_PER_REV);
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
  if(LastBatchValid && next.length[0]==8 &&
     (uint32_t)(HAL_GetTick()-LastBatchMs)<MECANUM_VELOCITY_REFRESH_MS) {
    const Mecanum_Batch_t *previous=NextValid?&NextBatch:&Batch;
    if(previous->length[0]==8 && !memcmp(next.frame,previous->frame,sizeof next.frame))return true;
  }
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
         !MotorBus_OwnerInFlight(MOTOR_BUS_FEEDBACK);
}

bool Mecanum_RequestPlannedStop(void)
{
  if (PlannedStopPending || BusStatus.stop_pending) return true;
  if (BusStatus.locked || BusStatus.error != MECANUM_ERROR_NONE || MotorBus_IsQuarantined() ||
      !BusStatus.tx_complete || !BusStatus.sent_velocity_valid) return false;
  for (unsigned i=0;i<4;i++) if (BusStatus.sent_rpm[i]) return false;
  /* The synchronized zero snapshot is already confirmed. Stop refreshing it
   * and let the current query finish before FE, rather than racing polling. */
  PlannedStopPending = true;
  NextValid = false;
  return true;
}

bool Mecanum_Test_Stop(void)
{
  PlannedStopPending = false;
  if (BusStatus.stop_pending)
    return true;
  if (BatchActive && BatchStarted && (!BatchZeroVelocity() || SyncCacheUncertain))
  {
    BusStatus.locked = true;
    BusStatus.error = MECANUM_ERROR_CANCELLED;
  }
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
  return PlannedStopPending || MecanumMotionPending ||
         BusStatus.stop_pending || BatchActive || NextValid ||
         BusStatus.stage == MECANUM_STAGE_FAULT;
}

/**********************************************************
*** 连续速度控制
**********************************************************/
bool Mecanum_Velocity_Request(float forward_mm_s, float left_mm_s, float yaw_rad_s, uint8_t acc)
{
  Mecanum_Batch_t next;
  ChassisModel_Velocity_t body = {forward_mm_s,left_mm_s,yaw_rad_s};
  ChassisModel_Wheels_t wheels;
  unsigned i;
  if (!Mecanum_Ready() || !isfinite(forward_mm_s) || !isfinite(left_mm_s) || !isfinite(yaw_rad_s) ||
      !ChassisModel_Inverse(&Geometry,&body,CHASSIS_EMM_MAX_COMMAND_RPM,&wheels))
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
    next.rpm[i] = wheels.command[i];
    VelocityFrame(next.frame[i], i, next.rpm[i], acc, true);
    next.length[i] = 8;
  }
  if(LastBatchValid && next.length[0]==8 &&
     (uint32_t)(HAL_GetTick()-LastBatchMs)<MECANUM_VELOCITY_REFRESH_MS) {
    const Mecanum_Batch_t *previous=NextValid?&NextBatch:&Batch;
    if(previous->length[0]==8 && !memcmp(next.frame,previous->frame,sizeof next.frame))return true;
  }
  NextBatch = next;
  NextValid = true;
  BusStatus.motion_sequence++;
  MecanumMotionPending = true;
  return true;
}
