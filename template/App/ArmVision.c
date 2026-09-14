#include "ArmVision.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "MaterialVision.h"

#include <string.h>

#define ARM_VISION_SAMPLE_COUNT 5U  /* 每次计算采用的连续坐标样本数量。 */
#define ARM_VISION_BASE_CALIBRATION_PULSES 80  /* Base标定试探运动的脉冲数。 */
#define ARM_VISION_X_CALIBRATION_FORWARD_PULSES 160  /* X标定先向外伸出的脉冲数。 */
#define ARM_VISION_MIN_CALIBRATION_PIXEL_RESPONSE 4  /* 单轴标定允许的最小绝对像素变化。 */
#define ARM_VISION_BASE_SPEED_RPM 5U  /* Base视觉微调使用的转速。 */
#define ARM_VISION_X_SPEED_RPM 8U  /* X视觉微调使用的转速。 */
#define ARM_VISION_MOVE_ACCELERATION 8U  /* 视觉微调使用的电机加速度。 */
#define ARM_VISION_ALIGNMENT_DEAD_ZONE_PIXELS 1  /* DX和DY均不超过此值时认为对准。 */
#define ARM_VISION_STABLE_COUNT 3U  /* 连续满足死区的帧组数量。 */
#define ARM_VISION_LARGE_ERROR_PIXELS 30  /* 使用大误差增益的像素分界。 */
#define ARM_VISION_LARGE_ERROR_GAIN 0.60f  /* 大误差位置修正比例。 */
#define ARM_VISION_SMALL_ERROR_GAIN 0.35f  /* 小误差位置修正比例。 */
#define ARM_VISION_MIN_CORRECTION_PULSES 2  /* 小于此值的修正直接置零。 */
#define ARM_VISION_CALIBRATION_SETTLE_MS 150U  /* 标定运动到位后的稳定等待时间。 */
#define ARM_VISION_FIRST_DATA_TIMEOUT_MS 3000U  /* 启动后等待首帧视觉数据的最长时间。 */
#define ARM_VISION_DATA_STALE_TIMEOUT_MS 300U  /* 连续无新视觉数据时的故障阈值。 */
#define ARM_VISION_MOTOR_ARRIVAL_TIMEOUT_MS 5000U  /* 视觉修正等待电机到位的最长时间。 */
#define ARM_VISION_CALIBRATION_TIMEOUT_MS 12000U  /* 视觉标定流程的最长运行时间。 */
#define ARM_VISION_ALIGN_TIMEOUT_MS 15000U  /* 视觉对准流程的最长运行时间。 */

typedef enum
{
  ARM_VISION_STATE_IDLE = 0,
  ARM_VISION_STATE_COLLECT,
  ARM_VISION_STATE_WAIT_POSITION_ACK,
  ARM_VISION_STATE_WAIT_ARRIVAL,
  ARM_VISION_STATE_WAIT_STATE_ACK,
  ARM_VISION_STATE_SETTLE,
  ARM_VISION_STATE_ALIGNED,
  ARM_VISION_STATE_ERROR
} ArmVision_StateTypeDef;

typedef enum
{
  ARM_VISION_PHASE_ALIGN = 0,
  ARM_VISION_PHASE_CAL_BASE_ZERO,
  ARM_VISION_PHASE_CAL_BASE_OUT,
  ARM_VISION_PHASE_CAL_BASE_RETURN,
  ARM_VISION_PHASE_CAL_X_ZERO,
  ARM_VISION_PHASE_CAL_X_OUT,
  ARM_VISION_PHASE_CAL_X_RETURN
} ArmVision_PhaseTypeDef;

static ArmVision_StateTypeDef ArmVision_State;
static ArmVision_PhaseTypeDef ArmVision_Phase;
static MechanicalArm_AxisTypeDef ArmVision_MoveAxis;
static Camera_DataTypeDef ArmVision_LatestData;
static uint8_t ArmVision_LatestDataReady;
static uint8_t ArmVision_ReferenceValid;
static uint8_t ArmVision_CalibrationValid;
static ArmVision_CalibrationSourceTypeDef ArmVision_CalibrationSource;
static uint8_t ArmVision_CalibrationTarget;
static uint8_t ArmVision_CalibrationResultReady;
static ArmVision_ErrorTypeDef ArmVision_Error;
static uint8_t ArmVision_SampleIndex;
static int16_t ArmVision_DxSamples[ARM_VISION_SAMPLE_COUNT];
static int16_t ArmVision_DySamples[ARM_VISION_SAMPLE_COUNT];
static float ArmVision_J11;
static float ArmVision_J12;
static float ArmVision_J21;
static float ArmVision_J22;
static int16_t ArmVision_BaseDx;
static int16_t ArmVision_BaseDy;
static int16_t ArmVision_XDx;
static int16_t ArmVision_XDy;
static float ArmVision_CalibrationDeterminant;
static MechanicalArm_AxisTypeDef ArmVision_ErrorAxis;
static uint8_t ArmVision_ErrorMotorCode;
static uint8_t ArmVision_ErrorStateFlags;
static uint8_t ArmVision_MoveDebugReady;
static ArmVision_MoveDebugDataTypeDef ArmVision_MoveDebugData;
static int16_t ArmVision_ZeroDx;
static int16_t ArmVision_ZeroDy;
static int32_t ArmVision_BaseTotal;
static int32_t ArmVision_XTotal;
static uint8_t ArmVision_CorrectionCount;
static uint8_t ArmVision_StableCount;
static uint8_t ArmVision_HasCameraData;
static uint32_t ArmVision_StartTick;
static uint32_t ArmVision_LastDataTick;
static uint32_t ArmVision_MotionStartTick;
static uint32_t ArmVision_LastPollTick;
static uint32_t ArmVision_SettleStartTick;
static uint32_t ArmVision_StatusSequence;
static uint32_t ArmVision_LastCameraSequence;

/**
  * 函    数：切换视觉联调状态
  * 参    数：State 新状态
  * 返 回 值：无
  * 说    明：状态序号用于OLED只在变化时刷新
  */
static void ArmVision_StateSet(ArmVision_StateTypeDef State)
{
  if (ArmVision_State != State)
  {
    ArmVision_State = State;
    ArmVision_StatusSequence++;
  }
}

/**
  * 函    数：设置视觉错误原因
  * 参    数：Error 新错误原因
  * 返 回 值：无
  * 说    明：错误原因供控制台查询，成功或主动停止时清零
  */
static void ArmVision_ErrorSet(ArmVision_ErrorTypeDef Error)
{
  if (ArmVision_Error != Error)
  {
    ArmVision_Error = Error;
    ArmVision_StatusSequence++;
  }
}

/** 只分类发送结果，不再次请求摄像头；旧任务错误由显示层保留。 */
static uint8_t ArmVision_CameraRequestAccepted(HAL_StatusTypeDef Result)
{
  Camera_SnapshotTypeDef Snapshot;

  if (Result == HAL_OK) return 1U;
  Camera_SnapshotGet(&Snapshot);
  if (Result == HAL_BUSY) ArmVision_ErrorSet(ARM_VISION_ERROR_CAMERA_BUSY);
  else if (!Snapshot.UsbConfigured) ArmVision_ErrorSet(ARM_VISION_ERROR_USB_OFF);
  else ArmVision_ErrorSet(ARM_VISION_ERROR_CAMERA_TX);
  return 0U;
}

/**
  * 函    数：记录电机错误上下文
  * 参    数：Axis 出错轴；MotorCode 驱动器错误码；StateFlags 原始状态字节
  * 返 回 值：无
  * 说    明：保留最近一次错误，供控制台输出具体故障位置和原始值
  */
static void ArmVision_ErrorInfoSet(MechanicalArm_AxisTypeDef Axis,
                                   uint8_t MotorCode,
                                   uint8_t StateFlags)
{
  ArmVision_ErrorAxis = Axis;
  ArmVision_ErrorMotorCode = MotorCode;
  ArmVision_ErrorStateFlags = StateFlags;
}

/**
  * 函    数：根据电机立即返回结果设置错误原因
  * 参    数：Result 电机请求结果；DefaultError 其他结果对应的原因
  * 返 回 值：无
  * 说    明：用于位置命令和状态读取的统一错误分类
  */
static void ArmVision_MotorResultErrorSet(
    MechanicalArm_ResultTypeDef Result,
    ArmVision_ErrorTypeDef DefaultError)
{
  if (Result == MECHANICAL_ARM_RESULT_ACK_TIMEOUT)
  {
    ArmVision_ErrorSet(DefaultError);
  }
  else if (Result == MECHANICAL_ARM_RESULT_MOTOR_ERROR)
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_MOTOR_ERROR);
  }
  else if (Result == MECHANICAL_ARM_RESULT_TX_ERROR)
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_MOTOR_TX);
  }
  else if (Result == MECHANICAL_ARM_RESULT_BUSY)
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_MOTOR_BUSY);
  }
  else if (Result == MECHANICAL_ARM_RESULT_PARAM_ERROR)
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_MOTOR_PARAM);
  }
  else
  {
    ArmVision_ErrorSet(DefaultError);
  }
}

/**
  * 函    数：计算5个有符号样本的中值
  * 参    数：Data 五个输入样本
  * 返 回 值：排序后的中间值
  * 说    明：使用简单插入排序，样本数量固定且很小
  */
static int16_t ArmVision_MedianGet(const int16_t *Data)
{
  int16_t Sorted[ARM_VISION_SAMPLE_COUNT];
  int16_t Value;
  uint8_t Index;
  uint8_t Insert;

  (void)memcpy(Sorted, Data, sizeof(Sorted));
  for (Index = 1U; Index < ARM_VISION_SAMPLE_COUNT; Index++)
  {
    Value = Sorted[Index];
    Insert = Index;
    while ((Insert > 0U) && (Sorted[Insert - 1U] > Value))
    {
      Sorted[Insert] = Sorted[Insert - 1U];
      Insert--;
    }
    Sorted[Insert] = Value;
  }
  return Sorted[ARM_VISION_SAMPLE_COUNT / 2U];
}

/**
  * 函    数：开始收集一组视觉样本
  * 参    数：Phase 样本所属阶段
  * 返 回 值：无
  * 说    明：每个阶段重新收集5个新帧，不复用运动前旧数据
  */
static void ArmVision_CollectStart(ArmVision_PhaseTypeDef Phase)
{
  ArmVision_Phase = Phase;
  ArmVision_SampleIndex = 0U;
  ArmVision_StateSet(ARM_VISION_STATE_COLLECT);
}

/**
  * 函    数：初始化一次视觉对准任务状态
  * 参    数：无
  * 返 回 值：无
  * 说    明：B2物料和B3圆环共用同一套Base、X二维修正流程；
  *           独立camera命令不进入本状态机
  */
static void ArmVision_AlignStart(void)
{
  ArmVision_StartTick = HAL_GetTick();
  ArmVision_LastDataTick = ArmVision_StartTick;
  ArmVision_HasCameraData = 0U;
  ArmVision_BaseTotal = 0;
  ArmVision_XTotal = 0;
  ArmVision_CorrectionCount = 0U;
  ArmVision_StableCount = 0U;
  ArmVision_MoveDebugReady = 0U;
  ArmVision_CollectStart(ARM_VISION_PHASE_ALIGN);
}

/**
  * 函    数：发送一次视觉微调位置命令
  * 参    数：Axis 运动轴；Pulses 相对实际位置脉冲
  * 返 回 值：1表示请求已启动，0表示启动失败
  * 说    明：Base使用5RPM，X使用8RPM，加速度均为8
  */
static uint8_t ArmVision_MoveStart(MechanicalArm_AxisTypeDef Axis,
                                   int32_t Pulses)
{
  MechanicalArm_ResultTypeDef Result;
  uint16_t Speed;

  Speed = (Axis == MECHANICAL_ARM_AXIS_BASE) ?
          ARM_VISION_BASE_SPEED_RPM : ARM_VISION_X_SPEED_RPM;
  Result = MechanicalArm_PositionEx(Axis,
                                    Pulses,
                                    Speed,
                                    ARM_VISION_MOVE_ACCELERATION,
                                    MECHANICAL_ARM_POSITION_RELATIVE_CURRENT);
  if (Result != MECHANICAL_ARM_RESULT_NONE)
  {
    ArmVision_ErrorInfoSet(Axis, 0U, 0U);
    ArmVision_MotorResultErrorSet(Result, ARM_VISION_ERROR_MOTOR_TX);
    ArmVision_StateSet(ARM_VISION_STATE_ERROR);
    return 0U;
  }
  ArmVision_MoveAxis = Axis;
  ArmVision_StateSet(ARM_VISION_STATE_WAIT_POSITION_ACK);
  return 1U;
}

/**
  * 函    数：结束一次标定或对准任务
  * 参    数：State ALIGNED或ERROR
  * 返 回 值：无
  * 说    明：停止接受当前摄像头目标，但不自动移动Z轴和夹爪
  */
static void ArmVision_TaskFinish(ArmVision_StateTypeDef State)
{
  if (State == ARM_VISION_STATE_ALIGNED)
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  }
  else if ((State == ARM_VISION_STATE_ERROR) &&
           (ArmVision_Error == ARM_VISION_ERROR_NONE))
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_INTERNAL);
  }
  if (State == ARM_VISION_STATE_ERROR)
  {
    (void)MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL);
  }
  Camera_RequestStop();
  ArmVision_StateSet(State);
}

/**
  * 函    数：检查二维标定矩阵是否可逆且响应足够明显
  * 参    数：无
  * 返 回 值：1表示可用于对准，0表示标定无效
  * 说    明：单轴像素变化至少4，两个响应向量夹角正弦不小于0.2
  */
static uint8_t ArmVision_CalibrationCheck(void)
{
  float BaseNorm;
  float XNorm;
  float Det;
  float BaseResponseSquare;
  float XResponseSquare;

  BaseNorm = ArmVision_J11 * ArmVision_J11 + ArmVision_J21 * ArmVision_J21;
  XNorm = ArmVision_J12 * ArmVision_J12 + ArmVision_J22 * ArmVision_J22;
  Det = ArmVision_J11 * ArmVision_J22 - ArmVision_J12 * ArmVision_J21;
  ArmVision_CalibrationDeterminant = Det;
  BaseResponseSquare = (float)ArmVision_BaseDx * ArmVision_BaseDx +
                       (float)ArmVision_BaseDy * ArmVision_BaseDy;
  XResponseSquare = (float)ArmVision_XDx * ArmVision_XDx +
                    (float)ArmVision_XDy * ArmVision_XDy;
  if ((BaseResponseSquare <
       (ARM_VISION_MIN_CALIBRATION_PIXEL_RESPONSE *
        ARM_VISION_MIN_CALIBRATION_PIXEL_RESPONSE)) ||
      (XResponseSquare <
       (ARM_VISION_MIN_CALIBRATION_PIXEL_RESPONSE *
        ARM_VISION_MIN_CALIBRATION_PIXEL_RESPONSE)))
  {
    return 0U;
  }
  if ((Det * Det) < (0.04f * BaseNorm * XNorm))
  {
    return 0U;
  }
  return 1U;
}

/**
  * 函    数：计算并启动下一次单轴对准修正
  * 参    数：Dx、Dy 最近5帧中值误差
  * 返 回 值：无
  * 说    明：每轮只移动归一化修正量较大的一个轴
  */
static void ArmVision_CorrectionStart(int16_t Dx, int16_t Dy)
{
  float Det;
  float Gain;
  float BaseFloat;
  float XFloat;
  int32_t BasePulse;
  int32_t XPulse;
  int32_t AbsDx;
  int32_t AbsDy;

  AbsDx = (Dx < 0) ? -(int32_t)Dx : (int32_t)Dx;
  AbsDy = (Dy < 0) ? -(int32_t)Dy : (int32_t)Dy;
  if ((AbsDx <= ARM_VISION_ALIGNMENT_DEAD_ZONE_PIXELS) &&
      (AbsDy <= ARM_VISION_ALIGNMENT_DEAD_ZONE_PIXELS))
  {
    ArmVision_StableCount++;
    if (ArmVision_StableCount >= ARM_VISION_STABLE_COUNT)
    {
      ArmVision_TaskFinish(ARM_VISION_STATE_ALIGNED);
    }
    else
    {
      ArmVision_CollectStart(ARM_VISION_PHASE_ALIGN);
    }
    return;
  }
  ArmVision_StableCount = 0U;
  Det = ArmVision_J11 * ArmVision_J22 - ArmVision_J12 * ArmVision_J21;
  Gain = ((AbsDx > ARM_VISION_LARGE_ERROR_PIXELS) ||
          (AbsDy > ARM_VISION_LARGE_ERROR_PIXELS)) ?
         ARM_VISION_LARGE_ERROR_GAIN : ARM_VISION_SMALL_ERROR_GAIN;
  BaseFloat = -Gain * (ArmVision_J22 * (float)Dx -
                       ArmVision_J12 * (float)Dy) / Det;
  XFloat = -Gain * (-ArmVision_J21 * (float)Dx +
                    ArmVision_J11 * (float)Dy) / Det;
  BasePulse = (int32_t)(BaseFloat + ((BaseFloat >= 0.0f) ? 0.5f : -0.5f));
  XPulse = (int32_t)(XFloat + ((XFloat >= 0.0f) ? 0.5f : -0.5f));
  if ((BasePulse > -ARM_VISION_MIN_CORRECTION_PULSES) &&
      (BasePulse < ARM_VISION_MIN_CORRECTION_PULSES))
    BasePulse = 0;
  if ((XPulse > -ARM_VISION_MIN_CORRECTION_PULSES) &&
      (XPulse < ARM_VISION_MIN_CORRECTION_PULSES))
    XPulse = 0;

  ArmVision_CorrectionCount++;
  ArmVision_Phase = ARM_VISION_PHASE_ALIGN;
  if ((BasePulse != 0) &&
      ((XPulse == 0) ||
       ((BasePulse < 0 ? -BasePulse : BasePulse) >=
        (XPulse < 0 ? -XPulse : XPulse))))
  {
    ArmVision_BaseTotal += BasePulse;
    if (ArmVision_MoveStart(MECHANICAL_ARM_AXIS_BASE, BasePulse) != 0U)
    {
      ArmVision_MoveDebugData.Axis = MECHANICAL_ARM_AXIS_BASE;
      ArmVision_MoveDebugData.Pulses = BasePulse;
      ArmVision_MoveDebugData.Dx = Dx;
      ArmVision_MoveDebugData.Dy = Dy;
      ArmVision_MoveDebugData.CorrectionCount = ArmVision_CorrectionCount;
      ArmVision_MoveDebugReady = 1U;
    }
  }
  else if (XPulse != 0)
  {
    ArmVision_XTotal += XPulse;
    if (ArmVision_MoveStart(MECHANICAL_ARM_AXIS_X, XPulse) != 0U)
    {
      ArmVision_MoveDebugData.Axis = MECHANICAL_ARM_AXIS_X;
      ArmVision_MoveDebugData.Pulses = XPulse;
      ArmVision_MoveDebugData.Dx = Dx;
      ArmVision_MoveDebugData.Dy = Dy;
      ArmVision_MoveDebugData.CorrectionCount = ArmVision_CorrectionCount;
      ArmVision_MoveDebugReady = 1U;
    }
  }
  else
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_CORRECTION_ZERO);
    ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
  }
}

/**
  * 函    数：处理一组5帧中值结果
  * 参    数：Dx、Dy 误差中值
  * 返 回 值：无
  * 说    明：按当前标定阶段或对准阶段推进状态机
  */
static void ArmVision_SampleResultProcess(int16_t Dx, int16_t Dy)
{
  switch (ArmVision_Phase)
  {
    case ARM_VISION_PHASE_CAL_BASE_ZERO:
      ArmVision_ZeroDx = Dx;
      ArmVision_ZeroDy = Dy;
      ArmVision_Phase = ARM_VISION_PHASE_CAL_BASE_OUT;
      (void)ArmVision_MoveStart(MECHANICAL_ARM_AXIS_BASE,
                                ARM_VISION_BASE_CALIBRATION_PULSES);
      break;

    case ARM_VISION_PHASE_CAL_BASE_OUT:
      ArmVision_BaseDx = (int16_t)(Dx - ArmVision_ZeroDx);
      ArmVision_BaseDy = (int16_t)(Dy - ArmVision_ZeroDy);
      ArmVision_J11 = (float)ArmVision_BaseDx /
                      (float)ARM_VISION_BASE_CALIBRATION_PULSES;
      ArmVision_J21 = (float)ArmVision_BaseDy /
                      (float)ARM_VISION_BASE_CALIBRATION_PULSES;
      ArmVision_Phase = ARM_VISION_PHASE_CAL_BASE_RETURN;
      (void)ArmVision_MoveStart(MECHANICAL_ARM_AXIS_BASE,
                                -ARM_VISION_BASE_CALIBRATION_PULSES);
      break;

    case ARM_VISION_PHASE_CAL_X_ZERO:
      ArmVision_ZeroDx = Dx;
      ArmVision_ZeroDy = Dy;
      ArmVision_Phase = ARM_VISION_PHASE_CAL_X_OUT;
      (void)ArmVision_MoveStart(
          MECHANICAL_ARM_AXIS_X, ARM_VISION_X_CALIBRATION_FORWARD_PULSES);
      break;

    case ARM_VISION_PHASE_CAL_X_OUT:
      ArmVision_XDx = (int16_t)(Dx - ArmVision_ZeroDx);
      ArmVision_XDy = (int16_t)(Dy - ArmVision_ZeroDy);
      ArmVision_J12 = (float)ArmVision_XDx /
                      (float)ARM_VISION_X_CALIBRATION_FORWARD_PULSES;
      ArmVision_J22 = (float)ArmVision_XDy /
                      (float)ARM_VISION_X_CALIBRATION_FORWARD_PULSES;
      ArmVision_Phase = ARM_VISION_PHASE_CAL_X_RETURN;
      (void)ArmVision_MoveStart(
          MECHANICAL_ARM_AXIS_X, -ARM_VISION_X_CALIBRATION_FORWARD_PULSES);
      break;

    case ARM_VISION_PHASE_ALIGN:
      ArmVision_CorrectionStart(Dx, Dy);
      break;

    default:
      ArmVision_ErrorSet(ARM_VISION_ERROR_INTERNAL);
      ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
      break;
  }
}

/**
  * 函    数：初始化机械臂视觉联调状态
  * 参    数：无
  * 返 回 值：无
  * 说    明：上电后机械参考和二维标定均默认无效
  */
void ArmVision_Init(void)
{
  (void)memset(&ArmVision_LatestData, 0, sizeof(ArmVision_LatestData));
  (void)memset(ArmVision_DxSamples, 0, sizeof(ArmVision_DxSamples));
  (void)memset(ArmVision_DySamples, 0, sizeof(ArmVision_DySamples));
  (void)memset(&ArmVision_MoveDebugData, 0, sizeof(ArmVision_MoveDebugData));
  ArmVision_State = ARM_VISION_STATE_IDLE;
  ArmVision_Phase = ARM_VISION_PHASE_ALIGN;
  ArmVision_MoveAxis = MECHANICAL_ARM_AXIS_INVALID;
  ArmVision_ReferenceValid = 0U;
  ArmVision_CalibrationValid = 0U;
  ArmVision_CalibrationSource = ARM_VISION_CALIBRATION_NONE;
  ArmVision_CalibrationTarget = 0U;
  ArmVision_CalibrationResultReady = 0U;
  ArmVision_Error = ARM_VISION_ERROR_NONE;
  ArmVision_CalibrationDeterminant = 0.0f;
  ArmVision_ErrorAxis = MECHANICAL_ARM_AXIS_INVALID;
  ArmVision_ErrorMotorCode = 0U;
  ArmVision_ErrorStateFlags = 0U;
  ArmVision_MoveDebugReady = 0U;
  ArmVision_SampleIndex = 0U;
  ArmVision_BaseDx = 0;
  ArmVision_BaseDy = 0;
  ArmVision_XDx = 0;
  ArmVision_XDy = 0;
  ArmVision_ZeroDx = 0;
  ArmVision_ZeroDy = 0;
  ArmVision_BaseTotal = 0;
  ArmVision_XTotal = 0;
  ArmVision_CorrectionCount = 0U;
  ArmVision_StableCount = 0U;
  ArmVision_HasCameraData = 0U;
  ArmVision_StartTick = 0U;
  ArmVision_LastDataTick = 0U;
  ArmVision_MotionStartTick = 0U;
  ArmVision_LastPollTick = 0U;
  ArmVision_SettleStartTick = 0U;
  ArmVision_J11 = 0.0f;
  ArmVision_J12 = 0.0f;
  ArmVision_J21 = 0.0f;
  ArmVision_J22 = 0.0f;
  ArmVision_LatestDataReady = 0U;
  ArmVision_StatusSequence = 1U;
  ArmVision_LastCameraSequence = 0U;
}

/**
  * 函    数：确认本次上电的机械参考姿态
  * 参    数：无
  * 返 回 值：视觉任务接收结果
  * 说    明：应先手动到达安全观察位并执行三轴zero命令
  */
ArmVision_ResultTypeDef ArmVision_ReferenceSet(void)
{
  if ((ArmVision_IsBusy() != 0U) || (MaterialVision_IsBusy() != 0U) ||
      (MechanicalArm_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy())
  {
    return ARM_VISION_RESULT_BUSY;
  }
  ArmVision_ReferenceValid = 1U;
  ArmVision_CalibrationValid = 0U;
  ArmVision_CalibrationSource = ARM_VISION_CALIBRATION_NONE;
  ArmVision_CalibrationTarget = 0U;
  ArmVision_CalibrationResultReady = 0U;
  ArmVision_ErrorInfoSet(MECHANICAL_ARM_AXIS_INVALID, 0U, 0U);
  ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  ArmVision_StatusSequence++;
  return ARM_VISION_RESULT_OK;
}

/**
  * 函    数：启动B3二维自动标定
  * 参    数：Ring 固定圆环编号
  * 返 回 值：视觉任务接收结果
  * 说    明：运行前Z轴必须位于观察高度，Base和X周围需保留安全行程
  */
ArmVision_ResultTypeDef ArmVision_CalibrationStart(Camera_RingTypeDef Ring)
{
  if ((ArmVision_IsBusy() != 0U) || (MaterialVision_IsBusy() != 0U) ||
      (MechanicalArm_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return ARM_VISION_RESULT_BUSY;
  if (ArmVision_ReferenceValid == 0U) return ARM_VISION_RESULT_NOT_READY;
  if ((Ring < CAMERA_RING_1) || (Ring > CAMERA_RING_3))
    return ARM_VISION_RESULT_PARAM_ERROR;
  if (!ArmVision_CameraRequestAccepted(Camera_RingStart(Ring)))
  {
    return ARM_VISION_RESULT_ERROR;
  }

  ArmVision_CalibrationValid = 0U;
  ArmVision_CalibrationResultReady = 0U;
  ArmVision_ErrorInfoSet(MECHANICAL_ARM_AXIS_INVALID, 0U, 0U);
  ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  ArmVision_CalibrationSource = ARM_VISION_CALIBRATION_RING;
  ArmVision_CalibrationTarget = (uint8_t)Ring;
  ArmVision_StartTick = HAL_GetTick();
  ArmVision_LastDataTick = ArmVision_StartTick;
  ArmVision_HasCameraData = 0U;
  ArmVision_CollectStart(ARM_VISION_PHASE_CAL_BASE_ZERO);
  return ARM_VISION_RESULT_OK;
}

/**
  * 函    数：启动B2物料二维自动标定
  * 参    数：Color 物料颜色
  * 返 回 值：视觉任务接收结果
  * 说    明：使用固定观察高度的静止物料测量Base/X对像素误差的影响；
  *           不要求先执行圆环识别，但必须先确认安全参考姿态
  */
ArmVision_ResultTypeDef ArmVision_MaterialCalibrationStart(
    Camera_ColorTypeDef Color)
{
  if ((ArmVision_IsBusy() != 0U) || (MaterialVision_IsBusy() != 0U) ||
      (MechanicalArm_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return ARM_VISION_RESULT_BUSY;
  if (ArmVision_ReferenceValid == 0U) return ARM_VISION_RESULT_NOT_READY;
  if ((Color < CAMERA_COLOR_RED) || (Color > CAMERA_COLOR_LIGHT_BLUE))
    return ARM_VISION_RESULT_PARAM_ERROR;
  if (!ArmVision_CameraRequestAccepted(Camera_MaterialStart(Color)))
  {
    return ARM_VISION_RESULT_ERROR;
  }

  ArmVision_CalibrationValid = 0U;
  ArmVision_CalibrationResultReady = 0U;
  ArmVision_ErrorInfoSet(MECHANICAL_ARM_AXIS_INVALID, 0U, 0U);
  ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  ArmVision_CalibrationSource = ARM_VISION_CALIBRATION_MATERIAL;
  ArmVision_CalibrationTarget = (uint8_t)Color;
  ArmVision_StartTick = HAL_GetTick();
  ArmVision_LastDataTick = ArmVision_StartTick;
  ArmVision_HasCameraData = 0U;
  ArmVision_CollectStart(ARM_VISION_PHASE_CAL_BASE_ZERO);
  return ARM_VISION_RESULT_OK;
}

/**
  * 函    数：启动圆环视觉任务
  * 参    数：Ring 圆环编号；Job 任务类型
  * 返 回 值：视觉任务接收结果
  * 说    明：第一阶段只开放ALIGN_ONLY，不执行Z下降和松爪
  */
ArmVision_ResultTypeDef ArmVision_RingStart(Camera_RingTypeDef Ring,
                                            ArmVision_JobTypeDef Job)
{
  if ((ArmVision_IsBusy() != 0U) || (MaterialVision_IsBusy() != 0U) ||
      (MechanicalArm_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return ARM_VISION_RESULT_BUSY;
  if ((ArmVision_ReferenceValid == 0U) || (ArmVision_CalibrationValid == 0U))
    return ARM_VISION_RESULT_NOT_READY;
  if ((Ring < CAMERA_RING_1) || (Ring > CAMERA_RING_3))
    return ARM_VISION_RESULT_PARAM_ERROR;
  if (Job != ARM_VISION_JOB_ALIGN_ONLY) return ARM_VISION_RESULT_NOT_READY;
  if (!ArmVision_CameraRequestAccepted(Camera_RingStart(Ring)))
  {
    return ARM_VISION_RESULT_ERROR;
  }

  ArmVision_ErrorInfoSet(MECHANICAL_ARM_AXIS_INVALID, 0U, 0U);
  ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  ArmVision_AlignStart();
  return ARM_VISION_RESULT_OK;
}

/**
  * 函    数：启动物料视觉任务
  * 参    数：Color 物料颜色；Job 任务类型
  * 返 回 值：视觉任务接收结果
  * 说    明：vision material用于自动对准，第一阶段只开放ALIGN_ONLY，
  *           不执行Z下降和夹取；独立协议测试使用camera material
  */
ArmVision_ResultTypeDef ArmVision_MaterialStart(Camera_ColorTypeDef Color,
                                                 ArmVision_JobTypeDef Job)
{
  if ((ArmVision_IsBusy() != 0U) || (MaterialVision_IsBusy() != 0U) ||
      (MechanicalArm_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return ARM_VISION_RESULT_BUSY;
  if ((ArmVision_ReferenceValid == 0U) || (ArmVision_CalibrationValid == 0U))
    return ARM_VISION_RESULT_NOT_READY;
  if ((Color < CAMERA_COLOR_RED) || (Color > CAMERA_COLOR_LIGHT_BLUE))
    return ARM_VISION_RESULT_PARAM_ERROR;
  if (Job != ARM_VISION_JOB_ALIGN_ONLY) return ARM_VISION_RESULT_NOT_READY;
  if (!ArmVision_CameraRequestAccepted(Camera_MaterialStart(Color)))
  {
    return ARM_VISION_RESULT_ERROR;
  }

  ArmVision_ErrorInfoSet(MECHANICAL_ARM_AXIS_INVALID, 0U, 0U);
  ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  ArmVision_AlignStart();
  return ARM_VISION_RESULT_OK;
}

/**
  * 函    数：推进机械臂视觉状态机
  * 参    数：无
  * 返 回 值：无
  * 说    明：主循环高频调用，不使用阻塞延时
  */
void ArmVision_Process(void)
{
  Camera_DataTypeDef Data;
  Camera_SnapshotTypeDef Snapshot;
  uint32_t Now;
  int16_t Dx;
  int16_t Dy;

  Now = HAL_GetTick();
  Camera_SnapshotGet(&Snapshot);
  if ((Snapshot.RequestActive != 0U) && (Snapshot.UsbConfigured != 0U) &&
      (Snapshot.TargetValid != 0U) && (Snapshot.HasValidData != 0U) &&
      (Snapshot.Data.Sequence != ArmVision_LastCameraSequence))
  {
    Data = Snapshot.Data;
    ArmVision_LastCameraSequence = Data.Sequence;
    ArmVision_LatestData = Data;
    ArmVision_LatestDataReady = 1U;
    ArmVision_LastDataTick = Data.Tick;
    ArmVision_HasCameraData = 1U;
    if (ArmVision_State == ARM_VISION_STATE_COLLECT)
    {
      ArmVision_DxSamples[ArmVision_SampleIndex] = Data.DX;
      ArmVision_DySamples[ArmVision_SampleIndex] = Data.DY;
      ArmVision_SampleIndex++;
      if (ArmVision_SampleIndex >= ARM_VISION_SAMPLE_COUNT)
      {
        Dx = ArmVision_MedianGet(ArmVision_DxSamples);
        Dy = ArmVision_MedianGet(ArmVision_DySamples);
        ArmVision_SampleIndex = 0U;
        ArmVision_SampleResultProcess(Dx, Dy);
      }
    }
  }

  if ((ArmVision_State == ARM_VISION_STATE_IDLE) ||
      (ArmVision_State == ARM_VISION_STATE_ALIGNED) ||
      (ArmVision_State == ARM_VISION_STATE_ERROR))
  {
    return;
  }
  if ((ArmVision_HasCameraData == 0U) &&
      ((Now - ArmVision_StartTick) > ARM_VISION_FIRST_DATA_TIMEOUT_MS))
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_CAMERA_FIRST_TIMEOUT);
    ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
    return;
  }
  if ((ArmVision_HasCameraData != 0U) &&
      ((Now - ArmVision_LastDataTick) > ARM_VISION_DATA_STALE_TIMEOUT_MS))
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_CAMERA_STALE);
    ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
    return;
  }
  if (((ArmVision_Phase == ARM_VISION_PHASE_ALIGN) &&
       ((Now - ArmVision_StartTick) > ARM_VISION_ALIGN_TIMEOUT_MS)) ||
      ((ArmVision_Phase != ARM_VISION_PHASE_ALIGN) &&
       ((Now - ArmVision_StartTick) > ARM_VISION_CALIBRATION_TIMEOUT_MS)))
  {
    ArmVision_ErrorSet(ARM_VISION_ERROR_VISION_TIMEOUT);
    ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
    return;
  }

  if (ArmVision_State == ARM_VISION_STATE_WAIT_ARRIVAL)
  {
    if ((Now - ArmVision_MotionStartTick) > ARM_VISION_MOTOR_ARRIVAL_TIMEOUT_MS)
    {
      ArmVision_ErrorInfoSet(ArmVision_MoveAxis, 0U, 0U);
      ArmVision_ErrorSet(ARM_VISION_ERROR_MOTOR_ARRIVAL_TIMEOUT);
      ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
    }
    else if (((Now - ArmVision_LastPollTick) >= 50U) &&
             (MechanicalArm_IsBusy() == 0U))
    {
      ArmVision_LastPollTick = Now;
      if (MechanicalArm_StateRead(ArmVision_MoveAxis) == MECHANICAL_ARM_RESULT_NONE)
      {
        ArmVision_StateSet(ARM_VISION_STATE_WAIT_STATE_ACK);
      }
    }
  }
  else if ((ArmVision_State == ARM_VISION_STATE_SETTLE) &&
           ((Now - ArmVision_SettleStartTick) >= ARM_VISION_CALIBRATION_SETTLE_MS))
  {
    if (ArmVision_Phase == ARM_VISION_PHASE_CAL_BASE_RETURN)
    {
      ArmVision_CollectStart(ARM_VISION_PHASE_CAL_X_ZERO);
    }
    else if (ArmVision_Phase == ARM_VISION_PHASE_CAL_X_RETURN)
    {
      if (ArmVision_CalibrationCheck() != 0U)
      {
        ArmVision_CalibrationValid = 1U;
        ArmVision_CalibrationResultReady = 1U;
        ArmVision_TaskFinish(ARM_VISION_STATE_ALIGNED);
      }
      else
      {
        ArmVision_ErrorSet(ARM_VISION_ERROR_MATRIX_INVALID);
        ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
      }
    }
    else
    {
      ArmVision_CollectStart(ArmVision_Phase);
    }
  }
}

/**
  * 函    数：停止当前视觉任务
  * 参    数：无
  * 返 回 值：无
  * 说    明：活动电机命令使用广播stop all终止，摄像头请求同时清除
  */
void ArmVision_Stop(void)
{
  if (ArmVision_IsBusy() != 0U)
  {
    (void)MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL);
  }
  Camera_RequestStop();
  ArmVision_ErrorInfoSet(MECHANICAL_ARM_AXIS_INVALID, 0U, 0U);
  ArmVision_ErrorSet(ARM_VISION_ERROR_NONE);
  ArmVision_StateSet(ARM_VISION_STATE_IDLE);
}

/**
  * 函    数：处理视觉任务所属的电机事件
  * 参    数：Event MechanicalArm完成事件
  * 返 回 值：1表示事件已消费，0表示应交给控制台
  * 说    明：位置ACK后轮询S_FLAG，到位才进入150ms稳定等待
  */
uint8_t ArmVision_MotorEventHandle(const MechanicalArm_EventTypeDef *Event)
{
  uint8_t Flags;

  if (Event == NULL) return 0U;
  if (ArmVision_State == ARM_VISION_STATE_WAIT_POSITION_ACK)
  {
    if ((Event->Action != MECHANICAL_ARM_ACTION_POSITION) ||
        (Event->Axis != ArmVision_MoveAxis)) return 0U;
    if (Event->Result != MECHANICAL_ARM_RESULT_OK)
    {
      ArmVision_ErrorInfoSet(Event->Axis, Event->MotorCode, 0U);
      ArmVision_MotorResultErrorSet(
          Event->Result, ARM_VISION_ERROR_MOTOR_POSITION_ACK_TIMEOUT);
      ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
    }
    else
    {
      ArmVision_MotionStartTick = HAL_GetTick();
      ArmVision_LastPollTick = 0U;
      ArmVision_StateSet(ARM_VISION_STATE_WAIT_ARRIVAL);
    }
    return 1U;
  }
  if (ArmVision_State == ARM_VISION_STATE_WAIT_STATE_ACK)
  {
    if ((Event->Action != MECHANICAL_ARM_ACTION_STATE) ||
        (Event->Axis != ArmVision_MoveAxis)) return 0U;
    if (Event->Result != MECHANICAL_ARM_RESULT_OK)
    {
      ArmVision_ErrorInfoSet(Event->Axis, Event->MotorCode, 0U);
      ArmVision_MotorResultErrorSet(
          Event->Result, ARM_VISION_ERROR_MOTOR_STATE_ACK_TIMEOUT);
      ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
      return 1U;
    }
    Flags = Event->StateFlags[(uint8_t)ArmVision_MoveAxis];
    if ((Flags & 0x0CU) != 0U)
    {
      ArmVision_ErrorInfoSet(ArmVision_MoveAxis, Event->MotorCode, Flags);
      ArmVision_ErrorSet(ARM_VISION_ERROR_MOTOR_ERROR);
      ArmVision_TaskFinish(ARM_VISION_STATE_ERROR);
    }
    else if ((Flags & 0x02U) != 0U)
    {
      ArmVision_SettleStartTick = HAL_GetTick();
      ArmVision_StateSet(ARM_VISION_STATE_SETTLE);
    }
    else
    {
      ArmVision_StateSet(ARM_VISION_STATE_WAIT_ARRIVAL);
    }
    return 1U;
  }
  return 0U;
}

/**
  * 函    数：查询视觉任务是否正在运行
  * 参    数：无
  * 返 回 值：1运行中，0空闲或已结束
  * 说    明：ALIGNED和ERROR允许操作者读取状态并启动下一任务
  */
uint8_t ArmVision_IsBusy(void)
{
  return ((ArmVision_State != ARM_VISION_STATE_IDLE) &&
          (ArmVision_State != ARM_VISION_STATE_ALIGNED) &&
          (ArmVision_State != ARM_VISION_STATE_ERROR)) ? 1U : 0U;
}

/**
  * 函    数：查询本次上电机械参考是否已确认
  * 参    数：无
  * 返 回 值：1已确认，0未确认
  * 说    明：vision ref只表示操作者确认了安全观察姿态
  */
uint8_t ArmVision_IsReferenceValid(void)
{
  return ArmVision_ReferenceValid;
}

/**
  * 函    数：查询二维标定是否有效
  * 参    数：无
  * 返 回 值：1有效，0无效
  * 说    明：重新确认机械参考后原标定自动失效
  */
uint8_t ArmVision_IsCalibrated(void)
{
  return ArmVision_CalibrationValid;
}

/**
  * 函    数：查询二维标定数据来源
  * 参    数：无
  * 返 回 值：圆环、物料或无有效标定来源
  * 说    明：标定矩阵本身与目标颜色无关，仅用于控制台状态显示
  */
ArmVision_CalibrationSourceTypeDef ArmVision_CalibrationSourceGet(void)
{
  return ArmVision_CalibrationSource;
}

/**
  * 函    数：取得二维标定数据来源名称
  * 参    数：无
  * 返 回 值：ASCII来源名称
  * 说    明：供控制台直接显示none、ring或material
  */
const char *ArmVision_CalibrationSourceNameGet(void)
{
  static const char *Names[] =
  {
    "none", "ring", "material"
  };

  return Names[(uint8_t)ArmVision_CalibrationSource];
}

/**
  * 函    数：查询二维标定目标编号
  * 参    数：无
  * 返 回 值：圆环编号或物料颜色编号
  * 说    明：无有效标定时返回0
  */
uint8_t ArmVision_CalibrationTargetGet(void)
{
  return ArmVision_CalibrationTarget;
}

/**
  * 函    数：读取一次二维标定结果
  * 参    数：Data 标定结果输出地址
  * 返 回 值：1有新的标定结果，0暂无新的标定结果
  * 说    明：结果只在标定成功后发布一次，Base步长为80脉冲，X步长为160脉冲
  */
uint8_t ArmVision_CalibrationResultGet(ArmVision_CalibrationDataTypeDef *Data)
{
  if ((Data == NULL) || (ArmVision_CalibrationResultReady == 0U))
  {
    return 0U;
  }
  Data->BasePulses = (uint16_t)ARM_VISION_BASE_CALIBRATION_PULSES;
  Data->BaseDx = ArmVision_BaseDx;
  Data->BaseDy = ArmVision_BaseDy;
  Data->XPulses = (uint16_t)((ARM_VISION_X_CALIBRATION_FORWARD_PULSES < 0) ?
                             -ARM_VISION_X_CALIBRATION_FORWARD_PULSES :
                             ARM_VISION_X_CALIBRATION_FORWARD_PULSES);
  Data->XDx = ArmVision_XDx;
  Data->XDy = ArmVision_XDy;
  ArmVision_CalibrationResultReady = 0U;
  return 1U;
}

/**
  * 函    数：读取标定矩阵诊断数据
  * 参    数：Data 单轴像素变化和行列式输出地址
  * 返 回 值：1表示参数有效，0表示地址为空
  * 说    明：矩阵无效时用于区分响应过小和两个轴方向接近平行
  */
uint8_t ArmVision_CalibrationDebugGet(
    ArmVision_CalibrationDebugDataTypeDef *Data)
{
  if (Data == NULL)
  {
    return 0U;
  }
  Data->BaseDx = ArmVision_BaseDx;
  Data->BaseDy = ArmVision_BaseDy;
  Data->XDx = ArmVision_XDx;
  Data->XDy = ArmVision_XDy;
  Data->Determinant = ArmVision_CalibrationDeterminant;
  return 1U;
}

/**
  * 函    数：取得视觉任务错误编号
  * 参    数：无
  * 返 回 值：当前错误枚举值
  * 说    明：ERROR状态下用于进一步区分摄像头、电机和算法原因
  */
ArmVision_ErrorTypeDef ArmVision_ErrorGet(void)
{
  return ArmVision_Error;
}

/**
  * 函    数：读取视觉电机错误上下文
  * 参    数：Info 电机轴、错误码和原始状态输出地址
  * 返 回 值：1表示地址有效，0表示地址为空
  * 说    明：控制台使用该信息输出具体故障位置
  */
uint8_t ArmVision_ErrorInfoGet(ArmVision_ErrorInfoTypeDef *Info)
{
  if (Info == NULL)
  {
    return 0U;
  }
  Info->Axis = ArmVision_ErrorAxis;
  Info->MotorCode = ArmVision_ErrorMotorCode;
  Info->StateFlags = ArmVision_ErrorStateFlags;
  return 1U;
}

/**
  * 函    数：取得视觉任务错误名称
  * 参    数：无
  * 返 回 值：ASCII错误名称
  * 说    明：名称直接用于vision status，避免控制台输出中文乱码
  */
const char *ArmVision_ErrorNameGet(void)
{
  static const char *Names[] =
  {
    "none",
    "camera_first_timeout",
    "camera_stale",
    "camera_tx",
    "motor_tx",
    "motor_busy",
    "motor_param",
    "motor_position_ack_timeout",
    "motor_state_ack_timeout",
    "motor_error",
    "motor_arrival_timeout",
    "correction_limit",
    "correction_zero",
    "matrix_invalid",
    "vision_timeout",
    "internal",
    "camera_busy",
    "usb_off"
  };

  return Names[(uint8_t)ArmVision_Error];
}

/**
  * 函    数：取得当前视觉状态名称
  * 参    数：无
  * 返 回 值：ASCII状态常量字符串
  * 说    明：供控制台和OLED显示
  */
const char *ArmVision_StateNameGet(void)
{
  static const char *Names[] =
  {
    "IDLE", "COLLECT", "MOVE_ACK", "ARRIVAL",
    "STATE_ACK", "SETTLE", "ALIGNED", "ERROR"
  };
  return Names[(uint8_t)ArmVision_State];
}

/**
  * 函    数：取得视觉状态变化序号
  * 参    数：无
  * 返 回 值：单调递增序号
  * 说    明：显示层可据此避免重复刷新OLED
  */
uint32_t ArmVision_StatusSequenceGet(void)
{
  return ArmVision_StatusSequence;
}

/**
  * 函    数：读取视觉修正进度
  * 参    数：Data 修正次数、累计脉冲和最新误差输出地址
  * 返 回 值：1表示地址有效，0表示地址为空
  * 说    明：供控制台查询当前对准过程，不清除视觉数据
  */
uint8_t ArmVision_ProgressGet(ArmVision_ProgressTypeDef *Data)
{
  if (Data == NULL)
  {
    return 0U;
  }
  Data->CorrectionCount = ArmVision_CorrectionCount;
  Data->BaseTotal = ArmVision_BaseTotal;
  Data->XTotal = ArmVision_XTotal;
  Data->Dx = ArmVision_LatestData.DX;
  Data->Dy = ArmVision_LatestData.DY;
  return 1U;
}

/**
  * 函    数：读取并清除一次视觉运动调试记录
  * 参    数：Data 实际运动轴、脉冲和误差输出地址
  * 返 回 值：1取得新记录，0暂无新记录或地址为空
  * 说    明：每次成功启动Base或X修正后发布一条记录
  */
uint8_t ArmVision_MoveDebugGet(ArmVision_MoveDebugDataTypeDef *Data)
{
  if ((Data == NULL) || (ArmVision_MoveDebugReady == 0U))
  {
    return 0U;
  }
  *Data = ArmVision_MoveDebugData;
  ArmVision_MoveDebugReady = 0U;
  return 1U;
}

/**
  * 函    数：读取最近一次视觉坐标
  * 参    数：Data 数据输出地址
  * 返 回 值：1取得新数据，0暂无新数据
  * 说    明：不影响ArmVision内部已经使用的中值样本
  */
uint8_t ArmVision_DataGet(Camera_DataTypeDef *Data)
{
  if ((Data == NULL) || (ArmVision_LatestDataReady == 0U))
  {
    return 0U;
  }
  *Data = ArmVision_LatestData;
  ArmVision_LatestDataReady = 0U;
  return 1U;
}

/**
  * 函    数：读取最近一次视觉坐标但不清除标志
  * 参    数：Data 数据输出地址
  * 返 回 值：1表示已有坐标，0表示尚未收到坐标
  * 说    明：供控制台调试输出使用，不影响OLED对新数据的读取
  */
uint8_t ArmVision_DataPeek(Camera_DataTypeDef *Data)
{
  if ((Data == NULL) || (ArmVision_LatestData.Sequence == 0U))
  {
    return 0U;
  }
  *Data = ArmVision_LatestData;
  return 1U;
}

/** 当前是否正在自动标定；只读，不推进任务。 */
uint8_t ArmVision_IsCalibrating(void)
{
  return (ArmVision_IsBusy() && ArmVision_Phase != ARM_VISION_PHASE_ALIGN) ? 1U : 0U;
}
