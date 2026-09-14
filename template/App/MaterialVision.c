#include "MaterialVision.h"
#include "chassis_route.h"
#include "chassis_motion.h"

#include "ArmVision.h"
#include "mecanum_chassis.h"
#include "mechanical_arm_config.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MATERIAL_VISION_TEST_NO_TIMEOUT 1U /* 测试阶段关闭视觉流程的高层自动超时。 */
#define MATERIAL_VISION_SAMPLE_COUNT 5U  /* 每组视觉中值采样的帧数。 */
#define MATERIAL_VISION_SEARCH_SPEED_MM_S 30.0f  /* 搜索物料时底盘向后速度。 */
#define MATERIAL_VISION_COARSE_SPEED_MM_S 15.0f  /* 底盘前后粗调速度。 */
#define MATERIAL_VISION_FINE_SPEED_MM_S 8.0f  /* 底盘前后精调速度。 */
#define MATERIAL_VISION_CHASSIS_CAL_SPEED_MM_S 20.0f  /* 底盘标定速度。 */
#define MATERIAL_VISION_CHASSIS_CAL_STEP_MM 20.0f  /* 底盘标定单次位移。 */
#define MATERIAL_VISION_CORRECTION_TIME_MS 200U /* 普通底盘修正持续时间。 */
#define MATERIAL_VISION_MIN_MOVE_TIME_MS 100U
#define MATERIAL_VISION_MOTOR_POLL_MS 50U
#define MATERIAL_VISION_X_ARRIVAL_TIMEOUT_MS 5000U
#define MATERIAL_VISION_X_SPEED_RPM 8U
#define MATERIAL_VISION_X_ACCELERATION 8U
#define MATERIAL_VISION_X_CAL_STEP_DEG 5.0f  /* X轴标定角度。 */
#define MATERIAL_VISION_LOCK_DX_PIXELS 80  /* 进入工作窗口的DX阈值。 */
#define MATERIAL_VISION_LOCK_DY_PIXELS 100  /* 进入工作窗口的DY阈值。 */
#define MATERIAL_VISION_ALIGN_DEAD_ZONE_PIXELS 1  /* 最终对准死区半宽。 */
#define MATERIAL_VISION_DX_HANDOFF_PIXELS 15     /* DX进入该窗口后交给X轴修正DY，不代表对准完成。 */
#define MATERIAL_VISION_STABLE_COUNT 3U  /* 连续满足死区的次数。 */
#define MATERIAL_VISION_FIRST_DATA_TIMEOUT_MS 3000U  /* 等待首帧的最长时间。 */
#define MATERIAL_VISION_DATA_STALE_TIMEOUT_MS 1200U /* 视觉数据失新的时间，容忍较慢的一帧。 */
#define MATERIAL_VISION_HOME_TIMEOUT_MS 8000U  /* Base回零最长时间。 */
#define MATERIAL_VISION_SEARCH_TIMEOUT_MS 10000U  /* 底盘搜索最长时间。 */
#define MATERIAL_VISION_TASK_TIMEOUT_MS 15000U  /* 自动对准最长时间。 */
#define MATERIAL_VISION_MOVE_SETTLE_MS 150U  /* 运动停止后的稳定时间。 */
#define MATERIAL_VISION_MIN_RESPONSE_PIXELS 3.0f  /* 标定响应的最小模长。 */
#define MATERIAL_VISION_MAX_X_TOTAL_PULSES 3200  /* X轴视觉累计修正安全范围。 */
#define MATERIAL_VISION_MAX_CHASSIS_TOTAL_MM 2000.0f  /* 底盘前后视觉累计位移范围。 */
#define MATERIAL_VISION_MIN_CHASSIS_STEP_MM 3.0f  /* 底盘最小修正位移。 */
#define MATERIAL_VISION_MIN_X_STEP_DEG 1.0f  /* X轴最小修正角度。 */
#define MATERIAL_VISION_RESPONSE_CROSS_LIMIT 0.2f  /* 两响应向量最小夹角正弦。 */
#define MATERIAL_VISION_PID_FORWARD_KP 0.20f  /* DX到前后速度的比例增益。 */
#define MATERIAL_VISION_PID_FORWARD_KI 0.00f  /* 前后速度积分增益，初始关闭。 */
#define MATERIAL_VISION_PID_FORWARD_KD 0.03f  /* DX变化的微分增益。 */
#define MATERIAL_VISION_PID_X_KP 0.08f  /* DY到X轴角度增益。 */
#define MATERIAL_VISION_PID_X_KI 0.00f  /* X轴积分增益，初始关闭。 */
#define MATERIAL_VISION_PID_X_KD 0.01f  /* DY变化的微分增益。 */
#define MATERIAL_VISION_PID_OUTPUT_LIMIT 30.0f  /* 底盘前后速度输出限幅。 */
#define MATERIAL_VISION_PID_X_OUTPUT_LIMIT 30.0f  /* X轴粗调单次角度输出上限，允许更快覆盖Y误差。 */
#define MATERIAL_VISION_PID_INTEGRAL_LIMIT 300.0f  /* PID积分项限幅。 */
#define MATERIAL_VISION_PID_FORWARD_SIGN 1.0f /* DX为负时底盘后退，需结合实测确认。 */
#define MATERIAL_VISION_PID_X_SIGN 1.0f   /* Positive DY now commands positive X correction. */

typedef enum
{
  MATERIAL_VISION_TASK_NONE = 0,
  MATERIAL_VISION_TASK_ALIGN,
  MATERIAL_VISION_TASK_CALIBRATION
} MaterialVision_TaskTypeDef;

typedef enum
{
  MATERIAL_VISION_PHASE_NONE = 0,
  MATERIAL_VISION_PHASE_ALIGN,
  MATERIAL_VISION_PHASE_CAL_BASELINE,
  MATERIAL_VISION_PHASE_CAL_FORWARD_SAMPLE,
  MATERIAL_VISION_PHASE_CAL_FORWARD_RETURN,
  MATERIAL_VISION_PHASE_CAL_X_BASELINE,
  MATERIAL_VISION_PHASE_CAL_X_SAMPLE,
  MATERIAL_VISION_PHASE_CAL_X_RETURN
} MaterialVision_PhaseTypeDef;

typedef struct
{
  float Dx;
  float Dy;
} MaterialVision_ResponseTypeDef;

static MaterialVision_StateTypeDef MaterialVision_State;
static MaterialVision_TaskTypeDef MaterialVision_Task;
static MaterialVision_PhaseTypeDef MaterialVision_Phase;
static Camera_ColorTypeDef MaterialVision_Color;
static Camera_DataTypeDef MaterialVision_Data;
static uint32_t MaterialVision_LastSequence;
static uint32_t MaterialVision_StartTick;
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
static uint32_t MaterialVision_LastDataTick;
#endif
static uint32_t MaterialVision_MoveStartTick;
static uint32_t MaterialVision_MoveDurationMs;
static uint32_t MaterialVision_SettleStartTick;
static uint32_t MaterialVision_LastPollTick;
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
static uint8_t MaterialVision_HasData;
#endif
static uint8_t MaterialVision_SampleIndex;
static uint8_t MaterialVision_LockCount;
static uint8_t MaterialVision_StableCount;
static int16_t MaterialVision_DxSamples[MATERIAL_VISION_SAMPLE_COUNT];
static int16_t MaterialVision_DySamples[MATERIAL_VISION_SAMPLE_COUNT];
static int16_t MaterialVision_BaselineDx;
static int16_t MaterialVision_BaselineDy;
static MaterialVision_ResponseTypeDef MaterialVision_Response[3];
static MaterialVision_CalibrationDataTypeDef MaterialVision_Calibration;
static uint8_t MaterialVision_Calibrated;
static int32_t MaterialVision_XTotal;
static float MaterialVision_ChassisTotal;
static MaterialVision_ErrorTypeDef MaterialVision_Error;
static float MaterialVision_ForwardIntegral;
static float MaterialVision_ForwardPreviousError;
static float MaterialVision_XIntegral;
static float MaterialVision_XPreviousError;

static const char *MaterialVision_ErrorNames[] =
{
  "none", "busy", "not_ready", "camera_first_timeout", "camera_stale",
  "camera_tx", "home_timeout", "motor_ack_timeout", "motor_arrival_timeout",
  "motor_error", "search_timeout", "response_invalid", "correction_zero",
  "x_limit", "task_timeout", "internal", "chassis_tx", "motor_busy",
  "motor_param", "motor_tx", "chassis_limit", "camera_busy", "usb_off"
};

/** 保留电机接口给出的真实失败原因，避免全部误报为超时。 */
static MaterialVision_ErrorTypeDef MaterialVision_MotorError(MechanicalArm_ResultTypeDef Result)
{
  switch (Result)
  {
    case MECHANICAL_ARM_RESULT_BUSY: return MATERIAL_VISION_ERROR_MOTOR_BUSY;
    case MECHANICAL_ARM_RESULT_PARAM_ERROR: return MATERIAL_VISION_ERROR_MOTOR_PARAM;
    case MECHANICAL_ARM_RESULT_TX_ERROR: return MATERIAL_VISION_ERROR_MOTOR_TX;
    case MECHANICAL_ARM_RESULT_ACK_TIMEOUT: return MATERIAL_VISION_ERROR_MOTOR_ACK_TIMEOUT;
    default: return MATERIAL_VISION_ERROR_MOTOR_ERROR;
  }
}

/** 区分USB未配置、发送忙和发送失败。 */
static MaterialVision_ErrorTypeDef MaterialVision_CameraError(HAL_StatusTypeDef Result)
{
  Camera_SnapshotTypeDef Snapshot;
  Camera_SnapshotGet(&Snapshot);
  if (Result == HAL_BUSY) return MATERIAL_VISION_ERROR_CAMERA_BUSY;
  if (Snapshot.UsbConfigured == 0U) return MATERIAL_VISION_ERROR_USB_OFF;
  return MATERIAL_VISION_ERROR_CAMERA_TX;
}

/** 计算固定五个样本的中值。 */
static int16_t MaterialVision_MedianGet(const int16_t *Data)
{
  int16_t Sorted[MATERIAL_VISION_SAMPLE_COUNT];
  int16_t Value;
  uint8_t Index;
  uint8_t Insert;

  (void)memcpy(Sorted, Data, sizeof(Sorted));
  for (Index = 1U; Index < MATERIAL_VISION_SAMPLE_COUNT; Index++)
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
  return Sorted[MATERIAL_VISION_SAMPLE_COUNT / 2U];
}

/** 读取一帧尚未处理的视觉数据。 */
static uint8_t MaterialVision_NewDataGet(Camera_DataTypeDef *Data)
{
  Camera_SnapshotTypeDef Snapshot;

  Camera_SnapshotGet(&Snapshot);
  if ((Data == NULL) || (Snapshot.RequestActive == 0U) ||
      (Snapshot.UsbConfigured == 0U) || (Snapshot.TargetValid == 0U) ||
      (Snapshot.HasValidData == 0U) ||
      (Snapshot.Data.Sequence == MaterialVision_LastSequence))
  {
    return 0U;
  }
  MaterialVision_LastSequence = Snapshot.Data.Sequence;
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  MaterialVision_LastDataTick = HAL_GetTick();
  MaterialVision_HasData = 1U;
#endif
  MaterialVision_Data = Snapshot.Data;
  *Data = Snapshot.Data;
  return 1U;
}

/** 将当前阶段切换为中值采样阶段。 */
static void MaterialVision_CollectStart(MaterialVision_PhaseTypeDef Phase)
{
  Camera_SnapshotTypeDef Snapshot;

  /* 稳定等待前已收到的帧不属于新采样组；只接收此后发布的序号。 */
  Camera_SnapshotGet(&Snapshot);
  MaterialVision_LastSequence = Snapshot.Data.Sequence;
  MaterialVision_Phase = Phase;
  MaterialVision_SampleIndex = 0U;
  MaterialVision_State = MATERIAL_VISION_STATE_COLLECT;
}

/** 根据当前运动类型进入一次稳定等待。 */
static void MaterialVision_SettleStart(void)
{
  MaterialVision_SettleStartTick = HAL_GetTick();
  MaterialVision_State = MATERIAL_VISION_STATE_SETTLE;
}

/** 停止全部视觉相关执行器并进入错误状态。 */
static void MaterialVision_Fail(MaterialVision_ErrorTypeDef Error)
{
  Camera_RequestStop();
  (void)Mecanum_Test_Stop();
  (void)MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL);
  MaterialVision_Error = Error;
  MaterialVision_State = MATERIAL_VISION_STATE_ERROR;
}

/** 启动底盘定时速度运动。 */
static uint8_t MaterialVision_ChassisMoveStart(float Forward,
                                                float Distance)
{
  float Speed;
  uint32_t Duration;

  Speed = (MaterialVision_Task == MATERIAL_VISION_TASK_CALIBRATION) ?
          MATERIAL_VISION_CHASSIS_CAL_SPEED_MM_S :
          (((fabsf((float)MaterialVision_Data.DX) > 30.0f) ||
            (fabsf((float)MaterialVision_Data.DY) > 30.0f)) ?
           MATERIAL_VISION_COARSE_SPEED_MM_S : MATERIAL_VISION_FINE_SPEED_MM_S);
  /* 视觉标定使用底盘前后运动，横向速度固定为0。 */
  if (Mecanum_Velocity_Start(Forward * Speed, 0.0f, 0.0f) == false)
  {
    MaterialVision_Error = MATERIAL_VISION_ERROR_CHASSIS_TX;
    return 0U;
  }
  Duration = (uint32_t)(fabsf(Distance) / Speed * 1000.0f);
  if (Duration < MATERIAL_VISION_MIN_MOVE_TIME_MS)
  {
    Duration = MATERIAL_VISION_MIN_MOVE_TIME_MS;
  }
  MaterialVision_MoveStartTick = HAL_GetTick();
  MaterialVision_MoveDurationMs = Duration;
  MaterialVision_State = MATERIAL_VISION_STATE_CHASSIS_MOVE;
  return 1U;
}

/** 启动一次底盘前后PID速度脉冲，左右速度始终保持为零。 */
static uint8_t MaterialVision_ForwardVelocityStart(float ForwardSpeed)
{
  if (Mecanum_Velocity_Start(ForwardSpeed, 0.0f, 0.0f) == false)
  {
    MaterialVision_Error = MATERIAL_VISION_ERROR_CHASSIS_TX;
    return 0U;
  }
  MaterialVision_MoveDurationMs = MATERIAL_VISION_CORRECTION_TIME_MS;
  MaterialVision_MoveStartTick = HAL_GetTick();
  MaterialVision_State = MATERIAL_VISION_STATE_CHASSIS_MOVE;
  return 1U;
}

/** 启动X轴相对角度修正。 */
static uint8_t MaterialVision_XMoveStart(float Degree)
{
  int32_t Pulses;
  MechanicalArm_ResultTypeDef Result;

  Pulses = (int32_t)(Degree * (float)MECHANICAL_ARM_COMMAND_PULSES_PER_REV /
                     MECHANICAL_ARM_DEGREES_PER_REV +
                     ((Degree >= 0.0f) ? 0.5f : -0.5f));
  if (Pulses == 0)
  {
    MaterialVision_Error = MATERIAL_VISION_ERROR_CORRECTION_ZERO;
    return 0U;
  }
  if ((MaterialVision_XTotal + Pulses > MATERIAL_VISION_MAX_X_TOTAL_PULSES) ||
      (MaterialVision_XTotal + Pulses < -MATERIAL_VISION_MAX_X_TOTAL_PULSES))
  {
    MaterialVision_Error = MATERIAL_VISION_ERROR_X_LIMIT;
    return 0U;
  }
  Result = MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_X, Pulses,
                                    MATERIAL_VISION_X_SPEED_RPM,
                                    MATERIAL_VISION_X_ACCELERATION,
                                    MECHANICAL_ARM_POSITION_RELATIVE_CURRENT);
  if (Result != MECHANICAL_ARM_RESULT_NONE)
  {
    MaterialVision_Error = MaterialVision_MotorError(Result);
    return 0U;
  }
  MaterialVision_XTotal += Pulses;
  MaterialVision_MoveStartTick = HAL_GetTick();
  MaterialVision_State = MATERIAL_VISION_STATE_X_ACK;
  return 1U;
}

/** 根据DX、DY运行一次简化PID并启动底盘或X轴修正。 */
static uint8_t MaterialVision_CorrectionStart(int16_t Dx, int16_t Dy)
{
  float ForwardOutput;
  float XOutput;

  /* DX较大时使用底盘前后速度；进入交接窗口后让X轴处理DY。 */
  if ((abs(Dx) > MATERIAL_VISION_DX_HANDOFF_PIXELS) ||
      ((abs(Dy) <= MATERIAL_VISION_ALIGN_DEAD_ZONE_PIXELS) &&
       (abs(Dx) > MATERIAL_VISION_ALIGN_DEAD_ZONE_PIXELS)))
  {
    MaterialVision_ForwardIntegral += (float)Dx;
    if (MaterialVision_ForwardIntegral > MATERIAL_VISION_PID_INTEGRAL_LIMIT)
      MaterialVision_ForwardIntegral = MATERIAL_VISION_PID_INTEGRAL_LIMIT;
    if (MaterialVision_ForwardIntegral < -MATERIAL_VISION_PID_INTEGRAL_LIMIT)
      MaterialVision_ForwardIntegral = -MATERIAL_VISION_PID_INTEGRAL_LIMIT;
    ForwardOutput = MATERIAL_VISION_PID_FORWARD_SIGN *
                    (MATERIAL_VISION_PID_FORWARD_KP * (float)Dx +
                     MATERIAL_VISION_PID_FORWARD_KI * MaterialVision_ForwardIntegral +
                     MATERIAL_VISION_PID_FORWARD_KD *
                     ((float)Dx - MaterialVision_ForwardPreviousError));
    MaterialVision_ForwardPreviousError = (float)Dx;
    if (ForwardOutput > MATERIAL_VISION_PID_OUTPUT_LIMIT)
      ForwardOutput = MATERIAL_VISION_PID_OUTPUT_LIMIT;
    if (ForwardOutput < -MATERIAL_VISION_PID_OUTPUT_LIMIT)
      ForwardOutput = -MATERIAL_VISION_PID_OUTPUT_LIMIT;
    if (MaterialVision_ChassisTotal + fabsf(ForwardOutput) * ((float)MATERIAL_VISION_CORRECTION_TIME_MS / 1000.0f) >
        MATERIAL_VISION_MAX_CHASSIS_TOTAL_MM)
    {
      MaterialVision_Error = MATERIAL_VISION_ERROR_CHASSIS_LIMIT;
      return 0U;
    }
    if (MaterialVision_ForwardVelocityStart(ForwardOutput) == 0U) return 0U;
    MaterialVision_ChassisTotal += fabsf(ForwardOutput) *
                                   ((float)MaterialVision_MoveDurationMs / 1000.0f);
    return 1U;
  }

  /* 窗口内先修正DY；DY达标而DX未达标时，上面的分支继续修正DX。 */
  if (fabsf((float)Dy) <= MATERIAL_VISION_ALIGN_DEAD_ZONE_PIXELS)
  {
    MaterialVision_Error = MATERIAL_VISION_ERROR_CORRECTION_ZERO;
    return 0U;
  }
  MaterialVision_XIntegral += (float)Dy;
  if (MaterialVision_XIntegral > MATERIAL_VISION_PID_INTEGRAL_LIMIT)
    MaterialVision_XIntegral = MATERIAL_VISION_PID_INTEGRAL_LIMIT;
  if (MaterialVision_XIntegral < -MATERIAL_VISION_PID_INTEGRAL_LIMIT)
    MaterialVision_XIntegral = -MATERIAL_VISION_PID_INTEGRAL_LIMIT;
  XOutput = MATERIAL_VISION_PID_X_SIGN *
            (MATERIAL_VISION_PID_X_KP * (float)Dy +
             MATERIAL_VISION_PID_X_KI * MaterialVision_XIntegral +
             MATERIAL_VISION_PID_X_KD *
             ((float)Dy - MaterialVision_XPreviousError));
  MaterialVision_XPreviousError = (float)Dy;
  if (XOutput > MATERIAL_VISION_PID_X_OUTPUT_LIMIT)
    XOutput = MATERIAL_VISION_PID_X_OUTPUT_LIMIT;
  if (XOutput < -MATERIAL_VISION_PID_X_OUTPUT_LIMIT)
    XOutput = -MATERIAL_VISION_PID_X_OUTPUT_LIMIT;
  if (fabsf(XOutput) < MATERIAL_VISION_MIN_X_STEP_DEG)
    XOutput = (XOutput < 0.0f) ? -MATERIAL_VISION_MIN_X_STEP_DEG :
              MATERIAL_VISION_MIN_X_STEP_DEG;
  return MaterialVision_XMoveStart(XOutput);
}

/** 检查底盘前后和X轴响应是否足够大且方向不共线。 */
static uint8_t MaterialVision_CalibrationCheck(void)
{
  float Cross;
  float ForwardNorm;
  float XNorm;

  ForwardNorm = sqrtf(MaterialVision_Response[0].Dx * MaterialVision_Response[0].Dx +
                   MaterialVision_Response[0].Dy * MaterialVision_Response[0].Dy);
  XNorm = sqrtf(MaterialVision_Response[2].Dx * MaterialVision_Response[2].Dx +
                MaterialVision_Response[2].Dy * MaterialVision_Response[2].Dy);
  if ((ForwardNorm * MATERIAL_VISION_CHASSIS_CAL_STEP_MM <
       MATERIAL_VISION_MIN_RESPONSE_PIXELS) ||
      (XNorm * MATERIAL_VISION_X_CAL_STEP_DEG <
       MATERIAL_VISION_MIN_RESPONSE_PIXELS))
    return 0U;
  Cross = fabsf(MaterialVision_Response[0].Dx * MaterialVision_Response[2].Dy -
                MaterialVision_Response[0].Dy * MaterialVision_Response[2].Dx);
  return (Cross >= MATERIAL_VISION_RESPONSE_CROSS_LIMIT * ForwardNorm * XNorm) ?
         1U : 0U;
}

/** 初始化底盘与X轴物料视觉状态机。 */
void MaterialVision_Init(void)
{
  (void)memset(&MaterialVision_Calibration, 0, sizeof(MaterialVision_Calibration));
  (void)memset(MaterialVision_Response, 0, sizeof(MaterialVision_Response));
  MaterialVision_State = MATERIAL_VISION_STATE_IDLE;
  MaterialVision_Task = MATERIAL_VISION_TASK_NONE;
  MaterialVision_Error = MATERIAL_VISION_ERROR_NONE;
  MaterialVision_Calibrated = 0U;
}

/** 启动物料自动搜索、粗调和X轴精调流程。 */
MaterialVision_ResultTypeDef MaterialVision_Start(Camera_ColorTypeDef Color)
{
  Camera_SnapshotTypeDef Snapshot;
  MechanicalArm_ResultTypeDef Result;

  if ((Color < CAMERA_COLOR_RED) || (Color > CAMERA_COLOR_LIGHT_BLUE))
    return MATERIAL_VISION_RESULT_PARAM_ERROR;
  if ((MaterialVision_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy())
    return MATERIAL_VISION_RESULT_BUSY;
  if (ArmVision_IsBusy() != 0U)
    return MATERIAL_VISION_RESULT_BUSY;
  if (ArmVision_IsReferenceValid() == 0U)
    return MATERIAL_VISION_RESULT_NOT_READY;
  if (MechanicalArm_IsBusy() != 0U)
    return MATERIAL_VISION_RESULT_BUSY;
  Mecanum_VelocityRefresh_Stop();
  (void)Mecanum_Test_Stop();
  MaterialVision_Color = Color;
  MaterialVision_Task = MATERIAL_VISION_TASK_ALIGN;
  MaterialVision_Error = MATERIAL_VISION_ERROR_NONE;
  MaterialVision_StartTick = HAL_GetTick();
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  MaterialVision_LastDataTick = MaterialVision_StartTick;
  MaterialVision_HasData = 0U;
#endif
  MaterialVision_LockCount = 0U;
  MaterialVision_StableCount = 0U;
  MaterialVision_XTotal = 0;
  MaterialVision_ChassisTotal = 0.0f;
  MaterialVision_ForwardIntegral = 0.0f;
  MaterialVision_ForwardPreviousError = 0.0f;
  MaterialVision_XIntegral = 0.0f;
  MaterialVision_XPreviousError = 0.0f;
  MaterialVision_LastSequence = 0U;
  Camera_SnapshotGet(&Snapshot);
  MaterialVision_LastSequence = Snapshot.Data.Sequence;
  Result = MechanicalArm_Home(MECHANICAL_ARM_AXIS_BASE, 0U);
  if (Result != MECHANICAL_ARM_RESULT_NONE)
  {
    MaterialVision_Error = MaterialVision_MotorError(Result);
    MaterialVision_State = MATERIAL_VISION_STATE_ERROR;
    return MATERIAL_VISION_RESULT_ERROR;
  }
  MaterialVision_State = MATERIAL_VISION_STATE_HOME_ACK;
  return MATERIAL_VISION_RESULT_OK;
}

/** 启动底盘和X轴像素响应标定流程。 */
MaterialVision_ResultTypeDef MaterialVision_CalibrationStart(Camera_ColorTypeDef Color)
{
  Camera_SnapshotTypeDef Snapshot;
  HAL_StatusTypeDef CameraResult;

  if ((Color < CAMERA_COLOR_RED) || (Color > CAMERA_COLOR_LIGHT_BLUE))
    return MATERIAL_VISION_RESULT_PARAM_ERROR;
  if ((MaterialVision_IsBusy() != 0U) || ChassisRoute_IsBusy() || ChassisMotion_IsBusy())
    return MATERIAL_VISION_RESULT_BUSY;
  if (ArmVision_IsBusy() != 0U)
    return MATERIAL_VISION_RESULT_BUSY;
  if (ArmVision_IsReferenceValid() == 0U)
    return MATERIAL_VISION_RESULT_NOT_READY;
  if (MechanicalArm_IsBusy() != 0U)
    return MATERIAL_VISION_RESULT_BUSY;
  Mecanum_VelocityRefresh_Stop();
  (void)Mecanum_Test_Stop();
  MaterialVision_Color = Color;
  MaterialVision_Task = MATERIAL_VISION_TASK_CALIBRATION;
  MaterialVision_Error = MATERIAL_VISION_ERROR_NONE;
  MaterialVision_Calibrated = 0U;
  MaterialVision_StartTick = HAL_GetTick();
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  MaterialVision_LastDataTick = MaterialVision_StartTick;
  MaterialVision_HasData = 0U;
#endif
  MaterialVision_XTotal = 0;
  MaterialVision_ChassisTotal = 0.0f;
  MaterialVision_LastSequence = 0U;
  Camera_SnapshotGet(&Snapshot);
  MaterialVision_LastSequence = Snapshot.Data.Sequence;
  CameraResult = Camera_MaterialStart(Color);
  if (CameraResult != HAL_OK)
  {
    MaterialVision_Error = MaterialVision_CameraError(CameraResult);
    MaterialVision_State = MATERIAL_VISION_STATE_ERROR;
    return MATERIAL_VISION_RESULT_ERROR;
  }
  MaterialVision_CollectStart(MATERIAL_VISION_PHASE_CAL_BASELINE);
  return MATERIAL_VISION_RESULT_OK;
}

/** 处理底盘和X轴物料视觉状态机。 */
void MaterialVision_Process(void)
{
  Camera_DataTypeDef Data;
  int16_t Dx;
  int16_t Dy;
  uint32_t Now;

  if ((MaterialVision_State == MATERIAL_VISION_STATE_IDLE) ||
      (MaterialVision_State == MATERIAL_VISION_STATE_ALIGNED) ||
      (MaterialVision_State == MATERIAL_VISION_STATE_ERROR))
    return;
  Now = HAL_GetTick();
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  if ((Now - MaterialVision_StartTick) >
      ((MaterialVision_Task == MATERIAL_VISION_TASK_CALIBRATION) ?
       12000U : MATERIAL_VISION_TASK_TIMEOUT_MS))
  {
    MaterialVision_Fail(MATERIAL_VISION_ERROR_TASK_TIMEOUT);
    return;
  }
#endif
  if ((MaterialVision_State == MATERIAL_VISION_STATE_HOME_ACK) ||
      (MaterialVision_State == MATERIAL_VISION_STATE_HOME_WAIT))
  {
    if ((Now - MaterialVision_StartTick) > MATERIAL_VISION_HOME_TIMEOUT_MS)
    {
      MaterialVision_Fail(MATERIAL_VISION_ERROR_HOME_TIMEOUT);
      return;
    }
    if ((MaterialVision_State == MATERIAL_VISION_STATE_HOME_WAIT) &&
        ((Now - MaterialVision_LastPollTick) >= MATERIAL_VISION_MOTOR_POLL_MS) &&
        (MechanicalArm_IsBusy() == 0U))
    {
      MaterialVision_LastPollTick = Now;
      if (MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_BASE) ==
          MECHANICAL_ARM_RESULT_NONE)
        MaterialVision_State = MATERIAL_VISION_STATE_HOME_WAIT;
    }
    return;
  }
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  if ((MaterialVision_HasData != 0U) &&
      ((Now - MaterialVision_LastDataTick) > MATERIAL_VISION_DATA_STALE_TIMEOUT_MS))
  {
    MaterialVision_Fail(MATERIAL_VISION_ERROR_CAMERA_STALE);
    return;
  }
#endif
  /*
   * Valid=0表示未发现目标，不参与坐标采样。测试模式允许持续等待；
   * 只有关闭TEST_NO_TIMEOUT后，以下高层首帧/搜索超时才生效。
   */
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  if ((MaterialVision_HasData == 0U) &&
      (MaterialVision_State != MATERIAL_VISION_STATE_SEARCH) &&
      ((Now - MaterialVision_StartTick) > MATERIAL_VISION_FIRST_DATA_TIMEOUT_MS))
  {
    MaterialVision_Fail(MATERIAL_VISION_ERROR_CAMERA_FIRST_TIMEOUT);
    return;
  }
#endif
  if (MaterialVision_State == MATERIAL_VISION_STATE_CHASSIS_MOVE)
  {
    if ((Now - MaterialVision_MoveStartTick) >= MaterialVision_MoveDurationMs)
    {
      (void)Mecanum_Test_Stop();
      MaterialVision_SettleStart();
    }
    return;
  }
  if (MaterialVision_State == MATERIAL_VISION_STATE_X_ARRIVAL)
  {
    if ((Now - MaterialVision_MoveStartTick) > MATERIAL_VISION_X_ARRIVAL_TIMEOUT_MS)
    {
      MaterialVision_Fail(MATERIAL_VISION_ERROR_MOTOR_ARRIVAL_TIMEOUT);
      return;
    }
    if (((Now - MaterialVision_LastPollTick) >= MATERIAL_VISION_MOTOR_POLL_MS) &&
        (MechanicalArm_IsBusy() == 0U))
    {
      MaterialVision_LastPollTick = Now;
      if (MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_X) ==
          MECHANICAL_ARM_RESULT_NONE)
        MaterialVision_State = MATERIAL_VISION_STATE_X_STATE;
    }
    return;
  }
  if ((MaterialVision_State == MATERIAL_VISION_STATE_X_ACK) ||
      (MaterialVision_State == MATERIAL_VISION_STATE_X_STATE))
    return;
  if (MaterialVision_State == MATERIAL_VISION_STATE_SETTLE)
  {
    if ((Now - MaterialVision_SettleStartTick) < MATERIAL_VISION_MOVE_SETTLE_MS)
      return;
    if (MaterialVision_Task == MATERIAL_VISION_TASK_CALIBRATION)
    {
      if ((MaterialVision_Phase == MATERIAL_VISION_PHASE_CAL_FORWARD_SAMPLE) ||
          (MaterialVision_Phase == MATERIAL_VISION_PHASE_CAL_X_SAMPLE))
        MaterialVision_CollectStart(MaterialVision_Phase);
      else if (MaterialVision_Phase == MATERIAL_VISION_PHASE_CAL_FORWARD_RETURN)
        MaterialVision_CollectStart(MATERIAL_VISION_PHASE_CAL_X_BASELINE);
      else if (MaterialVision_Phase == MATERIAL_VISION_PHASE_CAL_X_RETURN)
      {
        if (MaterialVision_CalibrationCheck() == 0U)
        {
          MaterialVision_Fail(MATERIAL_VISION_ERROR_RESPONSE_INVALID);
          return;
        }
        MaterialVision_Calibration.ForwardDxPerMm = MaterialVision_Response[0].Dx;
        MaterialVision_Calibration.ForwardDyPerMm = MaterialVision_Response[0].Dy;
        MaterialVision_Calibration.XDxPerDegree = MaterialVision_Response[2].Dx;
        MaterialVision_Calibration.XDyPerDegree = MaterialVision_Response[2].Dy;
        MaterialVision_Calibrated = 1U;
        Camera_RequestStop();
        MaterialVision_Task = MATERIAL_VISION_TASK_NONE;
        MaterialVision_State = MATERIAL_VISION_STATE_ALIGNED;
      }
    }
    else
      MaterialVision_CollectStart(MATERIAL_VISION_PHASE_ALIGN);
    return;
  }
  if (MaterialVision_NewDataGet(&Data) != 0U)
  {
    Dx = Data.DX;
    Dy = Data.DY;
    if (MaterialVision_State == MATERIAL_VISION_STATE_SEARCH)
    {
      if ((abs(Dx) <= MATERIAL_VISION_LOCK_DX_PIXELS) &&
          (abs(Dy) <= MATERIAL_VISION_LOCK_DY_PIXELS))
        MaterialVision_LockCount++;
      else
        MaterialVision_LockCount = 0U;
      if (MaterialVision_LockCount >= MATERIAL_VISION_STABLE_COUNT)
      {
        (void)Mecanum_Test_Stop();
        MaterialVision_CollectStart(MATERIAL_VISION_PHASE_ALIGN);
      }
    }
    else if (MaterialVision_State == MATERIAL_VISION_STATE_COLLECT)
    {
      MaterialVision_DxSamples[MaterialVision_SampleIndex] = Dx;
      MaterialVision_DySamples[MaterialVision_SampleIndex] = Dy;
      MaterialVision_SampleIndex++;
      if (MaterialVision_SampleIndex >= MATERIAL_VISION_SAMPLE_COUNT)
      {
        Dx = MaterialVision_MedianGet(MaterialVision_DxSamples);
        Dy = MaterialVision_MedianGet(MaterialVision_DySamples);
        switch (MaterialVision_Phase)
        {
          case MATERIAL_VISION_PHASE_CAL_BASELINE:
            MaterialVision_BaselineDx = Dx;
            MaterialVision_BaselineDy = Dy;
            MaterialVision_Phase = MATERIAL_VISION_PHASE_CAL_FORWARD_SAMPLE;
            if (MaterialVision_ChassisMoveStart(1.0f,
                                                MATERIAL_VISION_CHASSIS_CAL_STEP_MM) == 0U)
              MaterialVision_Fail(MaterialVision_Error);
            break;
          case MATERIAL_VISION_PHASE_CAL_FORWARD_SAMPLE:
            MaterialVision_Response[0].Dx = ((float)Dx - MaterialVision_BaselineDx) /
                                            MATERIAL_VISION_CHASSIS_CAL_STEP_MM;
            MaterialVision_Response[0].Dy = ((float)Dy - MaterialVision_BaselineDy) /
                                            MATERIAL_VISION_CHASSIS_CAL_STEP_MM;
            MaterialVision_Phase = MATERIAL_VISION_PHASE_CAL_FORWARD_RETURN;
            if (MaterialVision_ChassisMoveStart(-1.0f,
                                                MATERIAL_VISION_CHASSIS_CAL_STEP_MM) == 0U)
              MaterialVision_Fail(MaterialVision_Error);
            break;
          case MATERIAL_VISION_PHASE_CAL_X_BASELINE:
            MaterialVision_BaselineDx = Dx;
            MaterialVision_BaselineDy = Dy;
            MaterialVision_Phase = MATERIAL_VISION_PHASE_CAL_X_SAMPLE;
            if (MaterialVision_XMoveStart(MATERIAL_VISION_X_CAL_STEP_DEG) == 0U)
              MaterialVision_Fail(MaterialVision_Error);
            break;
          case MATERIAL_VISION_PHASE_CAL_X_SAMPLE:
            MaterialVision_Response[2].Dx = ((float)Dx - MaterialVision_BaselineDx) /
                                            MATERIAL_VISION_X_CAL_STEP_DEG;
            MaterialVision_Response[2].Dy = ((float)Dy - MaterialVision_BaselineDy) /
                                            MATERIAL_VISION_X_CAL_STEP_DEG;
            MaterialVision_Phase = MATERIAL_VISION_PHASE_CAL_X_RETURN;
            if (MaterialVision_XMoveStart(-MATERIAL_VISION_X_CAL_STEP_DEG) == 0U)
              MaterialVision_Fail(MaterialVision_Error);
            break;
          case MATERIAL_VISION_PHASE_ALIGN:
            if ((abs(Dx) <= MATERIAL_VISION_ALIGN_DEAD_ZONE_PIXELS) &&
                (abs(Dy) <= MATERIAL_VISION_ALIGN_DEAD_ZONE_PIXELS))
            {
              MaterialVision_StableCount++;
              if (MaterialVision_StableCount >= MATERIAL_VISION_STABLE_COUNT)
              {
                Camera_RequestStop();
                (void)Mecanum_Test_Stop();
                MaterialVision_State = MATERIAL_VISION_STATE_ALIGNED;
                MaterialVision_Task = MATERIAL_VISION_TASK_NONE;
              }
            }
            else
            {
              MaterialVision_StableCount = 0U;
              if (MaterialVision_CorrectionStart(Dx, Dy) == 0U)
                MaterialVision_Fail(MaterialVision_Error);
            }
            break;
          default:
            break;
        }
        if (MaterialVision_State == MATERIAL_VISION_STATE_COLLECT)
          MaterialVision_SampleIndex = 0U;
      }
    }
  }
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
  if (MaterialVision_State == MATERIAL_VISION_STATE_SEARCH)
  {
    if ((Now - MaterialVision_StartTick) > MATERIAL_VISION_SEARCH_TIMEOUT_MS)
      MaterialVision_Fail(MATERIAL_VISION_ERROR_SEARCH_TIMEOUT);
  }
#endif
}

/** 处理本模块发出的机械臂异步事件。 */
uint8_t MaterialVision_MotorEventHandle(const MechanicalArm_EventTypeDef *Event)
{
  uint8_t Flags;
  Camera_SnapshotTypeDef Snapshot;
  HAL_StatusTypeDef CameraResult;

  if (Event == NULL)
    return 0U;
  if ((MaterialVision_State == MATERIAL_VISION_STATE_HOME_ACK) &&
      (Event->Action == MECHANICAL_ARM_ACTION_HOME) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_BASE))
  {
    if (Event->Result != MECHANICAL_ARM_RESULT_OK)
    {
      MaterialVision_Fail(MaterialVision_MotorError(Event->Result));
    }
    else
    {
      MaterialVision_LastPollTick = 0U;
      MaterialVision_State = MATERIAL_VISION_STATE_HOME_WAIT;
    }
    return 1U;
  }
  if ((MaterialVision_State == MATERIAL_VISION_STATE_X_ACK) &&
      (Event->Action == MECHANICAL_ARM_ACTION_POSITION) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_X))
  {
    if (Event->Result != MECHANICAL_ARM_RESULT_OK)
      MaterialVision_Fail(MaterialVision_MotorError(Event->Result));
    else
    {
      MaterialVision_MoveStartTick = HAL_GetTick();
      MaterialVision_State = MATERIAL_VISION_STATE_X_ARRIVAL;
    }
    return 1U;
  }
  if ((MaterialVision_State == MATERIAL_VISION_STATE_X_STATE) &&
      (Event->Action == MECHANICAL_ARM_ACTION_STATE) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_X))
  {
    if (Event->Result != MECHANICAL_ARM_RESULT_OK)
      MaterialVision_Fail(MaterialVision_MotorError(Event->Result));
    else
    {
      Flags = Event->StateFlags[(uint8_t)MECHANICAL_ARM_AXIS_X];
      if ((Flags & 0x0CU) != 0U)
        MaterialVision_Fail(MATERIAL_VISION_ERROR_MOTOR_ERROR);
      else if ((Flags & 0x02U) != 0U)
      {
        MaterialVision_SettleStartTick = HAL_GetTick();
        MaterialVision_State = MATERIAL_VISION_STATE_SETTLE;
      }
      else
        MaterialVision_State = MATERIAL_VISION_STATE_X_ARRIVAL;
    }
    return 1U;
  }
  if ((MaterialVision_State == MATERIAL_VISION_STATE_HOME_WAIT) &&
      (Event->Action == MECHANICAL_ARM_ACTION_STATE) &&
      (Event->Axis == MECHANICAL_ARM_AXIS_BASE))
  {
    if (Event->Result != MECHANICAL_ARM_RESULT_OK)
      MaterialVision_Fail(MaterialVision_MotorError(Event->Result));
    else if ((Event->StateFlags[(uint8_t)MECHANICAL_ARM_AXIS_BASE] & 0x0CU) != 0U)
      MaterialVision_Fail(MATERIAL_VISION_ERROR_MOTOR_ERROR);
    else if ((Event->StateFlags[(uint8_t)MECHANICAL_ARM_AXIS_BASE] & 0x02U) != 0U)
    {
      CameraResult = Camera_MaterialStart(MaterialVision_Color);
      if (CameraResult != HAL_OK)
        MaterialVision_Fail(MaterialVision_CameraError(CameraResult));
      else
      {
        /* B2请求不会清空Camera的历史结构，重新记录序号避免把旧帧当首帧。 */
        Camera_SnapshotGet(&Snapshot);
        MaterialVision_LastSequence = Snapshot.Data.Sequence;
#if (MATERIAL_VISION_TEST_NO_TIMEOUT == 0U)
        MaterialVision_StartTick = HAL_GetTick();
        MaterialVision_LastDataTick = MaterialVision_StartTick;
#else
        MaterialVision_StartTick = HAL_GetTick();
#endif
        MaterialVision_State = MATERIAL_VISION_STATE_SEARCH;
        /* 初始搜索阶段向后移动；进入工作窗口后仍以前后速度修正DX。 */
        if (Mecanum_Velocity_Start(-MATERIAL_VISION_SEARCH_SPEED_MM_S,
                                   0.0f,
                                   0.0f) == false)
          MaterialVision_Fail(MATERIAL_VISION_ERROR_CHASSIS_TX);
      }
    }
    return 1U;
  }
  return 0U;
}

/** 查询新状态机是否正在运行。 */
uint8_t MaterialVision_IsBusy(void)
{
  return ((MaterialVision_State != MATERIAL_VISION_STATE_IDLE) &&
          (MaterialVision_State != MATERIAL_VISION_STATE_ALIGNED) &&
          (MaterialVision_State != MATERIAL_VISION_STATE_ERROR)) ? 1U : 0U;
}

/** 查询底盘和X轴响应标定是否有效。 */
uint8_t MaterialVision_IsCalibrated(void)
{
  return MaterialVision_Calibrated;
}

/** 查询新视觉状态机当前状态。 */
MaterialVision_StateTypeDef MaterialVision_StateGet(void)
{
  return MaterialVision_State;
}

/** 返回新视觉状态机的ASCII状态名称。 */
const char *MaterialVision_StateNameGet(void)
{
  static const char *Names[] =
  {
    "IDLE", "HOME_ACK", "HOME_WAIT", "SEARCH", "TARGET_LOCK", "COLLECT",
    "CHASSIS_MOVE", "X_ACK", "X_ARRIVAL", "X_STATE", "SETTLE", "ALIGNED", "ERROR"
  };
  return Names[(uint8_t)MaterialVision_State];
}

/** 返回新视觉状态机最近一次错误名称。 */
const char *MaterialVision_ErrorNameGet(void)
{
  if (MaterialVision_Error >= (sizeof(MaterialVision_ErrorNames) /
                               sizeof(MaterialVision_ErrorNames[0])))
    return "internal";
  return MaterialVision_ErrorNames[MaterialVision_Error];
}

/** 读取底盘和X轴的像素响应标定结果。 */
uint8_t MaterialVision_CalibrationGet(MaterialVision_CalibrationDataTypeDef *Data)
{
  if ((Data == NULL) || (MaterialVision_Calibrated == 0U))
    return 0U;
  *Data = MaterialVision_Calibration;
  return 1U;
}

/** 停止底盘、X轴和摄像头视觉流程。 */
void MaterialVision_Stop(void)
{
  if (MaterialVision_IsBusy() != 0U)
  {
    Camera_RequestStop();
    (void)Mecanum_Test_Stop();
    (void)MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL);
  }
  MaterialVision_Task = MATERIAL_VISION_TASK_NONE;
  MaterialVision_State = MATERIAL_VISION_STATE_IDLE;
  MaterialVision_Error = MATERIAL_VISION_ERROR_NONE;
}

/** 非消费式查询，供显示和控制台读取。 */
MaterialVision_ErrorTypeDef MaterialVision_ErrorGet(void)
{
  return MaterialVision_Error;
}

uint8_t MaterialVision_IsCalibrating(void)
{
  return (MaterialVision_IsBusy() && MaterialVision_Task == MATERIAL_VISION_TASK_CALIBRATION) ? 1U : 0U;
}
