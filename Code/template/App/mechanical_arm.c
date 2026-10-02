#include "mechanical_arm.h"
#include "chassis_route.h"
#include "chassis_motion.h"

#include "Emm_V5.h"
#include "motor_bus.h"
#include "mecanum_chassis.h"
#include <limits.h>
#include "mechanical_arm_config.h"
#include "motor_id_config.h"

#include <stdbool.h>
#include <string.h>

#define MECHANICAL_ARM_AXIS_COUNT 3U
/* 机械臂实际电机轴数量。 */

#define MECHANICAL_ARM_RX_BUFFER_SIZE 16U
/* UART5单次DMA空闲接收缓冲区长度。 */

#define MECHANICAL_ARM_ACK_TIMEOUT_MS 100U
/* 普通电机命令等待合法应答的最长时间。 */

#define MECHANICAL_ARM_CHECK_BYTE 0x6BU
/* 张大头串口协议默认校验字节。 */

typedef struct
{
  const char *Name;
  uint8_t MotorId;
  uint8_t PositiveDirection;
  MechanicalArm_ConfigTypeDef Config;
} MechanicalArm_AxisConfigTypeDef;

typedef enum
{
  MECHANICAL_ARM_STATE_IDLE = 0,
  MECHANICAL_ARM_STATE_WAIT_TX,
  MECHANICAL_ARM_STATE_WAIT_ACK
} MechanicalArm_StateTypeDef;

typedef struct
{
  MechanicalArm_ActionTypeDef Action;
  MechanicalArm_AxisTypeDef RequestedAxis;
  MechanicalArm_AxisTypeDef CurrentAxis;
  int32_t PositionPulses;
  int32_t CurrentPosition;
  int64_t CurrentPositionRaw;
  uint16_t SpeedRpm;
  uint8_t Acceleration;
  MechanicalArm_PositionModeTypeDef PositionMode;
  uint8_t HomeMode;
  uint8_t ExpectedFunction;
  uint8_t SuccessMask;
  uint8_t FailureMask;
  uint8_t StateFlags[MECHANICAL_ARM_AXIS_COUNT];
} MechanicalArm_RequestTypeDef;

static MechanicalArm_AxisConfigTypeDef MechanicalArm_AxisConfig[MECHANICAL_ARM_AXIS_COUNT] = {
    {"base",
     MOTOR_ID_ARM_BASE,
     MECHANICAL_ARM_BASE_POSITIVE_DIR,
     {60U, 20U, MECHANICAL_ARM_COMMAND_PULSES_PER_REV}},
    {"z",
     MOTOR_ID_ARM_Z,
     MECHANICAL_ARM_Z_POSITIVE_DIR,
     {500U, 120U, 0U}}, /* Z普通运动默认等效于config z 500 120 0；回零使用驱动器参数。 */
    {"x",
     MOTOR_ID_ARM_X,
     MECHANICAL_ARM_X_POSITIVE_DIR,
     {40U, 20U, MECHANICAL_ARM_COMMAND_PULSES_PER_REV}}};

static UART_HandleTypeDef *MechanicalArm_Uart;
static volatile uint8_t MechanicalArm_StopAllPending;
static MechanicalArm_StateTypeDef MechanicalArm_State;
static MechanicalArm_RequestTypeDef MechanicalArm_Request;
static MechanicalArm_EventTypeDef MechanicalArm_Event;
static volatile uint8_t MechanicalArm_EventReady;
static uint8_t MechanicalArm_Transmitted, MechanicalArm_Acknowledged;

/**
 * 函    数：取得当前轴配置项
 * 参    数：Axis 机械臂轴枚举
 * 返 回 值：配置项地址；轴无效时返回NULL
 * 说    明：all不是实际电机轴，不能取得单轴配置
 */
static MechanicalArm_AxisConfigTypeDef *MechanicalArm_AxisConfigGet(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis >= MECHANICAL_ARM_AXIS_ALL)
  {
    return NULL;
  }

  return &MechanicalArm_AxisConfig[(uint8_t)Axis];
}

/**
 * 函    数：将当前请求结果提交给上位机终端
 * 参    数：Result 执行结果；FailureAxis 失败轴；MotorCode 电机返回码
 * 返 回 值：无
 * 说    明：结果只保存一份，由ArmConsole在主循环中取走
 */
static void MechanicalArm_RequestFinish(MechanicalArm_ResultTypeDef Result,
                                        MechanicalArm_AxisTypeDef FailureAxis, uint8_t MotorCode)
{
  uint8_t Index;

  MechanicalArm_Event.Action = MechanicalArm_Request.Action;
  MechanicalArm_Event.Axis = MechanicalArm_Request.RequestedAxis;
  MechanicalArm_Event.FailureAxis = FailureAxis;
  MechanicalArm_Event.Result = Result;
  MechanicalArm_Event.PositionPulses = MechanicalArm_Request.PositionPulses;
  MechanicalArm_Event.CurrentPosition = MechanicalArm_Request.CurrentPosition;
  MechanicalArm_Event.CurrentPositionRaw = MechanicalArm_Request.CurrentPositionRaw;
  MechanicalArm_Event.HomeMode = MechanicalArm_Request.HomeMode;
  MechanicalArm_Event.MotorCode = MotorCode;
  MechanicalArm_Event.SuccessMask = MechanicalArm_Request.SuccessMask;
  MechanicalArm_Event.FailureMask = MechanicalArm_Request.FailureMask;
  MechanicalArm_Event.Transmitted = MechanicalArm_Transmitted;
  MechanicalArm_Event.Acknowledged = MechanicalArm_Acknowledged;
  for (Index = 0U; Index < MECHANICAL_ARM_AXIS_COUNT; Index++)
  {
    MechanicalArm_Event.StateFlags[Index] = MechanicalArm_Request.StateFlags[Index];
  }

  MechanicalArm_State = MECHANICAL_ARM_STATE_IDLE;

  MechanicalArm_EventReady = 1U;
}

/**
 * 函    数：发送当前轴对应的张大头电机指令
 * 参    数：无
 * 返 回 值：HAL发送启动状态
 * 说    明：所有帧仍由官方Emm_V5函数封装，本层只选择参数
 */
static HAL_StatusTypeDef MechanicalArm_CurrentCommandSend(void)
{
  MechanicalArm_AxisConfigTypeDef *AxisConfig;
  uint8_t Direction;
  uint32_t AbsolutePulses;
  MechanicalArm_Transmitted = 0;
  MechanicalArm_Acknowledged = 0;

  AxisConfig = MechanicalArm_AxisConfigGet(MechanicalArm_Request.CurrentAxis);
  if (AxisConfig == NULL)
  {
    return HAL_ERROR;
  }

  switch (MechanicalArm_Request.Action)
  {
  case MECHANICAL_ARM_ACTION_ENABLE:
    MechanicalArm_Request.ExpectedFunction = 0xF3U;
    Emm_V5_En_Control(AxisConfig->MotorId, true, false);
    break;

  case MECHANICAL_ARM_ACTION_DISABLE:
    MechanicalArm_Request.ExpectedFunction = 0xF3U;
    Emm_V5_En_Control(AxisConfig->MotorId, false, false);
    break;

  case MECHANICAL_ARM_ACTION_POSITION:
    MechanicalArm_Request.ExpectedFunction = 0xFDU;
    Direction = AxisConfig->PositiveDirection;
    if (MechanicalArm_Request.PositionPulses < 0)
    {
      Direction = (Direction == 0U) ? 1U : 0U;
      AbsolutePulses = (uint32_t)(-(int64_t)MechanicalArm_Request.PositionPulses);
    }
    else
    {
      AbsolutePulses = (uint32_t)MechanicalArm_Request.PositionPulses;
    }
    Emm_V5_Pos_Control(AxisConfig->MotorId, Direction, MechanicalArm_Request.SpeedRpm,
                       MechanicalArm_Request.Acceleration, AbsolutePulses,
                       (uint8_t)MechanicalArm_Request.PositionMode, false);
    break;

  case MECHANICAL_ARM_ACTION_POSITION_READ:
    MechanicalArm_Request.ExpectedFunction = 0x36U;
    Emm_V5_Read_Sys_Params(AxisConfig->MotorId, S_CPOS);
    break;

  case MECHANICAL_ARM_ACTION_STATE:
    MechanicalArm_Request.ExpectedFunction = 0x3AU;
    Emm_V5_Read_Sys_Params(AxisConfig->MotorId, S_FLAG);
    break;

  case MECHANICAL_ARM_ACTION_HOME_STATE:
    MechanicalArm_Request.ExpectedFunction = 0x3BU;
    Emm_V5_Read_Sys_Params(AxisConfig->MotorId, S_OFLAG);
    break;

  case MECHANICAL_ARM_ACTION_HOME_ABORT:
    MechanicalArm_Request.ExpectedFunction = 0x9CU;
    Emm_V5_Origin_Interrupt(AxisConfig->MotorId);
    break;

  case MECHANICAL_ARM_ACTION_ORIGIN_SET:
    MechanicalArm_Request.ExpectedFunction = 0x93U;
    Emm_V5_Origin_Set_O(AxisConfig->MotorId, true);
    break;

  case MECHANICAL_ARM_ACTION_HOME:
    MechanicalArm_Request.ExpectedFunction = 0x9AU;
    Emm_V5_Origin_Trigger_Return(AxisConfig->MotorId, MechanicalArm_Request.HomeMode, false);
    break;

  case MECHANICAL_ARM_ACTION_ZERO:
    MechanicalArm_Request.ExpectedFunction = 0x0AU;
    Emm_V5_Reset_CurPos_To_Zero(AxisConfig->MotorId);
    break;

  case MECHANICAL_ARM_ACTION_STOP:
    MechanicalArm_Request.ExpectedFunction = 0xFEU;
    Emm_V5_Stop_Now(AxisConfig->MotorId, false);
    break;

  default:
    return HAL_ERROR;
  }

  if (Emm_V5_TxStatusGet() != HAL_OK)
  {
    return Emm_V5_TxStatusGet();
  }

  MechanicalArm_State = MECHANICAL_ARM_STATE_WAIT_TX;
  return HAL_OK;
}

/**
 * 函    数：发送广播立即停止命令
 * 参    数：无
 * 返 回 值：HAL发送启动状态
 * 说    明：广播地址0用于stop all，发送完成即认为指令已发出
 */
static HAL_StatusTypeDef MechanicalArm_StopAllSend(void)
{
  MechanicalArm_Transmitted = 0;
  MechanicalArm_Acknowledged = 0;
  memset(&MechanicalArm_Request, 0, sizeof(MechanicalArm_Request));
  MechanicalArm_Request.Action = MECHANICAL_ARM_ACTION_STOP;
  MechanicalArm_Request.RequestedAxis = MECHANICAL_ARM_AXIS_ALL;
  MechanicalArm_Request.CurrentAxis = MECHANICAL_ARM_AXIS_ALL;

  Emm_V5_Stop_Now(0U, false);
  if (Emm_V5_TxStatusGet() != HAL_OK)
  {
    return Emm_V5_TxStatusGet();
  }

  MechanicalArm_State = MECHANICAL_ARM_STATE_WAIT_TX;
  return HAL_OK;
}

/**
 * 函    数：开始一个单轴或顺序多轴请求
 * 参    数：Action 动作；Axis 轴；Pulses 相对脉冲
 * 返 回 值：请求接收结果
 * 说    明：all从base开始，后续轴由应答处理函数推进
 */
static MechanicalArm_ResultTypeDef MechanicalArm_RequestStart(MechanicalArm_ActionTypeDef Action,
                                                              MechanicalArm_AxisTypeDef Axis,
                                                              int32_t Pulses, uint8_t HomeMode)
{
  MechanicalArm_AxisConfigTypeDef *AxisConfig;

  if (MechanicalArm_Uart == NULL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  /* Readback and emergency stops remain available while a route reserves motion. */
  if ((ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) &&
      (Action != MECHANICAL_ARM_ACTION_POSITION_READ) &&
      (Action != MECHANICAL_ARM_ACTION_STATE) && (Action != MECHANICAL_ARM_ACTION_STOP) &&
      (Action != MECHANICAL_ARM_ACTION_HOME_ABORT))
  {
    return MECHANICAL_ARM_RESULT_BUSY;
  }
  if (MechanicalArm_State != MECHANICAL_ARM_STATE_IDLE)
  {
    return MECHANICAL_ARM_RESULT_BUSY;
  }

  memset(&MechanicalArm_Request, 0, sizeof(MechanicalArm_Request));
  MechanicalArm_Request.Action = Action;
  MechanicalArm_Request.RequestedAxis = Axis;
  MechanicalArm_Request.CurrentAxis =
      (Axis == MECHANICAL_ARM_AXIS_ALL) ? MECHANICAL_ARM_AXIS_BASE : Axis;
  MechanicalArm_Request.PositionPulses = Pulses;
  MechanicalArm_Request.SpeedRpm = 0U;
  MechanicalArm_Request.Acceleration = 0U;
  MechanicalArm_Request.PositionMode = MECHANICAL_ARM_POSITION_RELATIVE_CURRENT;
  MechanicalArm_Request.HomeMode = HomeMode;
  if (Action == MECHANICAL_ARM_ACTION_POSITION)
  {
    AxisConfig = MechanicalArm_AxisConfigGet(Axis);
    if (AxisConfig == NULL)
    {
      return MECHANICAL_ARM_RESULT_PARAM_ERROR;
    }
    MechanicalArm_Request.SpeedRpm = AxisConfig->Config.SpeedRpm;
    MechanicalArm_Request.Acceleration = AxisConfig->Config.Acceleration;
  }
  if (MechanicalArm_CurrentCommandSend() != HAL_OK)
  {
    MechanicalArm_State = MECHANICAL_ARM_STATE_IDLE;
    return MECHANICAL_ARM_RESULT_TX_ERROR;
  }

  return MECHANICAL_ARM_RESULT_NONE;
}

/**
 * 函    数：推进all命令到下一根轴
 * 参    数：无
 * 返 回 值：1表示已发送下一轴；0表示全部完成或发送失败
 * 说    明：顺序固定为base、z、x
 */
static uint8_t MechanicalArm_NextAxisStart(void)
{
  MechanicalArm_AxisTypeDef NextAxis;

  if (MechanicalArm_Request.CurrentAxis >= MECHANICAL_ARM_AXIS_X)
  {
    return 0U;
  }

  NextAxis = (MechanicalArm_AxisTypeDef)(MechanicalArm_Request.CurrentAxis + 1U);
  MechanicalArm_Request.CurrentAxis = NextAxis;
  if (MechanicalArm_CurrentCommandSend() != HAL_OK)
  {
    MechanicalArm_Request.FailureMask |= (uint8_t)(1U << (uint8_t)NextAxis);
    MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_TX_ERROR, NextAxis, 0U);
    return 0U;
  }

  return 1U;
}

/**
 * 函    数：处理当前电机的合法应答
 * 参    数：Response 收到的4字节应答
 * 返 回 值：无
 * 说    明：enable/state all失败即停止，其余all动作记录失败并继续后续轴
 */
static void MechanicalArm_ResponseProcess(const uint8_t *Response)
{
  uint8_t AxisMask;
  uint8_t MotorCode;

  AxisMask = (uint8_t)(1U << (uint8_t)MechanicalArm_Request.CurrentAxis);
  MotorCode = Response[2];

  if (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_POSITION_READ)
  {
    uint32_t magnitude = ((uint32_t)Response[3] << 24) | ((uint32_t)Response[4] << 16) |
                         ((uint32_t)Response[5] << 8) | Response[6];
    int64_t raw = Response[2] ? -(int64_t)magnitude : (int64_t)magnitude;
    MechanicalArm_Request.CurrentPositionRaw = raw;
    if (raw > INT32_MAX || raw < INT32_MIN)
    {
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_MOTOR_ERROR,
                                  MechanicalArm_Request.CurrentAxis, 0);
      return;
    }
    MechanicalArm_Request.CurrentPosition = (int32_t)raw;
    MechanicalArm_Request.SuccessMask |= AxisMask;
    MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_OK, MECHANICAL_ARM_AXIS_INVALID, 0U);
    return;
  }

  if (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_STATE ||
      MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_HOME_STATE)
  {
    MechanicalArm_Request.StateFlags[(uint8_t)MechanicalArm_Request.CurrentAxis] = MotorCode;
    MechanicalArm_Request.SuccessMask |= AxisMask;
  }
  else if (MotorCode == 0x02U)
  {
    MechanicalArm_Request.SuccessMask |= AxisMask;
  }
  else
  {
    MechanicalArm_Request.FailureMask |= AxisMask;
    if ((MechanicalArm_Request.RequestedAxis != MECHANICAL_ARM_AXIS_ALL) ||
        (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_ENABLE) ||
        (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_STATE))
    {
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_MOTOR_ERROR,
                                  MechanicalArm_Request.CurrentAxis, MotorCode);
      return;
    }
  }

  if ((MechanicalArm_Request.RequestedAxis == MECHANICAL_ARM_AXIS_ALL) &&
      (MechanicalArm_Request.CurrentAxis < MECHANICAL_ARM_AXIS_X))
  {
    (void)MechanicalArm_NextAxisStart();
    return;
  }

  if (MechanicalArm_Request.FailureMask != 0U)
  {
    MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_MOTOR_ERROR,
                                MechanicalArm_Request.CurrentAxis, MotorCode);
  }
  else
  {
    MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_OK, MECHANICAL_ARM_AXIS_INVALID, MotorCode);
  }
}

/**
 * 函    数：处理电机应答超时
 * 参    数：无
 * 返 回 值：无
 * 说    明：配置类all命令记录失败并继续，其余命令立即结束且不重发
 */
static void MechanicalArm_TimeoutProcess(void)
{
  uint8_t AxisMask;

  AxisMask = (uint8_t)(1U << (uint8_t)MechanicalArm_Request.CurrentAxis);
  MechanicalArm_Request.FailureMask |= AxisMask;
  MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_ACK_TIMEOUT, MechanicalArm_Request.CurrentAxis,
                              0U);
}

/**
 * 函    数：初始化机械臂电机通信
 * 参    数：huart UART5句柄
 * 返 回 值：HAL执行状态
 * 说    明：绑定UART5到机械臂模块，清空运行中的请求/事件状态，
 *           最后启动UART5 ReceiveToIdle Normal DMA接收
 */
HAL_StatusTypeDef MechanicalArm_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL || huart->Instance != UART5)
    return HAL_ERROR;
  MechanicalArm_Uart = huart;
  MechanicalArm_State = MECHANICAL_ARM_STATE_IDLE;
  MechanicalArm_EventReady = 0;
  MechanicalArm_StopAllPending = 0;
  memset(&MechanicalArm_Request, 0, sizeof(MechanicalArm_Request));
  memset(&MechanicalArm_Event, 0, sizeof(MechanicalArm_Event));
  return MotorBus_Init(huart);
}

/**
 * 函    数：推进机械臂通信状态机
 * 参    数：无
 * 返 回 值：无
 * 说    明：在主循环高频调用，负责TX完成、应答校验、超时和停止抢占
 */
void MechanicalArm_Process(void)
{
  MotorBus_Event_t event;
  if (MechanicalArm_State == MECHANICAL_ARM_STATE_IDLE)
    return;
  if (!MotorBus_EventGet(MOTOR_BUS_ARM, &event))
    return;
  MechanicalArm_Transmitted = (uint8_t)event.tx_completed;
  if (MechanicalArm_StopAllPending)
  {
    MechanicalArm_StopAllPending = 0;
    if (MechanicalArm_StopAllSend() != HAL_OK)
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_TX_ERROR, MECHANICAL_ARM_AXIS_ALL, 0);
    return;
  }
  if (event.result == MOTOR_BUS_REPLY)
  {
    MechanicalArm_Transmitted = 1;
    MechanicalArm_Acknowledged = 1;
    MechanicalArm_ResponseProcess(event.data);
    return;
  }
  if (event.result == MOTOR_BUS_TX_DONE &&
      (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_STOP ||
       MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_HOME_ABORT))
  {
    MechanicalArm_Transmitted = 1;
    MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_OK, MECHANICAL_ARM_AXIS_INVALID, 0);
    return;
  }
  if (event.result == MOTOR_BUS_TIMEOUT)
  {
    MechanicalArm_TimeoutProcess();
    return;
  }
  MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_TX_ERROR, MechanicalArm_Request.CurrentAxis, 0);
}

/**
 * 函    数：请求使能单轴或全部轴
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：all按base、z、x逐台使能，任一失败后停止
 */
MechanicalArm_ResultTypeDef MechanicalArm_Enable(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis > MECHANICAL_ARM_AXIS_ALL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_ENABLE, Axis, 0, 0U);
}

/**
 * 函    数：请求失能单轴或全部轴
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：disable all会尝试全部三轴并汇总失败位
 */
MechanicalArm_ResultTypeDef MechanicalArm_Disable(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis > MECHANICAL_ARM_AXIS_ALL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_DISABLE, Axis, 0, 0U);
}

/**
 * 函    数：请求单轴相对位置运动
 * 参    数：Axis 目标轴；Pulses 带方向的相对脉冲
 * 返 回 值：请求接收结果
 * 说    明：脉冲必须非零；Base/X受当前RAM上限限制，Z轴不设置软件脉冲上限
 */
MechanicalArm_ResultTypeDef MechanicalArm_Position(MechanicalArm_AxisTypeDef Axis, int32_t Pulses)
{
  MechanicalArm_AxisConfigTypeDef *AxisConfig;

  AxisConfig = MechanicalArm_AxisConfigGet(Axis);
  if ((AxisConfig == NULL) || (Pulses == 0))
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  if ((Axis != MECHANICAL_ARM_AXIS_Z) && ((Pulses > (int32_t)AxisConfig->Config.PulseLimit) ||
                                          (Pulses < -(int32_t)AxisConfig->Config.PulseLimit)))
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_POSITION, Axis, Pulses, 0U);
}

/**
 * 函    数：使用指定参数请求单轴位置运动
 * 参    数：Axis 目标轴；Position 带方向脉冲；SpeedRpm 速度；
 *           Acceleration 加速度；Mode 绝对或相对实际位置模式
 * 返 回 值：请求接收结果
 * 说    明：视觉微调使用相对模式，已标定抓放位置可使用绝对模式
 */
MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef Axis,
                                                     int32_t Position, uint16_t SpeedRpm,
                                                     uint8_t Acceleration,
                                                     MechanicalArm_PositionModeTypeDef Mode)
{
  if ((MechanicalArm_AxisConfigGet(Axis) == NULL) ||
      ((Position == 0) && (Mode != MECHANICAL_ARM_POSITION_ABSOLUTE)) || (SpeedRpm < 1U) ||
      (SpeedRpm > 3000U) || (Acceleration < 1U) ||
      ((Mode != MECHANICAL_ARM_POSITION_ABSOLUTE) &&
       (Mode != MECHANICAL_ARM_POSITION_RELATIVE_CURRENT)))
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  if ((MechanicalArm_State != MECHANICAL_ARM_STATE_IDLE) ||
      ChassisRoute_IsBusy() || ChassisMotion_IsBusy())
  {
    return MECHANICAL_ARM_RESULT_BUSY;
  }

  (void)memset(&MechanicalArm_Request, 0, sizeof(MechanicalArm_Request));
  MechanicalArm_Request.Action = MECHANICAL_ARM_ACTION_POSITION;
  MechanicalArm_Request.RequestedAxis = Axis;
  MechanicalArm_Request.CurrentAxis = Axis;
  MechanicalArm_Request.PositionPulses = Position;
  MechanicalArm_Request.SpeedRpm = SpeedRpm;
  MechanicalArm_Request.Acceleration = Acceleration;
  MechanicalArm_Request.PositionMode = Mode;
  if (MechanicalArm_CurrentCommandSend() != HAL_OK)
  {
    MechanicalArm_State = MECHANICAL_ARM_STATE_IDLE;
    return MECHANICAL_ARM_RESULT_TX_ERROR;
  }
  return MECHANICAL_ARM_RESULT_NONE;
}

/**
 * 函    数：读取单轴实时位置
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：驱动器返回的带方向位置由结果事件CurrentPosition给出
 */
MechanicalArm_ResultTypeDef MechanicalArm_PositionRead(MechanicalArm_AxisTypeDef Axis)
{
  if (MechanicalArm_AxisConfigGet(Axis) == NULL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_POSITION_READ, Axis, 0, 0U);
}

/**
 * 函    数：请求读取单轴或全部轴S_FLAG
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：state all按base、z、x顺序读取
 */
MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis > MECHANICAL_ARM_AXIS_ALL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_STATE, Axis, 0, 0U);
}

/**
 * 函    数：保存单轴或全部轴当前单圈位置为机械原点
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：使用存储标志写入驱动器；当前位置将作为单圈机械零度，
 *           all按base、z、x逐台设置；不会让电机运动
 */
MechanicalArm_ResultTypeDef MechanicalArm_OriginSet(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis > MECHANICAL_ARM_AXIS_ALL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_ORIGIN_SET, Axis, 0, 0U);
}

/**
 * 函    数：触发单轴或全部轴执行指定模式回零
 * 参    数：Axis 目标轴；HomeMode 回零模式0至3
 * 返 回 值：请求接收结果
 * 说    明：成功结果只表示驱动器接受回零命令，不代表机械运动已经完成
 */
MechanicalArm_ResultTypeDef MechanicalArm_Home(MechanicalArm_AxisTypeDef Axis, uint8_t HomeMode)
{
  if ((Axis > MECHANICAL_ARM_AXIS_ALL) || (HomeMode > 3U))
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_HOME, Axis, 0, HomeMode);
}

/**
 * 函    数：请求将单轴或全部轴的当前位置设为零点
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：zero all按base、z、x顺序清零并汇总失败轴，不会驱动电机运动
 */
MechanicalArm_ResultTypeDef MechanicalArm_Zero(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis > MECHANICAL_ARM_AXIS_ALL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_ZERO, Axis, 0, 0U);
}

/**
 * 函    数：请求立即停止单轴或全部轴
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：stop all使用广播地址并可抢占当前普通请求
 */
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef Axis)
{
  MotorBus_Event_t discarded;
  if (Axis > MECHANICAL_ARM_AXIS_ALL || MechanicalArm_Uart == NULL)
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  if (Axis != MECHANICAL_ARM_AXIS_ALL)
    return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_STOP, Axis, 0, 0);
  /* Broadcast stop also addresses wheels: cancel their unsent synchronized batch first. */
  (void)Mecanum_Test_Stop();
  MechanicalArm_EventReady = 0;
  if (MechanicalArm_State != MECHANICAL_ARM_STATE_IDLE)
  {
    MechanicalArm_StopAllPending = 1;
    MotorBus_Cancel(MOTOR_BUS_ARM);
    return MECHANICAL_ARM_RESULT_NONE;
  }
  (void)MotorBus_EventGet(MOTOR_BUS_ARM, &discarded);
  if (MechanicalArm_StopAllSend() != HAL_OK)
    return MECHANICAL_ARM_RESULT_TX_ERROR;
  return MECHANICAL_ARM_RESULT_NONE;
}

/**
 * 函    数：设置单轴或全部轴的RAM运动配置
 * 参    数：Axis 目标轴；SpeedRpm 速度；Acceleration 加速度；PulseLimit 脉冲上限
 * 返 回 值：1表示成功，0表示参数非法
 * 说    明：配置不写入驱动器和Flash，复位后恢复默认值
 */
uint8_t MechanicalArm_ConfigSet(MechanicalArm_AxisTypeDef Axis, uint16_t SpeedRpm,
                                uint8_t Acceleration, uint32_t PulseLimit)
{
  uint8_t StartIndex;
  uint8_t EndIndex;
  uint8_t Index;

  if ((Axis > MECHANICAL_ARM_AXIS_ALL) || (SpeedRpm < 1U) || (SpeedRpm > 3000U) ||
      (Acceleration < 1U) || (PulseLimit > MECHANICAL_ARM_COMMAND_PULSES_PER_REV) ||
      ((PulseLimit == 0U) && (Axis != MECHANICAL_ARM_AXIS_Z)))
  {
    return 0U;
  }

  StartIndex = (Axis == MECHANICAL_ARM_AXIS_ALL) ? 0U : (uint8_t)Axis;
  EndIndex = (Axis == MECHANICAL_ARM_AXIS_ALL) ? (MECHANICAL_ARM_AXIS_COUNT - 1U) : (uint8_t)Axis;
  for (Index = StartIndex; Index <= EndIndex; Index++)
  {
    MechanicalArm_AxisConfig[Index].Config.SpeedRpm = SpeedRpm;
    MechanicalArm_AxisConfig[Index].Config.Acceleration = Acceleration;
    /* Z轴不设置软件脉冲上限，保留0作为无限制标志。 */
    MechanicalArm_AxisConfig[Index].Config.PulseLimit =
        (Index == (uint8_t)MECHANICAL_ARM_AXIS_Z) ? 0U : PulseLimit;
  }
  return 1U;
}

/**
 * 函    数：读取指定单轴的RAM运动配置
 * 参    数：Axis 目标轴；Config 配置输出地址
 * 返 回 值：1表示成功，0表示参数非法
 * 说    明：all不是单轴，调用者应分别读取三轴
 */
uint8_t MechanicalArm_ConfigGet(MechanicalArm_AxisTypeDef Axis, MechanicalArm_ConfigTypeDef *Config)
{
  MechanicalArm_AxisConfigTypeDef *AxisConfig;

  AxisConfig = MechanicalArm_AxisConfigGet(Axis);
  if ((AxisConfig == NULL) || (Config == NULL))
  {
    return 0U;
  }
  *Config = AxisConfig->Config;
  return 1U;
}

/**
 * 函    数：取得最近一次异步命令结果
 * 参    数：Event 结果输出地址
 * 返 回 值：1表示取得新结果，0表示暂无结果
 * 说    明：读取成功后清除结果标志
 */
uint8_t MechanicalArm_ResultGet(MechanicalArm_EventTypeDef *Event)
{
  uint32_t Primask;

  if ((Event == NULL) || (MechanicalArm_EventReady == 0U))
  {
    return 0U;
  }

  Primask = __get_PRIMASK();
  __disable_irq();
  *Event = MechanicalArm_Event;
  MechanicalArm_EventReady = 0U;
  __set_PRIMASK(Primask);
  return 1U;
}

/**
 * 函    数：查询机械臂通信是否忙
 * 参    数：无
 * 返 回 值：1表示存在活动请求，0表示空闲
 * 说    明：仅用于终端提示，不代替命令接口自身的忙校验
 */
uint8_t MechanicalArm_IsBusy(void)
{
  return (MechanicalArm_State == MECHANICAL_ARM_STATE_IDLE) ? 0U : 1U;
}

/**
 * 函    数：将ASCII轴名转换为轴枚举
 * 参    数：Name 以零结尾的轴名
 * 返 回 值：轴枚举；无法识别时返回INVALID
 * 说    明：支持base、z、x和all
 */
MechanicalArm_AxisTypeDef MechanicalArm_AxisGet(const char *Name)
{
  uint8_t Index;

  if (Name == NULL)
  {
    return MECHANICAL_ARM_AXIS_INVALID;
  }
  if (strcmp(Name, "all") == 0)
  {
    return MECHANICAL_ARM_AXIS_ALL;
  }
  for (Index = 0U; Index < MECHANICAL_ARM_AXIS_COUNT; Index++)
  {
    if (strcmp(Name, MechanicalArm_AxisConfig[Index].Name) == 0)
    {
      return (MechanicalArm_AxisTypeDef)Index;
    }
  }
  return MECHANICAL_ARM_AXIS_INVALID;
}

/**
 * 函    数：取得轴枚举对应的ASCII名称
 * 参    数：Axis 轴枚举
 * 返 回 值：轴名常量字符串
 * 说    明：非法枚举返回invalid
 */
const char *MechanicalArm_AxisNameGet(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis < MECHANICAL_ARM_AXIS_ALL)
  {
    return MechanicalArm_AxisConfig[(uint8_t)Axis].Name;
  }
  if (Axis == MECHANICAL_ARM_AXIS_ALL)
  {
    return "all";
  }
  return "invalid";
}

/**
 * 函    数：接收UART5 DMA空闲事件
 * 参    数：huart 串口句柄；Size 本次接收长度
 * 返 回 值：无
 * 说    明：中断中只复制最多16字节并立即重启DMA，解析留在主循环
 */
void MechanicalArm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  MotorBus_RxEventCallback(huart, Size);
}

/**
 * 函    数：记录UART5 DMA发送完成事件
 * 参    数：huart 串口句柄
 * 返 回 值：无
 * 说    明：只置位，状态机在主循环中推进
 */
void MechanicalArm_TxCpltCallback(UART_HandleTypeDef *huart)
{
  MotorBus_TxCpltCallback(huart);
}

/**
 * 函    数：恢复UART5错误后的DMA接收
 * 参    数：huart 串口句柄
 * 返 回 值：无
 * 说    明：活动请求由主循环报告通信失败，不在中断中重发命令
 */
void MechanicalArm_ErrorCallback(UART_HandleTypeDef *huart)
{
  MotorBus_ErrorCallback(huart);
}

MechanicalArm_ResultTypeDef MechanicalArm_HomeStateRead(MechanicalArm_AxisTypeDef Axis)
{
  if(Axis > MECHANICAL_ARM_AXIS_X) return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_HOME_STATE,Axis,0,0);
}
MechanicalArm_ResultTypeDef MechanicalArm_HomeAbort(MechanicalArm_AxisTypeDef Axis)
{
  if(Axis > MECHANICAL_ARM_AXIS_X) return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_HOME_ABORT,Axis,0,0);
}
