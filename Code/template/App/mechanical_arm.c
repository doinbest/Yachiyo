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
  int32_t CurrentSpeedRpm;
  int64_t CurrentPositionRaw;
  uint16_t SpeedRpm;
  uint8_t Acceleration;
  MechanicalArm_PositionModeTypeDef PositionMode;
  uint8_t HomeMode;
  uint8_t HomePrep;
  uint8_t HomeParams[20];
  uint8_t ExpectedFunction;
  MechanicalArm_DiagnosticTypeDef Diagnostic;
  uint8_t SuccessMask;
  uint8_t FailureMask;
  uint8_t StopIdMask;
  bool ScheduledStop;
  bool TrackingPosition;
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
     {2250U, 0U, 0U}}, /* At480 pulses/mm:250 mm/s; gear0 is direct speed, HOME keeps drive parameters. */
    {"x",
     MOTOR_ID_ARM_X,
     MECHANICAL_ARM_X_POSITIVE_DIR,
     {40U, 0U, MECHANICAL_ARM_COMMAND_PULSES_PER_REV}}};

static UART_HandleTypeDef *MechanicalArm_Uart;
static uint8_t MechanicalArm_StopPendingMask, MechanicalArm_StopMask;
static MechanicalArm_AxisTypeDef MechanicalArm_StopPendingAxis;
static uint8_t MechanicalArm_StopAllAddress;
static bool MechanicalArm_StopAllFailed;
static MechanicalArm_StateTypeDef MechanicalArm_State;
static MechanicalArm_RequestTypeDef MechanicalArm_Request;
static MechanicalArm_EventTypeDef MechanicalArm_Event;
static volatile uint8_t MechanicalArm_EventReady;
static uint8_t MechanicalArm_Transmitted, MechanicalArm_Acknowledged;
static uint32_t MechanicalArm_TxDoneMs;

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
  MechanicalArm_Event.SpeedRpm = MechanicalArm_Request.SpeedRpm;
  MechanicalArm_Event.TxDoneMs = MechanicalArm_TxDoneMs;
  MechanicalArm_Event.CurrentPosition = MechanicalArm_Request.CurrentPosition;
  MechanicalArm_Event.CurrentSpeedRpm = MechanicalArm_Request.CurrentSpeedRpm;
  MechanicalArm_Event.CurrentPositionRaw = MechanicalArm_Request.CurrentPositionRaw;
  MechanicalArm_Event.HomeMode = MechanicalArm_Request.HomeMode;
  MechanicalArm_Event.MotorCode = MotorCode;
  MechanicalArm_Event.SuccessMask = MechanicalArm_Request.SuccessMask;
  MechanicalArm_Event.FailureMask = MechanicalArm_Request.FailureMask;
  MechanicalArm_Event.Transmitted = MechanicalArm_Transmitted;
  MechanicalArm_Event.Acknowledged = MechanicalArm_Acknowledged;
  MechanicalArm_Event.StopIdMask = MechanicalArm_Request.StopIdMask;
  MechanicalArm_Event.Diagnostic = MechanicalArm_Request.Diagnostic;
  MechanicalArm_Event.Diagnostic.Function = MechanicalArm_Request.ExpectedFunction;
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
  MechanicalArm_TxDoneMs = 0;
  memset(&MechanicalArm_Request.Diagnostic, 0, sizeof MechanicalArm_Request.Diagnostic);
  MechanicalArm_Request.ExpectedFunction = 0;

  AxisConfig = MechanicalArm_AxisConfigGet(MechanicalArm_Request.CurrentAxis);
  if (AxisConfig == NULL)
  {
    return HAL_ERROR;
  }

  if(MechanicalArm_Request.Action==MECHANICAL_ARM_ACTION_ENABLE ||
     MechanicalArm_Request.Action==MECHANICAL_ARM_ACTION_HOME){
    if(MotorBus_IsQuarantined())return HAL_ERROR;
    int ready=MotorBus_PositionReplyEnsure(AxisConfig->MotorId);
    if(ready!=1)return ready<0?HAL_ERROR:HAL_BUSY;
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
    {
      int ready=MotorBus_PositionReplyEnsure(AxisConfig->MotorId);
      if(ready!=1)return ready<0?HAL_ERROR:HAL_BUSY;
    }
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

  case MECHANICAL_ARM_ACTION_SPEED_READ:
    MechanicalArm_Request.ExpectedFunction = 0x35U;
    Emm_V5_Read_Sys_Params(AxisConfig->MotorId, S_VEL);
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
    MechanicalArm_Request.Diagnostic.HomeStage = MECHANICAL_ARM_HOME_TRIGGER;
    if (MechanicalArm_Request.CurrentAxis == MECHANICAL_ARM_AXIS_BASE &&
        MechanicalArm_Request.HomeMode == 1U && MechanicalArm_Request.HomePrep < 3U) {
      uint8_t read[3] = {AxisConfig->MotorId, 0x22, 0x6b};
      bool writing = MechanicalArm_Request.HomePrep == 1U;
      MechanicalArm_Request.Diagnostic.HomeStage = (MechanicalArm_HomeStageTypeDef)
        (MECHANICAL_ARM_HOME_PARAMS_READ + MechanicalArm_Request.HomePrep);
      MechanicalArm_Request.ExpectedFunction = writing ? 0x4c : 0x22;
      if (!MotorBus_Submit(MOTOR_BUS_ARM, writing ? MechanicalArm_Request.HomeParams : read,
                          writing ? 20 : 3, writing ? 4 : 18, 0, false))
        return MotorBus_IsQuarantined() ? HAL_ERROR : HAL_BUSY;
      MechanicalArm_State = MECHANICAL_ARM_STATE_WAIT_TX;
      return HAL_OK;
    }
    MechanicalArm_Request.ExpectedFunction = 0x9AU;
    Emm_V5_Origin_Trigger_Return(AxisConfig->MotorId, MechanicalArm_Request.HomeMode, false);
    break;

  case MECHANICAL_ARM_ACTION_ZERO:
    MechanicalArm_Request.ExpectedFunction = 0x0AU;
    Emm_V5_Reset_CurPos_To_Zero(AxisConfig->MotorId);
    break;

  case MECHANICAL_ARM_ACTION_STOP:
    MechanicalArm_Request.ExpectedFunction = 0xFEU;
    if (MechanicalArm_Request.ScheduledStop)
    {
      /* Routine tracking hold: the same FE frame, queued between complete
       * wheel batches rather than abandoning another owner's Receive ACK. */
      uint8_t stop[5] = {AxisConfig->MotorId, 0xfe, 0x98, 0, 0x6b};
      if (!MotorBus_Submit(MOTOR_BUS_ARM, stop, sizeof stop, 0, 0, false))
        return MotorBus_IsQuarantined() ? HAL_ERROR : HAL_BUSY;
      MechanicalArm_State = MECHANICAL_ARM_STATE_WAIT_TX;
      return HAL_OK;
    }
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
 * 说    明：stop all逐ID1～8发送，TX完成推进，不等待单台应答
 */
static HAL_StatusTypeDef MechanicalArm_StopSend(uint8_t mask,MechanicalArm_AxisTypeDef axis)
{
  MechanicalArm_Transmitted = 0;
  MechanicalArm_Acknowledged = 0;
  memset(&MechanicalArm_Request, 0, sizeof(MechanicalArm_Request));
  MechanicalArm_Request.Action = MECHANICAL_ARM_ACTION_STOP;
  MechanicalArm_Request.RequestedAxis = axis;
  MechanicalArm_Request.CurrentAxis = axis;

  MechanicalArm_StopMask=mask;
  MechanicalArm_Request.StopIdMask=mask;
  MechanicalArm_StopAllAddress=1;
  while(!(mask&(1U<<(MechanicalArm_StopAllAddress-1U))))MechanicalArm_StopAllAddress++;
  MechanicalArm_StopAllFailed=false;
  const uint8_t stop[]={MechanicalArm_StopAllAddress,0xfe,0x98,0,0x6b};
  if(!MotorBus_Submit(MOTOR_BUS_ARM,stop,sizeof stop,0,0,true))return HAL_BUSY;

  MechanicalArm_State = MECHANICAL_ARM_STATE_WAIT_TX;
  return HAL_OK;
}

/**
 * 函    数：开始一个单轴或顺序多轴请求
 * 参    数：Action 动作；Axis 轴；Pulses 相对脉冲
 * 返 回 值：请求接收结果
 * 说    明：all从base开始，后续轴由应答处理函数推进
 */
static MechanicalArm_ResultTypeDef MechanicalArm_RequestStartEx(MechanicalArm_ActionTypeDef Action,
                                                              MechanicalArm_AxisTypeDef Axis,
                                                              int32_t Pulses, uint8_t HomeMode,
                                                              bool ScheduledStop)
{
  MechanicalArm_AxisConfigTypeDef *AxisConfig;
  HAL_StatusTypeDef SubmitResult;

  if (MechanicalArm_Uart == NULL)
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  /* Position scheduling belongs to the task. Readback and stops remain available;
   * homing/zeroing and drive configuration still require an idle chassis. */
  if ((ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) &&
      (Action != MECHANICAL_ARM_ACTION_POSITION_READ) &&
      (Action != MECHANICAL_ARM_ACTION_SPEED_READ) &&
      (Action != MECHANICAL_ARM_ACTION_STATE) && (Action != MECHANICAL_ARM_ACTION_HOME_STATE) &&
      (Action != MECHANICAL_ARM_ACTION_POSITION) && (Action != MECHANICAL_ARM_ACTION_STOP) &&
      (Action != MECHANICAL_ARM_ACTION_HOME_ABORT))
  {
    return MECHANICAL_ARM_RESULT_BUSY;
  }
  if (MechanicalArm_State != MECHANICAL_ARM_STATE_IDLE)
  {
    return MECHANICAL_ARM_RESULT_BUSY;
  }

  /* ALL advances directly on ACK: prepare every axis before accepting it. */
  if(Axis==MECHANICAL_ARM_AXIS_ALL &&
     (Action==MECHANICAL_ARM_ACTION_ENABLE || Action==MECHANICAL_ARM_ACTION_HOME)){
    if(MotorBus_IsQuarantined())return MECHANICAL_ARM_RESULT_TX_ERROR;
    bool ready=true;
    for(unsigned i=MECHANICAL_ARM_AXIS_BASE;i<=MECHANICAL_ARM_AXIS_X;i++){
      MechanicalArm_AxisConfigTypeDef *cfg=MechanicalArm_AxisConfigGet((MechanicalArm_AxisTypeDef)i);
      int r=MotorBus_PositionReplyEnsure(cfg->MotorId);
      if(r<0)return MECHANICAL_ARM_RESULT_TX_ERROR;
      if(!r)ready=false;
    }
    if(!ready)return MECHANICAL_ARM_RESULT_BUSY;
  }

  memset(&MechanicalArm_Request, 0, sizeof(MechanicalArm_Request));
  MechanicalArm_Request.Action = Action;
  MechanicalArm_Request.ScheduledStop = ScheduledStop;
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
  SubmitResult = MechanicalArm_CurrentCommandSend();
  if (SubmitResult != HAL_OK)
  {
    MechanicalArm_State = MECHANICAL_ARM_STATE_IDLE;
    return SubmitResult == HAL_BUSY ? MECHANICAL_ARM_RESULT_BUSY : MECHANICAL_ARM_RESULT_TX_ERROR;
  }

  return MECHANICAL_ARM_RESULT_NONE;
}

static MechanicalArm_ResultTypeDef MechanicalArm_RequestStart(MechanicalArm_ActionTypeDef Action,
                                                              MechanicalArm_AxisTypeDef Axis,
                                                              int32_t Pulses, uint8_t HomeMode)
{
  return MechanicalArm_RequestStartEx(Action, Axis, Pulses, HomeMode, false);
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

  if (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_SPEED_READ)
  {
    int32_t rpm = ((int32_t)Response[3] << 8) | Response[4];
    MechanicalArm_Request.CurrentSpeedRpm = Response[2] ? -rpm : rpm;
    MechanicalArm_Request.SuccessMask |= AxisMask;
    MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_OK, MECHANICAL_ARM_AXIS_INVALID, 0U);
    return;
  }

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
  else if (MotorCode == 0x02U || (MechanicalArm_Request.Action==MECHANICAL_ARM_ACTION_POSITION && MotorCode==0x9fU) ||
           (MechanicalArm_Request.Action==MECHANICAL_ARM_ACTION_HOME &&
            MechanicalArm_Request.HomeMode==0U && MotorCode==0x12U))
  {
    /* Nearest HOME may report already at the saved zero. This only accepts
     * the command result; GrabTask still verifies HOME flags and standstill. */
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
  /* Initialization stays quiet. Enable, HOME and FD prepare verified Receive
   * on demand before their first command; no drive Flash or origin changes. */
  MechanicalArm_StopPendingMask=MechanicalArm_StopMask=0;
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
  MechanicalArm_TxDoneMs = event.tx_ms;
  /* Snapshot the matched transaction before HOME preparation can clear the
   * original command's flags or dispatch the next configuration step. */
  MechanicalArm_Request.Diagnostic.BusEventValid = 1;
  MechanicalArm_Request.Diagnostic.BusResult = (uint8_t)event.result;
  MechanicalArm_Request.Diagnostic.WireTransmitted = (uint8_t)event.tx_completed;
  MechanicalArm_Request.Diagnostic.WireTxMs = event.tx_ms;
  MechanicalArm_Request.Diagnostic.UartError = event.uart_error;
  MechanicalArm_Request.Diagnostic.ReplyReceived = event.result == MOTOR_BUS_REPLY;
  MechanicalArm_Request.Diagnostic.ReplyLength = event.result == MOTOR_BUS_REPLY ? event.length : 0;
  if (event.result == MOTOR_BUS_REPLY) {
    unsigned length = event.length;
    if (length > sizeof MechanicalArm_Request.Diagnostic.Reply)
      length = sizeof MechanicalArm_Request.Diagnostic.Reply;
    memcpy(MechanicalArm_Request.Diagnostic.Reply, event.data, length);
  }
  if (MechanicalArm_StopPendingMask)
  {
    uint8_t mask=MechanicalArm_StopPendingMask;
    MechanicalArm_StopPendingMask=0;
    if (MechanicalArm_StopSend(mask,MechanicalArm_StopPendingAxis) != HAL_OK)
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_TX_ERROR, MECHANICAL_ARM_AXIS_ALL, 0);
    return;
  }
  if(MechanicalArm_Request.Action==MECHANICAL_ARM_ACTION_STOP &&
     !MechanicalArm_Request.ScheduledStop && MechanicalArm_StopMask)
  {
    if(event.result!=MOTOR_BUS_TX_DONE)MechanicalArm_StopAllFailed=true;
    MechanicalArm_StopMask&=(uint8_t)~(1U<<(MechanicalArm_StopAllAddress-1U));
    while(MechanicalArm_StopMask)
    {
      MechanicalArm_StopAllAddress=1;
      while(!(MechanicalArm_StopMask&(1U<<(MechanicalArm_StopAllAddress-1U))))MechanicalArm_StopAllAddress++;
      uint8_t stop[]={MechanicalArm_StopAllAddress,0xfe,0x98,0,0x6b};
      if(MotorBus_Submit(MOTOR_BUS_ARM,stop,sizeof stop,0,0,true))return;
      MechanicalArm_StopAllFailed=true;
      MechanicalArm_StopMask&=(uint8_t)~(1U<<(MechanicalArm_StopAllAddress-1U));
    }
    MechanicalArm_RequestFinish(MechanicalArm_StopAllFailed?MECHANICAL_ARM_RESULT_TX_ERROR:
      MECHANICAL_ARM_RESULT_OK,MECHANICAL_ARM_AXIS_ALL,0);
    return;
  }
  /* Direction HOME preserves the driver's other parameters. No HOME is
   * reported transmitted until configuration has been read back and verified. */
  if (MechanicalArm_Request.Action == MECHANICAL_ARM_ACTION_HOME &&
      MechanicalArm_Request.CurrentAxis == MECHANICAL_ARM_AXIS_BASE &&
      MechanicalArm_Request.HomeMode == 1U && MechanicalArm_Request.HomePrep < 3U) {
    uint8_t phase = MechanicalArm_Request.HomePrep;
    uint8_t *cmd = MechanicalArm_Request.HomeParams;
    MechanicalArm_Transmitted = MechanicalArm_Acknowledged = 0;
    if (event.result != MOTOR_BUS_REPLY ||
        (phase == 1U ? (event.length != 4U || event.data[2] != 2U) : event.length != 18U)) {
      MechanicalArm_RequestFinish(event.result == MOTOR_BUS_TIMEOUT ? MECHANICAL_ARM_RESULT_ACK_TIMEOUT :
        MECHANICAL_ARM_RESULT_MOTOR_ERROR, MECHANICAL_ARM_AXIS_BASE,
        event.result == MOTOR_BUS_REPLY && event.length == 4U ? event.data[2] : 0);
      return;
    }
    if (phase == 0U) {
      cmd[0] = MOTOR_ID_ARM_BASE; cmd[1] = 0x4c; cmd[2] = 0xae; cmd[3] = 0; /* RAM only */
      memcpy(cmd + 4, event.data + 2, 15);
      cmd[4] = 1; cmd[5] = MECHANICAL_ARM_BASE_POSITIVE_DIR ? 0 : 1;
      cmd[19] = 0x6b;
    } else if (phase == 2U && memcmp(cmd + 4, event.data + 2, 15)) {
      for (unsigned i = 0; i < 15; i++) if (cmd[4+i] != event.data[2+i]) {
        MechanicalArm_Request.Diagnostic.VerifyMismatch = 1;
        MechanicalArm_Request.Diagnostic.VerifyOffset = (uint8_t)i;
        MechanicalArm_Request.Diagnostic.VerifyExpected = cmd[4+i];
        MechanicalArm_Request.Diagnostic.VerifyActual = event.data[2+i];
        break;
      }
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_MOTOR_ERROR, MECHANICAL_ARM_AXIS_BASE, 0);
      return;
    }
    MechanicalArm_Request.HomePrep++;
    if (MechanicalArm_CurrentCommandSend() != HAL_OK)
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_TX_ERROR, MECHANICAL_ARM_AXIS_BASE, 0);
    return;
  }
  if (event.result == MOTOR_BUS_REPLY)
  {
    MechanicalArm_Transmitted = 1;
    MechanicalArm_Acknowledged = 1;
    if(event.length==4 && (event.data[2]==0xe2 || event.data[2]==0xee))
      MechanicalArm_RequestFinish(MECHANICAL_ARM_RESULT_MOTOR_ERROR,MechanicalArm_Request.CurrentAxis,event.data[2]);
    else MechanicalArm_ResponseProcess(event.data);
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
static MechanicalArm_ResultTypeDef MechanicalArm_PositionSubmit(MechanicalArm_AxisTypeDef Axis,
                                                     int32_t Position, uint16_t SpeedRpm,
                                                     uint8_t Acceleration,
                                                     MechanicalArm_PositionModeTypeDef Mode,
                                                     bool Tracking)
{
  if ((MechanicalArm_AxisConfigGet(Axis) == NULL) ||
      ((Position == 0) && (Mode != MECHANICAL_ARM_POSITION_ABSOLUTE)) || (SpeedRpm < 1U) ||
      (SpeedRpm > 3000U) ||
      ((Acceleration == 0U) && (Axis != MECHANICAL_ARM_AXIS_X) && (Axis != MECHANICAL_ARM_AXIS_Z)) ||
      ((Mode != MECHANICAL_ARM_POSITION_ABSOLUTE) &&
       (Mode != MECHANICAL_ARM_POSITION_RELATIVE_CURRENT)))
  {
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  }
  if (MechanicalArm_State != MECHANICAL_ARM_STATE_IDLE)
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
  MechanicalArm_Request.TrackingPosition = Tracking;
  HAL_StatusTypeDef submitted = MechanicalArm_CurrentCommandSend();
  if (submitted != HAL_OK)
  {
    MechanicalArm_State = MECHANICAL_ARM_STATE_IDLE;
    return submitted == HAL_BUSY ? MECHANICAL_ARM_RESULT_BUSY : MECHANICAL_ARM_RESULT_TX_ERROR;
  }
  return MECHANICAL_ARM_RESULT_NONE;
}

MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef Axis,
                                                     int32_t Position,uint16_t SpeedRpm,
                                                     uint8_t Acceleration,
                                                     MechanicalArm_PositionModeTypeDef Mode)
{
  return MechanicalArm_PositionSubmit(Axis,Position,SpeedRpm,Acceleration,Mode,false);
}
MechanicalArm_ResultTypeDef MechanicalArm_PositionTrackingEx(MechanicalArm_AxisTypeDef Axis,
                                                     int32_t Position,uint16_t SpeedRpm,
                                                     uint8_t Acceleration,
                                                     MechanicalArm_PositionModeTypeDef Mode)
{
  if(Axis!=MECHANICAL_ARM_AXIS_X || Mode!=MECHANICAL_ARM_POSITION_ABSOLUTE)
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  return MechanicalArm_PositionSubmit(Axis,Position,SpeedRpm,Acceleration,Mode,true);
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

MechanicalArm_ResultTypeDef MechanicalArm_SpeedRead(MechanicalArm_AxisTypeDef Axis)
{
  if (MechanicalArm_AxisConfigGet(Axis) == NULL) return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  return MechanicalArm_RequestStart(MECHANICAL_ARM_ACTION_SPEED_READ, Axis, 0, 0U);
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
 * 函    数：调度单轴正常停保持
 * 参    数：Axis 目标轴
 * 返 回 值：请求接收结果
 * 说    明：追踪取消旧前瞻目标，不抢占其他UART5事务；不用于紧急停车
 */
MechanicalArm_ResultTypeDef MechanicalArm_StopScheduled(MechanicalArm_AxisTypeDef Axis)
{
  if (Axis >= MECHANICAL_ARM_AXIS_ALL)
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  return MechanicalArm_RequestStartEx(MECHANICAL_ARM_ACTION_STOP, Axis, 0, 0, true);
}

/** @brief Request priority single-axis/all-axis stop; ALL also cancels wheel batches. */
MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef Axis)
{
  MotorBus_Event_t discarded;
  uint8_t mask;
  if (Axis > MECHANICAL_ARM_AXIS_ALL || MechanicalArm_Uart == NULL)
    return MECHANICAL_ARM_RESULT_PARAM_ERROR;
  mask=Axis==MECHANICAL_ARM_AXIS_ALL?0xffU:
    (uint8_t)(1U<<(MechanicalArm_AxisConfig[(unsigned)Axis].MotorId-1U));
  /* A scoped stop must not cancel wheel batches. ALL retains IDs1..8. */
  if(Axis==MECHANICAL_ARM_AXIS_ALL)(void)Mecanum_Test_Stop();
  MechanicalArm_EventReady = 0;
  if(MechanicalArm_State!=MECHANICAL_ARM_STATE_IDLE && MechanicalArm_Request.Action==MECHANICAL_ARM_ACTION_STOP &&
     !MechanicalArm_Request.ScheduledStop && !MechanicalArm_StopPendingMask && MechanicalArm_StopMask)
  {
    MechanicalArm_StopMask|=mask;
    MechanicalArm_Request.StopIdMask|=mask;
    if(MechanicalArm_Request.RequestedAxis!=Axis)MechanicalArm_Request.RequestedAxis=MECHANICAL_ARM_AXIS_ALL;
    return MECHANICAL_ARM_RESULT_NONE;
  }
  if (MechanicalArm_State != MECHANICAL_ARM_STATE_IDLE || MotorBus_OwnerBusy(MOTOR_BUS_ARM))
  {
    if(!MechanicalArm_StopPendingMask)MechanicalArm_StopPendingAxis=Axis;
    else if(MechanicalArm_StopPendingAxis!=Axis)MechanicalArm_StopPendingAxis=MECHANICAL_ARM_AXIS_ALL;
    MechanicalArm_StopPendingMask|=mask;
    MotorBus_Cancel(MOTOR_BUS_ARM);
    MechanicalArm_State=MECHANICAL_ARM_STATE_WAIT_TX;
    return MECHANICAL_ARM_RESULT_NONE;
  }
  (void)MotorBus_EventGet(MOTOR_BUS_ARM, &discarded);
  if (MechanicalArm_StopSend(mask,Axis) != HAL_OK)
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
      ((Acceleration == 0U) && (Axis != MECHANICAL_ARM_AXIS_X) && (Axis != MECHANICAL_ARM_AXIS_Z)) ||
      (PulseLimit > MECHANICAL_ARM_COMMAND_PULSES_PER_REV) ||
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
