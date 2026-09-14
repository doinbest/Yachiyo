#ifndef __MECHANICAL_ARM_H
#define __MECHANICAL_ARM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

  typedef enum
  {
    MECHANICAL_ARM_AXIS_BASE = 0,
    MECHANICAL_ARM_AXIS_Z,
    MECHANICAL_ARM_AXIS_X,
    MECHANICAL_ARM_AXIS_ALL,
    MECHANICAL_ARM_AXIS_INVALID
  } MechanicalArm_AxisTypeDef;

  typedef enum
  {
    MECHANICAL_ARM_ACTION_NONE = 0,
    MECHANICAL_ARM_ACTION_ENABLE,
    MECHANICAL_ARM_ACTION_DISABLE,
    MECHANICAL_ARM_ACTION_POSITION,
    MECHANICAL_ARM_ACTION_POSITION_READ,
    MECHANICAL_ARM_ACTION_STATE,
    MECHANICAL_ARM_ACTION_ORIGIN_SET,
    MECHANICAL_ARM_ACTION_HOME,
    MECHANICAL_ARM_ACTION_ZERO,
    MECHANICAL_ARM_ACTION_STOP
  } MechanicalArm_ActionTypeDef;

  typedef enum
  {
    MECHANICAL_ARM_POSITION_ABSOLUTE = 1U,
    MECHANICAL_ARM_POSITION_RELATIVE_CURRENT = 2U
  } MechanicalArm_PositionModeTypeDef;

  typedef enum
  {
    MECHANICAL_ARM_RESULT_NONE = 0,
    MECHANICAL_ARM_RESULT_OK,
    MECHANICAL_ARM_RESULT_BUSY,
    MECHANICAL_ARM_RESULT_PARAM_ERROR,
    MECHANICAL_ARM_RESULT_TX_ERROR,
    MECHANICAL_ARM_RESULT_MOTOR_ERROR,
    MECHANICAL_ARM_RESULT_ACK_TIMEOUT
  } MechanicalArm_ResultTypeDef;

  typedef struct
  {
    uint16_t SpeedRpm;
    uint8_t Acceleration;
    uint32_t PulseLimit;
  } MechanicalArm_ConfigTypeDef;

  typedef struct
  {
    MechanicalArm_ActionTypeDef Action;
    MechanicalArm_AxisTypeDef Axis;
    MechanicalArm_AxisTypeDef FailureAxis;
    MechanicalArm_ResultTypeDef Result;
    int32_t PositionPulses;
    int32_t CurrentPosition;
    int64_t
        CurrentPositionRaw; /* Full signed 0x36 protocol payload; inspect on legacy range error. */
    uint8_t HomeMode;
    uint8_t MotorCode;
    uint8_t StateFlags[3];
    uint8_t SuccessMask;
    uint8_t FailureMask;
    uint8_t Transmitted;  /* Complete TX observed; never inferred from accepted request. */
    uint8_t Acknowledged; /* Matched reply; stop uses TX-only and leaves this zero. */
  } MechanicalArm_EventTypeDef;

  HAL_StatusTypeDef MechanicalArm_Init(UART_HandleTypeDef *huart);
  void MechanicalArm_Process(void);
  MechanicalArm_ResultTypeDef MechanicalArm_Enable(MechanicalArm_AxisTypeDef Axis);
  MechanicalArm_ResultTypeDef MechanicalArm_Disable(MechanicalArm_AxisTypeDef Axis);
  MechanicalArm_ResultTypeDef MechanicalArm_Position(MechanicalArm_AxisTypeDef Axis,
                                                     int32_t Pulses);
  MechanicalArm_ResultTypeDef MechanicalArm_PositionEx(MechanicalArm_AxisTypeDef Axis,
                                                       int32_t Position, uint16_t SpeedRpm,
                                                       uint8_t Acceleration,
                                                       MechanicalArm_PositionModeTypeDef Mode);
  MechanicalArm_ResultTypeDef MechanicalArm_PositionRead(MechanicalArm_AxisTypeDef Axis);
  MechanicalArm_ResultTypeDef MechanicalArm_StateRead(MechanicalArm_AxisTypeDef Axis);
  /* 将当前位置保存为张大头驱动器的单圈机械零度，不会驱动电机运动。 */
  MechanicalArm_ResultTypeDef MechanicalArm_OriginSet(MechanicalArm_AxisTypeDef Axis);
  MechanicalArm_ResultTypeDef MechanicalArm_Home(MechanicalArm_AxisTypeDef Axis, uint8_t HomeMode);
  MechanicalArm_ResultTypeDef MechanicalArm_Zero(MechanicalArm_AxisTypeDef Axis);
  MechanicalArm_ResultTypeDef MechanicalArm_Stop(MechanicalArm_AxisTypeDef Axis);
  /* Stop(all) preempts active request; single-axis stop retains the existing busy guard.
 * Stop
   * results report completed TX only, remain available after ACK quarantine. */
  uint8_t MechanicalArm_ConfigSet(MechanicalArm_AxisTypeDef Axis, uint16_t SpeedRpm,
                                  uint8_t Acceleration, uint32_t PulseLimit);
  uint8_t MechanicalArm_ConfigGet(MechanicalArm_AxisTypeDef Axis,
                                  MechanicalArm_ConfigTypeDef *Config);
  uint8_t MechanicalArm_ResultGet(MechanicalArm_EventTypeDef *Event);
  uint8_t MechanicalArm_IsBusy(void);
  MechanicalArm_AxisTypeDef MechanicalArm_AxisGet(const char *Name);
  const char *MechanicalArm_AxisNameGet(MechanicalArm_AxisTypeDef Axis);
  void MechanicalArm_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
  void MechanicalArm_TxCpltCallback(UART_HandleTypeDef *huart);
  void MechanicalArm_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
