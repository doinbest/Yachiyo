#ifndef __ARM_VISION_H
#define __ARM_VISION_H

#include "Camera.h"
#include "mechanical_arm.h"

typedef enum
{
  ARM_VISION_JOB_ALIGN_ONLY = 0,
  ARM_VISION_JOB_PICK_MATERIAL = 2 /* 1为已退役B3任务，保持既有编号。 */
} ArmVision_JobTypeDef;

typedef enum
{
  ARM_VISION_RESULT_OK = 0,
  ARM_VISION_RESULT_BUSY,
  ARM_VISION_RESULT_NOT_READY,
  ARM_VISION_RESULT_PARAM_ERROR,
  ARM_VISION_RESULT_ERROR
} ArmVision_ResultTypeDef;

typedef enum
{
  ARM_VISION_CALIBRATION_NONE = 0,
  ARM_VISION_CALIBRATION_MATERIAL = 2 /* 1为已退役B3来源。 */
} ArmVision_CalibrationSourceTypeDef;

typedef enum
{
  ARM_VISION_ERROR_NONE = 0,
  ARM_VISION_ERROR_CAMERA_FIRST_TIMEOUT,
  ARM_VISION_ERROR_CAMERA_STALE,
  ARM_VISION_ERROR_CAMERA_TX,
  ARM_VISION_ERROR_MOTOR_TX,
  ARM_VISION_ERROR_MOTOR_BUSY,
  ARM_VISION_ERROR_MOTOR_PARAM,
  ARM_VISION_ERROR_MOTOR_POSITION_ACK_TIMEOUT,
  ARM_VISION_ERROR_MOTOR_STATE_ACK_TIMEOUT,
  ARM_VISION_ERROR_MOTOR_ERROR,
  ARM_VISION_ERROR_MOTOR_ARRIVAL_TIMEOUT,
  ARM_VISION_ERROR_CORRECTION_LIMIT,
  ARM_VISION_ERROR_CORRECTION_ZERO,
  ARM_VISION_ERROR_MATRIX_INVALID,
  ARM_VISION_ERROR_VISION_TIMEOUT,
  ARM_VISION_ERROR_INTERNAL,
  ARM_VISION_ERROR_CAMERA_BUSY,
  ARM_VISION_ERROR_USB_OFF
} ArmVision_ErrorTypeDef;

typedef struct
{
  uint16_t BasePulses;
  int16_t BaseDx;
  int16_t BaseDy;
  uint16_t XPulses;
  int16_t XDx;
  int16_t XDy;
} ArmVision_CalibrationDataTypeDef;

typedef struct
{
  int16_t BaseDx;
  int16_t BaseDy;
  int16_t XDx;
  int16_t XDy;
  float Determinant;
} ArmVision_CalibrationDebugDataTypeDef;

typedef struct
{
  MechanicalArm_AxisTypeDef Axis;
  uint8_t MotorCode;
  uint8_t StateFlags;
} ArmVision_ErrorInfoTypeDef;

typedef struct
{
  uint8_t CorrectionCount;
  int32_t BaseTotal;
  int32_t XTotal;
  int16_t Dx;
  int16_t Dy;
} ArmVision_ProgressTypeDef;

typedef struct
{
  MechanicalArm_AxisTypeDef Axis;
  int32_t Pulses;
  int16_t Dx;
  int16_t Dy;
  uint8_t CorrectionCount;
} ArmVision_MoveDebugDataTypeDef;

/** @brief 上电初始化；参考和标定默认无效。先初始化Camera和机械臂接口。 */
void ArmVision_Init(void);
/** @brief 仅确认当前参考位置，不运动；任何视觉任务或电机忙时返回BUSY。 */
ArmVision_ResultTypeDef ArmVision_ReferenceSet(void);
ArmVision_ResultTypeDef ArmVision_MaterialCalibrationStart(
    Camera_ColorTypeDef Color);
/** @brief B2颜色1~6的Base/X自动对准，需要有效参考和二维运动标定。
 * @param Job 当前只接受ALIGN_ONLY；OK表示受理，BUSY不切换目标或启动运动。 */
ArmVision_ResultTypeDef ArmVision_MaterialStart(Camera_ColorTypeDef Color,
                                                ArmVision_JobTypeDef Job);
/** @brief 主循环推进，按各自序号读取Camera快照；保留原首帧/失新/运动超时。 */
void ArmVision_Process(void);
void ArmVision_Stop(void);
uint8_t ArmVision_MotorEventHandle(const MechanicalArm_EventTypeDef *Event);
uint8_t ArmVision_IsBusy(void);
uint8_t ArmVision_IsReferenceValid(void);
uint8_t ArmVision_IsCalibrated(void);
uint8_t ArmVision_IsCalibrating(void);
ArmVision_CalibrationSourceTypeDef ArmVision_CalibrationSourceGet(void);
const char *ArmVision_CalibrationSourceNameGet(void);
uint8_t ArmVision_CalibrationTargetGet(void);
uint8_t ArmVision_CalibrationResultGet(
    ArmVision_CalibrationDataTypeDef *Data);
uint8_t ArmVision_CalibrationDebugGet(
    ArmVision_CalibrationDebugDataTypeDef *Data);
ArmVision_ErrorTypeDef ArmVision_ErrorGet(void);
uint8_t ArmVision_ErrorInfoGet(ArmVision_ErrorInfoTypeDef *Info);
const char *ArmVision_ErrorNameGet(void);
const char *ArmVision_StateNameGet(void);
uint32_t ArmVision_StatusSequenceGet(void);
uint8_t ArmVision_ProgressGet(ArmVision_ProgressTypeDef *Data);
uint8_t ArmVision_MoveDebugGet(ArmVision_MoveDebugDataTypeDef *Data);

#endif /* __ARM_VISION_H */
