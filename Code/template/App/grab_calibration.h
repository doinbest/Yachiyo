/** @file grab_calibration.h
 * @brief GrabTask-owned forward/X visual response sampling. No independent task/stop owner.
 */
#ifndef GRAB_CALIBRATION_H
#define GRAB_CALIBRATION_H
#include "Camera.h"
#include "mechanical_arm.h"

typedef enum
{
  GRAB_CAL_STATE_IDLE = 0,
  GRAB_CAL_STATE_COLLECT,
  GRAB_CAL_STATE_CHASSIS_MOVE,
  GRAB_CAL_STATE_X_ACK,
  GRAB_CAL_STATE_X_ARRIVAL,
  GRAB_CAL_STATE_X_STATE,
  GRAB_CAL_STATE_SETTLE,
  GRAB_CAL_STATE_ALIGNED,
  GRAB_CAL_STATE_ERROR
} GrabCalibration_StateTypeDef;

typedef enum
{
  GRAB_CAL_RESULT_OK = 0,
  GRAB_CAL_RESULT_BUSY,
  GRAB_CAL_RESULT_PARAM_ERROR,
  GRAB_CAL_RESULT_ERROR
} GrabCalibration_ResultTypeDef;

typedef enum
{
  GRAB_CAL_ERROR_NONE,
  GRAB_CAL_ERROR_CAMERA_TX,
  GRAB_CAL_ERROR_MOTOR_ACK_TIMEOUT,
  GRAB_CAL_ERROR_MOTOR_ARRIVAL_TIMEOUT,
  GRAB_CAL_ERROR_MOTOR_ERROR,
  GRAB_CAL_ERROR_RESPONSE_INVALID,
  GRAB_CAL_ERROR_INTERNAL,
  GRAB_CAL_ERROR_CHASSIS_TX,
  GRAB_CAL_ERROR_MOTOR_BUSY,
  GRAB_CAL_ERROR_MOTOR_PARAM,
  GRAB_CAL_ERROR_MOTOR_TX,
  GRAB_CAL_ERROR_CAMERA_BUSY,
  GRAB_CAL_ERROR_USB_OFF
} GrabCalibration_ErrorTypeDef;

typedef struct
{
  /* 响应单位分别为像素/mm和像素/电机轴角度(deg)，读取不消费标定。 */
  float ForwardDxPerMm;
  float ForwardDyPerMm;
  float XDxPerDegree;
  float XDyPerDegree;
} GrabCalibration_CalibrationDataTypeDef;

/** @brief Initialize RAM samples at boot; never use to cancel a task. */
void GrabCalibration_Init(void);
/** @brief Start sampling after GrabTask confirms pose, bounds and standstill. */
GrabCalibration_ResultTypeDef GrabCalibration_Start(Camera_ColorTypeDef color);
/** @brief Advance samples/moves in the cooperative loop under GrabTask ownership. */
void GrabCalibration_Process(void);
/** @brief Return 1 only for an event consumed by the current X sampling request. */
uint8_t GrabCalibration_MotorEventHandle(const MechanicalArm_EventTypeDef *event);
/** @brief Cancel pending samples; retain prior valid data. GrabTask owns physical parking. */
void GrabCalibration_Cancel(void);
/* Read-only diagnostics; no device requests and no sample consumption. */
GrabCalibration_StateTypeDef GrabCalibration_StateGet(void);
const char *GrabCalibration_StateNameGet(void);
const char *GrabCalibration_ErrorNameGet(void);
uint8_t GrabCalibration_IsCalibrated(void);
/** @brief Read last successful response (pixels/mm and pixels/motor-degree). */
uint8_t GrabCalibration_DataGet(GrabCalibration_CalibrationDataTypeDef *data);
#endif
