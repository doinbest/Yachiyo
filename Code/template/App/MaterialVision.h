#ifndef MATERIAL_VISION_H
#define MATERIAL_VISION_H

#include "Camera.h"
#include "mechanical_arm.h"

typedef enum
{
  MATERIAL_VISION_STATE_IDLE = 0,
  MATERIAL_VISION_STATE_SEARCH,
  MATERIAL_VISION_STATE_TARGET_LOCK,
  MATERIAL_VISION_STATE_COLLECT,
  MATERIAL_VISION_STATE_CHASSIS_MOVE,
  MATERIAL_VISION_STATE_X_ACK,
  MATERIAL_VISION_STATE_X_ARRIVAL,
  MATERIAL_VISION_STATE_X_STATE,
  MATERIAL_VISION_STATE_SETTLE,
  MATERIAL_VISION_STATE_ALIGNED,
  MATERIAL_VISION_STATE_ERROR
} MaterialVision_StateTypeDef;

typedef enum
{
  MATERIAL_VISION_RESULT_OK = 0,
  MATERIAL_VISION_RESULT_BUSY,
  MATERIAL_VISION_RESULT_PARAM_ERROR,
  MATERIAL_VISION_RESULT_NOT_READY,
  MATERIAL_VISION_RESULT_ERROR
} MaterialVision_ResultTypeDef;

typedef enum
{
  MATERIAL_VISION_ERROR_NONE,
  MATERIAL_VISION_ERROR_BUSY,
  MATERIAL_VISION_ERROR_NOT_READY,
  MATERIAL_VISION_ERROR_CAMERA_FIRST_TIMEOUT,
  MATERIAL_VISION_ERROR_CAMERA_STALE,
  MATERIAL_VISION_ERROR_CAMERA_TX,
  MATERIAL_VISION_ERROR_MOTOR_ACK_TIMEOUT,
  MATERIAL_VISION_ERROR_MOTOR_ARRIVAL_TIMEOUT,
  MATERIAL_VISION_ERROR_MOTOR_ERROR,
  MATERIAL_VISION_ERROR_SEARCH_TIMEOUT,
  MATERIAL_VISION_ERROR_RESPONSE_INVALID,
  MATERIAL_VISION_ERROR_CORRECTION_ZERO,
  MATERIAL_VISION_ERROR_X_LIMIT,
  MATERIAL_VISION_ERROR_TASK_TIMEOUT,
  MATERIAL_VISION_ERROR_INTERNAL,
  MATERIAL_VISION_ERROR_CHASSIS_TX,
  MATERIAL_VISION_ERROR_MOTOR_BUSY,
  MATERIAL_VISION_ERROR_MOTOR_PARAM,
  MATERIAL_VISION_ERROR_MOTOR_TX,
  MATERIAL_VISION_ERROR_CHASSIS_LIMIT,
  MATERIAL_VISION_ERROR_CAMERA_BUSY,
  MATERIAL_VISION_ERROR_USB_OFF
} MaterialVision_ErrorTypeDef;

typedef struct
{
  /* 响应单位分别为像素/mm和像素/电机轴角度(deg)，读取不消费标定。 */
  float ForwardDxPerMm;
  float ForwardDyPerMm;
  float XDxPerDegree;
  float XDyPerDegree;
} MaterialVision_CalibrationDataTypeDef;

/* 主循环先初始化Camera和机械臂；Process推进任务，查询接口不消费数据。 */
void MaterialVision_Init(void);
/** @brief 按Color(1~6)启动搜索和对准；Base保持当前位置，先确认机械参考。
 * @return OK表示已受理；BUSY不产生请求或运动，其他值表示参数/准备/发送错误。 */
MaterialVision_ResultTypeDef MaterialVision_Start(Camera_ColorTypeDef Color);
/** @brief 启动底盘前后/X响应标定；要求参考有效且两套视觉与电机空闲。 */
MaterialVision_ResultTypeDef MaterialVision_CalibrationStart(
    Camera_ColorTypeDef Color);
/** @brief 主循环推进；测试模式不因高层视觉超时自动退出，电机超时保留。 */
void MaterialVision_Process(void);
void MaterialVision_Stop(void);
uint8_t MaterialVision_MotorEventHandle(
    const MechanicalArm_EventTypeDef *Event);
/* 以下状态/错误/标定查询均只读；MotorEventHandle返回1表示消费该电机事件。 */
uint8_t MaterialVision_IsBusy(void);
uint8_t MaterialVision_IsCalibrated(void);
uint8_t MaterialVision_IsCalibrating(void);
MaterialVision_ErrorTypeDef MaterialVision_ErrorGet(void);
MaterialVision_StateTypeDef MaterialVision_StateGet(void);
const char *MaterialVision_StateNameGet(void);
const char *MaterialVision_ErrorNameGet(void);
uint8_t MaterialVision_CalibrationGet(
    MaterialVision_CalibrationDataTypeDef *Data);

#endif /* MATERIAL_VISION_H */
