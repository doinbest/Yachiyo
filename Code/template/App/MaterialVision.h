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

/* 兼容入口仅委托GrabTask；标定采样服务只由GrabTask推进。 */
void MaterialVision_Init(void);
/** @brief 人工确认当前安全观察姿态，设置GrabTask共用RAM参考，不运动或归零。
 * @return OK表示确认完成；BUSY表示任务或机械臂正在运行，参考保持原值。 */
MaterialVision_ResultTypeDef MaterialVision_ReferenceSet(void);
/** @brief 只读查询GrabTask共用观察姿态参考；普通停止保留。 */
uint8_t MaterialVision_IsReferenceValid(void);
/** @brief 按Color(1~6)启动搜索和对准；Base保持当前位置，先确认机械参考。
 * @return OK表示已受理；BUSY不产生请求或运动，其他值表示参数/准备/发送错误。 */
MaterialVision_ResultTypeDef MaterialVision_Start(Camera_ColorTypeDef Color);
/** @brief 启动底盘前后/X响应标定；要求参考有效且视觉任务与电机空闲。 */
MaterialVision_ResultTypeDef MaterialVision_CalibrationStart(
    Camera_ColorTypeDef Color);
void MaterialVision_Stop(void);
/** @brief GrabTask已取得任务所有权、确认停稳后调用；不重复检查GrabTask忙。
 * 采样分母为0x36实际轮位移/X电机轴角度；新尝试失败保留旧有效标定。 */
MaterialVision_ResultTypeDef MaterialVision_CalibrationBegin(Camera_ColorTypeDef Color);
void MaterialVision_CalibrationProcess(void);
uint8_t MaterialVision_CalibrationEvent(
    const MechanicalArm_EventTypeDef *Event);
void MaterialVision_CalibrationCancel(void);
/* 以下查询均只读；CalibrationEvent返回1表示消费该标定电机事件。 */
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
