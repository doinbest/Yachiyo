/**
  ******************************************************************************
  * @file    Camera.h
  * @brief   摄像头识别指令和坐标数据接口
  ******************************************************************************
  */
#ifndef __CAMERA_H
#define __CAMERA_H

#include "main.h"

#define CAMERA_RX_BUFFER_SIZE 256U /* USB接收暂存容量，可容纳多个64字节包。 */
#define CAMERA_FRAME_SIZE     14U  /* 摄像头固定返回帧长度。 */
#define CAMERA_DATA_STALE_MS  1200U

typedef enum
{
  CAMERA_COLOR_RED        = 0x01,
  CAMERA_COLOR_YELLOW     = 0x02,
  CAMERA_COLOR_BLUE       = 0x03,
  CAMERA_COLOR_GREEN      = 0x04,
  CAMERA_COLOR_BLACK      = 0x05,
  CAMERA_COLOR_LIGHT_BLUE = 0x06
} Camera_ColorTypeDef;

typedef struct
{
  uint8_t Function;
  uint8_t Target;
  uint16_t CX;
  uint16_t CY;
  int16_t DX;
  int16_t DY;
  uint32_t Tick;
  uint32_t Sequence;
} Camera_DataTypeDef;

typedef struct
{
  Camera_DataTypeDef Data; /**< 历史最后有效坐标；切目标后也保留，不能单靠它判断有效。 */
  uint8_t RequestActive;
  uint8_t RequestFunction;
  uint8_t RequestTarget;
  uint8_t UsbConfigured;
  uint8_t HasFrame; /**< 当前请求至少收到一帧Valid=0或1的匹配报文。 */
  uint8_t TargetValid; /**< 最近匹配报文的Valid值，尚无报文时为0。 */
  uint8_t HasValidData; /**< 当前快照可使用Data；无效帧/切目标/停止后清零。 */
  uint32_t LastFrameTick; /**< 最近匹配报文解析时刻，ms；包含Valid=0。 */
  uint32_t RxByteCount; /**< USB接收字节累计数，包含未匹配和被丢弃的字节。 */
  uint32_t ChecksumErrorCount; /**< 头尾符合格式的候选帧累加校验失败数。 */
  uint32_t OverflowCount; /**< USB暂存区或解析拼接缓冲区溢出次数。 */
} Camera_SnapshotTypeDef;

typedef enum
{
  CAMERA_VIS_IDLE = 0,
  CAMERA_VIS_OFF,
  CAMERA_VIS_WAIT,
  CAMERA_VIS_OK,
  CAMERA_VIS_LOST,
  CAMERA_VIS_STALE
} Camera_VisualStateTypeDef;

/* @brief 在MX_USB_DEVICE_Init之前清空协议状态。 */
HAL_StatusTypeDef Camera_Init(void);
/* @brief 主循环发送FF B2 Color FF；USB未就绪返回ERROR，忙返回BUSY，可稍后重试。 */
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef Color);
void Camera_RequestStop(void);
void Camera_Process(void);
/** @brief 消费一次尚未读取的有效坐标；getter不会发送请求或访问USB。 */
uint8_t Camera_DataGet(Camera_DataTypeDef *Data);
/** @brief 主循环非消费式读取诊断快照；NULL不操作，不推进任务或发送数据。 */
void Camera_SnapshotGet(Camera_SnapshotTypeDef *Snapshot);
/** @brief 按当前HAL毫秒时钟判断显示状态；不触发超时停车。 */
Camera_VisualStateTypeDef Camera_VisualStateGet(const Camera_SnapshotTypeDef *Snapshot);
const char *Camera_VisualStateNameGet(Camera_VisualStateTypeDef State);
/* @brief USB中断只追加字节；主循环调用Camera_Process解析。 */
void Camera_UsbRxCallback(const uint8_t *Data, uint32_t Size);

#endif /* __CAMERA_H */
