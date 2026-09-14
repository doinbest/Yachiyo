/**
  ******************************************************************************
  * @file    Camera.c
  * @brief   香橙派USB CDC识别指令和字节流解析
  ******************************************************************************
  */
#include "Camera.h"
#include "usbd_cdc_if.h"

#include <string.h>

static uint8_t Camera_InputBuffer[CAMERA_RX_BUFFER_SIZE];
static volatile uint16_t Camera_InputLength;
static volatile uint8_t Camera_InputOverflow;
static uint8_t Camera_StreamBuffer[CAMERA_RX_BUFFER_SIZE];
static uint16_t Camera_StreamLength;
static uint8_t Camera_RequestFunction;
static uint8_t Camera_RequestTarget;
static uint8_t Camera_RequestActive;
static uint8_t Camera_HasFrame;
static uint8_t Camera_TargetValid;
static uint8_t Camera_HasValidData;
static Camera_DataTypeDef Camera_LatestData;
static volatile uint8_t Camera_DataReady;
static uint32_t Camera_DataSequence;
static uint32_t Camera_LastFrameTick;
static volatile uint32_t Camera_RxByteCount;
static uint32_t Camera_ChecksumErrorCount;
static volatile uint32_t Camera_OverflowCount;

/**
  * 函    数：发送一次摄像头识别请求
  * 参    数：Function 功能码；Target 颜色或圆环编号
  * 返 回 值：HAL执行状态
  * 说    明：请求格式固定为FF Function Target FF，发送后持续接收返回帧
  */
static HAL_StatusTypeDef Camera_RequestStart(uint8_t Function, uint8_t Target)
{
  uint8_t TxData[4];
  uint8_t Status;
  uint32_t Primask;

  TxData[0] = 0xFFU;
  TxData[1] = Function;
  TxData[2] = Target;
  TxData[3] = 0xFFU;
  /* CDC_Transmit_FS复制到持久缓冲区，提交和请求状态更新之间禁止USB中断。 */
  Primask = __get_PRIMASK();
  __disable_irq();
  Status = CDC_Transmit_FS(TxData, sizeof(TxData));
  if (Status == USBD_OK)
  {
    Camera_InputLength = 0U;
    Camera_InputOverflow = 0U;
    Camera_StreamLength = 0U;
    Camera_DataReady = 0U;
    Camera_RequestFunction = Function;
    Camera_RequestTarget = Target;
    Camera_RequestActive = 1U;
    Camera_HasFrame = 0U;
    Camera_TargetValid = 0U;
    Camera_HasValidData = 0U;
  }
  __set_PRIMASK(Primask);
  if (Status != USBD_OK)
  {
    return (Status == USBD_BUSY) ? HAL_BUSY : HAL_ERROR;
  }
  return HAL_OK;
}

/**
  * 函    数：计算一帧摄像头数据的累加校验
  * 参    数：Frame 14字节候选帧
  * 返 回 值：下标1至11累加后的低8位
  * 说    明：不包含包头、SUM自身和包尾
  */
static uint8_t Camera_ChecksumGet(const uint8_t *Frame)
{
  uint8_t Index;
  uint8_t Sum;

  Sum = 0U;
  for (Index = 1U; Index <= 11U; Index++)
  {
    Sum = (uint8_t)(Sum + Frame[Index]);
  }
  return Sum;
}

/**
  * 函    数：删除字节流缓冲区头部数据
  * 参    数：Length 需要删除的字节数
  * 返 回 值：无
  * 说    明：错误候选帧只删除一个字节，以便从下一个FF重新同步
  */
static void Camera_StreamRemove(uint16_t Length)
{
  if (Length >= Camera_StreamLength)
  {
    Camera_StreamLength = 0U;
    return;
  }
  (void)memmove(Camera_StreamBuffer,
                &Camera_StreamBuffer[Length],
                Camera_StreamLength - Length);
  Camera_StreamLength = (uint16_t)(Camera_StreamLength - Length);
}

/**
  * 函    数：发布一帧合法坐标
  * 参    数：Frame 已通过协议校验的14字节数据帧
  * 返 回 值：无
  * 说    明：中心坐标为大端无符号16位，误差为大端有符号16位；Valid为01才发布
  */
static void Camera_DataPublish(const uint8_t *Frame)
{
  Camera_DataSequence++;
  Camera_LatestData.Function = Frame[1];
  Camera_LatestData.Target = Frame[2];
  Camera_LatestData.CX = (uint16_t)(((uint16_t)Frame[4] << 8U) | Frame[5]);
  Camera_LatestData.CY = (uint16_t)(((uint16_t)Frame[6] << 8U) | Frame[7]);
  Camera_LatestData.DX = (int16_t)(((uint16_t)Frame[8] << 8U) | Frame[9]);
  Camera_LatestData.DY = (int16_t)(((uint16_t)Frame[10] << 8U) | Frame[11]);
  Camera_LatestData.Tick = HAL_GetTick();
  Camera_LatestData.Sequence = Camera_DataSequence;
  Camera_DataReady = 1U;
}

/**
  * 函    数：初始化摄像头通信模块
  * 参    数：无
  * 返 回 值：HAL执行状态
  * 说    明：USB启动前清空接收和请求状态
  */
HAL_StatusTypeDef Camera_Init(void)
{
  Camera_InputLength = 0U;
  Camera_InputOverflow = 0U;
  Camera_StreamLength = 0U;
  Camera_RequestFunction = 0U;
  Camera_RequestTarget = 0U;
  Camera_RequestActive = 0U;
  Camera_HasFrame = 0U;
  Camera_TargetValid = 0U;
  Camera_HasValidData = 0U;
  Camera_DataReady = 0U;
  Camera_DataSequence = 0U;
  Camera_LastFrameTick = 0U;
  Camera_RxByteCount = 0U;
  Camera_ChecksumErrorCount = 0U;
  Camera_OverflowCount = 0U;
  (void)memset(&Camera_LatestData, 0, sizeof(Camera_LatestData));
  return HAL_OK;
}

/**
  * 函    数：启动指定颜色物料识别
  * 参    数：Color 颜色枚举
  * 返 回 值：HAL执行状态
  * 说    明：只发送一次FF B2 Color FF
  */
HAL_StatusTypeDef Camera_MaterialStart(Camera_ColorTypeDef Color)
{
  if ((Color < CAMERA_COLOR_RED) || (Color > CAMERA_COLOR_LIGHT_BLUE))
  {
    return HAL_ERROR;
  }
  return Camera_RequestStart(0xB2U, (uint8_t)Color);
}

/**
  * 函    数：启动指定圆环识别
  * 参    数：Ring 圆环枚举
  * 返 回 值：HAL执行状态
  * 说    明：当前Python不支持B3圆环编号识别，明确返回HAL_ERROR
  */
HAL_StatusTypeDef Camera_RingStart(Camera_RingTypeDef Ring)
{
  /* change.py通过B2自动从粗定位切换到色环精定位，没有B3编号识别。 */
  (void)Ring;
  return HAL_ERROR;
}

/**
  * 函    数：停止当前摄像头请求
  * 参    数：无
  * 返 回 值：无
  * 说    明：只清除软件请求状态，不向摄像头发送额外停止帧
  */
void Camera_RequestStop(void)
{
  uint32_t Primask;

  Primask = __get_PRIMASK();
  __disable_irq();
  Camera_RequestFunction = 0U;
  Camera_RequestTarget = 0U;
  Camera_RequestActive = 0U;
  Camera_HasFrame = 0U;
  Camera_TargetValid = 0U;
  Camera_HasValidData = 0U;
  Camera_DataReady = 0U;
  Camera_InputLength = 0U;
  Camera_StreamLength = 0U;
  __set_PRIMASK(Primask);
}

/**
  * 函    数：处理USB CDC字节流
  * 参    数：无
  * 返 回 值：无
  * 说    明：支持半帧、整帧和多帧粘包，错误帧从下一个FF重新同步
  */
void Camera_Process(void)
{
  uint8_t InputCopy[CAMERA_RX_BUFFER_SIZE];
  uint16_t InputLength;
  uint16_t CopyLength;
  uint32_t Primask;

  Primask = __get_PRIMASK();
  __disable_irq();
  InputLength = Camera_InputLength;
  if (InputLength > 0U)
  {
    (void)memcpy(InputCopy, Camera_InputBuffer, InputLength);
  }
  Camera_InputLength = 0U;
  if (Camera_InputOverflow != 0U)
  {
    Camera_StreamLength = 0U;
    Camera_InputOverflow = 0U;
  }
  __set_PRIMASK(Primask);

  if (InputLength > 0U)
  {
    CopyLength = InputLength;
    if ((uint16_t)(Camera_StreamLength + CopyLength) > CAMERA_RX_BUFFER_SIZE)
    {
      Camera_StreamLength = 0U;
      Primask = __get_PRIMASK();
      __disable_irq();
      Camera_OverflowCount++; /* 与USB回调共享计数，保护读改写。 */
      __set_PRIMASK(Primask);
    }
    (void)memcpy(&Camera_StreamBuffer[Camera_StreamLength], InputCopy, CopyLength);
    Camera_StreamLength = (uint16_t)(Camera_StreamLength + CopyLength);
  }

  while (Camera_StreamLength >= CAMERA_FRAME_SIZE)
  {
    if (Camera_StreamBuffer[0] != 0xFFU)
    {
      Camera_StreamRemove(1U);
      continue;
    }
    if (Camera_StreamBuffer[13] != 0xFEU)
    {
      Camera_StreamRemove(1U);
      continue;
    }
    if (Camera_StreamBuffer[12] != Camera_ChecksumGet(Camera_StreamBuffer))
    {
      Camera_ChecksumErrorCount++;
      Camera_StreamRemove(1U);
      continue;
    }
    if ((Camera_StreamBuffer[1] == Camera_RequestFunction) &&
        (Camera_StreamBuffer[2] == Camera_RequestTarget) &&
        (Camera_RequestActive != 0U) && (Camera_StreamBuffer[3] <= 0x01U))
    {
      Camera_HasFrame = 1U;
      Camera_LastFrameTick = HAL_GetTick();
      if (Camera_StreamBuffer[3] == 0x01U)
      {
        Camera_TargetValid = 1U;
        Camera_HasValidData = 1U;
        Camera_DataPublish(Camera_StreamBuffer);
      }
      else if (Camera_StreamBuffer[3] == 0x00U)
      {
        Camera_TargetValid = 0U;
        Camera_HasValidData = 0U;
        Camera_DataReady = 0U;
      }
    }
    Camera_StreamRemove(CAMERA_FRAME_SIZE);
  }
}

/**
  * 函    数：读取最新摄像头坐标
  * 参    数：Data 数据输出地址
  * 返 回 值：1表示取得新数据，0表示暂无新数据
  * 说    明：读取后清除新数据标志，当前识别请求保持运行
  */
uint8_t Camera_DataGet(Camera_DataTypeDef *Data)
{
  uint32_t Primask;
  uint8_t Ready;

  if (Data == NULL)
  {
    return 0U;
  }
  Ready = 0U;
  Primask = __get_PRIMASK();
  __disable_irq();
  if (Camera_DataReady != 0U)
  {
    *Data = Camera_LatestData;
    Camera_DataReady = 0U;
    Ready = 1U;
  }
  __set_PRIMASK(Primask);
  return Ready;
}

void Camera_SnapshotGet(Camera_SnapshotTypeDef *Snapshot)
{
  uint32_t Primask;

  if (Snapshot == NULL)
  {
    return;
  }
  Primask = __get_PRIMASK();
  __disable_irq();
  Snapshot->Data = Camera_LatestData;
  Snapshot->RequestActive = Camera_RequestActive;
  Snapshot->RequestFunction = Camera_RequestFunction;
  Snapshot->RequestTarget = Camera_RequestTarget;
  Snapshot->UsbConfigured = CDC_IsConfigured_FS();
  Snapshot->HasFrame = Camera_HasFrame;
  Snapshot->TargetValid = Camera_TargetValid;
  Snapshot->HasValidData = Camera_HasValidData;
  Snapshot->LastFrameTick = Camera_LastFrameTick;
  Snapshot->RxByteCount = Camera_RxByteCount;
  Snapshot->ChecksumErrorCount = Camera_ChecksumErrorCount;
  Snapshot->OverflowCount = Camera_OverflowCount;
  __set_PRIMASK(Primask);
}

Camera_VisualStateTypeDef Camera_VisualStateGet(const Camera_SnapshotTypeDef *Snapshot)
{
  if ((Snapshot == NULL) || (Snapshot->RequestActive == 0U))
  {
    return CAMERA_VIS_IDLE;
  }
  if (Snapshot->UsbConfigured == 0U)
  {
    return CAMERA_VIS_OFF;
  }
  if (Snapshot->HasFrame == 0U)
  {
    return CAMERA_VIS_WAIT;
  }
  if ((uint32_t)(HAL_GetTick() - Snapshot->LastFrameTick) > CAMERA_DATA_STALE_MS)
  {
    return CAMERA_VIS_STALE;
  }
  return (Snapshot->TargetValid != 0U) ? CAMERA_VIS_OK : CAMERA_VIS_LOST;
}

const char *Camera_VisualStateNameGet(Camera_VisualStateTypeDef State)
{
  static const char *const Names[] = {"Idle", "Off", "Wait", "Ok", "Lost", "Stale"};

  if ((uint32_t)State >= (uint32_t)(sizeof(Names) / sizeof(Names[0])))
  {
    return "Idle";
  }
  return Names[State];
}

/**
  * @brief 保存USB CDC接收字节，支持64字节USB包及任意分段。
  * @param Data USB接收缓冲区，返回前完成复制。
  * @param Size 本次字节数。
  * @note 中断调用；溢出时丢弃待解析数据，避免跨缺失字节拼帧。
  */
void Camera_UsbRxCallback(const uint8_t *Data, uint32_t Size)
{
  if ((Data == NULL) || (Size == 0U))
  {
    return;
  }
  Camera_RxByteCount += Size;
  if (Size <= (uint32_t)(CAMERA_RX_BUFFER_SIZE - Camera_InputLength))
  {
    (void)memcpy(&Camera_InputBuffer[Camera_InputLength], Data, Size);
    Camera_InputLength = (uint16_t)(Camera_InputLength + Size);
  }
  else
  {
    Camera_InputLength = 0U;
    Camera_InputOverflow = 1U;
    Camera_OverflowCount++;
  }
}
