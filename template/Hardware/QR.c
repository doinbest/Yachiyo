/**
  ******************************************************************************
  * @file    QR.c
  * @brief   比赛二维码任务码接收与解析实现
  ******************************************************************************
  */
#include "QR.h"

#include <string.h>

static char QR_RxData[QR_TASK_CODE_BUFFER_SIZE];
static QR_SnapshotTypeDef QR_Latest;
static volatile uint8_t QR_TaskCodeReady;
static uint8_t QR_RxIndex;
static uint8_t QR_RxInvalid;

/**
  * 函    数：复位二维码当前帧接收状态
  * 参    数：无
  * 返 回 值：无
  * 说    明：丢弃尚未完成或格式错误的任务码
  */
static void QR_ReceiveReset(void)
{
  QR_RxIndex = 0U;
  QR_RxInvalid = 0U;
  (void)memset(QR_RxData, 0, sizeof(QR_RxData));
}

/**
  * 函    数：检查当前位置允许的任务码字符
  * 参    数：Index 当前字符位置，范围0至14
  * 参    数：Data 待检查字符
  * 返 回 值：1字符符合格式；0字符不符合格式
  * 说    明：位置3、7、11必须为加号，其余位置必须为数字
  */
static uint8_t QR_CharacterCheck(uint8_t Index, uint8_t Data)
{
  if ((Index == 3U) || (Index == 7U) || (Index == 11U))
  {
    return (Data == (uint8_t)'+') ? 1U : 0U;
  }

  return ((Data >= (uint8_t)'0') && (Data <= (uint8_t)'9')) ? 1U : 0U;
}

/**
  * 函    数：二维码解析器初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：清空接收状态；串口首次接收由main.c启动
  */
void QR_Init(void)
{
  QR_TaskCodeReady = 0U;
  (void)memset(&QR_Latest, 0, sizeof(QR_Latest));
  QR_ReceiveReset();
}

/**
  * 函    数：处理二维码模块收到的一个字节
  * 参    数：Data UART4本次收到的字节
  * 返 回 值：无
  * 说    明：接收DDD+DDD+DDD+DDD格式，以回车作为帧结束符
  */
void QR_ReceiveData(uint8_t Data)
{
  QR_Latest.Received++;
  if (Data == (uint8_t)'\n')
  {
    return;
  }

  if (Data == (uint8_t)'\r')
  {
    if ((QR_RxIndex == QR_TASK_CODE_LENGTH) && !QR_RxInvalid)
    {
      QR_RxData[QR_TASK_CODE_LENGTH] = '\0';
      (void)memcpy(QR_Latest.Code, QR_RxData, sizeof(QR_Latest.Code));
      QR_Latest.Valid = 1U;
      QR_Latest.Sequence++;
      QR_Latest.Accepted++;
      QR_Latest.ReceivedTick = HAL_GetTick();
      QR_TaskCodeReady = 1U;
    }
    else
    {
      QR_Latest.Rejected++;
    }

    QR_ReceiveReset();
    return;
  }

  if (QR_RxInvalid) return;
  if ((QR_RxIndex < QR_TASK_CODE_LENGTH) &&
      (QR_CharacterCheck(QR_RxIndex, Data) == 1U))
  {
    QR_RxData[QR_RxIndex] = (char)Data;
    QR_RxIndex++;
    return;
  }

  /* Discard through CR: a valid-looking suffix is not a complete valid frame. */
  QR_RxInvalid = 1U;
}

void QR_ErrorCallback(void)
{
  QR_Latest.UartErrors++;
  QR_RxInvalid = 1U;
}

void QR_SnapshotGet(QR_SnapshotTypeDef *Snapshot)
{
  uint32_t Primask;
  if (Snapshot == NULL) return;
  Primask = __get_PRIMASK();
  __disable_irq();
  *Snapshot = QR_Latest;
  __set_PRIMASK(Primask);
}

/**
  * 函    数：读取最新二维码任务码
  * 参    数：TaskCode 调用者提供的16字节输出缓冲区
  * 返 回 值：1读取成功；0尚无新任务码或参数无效
  * 说    明：复制数据后清除新任务码标志，应在主循环调用
  */
uint8_t QR_TaskCodeGet(char *TaskCode)
{
  uint32_t Primask;

  if (TaskCode == NULL)
  {
    return 0U;
  }

  Primask = __get_PRIMASK();
  __disable_irq();
  if (QR_TaskCodeReady == 0U)
  {
    __set_PRIMASK(Primask);
    return 0U;
  }

  (void)memcpy(TaskCode, QR_Latest.Code, sizeof(QR_Latest.Code));
  QR_TaskCodeReady = 0U;
  __set_PRIMASK(Primask);

  return 1U;
}
