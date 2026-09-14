/**
  ******************************************************************************
  * @file    QR.h
  * @brief   比赛二维码任务码接收与解析接口
  ******************************************************************************
  */
#ifndef __QR_H
#define __QR_H

#include "main.h"

#define QR_TASK_CODE_LENGTH 15U  /* 不包含结束符的二维码任务码长度。 */
#define QR_TASK_CODE_BUFFER_SIZE 16U  /* 包含字符串结束符的任务码缓冲区大小。 */

/** Latest valid code and counters; querying does not consume the UI update. */
typedef struct
{
  char Code[QR_TASK_CODE_BUFFER_SIZE];
  uint8_t Valid;
  uint32_t Sequence;
  uint32_t ReceivedTick;
  uint32_t Received;     /**< UART4 byte count, including delimiters. */
  uint32_t Accepted;     /**< Valid CR-terminated frames, including repeated codes. */
  uint32_t Rejected;     /**< Invalid CR-terminated frames. */
  uint32_t UartErrors;
} QR_SnapshotTypeDef;

/** @brief Main-loop atomic snapshot. @param Snapshot Output; NULL is ignored. */
void QR_SnapshotGet(QR_SnapshotTypeDef *Snapshot);
/** @brief UART4 error IRQ: count the error and discard the damaged frame. */
void QR_ErrorCallback(void);

/**
  * 函    数：二维码解析器初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：清空接收状态；串口首次接收由main.c启动
  */
void QR_Init(void);

/**
  * 函    数：处理二维码模块收到的一个字节
  * 参    数：Data UART4本次收到的字节
  * 返 回 值：无
  * 说    明：接收DDD+DDD+DDD+DDD格式，以回车作为帧结束符
  */
void QR_ReceiveData(uint8_t Data);

/**
  * 函    数：读取最新二维码任务码
  * 参    数：TaskCode 调用者提供的16字节输出缓冲区
  * 返 回 值：1读取成功；0尚无新任务码或参数无效
  * 说    明：复制数据后清除新任务码标志，应在主循环调用
  */
uint8_t QR_TaskCodeGet(char *TaskCode);

#endif /* __QR_H */
