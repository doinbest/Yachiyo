/**
 * @file    tjc_screen.c
 * @brief   陶晶驰串口屏 USART3 通信驱动实现。
 */
#include "tjc_screen.h"

#include <stdio.h>
#include <string.h>

#define TJC_END_BYTE             0xFFU
#define TJC_END_BYTE_COUNT       3U
#define TJC_TX_BUFFER_LENGTH     256U
#define TJC_RX_RING_LENGTH       256U
#define TJC_COMMAND_BUFFER_LENGTH 128U

static UART_HandleTypeDef *tjc_uart = NULL;
static uint8_t tjc_tx_buffer[TJC_TX_BUFFER_LENGTH];
static uint8_t tjc_rx_byte = 0U;

static uint8_t tjc_rx_ring[TJC_RX_RING_LENGTH];
static volatile uint16_t tjc_rx_write = 0U;
static volatile uint16_t tjc_rx_read = 0U;

static uint8_t tjc_frame_buffer[TJC_RX_FRAME_MAX_LENGTH];
static uint16_t tjc_frame_length = 0U;
static uint8_t tjc_end_count = 0U;
static bool tjc_frame_overflow = false;

static uint8_t tjc_last_frame[TJC_RX_FRAME_MAX_LENGTH];
static uint16_t tjc_last_frame_length = 0U;
static bool tjc_frame_is_new = false;

static volatile uint32_t tjc_tx_count = 0U;
static volatile uint32_t tjc_rx_frame_count = 0U;
static volatile uint32_t tjc_rx_overflow_count = 0U;
static volatile uint32_t tjc_uart_error_count = 0U;
static volatile bool tjc_tx_busy = false;

/**********************************************************
*** 接收环形缓冲区操作
**********************************************************/
/**
  * @brief    将中断收到的一个字节写入环形缓冲区
  * @param    data ：USART3收到的单字节数据
  * @retval   true  ：写入成功
  * @retval   false ：环形缓冲区已满，本字节被丢弃
  */
static bool TJC_Rx_Byte_Write(uint8_t data)
{
  uint16_t next_write;

  next_write = (uint16_t)((tjc_rx_write + 1U) % TJC_RX_RING_LENGTH);
  if (next_write == tjc_rx_read)
  {
    ++tjc_rx_overflow_count;
    return false;
  }

  tjc_rx_ring[tjc_rx_write] = data;
  tjc_rx_write = next_write;
  return true;
}

/**
  * @brief    从环形缓冲区读取一个字节
  * @param    data ：用于保存读出字节的变量地址
  * @retval   true  ：成功读取一个字节
  * @retval   false ：参数为空或环形缓冲区没有数据
  */
static bool TJC_Rx_Byte_Read(uint8_t *data)
{
  if ((data == NULL) || (tjc_rx_read == tjc_rx_write))
  {
    return false;
  }

  *data = tjc_rx_ring[tjc_rx_read];
  tjc_rx_read = (uint16_t)((tjc_rx_read + 1U) % TJC_RX_RING_LENGTH);
  return true;
}

/**********************************************************
*** 接收帧保存
**********************************************************/
/**
  * @brief    保存一帧去除末尾FFFFFF后的串口屏数据
  * @param    无
  * @retval   无
  */
static void TJC_Frame_Save(void)
{
  uint16_t payload_length;

  if (tjc_frame_overflow || (tjc_frame_length < TJC_END_BYTE_COUNT))
  {
    ++tjc_rx_overflow_count;
  }
  else
  {
    payload_length = (uint16_t)(tjc_frame_length - TJC_END_BYTE_COUNT);
    memcpy(tjc_last_frame, tjc_frame_buffer, payload_length);
    tjc_last_frame_length = payload_length;
    tjc_frame_is_new = true;
    ++tjc_rx_frame_count;
  }

  tjc_frame_length = 0U;
  tjc_end_count = 0U;
  tjc_frame_overflow = false;
}

/**********************************************************
*** 串口屏初始化
**********************************************************/
/**
  * @brief    初始化陶晶驰串口屏驱动并启动USART3接收中断
  * @param    huart ：串口屏使用的串口句柄，当前工程传入&huart3
  * @retval   HAL_OK    ：初始化成功并已开始接收
  * @retval   HAL_ERROR ：句柄为空或USART3 TX DMA未配置
  * @retval   HAL_BUSY  ：串口接收状态忙
  */
HAL_StatusTypeDef TJC_Init(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart->hdmatx == NULL))
  {
    return HAL_ERROR;
  }

  tjc_uart = huart;
  tjc_rx_write = 0U;
  tjc_rx_read = 0U;
  tjc_frame_length = 0U;
  tjc_end_count = 0U;
  tjc_frame_overflow = false;
  tjc_last_frame_length = 0U;
  tjc_frame_is_new = false;
  tjc_tx_count = 0U;
  tjc_rx_frame_count = 0U;
  tjc_rx_overflow_count = 0U;
  tjc_uart_error_count = 0U;
  tjc_tx_busy = false;

  return HAL_UART_Receive_IT(tjc_uart, &tjc_rx_byte, 1U);
}

/**********************************************************
*** 串口屏数据处理
**********************************************************/
/**
  * @brief    从环形缓冲区取出数据并识别以FFFFFF结尾的完整帧
  * @param    无
  * @retval   无
  */
void TJC_Process(void)
{
  uint8_t data;

  while (TJC_Rx_Byte_Read(&data))
  {
    if (tjc_frame_length < TJC_RX_FRAME_MAX_LENGTH)
    {
      tjc_frame_buffer[tjc_frame_length] = data;
      ++tjc_frame_length;
    }
    else
    {
      tjc_frame_overflow = true;
    }

    if (data == TJC_END_BYTE)
    {
      ++tjc_end_count;
      if (tjc_end_count >= TJC_END_BYTE_COUNT)
      {
        TJC_Frame_Save();
      }
    }
    else
    {
      tjc_end_count = 0U;
    }
  }
}

/**********************************************************
*** 串口屏命令发送
**********************************************************/
/**
  * @brief    使用USART3 TX DMA发送一条陶晶驰ASCII命令
  * @param    command ：不包含末尾FFFFFF的命令字符串
  * @retval   HAL_OK      ：DMA发送已经启动
  * @retval   HAL_BUSY    ：上一条DMA命令尚未发送完成
  * @retval   HAL_ERROR   ：参数错误、命令过长或DMA启动失败
  */
HAL_StatusTypeDef TJC_Command_Send(const char *command)
{
  size_t command_length;
  uint16_t tx_length;
  HAL_StatusTypeDef status;

  if ((tjc_uart == NULL) || (command == NULL))
  {
    return HAL_ERROR;
  }

  if (tjc_tx_busy)
  {
    return HAL_BUSY;
  }

  command_length = strlen(command);
  if ((command_length == 0U) ||
      (command_length > (TJC_TX_BUFFER_LENGTH - TJC_END_BYTE_COUNT)))
  {
    return HAL_ERROR;
  }

  memcpy(tjc_tx_buffer, command, command_length);
  tjc_tx_buffer[command_length] = TJC_END_BYTE;
  tjc_tx_buffer[command_length + 1U] = TJC_END_BYTE;
  tjc_tx_buffer[command_length + 2U] = TJC_END_BYTE;
  tx_length = (uint16_t)(command_length + TJC_END_BYTE_COUNT);

  tjc_tx_busy = true;
  status = HAL_UART_Transmit_DMA(tjc_uart, tjc_tx_buffer, tx_length);
  if (status != HAL_OK)
  {
    tjc_tx_busy = false;
  }
  return status;
}

/**
  * @brief    设置串口屏文本控件的txt属性
  * @param    object_name ：文本控件名称，例如"t0"
  * @param    text        ：需要显示的字符串
  * @retval   HAL状态，含义与TJC_Command_Send()一致
  */
HAL_StatusTypeDef TJC_Text_Set(const char *object_name, const char *text)
{
  char command[TJC_COMMAND_BUFFER_LENGTH];
  int length;

  if ((object_name == NULL) || (text == NULL))
  {
    return HAL_ERROR;
  }

  length = snprintf(command, sizeof(command), "%s.txt=\"%s\"",
                    object_name, text);
  if ((length <= 0) || ((uint32_t)length >= sizeof(command)))
  {
    return HAL_ERROR;
  }
  return TJC_Command_Send(command);
}

/**
  * @brief    设置串口屏数字控件的val属性
  * @param    object_name ：数字控件名称，例如"n0"
  * @param    value       ：需要显示的有符号整数
  * @retval   HAL状态，含义与TJC_Command_Send()一致
  */
HAL_StatusTypeDef TJC_Number_Set(const char *object_name, int32_t value)
{
  char command[TJC_COMMAND_BUFFER_LENGTH];
  int length;

  if (object_name == NULL)
  {
    return HAL_ERROR;
  }

  length = snprintf(command, sizeof(command), "%s.val=%ld",
                    object_name, (long)value);
  if ((length <= 0) || ((uint32_t)length >= sizeof(command)))
  {
    return HAL_ERROR;
  }
  return TJC_Command_Send(command);
}

/**
  * @brief    切换陶晶驰串口屏页面
  * @param    page_id ：目标页面编号
  * @retval   HAL状态，含义与TJC_Command_Send()一致
  */
HAL_StatusTypeDef TJC_Page_Show(uint8_t page_id)
{
  char command[20];
  int length;

  length = snprintf(command, sizeof(command), "page %u",
                    (unsigned int)page_id);
  if ((length <= 0) || ((uint32_t)length >= sizeof(command)))
  {
    return HAL_ERROR;
  }
  return TJC_Command_Send(command);
}

/**********************************************************
*** 串口屏接收帧读取
**********************************************************/
/**
  * @brief    读取一帧已经去除末尾FFFFFF的串口屏返回数据
  * @param    data       ：调用者提供的接收数组
  * @param    length     ：返回实际数据长度
  * @param    buffer_len ：调用者接收数组的最大长度
  * @retval   true  ：成功取出一帧新数据
  * @retval   false ：参数错误、缓冲区过小或当前没有新帧
  */
bool TJC_Frame_Get(uint8_t *data, uint16_t *length, uint16_t buffer_len)
{
  if ((data == NULL) || (length == NULL) || (!tjc_frame_is_new) ||
      (buffer_len < tjc_last_frame_length))
  {
    return false;
  }

  memcpy(data, tjc_last_frame, tjc_last_frame_length);
  *length = tjc_last_frame_length;
  tjc_frame_is_new = false;
  return true;
}

/**
  * @brief    读取当前串口屏通信统计状态
  * @param    status ：用于保存通信状态的结构体地址
  * @retval   true  ：状态读取成功
  * @retval   false ：参数为空
  */
bool TJC_Status_Get(TJC_Status_t *status)
{
  uint32_t interrupt_state;

  if (status == NULL)
  {
    return false;
  }

  interrupt_state = __get_PRIMASK();
  __disable_irq();
  status->tx_count = tjc_tx_count;
  status->rx_frame_count = tjc_rx_frame_count;
  status->rx_overflow_count = tjc_rx_overflow_count;
  status->uart_error_count = tjc_uart_error_count;
  status->tx_busy = tjc_tx_busy;
  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
  return true;
}

/**********************************************************
*** HAL串口回调处理
**********************************************************/
/**
  * @brief    处理USART3单字节接收完成事件并继续接收下一字节
  * @param    huart ：触发HAL接收完成回调的串口句柄
  * @retval   无
  */
void TJC_Rx_Callback(UART_HandleTypeDef *huart)
{
  if ((tjc_uart == NULL) || (huart == NULL) ||
      (huart->Instance != tjc_uart->Instance))
  {
    return;
  }

  (void)TJC_Rx_Byte_Write(tjc_rx_byte);
  if (HAL_UART_Receive_IT(tjc_uart, &tjc_rx_byte, 1U) != HAL_OK)
  {
    ++tjc_uart_error_count;
  }
}

/**
  * @brief    处理USART3 TX DMA发送完成事件
  * @param    huart ：触发HAL发送完成回调的串口句柄
  * @retval   无
  */
void TJC_Tx_Callback(UART_HandleTypeDef *huart)
{
  if ((tjc_uart == NULL) || (huart == NULL) ||
      (huart->Instance != tjc_uart->Instance))
  {
    return;
  }

  tjc_tx_busy = false;
  ++tjc_tx_count;
}

/**
  * @brief    记录USART3通信错误并在接收停止时重新启动接收
  * @param    huart ：触发HAL错误回调的串口句柄
  * @retval   无
  */
void TJC_Error_Callback(UART_HandleTypeDef *huart)
{
  if ((tjc_uart == NULL) || (huart == NULL) ||
      (huart->Instance != tjc_uart->Instance))
  {
    return;
  }

  ++tjc_uart_error_count;
  if (tjc_uart->gState == HAL_UART_STATE_READY)
  {
    tjc_tx_busy = false;
  }
  if (tjc_uart->RxState == HAL_UART_STATE_READY)
  {
    if (HAL_UART_Receive_IT(tjc_uart, &tjc_rx_byte, 1U) != HAL_OK)
    {
      ++tjc_uart_error_count;
    }
  }
}
