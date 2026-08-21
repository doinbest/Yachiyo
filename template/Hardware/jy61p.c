/**
 * @file    jy61p.c
 * @brief   维特 JY61P 三轴角度读取驱动实现。
 */
#include "jy61p.h"

#define JY61P_FRAME_HEADER       0x55U
#define JY61P_ANGLE_FRAME_TYPE   0x53U
#define JY61P_FRAME_LENGTH       11U
#define JY61P_ANGLE_SCALE        (180.0f / 32768.0f)
#define JY61P_SEND_TIMEOUT_MS    100U

static UART_HandleTypeDef *jy61p_uart = NULL;
static uint8_t jy61p_rx_data = 0U;
static uint8_t jy61p_rx_buffer[JY61P_FRAME_LENGTH];
static uint8_t jy61p_rx_count = 0U;
static volatile JY61P_Angle_t jy61p_latest_angle;
static volatile bool jy61p_angle_is_new = false;

/**********************************************************
*** 角度数据换算
**********************************************************/
/**
  * @brief    将 JY61P 的两个低字节优先数据转换为有符号16位数
  * @param    low_byte  ：数据低8位
  * @param    high_byte ：数据高8位
  * @retval   合并后的有符号16位原始数据
  */
static int16_t JY61P_Bytes_To_Int16(uint8_t low_byte, uint8_t high_byte)
{
  uint16_t raw_data;

  raw_data = (uint16_t)low_byte | ((uint16_t)high_byte << 8U);
  return (int16_t)raw_data;
}

/**********************************************************
*** 完整角度帧处理
**********************************************************/
/**
  * @brief    校验11字节角度帧，并更新三轴角度
  * @param    无
  * @retval   无
  * @note     角度换算公式：原始值 / 32768 * 180度
  */
static void JY61P_Angle_Frame_Update(void)
{
  uint8_t index;
  uint8_t checksum = 0U;
  int16_t raw_roll;
  int16_t raw_pitch;
  int16_t raw_yaw;

  for (index = 0U; index < (JY61P_FRAME_LENGTH - 1U); ++index)
  {
    checksum = (uint8_t)(checksum + jy61p_rx_buffer[index]);
  }

  if (checksum != jy61p_rx_buffer[JY61P_FRAME_LENGTH - 1U])
  {
    return;
  }

  raw_roll = JY61P_Bytes_To_Int16(jy61p_rx_buffer[2],
                                  jy61p_rx_buffer[3]);
  raw_pitch = JY61P_Bytes_To_Int16(jy61p_rx_buffer[4],
                                   jy61p_rx_buffer[5]);
  raw_yaw = JY61P_Bytes_To_Int16(jy61p_rx_buffer[6],
                                 jy61p_rx_buffer[7]);

  jy61p_latest_angle.roll = (float)raw_roll * JY61P_ANGLE_SCALE;
  jy61p_latest_angle.pitch = (float)raw_pitch * JY61P_ANGLE_SCALE;
  jy61p_latest_angle.yaw = (float)raw_yaw * JY61P_ANGLE_SCALE;
  ++jy61p_latest_angle.update_count;
  jy61p_angle_is_new = true;
}

/**********************************************************
*** 单字节数据解析
**********************************************************/
/**
  * @brief    按照 0x55、0x53、剩余9字节的顺序组装角度帧
  * @param    rx_data ：UART4 本次收到的一个字节
  * @retval   无
  */
static void JY61P_Byte_Process(uint8_t rx_data)
{
  if (jy61p_rx_count == 0U)
  {
    if (rx_data == JY61P_FRAME_HEADER)
    {
      jy61p_rx_buffer[0] = rx_data;
      jy61p_rx_count = 1U;
    }
    return;
  }

  if (jy61p_rx_count == 1U)
  {
    if (rx_data == JY61P_ANGLE_FRAME_TYPE)
    {
      jy61p_rx_buffer[1] = rx_data;
      jy61p_rx_count = 2U;
    }
    else if (rx_data != JY61P_FRAME_HEADER)
    {
      /* 当前字节不是角度帧类型，也不是下一帧包头，重新等待0x55。 */
      jy61p_rx_count = 0U;
    }
    return;
  }

  jy61p_rx_buffer[jy61p_rx_count] = rx_data;
  ++jy61p_rx_count;

  if (jy61p_rx_count >= JY61P_FRAME_LENGTH)
  {
    JY61P_Angle_Frame_Update();
    jy61p_rx_count = 0U;
  }
}

/**********************************************************
*** JY61P初始化
**********************************************************/
/**
  * @brief    初始化 JY61P 串口驱动和角度解析状态
  * @param    huart ：JY61P 使用的串口句柄，当前工程传入 &huart4
  * @retval   true  ：初始化成功
  * @retval   false ：串口句柄为空，初始化失败
  */
bool JY61P_Init(UART_HandleTypeDef *huart)
{
  HAL_StatusTypeDef hal_status;

  if (huart == NULL)
  {
    return false;
  }

  jy61p_uart = huart;
  jy61p_rx_count = 0U;
  jy61p_latest_angle.roll = 0.0f;
  jy61p_latest_angle.pitch = 0.0f;
  jy61p_latest_angle.yaw = 0.0f;
  jy61p_latest_angle.update_count = 0U;
  jy61p_angle_is_new = false;

  /* 参考 F103 HAL 例程，从第一个字节开始持续使用 UART4 中断接收。 */
  hal_status = HAL_UART_Receive_IT(jy61p_uart, &jy61p_rx_data, 1U);
  return (hal_status == HAL_OK);
}

/**********************************************************
*** 串口接收完成回调
**********************************************************/
/**
  * @brief    处理 UART4 收到的一个字节并重新开启下一字节接收
  * @param    huart ：触发接收完成事件的串口句柄
  * @retval   无
  */
void JY61P_Rx_Callback(UART_HandleTypeDef *huart)
{
  if ((jy61p_uart == NULL) || (huart != jy61p_uart))
  {
    return;
  }

  JY61P_Byte_Process(jy61p_rx_data);
  (void)HAL_UART_Receive_IT(jy61p_uart, &jy61p_rx_data, 1U);
}

/**********************************************************
*** 串口错误回调
**********************************************************/
/**
  * @brief    清除 UART4 接收错误并重新开启单字节接收
  * @param    huart ：触发错误事件的串口句柄
  * @retval   无
  */
void JY61P_Error_Callback(UART_HandleTypeDef *huart)
{
  if ((jy61p_uart == NULL) || (huart != jy61p_uart))
  {
    return;
  }

  if ((HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) != 0U)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
  }

  jy61p_rx_count = 0U;
  (void)HAL_UART_Receive_IT(jy61p_uart, &jy61p_rx_data, 1U);
}

/**********************************************************
*** 三轴角度读取
**********************************************************/
/**
  * @brief    读取最近一次校验正确的横滚角、俯仰角和航向角
  * @param    angle ：用于保存三轴角度的结构体地址
  * @retval   true  ：本次取出了一个新的角度数据
  * @retval   false ：参数为空，或上次读取后尚未收到新的角度帧
  */
bool JY61P_Angle_Get(JY61P_Angle_t *angle)
{
  uint32_t interrupt_state;

  if ((angle == NULL) || (!jy61p_angle_is_new))
  {
    return false;
  }

  /* 防止 UART4 中断在结构体复制过程中更新其中一部分成员。 */
  interrupt_state = __get_PRIMASK();
  __disable_irq();
  angle->roll = jy61p_latest_angle.roll;
  angle->pitch = jy61p_latest_angle.pitch;
  angle->yaw = jy61p_latest_angle.yaw;
  angle->update_count = jy61p_latest_angle.update_count;
  jy61p_angle_is_new = false;

  if (interrupt_state == 0U)
  {
    __enable_irq();
  }

  return true;
}

/**********************************************************
*** 航向角清零
**********************************************************/
/**
  * @brief    向 JY61P 发送解锁命令和 Z 轴航向角清零命令
  * @param    无
  * @retval   HAL_OK      ：两条命令均发送成功
  * @retval   其他HAL状态 ：串口未初始化或发送失败
  */
HAL_StatusTypeDef JY61P_Yaw_Zero(void)
{
  uint8_t unlock_command[5] = {0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U};
  uint8_t zero_command[5] = {0xFFU, 0xAAU, 0x01U, 0x04U, 0x00U};
  HAL_StatusTypeDef status;

  if (jy61p_uart == NULL)
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Transmit(jy61p_uart, unlock_command,
                             sizeof(unlock_command), JY61P_SEND_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(10U);
  return HAL_UART_Transmit(jy61p_uart, zero_command,
                           sizeof(zero_command), JY61P_SEND_TIMEOUT_MS);
}
