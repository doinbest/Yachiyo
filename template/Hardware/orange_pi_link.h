/**
 * @file    orange_pi_link.h
 * @brief   香橙派与STM32之间的USART可靠帧接收接口。
 *
 * USART1接收中断只把字节写入环形缓冲区；协议版本、类型、序号、长度、
 * 时间戳和CRC16均在主循环解析，不直接操作底盘或机械机构。
 */
#ifndef ORANGE_PI_LINK_H
#define ORANGE_PI_LINK_H

#include "main.h"
#include <stdbool.h>

#define ORANGE_PI_PROTOCOL_VERSION       1U
#define ORANGE_PI_PAYLOAD_MAX_LENGTH     32U

/** 第一版已经定义的香橙派消息类型。 */
typedef enum
{
  ORANGE_PI_MSG_HEARTBEAT = 0x01,
  ORANGE_PI_MSG_TASK_CODE = 0x10,
  ORANGE_PI_MSG_START = 0x11,
  ORANGE_PI_MSG_STOP = 0x12,
  ORANGE_PI_MSG_TARGET_OBSERVATION = 0x20,
  ORANGE_PI_MSG_OBSTACLE_OBSERVATION = 0x21
} OrangePiMessageType_t;

/** CRC正确且版本受支持的一帧香橙派数据。 */
typedef struct
{
  uint8_t type;
  uint16_t seq;
  uint16_t payload_length;
  uint32_t remote_timestamp_ms;
  uint8_t payload[ORANGE_PI_PAYLOAD_MAX_LENGTH];
} OrangePiFrame_t;

/** 香橙派链路统计状态，便于Keil Watch和屏幕调试。 */
typedef struct
{
  uint32_t valid_frame_count;
  uint32_t crc_error_count;
  uint32_t length_error_count;
  uint32_t version_error_count;
  uint32_t frame_timeout_count;
  uint32_t rx_overflow_count;
  uint32_t queue_overflow_count;
  uint32_t uart_error_count;
  uint32_t last_valid_frame_ms;
  uint32_t last_remote_timestamp_ms;
  bool has_valid_frame;
} OrangePiLinkStatus_t;

/**
 * @brief    初始化香橙派串口链路
 * @param    huart ：已经由CubeMX初始化的串口，当前工程使用&huart1
 * @retval   true  初始化成功
 * @retval   false 串口句柄为空
 * @note     必须先由CubeMX配置USART1_IRQn，再在主循环调用Process
 */
bool OrangePi_Link_Init(UART_HandleTypeDef *huart);

/**
 * @brief    从环形缓冲区取出并解析有限数量的输入字节
 * @param    无
 * @retval   无
 * @note     不执行电机动作，应在while(1)中持续调用
 */
void OrangePi_Link_Process(void);

/**
 * @brief    读取一帧已经通过版本、长度和CRC检查的数据
 * @param    frame ：调用者提供的输出结构体
 * @retval   true  成功取出一帧
 * @retval   false 参数为空或接收队列为空
 */
bool OrangePi_Link_Frame_Get(OrangePiFrame_t *frame);

/**
 * @brief    判断香橙派数据链路是否仍然新鲜
 * @param    max_age_ms ：允许距离最近有效帧的最大时间，单位ms
 * @retval   true  已收到有效帧且未超过允许时间
 * @retval   false 尚未收到有效帧或已经超时
 */
bool OrangePi_Link_Is_Alive(uint32_t max_age_ms);

/**
 * @brief    读取香橙派链路统计
 * @param    status ：调用者提供的输出结构体
 * @retval   true  状态读取成功
 * @retval   false 参数为空
 */
bool OrangePi_Link_Status_Get(OrangePiLinkStatus_t *status);

/**
 * @brief    计算CRC16-CCITT-FALSE
 * @param    data   ：输入数据
 * @param    length ：数据长度，单位byte
 * @retval   CRC16结果；参数为空且长度非0时返回0
 * @note     初值0xFFFF，多项式0x1021，不反转，结果不异或
 */
uint16_t OrangePi_Link_Crc16_Calc(const uint8_t *data, uint16_t length);

/**
 * @brief    处理香橙派USART单字节接收完成事件
 * @param    huart ：触发HAL回调的串口句柄
 * @retval   无
 * @note     只写环形缓冲并重启接收，由HAL_UART_RxCpltCallback()调用
 */
void OrangePi_Link_Rx_Callback(UART_HandleTypeDef *huart);

/**
 * @brief    记录香橙派USART错误并重新启动接收
 * @param    huart ：触发HAL错误回调的串口句柄
 * @retval   无
 */
void OrangePi_Link_Error_Callback(UART_HandleTypeDef *huart);

#endif /* ORANGE_PI_LINK_H */
