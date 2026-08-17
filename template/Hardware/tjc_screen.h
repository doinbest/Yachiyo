/**
 * @file    tjc_screen.h
 * @brief   陶晶驰串口屏 USART3 通信驱动。
 *
 * 当前工程使用 PB10/USART3_TX 向串口屏发送数据，使用
 * PB11/USART3_RX 接收串口屏数据。发送采用 DMA，接收采用
 * 单字节中断和环形缓冲区，完整帧在主循环中解析。
 */
#ifndef TJC_SCREEN_H
#define TJC_SCREEN_H

#include "main.h"
#include <stdbool.h>

#define TJC_RX_FRAME_MAX_LENGTH 64U

/** 陶晶驰串口屏通信状态，便于在 Keil Watch 中观察。 */
typedef struct
{
  uint32_t tx_count;           /**< DMA发送完成的命令数量。 */
  uint32_t rx_frame_count;     /**< 收到的完整串口屏帧数量。 */
  uint32_t rx_overflow_count;  /**< 接收环形缓冲区或帧缓冲区溢出次数。 */
  uint32_t uart_error_count;   /**< USART3错误回调累计次数。 */
  bool tx_busy;                /**< true表示USART3 TX DMA正在发送。 */
} TJC_Status_t;

/**********************************************************
*** 串口屏初始化
**********************************************************/
/**
  * @brief    初始化陶晶驰串口屏驱动并启动USART3接收中断
  * @param    huart ：串口屏使用的串口句柄，当前工程传入&huart3
  * @retval   HAL_OK    ：初始化成功并已开始接收
  * @retval   HAL_ERROR ：句柄为空或USART3 TX DMA未配置
  * @retval   HAL_BUSY  ：串口接收状态忙
  * @note     必须在MX_USART3_UART_Init()和MX_DMA_Init()之后调用
  */
HAL_StatusTypeDef TJC_Init(UART_HandleTypeDef *huart);

/**********************************************************
*** 串口屏数据处理
**********************************************************/
/**
  * @brief    从环形缓冲区取出数据并识别以FFFFFF结尾的完整帧
  * @param    无
  * @retval   无
  * @note     本函数不阻塞，应在while(1)中持续调用
  */
void TJC_Process(void);

/**********************************************************
*** 串口屏命令发送
**********************************************************/
/**
  * @brief    使用USART3 TX DMA发送一条陶晶驰ASCII命令
  * @param    command ：不包含末尾FFFFFF的命令字符串
  * @retval   HAL_OK      ：DMA发送已经启动
  * @retval   HAL_BUSY    ：上一条DMA命令尚未发送完成
  * @retval   HAL_ERROR   ：参数错误、命令过长或DMA启动失败
  * @note     函数内部会自动在命令末尾添加3个0xFF结束字节
  */
HAL_StatusTypeDef TJC_Command_Send(const char *command);

/**
  * @brief    设置串口屏文本控件的txt属性
  * @param    object_name ：文本控件名称，例如"t0"
  * @param    text        ：需要显示的字符串
  * @retval   HAL状态，含义与TJC_Command_Send()一致
  */
HAL_StatusTypeDef TJC_Text_Set(const char *object_name, const char *text);

/**
  * @brief    设置串口屏数字控件的val属性
  * @param    object_name ：数字控件名称，例如"n0"
  * @param    value       ：需要显示的有符号整数
  * @retval   HAL状态，含义与TJC_Command_Send()一致
  */
HAL_StatusTypeDef TJC_Number_Set(const char *object_name, int32_t value);

/**
  * @brief    切换陶晶驰串口屏页面
  * @param    page_id ：目标页面编号
  * @retval   HAL状态，含义与TJC_Command_Send()一致
  */
HAL_StatusTypeDef TJC_Page_Show(uint8_t page_id);

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
bool TJC_Frame_Get(uint8_t *data, uint16_t *length, uint16_t buffer_len);

/**
  * @brief    读取当前串口屏通信统计状态
  * @param    status ：用于保存通信状态的结构体地址
  * @retval   true  ：状态读取成功
  * @retval   false ：参数为空
  */
bool TJC_Status_Get(TJC_Status_t *status);

/**********************************************************
*** HAL串口回调接口
**********************************************************/
/**
  * @brief    处理USART3单字节接收完成事件并继续接收下一字节
  * @param    huart ：触发HAL接收完成回调的串口句柄
  * @retval   无
  * @note     由HAL_UART_RxCpltCallback()调用
  */
void TJC_Rx_Callback(UART_HandleTypeDef *huart);

/**
  * @brief    处理USART3 TX DMA发送完成事件
  * @param    huart ：触发HAL发送完成回调的串口句柄
  * @retval   无
  * @note     由HAL_UART_TxCpltCallback()调用
  */
void TJC_Tx_Callback(UART_HandleTypeDef *huart);

/**
  * @brief    记录USART3通信错误并在接收停止时重新启动接收
  * @param    huart ：触发HAL错误回调的串口句柄
  * @retval   无
  * @note     由HAL_UART_ErrorCallback()调用
  */
void TJC_Error_Callback(UART_HandleTypeDef *huart);

#endif /* TJC_SCREEN_H */
