/**
 * @file    jy61p.h
 * @brief   维特 JY61P 三轴角度读取驱动。
 *
 * JY61P 通过 UART4 与 STM32 通信：PC10 为 UART4_TX，PC11 为 UART4_RX。
 * 当前工程采用115200、8N1，并参考 F103 HAL 工程使用单字节中断接收。
 */
#ifndef JY61P_H
#define JY61P_H

#include "main.h"
#include <stdbool.h>

/** JY61P 三轴角度数据，三个角度的单位均为度。 */
typedef struct
{
  float roll;              /**< 横滚角：绕 X 轴旋转。 */
  float pitch;             /**< 俯仰角：绕 Y 轴旋转。 */
  float yaw;               /**< 航向角：绕 Z 轴旋转。 */
  uint32_t update_count;   /**< 校验正确的角度帧累计数量。 */
} JY61P_Angle_t;

/**********************************************************
*** JY61P初始化
**********************************************************/
/**
  * @brief    初始化 JY61P 串口驱动和角度解析状态
  * @param    huart ：JY61P 使用的串口句柄，当前工程传入 &huart4
  * @retval   true  ：初始化成功
  * @retval   false ：串口句柄为空，初始化失败
  * @note     必须在 MX_UART4_Init() 之后调用
  */
bool JY61P_Init(UART_HandleTypeDef *huart);

/**********************************************************
*** 串口接收完成回调
**********************************************************/
/**
  * @brief    处理 UART4 收到的一个字节并重新开启下一字节接收
  * @param    huart ：触发接收完成事件的串口句柄
  * @retval   无
  * @note     应由 HAL_UART_RxCpltCallback() 调用
  */
void JY61P_Rx_Callback(UART_HandleTypeDef *huart);

/**********************************************************
*** 串口错误回调
**********************************************************/
/**
  * @brief    清除 UART4 接收错误并重新开启单字节接收
  * @param    huart ：触发错误事件的串口句柄
  * @retval   无
  * @note     应由 HAL_UART_ErrorCallback() 调用
  */
void JY61P_Error_Callback(UART_HandleTypeDef *huart);

/**********************************************************
*** 三轴角度读取
**********************************************************/
/**
  * @brief    读取最近一次校验正确的横滚角、俯仰角和航向角
  * @param    angle ：用于保存三轴角度的结构体地址
  * @retval   true  ：本次取出了一个新的角度数据
  * @retval   false ：参数为空，或上次读取后尚未收到新的角度帧
  * @note     返回false时不会修改调用者原有的角度数据
  */
bool JY61P_Angle_Get(JY61P_Angle_t *angle);

/**********************************************************
*** 航向角清零
**********************************************************/
/**
  * @brief    向 JY61P 发送解锁命令和 Z 轴航向角清零命令
  * @param    无
  * @retval   HAL_OK      ：两条命令均发送成功
  * @retval   其他HAL状态 ：串口未初始化或发送失败
  * @note     该函数会阻塞约10ms，不应在主循环中反复调用
  */
HAL_StatusTypeDef JY61P_Yaw_Zero(void);

#endif /* JY61P_H */
