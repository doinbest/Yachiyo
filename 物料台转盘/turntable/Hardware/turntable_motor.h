#ifndef TURNTABLE_MOTOR_H
#define TURNTABLE_MOTOR_H
#include "stm32f1xx_hal.h"
typedef enum { MOTOR_WAIT, MOTOR_OK, MOTOR_COMM_ERROR, MOTOR_REJECTED } Motor_Result_t;
/** @brief USART 初始化后开启单字节中断接收；一次仅允许一个请求。 */
void Motor_Init(UART_HandleTypeDef *uart);
/** @brief 读取 0x35 速度、0x36 位置或 0x3A 状态；返回是否已发送。 */
uint8_t Motor_Read(uint8_t function);
/** @brief 使能电机，不修改永久配置；返回是否已发送。 */
uint8_t Motor_Enable(void);
/** @brief Emm 绝对位置运动。 @param pulses 有符号命令脉冲，3200/圈。 @return 是否已发送。 */
uint8_t Motor_Move(int32_t pulses);
/** @brief 取消待处理事务并优先发送停止；返回发送结果，不代表已停车。 */
uint8_t Motor_Stop(void);
/** @brief 主循环解析及超时。 @param value 成功时返回有符号读值或 ACK 状态码。 */
Motor_Result_t Motor_Poll(int64_t *value);
#endif
