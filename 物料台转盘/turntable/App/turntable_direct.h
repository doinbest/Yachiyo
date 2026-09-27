#ifndef TURNTABLE_DIRECT_H
#define TURNTABLE_DIRECT_H
#include "stm32f1xx_hal.h"
typedef enum { DIRECT_IDLE, DIRECT_ENABLE_STEP, DIRECT_STEP,
               DIRECT_DWELL, DIRECT_STOP_PENDING } Turntable_DirectState_t;
/** @brief 本机操作状态，不是电机实际状态或到位反馈。 */
typedef struct {
  Turntable_DirectState_t state;
  uint32_t pa15_events, pb3_events, tx_attempts;
  HAL_StatusTypeDef tx_result;
  uint8_t automatic;
} Turntable_DirectStatus_t;
extern volatile Turntable_DirectStatus_t Turntable_DirectStatus;
/** @brief GPIO/UART 初始化后调用；不启动串口接收，上电静止等待按键。 */
void Turntable_DirectInit(UART_HandleTypeDef *uart);
/** @brief 主循环调用；按键直发，不等待应答，单步结束仅按时间估计。 */
void Turntable_DirectProcess(void);
#endif
