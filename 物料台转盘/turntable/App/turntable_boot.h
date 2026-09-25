#ifndef TURNTABLE_BOOT_H
#define TURNTABLE_BOOT_H
#include "stm32f1xx_hal.h"
/** @brief 仅反映本机发送结果，不代表驱动器应答或已旋转。 */
typedef struct {
  uint8_t enable_sent;
  uint8_t velocity_sent;
  HAL_StatusTypeDef enable_result;
  HAL_StatusTypeDef velocity_result;
} Turntable_BootStatus_t;
extern volatile Turntable_BootStatus_t Turntable_BootStatus;
/** @brief USART 初始化后调用；准备上电直发测试，不启动按键或接收。 */
void Turntable_BootInit(UART_HandleTypeDef *uart);
/** @brief 主循环调用；1 秒后使能，再隔 100 ms 发一次 Emm 连续转动指令。 */
void Turntable_BootProcess(void);
#endif
