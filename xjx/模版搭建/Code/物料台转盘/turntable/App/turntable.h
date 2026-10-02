#ifndef TURNTABLE_H
#define TURNTABLE_H
#include "stm32f1xx_hal.h"
typedef enum {
  TURNTABLE_IDLE, TURNTABLE_STARTING, TURNTABLE_MOVING,
  TURNTABLE_DWELL, TURNTABLE_STOPPING, TURNTABLE_FAULT
} Turntable_State_t;
typedef enum {
  TURNTABLE_ERROR_NONE, TURNTABLE_ERROR_COMM, TURNTABLE_ERROR_DRIVER,
  TURNTABLE_ERROR_MOTION_TIMEOUT, TURNTABLE_ERROR_RANGE
} Turntable_Error_t;
typedef struct {
  Turntable_State_t state;
  Turntable_Error_t error;
  int32_t target_pulses;
  int64_t position_units;
  uint32_t completed_moves;
  uint8_t automatic;
  uint8_t stop_confirmed;
} Turntable_Status_t;
/** @brief 调试器可观察的状态；停止请求与停止反馈分别记录。 */
extern volatile Turntable_Status_t Turntable_Status;
/** @brief GPIO/USART 初始化后调用；上电不运动，不执行机械回零。 @param uart 电机串口。 */
void Turntable_Init(UART_HandleTypeDef *uart);
/** @brief 主循环持续调用；处理低电平按键、运动反馈和毫秒计时。 */
void Turntable_Process(void);
#endif
