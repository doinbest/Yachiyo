#ifndef CONSOLE_RX_H
#define CONSOLE_RX_H
#include "main.h"
typedef struct {
  uint32_t bytes_received, uart_errors, overruns, framing_errors, noise_errors;
  uint32_t dma_errors, restarts, restart_failures;
} ConsoleRx_Stats_t;
/** Start continuous RX; DMA buffer must reside in DMA-accessible SRAM, not CCM. */
HAL_StatusTypeDef ConsoleRx_Init(UART_HandleTypeDef *uart);
/** Main-loop recovery after UART/DMA error. Never stops TX DMA. */
void ConsoleRx_Process(void);
/** IRQ: consume the live DMA write position; HT/TC/IDLE must all remain enabled. */
void ConsoleRx_RxEventCallback(UART_HandleTypeDef *uart);
void ConsoleRx_ErrorCallback(UART_HandleTypeDef *uart);
void ConsoleRx_GetStats(ConsoleRx_Stats_t *out);
#endif
