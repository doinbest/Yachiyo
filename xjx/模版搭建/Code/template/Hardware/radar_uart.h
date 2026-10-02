/** @file radar_uart.h
 * @brief USART2 circular DMA reception and nonblocking LDS50C commands.
 */
#ifndef RADAR_UART_H
#define RADAR_UART_H
#include "main.h"
#include "lds_parser.h"
#define RADAR_UART_RX_BYTES 4096U
#define RADAR_UART_PROCESS_BYTES 512U
typedef struct {
  uint32_t received_bytes, parsed_bytes, packets, status_packets, bad;
  uint32_t overwritten_bytes, overflow_events, queue_full, loss_generation;
  uint32_t uart_errors, restarts, restart_failures, tx_errors, commands_sent, alarms;
  uint16_t last_alarm;
  bool configured, configuring, stop_pending, command_failed, rx_ready;
} RadarUart_Status_t;
HAL_StatusTypeDef RadarUart_Init(UART_HandleTypeDef *uart);
/** @brief Main-loop service, at most 512 input bytes; no HAL_Delay. */
void RadarUart_Process(void);
void RadarUart_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void RadarUart_TxCpltCallback(UART_HandleTypeDef *uart);
void RadarUart_ErrorCallback(UART_HandleTypeDef *uart);
/** @brief Schedule STOP/MM/energy/deshadow/filter/START, 20ms TX-completion gaps. */
bool RadarUart_StartScan(void);
/** @brief Schedule LSTOPH; reception and lifetime diagnostics remain active. */
void RadarUart_StopScan(void);
bool RadarUart_ReadEvent(LdsEvent_t *event);
void RadarUart_StatusGet(RadarUart_Status_t *status);
#endif
