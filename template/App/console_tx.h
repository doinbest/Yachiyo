#ifndef CONSOLE_TX_H
#define CONSOLE_TX_H
#include "main.h"
#include <stdbool.h>
/* Wireless pacing trial values; validate on the actual link after flashing. */
#define CONSOLE_TX_CHUNK_BYTES 32U
#define CONSOLE_TX_GAP_MS 30U
/** USART1 only. Main-loop producers; IRQ callbacks only set completion flags. */
void ConsoleTx_Init(UART_HandleTypeDef *uart);
/** @return copied into reply queue, NOT transmission completion. */
bool ConsoleTx_Write(const uint8_t *data, uint16_t size);
/** Latest telemetry replaces an unsent telemetry line; replies take priority. */
bool ConsoleTx_Telemetry(const char *data, uint16_t size);
/** Cancel only the unsent telemetry slot. An active JSON line finishes intact. */
void ConsoleTx_TelemetryCancel(void);
/** Whether a new telemetry snapshot can be prepared without a TX backlog. */
bool ConsoleTx_TelemetryReady(void);
/** @brief Copy latest QR event into its independent pending slot.
 * @return Accepted into RAM, not transmitted. Reply > event > telemetry.
 */
bool ConsoleTx_Event(const char *data, uint16_t size);
/** @brief Cancel an unsent QR event. Never aborts an in-flight transmission. */
void ConsoleTx_EventCancel(void);
/** @return QR events replaced, cancelled, rejected or lost during TX. */
uint32_t ConsoleTx_EventDropped(void);
void ConsoleTx_Process(void);
void ConsoleTx_TxCpltCallback(UART_HandleTypeDef *uart);
void ConsoleTx_ErrorCallback(UART_HandleTypeDef *uart);
uint32_t ConsoleTx_Dropped(void);
uint32_t ConsoleTx_ReplyDropped(void);
#endif
