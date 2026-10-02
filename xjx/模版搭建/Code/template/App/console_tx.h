#ifndef CONSOLE_TX_H
#define CONSOLE_TX_H
#include "main.h"
#include <stdbool.h>
/* Wireless pacing trial values; validate on the actual link after flashing. */
#define CONSOLE_TX_CHUNK_BYTES 32U
#define CONSOLE_TX_GAP_MS 30U
enum { CONSOLE_DEBUG_IMU, CONSOLE_DEBUG_VISION, CONSOLE_DEBUG_CAMERA, CONSOLE_DEBUG_COUNT };
typedef struct {
  uint32_t bytes_sent, reply_dropped, urgent_dropped, debug_dropped;
  uint16_t reply_pending, reply_peak;
} ConsoleTx_Stats_t;
/** Main-loop only: reserved FIFO for stop replies, selected at text/frame boundaries. */
/** @brief 主循环检查紧急回复队列及其在途DMA是否已全部结束。 */
bool ConsoleTx_UrgentIdle(void);
bool ConsoleTx_Urgent(const char *data, uint16_t size);
/** Main-loop only: each debug source keeps its latest unsent sample (max 320 bytes). */
bool ConsoleTx_Debug(unsigned source, const char *data, uint16_t size);
void ConsoleTx_DebugCancel(unsigned source);
void ConsoleTx_GetStats(ConsoleTx_Stats_t *out);
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
/** @brief Independent low-priority radar page, <=256 bytes. Never overwrites
 * chassis telemetry; urgent replies win at the next complete page boundary. */
bool ConsoleTx_Bulk(const char *data, uint16_t size);
bool ConsoleTx_BulkReady(void);
/** @brief Copy latest QR event into its independent pending slot.
 * @return Accepted into RAM, not transmitted. Reply > event > telemetry.
 */
bool ConsoleTx_Event(const char *data, uint16_t size);
/** @brief Cancel an unsent QR event. Never aborts an in-flight transmission. */
void ConsoleTx_EventCancel(void);
/** @return QR events replaced, cancelled, rejected or lost during TX. */
uint32_t ConsoleTx_EventDropped(void);
/** @brief Pause latest-only telemetry/debug producers, drain any started frame intact.
 * Main-loop only; ordinary and urgent replies remain available. */
void ConsoleTx_BackgroundPause(bool paused);
void ConsoleTx_Process(void);
void ConsoleTx_TxCpltCallback(UART_HandleTypeDef *uart);
void ConsoleTx_ErrorCallback(UART_HandleTypeDef *uart);
uint32_t ConsoleTx_Dropped(void);
uint32_t ConsoleTx_ReplyDropped(void);
#endif
