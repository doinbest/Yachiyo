#include "radar_uart.h"
#include <string.h>

#define RADAR_EVENT_QUEUE 2U
#define RADAR_COMMAND_GAP_MS 20U
#define RADAR_COMMAND_TIMEOUT_MS 500U
typedef enum { COMMAND_NONE, COMMAND_START, COMMAND_STOP } CommandMode_t;
static UART_HandleTypeDef *radar_uart;
/* The current scatter file places ordinary ZI in DMA-accessible SRAM, not CCM.
 * uint32_t storage also supplies the DMA buffer's alignment. */
static uint32_t rx_words[RADAR_UART_RX_BYTES / 4U];
static uint8_t work[RADAR_UART_PROCESS_BYTES];
static uint16_t work_count, work_at;
static LdsParser_t parser;
static LdsEvent_t events[RADAR_EVENT_QUEUE];
static uint8_t event_head, event_count;
static RadarUart_Status_t diagnostics;
static volatile uint32_t irq_errors;
static volatile uint32_t produced, tc_epochs, inferred_epochs;
static uint32_t ring_base, consumed;
static volatile uint32_t last_epoch;
static volatile uint16_t last_position;
static volatile bool rx_error;
static CommandMode_t command_mode, in_flight_mode;
static uint8_t command_index;
static uint32_t command_generation, in_flight_generation;
static volatile bool tx_active, tx_done;
static volatile uint32_t tx_completed_tick;
static uint32_t tx_started_tick, command_due_tick;
static const char startup[6][7] = {
  "LSTOPH", "LMDMMH", "LOCONH", "LFFF1H", "LSSS1H", "LSTARH"
};
static const char stop_command[7] = "LSTOPH";

/* Use live NDTR for all three events. A wrap first observed by IDLE/main is
 * inferred once; the subsequently delivered TC merely catches up that epoch.
 * HT/TC stay enabled, so a main-loop stall spanning multiple turns is counted. */
static void capture_cursor(bool transfer_complete)
{
  uint32_t epoch, candidate, remaining;
  uint16_t position;
  if (!radar_uart || !radar_uart->hdmarx || !diagnostics.rx_ready) return;
  if (transfer_complete) tc_epochs++;
  remaining = __HAL_DMA_GET_COUNTER(radar_uart->hdmarx);
  if (remaining > RADAR_UART_RX_BYTES) return;
  position = (uint16_t)((RADAR_UART_RX_BYTES - remaining) % RADAR_UART_RX_BYTES);
  epoch = tc_epochs > inferred_epochs ? tc_epochs : inferred_epochs;
  if (position < last_position && epoch == last_epoch) {
    epoch++; inferred_epochs = epoch;
  }
  candidate = ring_base + epoch * RADAR_UART_RX_BYTES + position;
  if ((int32_t)(candidate - produced) >= 0) {
    produced = candidate; last_position = position; last_epoch = epoch;
  }
}

static uint32_t producer_snapshot(void)
{
  uint32_t mask = __get_PRIMASK(), result;
  __disable_irq(); capture_cursor(false); result = produced; __set_PRIMASK(mask);
  return result;
}

static void discard_input(void)
{
  consumed = producer_snapshot(); work_count = work_at = 0;
  event_head = event_count = 0; LdsParser_Reset(&parser);
}

static void reset_cursor(void)
{
  ring_base = produced; consumed = produced;
  tc_epochs = inferred_epochs = last_epoch = 0; last_position = 0;
}

static HAL_StatusTypeDef start_receive(void)
{
  HAL_StatusTypeDef result;
  reset_cursor(); diagnostics.rx_ready = false;
  result = HAL_UARTEx_ReceiveToIdle_DMA(radar_uart, (uint8_t *)rx_words, RADAR_UART_RX_BYTES);
  diagnostics.rx_ready = result == HAL_OK;
  return result;
}

HAL_StatusTypeDef RadarUart_Init(UART_HandleTypeDef *uart)
{
  if (!uart || uart->Instance != USART2 || !uart->hdmarx) return HAL_ERROR;
  radar_uart = uart;
  memset(&diagnostics, 0, sizeof diagnostics); LdsParser_Init(&parser);
  irq_errors = 0;
  produced = ring_base = consumed = tc_epochs = inferred_epochs = last_epoch = 0;
  last_position = 0; work_count = work_at = event_head = event_count = 0;
  rx_error = tx_active = tx_done = false;
  command_mode = in_flight_mode = COMMAND_NONE; command_index = 0;
  command_generation = in_flight_generation = 0;
  command_due_tick = HAL_GetTick();
  return start_receive();
}

void RadarUart_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  (void)size;
  if (uart != radar_uart) return;
  capture_cursor(HAL_UARTEx_GetRxEventType(uart) == HAL_UART_RXEVENT_TC);
}

void RadarUart_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart != radar_uart || !tx_active) return;
  tx_completed_tick = HAL_GetTick(); tx_done = true;
}

void RadarUart_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart != radar_uart) return;
  irq_errors++; rx_error = true;
}

bool RadarUart_StartScan(void)
{
  if (!radar_uart) return false;
  discard_input(); command_mode = COMMAND_START; command_index = 0;
  command_generation++;
  diagnostics.configured = false; diagnostics.command_failed = false;
  command_due_tick = HAL_GetTick();
  return true;
}

void RadarUart_StopScan(void)
{
  if (!radar_uart) return;
  command_mode = COMMAND_STOP; command_index = 0;
  command_generation++;
  diagnostics.configured = false;
  command_due_tick = HAL_GetTick();
}

static void service_commands(uint32_t now)
{
  HAL_StatusTypeDef result;
  const char *command;
  if (tx_done) {
    tx_done = false; tx_active = false; diagnostics.commands_sent++;
    command_due_tick = tx_completed_tick + RADAR_COMMAND_GAP_MS;
    if (in_flight_mode == command_mode && in_flight_generation == command_generation) {
      if (command_mode == COMMAND_STOP) command_mode = COMMAND_NONE;
      else if (++command_index == 6U) {
        command_mode = COMMAND_NONE; diagnostics.configured = true;
      }
    }
  }
  if (tx_active) {
    if ((uint32_t)(now - tx_started_tick) < RADAR_COMMAND_TIMEOUT_MS) return;
    HAL_UART_AbortTransmit(radar_uart); tx_active = false;
    diagnostics.tx_errors++; diagnostics.command_failed = true;
    command_mode = COMMAND_NONE; diagnostics.configured = false;
    return;
  }
  if (command_mode == COMMAND_NONE || (int32_t)(now - command_due_tick) < 0) return;
  command = command_mode == COMMAND_START ? startup[command_index] : stop_command;
  in_flight_mode = command_mode; in_flight_generation = command_generation;
  tx_active = true; tx_started_tick = now;
  result = HAL_UART_Transmit_IT(radar_uart, (uint8_t *)command, 6U);
  if (result != HAL_OK) {
    tx_active = false;
    if (result != HAL_BUSY) {
      diagnostics.tx_errors++; diagnostics.command_failed = true;
      command_mode = COMMAND_NONE; diagnostics.configured = false;
    }
  }
}

static void queue_event(void)
{
  uint8_t tail = (uint8_t)((event_head + event_count) % RADAR_EVENT_QUEUE);
  events[tail] = *LdsParser_Event(&parser); event_count++;
}

void RadarUart_Process(void)
{
  uint32_t producer, available, read_start;
  uint16_t take, first, budget = RADAR_UART_PROCESS_BYTES;
  if (!radar_uart) return;
  service_commands(HAL_GetTick());
  if (rx_error) {
    rx_error = false; diagnostics.loss_generation++;
    discard_input(); HAL_UART_AbortReceive(radar_uart);
    if (start_receive() == HAL_OK) diagnostics.restarts++;
    else diagnostics.restart_failures++;
  }
  /* Receive restart failures are retried by the main loop, preserving the
   * scan's timeout and published map rather than consuming a recovery quota. */
  if (!diagnostics.rx_ready) {
    if (start_receive() == HAL_OK) diagnostics.restarts++;
    else { diagnostics.restart_failures++; return; }
  }
  producer = producer_snapshot(); available = producer - consumed;
  if (available > RADAR_UART_RX_BYTES) {
    diagnostics.overwritten_bytes += available - RADAR_UART_RX_BYTES;
    diagnostics.overflow_events++; diagnostics.loss_generation++;
    discard_input(); return;
  }
  while (budget) {
    if (event_count == RADAR_EVENT_QUEUE) { diagnostics.queue_full++; return; }
    if (LdsParser_Poll(&parser)) { queue_event(); continue; }
    if (work_at == work_count) {
      producer = producer_snapshot(); available = producer - consumed;
      if (!available) break;
      if (available > RADAR_UART_RX_BYTES) {
        diagnostics.overwritten_bytes += available - RADAR_UART_RX_BYTES;
        diagnostics.overflow_events++; diagnostics.loss_generation++;
        discard_input(); return;
      }
      take = (uint16_t)(available < budget ? available : budget);
      read_start = consumed;
      first = (uint16_t)(RADAR_UART_RX_BYTES - ((consumed - ring_base) % RADAR_UART_RX_BYTES));
      if (first > take) first = take;
      memcpy(work, (uint8_t *)rx_words + ((consumed - ring_base) % RADAR_UART_RX_BYTES), first);
      if (first < take) memcpy(work + first, rx_words, take - first);
      producer = producer_snapshot();
      if (producer - read_start > RADAR_UART_RX_BYTES) {
        diagnostics.overwritten_bytes += producer - read_start - RADAR_UART_RX_BYTES;
        diagnostics.overflow_events++; diagnostics.loss_generation++;
        discard_input(); return;
      }
      consumed += take; work_count = take; work_at = 0;
    }
    if (LdsParser_FeedByte(&parser, work[work_at++])) queue_event();
    diagnostics.parsed_bytes++; budget--;
  }
}

bool RadarUart_ReadEvent(LdsEvent_t *event)
{
  if (!event || !event_count) return false;
  *event = events[event_head]; event_head = (uint8_t)((event_head + 1U) % RADAR_EVENT_QUEUE);
  event_count--; return true;
}

void RadarUart_StatusGet(RadarUart_Status_t *status)
{
  if (!status) return;
  *status = diagnostics;
  status->uart_errors = irq_errors;
  status->received_bytes = producer_snapshot(); status->packets = parser.stats.packets;
  status->status_packets = parser.stats.status_packets;
  status->bad = parser.stats.bad_checksum + parser.stats.bad_length;
  status->alarms = parser.stats.alarms; status->last_alarm = parser.stats.last_alarm;
  status->configuring = command_mode == COMMAND_START;
  status->stop_pending = command_mode == COMMAND_STOP || (tx_active && in_flight_mode == COMMAND_STOP);
}
