#include "radar_uart.h"
#include <string.h>

#define RADAR_EVENT_QUEUE 2U
#define RADAR_COMMAND_GAP_MS 20U
#define RADAR_COMMAND_TIMEOUT_MS 500U
#define RADAR_PROBE_REPLY_MS 2000U
#define RADAR_PROBE_TEXT_GAP_MS 50U
typedef enum { COMMAND_NONE, COMMAND_START, COMMAND_STOP, COMMAND_PROBE } CommandMode_t;
static UART_HandleTypeDef *radar_uart;
/* The current scatter file places ordinary ZI in DMA-accessible SRAM, not CCM.
 * uint32_t storage also supplies the DMA buffer's alignment. */
static uint32_t rx_words[RADAR_UART_RX_BYTES / 4U];
static uint8_t work[RADAR_UART_PROCESS_BYTES];
static uint16_t work_count, work_at;
static LdsParser_t parser;
static LdsEvent_t events[RADAR_EVENT_QUEUE];
static uint32_t event_ends[RADAR_EVENT_QUEUE];
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
static const char probe_command[7] = "LUUIDH";
static const char sn_header[] = "PRODUCT SN:";
static RadarUart_Probe_t probe;
static uint32_t probe_tick, probe_rx_start, probe_parsed, probe_generation, probe_last_byte;
static uint8_t sn_match, sn_length;
static bool probe_receiving, sn_collecting, sn_complete;
/* Diagnostic storage is not DMA-owned. Only the main-loop reader writes it. */
static uint8_t raw_bytes[RADAR_UART_RAW_BYTES];
static RadarUart_Raw_t raw_capture;
static uint16_t raw_head;

static void raw_segment_lost(uint32_t position)
{
  if (!raw_capture.active) return;
  raw_capture.losses++; raw_capture.truncated = true;
  raw_capture.total = 0; raw_head = 0;
  raw_capture.window_start = position - raw_capture.first_position;
  raw_capture.received = raw_capture.window_start;
}

static void raw_append(const uint8_t *bytes, uint16_t count, uint32_t position)
{
  uint16_t i, tail;
  if (!raw_capture.active) return;
  for (i = 0; i < count; i++, position++) {
    if ((int32_t)(position - raw_capture.first_position) < 0) continue;
    if (!raw_capture.total) raw_capture.window_start = position - raw_capture.first_position;
    if (raw_capture.total == RADAR_UART_RAW_BYTES) {
      raw_head = (uint16_t)((raw_head + 1U) % RADAR_UART_RAW_BYTES);
      raw_capture.window_start++; raw_capture.truncated = true;
    } else raw_capture.total++;
    tail = (uint16_t)((raw_head + raw_capture.total - 1U) % RADAR_UART_RAW_BYTES);
    raw_bytes[tail] = bytes[i];
    raw_capture.received = position - raw_capture.first_position + 1U;
  }
}

static bool probe_active(void)
{
  return probe.state == RADAR_PROBE_TX || probe.state == RADAR_PROBE_WAITING;
}

static void probe_snapshot(uint32_t now)
{
  probe.elapsed_ms = now - probe_tick;
  if (probe_receiving) {
    probe.rx_bytes = produced - probe_rx_start;
  }
}

static void probe_finish(RadarUart_ProbeState_t state, const char *reason, uint32_t now)
{
  probe_snapshot(now); probe.state = state; probe.reason = reason;
}

/* Header facts: manufacturer's LDS-50C ROS UART driver queries LUUIDH and
 * searches PRODUCT SN:. No SDK source is copied. Recognize fragmented text
 * even amid continuous alarms; an echo or binary reception is not an answer. */
static void probe_byte(uint8_t byte, uint32_t position, uint32_t now)
{
  if (!probe_active() || !probe_receiving || (int32_t)(position - probe_rx_start) < 0) return;
  probe_parsed++;
  if (probe.raw_count < sizeof probe.raw) probe.raw[probe.raw_count++] = byte;
  if (sn_complete) return;
  if (sn_collecting) {
    if ((byte >= '0' && byte <= '9') || (byte >= 'A' && byte <= 'Z') ||
        (byte >= 'a' && byte <= 'z') || byte == '-' || byte == '_') {
      if (sn_length < sizeof probe.sn - 1U) probe.sn[sn_length++] = (char)byte;
      probe_last_byte = now;
    } else if (sn_length) { sn_complete = true; sn_collecting = false; }
    else if (byte != ' ' && byte != '\t') { sn_collecting = false; sn_match = 0; }
    return;
  }
  if (byte == (uint8_t)sn_header[sn_match]) {
    if (++sn_match == sizeof sn_header - 1U) { sn_collecting = true; sn_match = 0; }
  } else sn_match = byte == (uint8_t)sn_header[0] ? 1U : 0U;
}

static void service_probe(uint32_t now)
{
  if (!probe_active()) return;
  probe_snapshot(now);
  if (sn_collecting && sn_length && (uint32_t)(now - probe_last_byte) >= RADAR_PROBE_TEXT_GAP_MS)
    sn_complete = true;
  if (probe.tx_complete && sn_complete) probe_finish(RADAR_PROBE_CONFIRMED, "sn_reply", now);
  else if (probe.tx_complete && (uint32_t)(now - tx_completed_tick) >= RADAR_PROBE_REPLY_MS)
    probe_finish(RADAR_PROBE_UNCONFIRMED,
      !probe.rx_bytes ? "no_reply" : probe.alarms &&
        (int32_t)(probe_parsed - probe.alarms * 6U) >= -5 &&
        (int32_t)(probe_parsed - probe.alarms * 6U) <= 5 ? "alarm_only" : "reply_unknown", now);
  else if (!tx_active && command_mode == COMMAND_PROBE && (uint32_t)(now - probe_tick) >= RADAR_COMMAND_TIMEOUT_MS) {
    command_mode = COMMAND_NONE; probe_finish(RADAR_PROBE_ERROR, "tx_busy", now);
  }
}

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
  if (probe_active()) {
    sn_collecting = sn_complete = false; sn_match = sn_length = 0;
    memset(probe.sn, 0, sizeof probe.sn);
  }
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
  memset(&probe, 0, sizeof probe); probe.reason = "not_run";
  memset(&raw_capture, 0, sizeof raw_capture); raw_head = 0;
  probe_receiving = sn_collecting = sn_complete = false; sn_match = sn_length = 0;
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
  if (!radar_uart || probe_active()) return false;
  discard_input(); command_mode = COMMAND_START; command_index = 0;
  command_generation++;
  diagnostics.configured = false; diagnostics.command_failed = false;
  command_due_tick = HAL_GetTick();
  return true;
}

void RadarUart_StopScan(void)
{
  if (!radar_uart) return;
  if (probe_active()) probe_finish(RADAR_PROBE_CANCELLED, "operator_stop", HAL_GetTick());
  command_mode = COMMAND_STOP; command_index = 0;
  command_generation++;
  diagnostics.configured = false;
  command_due_tick = HAL_GetTick();
}

bool RadarUart_ProbeStart(void)
{
  uint32_t id = probe.id + 1U;
  if (!radar_uart || probe_active() || tx_active || command_mode != COMMAND_NONE || diagnostics.configured) return false;
  memset(&probe, 0, sizeof probe); probe.id = id ? id : 1U;
  probe.state = RADAR_PROBE_TX; probe.reason = "query_sending";
  probe_receiving = sn_collecting = sn_complete = false; sn_match = sn_length = 0;
  probe_parsed = 0;
  probe_tick = HAL_GetTick(); command_due_tick = probe_tick;
  command_mode = COMMAND_PROBE; probe_generation = ++command_generation;
  return true;
}

void RadarUart_ProbeGet(RadarUart_Probe_t *status)
{
  if (!status) return;
  if (probe_active()) { (void)producer_snapshot(); probe_snapshot(HAL_GetTick()); }
  *status = probe;
}

static void service_commands(uint32_t now)
{
  HAL_StatusTypeDef result;
  const char *command;
  if (tx_done) {
    tx_done = false; tx_active = false; diagnostics.commands_sent++;
    if (in_flight_mode == COMMAND_PROBE && in_flight_generation == probe_generation) {
      probe.tx_complete = true;
      if (probe_active()) { probe.state = RADAR_PROBE_WAITING; probe.reason = "query_waiting"; }
    }
    command_due_tick = tx_completed_tick + RADAR_COMMAND_GAP_MS;
    if (in_flight_mode == command_mode && in_flight_generation == command_generation) {
      if (command_mode == COMMAND_STOP) command_mode = COMMAND_NONE;
      else if (command_mode == COMMAND_PROBE) command_mode = COMMAND_NONE;
      else if (++command_index == 6U) {
        command_mode = COMMAND_NONE; diagnostics.configured = true;
      }
    }
  }
  if (tx_active) {
    if ((uint32_t)(now - tx_started_tick) < RADAR_COMMAND_TIMEOUT_MS) return;
    HAL_UART_AbortTransmit(radar_uart); tx_active = false;
    diagnostics.tx_errors++; diagnostics.command_failed = true;
    if (probe_active()) probe_finish(RADAR_PROBE_ERROR, "tx_timeout", now);
    command_mode = COMMAND_NONE; diagnostics.configured = false;
    return;
  }
  if (command_mode == COMMAND_NONE || (int32_t)(now - command_due_tick) < 0) return;
  command = command_mode == COMMAND_START ? startup[command_index] : command_mode == COMMAND_PROBE ? probe_command : stop_command;
  in_flight_mode = command_mode; in_flight_generation = command_generation;
  tx_active = true; tx_started_tick = now;
  /* Gate by absolute DMA cursor, excluding old bytes in both work[] and DMA.
   * Arm before IT transmission: a fast reply may precede main-loop TX service. */
  if (command_mode == COMMAND_PROBE) {
    probe_rx_start = producer_snapshot();
  }
  result = HAL_UART_Transmit_IT(radar_uart, (uint8_t *)command, 6U);
  if (result == HAL_OK) {
    memcpy(diagnostics.last_tx, command, sizeof diagnostics.last_tx);
    if (command_mode == COMMAND_PROBE) probe_receiving = true;
  }
  if (result != HAL_OK) {
    tx_active = false;
    if (result != HAL_BUSY) {
      diagnostics.tx_errors++; diagnostics.command_failed = true;
      if (probe_active()) probe_finish(RADAR_PROBE_ERROR, "tx_error", now);
      command_mode = COMMAND_NONE; diagnostics.configured = false;
    }
  }
}

static void queue_event(void)
{
  uint8_t tail = (uint8_t)((event_head + event_count) % RADAR_EVENT_QUEUE);
  events[tail] = *LdsParser_Event(&parser); event_count++;
  /* Poll may expose an inner frame while trailing bytes stay in parser.frame.
   * Subtracting those bytes identifies the frame's true RX end, not Poll time. */
  event_ends[tail] = consumed - work_count + work_at - parser.used;
  /* Count only frames completed by this query's fresh input. Old buffered
   * alarms and DMA bytes ahead of the parser must not skew its result. */
  if (probe_active() && probe_receiving &&
      (int32_t)(consumed - work_count + work_at - probe_rx_start) > 0 &&
      events[tail].kind == LDS_EVENT_ALARM) {
    probe.alarms++; probe.last_alarm = events[tail].data.alarm_code;
  }
}

void RadarUart_Process(void)
{
  uint32_t producer, available, read_start;
  uint16_t take, first, budget = RADAR_UART_PROCESS_BYTES;
  if (!radar_uart) return;
  service_commands(HAL_GetTick());
  if (rx_error) {
    rx_error = false; diagnostics.loss_generation++;
    raw_segment_lost(producer_snapshot());
    discard_input(); HAL_UART_AbortReceive(radar_uart);
    if (start_receive() == HAL_OK) diagnostics.restarts++;
    else diagnostics.restart_failures++;
  }
  /* Receive restart failures are retried by the main loop, preserving the
   * scan's timeout and published map rather than consuming a recovery quota. */
  if (!diagnostics.rx_ready) {
    if (start_receive() == HAL_OK) diagnostics.restarts++;
    else { diagnostics.restart_failures++; service_probe(HAL_GetTick()); return; }
  }
  producer = producer_snapshot(); available = producer - consumed;
  if (available > RADAR_UART_RX_BYTES) {
    diagnostics.overwritten_bytes += available - RADAR_UART_RX_BYTES;
    diagnostics.overflow_events++; diagnostics.loss_generation++;
    raw_segment_lost(producer);
    discard_input(); service_probe(HAL_GetTick()); return;
  }
  while (budget) {
    if (event_count == RADAR_EVENT_QUEUE) { diagnostics.queue_full++; break; }
    if (LdsParser_Poll(&parser)) { queue_event(); continue; }
    if (work_at == work_count) {
      producer = producer_snapshot(); available = producer - consumed;
      if (!available) break;
      if (available > RADAR_UART_RX_BYTES) {
        diagnostics.overwritten_bytes += available - RADAR_UART_RX_BYTES;
        diagnostics.overflow_events++; diagnostics.loss_generation++;
        raw_segment_lost(producer);
        discard_input(); service_probe(HAL_GetTick()); return;
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
        raw_segment_lost(producer);
        discard_input(); service_probe(HAL_GetTick()); return;
      }
      raw_append(work, take, read_start);
      consumed += take; work_count = take; work_at = 0;
    }
    probe_byte(work[work_at], consumed - work_count + work_at, HAL_GetTick());
    if (LdsParser_FeedByte(&parser, work[work_at++])) queue_event();
    diagnostics.parsed_bytes++; budget--;
  }
  service_probe(HAL_GetTick());
}

bool RadarUart_ReadEvent(LdsEvent_t *event)
{ return RadarUart_ReadEventAt(event, 0); }

bool RadarUart_ReadEventAt(LdsEvent_t *event, uint32_t *end_position)
{
  if (!event || !event_count) return false;
  if (end_position) *end_position = event_ends[event_head];
  *event = events[event_head]; event_head = (uint8_t)((event_head + 1U) % RADAR_EVENT_QUEUE);
  event_count--; return true;
}

void RadarUart_RawStart(void)
{
  memset(&raw_capture, 0, sizeof raw_capture); raw_head = 0;
  raw_capture.first_position = producer_snapshot();
  raw_capture.active = raw_capture.valid = true;
}

void RadarUart_RawStop(void)
{
  uint32_t end, position, remaining, crc = 0xffffffffU;
  uint16_t take;
  unsigned i, bit;
  if (!raw_capture.active) return;
  /* work[] was recorded on the successful DMA copy. Freeze the still unread
   * DMA tail independently of parser admission, including alarms/bad frames. */
  end = producer_snapshot(); position = consumed; remaining = end - position;
  if (remaining > RADAR_UART_RX_BYTES) {
    position = end - RADAR_UART_RX_BYTES;
    raw_segment_lost(position); remaining = RADAR_UART_RX_BYTES;
  }
  if (rx_error) raw_segment_lost(position);
  while (remaining) {
    take = (uint16_t)(RADAR_UART_RX_BYTES - ((position - ring_base) % RADAR_UART_RX_BYTES));
    if (take > remaining) take = (uint16_t)remaining;
    raw_append((uint8_t *)rx_words + ((position - ring_base) % RADAR_UART_RX_BYTES), take, position);
    position += take; remaining -= take;
  }
  for (i = 0; i < raw_capture.total; i++) {
    crc ^= raw_bytes[(raw_head + i) % RADAR_UART_RAW_BYTES];
    for (bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ ((crc & 1U) ? 0xedb88320U : 0U);
  }
  raw_capture.crc32 = crc ^ 0xffffffffU; raw_capture.active = false;
}

void RadarUart_RawGet(RadarUart_Raw_t *out)
{ if (out) *out = raw_capture; }

uint16_t RadarUart_RawRead(uint32_t offset, uint8_t *bytes, uint16_t capacity)
{
  uint16_t count, first, at;
  if (!bytes || !raw_capture.valid || raw_capture.active || offset >= raw_capture.total) return 0;
  count = (uint16_t)(raw_capture.total - offset);
  if (count > capacity) count = capacity;
  at = (uint16_t)((raw_head + offset) % RADAR_UART_RAW_BYTES);
  first = (uint16_t)(RADAR_UART_RAW_BYTES - at); if (first > count) first = count;
  memcpy(bytes, raw_bytes + at, first);
  if (first < count) memcpy(bytes + first, raw_bytes, count - first);
  return count;
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
