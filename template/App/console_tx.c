#include "console_tx.h"
#include <string.h>
#define REPLY_SIZE 8192U
#define FRAME_SIZE 2048U
#define EVENT_SIZE 128U
enum { TX_REPLY, TX_EVENT, TX_TELEMETRY };
static UART_HandleTypeDef *port;
static uint8_t replies[REPLY_SIZE], active[FRAME_SIZE];
static char telemetry[FRAME_SIZE];
static char event[EVENT_SIZE];
static uint16_t read_at, write_at, used, telemetry_size, event_size, active_size;
static uint16_t active_offset, chunk_size;
static uint32_t started, dropped, reply_dropped, event_dropped;
static volatile uint32_t completed_at;
static uint32_t gap_started;
static bool gap_valid;
static uint8_t active_kind, busy;
static volatile uint8_t completed, failed;

void ConsoleTx_Init(UART_HandleTypeDef *uart)
{
  port = uart;
  read_at = write_at = used = telemetry_size = event_size = active_size = 0;
  started = dropped = reply_dropped = event_dropped = 0;
  active_kind = busy = completed = failed = 0;
  active_offset = chunk_size = 0;
  completed_at = gap_started = 0;
  gap_valid = false;
}
bool ConsoleTx_Write(const uint8_t *data, uint16_t size)
{
  uint16_t i;
  if (!port || !data || !size || size > REPLY_SIZE - used)
  {
    reply_dropped++;
    return false;
  }
  for (i = 0; i < size; i++)
  {
    replies[write_at] = data[i];
    write_at = (write_at + 1U) % REPLY_SIZE;
  }
  used += size;
  return true;
}
bool ConsoleTx_Telemetry(const char *data, uint16_t size)
{
  if (!port || !data || !size || size > FRAME_SIZE)
  {
    dropped++;
    return false;
  }
  if (telemetry_size)
    dropped++;
  memcpy(telemetry, data, size);
  telemetry_size = size;
  return true;
}
bool ConsoleTx_Event(const char *data, uint16_t size)
{
  if (!port || !data || !size || size > EVENT_SIZE)
  {
    event_dropped++;
    return false;
  }
  if (event_size) event_dropped++;
  memcpy(event, data, size);
  event_size = size;
  return true;
}
void ConsoleTx_TelemetryCancel(void)
{
  if (telemetry_size) dropped++;
  telemetry_size = 0;
}
bool ConsoleTx_TelemetryReady(void)
{
  return port && !busy && !active_size && !used && !event_size && !telemetry_size;
}
void ConsoleTx_EventCancel(void)
{
  if (event_size) event_dropped++;
  event_size = 0;
}
static void ConsoleTx_ActiveDropped(void)
{
  if (active_kind == TX_TELEMETRY) dropped++;
  else if (active_kind == TX_EVENT) event_dropped++;
  else reply_dropped++;
}
void ConsoleTx_Process(void)
{
  uint16_t i, n;
  HAL_StatusTypeDef status;
  if (!port)
    return;
  if (busy)
  {
    if (failed || (!completed && (uint32_t)(HAL_GetTick() - started) > 500U))
    {
      (void)HAL_UART_AbortTransmit(port);
      ConsoleTx_ActiveDropped();
      active_size = 0;
      gap_started = HAL_GetTick();
    }
    else if (!completed)
      return;
    else
    {
      active_offset += chunk_size;
      if (active_offset == active_size) active_size = 0;
      gap_started = completed_at;
    }
    gap_valid = true;
    busy = completed = failed = 0;
  }
  if (gap_valid && (uint32_t)(HAL_GetTick() - gap_started) < CONSOLE_TX_GAP_MS)
    return;
  if (!active_size)
  {
    active_offset = 0;
    if (used)
    {
      n = used > 256U ? 256U : used;
      for (i = 0; i < n; i++)
        active[i] = replies[(read_at + i) % REPLY_SIZE];
      active_size = n;
      active_kind = TX_REPLY;
    }
    else if (event_size)
    {
      memcpy(active, event, event_size);
      active_size = event_size;
      active_kind = TX_EVENT;
    }
    else if (telemetry_size)
    {
      memcpy(active, telemetry, telemetry_size);
      active_size = telemetry_size;
      active_kind = TX_TELEMETRY;
    }
    else
      return;
  }
  /* HAL may complete immediately in an IRQ; set flags before enabling TX. */
  completed = failed = 0;
  busy = 1;
  started = HAL_GetTick();
  chunk_size = active_size - active_offset;
  if (chunk_size > CONSOLE_TX_CHUNK_BYTES) chunk_size = CONSOLE_TX_CHUNK_BYTES;
  status = HAL_UART_Transmit_IT(port, active + active_offset, chunk_size);
  /* Commit queue removal only when HAL takes the frame (or rejects it).
     HAL_BUSY leaves each slot intact, so a later reply can still take priority. */
  if (active_offset == 0 && status != HAL_BUSY)
  {
    if (active_kind == TX_REPLY)
    {
      read_at = (read_at + active_size) % REPLY_SIZE;
      used -= active_size;
    }
    else if (active_kind == TX_EVENT) event_size = 0;
    else telemetry_size = 0;
  }
  if (status != HAL_OK)
  {
    busy = 0;
    if (status != HAL_BUSY)
    {
      ConsoleTx_ActiveDropped();
      active_size = 0;
      gap_started = HAL_GetTick();
      gap_valid = true;
    }
    else if (active_offset == 0)
      active_size = 0; /* No byte sent: a reply may still take priority. */
    /* Once started, retain this line across gaps/HAL_BUSY; never splice replies. */
  }
}
void ConsoleTx_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == port)
  {
    completed_at = HAL_GetTick();
    completed = 1;
  }
}
void ConsoleTx_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == port)
    failed = 1;
}
uint32_t ConsoleTx_Dropped(void)
{
  return dropped;
}
uint32_t ConsoleTx_ReplyDropped(void)
{
  return reply_dropped;
}
uint32_t ConsoleTx_EventDropped(void)
{
  return event_dropped;
}
