#include "console_tx.h"
#include <string.h>
#define REPLY_SIZE 8192U
#define FRAME_SIZE 2048U
#define EVENT_SIZE 128U
#define URGENT_SIZE 1024U
#define DEBUG_SIZE 320U
enum { TX_REPLY, TX_EVENT, TX_TELEMETRY, TX_URGENT, TX_DEBUG, TX_BULK };
static char bulk[256];
static uint16_t bulk_size;
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
static uint8_t urgent[URGENT_SIZE];
static uint16_t urgent_read, urgent_write, urgent_used, reply_peak;
static char debug[CONSOLE_DEBUG_COUNT][DEBUG_SIZE];
static uint16_t debug_size[CONSOLE_DEBUG_COUNT];
static uint32_t debug_started[CONSOLE_DEBUG_COUNT];
static bool debug_sent[CONSOLE_DEBUG_COUNT];
static unsigned debug_next, active_debug;
static bool reply_continues, background_paused;
static uint32_t bytes_sent, urgent_dropped, debug_dropped;

void ConsoleTx_Init(UART_HandleTypeDef *uart)
{
  port = uart;
  read_at = write_at = used = telemetry_size = event_size = active_size = 0;
  started = dropped = reply_dropped = event_dropped = 0;
  active_kind = busy = completed = failed = 0;
  active_offset = chunk_size = 0;
  completed_at = gap_started = 0;
  gap_valid = false;
  urgent_read = urgent_write = urgent_used = reply_peak = 0;
  bytes_sent = urgent_dropped = debug_dropped = 0;
  memset(debug_size, 0, sizeof(debug_size));
  memset(debug_sent, 0, sizeof(debug_sent));
  debug_next = active_debug = 0;
  reply_continues = background_paused = false;
  bulk_size=0;
}
bool ConsoleTx_BulkReady(void)
{
  return port && !background_paused && !bulk_size && !(active_size && active_kind==TX_BULK);
}
bool ConsoleTx_Bulk(const char *data,uint16_t size)
{
  if(!data || !size || size>sizeof(bulk) || !ConsoleTx_BulkReady()) return false;
  memcpy(bulk,data,size);bulk_size=size;return true;
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
  if (used > reply_peak) reply_peak = used;
  return true;
}
bool ConsoleTx_Urgent(const char *data, uint16_t size)
{
  uint16_t i;
  if (!port || !data || !size || size > URGENT_SIZE - urgent_used)
  { urgent_dropped++; return false; }
  for (i = 0; i < size; i++) {
    urgent[urgent_write] = (uint8_t)data[i];
    urgent_write = (urgent_write + 1U) % URGENT_SIZE;
  }
  urgent_used += size;
  return true;
}
bool ConsoleTx_Debug(unsigned source, const char *data, uint16_t size)
{
  if (!port || source >= CONSOLE_DEBUG_COUNT || !data || !size || size > DEBUG_SIZE)
  { debug_dropped++; return false; }
  if (background_paused) return false;
  if (debug_size[source]) debug_dropped++;
  memcpy(debug[source], data, size);
  debug_size[source] = size;
  return true;
}
void ConsoleTx_DebugCancel(unsigned source)
{
  if (source < CONSOLE_DEBUG_COUNT) {
    if (debug_size[source]) debug_dropped++;
    debug_size[source] = 0;
  }
}
void ConsoleTx_GetStats(ConsoleTx_Stats_t *out)
{
  if (!out) return;
  out->bytes_sent = bytes_sent;
  out->reply_pending = used;
  out->reply_peak = reply_peak;
  out->reply_dropped = reply_dropped;
  out->urgent_dropped = urgent_dropped;
  out->debug_dropped = debug_dropped;
}
bool ConsoleTx_Telemetry(const char *data, uint16_t size)
{
  if (!port || !data || !size || size > FRAME_SIZE)
  {
    dropped++;
    return false;
  }
  if (background_paused) return false;
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
void ConsoleTx_BackgroundPause(bool paused)
{
  unsigned i;
  if (background_paused == paused) return;
  background_paused = paused;
  if (!paused) return;
  ConsoleTx_TelemetryCancel();
  bulk_size=0;
  for (i = 0; i < CONSOLE_DEBUG_COUNT; i++) ConsoleTx_DebugCancel(i);
  /* Unstarted selection after HAL_BUSY can be discarded; started frames drain intact. */
  if (!busy && active_offset == 0 &&
      (active_kind == TX_TELEMETRY || active_kind == TX_DEBUG || active_kind==TX_BULK)) active_size = 0;
}
bool ConsoleTx_UrgentIdle(void)
{
  return port && !urgent_used && !(active_size && active_kind == TX_URGENT);
}
bool ConsoleTx_TelemetryReady(void)
{
  return port && !background_paused && !busy && !active_size && !used && !urgent_used && !event_size && !telemetry_size;
}
void ConsoleTx_EventCancel(void)
{
  if (event_size) event_dropped++;
  event_size = 0;
}
static void ConsoleTx_ActiveDropped(void)
{
  if (active_kind == TX_TELEMETRY || active_kind == TX_BULK) dropped++;
  else if (active_kind == TX_EVENT) event_dropped++;
  else if (active_kind == TX_URGENT) urgent_dropped++;
  else if (active_kind == TX_DEBUG) debug_dropped++;
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
      reply_continues = false;
      active_size = 0;
      gap_started = HAL_GetTick();
    }
    else if (!completed)
      return;
    else
    {
      active_offset += chunk_size;
      bytes_sent += chunk_size;
      if (active_offset == active_size) {
        if (active_kind == TX_REPLY)
          reply_continues = used && active[active_size - 1U] != '\n';
        active_size = 0;
      }
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
    if (urgent_used && !reply_continues)
    {
      active_size = urgent_used;
      for (i = 0; i < active_size; i++) active[i] = urgent[(urgent_read + i) % URGENT_SIZE];
      active_kind = TX_URGENT;
    }
    else if (used)
    {
      n = used > 256U ? 256U : used;
      for (i = 0; i < n; i++) {
        active[i] = replies[(read_at + i) % REPLY_SIZE];
        if (active[i] == '\n') { n = i + 1U; break; }
      }
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
    else if (bulk_size)
    {
      memcpy(active,bulk,bulk_size);active_size=bulk_size;active_kind=TX_BULK;
    }
    else {
      for (i = 0; i < CONSOLE_DEBUG_COUNT; i++) {
        active_debug = (debug_next + i) % CONSOLE_DEBUG_COUNT;
        if (debug_size[active_debug] && (!debug_sent[active_debug] ||
            (uint32_t)(HAL_GetTick() - debug_started[active_debug]) >= 500U)) break;
      }
      if (i == CONSOLE_DEBUG_COUNT) return;
      active_size = debug_size[active_debug];
      memcpy(active, debug[active_debug], active_size);
      active_kind = TX_DEBUG;
    }
  }
  /* HAL may complete immediately in an IRQ; set flags before enabling TX. */
  completed = failed = 0;
  busy = 1;
  started = HAL_GetTick();
  chunk_size = active_size - active_offset;
  if (chunk_size > CONSOLE_TX_CHUNK_BYTES) chunk_size = CONSOLE_TX_CHUNK_BYTES;
  status = HAL_UART_Transmit_DMA(port, active + active_offset, chunk_size);
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
    else if (active_kind == TX_BULK) bulk_size=0;
    else if (active_kind == TX_URGENT) {
      urgent_read = (urgent_read + active_size) % URGENT_SIZE;
      urgent_used -= active_size;
    }
    else if (active_kind == TX_DEBUG) {
      debug_size[active_debug] = 0;
      debug_started[active_debug] = HAL_GetTick();
      debug_sent[active_debug] = true;
      debug_next = (active_debug + 1U) % CONSOLE_DEBUG_COUNT;
    }
    else telemetry_size = 0;
  }
  if (status != HAL_OK)
  {
    busy = 0;
    if (status != HAL_BUSY)
    {
      ConsoleTx_ActiveDropped();
      reply_continues = false;
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
