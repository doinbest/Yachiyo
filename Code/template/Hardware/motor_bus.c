#include "motor_bus.h"
#include <string.h>

typedef struct
{
  uint8_t data[MOTOR_BUS_FRAME_MAX], length, reply_length;
  uint32_t token;
  bool queued, event_ready, priority;
  MotorBus_Event_t event;
} Slot;
static UART_HandleTypeDef *bus_uart;
static Slot slots[MOTOR_BUS_OWNER_COUNT], active;
static MotorBus_Owner_t active_owner;
static MotorBus_Owner_t reserved_owner = MOTOR_BUS_OWNER_COUNT;
static bool running, waiting_reply, cancelled, quarantined, gap_valid;
static bool active_tx_completed;
static uint32_t started_ms, gap_ms;
static volatile bool tx_done, io_error;
static volatile bool io_overflow;
static volatile uint32_t io_uart_error, io_tick_ms;
static MotorBus_Diagnostic_t first_fault;
static volatile uint32_t tx_done_ms;
static uint8_t dma_rx[64], rx_ring[256], parse[32];
static uint32_t rx_time[256];
static volatile uint16_t rx_write, rx_read;
/* HAL circular RX callback Size is a buffer position (1..64), not a new length. */
static uint16_t dma_rx_position;
static uint8_t parse_length;
/* One abandoned read reply must be consumed before another ordinary request.
 * Stop frames may still transmit; no drive motion command gets this exception. */
static bool drain_pending;
static bool diagnostic_reads;
static MotorBus_Owner_t drain_owner;
static uint8_t drain_address, drain_function, drain_length;
static uint32_t drain_started_ms;
static MotorBus_Recovery_t recovery = {false, false, 0, 0, "idle"};
static uint8_t recovery_step, recovery_round, recovery_kind;
static uint32_t recovery_started, recovery_wait;
static volatile uint32_t last_rx_ms;
static void RecoveryProcess(void);
static void RecoveryFail(const char *reason);

static bool RecoveryRequest(MotorBus_Owner_t owner)
{
  return recovery.active && owner == MOTOR_BUS_RECOVERY;
}

/* Main-loop only. Preserve the first cause across subsequent rejected requests. */
static void Quarantine(const char *reason, uint32_t tick_ms, uint32_t uart_error)
{
  if (!quarantined)
  {
    first_fault.reason = reason;
    first_fault.tick_ms = tick_ms;
    first_fault.uart_error = uart_error;
    first_fault.owner = active.length ? active_owner : MOTOR_BUS_OWNER_COUNT;
    first_fault.address = active.length ? active.data[0] : 0U;
    first_fault.function = active.length ? active.data[1] : 0U;
    first_fault.active = running;
    first_fault.waiting_reply = waiting_reply;
  }
  quarantined = true;
}

static bool FeedbackRead(MotorBus_Owner_t owner, const uint8_t *data,
                         uint8_t length, uint8_t reply_length)
{
  return owner == MOTOR_BUS_FEEDBACK && data && length == 3U &&
         data[0] >= 1U && data[0] <= 4U && data[2] == 0x6b &&
         ((data[1] == 0x35 && reply_length == 6U) ||
          (data[1] == 0x36 && reply_length == 8U) ||
          (data[1] == 0x3a && reply_length == 4U));
}

static HAL_StatusTypeDef ReceiveStart(void)
{
  /* Only initialization/error recovery starts DMA. Keep HT enabled so a stream
   * without IDLE is copied before the DMA buffer wraps and overwrites it. */
  dma_rx_position = 0;
  return HAL_UARTEx_ReceiveToIdle_DMA(bus_uart, dma_rx, sizeof(dma_rx));
}
static void Complete(MotorBus_Result_t result, const uint8_t *data, uint8_t length)
{
  Slot *s = &slots[active_owner];
  s->event.result = result;
  s->event.token = active.token;
  s->event.tick_ms = HAL_GetTick();
  s->event.length = length;
  s->event.tx_completed = active_tx_completed;
  if (length)
    memcpy(s->event.data, data, length);
  s->event_ready = true;
  running = false;
  waiting_reply = false;
}
HAL_StatusTypeDef MotorBus_Init(UART_HandleTypeDef *uart)
{
  if (!uart)
    return HAL_ERROR;
  if (bus_uart)
    return bus_uart == uart ? HAL_OK : HAL_ERROR;
  bus_uart = uart;
  memset(slots, 0, sizeof(slots));
  {
    HAL_StatusTypeDef result = ReceiveStart();
    if (result != HAL_OK)
      bus_uart = NULL;
    return result;
  }
}
bool MotorBus_Submit(MotorBus_Owner_t owner, const uint8_t *data, uint8_t length,
                     uint8_t reply_length, uint32_t token, bool priority)
{
  Slot *s;
  if (!bus_uart || owner >= MOTOR_BUS_OWNER_COUNT || !data || length < 3 ||
      length > MOTOR_BUS_FRAME_MAX || reply_length > MOTOR_BUS_FRAME_MAX ||
      (recovery.active && owner != MOTOR_BUS_RECOVERY) ||
      (quarantined && (!priority || reply_length) && !RecoveryRequest(owner) &&
       !(diagnostic_reads && FeedbackRead(owner, data, length, reply_length))))
    return false;
  s = &slots[owner];
  if (s->queued || s->event_ready || (running && active_owner == owner))
    return false;
  memcpy(s->data, data, length);
  s->length = length;
  s->reply_length = reply_length;
  s->token = token;
  s->priority = priority;
  s->queued = true;
  /* Only emergency Stop may abandon another owner's ordinary transaction.
   * DMA is never aborted here: Cancel waits for TX complete if still on wire.
   * Its abandoned response is quarantined because the protocol has no sequence. */
  if (priority && length == 5U && data[1] == 0xfe && running && !active.priority)
    MotorBus_Cancel(active_owner);
  return true;
}
bool MotorBus_EventGet(MotorBus_Owner_t owner, MotorBus_Event_t *event)
{
  if (owner >= MOTOR_BUS_OWNER_COUNT || !event || !slots[owner].event_ready)
    return false;
  *event = slots[owner].event;
  slots[owner].event_ready = false;
  return true;
}
bool MotorBus_OwnerBusy(MotorBus_Owner_t owner)
{
  return owner >= MOTOR_BUS_OWNER_COUNT || slots[owner].queued || slots[owner].event_ready ||
         (running && owner == active_owner);
}
bool MotorBus_Reserve(MotorBus_Owner_t owner)
{
  if (!bus_uart || owner >= MOTOR_BUS_OWNER_COUNT ||
      (reserved_owner != MOTOR_BUS_OWNER_COUNT && reserved_owner != owner))
    return false;
  reserved_owner = owner;
  return true;
}
void MotorBus_Release(MotorBus_Owner_t owner)
{
  if (reserved_owner == owner)
    reserved_owner = MOTOR_BUS_OWNER_COUNT;
}
void MotorBus_Cancel(MotorBus_Owner_t owner)
{
  Slot *s;
  if (owner >= MOTOR_BUS_OWNER_COUNT)
    return;
  s = &slots[owner];
  if (s->queued)
  {
    s->queued = false;
    s->event_ready = true;
    s->event.result = MOTOR_BUS_CANCELLED;
    s->event.token = s->token;
    s->event.tick_ms = HAL_GetTick();
    s->event.length = 0;
    s->event.tx_completed = false;
  }
  if (running && owner == active_owner)
  {
    cancelled = true;
    if (active.reply_length)
    {
      if ((owner == MOTOR_BUS_FEEDBACK || owner == MOTOR_BUS_ARM || owner == MOTOR_BUS_GUARD ||
           owner == MOTOR_BUS_TURNTABLE) &&
          active.length == 3U && (active.data[1] == 0x35 || active.data[1] == 0x36 ||
                                 active.data[1] == 0x3a || active.data[1] == 0x3b))
      {
        if (!drain_pending)
        {
          drain_pending = true;
          drain_owner = owner;
          drain_address = active.data[0];
          drain_function = active.data[1];
          drain_length = active.reply_length;
          drain_started_ms = waiting_reply ? started_ms : HAL_GetTick();
        }
      }
      else
      {
        /* Only a cancelled chassis motion ACK differs from every read function.
         * Keep the motion quarantine; allow fresh read-only diagnostics. */
        if (!quarantined && owner == MOTOR_BUS_CHASSIS &&
            (active.data[1] == 0xf6 || active.data[1] == 0xfd))
          diagnostic_reads = true;
        else if (owner != MOTOR_BUS_CHASSIS)
          diagnostic_reads = false;
        Quarantine("cancelled_reply", HAL_GetTick(), 0U);
      }
    }
    if (waiting_reply)
      Complete(MOTOR_BUS_CANCELLED, NULL, 0);
  }
}
void MotorBus_RxBytes(const uint8_t *data, uint16_t length)
{
  uint16_t i, next;
  if (length) last_rx_ms = HAL_GetTick();
  for (i = 0; i < length; i++)
  {
    next = (uint16_t)((rx_write + 1U) & 255U);
    if (next == rx_read)
    {
      if (!io_error)
      {
        io_overflow = true;
        io_uart_error = 0U;
        io_tick_ms = HAL_GetTick();
        io_error = true;
      }
      break;
    }
    rx_ring[rx_write] = data[i];
    rx_time[rx_write] = HAL_GetTick();
    rx_write = next;
  }
}
static uint8_t ReplyLength(uint8_t function)
{
  if (function == 0x35)
    return 6;
  if (function == 0x36)
    return 8;
  if (function == 0x3a)
    return 4;
  if (running && function == active.data[1])
    return active.reply_length;
  return 4;
}
static void ParseByte(uint8_t value, uint32_t arrival_ms)
{
  uint8_t length;
  if (parse_length == sizeof(parse))
  {
    memmove(parse, parse + 1, --parse_length);
  }
  parse[parse_length++] = value;
  while (parse_length >= 2)
  {
    if (parse[0] == 0 || parse[0] > 8)
    {
      memmove(parse, parse + 1, --parse_length);
      continue;
    }
    length = ReplyLength(parse[1]);
    if (length < 4)
    {
      memmove(parse, parse + 1, --parse_length);
      continue;
    }
    if (parse_length < length)
      return;
    if (parse[length - 1] != 0x6b)
    {
      memmove(parse, parse + 1, --parse_length);
      continue;
    }
    if (drain_pending && parse[0] == drain_address && parse[1] == drain_function &&
        length == drain_length)
      drain_pending = false;
    else if (running && waiting_reply &&
        (!quarantined || RecoveryRequest(active_owner) || (diagnostic_reads &&
         FeedbackRead(active_owner, active.data, active.length, active.reply_length))) &&
        parse[0] == active.data[0] &&
        parse[1] == active.data[1] && length == active.reply_length)
    {
      Complete(MOTOR_BUS_REPLY, parse, length);
      slots[active_owner].event.tick_ms = arrival_ms;
    }
    parse_length = (uint8_t)(parse_length - length);
    memmove(parse, parse + length, parse_length);
  }
}
void MotorBus_Process(void)
{
  unsigned i, pass;
  uint32_t now = HAL_GetTick();
  HAL_StatusTypeDef result;
  if (!bus_uart)
    return;
  if (io_error)
  {
    if (recovery.active) RecoveryFail("uart_error");
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Quarantine(io_overflow ? "rx_overflow" : "uart_error", io_tick_ms, io_uart_error);
    io_error = false;
    __set_PRIMASK(primask);
    diagnostic_reads = false;
    (void)HAL_UART_DMAStop(bus_uart);
    (void)ReceiveStart();
    if (running)
      Complete(MOTOR_BUS_IO_ERROR, NULL, 0);
    gap_ms = now;
    gap_valid = true;
    tx_done = false;
  }
  if (running && tx_done)
  {
    tx_done = false;
    gap_ms = tx_done_ms;
    gap_valid = true;
    active_tx_completed = true;
    if (cancelled && active_owner == drain_owner && drain_pending)
      drain_started_ms = tx_done_ms;
    if (cancelled)
      Complete(MOTOR_BUS_CANCELLED, NULL, 0);
    else if (!active.reply_length)
      Complete(MOTOR_BUS_TX_DONE, NULL, 0);
    else
    {
      waiting_reply = true;
      started_ms = tx_done_ms;
    }
    if (!running)
      slots[active_owner].event.tick_ms = tx_done_ms;
  }
  /* Expire before parsing: data processed beyond the deadline cannot rescue it. */
  if (drain_pending && (uint32_t)(now - drain_started_ms) >= MOTOR_BUS_TIMEOUT_MS)
  {
    drain_pending = false;
    diagnostic_reads = false;
    if (!quarantined)
    {
      Quarantine("drain_timeout", now, 0U);
      first_fault.owner = drain_owner;
      first_fault.address = drain_address;
      first_fault.function = drain_function;
    }
  }
  if (running && (uint32_t)(now - started_ms) >= MOTOR_BUS_TIMEOUT_MS)
  {
    diagnostic_reads = false;
    Quarantine(waiting_reply ? "reply_timeout" : "tx_timeout", now, 0U);
    if (!waiting_reply)
    {
      (void)HAL_UART_DMAStop(bus_uart);
      (void)ReceiveStart();
      gap_ms = now;
      gap_valid = true;
    }
    Complete(MOTOR_BUS_TIMEOUT, NULL, 0);
  }
  /* A fast driver may reply before foreground observes TX complete. Keep those bytes. */
  if (!running || waiting_reply)
    while (rx_read != rx_write)
    {
      uint8_t b = rx_ring[rx_read];
      uint32_t arrival_ms = rx_time[rx_read];
      rx_read = (uint16_t)((rx_read + 1U) & 255U);
      ParseByte(b, arrival_ms);
    }
  RecoveryProcess();
  if (running || (gap_valid && (uint32_t)(now - gap_ms) < MOTOR_BUS_GAP_MS))
    return;
  for (pass = 0; pass < 2; pass++)
    for (i = 0; i < MOTOR_BUS_OWNER_COUNT; i++)
    {
      Slot *s = &slots[i];
      if (!s->queued || s->priority != (pass == 0))
        continue;
      if (drain_pending && !(s->priority && s->length == 5U &&
                             s->data[1] == 0xfe && s->reply_length == 0U))
        continue;
      if (!s->priority && reserved_owner != MOTOR_BUS_OWNER_COUNT && i != (unsigned)reserved_owner)
        continue;
      active = *s;
      s->queued = false;
      active_owner = (MotorBus_Owner_t)i;
      running = true;
      waiting_reply = false;
      cancelled = false;
      tx_done = false;
      started_ms = now;
      active_tx_completed = false;
      /* Discard pre-command bytes; only replies after this request can match. */
      if (!drain_pending)
      {
        parse_length = 0;
        rx_read = rx_write;
      }
      if (quarantined && (!active.priority || active.reply_length) && !RecoveryRequest(active_owner) &&
          !(diagnostic_reads && FeedbackRead(active_owner, active.data,
                                            active.length, active.reply_length)))
      {
        Complete(MOTOR_BUS_CANCELLED, NULL, 0);
        return;
      }
      result = HAL_UART_Transmit_DMA(bus_uart, active.data, active.length);
      if (result != HAL_OK)
      {
        diagnostic_reads = false;
        gap_ms = now;
        gap_valid = true;
        Complete(MOTOR_BUS_IO_ERROR, NULL, 0);
      }
      return;
    }
}
void MotorBus_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  if (uart != bus_uart || size == 0 || size > sizeof(dma_rx))
    return;
  /* Circular DMA remains active after IDLE/HT/TC. Restarting it here returns
   * HAL_BUSY and used to quarantine the bus after the first motor response. */
  if (size > dma_rx_position)
  {
    MotorBus_RxBytes(dma_rx + dma_rx_position, size - dma_rx_position);
  }
  else if (size < dma_rx_position)
  {
    MotorBus_RxBytes(dma_rx + dma_rx_position, sizeof(dma_rx) - dma_rx_position);
    MotorBus_RxBytes(dma_rx, size);
  }
  /* Keep size==64 as 64: a following IDLE at the same TC position is a no-op. */
  dma_rx_position = size;
}
void MotorBus_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == bus_uart && running)
  {
    tx_done_ms = HAL_GetTick();
    tx_done = true;
  }
}
void MotorBus_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart && uart == bus_uart && !io_error)
  {
    io_overflow = false;
    io_uart_error = uart->ErrorCode; /* ReceiveStart clears this HAL field. */
    io_tick_ms = HAL_GetTick();
    io_error = true;
  }
}
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *out)
{
  if (!out)
    return;
  *out = first_fault;
  out->locked = quarantined;
  if (!out->reason)
    out->reason = "none";
}
bool MotorBus_IsQuarantined(void)
{
  return quarantined;
}
bool MotorBus_FeedbackAllowed(void)
{
  return !quarantined || diagnostic_reads;
}
bool MotorBus_RecoverAfterReset(void)
{
  unsigned i;
  if (running || recovery.active)
    return false;
  for (i = 0; i < MOTOR_BUS_OWNER_COUNT; i++)
    if (slots[i].queued || slots[i].event_ready)
      return false;
  rx_read = rx_write;
  parse_length = 0;
  drain_pending = false;
  diagnostic_reads = false;
  quarantined = false;
  memset(&first_fault, 0, sizeof(first_fault));
  return true;
}

static void RecoveryFail(const char *reason)
{
  recovery.active = false;
  recovery.success = false;
  recovery.reason = reason;
  quarantined = true;
  diagnostic_reads = false;
  reserved_owner = MOTOR_BUS_OWNER_COUNT;
}
void MotorBus_RecoveryCancel(void)
{
  if (!recovery.active) return;
  MotorBus_Cancel(MOTOR_BUS_RECOVERY);
  RecoveryFail("cancelled");
}
void MotorBus_RecoveryGet(MotorBus_Recovery_t *out)
{
  if (out) *out = recovery;
}
bool MotorBus_RecoveryStart(void)
{
  unsigned i;
  if (!bus_uart || running || recovery.active || io_error ||
      reserved_owner != MOTOR_BUS_OWNER_COUNT) return false;
  for (i=0;i<MOTOR_BUS_OWNER_COUNT;i++)
    if (slots[i].queued || slots[i].event_ready) return false;
  recovery.active=true;recovery.success=false;recovery.address=1;
  recovery.checked_mask=0;recovery.reason="draining";
  recovery_step=0;recovery_round=0;recovery_kind=0;
  recovery_started=recovery_wait=HAL_GetTick();
  quarantined=true;diagnostic_reads=false;drain_pending=false;
  reserved_owner=MOTOR_BUS_RECOVERY;
  return true;
}
static void RecoveryProcess(void)
{
  MotorBus_Event_t e;
  uint8_t cmd[5], length, reply;
  static const uint8_t functions[3]={0x35,0x3a,0x3b};
  uint32_t now=HAL_GetTick();
  if (!recovery.active) {
    /* Cancellation/IO failure still lets the in-flight DMA complete. */
    (void)MotorBus_EventGet(MOTOR_BUS_RECOVERY,&e);
    return;
  }
  if ((uint32_t)(now-recovery_started)>8000U) { RecoveryFail("recovery_timeout");return; }
  if (recovery_step==0 || recovery_step==3) {
    if ((uint32_t)(now-recovery_wait)<500U || (uint32_t)(now-last_rx_ms)<500U) return;
    if (recovery_step==0) {
      if (HAL_UART_DMAStop(bus_uart)!=HAL_OK || ReceiveStart()!=HAL_OK) {
        RecoveryFail("rx_restart_failed");return;
      }
      rx_read=rx_write;parse_length=0;tx_done=false;
      recovery_step=1;recovery.reason="stopping";
    } else { recovery_step=4;recovery.reason="checking"; }
  }
  if (MotorBus_EventGet(MOTOR_BUS_RECOVERY,&e)) {
    if ((recovery_step<4 && e.result!=MOTOR_BUS_TX_DONE) ||
        (recovery_step==4 && e.result!=MOTOR_BUS_REPLY)) {
      RecoveryFail(e.result==MOTOR_BUS_TIMEOUT?"reply_timeout":"transport_error");return;
    }
    if (recovery_step==4) {
      if (recovery_kind==0 && (((unsigned)e.data[3]<<8)|e.data[4])>1U) {
        RecoveryFail("motor_moving");return;
      }
      if (recovery_kind==1 && (e.data[2]&0x0cU)) { RecoveryFail("motor_fault");return; }
      if (recovery_kind==2 && ((e.data[2]&0x34U) || (e.data[2]&3U)!=3U)) {
        RecoveryFail("homing_or_driver_fault");return;
      }
      if (++recovery_kind<3) goto submit;
      recovery_kind=0;
      recovery.checked_mask|=(uint8_t)(1U<<(recovery.address-1U));
    }
    if (++recovery.address>8) {
      recovery.address=1;
      if (recovery_step==1) { recovery_step=2;recovery.reason="aborting_home"; }
      else if (recovery_step==2) {
        recovery_step=3;recovery_wait=now;recovery.reason="draining";return;
      } else if (++recovery_round>=3) {
        recovery.active=false;recovery.success=true;recovery.reason="complete";
        quarantined=false;reserved_owner=MOTOR_BUS_OWNER_COUNT;
        /* Preserve first_fault for diagnosis; locked reports current state. */
        return;
      }
    }
  }
submit:
  if (MotorBus_OwnerBusy(MOTOR_BUS_RECOVERY)) return;
  cmd[0]=recovery.address;
  if (recovery_step==1) { cmd[1]=0xfe;cmd[2]=0x98;cmd[3]=0;cmd[4]=0x6b;length=5;reply=0; }
  else if (recovery_step==2) { cmd[1]=0x9c;cmd[2]=0x48;cmd[3]=0x6b;length=4;reply=0; }
  else { cmd[1]=functions[recovery_kind];cmd[2]=0x6b;length=3;reply=recovery_kind==0?6:4; }
  if (!MotorBus_Submit(MOTOR_BUS_RECOVERY,cmd,length,reply,0,true)) RecoveryFail("submit_failed");
}
