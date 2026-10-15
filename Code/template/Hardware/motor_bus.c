#include "motor_bus.h"
#include <string.h>

typedef struct
{
  uint8_t data[MOTOR_BUS_FRAME_MAX], length, reply_length, reply_address;
  uint32_t token, queued_ms;
  bool queued, event_ready, priority;
  MotorBus_Event_t event;
} Slot;
static UART_HandleTypeDef *bus_uart;
static Slot slots[MOTOR_BUS_OWNER_COUNT], active;
static MotorBus_Owner_t active_owner;
static MotorBus_Owner_t reserved_owner = MOTOR_BUS_OWNER_COUNT;
static bool running, waiting_reply, cancelled, quarantined;
static bool active_tx_completed;
/* Startup FE may precede the first request while shared TTL nodes power up.
 * Never reopen this exception after a TX attempt, including explicit recovery. */
static bool tx_attempted;
static uint32_t startup_fe_recovered, startup_fe_last_ms;
static uint32_t rx_warning_count, rx_warning_ms, rx_warning_error;
static uint32_t started_ms;
static volatile bool tx_done, io_error;
static volatile bool io_overflow;
static volatile uint32_t io_uart_error, io_tick_ms;
static MotorBus_Diagnostic_t first_fault;
static volatile uint32_t tx_done_ms;
static volatile bool wire_tx_completed;
static volatile uint32_t wire_tx_ms;
static uint8_t dma_rx[64], rx_ring[256], parse[MOTOR_BUS_FRAME_MAX];
static uint32_t rx_time[256];
static volatile uint16_t rx_write, rx_read;
static volatile uint32_t rx_total;
static volatile uint8_t rx_recent[16];
static uint32_t request_rx_start, process_last_ms, process_gap_ms;
static uint16_t process_rx_pending;
static bool process_seen;
/* HAL circular RX callback Size is a buffer position (1..64), not a new length. */
static uint16_t dma_rx_position;
static uint8_t parse_length;
static unsigned next_owner;
static bool chassis_yield;
static uint32_t chassis_yield_mask;
static MotorBus_Timing_t timing;
static uint8_t position_reply_mode[8], position_config_id, position_config_phase;
static bool position_config_pending;
static uint8_t position_config[33];
static struct { bool active, reached; uint32_t tx_ms; } position_motion[8];
static void PositionConfigProcess(void);

int MotorBus_PositionReplyEnsure(uint8_t address)
{
  if(address<5 || address>8)return -1;
  if(position_reply_mode[address-1]==2)return 1;
  if(position_reply_mode[address-1]==3)return -1;
  position_reply_mode[address-1]=1;
  return 0;
}
bool MotorBus_PositionReached(uint8_t address,uint32_t accepted_ms)
{
  return address>=5 && address<=8 && position_motion[address-1].active &&
    position_motion[address-1].reached &&
    (int32_t)(position_motion[address-1].tx_ms-accepted_ms)>=0;
}
static void PositionConfigProcess(void)
{
  MotorBus_Event_t event;
  if(position_config_pending && MotorBus_EventGet(MOTOR_BUS_POSITION_CONFIG,&event)) {
    position_config_pending=false;
    bool valid=event.result==MOTOR_BUS_REPLY &&
      (position_config_phase==1 ? event.length==4 && event.data[2]==2 :
       event.length==33 && event.data[2]==33 && event.data[3]==21);
    if(!valid) { position_reply_mode[position_config_id-1]=event.result==MOTOR_BUS_CANCELLED?0:3;position_config_id=0;return; }
    if(position_config_phase==0) {
      memcpy(position_config,event.data,33);
      if(position_config[22]==3) {position_reply_mode[position_config_id-1]=2;position_config_id=0;return;}
      position_config[1]=0x48;position_config[2]=0xd1;position_config[3]=0;
      position_config[22]=3;position_config_phase=1;
    } else if(position_config_phase==1)position_config_phase=2;
    else {
      position_reply_mode[position_config_id-1]=memcmp(position_config+4,event.data+4,28)?3:2;
      position_config_id=0;return;
    }
  }
  if(position_config_pending || quarantined || MotorBus_ProbeActive())return;
  if(!position_config_id)for(unsigned i=4;i<8;i++)if(position_reply_mode[i]==1) {
    position_config_id=(uint8_t)(i+1);position_config_phase=0;break;
  }
  if(!position_config_id)return;
  const uint8_t read[]={position_config_id,0x42,0x6c,0x6b};
  bool write=position_config_phase==1;
  if(MotorBus_Submit(MOTOR_BUS_POSITION_CONFIG,write?position_config:read,
       write?33:4,write?4:33,0,false))position_config_pending=true;
}
#ifndef MOTOR_BUS_REPLY_SETTLE_MS
#define MOTOR_BUS_REPLY_SETTLE_MS 2U
#endif
/* Same-image car-place A/B: 0 ms reproduced UART5 FE, 2 ms completed.
 * Retain Receive ACK, then allow the shared TTL line to hand over. */
static uint32_t reply_settle_ms = MOTOR_BUS_REPLY_SETTLE_MS;
static uint32_t reply_complete_ms;
static bool reply_complete_valid;
/* Per-address FE/9C replies from urgent chains cannot confirm a later request. */
static uint8_t urgent_drain[8][2];
static uint32_t urgent_started[8][2];
static bool Urgent(const uint8_t *data, uint8_t length, bool priority)
{ return priority && ((length==5U && data[1]==0xfe) || (length==4U && data[1]==0x9c)); }
static bool UrgentPending(void)
{ for(unsigned i=0;i<8;i++)for(unsigned k=0;k<2;k++)if(urgent_drain[i][k])return true;return false; }
static uint8_t ExpectedReplyLength(const uint8_t *data,uint8_t length)
{
  if(length==3U) {
    switch(data[1]) {
      case 0x35:return 6; case 0x36:return 8; case 0x3a:case 0x3b:return 4;
      default:return 0;
    }
  }
  switch(data[1]) {
    case 0x06:case 0x0a:case 0x0e:case 0x0f:case 0xf3:case 0xf6:
    case 0xfd:case 0xfe:case 0xff:case 0x93:case 0x9a:case 0x9c:return 4;
    default:return 0;
  }
}
void MotorBus_TimingGet(MotorBus_Timing_t *out) { if(out)*out=timing; }

/* One abandoned read reply must be consumed before another ordinary request.
 * Stop frames may still transmit; no drive motion command gets this exception. */
static bool drain_pending;
static bool diagnostic_reads;
static MotorBus_Owner_t drain_owner;
static uint8_t drain_address, drain_function, drain_length;
static uint32_t drain_started_ms;
static MotorBus_Recovery_t recovery = {.reason="idle"};
static uint8_t recovery_step, recovery_round, recovery_kind;
static uint32_t recovery_started, recovery_wait;
static bool recovery_motion_wait;
static uint32_t recovery_rx_start;
static volatile uint32_t last_rx_ms;
static bool probe_active, probe_ready;
static uint8_t probe_address;
static uint32_t probe_since;
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
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    uint32_t total=rx_total;
    first_fault.rx_bytes=total-request_rx_start;
    first_fault.last_rx_ms=last_rx_ms;
    first_fault.rx_sample_length=(uint8_t)(first_fault.rx_bytes<16U?first_fault.rx_bytes:16U);
    for(unsigned j=0;j<first_fault.rx_sample_length;j++)
      first_fault.rx_sample[j]=rx_recent[(total-first_fault.rx_sample_length+j)&15U];
    __set_PRIMASK(mask);
    first_fault.rx_pending=process_rx_pending;
    first_fault.parse_bytes=parse_length;
    first_fault.process_gap_ms=process_gap_ms;
  }
  quarantined = true;
  recovery.success = false;
}

static bool FeedbackRead(MotorBus_Owner_t owner, const uint8_t *data,
                         uint8_t length, uint8_t reply_length)
{
  if(!data || length!=3U || data[2]!=0x6b)return false;
  bool address_valid=(owner==MOTOR_BUS_FEEDBACK && data[0]>=1U && data[0]<=4U) ||
    (owner==MOTOR_BUS_ARM && data[0]>=5U && data[0]<=7U) ||
    (owner==MOTOR_BUS_TURNTABLE && data[0]==8U) ||
    ((owner==MOTOR_BUS_GUARD || owner==MOTOR_BUS_PROBE) && data[0]>=1U && data[0]<=8U);
  return address_valid && ((data[1]==0x35 && reply_length==6U) ||
    (data[1]==0x36 && reply_length==8U) ||
    ((data[1]==0x3a || data[1]==0x3b) && reply_length==4U));
}

static bool ProbeRead(MotorBus_Owner_t owner, const uint8_t *data,
                      uint8_t length, uint8_t reply_length)
{
  if(!probe_active || !probe_ready || owner!=MOTOR_BUS_PROBE || !data ||
     data[0]!=probe_address)return false;
  if(FeedbackRead(owner,data,length,reply_length))return true;
  if(length==4U && data[1]==0x42 && data[2]==0x6c && data[3]==0x6b)
    return reply_length==33U; /* Current car uses Emm configuration layout. */
  return length==3U && data[2]==0x6b &&
    (((data[1]==0x24 || data[1]==0x27) && reply_length==5U) ||
     (data[1]==0x37 && reply_length==8U) || (data[1]==0x22 && reply_length==18U));
}
bool MotorBus_ProbeStart(uint8_t address)
{
  if (!bus_uart || address<1U || address>8U || probe_active || recovery.active ||
      reserved_owner!=MOTOR_BUS_OWNER_COUNT || MotorBus_OwnerBusy(MOTOR_BUS_PROBE)) return false;
  /* Drain an already active read; never reserve around an active motion/stop. */
  if (running && !FeedbackRead(active_owner,active.data,active.length,active.reply_length)) return false;
  for(unsigned i=0;i<MOTOR_BUS_OWNER_COUNT;i++)
    if(slots[i].queued && !FeedbackRead((MotorBus_Owner_t)i,slots[i].data,
                                      slots[i].length,slots[i].reply_length)) return false;
  reserved_owner=MOTOR_BUS_PROBE;
  probe_address=address;probe_active=true;probe_ready=false;probe_since=HAL_GetTick();
  return true;
}
bool MotorBus_ProbeReady(void) { return probe_active && probe_ready; }
bool MotorBus_ProbeActive(void) { return probe_active; }
void MotorBus_ProbeStop(void)
{
  MotorBus_Cancel(MOTOR_BUS_PROBE);
  MotorBus_Release(MOTOR_BUS_PROBE);
  probe_active=probe_ready=false;
}

static HAL_StatusTypeDef ReceiveStart(void)
{
  /* Only initialization/error recovery starts DMA. Keep HT enabled so a stream
   * without IDLE is copied before the DMA buffer wraps and overwrites it. */
  uint32_t mask=__get_PRIMASK();__disable_irq();
  dma_rx_position = sizeof(dma_rx); /* DMA position zero is the same boundary. */
  HAL_StatusTypeDef result=HAL_UARTEx_ReceiveToIdle_DMA(bus_uart, dma_rx, sizeof(dma_rx));
  __set_PRIMASK(mask);return result;
}
static void Complete(MotorBus_Result_t result, const uint8_t *data, uint8_t length)
{
  Slot *s = &slots[active_owner];
  if(active.reply_address>=1 && active.reply_address<=8) {
    MotorBus_IdTiming_t *t=&timing.id[active.reply_address-1U];
    if(result==MOTOR_BUS_TIMEOUT)t->timeouts++;
  }
  s->event.result = result;
  s->event.token = active.token;
  s->event.tick_ms = HAL_GetTick();
  s->event.length = length;
  s->event.tx_completed = active_tx_completed;
  s->event.tx_ms = active_tx_completed ? tx_done_ms : 0U;
  s->event.uart_error = 0U;
  if (length)
    memcpy(s->event.data, data, length);
  s->event_ready = true;
  running = false;
  waiting_reply = false;
  if (active_owner==MOTOR_BUS_PROBE && result!=MOTOR_BUS_REPLY) {
    probe_ready=false;probe_since=HAL_GetTick();
  }
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
  uint8_t address=data && length?data[0]:0;
  return MotorBus_SubmitReplyTo(owner,data,length,address,reply_length,token,priority);
}
bool MotorBus_SubmitReplyTo(MotorBus_Owner_t owner,const uint8_t *data,uint8_t length,
                           uint8_t reply_address,uint8_t reply_length,uint32_t token,bool priority)
{
  Slot *s;
  if(data && length>=3U && data[0]>=1U && data[0]<=8U && !reply_length && !Urgent(data,length,priority))
    reply_length=ExpectedReplyLength(data,length);
  if (!bus_uart || owner >= MOTOR_BUS_OWNER_COUNT || !data || length < 3 ||
      length > MOTOR_BUS_FRAME_MAX || reply_length > MOTOR_BUS_FRAME_MAX ||
      (recovery.active && owner != MOTOR_BUS_RECOVERY) ||
      (owner==MOTOR_BUS_PROBE && (priority || reply_address!=probe_address ||
                                 !ProbeRead(owner,data,length,reply_length))) ||
      (probe_active && !Urgent(data,length,priority) &&
       !ProbeRead(owner,data,length,reply_length)) ||
      (quarantined && (!priority || reply_length) && !RecoveryRequest(owner) &&
       !ProbeRead(owner,data,length,reply_length) &&
       !(diagnostic_reads && FeedbackRead(owner, data, length, reply_length))))
    return false;
  s = &slots[owner];
  if (s->queued || s->event_ready || (running && active_owner == owner))
    return false;
  memcpy(s->data, data, length);
  s->length = length;
  s->reply_length = reply_length;
  if(data[0]>=1 && data[0]<=8 && data[1]==0xfd)
    position_motion[data[0]-1U].active=position_motion[data[0]-1U].reached=false;
  s->reply_address=reply_address; s->queued_ms=HAL_GetTick();
  if(reply_address>=1 && reply_address<=8)timing.id[reply_address-1].requests++;
  s->token = token;
  s->priority = priority;
  s->queued = true;
  /* Proof applies to the stopped caches just checked, not to any later batch. */
  if (owner == MOTOR_BUS_CHASSIS && (data[1] == 0xf6 || data[1] == 0xfd))
    recovery.success = false;
  /* Only emergency Stop may abandon another owner's ordinary transaction.
   * DMA is never aborted here: Cancel waits for TX complete if still on wire.
   * Its abandoned response is quarantined because the protocol has no sequence. */
  if (Urgent(data,length,priority) && running && !active.priority)
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
bool MotorBus_OwnerInFlight(MotorBus_Owner_t owner)
{ return running && active_owner==owner; }
bool MotorBus_Reserve(MotorBus_Owner_t owner)
{
  if (!bus_uart || owner >= MOTOR_BUS_OWNER_COUNT ||
      (reserved_owner != MOTOR_BUS_OWNER_COUNT && reserved_owner != owner))
    return false;
  if(owner==MOTOR_BUS_CHASSIS && chassis_yield) {
    for(unsigned i=1;i<MOTOR_BUS_OWNER_COUNT;i++)
      if((chassis_yield_mask&(1UL<<i)) && !MotorBus_OwnerBusy((MotorBus_Owner_t)i))
        chassis_yield_mask&=~(1UL<<i);
    if(chassis_yield_mask)return false;
    chassis_yield=false;
  }
  reserved_owner = owner;
  return true;
}
void MotorBus_Release(MotorBus_Owner_t owner)
{
  if (reserved_owner == owner)
  { reserved_owner = MOTOR_BUS_OWNER_COUNT;
    if(owner==MOTOR_BUS_CHASSIS) {
      chassis_yield=true;chassis_yield_mask=0;
      for(unsigned i=1;i<MOTOR_BUS_OWNER_COUNT;i++)
        if(i!=(unsigned)owner && slots[i].queued && !slots[i].priority)
          chassis_yield_mask|=1UL<<i;
    }
  }
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
    s->event.uart_error = 0U;
  }
  if (running && owner == active_owner)
  {
    cancelled = true;
    if (active.reply_length)
    {
      if (((owner == MOTOR_BUS_FEEDBACK || owner == MOTOR_BUS_ARM || owner == MOTOR_BUS_GUARD ||
           owner == MOTOR_BUS_TURNTABLE || owner == MOTOR_BUS_PROBE) &&
          active.length == 3U && (active.data[1] == 0x35 || active.data[1] == 0x36 ||
                                 active.data[1] == 0x3a || active.data[1] == 0x3b)) ||
          (owner == MOTOR_BUS_CHASSIS && active.length == 8U && active.data[1] == 0xf6 &&
           active.data[3] == 0U && active.data[4] == 0U))
      {
        /* A zero-RPM F6 cannot leave latent motion. Drain its unsequenced ACK
         * before later requests, while priority FE stops remain immediately
         * eligible. Nonzero/position cancellation retains quarantine below. */
        if (!drain_pending)
        {
          drain_pending = true;
          drain_owner = owner;
          drain_address = active.reply_address;
          drain_function = active.data[1];
          drain_length = active.reply_length;
          drain_started_ms = waiting_reply ? started_ms : HAL_GetTick();
        }
      }
      else
      {
        /* Chassis and arm motion/stop ACKs cannot match 35/36/3A/3B reads.
         * Preserve motion quarantine while permitting fresh stop evidence.
         * A prior UART/timeout fault must not gain this cancellation exception. */
        bool distinct_ack=(owner==MOTOR_BUS_CHASSIS && (active.data[1]==0xf6 || active.data[1]==0xfd)) ||
          ((owner==MOTOR_BUS_ARM || owner==MOTOR_BUS_TURNTABLE) &&
           (active.data[1]==0xfd || active.data[1]==0xfe));
        if(distinct_ack && (!quarantined || diagnostic_reads))diagnostic_reads=true;
        else if(owner!=MOTOR_BUS_CHASSIS)diagnostic_reads=false;
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
    rx_recent[rx_total&15U]=data[i];rx_total++;
    rx_ring[rx_write] = data[i];
    rx_time[rx_write] = HAL_GetTick();
    rx_write = next;
  }
}
static uint8_t ReplyLength(uint8_t function)
{
  if(function==0x24 || function==0x27)return 5;
  if(function==0x37)return 8;
  if(function==0x22)return 18;
  if(function==0x42)return 33;
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
    /* Query signs/flags cannot be E2/EE: these are short rejection frames. */
    if(parse_length>=4 && (parse[2]==0xe2 || parse[2]==0xee) && parse[3]==0x6b)length=4;
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
    if(length==4 && parse[1]==0xfd && parse[2]==0x9f) {
      unsigned id=parse[0]-1U;
      if(position_motion[id].active && (int32_t)(arrival_ms-position_motion[id].tx_ms)>=0) {
        position_motion[id].reached=true;
        reply_complete_ms=arrival_ms;reply_complete_valid=true;
        /* Completion is stronger evidence than a missing Receive ACK, but
         * only the same on-wire FD can consume it; unrelated queries cannot. */
        if(running && waiting_reply && !quarantined && active.data[1]==0xfd &&
           active.reply_address==parse[0] && (int32_t)(arrival_ms-started_ms)>=0 &&
           (uint32_t)(arrival_ms-started_ms)<MOTOR_BUS_TIMEOUT_MS) {
          Complete(MOTOR_BUS_REPLY,parse,4);slots[active_owner].event.tick_ms=arrival_ms;
        }
      }
    }
    else if((parse[1]==0xfe || parse[1]==0x9c) && length==4 &&
       urgent_drain[parse[0]-1U][parse[1]==0x9c?1:0]) {
      urgent_drain[parse[0]-1U][parse[1]==0x9c?1:0]--;timing.drained++;
      MotorBus_IdTiming_t *t=&timing.id[parse[0]-1U];t->reply_ms=arrival_ms;
      if(parse[2]==0xe2 || parse[2]==0xee)t->rejected++;else if(parse[2]==2)t->success++;
    }
    else if (drain_pending && (int32_t)(arrival_ms-drain_started_ms)<(int32_t)MOTOR_BUS_TIMEOUT_MS &&
        parse[0] == drain_address && parse[1] == drain_function &&
        (length == drain_length || (length==4 && (parse[2]==0xe2 || parse[2]==0xee))))
    { drain_pending = false; timing.drained++; }
    else if (running && waiting_reply &&
        (int32_t)(arrival_ms-started_ms)>=0 &&
        (int32_t)(arrival_ms-started_ms)<(int32_t)MOTOR_BUS_TIMEOUT_MS &&
        (!quarantined || RecoveryRequest(active_owner) ||
         ProbeRead(active_owner,active.data,active.length,active.reply_length) || (diagnostic_reads &&
         FeedbackRead(active_owner, active.data, active.length, active.reply_length))) &&
        parse[0] == active.reply_address &&
        parse[1] == active.data[1] && (length == active.reply_length ||
          (length==4 && (parse[2]==0xe2 || parse[2]==0xee))) &&
        !(active.length>3 && parse[2]==0x9f))
    {
      reply_complete_ms=arrival_ms;reply_complete_valid=true;
      Complete(MOTOR_BUS_REPLY, parse, length);
      slots[active_owner].event.tick_ms = arrival_ms;
      if(active.reply_address>=1 && active.reply_address<=8) {
        MotorBus_IdTiming_t *t=&timing.id[active.reply_address-1];
        t->reply_ms=arrival_ms;
        t->reply_last_ms=(int32_t)(arrival_ms-tx_done_ms)>0?arrival_ms-tx_done_ms:0;
        if(t->reply_last_ms>t->reply_max_ms)t->reply_max_ms=t->reply_last_ms;
        if(length==4 && (parse[2]==0xe2 || parse[2]==0xee))t->rejected++;else t->success++;
      }
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
  process_gap_ms=process_seen?now-process_last_ms:0U;
  process_last_ms=now;process_seen=true;
  process_rx_pending=(uint16_t)((rx_write-rx_read)&255U);
  if (io_error)
  {
    uint32_t error_tick, error_code;
    bool overflow, startup_fe;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    error_tick = io_tick_ms; error_code = io_uart_error; overflow = io_overflow;
    io_error = false;
    __set_PRIMASK(primask);
    if(probe_active) { probe_ready=false;probe_since=now; }
    startup_fe = !tx_attempted && !running && !quarantined && !recovery.active &&
                 !overflow && error_code == HAL_UART_ERROR_FE;
    /* HAL has already aborted RX DMA and restored RxState before this callback.
     * Restart RX only: DMAStop would also abort the in-flight HOME/position TX.
     * Keep its original deadline and require a complete real reply; never replay motion. */
    if(!startup_fe && !quarantined && !recovery.active && !probe_active && !overflow && error_code &&
       !(error_code & ~(HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE)))
    {
      rx_read=rx_write;parse_length=0;
      result=ReceiveStart();
      if(result==HAL_OK) {
        rx_warning_count++;rx_warning_ms=error_tick;rx_warning_error=error_code;
        goto io_processed;
      }
      Quarantine("rx_restart_failed",error_tick,error_code);
    }
    if (recovery.active) {recovery.uart_error=error_code;RecoveryFail("uart_error");}
    if (!startup_fe) Quarantine(overflow ? "rx_overflow" : "uart_error", error_tick, error_code);
    diagnostic_reads = false;
    result = HAL_UART_DMAStop(bus_uart);
    if (startup_fe)
    {
      /* No pre-restart fragment can become a response to a later request. */
      rx_read = rx_write;
      parse_length = 0;
      if (result == HAL_OK) result = ReceiveStart();
      if (result == HAL_OK)
      {
        startup_fe_recovered++;
        startup_fe_last_ms = error_tick;
      }
      else Quarantine("rx_restart_failed", error_tick, error_code);
    }
    else (void)ReceiveStart();
    if (running) {
      Complete(MOTOR_BUS_IO_ERROR, NULL, 0);
      slots[active_owner].event.uart_error=error_code;
    }
    tx_done = false;
  }
io_processed:
  if (running && tx_done)
  {
    tx_done = false;
    active_tx_completed = true;
    if(active.data[0]>=1 && active.data[0]<=8) {
      unsigned id=active.data[0]-1U;
      if(active.data[1]==0xfd) {
        position_motion[id].active=true;position_motion[id].reached=false;
        position_motion[id].tx_ms=tx_done_ms;
      } else if(active.data[1]==0xfe || active.data[1]==0x9a || active.data[1]==0x9c ||
                active.data[1]==0x0a || active.data[1]==0xf3) {
        position_motion[id].active=position_motion[id].reached=false;
      }
    }
    if(active.reply_address>=1 && active.reply_address<=8)timing.id[active.reply_address-1].tx_ms=tx_done_ms;
    if(Urgent(active.data,active.length,active.priority)) {
      unsigned k=active.data[1]==0x9c?1:0;
      unsigned first=active.data[0]?active.data[0]-1U:0U,last=active.data[0]?first+1U:8U;
      for(unsigned id=first;id<last;id++) {
        if(urgent_drain[id][k]<255)urgent_drain[id][k]++;
        urgent_started[id][k]=tx_done_ms;
      }
    }
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
  /* A fast driver may reply before foreground observes TX complete. Keep those bytes. */
  if (!running || waiting_reply)
    while (rx_read != rx_write)
    {
      uint8_t b = rx_ring[rx_read];
      uint32_t arrival_ms = rx_time[rx_read];
      rx_read = (uint16_t)((rx_read + 1U) & 255U);
      ParseByte(b, arrival_ms);
    }
  /* Only frames completed before the deadline can clear the wait above. */
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
    }
    Complete(MOTOR_BUS_TIMEOUT, NULL, 0);
  }
  RecoveryProcess();
  PositionConfigProcess();
  for(unsigned id=0;id<8;id++)for(unsigned k=0;k<2;k++)
    if(urgent_drain[id][k] && now-urgent_started[id][k]>=MOTOR_BUS_TIMEOUT_MS) {
      timing.id[id].timeouts+=urgent_drain[id][k];
      urgent_drain[id][k]=0;
      /* Recovery has its own 500ms quiet/drain phase and stopped checks. */
      if(!recovery.active)Quarantine("stop_drain_timeout",now,0);
    }
  if(probe_active && !probe_ready && !running && !drain_pending && !UrgentPending() &&
     (uint32_t)(now-probe_since)>=MOTOR_BUS_TIMEOUT_MS &&
     (uint32_t)(now-last_rx_ms)>=MOTOR_BUS_TIMEOUT_MS) {
    parse_length=0;rx_read=rx_write;probe_ready=true;
  }
  if (running)
    return;
  for (pass = 0; pass < 2; pass++)
    for (i = 0; i < MOTOR_BUS_OWNER_COUNT; i++)
    {
      unsigned candidate=pass==0?i:(next_owner+i)%MOTOR_BUS_OWNER_COUNT;
      Slot *s = &slots[candidate];
      if (!s->queued || s->priority != (pass == 0))
        continue;
      if(probe_active && !Urgent(s->data,s->length,s->priority) &&
         !ProbeRead((MotorBus_Owner_t)candidate,s->data,s->length,s->reply_length)) continue;
      /* Complete response is necessary, but the shared line must also hand over.
       * Do not add a TX-based gap or delay urgent stopping behind normal ACKs. */
      if(!Urgent(s->data,s->length,s->priority) && reply_complete_valid &&
         (uint32_t)(now-reply_complete_ms)<reply_settle_ms) continue;
      /* Live TTL tests: an immediate FE overlaps the preceding query reply,
       * and back-to-back FE frames lose ACKs. Bound only urgent-stop spacing;
       * ordinary Receive transactions use the short reply handover above.
       * Do not make emergency delivery depend on a potentially missing ACK. */
      if (Urgent(s->data,s->length,s->priority) && wire_tx_completed &&
          (uint32_t)(now-wire_tx_ms)<2U)
        continue;
      if ((drain_pending || UrgentPending()) && !Urgent(s->data,s->length,s->priority))
        continue;
      if (!s->priority && reserved_owner != MOTOR_BUS_OWNER_COUNT && candidate != (unsigned)reserved_owner)
        continue;
      if(!s->priority) {
        /* An atomic wheel batch is one opportunity, not five cursor resets. */
        if(candidate!=MOTOR_BUS_CHASSIS || reserved_owner!=MOTOR_BUS_CHASSIS)
          next_owner=(candidate+1U)%MOTOR_BUS_OWNER_COUNT;
        if(candidate!=MOTOR_BUS_CHASSIS)chassis_yield_mask&=~(1UL<<candidate); }
      timing.transactions++;
      if(Urgent(s->data,s->length,s->priority))timing.emergency++;else timing.ordinary++;
      if(s->reply_address>=1 && s->reply_address<=8) {
        MotorBus_IdTiming_t *t=&timing.id[s->reply_address-1];
        t->queue_last_ms=now-s->queued_ms;
        if(t->queue_last_ms>t->queue_max_ms)t->queue_max_ms=t->queue_last_ms;
      }
      request_rx_start=rx_total;
      active = *s;
      s->queued = false;
      active_owner = (MotorBus_Owner_t)candidate;
      running = true;
      waiting_reply = false;
      cancelled = false;
      tx_done = false;
      started_ms = now;
      active_tx_completed = false;
      /* Preserve partial asynchronous 9F frames across a query submission;
       * ordinary replies still require a reception timestamp after this TX. */
      if (quarantined && (!active.priority || active.reply_length) && !RecoveryRequest(active_owner) &&
          !ProbeRead(active_owner,active.data,active.length,active.reply_length) &&
          !(diagnostic_reads && FeedbackRead(active_owner, active.data,
                                            active.length, active.reply_length)))
      {
        Complete(MOTOR_BUS_CANCELLED, NULL, 0);
        return;
      }
      tx_attempted = true;
      if(active_owner==MOTOR_BUS_RECOVERY)recovery_rx_start=rx_total;
      result = HAL_UART_Transmit_DMA(bus_uart, active.data, active.length);
      if (result != HAL_OK)
      {
        diagnostic_reads = false;
        Complete(MOTOR_BUS_IO_ERROR, NULL, 0);
      }
      return;
    }
}
void MotorBus_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  if (uart != bus_uart || !uart->hdmarx || size > sizeof(dma_rx))
    return;
  /* IDLE can run before a pending HT/TC callback. Its old Size is not a wrap:
   * consume only the live DMA cursor, just as the console RX does. */
  uint16_t remaining=(uint16_t)__HAL_DMA_GET_COUNTER(uart->hdmarx);
  if(remaining>sizeof(dma_rx))return;
  size=(uint16_t)(sizeof(dma_rx)-remaining);
  if(!size)size=sizeof(dma_rx);
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
    wire_tx_ms = tx_done_ms;
    wire_tx_completed = true;
    if(active_owner==MOTOR_BUS_RECOVERY)recovery.tx_sent=true;
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
  out->rx_warning_count=rx_warning_count;out->rx_warning_ms=rx_warning_ms;out->rx_warning_error=rx_warning_error;
  out->startup_fe_recovered = startup_fe_recovered;
  out->startup_fe_last_ms = startup_fe_last_ms;
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
  if (running || recovery.active || probe_active)
    return false;
  for (i = 0; i < MOTOR_BUS_OWNER_COUNT; i++)
    if (slots[i].queued || slots[i].event_ready)
      return false;
  rx_read = rx_write;
  parse_length = 0;
  drain_pending = false;
  memset(urgent_drain,0,sizeof urgent_drain);
  diagnostic_reads = false;
  quarantined = false;
  memset(&first_fault, 0, sizeof(first_fault));
  memset(position_motion,0,sizeof position_motion);
  memset(position_reply_mode,0,sizeof position_reply_mode);
  position_config_id=position_config_phase=0;position_config_pending=false;
  return true;
}

static void RecoveryEvidence(MotorBus_Recovery_t *out)
{
  out->elapsed_ms=HAL_GetTick()-recovery_started;
  out->process_gap_ms=process_gap_ms;
  uint32_t mask=__get_PRIMASK();__disable_irq();
  uint32_t total=rx_total;
  out->rx_bytes=total-recovery_rx_start;out->last_rx_ms=last_rx_ms;
  out->rx_pending=(uint16_t)((rx_write-rx_read)&255U);
  out->dma_position=dma_rx_position;
  out->dma_remaining=bus_uart && bus_uart->hdmarx?(uint16_t)__HAL_DMA_GET_COUNTER(bus_uart->hdmarx):0xffffU;
  out->rx_sample_length=(uint8_t)(out->rx_bytes<16U?out->rx_bytes:16U);
  for(unsigned i=0;i<out->rx_sample_length;i++)out->rx_sample[i]=rx_recent[(total-out->rx_sample_length+i)&15U];
  __set_PRIMASK(mask);out->parse_bytes=parse_length;
}
static void RecoveryFail(const char *reason)
{
  RecoveryEvidence(&recovery);
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
  if (out) {*out = recovery;if(recovery.active)RecoveryEvidence(out);}
}
bool MotorBus_RecoveryStart(void)
{
  unsigned i;
  if (!bus_uart || running || recovery.active || io_error ||
      reserved_owner != MOTOR_BUS_OWNER_COUNT) return false;
  for (i=0;i<MOTOR_BUS_OWNER_COUNT;i++)
    if (slots[i].queued || slots[i].event_ready) return false;
  memset(&recovery,0,sizeof recovery);recovery.active=true;recovery.address=1;
  recovery.stage="quiet_before_stop";recovery_rx_start=rx_total;
  recovery.checked_mask=0;recovery.reason="draining";
  recovery.speed_address=0;recovery.speed_rpm=0;recovery.speed_ms=0;
  recovery_motion_wait=false;
  recovery_step=0;recovery_round=0;recovery_kind=0;
  recovery_started=recovery_wait=HAL_GetTick();
  quarantined=true;diagnostic_reads=false;drain_pending=false;memset(urgent_drain,0,sizeof urgent_drain);
  reserved_owner=MOTOR_BUS_RECOVERY;
  return true;
}
static void RecoveryProcess(void)
{
  MotorBus_Event_t e;
  uint8_t cmd[8], length, reply;
  static const uint8_t functions[3]={0x35,0x3a,0x3b};
  uint32_t now=HAL_GetTick();
  if (!recovery.active) {
    /* Cancellation/IO failure still lets the in-flight DMA complete. */
    (void)MotorBus_EventGet(MOTOR_BUS_RECOVERY,&e);
    return;
  }
  static const char *stages[]={"quiet_before_stop","stop_ids","abort_home","quiet_before_check",
    "health_checks","clear_wheel_cache","sync_wheels","verify_wheels"};
  recovery.stage=stages[recovery_step];
  if ((uint32_t)(now-recovery_started)>8000U) {
    RecoveryFail(recovery_motion_wait?"motor_moving":"recovery_timeout");return;
  }
  if (recovery_step==0 || recovery_step==3) {
    if ((uint32_t)(now-recovery_wait)<500U || (uint32_t)(now-last_rx_ms)<500U) return;
    if (recovery_step==0) {
      recovery.stage="rx_restart";
      if (HAL_UART_DMAStop(bus_uart)!=HAL_OK || ReceiveStart()!=HAL_OK) {
        recovery.uart_error=bus_uart->ErrorCode;
        RecoveryFail("rx_restart_failed");return;
      }
      if(io_error)return; /* Capture the IRQ fault before sending the first stop. */
      rx_read=rx_write;parse_length=0;tx_done=false;
      recovery_step=1;recovery.reason="stopping";
      recovery.stage="stop_ids";
    } else { recovery_step=4;recovery.reason="checking"; }
  }
  if (MotorBus_EventGet(MOTOR_BUS_RECOVERY,&e)) {
    if ((recovery_step<4 && e.result!=MOTOR_BUS_TX_DONE) ||
        (recovery_step>=4 && e.result!=MOTOR_BUS_REPLY)) {
      RecoveryFail(e.result==MOTOR_BUS_TIMEOUT?(e.tx_completed?"reply_timeout":"tx_timeout"):"transport_error");return;
    }
    if((recovery_step==4 || recovery_step==7) && e.length==4 && (e.data[2]==0xe2 || e.data[2]==0xee)) {
      RecoveryFail("driver_rejected");return;
    }
    if(recovery_step==5 && e.data[2]!=2U) {
      RecoveryFail("zero_ack_failed");return;
    }
    if(recovery_step==6 && e.data[2]!=2U) {
      /* Every wheel already ACKed its zero cache. A refused addressed trigger
       * cannot execute an old nonzero cache; verify actual rest before release. */
      recovery.sync_warning_mask|=(uint8_t)(1U<<(recovery.address-1U));
    }
    if (recovery_step==4 || recovery_step==7) {
      if (recovery_kind==0) {
        unsigned speed=((unsigned)e.data[3]<<8)|e.data[4];
        recovery.speed_address=recovery.address;
        recovery.speed_rpm=e.data[2]?-(int32_t)speed:(int32_t)speed;
        recovery.speed_ms=e.tick_ms;
        if (speed>1U) {
          /* FE/9C TX is not stopped proof. Let fresh feedback settle within
           * the existing recovery deadline, without sending new motion. */
          recovery_motion_wait=true;recovery.reason="waiting_stopped";
          recovery_round=0;
          recovery.checked_mask&=recovery_step==7?0xf0U:0U;
          return;
        }
        if (recovery_motion_wait) {
          recovery_motion_wait=false;
          recovery.reason=recovery_step==7?"verifying_zero":"checking";
          recovery.address=1;recovery_kind=0;
          goto submit; /* Repeat the entire stopped check after motion. */
        }
      }
      if (recovery_kind==1 && ((e.data[2]&0x0cU) || !(e.data[2]&1U))) { RecoveryFail("motor_fault");return; }
      if (recovery_kind==2 && ((e.data[2]&0x34U) || (e.data[2]&3U)!=3U)) {
        RecoveryFail("homing_or_driver_fault");return;
      }
      if (++recovery_kind<3) goto submit;
      recovery_kind=0;
      recovery.checked_mask|=(uint8_t)(1U<<(recovery.address-1U));
    }
    if (++recovery.address>(recovery_step>=5?4U:8U)) {
      recovery.address=1;
      if (recovery_step==1) { recovery_step=2;recovery.reason="aborting_home"; }
      else if (recovery_step==2) {
        recovery_step=3;recovery_wait=now;recovery.reason="draining";return;
      } else if (recovery_step==4 && ++recovery_round>=3) {
        /* Drain + fresh read rounds separate abandoned F6/FD acknowledgements
         * from these writes. Overwrite all four caches before any sync. */
        recovery_step=5;recovery.reason="zeroing_chassis";
      } else if (recovery_step==5) {
        recovery_step=6;recovery.reason="syncing_zero";
      } else if (recovery_step==6) {
        recovery_step=7;recovery.reason="verifying_zero";
      } else if (recovery_step==7) {
        RecoveryEvidence(&recovery);
        recovery.active=false;recovery.success=true;recovery.reason="complete";
        quarantined=false;reserved_owner=MOTOR_BUS_OWNER_COUNT;
        memset(position_motion,0,sizeof position_motion);
        memset(position_reply_mode,0,sizeof position_reply_mode);
        position_config_id=position_config_phase=0;position_config_pending=false;
        /* Preserve first_fault for diagnosis; locked reports current state. */
        return;
      }
    }
  }
submit:
  if (MotorBus_OwnerBusy(MOTOR_BUS_RECOVERY)) return;
  recovery.stage=stages[recovery_step];
  if (recovery_motion_wait && (uint32_t)(now-recovery.speed_ms)<100U) return;
  cmd[0]=recovery.address;
  if (recovery_step==1) { cmd[1]=0xfe;cmd[2]=0x98;cmd[3]=0;cmd[4]=0x6b;length=5;reply=0; }
  else if (recovery_step==2) { cmd[1]=0x9c;cmd[2]=0x48;cmd[3]=0x6b;length=4;reply=0; }
  else if (recovery_step==5) {
    cmd[1]=0xf6;                  /* Emm velocity command */
    cmd[2]=0;                    /* Zero RPM has no direction */
    cmd[3]=0;cmd[4]=0;            /* 0 RPM, not enable/reset/home */
    cmd[5]=0;                    /* No acceleration ramp */
    cmd[6]=1;                    /* Replace pending synchronous command */
    cmd[7]=0x6b;length=8;reply=4; /* Receive ACK required from each wheel */
  }
  else if (recovery_step==6) {
    /* Addressed sync never executes pending arm/turntable commands (IDs5..8). */
    cmd[1]=0xff;cmd[2]=0x66;cmd[3]=0x6b;length=4;reply=4;
  }
  else { cmd[1]=functions[recovery_kind];cmd[2]=0x6b;length=3;reply=recovery_kind==0?6:4; }
  if (!MotorBus_Submit(MOTOR_BUS_RECOVERY,cmd,length,reply,0,true)) RecoveryFail("submit_failed");
  else {recovery.function=cmd[1];recovery.tx_sent=false;}
}
