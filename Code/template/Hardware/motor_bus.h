#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H
#include "main.h"
#include <stdbool.h>
#include <string.h>

#define MOTOR_BUS_FRAME_MAX 40U /* Emm configuration read returns 33 bytes. */
#define MOTOR_BUS_TIMEOUT_MS 100U
typedef enum
{
  MOTOR_BUS_STOP,
  MOTOR_BUS_CHASSIS,
  MOTOR_BUS_ARM,
  MOTOR_BUS_LEGACY,
  MOTOR_BUS_FEEDBACK,
  MOTOR_BUS_RECOVERY,
  MOTOR_BUS_GUARD,
  MOTOR_BUS_TURNTABLE,
  MOTOR_BUS_PROBE,
  MOTOR_BUS_POSITION_CONFIG,
  MOTOR_BUS_OWNER_COUNT
} MotorBus_Owner_t;
typedef enum
{
  MOTOR_BUS_TX_DONE,
  MOTOR_BUS_REPLY,
  MOTOR_BUS_TIMEOUT,
  MOTOR_BUS_IO_ERROR,
  MOTOR_BUS_CANCELLED
} MotorBus_Result_t;
typedef struct
{
  MotorBus_Result_t result;
  uint32_t token, tick_ms;
  uint32_t tx_ms; /* DMA completion timestamp; valid only when tx_completed. */
  uint8_t data[MOTOR_BUS_FRAME_MAX], length;
  bool tx_completed;
  uint32_t uart_error; /* Current transaction's HAL error; not the first historical fault. */
  uint8_t read_retries; /* Only a bounded wheel 3A read retry; never motion replay. */
  uint32_t first_read_tx_ms; /* Conservative age floor if either read attempt replied. */
} MotorBus_Event_t;
/** First quarantine cause, retained until explicit recovery. When active=false,
 * owner/address/function identify the last dispatched transaction, not a cause. */
typedef struct
{
  const char *reason;
  uint32_t tick_ms, uart_error;
  MotorBus_Owner_t owner;
  uint8_t address, function;
  bool locked, active, waiting_reply;
  /** Pre-first-TX framing faults successfully recovered without locking the bus.
   * Counters survive manual recovery; last_ms is MCU uptime of the last FE. */
  uint32_t startup_fe_recovered, startup_fe_last_ms;
  uint32_t rx_warning_count, rx_warning_ms, rx_warning_error; /* RX restarted without motion replay. */
  uint32_t rx_preserved_replies; /* Complete pre-error matched replies retained; lifetime counter. */
  uint16_t rx_error_pending; /* Bytes queued at the last successful RX restart's error boundary. */
  /* First-fault receive evidence, retained through manual recovery. Bytes are
   * from the request dispatch; sample is its last up to 16 received bytes. */
  uint32_t rx_bytes, last_rx_ms, process_gap_ms;
  uint16_t rx_pending;
  uint8_t parse_bytes, rx_sample_length, rx_sample[16];
  uint32_t tx_ms;
  uint8_t tx_length, tx_sample[16]; /* First-fault command, at most first16 bytes; length is full frame. */
  uint32_t read_retry_count, read_retry_ms, read_retry_rx_bytes;
  uint8_t read_retry_address, read_retry_sample_length, read_retry_sample[16];
} MotorBus_Diagnostic_t;
/** Current recoverable transport fault; historical diagnostics never trigger motion. */
static inline bool MotorBus_TransientFault(const MotorBus_Diagnostic_t *fault)
{
  if(!fault || !fault->locked || !fault->reason)return false;
  if(!strcmp(fault->reason,"reply_timeout") || !strcmp(fault->reason,"tx_timeout") ||
     !strcmp(fault->reason,"rx_overflow") || !strcmp(fault->reason,"drain_timeout"))return true;
  return !strcmp(fault->reason,"uart_error") && fault->uart_error &&
    !(fault->uart_error & ~(HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE));
}
/** @brief Read first fault without UART traffic, clearing it, or enabling motion.
 * @param out Snapshot destination; NULL is ignored. Call from the main loop.
 * tick_ms is milliseconds since MCU startup; uart_error is HAL_UART_ERROR_* bits. */
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *out);
/** Init once after UART/DMA setup; UART5 RX must use DMA_CIRCULAR.
 * Main loop is the sole submit/event consumer. */
HAL_StatusTypeDef MotorBus_Init(UART_HandleTypeDef *uart);
/** Copies the complete frame; true means accepted, never means transmitted. */
bool MotorBus_Submit(MotorBus_Owner_t owner, const uint8_t *data, uint8_t length,
                     uint8_t reply_length, uint32_t token, bool priority);
/** @brief Submit a copied frame with a distinct reply address (broadcast FF: ID1). */
bool MotorBus_SubmitReplyTo(MotorBus_Owner_t owner, const uint8_t *data, uint8_t length,
                           uint8_t reply_address, uint8_t reply_length,
                           uint32_t token, bool priority);
typedef struct
{
  uint32_t requests, success, rejected, timeouts;
  uint32_t tx_ms, reply_ms, reply_last_ms, reply_max_ms, queue_last_ms, queue_max_ms;
} MotorBus_IdTiming_t;
typedef struct
{
  MotorBus_IdTiming_t id[8];
  uint32_t transactions, ordinary, emergency, drained;
} MotorBus_Timing_t;
/** @brief Main-loop RAM snapshot; timestamps/durations in MCU milliseconds. */
void MotorBus_TimingGet(MotorBus_Timing_t *out);
bool MotorBus_EventGet(MotorBus_Owner_t owner, MotorBus_Event_t *event);
/* IDs5..8: read/preserve parameters, select Both in RAM, then verify.
 * Returns 1 ready, 0 pending, -1 failed; no motion is sent by this helper. */
int MotorBus_PositionReplyEnsure(uint8_t address);
/* Same preserved RAM configuration/readback, selecting driver Receive=1 or
 * Both=3. Pending transitions never grant readiness from an older mode cache. */
int MotorBus_PositionReplyEnsureMode(uint8_t address,uint8_t mode);
/* Stop/arrival feedback owns the bus before background configuration resumes.
 * In-flight traffic completes or is preempted by the existing urgent-stop path;
 * only unsent configuration work is cancelled here. Main-loop only. */
void MotorBus_PositionConfigStopHold(bool hold);
/* Matching FD/9F after the latest transmitted position command. Receive/02,
 * stops, HOME and superseded position targets never establish completion. */
bool MotorBus_PositionReached(uint8_t address, uint32_t accepted_ms);
bool MotorBus_OwnerBusy(MotorBus_Owner_t owner);
/** @brief True only while this owner has an on-wire/waiting transaction. */
bool MotorBus_OwnerInFlight(MotorBus_Owner_t owner);
/** Reserve complete transaction; existing in-flight request drains, stop can preempt. */
bool MotorBus_Reserve(MotorBus_Owner_t owner);
void MotorBus_Release(MotorBus_Owner_t owner);
/** Cancel unsent request; an in-flight DMA completes using its original buffer. */
void MotorBus_Cancel(MotorBus_Owner_t owner);
void MotorBus_Process(void);
void MotorBus_RxBytes(const uint8_t *data, uint16_t length);
/** @brief Forward HAL circular RX IDLE/HT/TC events from IRQ.
 * @param uart Initialized motor UART.
 * @param size HAL's buffer position, not the length since the previous event.
 * Copies only new bytes; normal callbacks do not stop or restart reception. */
void MotorBus_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void MotorBus_TxCpltCallback(UART_HandleTypeDef *uart);
void MotorBus_ErrorCallback(UART_HandleTypeDef *uart);
#define MOTOR_BUS_RX_ERROR_DIAG 1
#define MOTOR_BUS_RX_ERROR_DMA_BYTES 64U
typedef struct {
  uint32_t sequence,tick_ms,uart_sr,uart_error,request_seq,token,tx_ms,rx_total;
  uint16_t dma_remaining,dma_remaining_end,dma_cursor,queued;
  uint8_t phase,owner,active,waiting,tx_complete,tx_length,partial;
  uint8_t dma[MOTOR_BUS_RX_ERROR_DMA_BYTES],tx[MOTOR_BUS_FRAME_MAX];
} MotorBus_RxErrorSnapshot_t;
/* UART5 IRQ calls this before HAL may abort DMA. Diagnostic only: no parsing,
 * DMA restart or UART error acknowledgement. phase1=IRQ, phase0=callback fallback. */
void MotorBus_UartErrorIRQ(UART_HandleTypeDef *uart,uint32_t status_register);
/* Atomic CPU-only snapshot. first_fault selects the frozen matching quarantine
 * image; false selects latest error. DMA data includes old circular-buffer bytes
 * and is NEVER fed back into the parser or used as motion/standstill evidence. */
bool MotorBus_RxErrorGet(MotorBus_RxErrorSnapshot_t *out,bool first_fault);
bool MotorBus_IsQuarantined(void);
/** Targeted read-only diagnostics. Reserve ordinary traffic, wait for old reads
 * and a 100 ms quiet line, then permit only this ID's 35/36/3A/3B queries.
 * May run while quarantined; never clears faults or permits motor motion.
 * Caller must first ensure task producers are idle. Emergency stops preempt. */
bool MotorBus_ProbeStart(uint8_t address);
bool MotorBus_ProbeReady(void);
bool MotorBus_ProbeActive(void);
void MotorBus_ProbeStop(void);
/** Read-only feedback remains available after cancelled chassis motion ACKs.
 * This never grants permission to resume motion or clears a fault. */
bool MotorBus_FeedbackAllowed(void);
/** Explicit physical driver reset + quiet bus required; Stop does not clear caches. */
bool MotorBus_RecoverAfterReset(void);
typedef struct
{
  bool active, success;
  uint8_t address, checked_mask;
  uint8_t sync_warning_mask; /* Addressed zero sync refused; zero caches and fresh rest still verified. */
  const char *reason;
  uint8_t speed_address;
  int32_t speed_rpm;
  uint32_t speed_ms;
  /* This attempt's failure evidence, independent of the retained first fault. */
  const char *stage;
  uint32_t uart_error, elapsed_ms, rx_bytes, last_rx_ms, process_gap_ms;
  uint16_t rx_pending, dma_remaining, dma_position;
  uint8_t function, parse_bytes, rx_sample_length, rx_sample[16];
  bool tx_sent;
  uint32_t tx_ms;
  uint8_t tx_length, tx_sample[16]; /* This attempt's command, not a previous recovery stage. */
} MotorBus_Recovery_t;
static inline bool MotorBus_RecoveryRetryable(const MotorBus_Recovery_t *s)
{
  if(!s || s->active || s->success || !s->reason)return false;
  if(!strcmp(s->reason,"reply_timeout"))return true;
  return (!strcmp(s->reason,"uart_error") || !strcmp(s->reason,"rx_restart_failed")) && s->uart_error &&
    !(s->uart_error & ~(HAL_UART_ERROR_NE | HAL_UART_ERROR_FE | HAL_UART_ERROR_ORE));
}
/** @brief Start explicit recovery after all producers have stopped and drained.
 * Keeps normal traffic locked; stops/aborts IDs 1..8, drains old replies, then
 * checks three rounds of speed/state/home flags, overwrites IDs1..4 sync caches
 * with 0 RPM (Receive ACK each), triggers addressed sync, then verifies fresh
 * wheel speed/state/home flags. Refused addressed sync is a warning only after
 * all zero-cache ACKs; success still requires fresh stopped wheel feedback.
 * Rechecks residual speed every 100 ms within the existing 8 s limit; any
 * moving sample restarts stopped validation, persistent motion stays locked.
 * Never enables, resets position, homes, broadcasts sync or resumes a task.
 * Returns false if another transaction, event or reservation remains. */
bool MotorBus_RecoveryStart(void);
/** @brief Cancel recovery; leave motion locked. Main-loop only. */
void MotorBus_RecoveryCancel(void);
/** @brief Read recovery progress without bus traffic. */
void MotorBus_RecoveryGet(MotorBus_Recovery_t *out);
#endif
