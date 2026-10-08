#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H
#include "main.h"
#include <stdbool.h>

#define MOTOR_BUS_FRAME_MAX 32U
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
} MotorBus_Diagnostic_t;
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
bool MotorBus_IsQuarantined(void);
/** Read-only feedback remains available after cancelled chassis motion ACKs.
 * This never grants permission to resume motion or clears a fault. */
bool MotorBus_FeedbackAllowed(void);
/** Explicit physical driver reset + quiet bus required; Stop does not clear caches. */
bool MotorBus_RecoverAfterReset(void);
typedef struct
{
  bool active, success;
  uint8_t address, checked_mask;
  const char *reason;
} MotorBus_Recovery_t;
/** @brief Start explicit recovery after all producers have stopped and drained.
 * Keeps normal traffic locked; stops/aborts IDs 1..8, drains old replies, then
 * checks three rounds of speed/state/home flags, overwrites IDs1..4 sync caches
 * with 0 RPM (Receive ACK each), triggers addressed sync, then verifies fresh
 * wheel speed/state/home flags. success means caches cleared and stopped.
 * Never enables, resets position, homes, broadcasts sync or resumes a task.
 * Returns false if another transaction, event or reservation remains. */
bool MotorBus_RecoveryStart(void);
/** @brief Cancel recovery; leave motion locked. Main-loop only. */
void MotorBus_RecoveryCancel(void);
/** @brief Read recovery progress without bus traffic. */
void MotorBus_RecoveryGet(MotorBus_Recovery_t *out);
#endif
