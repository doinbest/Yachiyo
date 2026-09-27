#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick;
static uint8_t *dma;
static unsigned tx_count;
static HAL_StatusTypeDef tx_result;
static UART_HandleTypeDef uart;
uint32_t HAL_GetTick(void)
{
  return tick;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  (void)n;
  dma = p;
  tx_count++;
  return tx_result;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  (void)p;
  (void)n;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u)
{
  (void)u;
  return HAL_OK;
}
static void finish(void)
{
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
}
int main(void)
{
  MotorBus_Event_t e;
  uint8_t cmd[] = {1, 0x35, 0x6b};
  uint8_t reply[] = {2, 0x35, 0, 0, 42, 0x6b, 1, 0x35, 1, 0x6b, 2, 0x6b};
  assert(MotorBus_Init(&uart) == HAL_OK);
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, 6, 17, false));
  cmd[0] = 99;
  MotorBus_Process();
  assert(dma[0] == 1 && tx_count == 1);
  assert(!MotorBus_EventGet(MOTOR_BUS_ARM, &e));
  finish();
  assert(!MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e));
  MotorBus_RxBytes(reply, 9);
  MotorBus_Process();
  assert(!MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e));
  MotorBus_RxBytes(reply + 9, 3);
  MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e));
  assert(e.result == MOTOR_BUS_REPLY && e.token == 17 && e.data[3] == 0x6b);
  cmd[0] = 3;
  assert(MotorBus_Submit(MOTOR_BUS_ARM, cmd, 3, 6, 18, false));
  MotorBus_Process();
  assert(tx_count == 1);
  tick = 10;
  MotorBus_Process();
  assert(tx_count == 2);
  finish();
  tick = 110;
  MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM, &e) && e.result == MOTOR_BUS_TIMEOUT);
  assert(MotorBus_IsQuarantined());
  assert(!MotorBus_Submit(MOTOR_BUS_ARM, cmd, 3, 6, 19, false));
  assert(MotorBus_Submit(MOTOR_BUS_STOP, cmd, 3, 0, 20, true));
  tick = 120;
  MotorBus_Process();
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_STOP, &e) && e.result == MOTOR_BUS_TX_DONE);
  assert(MotorBus_RecoverAfterReset());
  /* Reservation spans gaps between commands, even if arm was queued first. */
  cmd[0] = 5;
  assert(MotorBus_Submit(MOTOR_BUS_ARM, cmd, 3, 0, 100, false));
  assert(MotorBus_Reserve(MOTOR_BUS_CHASSIS));
  cmd[0] = 1;
  assert(MotorBus_Submit(MOTOR_BUS_CHASSIS, cmd, 3, 0, 101, false));
  tick = 130;
  MotorBus_Process();
  assert(dma[0] == 1);
  tick = 131;
  MotorBus_TxCpltCallback(&uart);
  tick = 137;
  MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS, &e) && e.tick_ms == 131);
  unsigned saved = tx_count;
  tick = 151;
  MotorBus_Process();
  assert(tx_count == saved);
  MotorBus_Release(MOTOR_BUS_CHASSIS);
  MotorBus_Process();
  assert(dma[0] == 5);
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM, &e) && e.token == 100);
  cmd[0] = 1;
  tick = 170;
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, 6, 102, false));
  MotorBus_Process();
  /* Reply before foreground sees TX done must be retained; timestamp is arrival. */
  tick = 171;
  MotorBus_RxBytes(reply + 6, 6);
  MotorBus_Process();
  tick = 172;
  MotorBus_TxCpltCallback(&uart);
  tick = 180;
  MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e) && e.result == MOTOR_BUS_REPLY &&
         e.tick_ms == 171);
  /* Stop preempts an arm reply wait after TX, instead of waiting 100 ms. */
  uint8_t emergency[] = {1, 0xfe, 0x98, 0, 0x6b};
  cmd[0] = 5;
  tick = 190;
  assert(MotorBus_Submit(MOTOR_BUS_ARM, cmd, 3, 6, 103, false));
  MotorBus_Process();
  finish();
  tick = 201;
  assert(MotorBus_Submit(MOTOR_BUS_STOP, emergency, 5, 0, 104, true));
  MotorBus_Process();
  assert(dma[1] == 0xfe);
  assert(MotorBus_EventGet(MOTOR_BUS_ARM, &e) && e.result == MOTOR_BUS_CANCELLED && e.tx_completed);
  assert(MotorBus_IsQuarantined());
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_STOP, &e) && e.result == MOTOR_BUS_TX_DONE);
  assert(MotorBus_RecoverAfterReset());
  tick = 212;
  assert(MotorBus_Submit(MOTOR_BUS_ARM, cmd, 3, 6, 105, false));
  MotorBus_Process();
  assert(dma[1] == 0x35);
  assert(MotorBus_Submit(MOTOR_BUS_STOP, emergency, 5, 0, 106, true));
  MotorBus_Process();
  assert(dma[1] == 0x35); /* In-flight DMA buffer remains untouched. */
  assert(!MotorBus_EventGet(MOTOR_BUS_ARM, &e));
  tick = 213;
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM, &e) && e.result == MOTOR_BUS_CANCELLED);
  tick = 222;
  MotorBus_Process();
  assert(dma[1] == 0x35);
  tick = 223;
  MotorBus_Process();
  assert(dma[1] == 0xfe);
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_STOP, &e) && e.result == MOTOR_BUS_TX_DONE);
  assert(MotorBus_RecoverAfterReset());
  /* A cancelled read must drain its reply without permanently locking Stop. */
  tick = 240;
  cmd[0] = 1;
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, 6, 107, false));
  MotorBus_Process();
  finish();
  MotorBus_RxBytes(reply + 6, 3);
  MotorBus_Process(); /* Partial old response must survive priority Stop TX. */
  tick = 251;
  assert(MotorBus_Submit(MOTOR_BUS_STOP, emergency, 5, 0, 108, true));
  MotorBus_Process();
  assert(dma[1] == 0xfe);
  assert(!MotorBus_IsQuarantined());
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e) && e.result == MOTOR_BUS_CANCELLED);
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_STOP, &e));
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, 6, 109, false));
  saved = tx_count;
  tick = 270;
  MotorBus_Process();
  assert(tx_count == saved); /* No new query can steal the abandoned reply. */
  MotorBus_RxBytes(reply + 9, 3);
  MotorBus_Process();
  assert(tx_count == saved + 1);
  assert(!MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e));
  finish();
  MotorBus_RxBytes(reply + 6, 6);
  MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e) && e.result == MOTOR_BUS_REPLY && e.token == 109);
  /* Missing abandoned response remains a real fault, not automatic recovery. */
  tick = 290;
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK, cmd, 3, 6, 110, false));
  MotorBus_Process();
  MotorBus_Cancel(MOTOR_BUS_FEEDBACK); /* Also cover cancellation during DMA. */
  assert(!MotorBus_IsQuarantined());
  tick = 291;
  finish();
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK, &e) && e.result == MOTOR_BUS_CANCELLED);
  tick = 391;
  MotorBus_Process();
  assert(MotorBus_IsQuarantined());
  MotorBus_Diagnostic_t diagnostic;
  MotorBus_DiagnosticGet(&diagnostic);
  assert(!strcmp(diagnostic.reason, "drain_timeout"));
  assert(diagnostic.owner == MOTOR_BUS_FEEDBACK && diagnostic.address == 1 && diagnostic.function == 0x35);
  puts("motor_bus_test: cancellation drains read reply, Stop priority, fault isolation OK");
}
