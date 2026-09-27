#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"

static UART_HandleTypeDef uart;
static uint32_t tick;
static unsigned tx_count;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)u; (void)p; (void)n; tx_count++; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ (void)p; (void)n; u->ErrorCode=0; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u; return HAL_OK; }

static void query(void)
{
  const uint8_t cmd[] = {7, 0x3a, 0x6b};
  tick += 20;
  assert(MotorBus_Submit(MOTOR_BUS_ARM, cmd, sizeof(cmd), 4, 1, false));
  MotorBus_Process();
}
int main(void)
{
  MotorBus_Diagnostic_t d;
  MotorBus_Event_t e;
  const uint8_t reply[] = {7, 0x3a, 3, 0x6b};
  const uint8_t cmd[] = {7, 0x3a, 0x6b};
  uint8_t burst[256] = {0};
  assert(MotorBus_Init(&uart) == HAL_OK);
  /* Repeat the user's read-only sequence; successful replies must not lock. */
  for (unsigned i=0; i<3; i++)
  {
    query();
    MotorBus_TxCpltCallback(&uart);
    MotorBus_RxBytes(reply, sizeof(reply));
    MotorBus_Process();
    assert(MotorBus_EventGet(MOTOR_BUS_ARM, &e) && e.result==MOTOR_BUS_REPLY);
    MotorBus_DiagnosticGet(&d);
    assert(!d.locked && !strcmp(d.reason,"none"));
  }
  /* An idle UART fault must survive ReceiveStart clearing HAL ErrorCode. */
  tick=1000; uart.ErrorCode=HAL_UART_ERROR_FE;
  MotorBus_ErrorCallback(&uart);
  tick=1003; MotorBus_Process();
  MotorBus_DiagnosticGet(&d);
  assert(d.locked && !strcmp(d.reason,"uart_error") && d.tick_ms==1000);
  assert(d.uart_error==HAL_UART_ERROR_FE && !d.active && !d.waiting_reply);
  assert(d.owner==MOTOR_BUS_ARM && d.address==7 && d.function==0x3a);
  assert(uart.ErrorCode==0);
  unsigned previous=tx_count;
  assert(!MotorBus_Submit(MOTOR_BUS_ARM,cmd,sizeof(cmd),4,2,false));
  MotorBus_Process(); assert(tx_count==previous);
  uart.ErrorCode=HAL_UART_ERROR_ORE; MotorBus_ErrorCallback(&uart); MotorBus_Process();
  MotorBus_DiagnosticGet(&d);
  assert(d.uart_error==HAL_UART_ERROR_FE && d.tick_ms==1000); /* first cause */
  assert(MotorBus_RecoverAfterReset());
  MotorBus_DiagnosticGet(&d); assert(!d.locked && !strcmp(d.reason,"none"));

  query(); MotorBus_TxCpltCallback(&uart); MotorBus_Process();
  tick+=MOTOR_BUS_TIMEOUT_MS; MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e) && e.result==MOTOR_BUS_TIMEOUT);
  MotorBus_DiagnosticGet(&d);
  assert(!strcmp(d.reason,"reply_timeout") && d.active && d.waiting_reply);
  assert(d.address==7 && d.function==0x3a && !d.uart_error);
  assert(MotorBus_RecoverAfterReset());

  query(); tick+=MOTOR_BUS_TIMEOUT_MS; MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e));
  MotorBus_DiagnosticGet(&d);
  assert(!strcmp(d.reason,"tx_timeout") && d.active && !d.waiting_reply);
  assert(MotorBus_RecoverAfterReset());

  query(); MotorBus_TxCpltCallback(&uart); MotorBus_Process();
  MotorBus_Cancel(MOTOR_BUS_ARM);
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e));
  MotorBus_DiagnosticGet(&d);
  assert(!strcmp(d.reason,"cancelled_reply") && d.address==7);
  assert(MotorBus_RecoverAfterReset());

  MotorBus_RxBytes(burst,sizeof(burst)); MotorBus_Process();
  MotorBus_DiagnosticGet(&d);
  assert(!strcmp(d.reason,"rx_overflow") && !d.uart_error);
  puts("motor_bus_diagnostic_test: repeat reads, first idle UART fault, timeout, cancellation, overflow PASS");
}
