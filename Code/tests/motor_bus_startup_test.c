#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
#include "mechanical_arm.h"

static uint32_t tick;
static unsigned tx_count, rx_starts;
static HAL_StatusTypeDef rx_result, stop_result;
static UART_HandleTypeDef uart = {.Instance=UART5};
UART_HandleTypeDef huart5;
static uint8_t sent[32];
uint32_t HAL_GetTick(void) { return tick; }
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
bool Mecanum_Test_Stop(void) { return true; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u; memcpy(sent,p,n); tx_count++; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)p; (void)n; rx_starts++; u->ErrorCode=0; return rx_result; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u; return stop_result; }
static void frame_error(void)
{ uart.ErrorCode=HAL_UART_ERROR_FE; MotorBus_ErrorCallback(&uart); MotorBus_Process(); }

int main(int argc,char **argv)
{
  MotorBus_Diagnostic_t d;
  MechanicalArm_EventTypeDef event;
  const uint8_t ack[]={6,0x9a,2,0x6b};
  const uint8_t stale[]={6,0x9a,2};
  assert(MechanicalArm_Init(&uart)==HAL_OK);
  tick=29;
  if(argc>1 && strncmp(argv[1],"active_",7)) {
    if(!strcmp(argv[1],"restart_fail")) { rx_result=HAL_ERROR; frame_error(); }
    else if(!strcmp(argv[1],"stop_fail")) { stop_result=HAL_ERROR; frame_error(); }
    else if(!strcmp(argv[1],"dma_fail")) {
      uart.ErrorCode=HAL_UART_ERROR_DMA; MotorBus_ErrorCallback(&uart); MotorBus_Process();
    } else {
      uint8_t flood[256]={0}; MotorBus_RxBytes(flood,sizeof flood); MotorBus_Process();
    }
    assert(MotorBus_IsQuarantined() && tx_count==0);
    MotorBus_DiagnosticGet(&d);
    assert(d.startup_fe_recovered==0);
    if(!strcmp(argv[1],"restart_fail") || !strcmp(argv[1],"stop_fail"))
      assert(!strcmp(d.reason,"rx_restart_failed") && d.uart_error==HAL_UART_ERROR_FE);
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_TX_ERROR);
    puts("motor_bus_startup_test: unrecovered receive fault blocks homing PASS");
    return 0;
  }
  MotorBus_RxBytes(stale,sizeof stale);
  frame_error();
  assert(!MotorBus_IsQuarantined()); /* Reproduce the reported 29 ms boot FE. */
  MotorBus_DiagnosticGet(&d);
  assert(d.startup_fe_recovered==1 && d.startup_fe_last_ms==29 && !strcmp(d.reason,"none"));
  assert(rx_starts==2 && tx_count==0);
  tick=38; frame_error();
  assert(!MotorBus_IsQuarantined() && rx_starts==3);
  tick=1000;
  assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_BUSY);
  /* The third startup transient preserves the pending Receive preflight. */
  frame_error(); assert(tx_count==1 && !MotorBus_IsQuarantined());
  assert(sent[0]==6 && sent[1]==0x42);
  uint8_t cfg[33]={6,0x42,33,21};cfg[22]=1;cfg[32]=0x6b;
  MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(cfg,sizeof cfg);MotorBus_Process();
  tick+=3;MotorBus_Process();tx_count=0; /* Subsequent counts isolate HOME, not preflight. */
  assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_Z,2)==MECHANICAL_ARM_RESULT_NONE);
  MotorBus_Process();
  assert(tx_count==1 && sent[0]==6 && sent[1]==0x9a && sent[2]==2);
  if(argc>1) {
    tick++;
    uart.ErrorCode=!strcmp(argv[1],"active_ne")?HAL_UART_ERROR_NE:
      !strcmp(argv[1],"active_ore")?HAL_UART_ERROR_ORE:HAL_UART_ERROR_FE;
    MotorBus_ErrorCallback(&uart);MotorBus_Process();MechanicalArm_Process();
    MotorBus_DiagnosticGet(&d);
    assert(!d.locked && tx_count==1 && d.rx_warning_count==1);
    assert(!MechanicalArm_ResultGet(&event));
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();
    if(!strcmp(argv[1],"active_missing")) {
      tick+=100;MotorBus_Process();MechanicalArm_Process();
      assert(MotorBus_IsQuarantined());
      assert(MechanicalArm_ResultGet(&event) && event.Result!=MECHANICAL_ARM_RESULT_OK);
    } else {
      MotorBus_RxBytes(ack,sizeof ack);MotorBus_Process();MechanicalArm_Process();
      assert(MechanicalArm_ResultGet(&event) && event.Result==MECHANICAL_ARM_RESULT_OK);
    }
    assert(tx_count==1); /* HOME was never replayed. */
    puts("motor_bus_startup_test: in-flight RX restart, real reply or timeout PASS");
    return 0;
  }
  MotorBus_TxCpltCallback(&uart); MotorBus_Process(); MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&event)); /* Stale pre-HOME data cannot ACK. */
  MotorBus_RxBytes(ack,sizeof ack); MotorBus_Process(); MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&event) && event.Result==MECHANICAL_ARM_RESULT_OK);
  tick+=20; frame_error();
  MotorBus_DiagnosticGet(&d);
  assert(!d.locked && tx_count==1);
  assert(d.startup_fe_recovered==3 && d.startup_fe_last_ms==1000);
  assert(MotorBus_RecoverAfterReset());
  tick+=20; frame_error();
  assert(!MotorBus_IsQuarantined()); /* Runtime line noise is recoverable too. */
  puts("motor_bus_startup_test: boot FE recovery, Z HOME, post-TX RX warning PASS");
}
