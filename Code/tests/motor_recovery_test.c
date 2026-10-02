#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t sent[32];
static unsigned sends, handled;
static int moving, missing, rx_fail;
static uint8_t state_flags=3, home_flags=3, stop_mask, abort_mask;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;memcpy(sent,p,n);sends++;return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;return rx_fail?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
static void step(void)
{
  uint8_t r[8]={sent[0],sent[1],0,0,0,0x6b};
  if(sends!=handled) {
    handled=sends;MotorBus_TxCpltCallback(&uart);
    if(sent[1]==0x35) { r[4]=moving?20:0;if(!missing)MotorBus_RxBytes(r,6); }
    else if(sent[1]==0x3a || sent[1]==0x3b) {
      r[2]=sent[1]==0x3a?state_flags:home_flags;r[3]=0x6b;if(!missing)MotorBus_RxBytes(r,4);
    } else {
      assert(sent[0]>=1 && sent[0]<=7);
      assert(sent[1]==0xfe || sent[1]==0x9c); /* Never replays motion. */
      if(sent[1]==0xfe)stop_mask|=1U<<(sent[0]-1);
      else abort_mask|=1U<<(sent[0]-1);
    }
  }
  tick+=10;MotorBus_Process();
}
static MotorBus_Recovery_t run(void)
{
  MotorBus_Recovery_t s;unsigned i;
  for(i=0;i<1000;i++){step();MotorBus_RecoveryGet(&s);if(!s.active)return s;}
  assert(0);return s;
}
int main(void)
{
  MotorBus_Recovery_t s;uint8_t move[]={5,0xfd,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  uart.ErrorCode=4;MotorBus_ErrorCallback(&uart);MotorBus_Process();
  assert(MotorBus_IsQuarantined());
  assert(MotorBus_RecoveryStart());
  assert(!MotorBus_Submit(MOTOR_BUS_ARM,move,3,4,0,false));
  s=run();assert(s.success && !MotorBus_IsQuarantined());
  assert(s.checked_mask==0x7f && tick>=500);
  assert(stop_mask==0x7f && abort_mask==0x7f);
  moving=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined());assert(!strcmp(s.reason,"motor_moving"));
  moving=0;missing=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined());assert(s.address==1);
  missing=0;rx_fail=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && !strcmp(s.reason,"rx_restart_failed"));
  rx_fail=0;assert(MotorBus_RecoveryStart());s=run();assert(s.success);
  state_flags=7;assert(MotorBus_RecoveryStart());s=run();assert(!s.success);
  state_flags=3;home_flags=7;assert(MotorBus_RecoveryStart());s=run();assert(!s.success);
  home_flags=3;assert(MotorBus_RecoveryStart());
  uart.ErrorCode=4;MotorBus_ErrorCallback(&uart);step();
  MotorBus_RecoveryGet(&s);assert(!s.active && !s.success && MotorBus_IsQuarantined());
  assert(MotorBus_RecoveryStart());MotorBus_RecoveryCancel();
  MotorBus_RecoveryGet(&s);assert(!s.active && !s.success && MotorBus_IsQuarantined());
  puts("motor_recovery_test: recover, motion rejection, missing driver, RX failure and cancel OK");
}
