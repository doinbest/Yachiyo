#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t sent[32];
static unsigned sends, handled;
static int moving, missing, rx_fail, zero_nack, sync_missing, post_moving;
static uint8_t state_flags=3, home_flags=3, stop_mask, abort_mask;
/* Driver boundary fixture: latent wheel commands must be replaced before any
 * sync, and recovery must not trigger the arm/turntable caches. */
static uint8_t zero_mask, sync_mask, post_mask;
static int cached[8]={20,30,40,50,60,70,80,90};
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
    if(sent[1]==0x35) {
      r[4]=(moving || (post_moving && sync_mask==15))?20:0;
      if(sync_mask==15 && sent[0]<=4)post_mask|=1U<<(sent[0]-1);
      if(!missing)MotorBus_RxBytes(r,6);
    }
    else if(sent[1]==0x3a || sent[1]==0x3b) {
      r[2]=sent[1]==0x3a?state_flags:home_flags;r[3]=0x6b;if(!missing)MotorBus_RxBytes(r,4);
    } else if(sent[1]==0xf6) {
      assert(sent[0]>=1 && sent[0]<=4);
      assert(sent[3]==0 && sent[4]==0 && sent[5]==0 && sent[6]==1 && sent[7]==0x6b);
      r[2]=zero_nack?0xe2:2;r[3]=0x6b;
      if(!zero_nack) { cached[sent[0]-1]=0;zero_mask|=1U<<(sent[0]-1); }
      MotorBus_RxBytes(r,4);
    } else if(sent[1]==0xff) {
      assert(sent[0]>=1 && sent[0]<=4); /* Never broadcast to IDs5..8. */
      assert(zero_mask==15 && cached[sent[0]-1]==0);
      assert(sent[2]==0x66 && sent[3]==0x6b);
      sync_mask|=1U<<(sent[0]-1);r[2]=2;r[3]=0x6b;
      if(!sync_missing)MotorBus_RxBytes(r,4);
    } else {
      assert(sent[0]>=1 && sent[0]<=8);
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
  zero_mask=sync_mask=post_mask=0;
  for(i=0;i<1000;i++){step();MotorBus_RecoveryGet(&s);if(!s.active)return s;}
  assert(0);return s;
}
int main(void)
{
  MotorBus_Recovery_t s;uint8_t move[]={5,0xfd,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  /* Exercise recovery from a runtime FE, after the startup-only exception ends. */
  {
    const uint8_t query[]={7,0x3a,0x6b}; MotorBus_Event_t event;
    assert(MotorBus_Submit(MOTOR_BUS_ARM,query,sizeof query,4,1,false));
    MotorBus_Process(); step();
    assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event) && event.result==MOTOR_BUS_REPLY);
  }
  uart.ErrorCode=4;MotorBus_ErrorCallback(&uart);MotorBus_Process();
  assert(MotorBus_IsQuarantined());
  assert(MotorBus_RecoveryStart());
  assert(!MotorBus_Submit(MOTOR_BUS_ARM,move,3,4,0,false));
  s=run();assert(s.success && !MotorBus_IsQuarantined());
  assert(s.checked_mask==0xff && tick>=500);
  assert(stop_mask==0xff && abort_mask==0xff);
  assert(zero_mask==15 && sync_mask==15 && post_mask==15);
  for(unsigned i=0;i<4;i++)assert(cached[i]==0);
  for(unsigned i=4;i<8;i++)assert(cached[i]==(int)(i+2)*10);
  zero_nack=1;sync_mask=0;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined() && sync_mask==0);
  assert(!strcmp(s.reason,"zero_ack_failed"));
  zero_nack=0;sync_missing=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined());
  sync_missing=0;post_moving=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined());
  post_moving=0;assert(MotorBus_RecoveryStart());s=run();assert(s.success);
  /* Stop/cancel can interrupt a zero write; no subsequent sync or unlock. */
  assert(MotorBus_RecoveryStart());zero_mask=sync_mask=0;
  for(unsigned i=0;i<800;i++) {
    step();MotorBus_RecoveryGet(&s);
    if(!strcmp(s.reason,"zeroing_chassis") && sent[1]==0xf6)break;
    assert(i<799);
  }
  MotorBus_RecoveryCancel();step();MotorBus_RecoveryGet(&s);
  assert(!s.active && !s.success && MotorBus_IsQuarantined() && sync_mask==0);
  assert(MotorBus_RecoveryStart());s=run();assert(s.success);
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
