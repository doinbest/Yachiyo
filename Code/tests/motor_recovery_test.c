#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t sent[32];
static unsigned sends, handled;
static int moving, missing, rx_fail, zero_nack, sync_missing, post_moving;
static bool restart_noise, sync_nack;
static bool tx_never_completes;
static uint32_t injected_noise;
static uint8_t noise_address=7,noise_function=0x35;
static unsigned transient_moving, speed_reads;
static unsigned post_transient;
static uint8_t speed_sign, stopped_speed;
static uint8_t state_flags=3, home_flags=3, stop_mask, abort_mask;
/* Driver boundary fixture: latent wheel commands must be replaced before any
 * sync, and recovery must not trigger the arm/turntable caches. */
static uint8_t zero_mask, sync_mask, post_mask;
static int cached[8]={20,30,40,50,60,70,80,90};
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;memcpy(sent,p,n);sends++;return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)p;(void)n;if(restart_noise){restart_noise=false;u->ErrorCode=HAL_UART_ERROR_NE;MotorBus_ErrorCallback(u);}return rx_fail?HAL_ERROR:HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
static void step(void)
{
  uint8_t r[8]={sent[0],sent[1],0,0,0,0x6b};
  if(sends!=handled) {
    handled=sends;
    if(tx_never_completes){tick+=10;MotorBus_Process();return;}
    MotorBus_TxCpltCallback(&uart);
    if(sent[1]==0x35) {
      r[2]=speed_sign;
      r[4]=(moving || (post_moving && sync_mask==15))?20:stopped_speed;
      if(sent[0]==5 && transient_moving) { r[4]=20;transient_moving--; }
      if(sent[0]==3 && sync_mask==15 && post_transient) { r[4]=20;post_transient--; }
      speed_reads++;
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
      sync_mask|=1U<<(sent[0]-1);r[2]=sync_nack?0xe2:2;r[3]=0x6b;
      if(!sync_missing)MotorBus_RxBytes(r,4);
    } else {
      assert(sent[0]>=1 && sent[0]<=8);
      assert(sent[1]==0xfe || sent[1]==0x9c); /* Never replays motion. */
      if(sent[1]==0xfe)stop_mask|=1U<<(sent[0]-1);
      else abort_mask|=1U<<(sent[0]-1);
    }
  }
  if(injected_noise && sent[0]==noise_address && sent[1]==noise_function){
    uart.ErrorCode=injected_noise;injected_noise=0;MotorBus_ErrorCallback(&uart);
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
  MotorBus_Diagnostic_t fault={.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
  assert(MotorBus_TransientFault(&fault));
  fault.locked=false;assert(!MotorBus_TransientFault(&fault)); /* historical evidence only */
  fault.locked=true;fault.uart_error=HAL_UART_ERROR_DMA;assert(!MotorBus_TransientFault(&fault));
  fault.reason="reply_timeout";assert(MotorBus_TransientFault(&fault));
  fault.reason="motor_fault";assert(!MotorBus_TransientFault(&fault));
  assert(MotorBus_Init(&uart)==HAL_OK);
  /* Exercise recovery from a runtime FE, after the startup-only exception ends. */
  {
    const uint8_t query[]={7,0x3a,0x6b}; MotorBus_Event_t event;
    assert(MotorBus_Submit(MOTOR_BUS_ARM,query,sizeof query,4,1,false));
    MotorBus_Process(); step();
    assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event) && event.result==MOTOR_BUS_REPLY);
  }
  uart.ErrorCode=HAL_UART_ERROR_DMA;MotorBus_ErrorCallback(&uart);MotorBus_Process();
  assert(MotorBus_IsQuarantined());
  assert(MotorBus_RecoveryStart());
  assert(!MotorBus_Submit(MOTOR_BUS_ARM,move,3,4,0,false));
  s=run();assert(s.success && !MotorBus_IsQuarantined());
  assert(s.checked_mask==0xff && tick>=500);
  assert(stop_mask==0xff && abort_mask==0xff);
  assert(zero_mask==15 && sync_mask==15 && post_mask==15);
  for(unsigned i=0;i<4;i++)assert(cached[i]==0);
  for(unsigned i=4;i<8;i++)assert(cached[i]==(int)(i+2)*10);
  /* Complete replies before a transient UART error must survive in recovery too. */
  const uint32_t noise_codes[]={HAL_UART_ERROR_NE,HAL_UART_ERROR_FE,HAL_UART_ERROR_ORE};
  for(unsigned i=0;i<3;i++) {
    injected_noise=noise_codes[i];assert(MotorBus_RecoveryStart());s=run();
    assert(!injected_noise && s.success && !MotorBus_IsQuarantined());
  }
  /* Base can report residual speed after FE/abort; fresh stopped reads must
   * allow the same recovery request to finish without a second click. */
  transient_moving=2;speed_reads=0;
  assert(MotorBus_RecoveryStart());s=run();
  assert(s.success && !MotorBus_IsQuarantined() && transient_moving==0);
  assert(speed_reads>=34 && s.checked_mask==0xff);
  post_transient=2;assert(MotorBus_RecoveryStart());s=run();
  assert(s.success && post_transient==0 && s.checked_mask==0xff && post_mask==15);
  stopped_speed=1;speed_sign=1;assert(MotorBus_RecoveryStart());s=run();
  assert(s.success && s.speed_rpm==-1 && s.speed_address==4 && s.speed_ms<=tick);
  stopped_speed=0;speed_sign=0;
  sync_nack=true;assert(MotorBus_RecoveryStart());s=run();
  assert(s.success && !MotorBus_IsQuarantined() && zero_mask==15 && post_mask==15 && s.sync_warning_mask==15);
  post_moving=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined() && !strcmp(s.reason,"motor_moving"));
  post_moving=0;
  sync_nack=false;
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
  moving=1;speed_sign=1;
  unsigned started=tick;
  assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined());assert(!strcmp(s.reason,"motor_moving"));
  assert(tick-started>8000 && s.speed_rpm==-20 && s.speed_address==1);
  assert(tick-s.speed_ms<150 && zero_mask==0 && sync_mask==0);
  /* Cancelling during speed rechecks must not proceed to zero/sync. */
  assert(MotorBus_RecoveryStart());zero_mask=sync_mask=0;
  for(unsigned i=0;i<800;i++) {
    step();MotorBus_RecoveryGet(&s);
    if(!strcmp(s.reason,"waiting_stopped"))break;
    assert(i<799);
  }
  MotorBus_RecoveryCancel();step();MotorBus_RecoveryGet(&s);
  assert(!s.active && !s.success && !strcmp(s.reason,"cancelled") &&
         MotorBus_IsQuarantined() && zero_mask==0 && sync_mask==0);
  speed_sign=0;
  moving=0;missing=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && MotorBus_IsQuarantined());assert(s.address==1);
  missing=0;rx_fail=1;assert(MotorBus_RecoveryStart());s=run();
  assert(!s.success && !strcmp(s.reason,"rx_restart_failed"));
  rx_fail=0;assert(MotorBus_RecoveryStart());s=run();assert(s.success);
  state_flags=7;assert(MotorBus_RecoveryStart());s=run();assert(!s.success);
  state_flags=2;assert(MotorBus_RecoveryStart());s=run();assert(!s.success); /* no disabled-axis auto resume */
  state_flags=3;home_flags=7;assert(MotorBus_RecoveryStart());s=run();assert(!s.success);
  home_flags=3;assert(MotorBus_RecoveryStart());
  uart.ErrorCode=4;MotorBus_ErrorCallback(&uart);step();
  MotorBus_RecoveryGet(&s);assert(s.active && !s.success && MotorBus_IsQuarantined());
  s=run();assert(s.success);
  assert(MotorBus_RecoveryStart());
  uart.ErrorCode=HAL_UART_ERROR_DMA;MotorBus_ErrorCallback(&uart);step();
  MotorBus_RecoveryGet(&s);assert(!s.active && !s.success && MotorBus_IsQuarantined());
  assert(MotorBus_RecoveryStart());MotorBus_RecoveryCancel();
  MotorBus_RecoveryGet(&s);assert(!s.active && !s.success && MotorBus_IsQuarantined());
  restart_noise=true;assert(MotorBus_RecoveryStart());s=run();
  assert(s.success && !MotorBus_IsQuarantined()); /* Noise during RX restart is retained for the next pass. */
  tx_never_completes=true;assert(MotorBus_RecoveryStart());s=run();
  assert(!strcmp(s.reason,"tx_timeout") && !s.tx_sent && !MotorBus_RecoveryRetryable(&s));
  puts("motor_recovery_test: recover, motion rejection, missing driver, RX failure and cancel OK");
}
