#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_recovery_main
#include "motor_recovery_test.c"
#undef main
#pragma GCC diagnostic pop
int main(int argc,char **argv)
{
  MotorBus_Recovery_t s;
  assert(MotorBus_Init(&uart)==HAL_OK);
  tick=1000;moving=1;
  assert(MotorBus_RecoveryStart());
  for(unsigned i=0;i<500;i++){
    step();MotorBus_RecoveryGet(&s);
    if(!strcmp(s.reason,"waiting_stopped"))break;
    assert(i<499);
  }
  assert(s.active && s.address==1 && s.function==0x35 && s.tx_sent && s.tx_ms);
  uint32_t previous_tx=s.tx_ms;
  unsigned before=sends;
  tick=s.speed_ms+100;
  /* A real complete unsolicited 9F enforces ordinary 2 ms line handover. */
  const uint8_t notify[]={7,0xfd,0x9f,0x6b};
  MotorBus_RxBytes(notify,sizeof notify);MotorBus_Process();
  MotorBus_RecoveryGet(&s);
  assert(sends==before && !s.tx_sent);
  printf("queued_repeat addr=%u func=%02X tx_sent=%u tx_len=%u tx_ms=%lu previous_tx=%lu now=%lu\n",s.address,s.function,s.tx_sent,s.tx_length,(unsigned long)s.tx_ms,(unsigned long)previous_tx,(unsigned long)tick);
  assert(!s.tx_length && !s.tx_ms);
  if(argc>1 && !strcmp(argv[1],"dispatch")) {
    tick+=3;MotorBus_Process();step();MotorBus_RecoveryGet(&s);
    assert(s.tx_sent && s.tx_length==3 && s.tx_ms>previous_tx);
    moving=0;s=run();assert(s.success);puts("recovery evidence: new dispatch owns fresh TX, recovery completes PASS");return 0;
  }
  /* A second error now freezes the same contradictory attempt snapshot. */
  uart.ErrorCode=HAL_UART_ERROR_DMA;MotorBus_ErrorCallback(&uart);MotorBus_Process();
  MotorBus_RecoveryGet(&s);
  assert(!s.active && !s.tx_sent && !s.tx_ms && !s.tx_length);
  printf("failed_repeat reason=%s tx_sent=%u tx_len=%u tx_ms=%lu\n",s.reason,s.tx_sent,s.tx_length,(unsigned long)s.tx_ms);
  return 0;
}
