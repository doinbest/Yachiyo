#define main unused_legacy_main
#include "mecanum_bus_test.c"
#undef main

int main(int argc,char **argv)
{
  MotorBus_Event_t e;
  MotorBus_Diagnostic_t d;
  Mecanum_Status_t status;
  const uint8_t move[]={5,0xfd,0,0,10,20,0,0,0,100,0,0,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  /* This regression deliberately exercises abandoned Receive compatibility;
   * the production default None path has no pending motion ACK to cancel. */
  assert(MotorBus_PositionReplyEnsureMode(5,1)==0);MotorBus_Process();
  uint8_t cfg[33]={5,0x42,33,21};cfg[22]=1;cfg[32]=0x6b;
  MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(cfg,33);MotorBus_Process();in_flight=false;
  assert(MotorBus_PositionReplyEnsureMode(5,1)==1);count=0;
  tick=1000;
  assert(MotorBus_Submit(MOTOR_BUS_ARM,move,sizeof move,4,99,false));
  MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();in_flight=false;
  MotorBus_Cancel(MOTOR_BUS_ARM);
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e) && e.result==MOTOR_BUS_CANCELLED);
  assert(MotorBus_IsQuarantined() && MotorBus_FeedbackAllowed());
  tick=1004;
  Mecanum_Feedback_StopMonitor(true);Mecanum_Process();MotorBus_Process();
  assert(count==2 && frame[1][0]==1 && frame[1][1]==0x35);
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();in_flight=false;
  tick=1005;
  const uint8_t stopped[]={1,0x35,0,0,0,0x6b};
  bool missing=argc>1 && !strcmp(argv[1],"missing");
  bool fatal=argc>1 && !strcmp(argv[1],"fatal");
  if(!missing)MotorBus_RxBytes(stopped,sizeof stopped);
  uart.ErrorCode=fatal?HAL_UART_ERROR_DMA:argc>1 && !strcmp(argv[1],"fe")?HAL_UART_ERROR_FE:argc>1 && !strcmp(argv[1],"ore")?HAL_UART_ERROR_ORE:HAL_UART_ERROR_NE;MotorBus_ErrorCallback(&uart);MotorBus_Process();Mecanum_Process();
  MotorBus_DiagnosticGet(&d);Mecanum_StatusGet(&status);
  Mecanum_Feedback_t f;assert(Mecanum_FeedbackGet(0,&f));
  if(missing){
    assert(MotorBus_FeedbackAllowed() && !f.speed_valid);
    tick+=100;MotorBus_Process();Mecanum_Process();
    assert(!MotorBus_FeedbackAllowed() && MotorBus_IsQuarantined());
  }else if(fatal)assert(!MotorBus_FeedbackAllowed() && !f.speed_valid && status.locked);
  else assert(MotorBus_FeedbackAllowed() && f.speed_valid && f.speed_rpm==0 && status.locked);
  const uint8_t forbidden[]={1,0xf6,0,0,10,0,1,0x6b};
  assert(!MotorBus_Submit(MOTOR_BUS_CHASSIS,forbidden,sizeof forbidden,4,100,false));
  printf("stop_monitor_noise: feedback_allowed=%u speed_valid=%u chassis_locked=%u chassis_error=%u warnings=%lu first_reason=%s sends=%u\n",
    MotorBus_FeedbackAllowed(),f.speed_valid,status.locked,status.error,(unsigned long)d.rx_warning_count,d.reason,count);
  return 0;
}
