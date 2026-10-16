/* Production bus: interrupted motion remains locked, but stop evidence is readable. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main legacy_bus_fixture_main
#include "motor_bus_test.c"
#undef main
#pragma GCC diagnostic pop
static void compatibility_receive(void)
{
  assert(MotorBus_PositionReplyEnsureMode(7,1)==0);MotorBus_Process();
  uint8_t cfg[33]={7,0x42,33,21};cfg[22]=1;cfg[32]=0x6b;
  MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(cfg,33);MotorBus_Process();
  assert(MotorBus_PositionReplyEnsureMode(7,1)==1);
}
static void cancelled_motion_allows_stop_evidence(unsigned owner,uint8_t function)
{
  MotorBus_Event_t event;
  uint8_t motion[]={7,0xfd,0,0,20,8,0,0,0,0,10,0,0x6b};
  uint8_t stop[]={7,0xfe,0x98,0,0x6b};
  uint8_t read[]={7,0x36,0x6b};
  uint8_t late[]={7,0xfd,2,0x6b};
  uint8_t position[]={7,0x36,0,0,0,0,42,0x6b};
  assert(MotorBus_RecoverAfterReset());
  tx_result=HAL_OK;tick=100;tx_count=0;
  assert(MotorBus_Init(&uart)==HAL_OK);
  compatibility_receive();
  const uint8_t *cmd=function==0xfe?stop:motion;
  unsigned length=function==0xfe?sizeof(stop):sizeof(motion);
  assert(MotorBus_Submit(owner,cmd,(uint8_t)length,4,1,false));
  MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  MotorBus_Cancel(owner);
  assert(MotorBus_EventGet(owner,&event) && event.result==MOTOR_BUS_CANCELLED);
  assert(MotorBus_IsQuarantined());
  assert(MotorBus_FeedbackAllowed());
  assert(!MotorBus_Submit(MOTOR_BUS_ARM,motion,sizeof(motion),4,2,false));
  assert(MotorBus_Submit(MOTOR_BUS_ARM,read,sizeof(read),8,3,false));
  MotorBus_Process();assert(dma[1]==0x36);
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  late[1]=function;
  MotorBus_RxBytes(late,sizeof(late));MotorBus_Process();
  assert(!MotorBus_EventGet(MOTOR_BUS_ARM,&event));
  MotorBus_RxBytes(position,sizeof(position));MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event) && event.result==MOTOR_BUS_REPLY && event.data[6]==42);
  assert(MotorBus_IsQuarantined());
  uint8_t state[]={6,0x3a,0x6b},reply[]={6,0x3a,3,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_ARM,state,sizeof(state),4,4,false));
  MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  MotorBus_RxBytes(reply,sizeof(reply));MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event) && event.result==MOTOR_BUS_REPLY);
}
int main(void)
{
  cancelled_motion_allows_stop_evidence(MOTOR_BUS_ARM,0xfd);
  cancelled_motion_allows_stop_evidence(MOTOR_BUS_ARM,0xfe);
  cancelled_motion_allows_stop_evidence(MOTOR_BUS_CHASSIS,0xfd);
  /* A genuine missing ACK does not gain the cancellation-only read exception. */
  uint8_t cmd[]={7,0xfd,0,0,20,8,0,0,0,0,10,0,0x6b};
  MotorBus_Event_t event;assert(MotorBus_RecoverAfterReset());tick=100;assert(MotorBus_Init(&uart)==HAL_OK);
  compatibility_receive();
  assert(MotorBus_Submit(MOTOR_BUS_ARM,cmd,sizeof(cmd),4,5,false));
  MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();tick+=101;MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event));
  assert(MotorBus_IsQuarantined() && !MotorBus_FeedbackAllowed());
  puts("cancelled arm/chassis motion: fresh position/state, late ACK ignored, motion remains locked PASS");
}
