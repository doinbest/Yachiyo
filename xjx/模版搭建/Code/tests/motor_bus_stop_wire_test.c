#define main original_ack_test_main
#include "motor_bus_ack_timing_test.c"
#undef main
int main(void)
{
  MotorBus_Event_t e;
  uint8_t query[]={2,0x35,0x6b}, stop[]={8,0xfe,0x98,0,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  now=1000;assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,query,3,6,1,false));
  MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,2,true));
  unsigned before=sends;MotorBus_Process();assert(sends==before);
  now++;uint8_t reply[]={2,0x35,0,0,0,0x6b};
  MotorBus_RxBytes(reply,sizeof reply);MotorBus_Process();assert(sends==before);
  now++;MotorBus_Process();assert(sends==before+1 && sent[0]==8);
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_STOP,&e) && e.result==MOTOR_BUS_TX_DONE);
  stop[0]=7;assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,3,true));
  before=sends;now++;MotorBus_Process();assert(sends==before);
  now++;MotorBus_Process();assert(sends==before+1 && sent[0]==7);
  /* Bounded wire spacing, never a dependency on the previous stop ACK. */
  assert(!MotorBus_IsQuarantined());
  puts("urgent stop wire: cancelled read drains before stop, 2ms bounded spacing without ACK dependency PASS");
}
