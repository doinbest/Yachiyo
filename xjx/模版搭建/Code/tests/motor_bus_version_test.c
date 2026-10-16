#define main original_probe_regression_main
#include "motor_bus_probe_test.c"
#undef main
static void version_query(unsigned token)
{
  uint8_t cmd[]={5,0x1f,0x6b};
  bool accepted=MotorBus_Submit(MOTOR_BUS_PROBE,cmd,3,7,token,false);
  if(!accepted)fprintf(stderr,"version submit token=%u tick=%lu ready=%u busy=%u\n",token,(unsigned long)tick,MotorBus_ProbeReady(),MotorBus_OwnerBusy(MOTOR_BUS_PROBE));
  assert(accepted);
  advance(3);assert(frame[0]==5 && frame[1]==0x1f && frame[2]==0x6b);tx_done();
}
int main(void)
{
  assert(MotorBus_Init(&uart)==HAL_OK);start(5);
  MotorBus_Event_t e;
  uint8_t short_reply[]={5,0x1f,0xf4,0x78,0x6b};
  uint8_t long_reply[]={5,0x1f,0,200,0x03,20,0x6b};
  version_query(1);
  MotorBus_RxBytes(short_reply,3);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  MotorBus_RxBytes(short_reply+3,2);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);
  assert(e.length==5 && !memcmp(e.data,short_reply,5));
  version_query(2);
  MotorBus_RxBytes(long_reply,5);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  MotorBus_RxBytes(long_reply+5,2);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);
  assert(e.length==7 && !memcmp(e.data,long_reply,7));
  /* A request echo alone is not a reply; a foreign driver cannot complete it. */
  version_query(3);uint8_t echo[]={5,0x1f,0x6b},foreign[]={6,0x1f,0xf4,0x78,0x6b};
  MotorBus_RxBytes(echo,3);MotorBus_RxBytes(foreign,5);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  MotorBus_RxBytes(short_reply,5);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);assert(e.length==5);
  /* Unsupported-command rejection belongs only to this selected version read. */
  version_query(4);uint8_t rejected[]={5,0,0xee,0x6b};
  MotorBus_RxBytes(rejected,4);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);assert(e.length==4 && e.data[2]==0xee);
  /* Cancellation drains a real5-byte reply without acknowledging the cancelled read. */
  version_query(5);MotorBus_Cancel(MOTOR_BUS_PROBE);event(MOTOR_BUS_PROBE,MOTOR_BUS_CANCELLED);
  MotorBus_RxBytes(short_reply,5);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  advance(101);assert(MotorBus_ProbeReady());version_query(6);MotorBus_RxBytes(long_reply,7);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);assert(e.length==7);
  assert(!MotorBus_IsQuarantined());
  version_query(7);unsigned before=transmissions;advance(101);event(MOTOR_BUS_PROBE,MOTOR_BUS_TIMEOUT);assert(transmissions==before);
  MotorBus_ProbeStop();assert(MotorBus_IsQuarantined());
  puts("motor_bus_version_test: 5/7-byte split replies, echo, foreign ID, rejection, cancellation drain and no retransmit PASS");return 0;
}
