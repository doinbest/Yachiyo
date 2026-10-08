#define main timing_fixture_main
#include "motor_bus_ack_timing_test.c"
#undef main
int main(void)
{
  static const MotorBus_Owner_t owners[]={MOTOR_BUS_CHASSIS,MOTOR_BUS_ARM,MOTOR_BUS_LEGACY,MOTOR_BUS_FEEDBACK,MOTOR_BUS_GUARD,MOTOR_BUS_TURNTABLE};
  uint32_t submitted[MOTOR_BUS_OWNER_COUNT]={0},maximum[MOTOR_BUS_OWNER_COUNT]={0};
  unsigned services[MOTOR_BUS_OWNER_COUNT]={0};
  uint32_t complete_at=0;bool in_flight=false;
  unsigned handled=0;
  assert(MotorBus_Init(&uart)==HAL_OK);
  for(now=0;now<10000;now++) {
    for(unsigned i=0;i<sizeof owners/sizeof owners[0];i++) {
      MotorBus_Owner_t owner=owners[i];MotorBus_Event_t event;
      if(MotorBus_EventGet(owner,&event)) {
        assert(event.result==MOTOR_BUS_REPLY);services[owner]++;
        uint32_t wait=now-submitted[owner];if(wait>maximum[owner])maximum[owner]=wait;
      }
      if(!MotorBus_OwnerBusy(owner)) {
        uint8_t id=i==0?(uint8_t)(1+services[owner]%4):i==1?7:i==2?5:i==3?2:i==4?6:8;
        uint8_t query[]={id,0x3a,0x6b};
        assert(MotorBus_Submit(owner,query,3,0,now,false));submitted[owner]=now;
      }
    }
    if(in_flight && now>=complete_at) {
      uint8_t reply[]={sent[0],sent[1],3,0x6b};
      MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(reply,4);in_flight=false;
    }
    MotorBus_Process();
    if(sends!=handled) {handled=sends;in_flight=true;complete_at=now+1+sends%8;}
  }
  MotorBus_Timing_t timing;MotorBus_TimingGet(&timing);
  for(unsigned i=0;i<8;i++)assert(timing.id[i].success>20);
  for(unsigned i=0;i<sizeof owners/sizeof owners[0];i++) {
    assert(services[owners[i]]>200 && maximum[owners[i]]<=96);
    printf("owner=%u services=%u max_wait=%lu ms\n",owners[i],services[owners[i]],(unsigned long)maximum[owners[i]]);
  }
  puts("motor bus: 10s eight-ID mixed queries, rotating owners, no starvation PASS");
  return 0;
}
