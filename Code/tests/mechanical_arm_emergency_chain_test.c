#define main legacy_arm_fixture_main
#include "mechanical_arm_bus_test.c"
#undef main
int main(int argc,char **argv)
{
  (void)argv;
  MechanicalArm_EventTypeDef result;
  assert(MechanicalArm_Init(&uart)==HAL_OK);
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_ALL)==MECHANICAL_ARM_RESULT_NONE);
  for(unsigned id=1;id<=8;id++) {
    tx_outcome=argc>1 && id==1?HAL_ERROR:HAL_OK;
    MotorBus_Process();assert(tx_count==id && frame[0]==id && frame[1]==0xfe);
    if(tx_outcome==HAL_OK)MotorBus_TxCpltCallback(&uart);
    MotorBus_Process();MechanicalArm_Process();tick+=2;
  }
  assert(MechanicalArm_ResultGet(&result));
  assert(result.Result==(argc>1?MECHANICAL_ARM_RESULT_TX_ERROR:MECHANICAL_ARM_RESULT_OK));
  uint8_t late[32];unsigned n=0;
  for(unsigned id=argc>1?2:1;id<=8;id++) {
    late[n++]=(uint8_t)id;late[n++]=0xfe;late[n++]=2;late[n++]=0x6b;
  }
  MotorBus_RxBytes(late,7);MotorBus_Process();MotorBus_RxBytes(late+7,(uint16_t)(n-7));MotorBus_Process();
  assert(!MotorBus_IsQuarantined());
  puts("arm stop-all: IDs1..8 TX-driven despite missing ACK/first TX error, fragmented late ACK drain PASS");
}
