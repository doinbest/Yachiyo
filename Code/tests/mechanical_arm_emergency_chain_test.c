#define main legacy_arm_fixture_main
#include "mechanical_arm_bus_test.c"
#undef main
static void single_axis_stop_preempts_ack_and_keeps_scope(unsigned index)
{
  MechanicalArm_EventTypeDef result;
    tx_outcome=HAL_OK;tx_count=0;tick=0;
    assert(MechanicalArm_Init(&uart)==HAL_OK);
    assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_Z,100)==MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Process();assert(frame[0]==6 && frame[1]==0xfd);
    MotorBus_TxCpltCallback(&uart);MotorBus_Process(); /* Position ACK deliberately absent. */
    MechanicalArm_AxisTypeDef axis=(MechanicalArm_AxisTypeDef)index;
    assert(MechanicalArm_Stop(axis)==MECHANICAL_ARM_RESULT_NONE);
    for(unsigned n=0;n<100 && tx_count<2;n++) {
      MotorBus_Process();MechanicalArm_Process();tick++;
    }
    assert(tx_count==2 && frame[0]==5+index && frame[1]==0xfe);
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&result) && result.Action==MECHANICAL_ARM_ACTION_STOP &&
           result.Axis==axis && result.Transmitted && result.Result==MECHANICAL_ARM_RESULT_OK);
    assert(MotorBus_IsQuarantined()); /* Missing motion ACK keeps the existing isolation; FE still transmits. */
}
int main(int argc,char **argv)
{
  if(argc>1 && !strncmp(argv[1],"single_",7)) {
    unsigned axis=(unsigned)(argv[1][7]-'0');assert(axis<3);
    single_axis_stop_preempts_ack_and_keeps_scope(axis);
    puts("single-axis stop: pending position ACK preempted, selected ID only PASS");return 0;
  }
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
  /* A normal ACK leaves all three scoped stops and later positions usable. */
  for(unsigned axis=0;axis<3;axis++) {
    tick+=200;MotorBus_Process();
    assert(MechanicalArm_Position((MechanicalArm_AxisTypeDef)axis,100)==MECHANICAL_ARM_RESULT_NONE);
    unsigned before=tx_count;MotorBus_Process();assert(tx_count==before+1 && frame[1]==0xfd);
    uint8_t ack[]={frame[0],0xfd,2,0x6b};
    MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(ack,4);MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&result) && result.Result==MECHANICAL_ARM_RESULT_OK);
    assert(MechanicalArm_Stop((MechanicalArm_AxisTypeDef)axis)==MECHANICAL_ARM_RESULT_NONE);
    tick+=10;MotorBus_Process();assert(frame[0]==5+axis && frame[1]==0xfe);
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();MechanicalArm_Process();
    assert(MechanicalArm_ResultGet(&result) && result.Result==MECHANICAL_ARM_RESULT_OK);
    ack[1]=0xfe;MotorBus_RxBytes(ack,4);MotorBus_Process();
    assert(!MotorBus_IsQuarantined());
  }
  /* Stop queued Z then X while a position is still unsent: keep both scopes,
   * never deliver the obsolete position or add Base/wheels/ID8. */
  tick+=10;MotorBus_Process();unsigned before=tx_count;
  assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_Z,100)==MECHANICAL_ARM_RESULT_NONE);
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE);
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_X)==MECHANICAL_ARM_RESULT_NONE);
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_Z)==MECHANICAL_ARM_RESULT_NONE);
  MechanicalArm_Process();
  for(unsigned id=6;id<=7;id++) {
    tick+=2;MotorBus_Process();assert(tx_count==before+id-5 && frame[0]==id && frame[1]==0xfe);
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();MechanicalArm_Process();
    uint8_t ack[]={(uint8_t)id,0xfe,2,0x6b};MotorBus_RxBytes(ack,4);MotorBus_Process();
  }
  assert(MechanicalArm_ResultGet(&result) && result.Result==MECHANICAL_ARM_RESULT_OK);
  assert(result.StopIdMask==0x60U);
  assert(!MechanicalArm_IsBusy() && !MotorBus_IsQuarantined());
  puts("arm stop-all: IDs1..8 TX-driven despite missing ACK/first TX error, fragmented late ACK drain PASS");
}
