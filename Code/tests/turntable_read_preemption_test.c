/* Reuse the virtual drive, with the real turntable and MotorBus linked below. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_turntable_main
#include "turntable_test.c"
#undef main
#pragma GCC diagnostic pop
static bool permit_expired_motion;

static void setup_preemption(void)
{
  tick=10;sends=handled=position_commands=stop_commands=homes=aborts=0;
  permit_expired_motion=false;
  speed_rpm=moving=missing=reject=hold_position=missing_motion_ack=disabled=0;
  reached_missing=home_failed=force_home_running=motor_fault=power_disabled=0;
  home_busy=arrival_bias=0;position_raw=16384;saved_zero=0;console[0]=0;
  assert(MotorBus_Init(&uart)==HAL_OK);
  fixture_none_modes(&uart,&tick);handled=sends;Turntable_Init();
  assert(Turntable_Origin());run();
  assert(Turntable_InventorySet(1,TURNTABLE_OCCUPIED,2));
  console[0]=0;
}
static void wait_query(uint8_t function,bool home,bool dma)
{
  unsigned before=home?homes:position_commands;
  assert(home?Turntable_Index(1):Turntable_Index(2));
  for(unsigned i=0;i<500;i++) {
    step();
    if((home?homes:position_commands)>before && sends!=handled && sent[1]==function) {
      if(!dma){handled=sends;MotorBus_TxCpltCallback(&uart);MotorBus_Process();}
      assert(MotorBus_OwnerInFlight(MOTOR_BUS_TURNTABLE));return;
    }
  }
  assert(0 && "ID8 read was not issued");
}
static void old_reply(uint8_t function)
{
  uint8_t r[8]={8,function,0,0,0,0,0,0x6b};unsigned length=4;
  if(function==0x36)length=8;
  else if(function==0x35){length=6;r[5]=0x6b;}
  else {r[2]=3;r[3]=0x6b;}
  MotorBus_RxBytes(r,(uint16_t)length);MotorBus_Process();
}
static void priority_stop(uint8_t function,bool dma,bool deliver_old)
{
  static const uint8_t command[]={1,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,command,5,0,999,true));
  if(dma){handled=sends;MotorBus_TxCpltCallback(&uart);MotorBus_Process();}
  Turntable_Process();
  assert(!strstr(console,"ERR turntable fault") ||
    (permit_expired_motion && strstr(console,"position_timeout")));
  Turntable_Status_t s;Turntable_StatusGet(&s);
  assert(Turntable_IsBusy() && !s.arrived && !s.feedback_valid && s.reference_valid);
  /* Complete the wheel stop ourselves; the drive fixture handles ID8 only. */
  for(unsigned i=0;i<20 && (sent[0]!=1 || sent[1]!=0xfe);i++){tick++;MotorBus_Process();}
  assert(sent[0]==1 && sent[1]==0xfe);handled=sends;
  MotorBus_TxCpltCallback(&uart);
  {const uint8_t ack[]={1,0xfe,2,0x6b};MotorBus_RxBytes(ack,4);}
  MotorBus_Process();MotorBus_Event_t event;
  assert(MotorBus_EventGet(MOTOR_BUS_STOP,&event) && event.result==MOTOR_BUS_TX_DONE);
  if(deliver_old)old_reply(function);
}
static void resume_read(uint8_t function,bool home,bool dma)
{
  setup_preemption();wait_query(function,home,dma);
  unsigned moves=position_commands,started_homes=homes,stops=stop_commands;
  priority_stop(function,dma,true);
  run();Turntable_Status_t s;Turntable_StatusGet(&s);
  assert(s.arrived && s.feedback_valid && s.reference_valid);
  assert(s.slot==(home?1:2));
  assert(position_commands==moves && homes==started_homes && stop_commands==stops);
  assert(Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED);
  assert(!MotorBus_IsQuarantined() && !strstr(console,"ERR turntable fault"));
}
static void missing_old_reply_remains_fault(void)
{
  setup_preemption();wait_query(0x36,false,false);priority_stop(0x36,false,false);
  tick+=110;MotorBus_Process();Turntable_Process();
  assert(MotorBus_IsQuarantined());MotorBus_Diagnostic_t d;MotorBus_DiagnosticGet(&d);
  assert(!strcmp(d.reason,"drain_timeout"));
}
static void cancelled_command_is_not_replayed(uint8_t function)
{
  setup_preemption();
  assert(function==0xfd?Turntable_Index(2):function==0x9a?Turntable_Index(1):
    function==0xf3?Turntable_Enable(true):Turntable_SaveZero());
  for(unsigned i=0;i<500;i++) {
    step();
    if(sends!=handled && sent[1]==function)break;
    assert(i<499);
  }
  handled=sends; /* Cancel while TX is still in flight; None finishes at TC. */
  const uint8_t command[]={1,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,command,5,0,999,true));
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  Turntable_Process();
  assert(strstr(console,"ERR turntable fault reason=transport_error"));
  /* A parameter write can be uncertain; a None action has no abandoned ACK. */
  assert(MotorBus_IsQuarantined()==(function==0x93));
}
static void repeated_preemption_keeps_motion_deadline(bool dense)
{
  setup_preemption();hold_position=1;
  permit_expired_motion=dense;
  uint32_t began=tick,next_cancel=tick;unsigned interrupted=0;
  assert(Turntable_Index(2));
  for(unsigned i=0;i<900 && Turntable_IsBusy();i++) {
    step();
    if(position_commands && sends!=handled && sent[1]==0x36 && tick>=next_cancel &&
       !strstr(console,"ERR turntable")) {
      handled=sends;MotorBus_TxCpltCallback(&uart);MotorBus_Process();
      priority_stop(0x36,false,true);interrupted++;next_cancel=tick+(dense?0:800);
      if(!strstr(console,"ERR turntable"))console[0]=0;
    }
  }
  assert(interrupted>=3 && !Turntable_IsBusy());
  assert(strstr(console,"position_timeout") && tick-began<5700);
  assert(position_commands==1); /* Cancellation never restarts the move/budget. */
}
static void explicit_stop_revokes_read_continuation(void)
{
  setup_preemption();wait_query(0x36,false,false);priority_stop(0x36,false,true);
  unsigned moves=position_commands,home_count=homes;
  Turntable_Stop();run();Turntable_Status_t s;Turntable_StatusGet(&s);
  assert(!s.arrived && !strcmp(s.reason,"stopped") && s.feedback_valid);
  assert(position_commands==moves && homes==home_count);
}
int main(int argc,char **argv)
{
  if(argc>1 && !strcmp(argv[1],"missing"))missing_old_reply_remains_fault();
  else if(argc>1 && !strcmp(argv[1],"motion"))cancelled_command_is_not_replayed(0xfd);
  else if(argc>1 && !strcmp(argv[1],"home_command"))cancelled_command_is_not_replayed(0x9a);
  else if(argc>1 && !strcmp(argv[1],"enable_command"))cancelled_command_is_not_replayed(0xf3);
  else if(argc>1 && !strcmp(argv[1],"save_command"))cancelled_command_is_not_replayed(0x93);
  else if(argc>1 && !strcmp(argv[1],"deadline"))repeated_preemption_keeps_motion_deadline(false);
  else if(argc>1 && !strcmp(argv[1],"deadline_dense"))repeated_preemption_keeps_motion_deadline(true);
  else if(argc>1 && !strcmp(argv[1],"stop"))explicit_stop_revokes_read_continuation();
  else {
    const uint8_t functions[]={0x36,0x35,0x3a};
    for(unsigned i=0;i<3;i++)for(unsigned dma=0;dma<2;dma++)resume_read(functions[i],false,dma!=0);
    resume_read(0x3b,true,false);resume_read(0x3b,true,true);
  }
  puts("ID8 read preemption: PASS");return 0;
}
