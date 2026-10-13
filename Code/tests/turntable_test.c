#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "turntable.h"
#include "motor_bus.h"

static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t sent[32];
static unsigned sends, handled, position_commands, zero_commands, stop_commands;
static int32_t position_raw = 16384; /* Actual driver position = 90 degrees. */
static int speed_rpm, moving, missing, reject, hold_position, missing_motion_ack, disabled;
static int reached_missing, home_failed, force_home_running, motor_fault;
static unsigned last_pulses, last_direction, power_disabled, saves, homes, home_busy, aborts;
static int32_t saved_zero;
static int32_t arrival_bias;
static char console[2048];
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;memcpy(sent,p,n);sends++;return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
bool ConsoleTx_Write(const uint8_t *p,uint16_t n)
{ size_t used=strlen(console);assert(used+n<sizeof(console));memcpy(console+used,p,n);console[used+n]=0;return true; }
static void step(void)
{
  if(sends!=handled) {
    uint8_t r[8]={sent[0],sent[1],0,0,0,0,0,0x6b};
    unsigned length=4;
    handled=sends;MotorBus_TxCpltCallback(&uart);
    assert(sent[0]==8);
    if(sent[1]==0x36) {
      uint32_t mag=position_raw<0?(uint32_t)-(int64_t)position_raw:(uint32_t)position_raw;
      r[2]=position_raw<0;r[3]=mag>>24;r[4]=mag>>16;r[5]=mag>>8;r[6]=mag;length=8;
    } else if(sent[1]==0x35) {
      r[3]=(uint8_t)((unsigned)speed_rpm>>8);r[4]=(uint8_t)speed_rpm;r[5]=0x6b;length=6;
    } else if(sent[1]==0x3a) { r[2]=moving || reached_missing?0x81:3;if(disabled || power_disabled)r[2]&=(uint8_t)~1U;r[2]|=motor_fault;r[3]=0x6b; }
    else if(sent[1]==0xfd) {
      last_pulses=((unsigned)sent[6]<<24)|((unsigned)sent[7]<<16)|((unsigned)sent[8]<<8)|sent[9];
      last_direction=sent[2];assert(!sent[11]);position_commands++;
      /* This car's mode 2 behaves as an absolute target in repeated moves,
       * as observed on Base and now ID8. Do not model it as a working delta. */
      assert(sent[10]==1 || sent[10]==2);
      if(!hold_position && !reject) position_raw=(int32_t)lround((double)last_pulses*65536/3200)*(last_direction?-1:1)+arrival_bias;
      r[2]=reject?0xee:2;r[3]=0x6b;if(missing_motion_ack)length=0;
    } else if(sent[1]==0xfe) { moving=0;speed_rpm=0;r[2]=2;r[3]=0x6b;stop_commands++; }
    else if(sent[1]==0xf3) { assert(sent[2]==0xab && sent[3]<=1);power_disabled=!sent[3];r[2]=2;r[3]=0x6b; }
    else if(sent[1]==0x93) { assert(sent[2]==0x88 && sent[3]==1);saved_zero=position_raw;saves++;r[2]=2;r[3]=0x6b; }
    else if(sent[1]==0x9a) { assert(sent[2]==0 && !sent[3]);homes++;home_busy=4;r[2]=2;r[3]=0x6b; }
    else if(sent[1]==0x3b) { r[2]=(home_failed?8:0)|((home_busy || force_home_running)?4:0);r[3]=0x6b;if(home_busy && !--home_busy)position_raw=saved_zero+65536; }
    else if(sent[1]==0x9c) { aborts++;home_busy=0;r[2]=2;r[3]=0x6b; }
    else { zero_commands++;assert(0); }
    /* Split every response; address 7 noise cannot consume ID8's event. */
    if(length && !missing) {
      const uint8_t noise[]={7,0x3a,3,0x6b};
      MotorBus_RxBytes(noise,sizeof(noise));
      MotorBus_RxBytes(r,2);MotorBus_Process();
      MotorBus_RxBytes(r+2,(uint16_t)(length-2));
    }
  }
  tick+=10;MotorBus_Process();Turntable_Process();
}
static void run(void)
{
  unsigned i;
  for(i=0;i<2000 && Turntable_IsBusy();i++) step();
  assert(!Turntable_IsBusy());
}
static void command(unsigned n,char *args[])
{ console[0]=0;assert(Turntable_Command(n,args)); }
int main(void)
{
  Turntable_Status_t s;Turntable_Inventory_t inv;
  char *get[]={"turntable","get","slot2_deg"};
  char *set[]={"turntable","set","slot2_deg","270"};
  char *empty[]={"turntable","inventory","empty"};
  assert(MotorBus_Init(&uart)==HAL_OK);Turntable_Init();
  /* The coordinator releases startup once; no early traffic before that gate. */
  for(unsigned i=0;i<10;i++) {
    Turntable_BootHomeProcess(false);step();
    assert(!sends && !homes && !Turntable_IsBusy() && !MotorBus_IsQuarantined());
  }
  Turntable_BootHomeProcess(true);
  assert(Turntable_IsBusy() && !Turntable_ReferenceValid());
  run();Turntable_StatusGet(&s);
  assert(homes==1 && s.reference_valid && s.arrived && s.slot==1);
  assert(strstr(console,"boot_home accepted mode=near"));
  assert(position_raw==saved_zero+65536 && fabsf(s.angle_deg)<0.01f && zero_commands==0 && saves==0);
  assert(Turntable_InventoryGet(1).state==TURNTABLE_UNKNOWN);
  Turntable_BootHomeProcess(true);step();assert(homes==1);
  Turntable_Init();Turntable_Stop();run();Turntable_BootHomeProcess(true);step();assert(homes==1);
  /* Explicit manual actions take ownership; no late boot home may follow. */
  Turntable_Init();assert(Turntable_Origin());run();Turntable_BootHomeProcess(true);step();assert(homes==1);
  homes=0;Turntable_Init();
  assert(!Turntable_ReferenceValid());assert(!Turntable_Index(1));
  inv=Turntable_InventoryGet(1);assert(inv.state==TURNTABLE_UNKNOWN && !inv.color);
  assert(!Turntable_FindEmpty());command(3,get);assert(strstr(console,"value=90"));assert(Turntable_SlotConfigured(3));
  assert(Turntable_Origin());run();Turntable_StatusGet(&s);
  assert(s.reference_valid && s.feedback_valid && !s.moving);
  assert(fabsf(s.angle_deg)<0.01f && zero_commands==0);
  assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(s.arrived && s.slot==2 && fabsf(s.target_deg-90)<.01f && fabsf(s.angle_deg-90)<1);
  assert(Turntable_Index(3));run();Turntable_StatusGet(&s);
  assert(s.arrived && s.slot==3 && fabsf(s.target_deg-180)<.01f && fabsf(s.angle_deg-180)<1);
  assert(Turntable_Index(1));run();
  assert(Turntable_SlotConfigured(2));command(4,set);assert(strstr(console,"OK turntable set"));
  assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(s.arrived && s.slot==2 && fabsf(s.angle_deg-270)<1);
  assert(last_direction==0 && last_pulses==2400); /* 360 -> 270: shortest -90. */
  command(3,empty);assert(Turntable_FindEmpty()==1);
  assert(Turntable_InventorySet(1,TURNTABLE_OCCUPIED,2));assert(Turntable_FindEmpty()==2);
  assert(!Turntable_InventorySet(1,TURNTABLE_OCCUPIED,0));
  for(uint8_t color=1;color<=6;color++) {
    assert(Turntable_InventorySet(1,TURNTABLE_OCCUPIED,color));
    assert(Turntable_InventoryGet(1).color==color);
  }
  assert(!Turntable_InventorySet(1,TURNTABLE_OCCUPIED,7));
  {
    unsigned before_home=homes, before_position=position_commands, before_save=saves;
    /* Slot 1 uses the verified HOME path, independently of position reached bit1. */
    reached_missing=1;console[0]=0;
    assert(Turntable_Index(1));Turntable_StatusGet(&s);
    assert(s.slot==1 && s.target_deg==0 && !s.arrived);
    for(unsigned i=0;i<12;i++)step();
    assert(Turntable_IsBusy()); /* HOME ACK cannot advance the arm to release. */
    run();Turntable_StatusGet(&s);
    assert(homes==before_home+1 && position_commands==before_position && saves==before_save);
    assert(s.arrived && s.feedback_valid && !s.moving && !strcmp(s.reason,"home_complete"));
    assert(fabsf(s.angle_deg)<0.01f && position_raw==saved_zero+65536);
    assert(strstr(console,"OK turntable home complete ref=1"));
    /* Already at slot 1 (about 0.396 degrees away): no zero-distance FD required. */
    position_raw-=72;
    assert(Turntable_Index(1));run();Turntable_StatusGet(&s);
    assert(homes==before_home+2 && position_commands==before_position && s.arrived);
    assert(Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED && Turntable_InventoryGet(1).color==6);
    assert(Turntable_InventoryGet(2).state==TURNTABLE_EMPTY && Turntable_InventoryGet(3).state==TURNTABLE_EMPTY);
    /* A failed HOME may establish an explicitly approximate reference; a stop cancels HOME and keeps inventory/reference. */
    home_failed=1;assert(Turntable_Index(1));run();Turntable_StatusGet(&s);
    assert(s.arrived && s.reference_degraded && !strcmp(s.reason,"home_approximate"));
    assert(strstr(console,"reference=approximate"));home_failed=0;
    unsigned before_abort=aborts;
    assert(Turntable_Index(1));
    for(unsigned i=0;i<12;i++)step();
    Turntable_Stop();run();Turntable_StatusGet(&s);
    assert(aborts==before_abort+1 && !s.arrived && s.reference_valid && !s.moving && s.feedback_valid && !strcmp(s.reason,"stopped"));
    assert(Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED);
    assert(Turntable_Index(1));run();Turntable_StatusGet(&s);assert(s.arrived);
    reached_missing=0;
  }
  /* Failed HOME cannot be accepted while running, moving, disabled or drifting. */
  for(unsigned scenario=0;scenario<4;scenario++) {
    home_failed=1;console[0]=0;
    force_home_running=scenario==0;speed_rpm=scenario==1?1:0;disabled=scenario==2;
    assert(Turntable_Home());
    for(unsigned n=0;n<100;n++) {
      if(scenario==3)position_raw+=1000;
      step();
    }
    Turntable_StatusGet(&s);assert(Turntable_IsBusy() && !s.arrived);
    force_home_running=disabled=0;speed_rpm=0;run();Turntable_StatusGet(&s);
    assert(s.arrived && s.reference_degraded);
    Turntable_Stop();run();Turntable_StatusGet(&s);assert(s.reference_degraded);
    home_failed=0;assert(Turntable_Home());run();Turntable_StatusGet(&s);
    assert(s.arrived && !s.reference_degraded);
  }
  home_failed=1;motor_fault=8;console[0]=0;assert(Turntable_Home());
  for(unsigned n=0;n<50 && !strstr(console,"reason=driver_fault");n++)step();
  assert(strstr(console,"reason=driver_fault"));
  motor_fault=0;run();Turntable_StatusGet(&s);assert(!s.arrived);
  home_failed=0;assert(Turntable_Home());run();
  assert(Turntable_Jog(-45));run();assert(last_direction==0 && last_pulses==2800);
  assert(strstr(console,"OK turntable arrived"));
  {
    char *tolerance[]={"turntable","get","pos_tol_deg"};command(3,tolerance);
    assert(strstr(console,"value=2.5"));
    reached_missing=1; /* Real ID8 report: flags=0x81, no position reached bit. */
    for(int sign=-1;sign<=1;sign+=2){
      console[0]=0;arrival_bias=(int32_t)lround(sign*2.4*65536/360);
      unsigned stopped=stop_commands;int32_t before=position_raw;
      assert(Turntable_Jog(90));run();Turntable_StatusGet(&s);
      assert(s.arrived && !s.moving && s.feedback_valid && stop_commands==stopped && !power_disabled);
      double commanded=(last_direction?-1.0:1.0)*last_pulses*360/3200;
      assert(fabs(commanded-(before*360.0/65536+90))<.057); /* Exact target, not tolerance edge. */
    }
    console[0]=0;arrival_bias=(int32_t)lround(2.6*65536/360);
    assert(Turntable_Jog(90));run();Turntable_StatusGet(&s);
    assert(!s.arrived && !strcmp(s.reason,"position_timeout"));
    assert(strstr(console,"error_deg=") && strstr(console,"flags=0x81") && strstr(console,"budget_ms="));
    arrival_bias=0;console[0]=0;hold_position=1;
    assert(Turntable_Jog(90));for(unsigned i=0;i<350;i++)step();
    assert(Turntable_IsBusy() && !strstr(console,"position_timeout"));
    position_raw=(int32_t)lround((double)last_pulses*65536/3200)*(last_direction?-1:1);
    hold_position=0;run();Turntable_StatusGet(&s);assert(s.arrived);
    reached_missing=0;
  }
  {
    int32_t initial=position_raw;
    console[0]=0;
    assert(Turntable_Jog(-90));run();Turntable_StatusGet(&s);
    assert(s.arrived && abs(position_raw-(initial-16384))<=12);
    assert(Turntable_Jog(-90));run();Turntable_StatusGet(&s);
    assert(s.arrived && abs(position_raw-(initial-32768))<=12);
    assert(Turntable_Jog(180));run();assert(abs(position_raw-initial)<=12);
  }
  Turntable_Stop();Turntable_StatusGet(&s);assert(!s.feedback_valid);
  Turntable_Stop();run();Turntable_StatusGet(&s);
  assert(s.feedback_valid && !s.moving && !strcmp(s.reason,"stopped"));
  assert(s.reference_valid && Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED);
  /* ACK success and position alone cannot finish until speed/state settle. */
  moving=1;speed_rpm=20;assert(Turntable_Index(1));
  for(unsigned i=0;i<35;i++) step();
  assert(Turntable_IsBusy());
  moving=0;speed_rpm=0;run();Turntable_StatusGet(&s);assert(s.arrived);
  /* Loss of enable at target is not arrival, even with a reached flag. */
  disabled=1;assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(!s.arrived && !strcmp(s.reason,"position_timeout"));disabled=0;
  hold_position=1;assert(Turntable_Index(3));run();Turntable_StatusGet(&s);
  assert(!s.arrived && !strcmp(s.reason,"position_timeout"));hold_position=0;
  reject=1;assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(!s.arrived && !strcmp(s.reason,"driver_rejected"));reject=0;
  /* Cancel during read: consume late read reply, then fresh stop confirmation. */
  assert(Turntable_Index(2));step();Turntable_Stop();run();
  assert(!MotorBus_IsQuarantined());
  assert(Turntable_ReferenceValid());
  /* Release ID8 for hand alignment; saving mechanical zero persists in the driver. */
  position_raw=-16384;assert(Turntable_Origin());run();
  assert(Turntable_Jog(-90));run();Turntable_StatusGet(&s);
  assert(s.arrived && position_raw==-32768 && last_direction==1 && last_pulses==1600);
  assert(Turntable_Jog(180));run();Turntable_StatusGet(&s);
  assert(s.arrived && position_raw==0 && last_pulses==0);
  assert(Turntable_Enable(false));run();assert(power_disabled);
  position_raw=12345; /* operator aligned the first slot */
  assert(Turntable_Enable(true));run();assert(!power_disabled);
  assert(Turntable_SaveZero());run();assert(saves==1 && saved_zero==12345);
  position_raw+=20000;
  unsigned before_manual_home=homes;
  assert(Turntable_Home());
  for(unsigned i=0;i<12;i++)step();
  assert(Turntable_IsBusy()); /* ACK alone is not home completion. */
  run();Turntable_StatusGet(&s);
  assert(homes==before_manual_home+1 && s.arrived && s.reference_valid && s.slot==1);
  assert(!strcmp(s.reason,"home_complete") && fabsf(s.angle_deg)<0.01f);
  assert(position_raw==saved_zero+65536 && zero_commands==0);
  assert(Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED);
  unsigned before_manual_abort=aborts;
  assert(Turntable_Home());step();Turntable_Stop();run();assert(aborts==before_manual_abort+1);
  {
    unsigned before=position_commands;
    console[0]=0;assert(Turntable_Jog(-90));
    for(unsigned i=0;i<500;i++) {
      step();
      if(position_commands>before && sends!=handled && sent[1]==0x3a)break;
      assert(i<499);
    }
    uart.ErrorCode=HAL_UART_ERROR_FE;MotorBus_ErrorCallback(&uart);
    MotorBus_Process();Turntable_Process();run();
    assert(strstr(console,"ERR turntable fault reason=transport_error"));
    assert(strstr(console,"bus_reason=uart_error addr=8 func=0x3A uart_error=0x00000004"));
    assert(MotorBus_IsQuarantined() && Turntable_ReferenceValid());
    assert(MotorBus_RecoverAfterReset());console[0]=0;
  }
  {
    assert(Turntable_Index(2));run();
    unsigned moved=position_commands,homed=homes,stopped=stop_commands;
    for(unsigned i=0;i<150;i++)step();
    assert(Turntable_VerifySlot(2));run();Turntable_StatusGet(&s);
    assert(s.arrived && s.feedback_valid && !s.moving);
    assert(position_commands==moved && homes==homed && stop_commands==stopped);
    position_raw+=2000;assert(Turntable_VerifySlot(2));run();Turntable_StatusGet(&s);
    assert(!s.arrived && !strcmp(s.reason,"position_changed"));
    assert(position_commands==moved && homes==homed && stop_commands==stopped);
  }
  missing=1;assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(strstr(console,"ERR turntable fault reason=feedback_timeout"));
  assert(MotorBus_IsQuarantined() && !s.arrived && !s.feedback_valid);
  Turntable_Stop();run();Turntable_StatusGet(&s);
  assert(!strcmp(s.reason,"stopped_unverified") && Turntable_ReferenceValid());
  assert(Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED);
  missing=0;assert(MotorBus_RecoverAfterReset());
  {
    unsigned before_stop=stop_commands;
    /* The motor executes, but its ACK is missing: still send priority Stop. */
    int32_t old_position=position_raw;
    missing_motion_ack=1;assert(Turntable_Index(3));run();Turntable_StatusGet(&s);
    assert(position_raw!=old_position);
    assert(stop_commands==before_stop+1 && !s.arrived && !s.feedback_valid);
    assert(MotorBus_IsQuarantined());missing_motion_ack=0;
  }
  assert(position_commands>=6);
  /* Direct positive half-turn to slot3+90, with no intermediate slot3 command. */
  missing=reject=hold_position=missing_motion_ack=disabled=motor_fault=0;
  reached_missing=moving=speed_rpm=arrival_bias=0;handled=sends;position_raw=0;
  assert(MotorBus_RecoverAfterReset());
  assert(MotorBus_Init(&uart)==HAL_OK);Turntable_Init();assert(Turntable_Origin());run();
  float slot3;assert(Turntable_SlotAngleGet(3,&slot3) && slot3==180);
  position_raw=16300; /* Slightly below 90 must still take the positive half-turn. */
  unsigned before_clearance=position_commands;
  assert(Turntable_AngleStart(slot3+90,0,true));run();
  assert(position_commands==before_clearance+1 && last_pulses==2400 && last_direction==0);
  assert(Turntable_AngleVerify(slot3+90,0));run();assert(position_commands==before_clearance+1);
  assert(Turntable_AngleStart(slot3,3,false));run();assert(last_pulses==1600);
  Turntable_StatusGet(&s);assert(s.slot==3 && s.arrived);
  /* Interrupted outward move retries the saved target, not another relative +180. */
  position_raw=16384;hold_position=1;before_clearance=position_commands;
  assert(Turntable_AngleStart(270,0,true));
  for(unsigned n=0;n<300 && position_commands==before_clearance;n++)step();
  assert(position_commands==before_clearance+1);Turntable_Stop();run();
  position_raw=32768;hold_position=0;assert(Turntable_AngleStart(270,0,true));run();
  assert(last_pulses==2400 && position_raw==49152);
  puts("turntable_test: reference offset, shortest index, shared bus, inventory, arrival and stop OK");
}
