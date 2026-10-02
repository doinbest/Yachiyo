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
static unsigned last_pulses, last_direction;
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
    } else if(sent[1]==0x3a) { r[2]=moving?1:3;if(disabled)r[2]&=(uint8_t)~1U;r[3]=0x6b; }
    else if(sent[1]==0xfd) {
      last_pulses=((unsigned)sent[6]<<24)|((unsigned)sent[7]<<16)|((unsigned)sent[8]<<8)|sent[9];
      last_direction=sent[2];assert(sent[10]==2 && !sent[11]);position_commands++;
      if(!hold_position && !reject) position_raw+=(int32_t)lround((double)last_pulses*65536/3200)*(last_direction?-1:1);
      r[2]=reject?0xee:2;r[3]=0x6b;if(missing_motion_ack)length=0;
    } else if(sent[1]==0xfe) { moving=0;speed_rpm=0;length=0;stop_commands++; }
    else if(sent[1]==0xf3) { assert(sent[2]==0xab && sent[3]==1);r[2]=2;r[3]=0x6b; }
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
  assert(!Turntable_ReferenceValid());assert(!Turntable_Index(1));
  inv=Turntable_InventoryGet(1);assert(inv.state==TURNTABLE_UNKNOWN && !inv.color);
  assert(!Turntable_FindEmpty());command(3,get);assert(strstr(console,"value=unset"));
  assert(Turntable_Origin());run();Turntable_StatusGet(&s);
  assert(s.reference_valid && s.feedback_valid && !s.moving);
  assert(fabsf(s.angle_deg)<0.01f && zero_commands==0);
  assert(!Turntable_Index(2));command(4,set);assert(strstr(console,"OK turntable set"));
  assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(s.arrived && s.slot==2 && fabsf(s.angle_deg-270)<1);
  assert(last_direction==1 && last_pulses==800); /* -90, not +270. */
  command(3,empty);assert(Turntable_FindEmpty()==1);
  assert(Turntable_InventorySet(1,TURNTABLE_OCCUPIED,2));assert(Turntable_FindEmpty()==2);
  assert(!Turntable_InventorySet(1,TURNTABLE_OCCUPIED,0));
  for(uint8_t color=1;color<=6;color++) {
    assert(Turntable_InventorySet(1,TURNTABLE_OCCUPIED,color));
    assert(Turntable_InventoryGet(1).color==color);
  }
  assert(!Turntable_InventorySet(1,TURNTABLE_OCCUPIED,7));
  assert(Turntable_Index(1));run();assert(last_direction==0 && last_pulses==800);
  assert(Turntable_Jog(-45));run();assert(last_direction==1 && last_pulses==400);
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
  hold_position=1;assert(Turntable_Index(1));run();Turntable_StatusGet(&s);
  assert(!s.arrived && !strcmp(s.reason,"position_timeout"));hold_position=0;
  reject=1;assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(!s.arrived && !strcmp(s.reason,"driver_rejected"));reject=0;
  /* Cancel during read: consume late read reply, then fresh stop confirmation. */
  assert(Turntable_Index(2));step();Turntable_Stop();run();
  assert(!MotorBus_IsQuarantined());
  assert(Turntable_ReferenceValid());
  missing=1;assert(Turntable_Index(2));run();Turntable_StatusGet(&s);
  assert(MotorBus_IsQuarantined() && !s.arrived && !s.feedback_valid);
  Turntable_Stop();run();Turntable_StatusGet(&s);
  assert(!strcmp(s.reason,"stopped_unverified") && Turntable_ReferenceValid());
  assert(Turntable_InventoryGet(1).state==TURNTABLE_OCCUPIED);
  missing=0;assert(MotorBus_RecoverAfterReset());
  {
    unsigned before_stop=stop_commands;
    /* The motor executes, but its ACK is missing: still send priority Stop. */
    int32_t old_position=position_raw;
    missing_motion_ack=1;assert(Turntable_Index(1));run();Turntable_StatusGet(&s);
    assert(position_raw!=old_position);
    assert(stop_commands==before_stop+1 && !s.arrived && !s.feedback_valid);
    assert(MotorBus_IsQuarantined());missing_motion_ack=0;
  }
  assert(position_commands>=6);
  puts("turntable_test: reference offset, shortest index, shared bus, inventory, arrival and stop OK");
}
