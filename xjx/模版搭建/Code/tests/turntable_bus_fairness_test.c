#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "turntable.h"
#include "motor_bus.h"

/* Real MotorBus scheduler and wire replies, with the existing arm 25ms and
 * wheel 16ms query cadence. A 1 RPM / 90 degree index takes 15 seconds. */
static uint32_t tick=1, last_move_tick;
static UART_HandleTypeDef uart;
static uint8_t sent[32];
static unsigned sends,handled,arm_replies,wheel_replies;
static double actual_deg=90.0,target_deg=90.0;
static unsigned motor_rpm;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;memcpy(sent,p,n);sends++;return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
bool ConsoleTx_Write(const uint8_t *p,uint16_t n) { (void)p;(void)n;return true; }
static void step(void)
{
  double distance=fabs(target_deg-actual_deg);
  double move=(double)(tick-last_move_tick)*motor_rpm*6.0/1000.0;
  if(move>distance)move=distance;
  actual_deg+=target_deg<actual_deg?-move:move;
  last_move_tick=tick;
  if(sends!=handled) {
    uint8_t r[8]={sent[0],sent[1],0,0,0,0,0,0x6b};
    unsigned length=4;
    bool moving=fabs(actual_deg-target_deg)>0.001;
    handled=sends;MotorBus_TxCpltCallback(&uart);
    if(sent[1]==0x36) {
      int64_t raw=sent[0]==8?(int64_t)llround(actual_deg*65536.0/360.0):0;
      uint32_t mag=raw<0?(uint32_t)-raw:(uint32_t)raw;
      r[2]=raw<0;r[3]=mag>>24;r[4]=mag>>16;r[5]=mag>>8;r[6]=mag;length=8;
    } else if(sent[1]==0x35) {
      r[4]=sent[0]==8 && moving?(uint8_t)motor_rpm:0;r[5]=0x6b;length=6;
    } else if(sent[1]==0x3a) { r[2]=sent[0]==8 && moving?1:3;r[3]=0x6b; }
    else if(sent[1]==0xfd) {
      unsigned pulses=((unsigned)sent[6]<<24)|((unsigned)sent[7]<<16)|((unsigned)sent[8]<<8)|sent[9];
      assert(sent[0]==8 && sent[10]==2);
      motor_rpm=((unsigned)sent[3]<<8)|sent[4];
      target_deg=actual_deg+(sent[2]?-1.0:1.0)*pulses*360.0/3200.0;
      r[2]=2;r[3]=0x6b;
    } else if(sent[1]==0xf3) { assert(sent[0]==8);r[2]=2;r[3]=0x6b; }
    else if(sent[1]==0xfe) { target_deg=actual_deg;length=0; }
    else assert(0);
    if(length)MotorBus_RxBytes(r,(uint16_t)length);
  }
  tick++;MotorBus_Process();Turntable_Process();
}
int main(void)
{
  Turntable_Status_t status;
  uint32_t arm_time[6]={0},wheel_time[12]={0},arm_last=0,wheel_last=0,started;
  unsigned arm_index=0,wheel_index=0;
  char *speed[]={"turntable","set","speed_rpm","1"};
  char *slot[]={"turntable","set","slot2_deg","270"};
  static const uint8_t arm_address[]={7,6,5},functions[]={0x35,0x36,0x3a};
  assert(MotorBus_Init(&uart)==HAL_OK);Turntable_Init();
  assert(Turntable_Origin());
  for(unsigned i=0;i<1000 && Turntable_IsBusy();i++)step();
  assert(!Turntable_IsBusy() && Turntable_ReferenceValid());
  assert(Turntable_Command(4,speed) && Turntable_Command(4,slot));
  assert(Turntable_Index(2));started=tick;
  while(Turntable_IsBusy() && tick-started<18000U) {
    MotorBus_Event_t e;
    if(MotorBus_EventGet(MOTOR_BUS_ARM,&e)) {
      assert(e.result==MOTOR_BUS_REPLY && e.data[0]>=5 && e.data[0]<=7);
      arm_time[e.token%6]=tick;arm_replies++;
    }
    if(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e)) {
      assert(e.result==MOTOR_BUS_REPLY && e.data[0]>=1 && e.data[0]<=4);
      wheel_time[e.token%12]=tick;wheel_replies++;
    }
    if(!MotorBus_OwnerBusy(MOTOR_BUS_ARM) && tick-arm_last>=25U) {
      uint8_t query[]={arm_address[arm_index/2],arm_index%2?0x3a:0x36,0x6b};
      assert(MotorBus_Submit(MOTOR_BUS_ARM,query,3,arm_index%2?4:8,arm_index,false));
      arm_last=tick;arm_index=(arm_index+1)%6;
    }
    if(!MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK) && tick-wheel_last>=16U) {
      uint8_t query[]={(uint8_t)(wheel_index%4+1),functions[wheel_index/4],0x6b};
      const uint8_t lengths[]={6,8,4};
      assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,query,3,lengths[wheel_index/4],wheel_index,false));
      wheel_last=tick;wheel_index=(wheel_index+1)%12;
    }
    step();
    /* Same 2000ms freshness budget used by GrabTask; no exemptions for indexing. */
    for(unsigned i=0;i<6;i++) assert(tick-(arm_time[i]?arm_time[i]:started)<2000U);
    for(unsigned i=0;i<12;i++) assert(tick-(wheel_time[i]?wheel_time[i]:started)<2000U);
  }
  Turntable_StatusGet(&status);
  if (Turntable_IsBusy() || !status.arrived || !status.feedback_valid)
    fprintf(stderr,"index result %s / %s, elapsed=%lu, actual=%.3f target=%.3f, arm=%u wheel=%u\n",
            status.state,status.reason,(unsigned long)(tick-started),actual_deg,target_deg,arm_replies,wheel_replies);
  assert(tick-started>14000U && !Turntable_IsBusy() && status.arrived && status.feedback_valid);
  assert(arm_replies>100 && wheel_replies>100);
  printf("turntable_bus_fairness_test: 15s index, %u arm and %u wheel replies within 2s freshness OK\n",arm_replies,wheel_replies);
}
