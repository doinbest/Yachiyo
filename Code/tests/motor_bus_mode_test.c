/* Actual bus/parameter state machines, with a virtual UART and stationary drives. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick=1;
static UART_HandleTypeDef uart;
static uint8_t wire[40],wire_size,mode[4],initial[4];
static bool pending,polling,drop_config;
static unsigned writes,queries,feedback_index;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;assert(!pending);memcpy(wire,p,n);wire_size=n;pending=true;return HAL_OK;}
#include "../template/Hardware/motor_bus.c"
#ifndef MOTOR_BUS_MODE_TEST_DIAG
/* Before implementation, demonstrate the missing entry point as an assertion. */
static bool MotorBus_ModeTestStart(uint8_t mode_value,uint16_t gap)
{(void)mode_value;(void)gap;return false;}
#endif
static void service(void)
{
  if(!pending)return;
  uint8_t r[40]={wire[0],wire[1]};unsigned n=0,id=wire[0]-1;
  assert(id<4);
  if(wire[1]==0x42){
    assert(wire_size==4 && wire[2]==0x6c);n=33;r[2]=33;r[3]=21;
    for(unsigned i=4;i<32;i++)r[i]=(uint8_t)(i+id);
    r[22]=mode[id];if(drop_config)n=0;
  }else if(wire[1]==0x48){
    assert(wire_size==33 && wire[2]==0xd1 && wire[3]==0 && wire[22]<=1);
    for(unsigned i=4;i<32;i++)if(i!=22)assert(wire[i]==(uint8_t)(i+id));
    mode[id]=wire[22];writes++;r[2]=2;n=4;
  }else if(wire[1]==0x35 || wire[1]==0x36 || wire[1]==0x3a){
    n=wire[1]==0x35?6:wire[1]==0x36?8:4;if(wire[1]==0x3a)r[2]=3;queries++;
  }else {assert(wire[1]==0xfe);n=4;r[2]=2;}
  if(n)r[n-1]=0x6b;
  pending=false;MotorBus_TxCpltCallback(&uart);if(n)MotorBus_RxBytes(r,n);
}
#ifdef MOTOR_BUS_MODE_TEST_DIAG
static void step(void)
{
  tick++;service();MotorBus_Process();MotorBus_ModeTestProcess();
  MotorBus_Event_t e;(void)MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e);
  if(polling && !MotorBus_OwnerBusy(MOTOR_BUS_FEEDBACK)){
    static const uint8_t fn[]={0x35,0x36,0x3a};
    uint8_t q[]={(uint8_t)(feedback_index%4+1),fn[(feedback_index/4)%3],0x6b};
    if(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,q[1]==0x35?6:q[1]==0x36?8:4,++feedback_index,false)){}
  }
}
#endif
int main(int argc,char **argv)
{
  (void)argv;assert(MotorBus_Init(&uart)==HAL_OK);
  for(unsigned i=0;i<4;i++)mode[i]=initial[i]=(uint8_t)(i&1U);
  uint16_t gap=(uint16_t)(argc>2?atoi(argv[2]):3);
  assert(MotorBus_ModeTestStart(1,gap));
#ifdef MOTOR_BUS_MODE_TEST_DIAG
  MotorBus_ModeTestStatus_t s;uint8_t move[]={1,0xf6,0,0,0,0,0,0x6b};
  assert(!MotorBus_Submit(MOTOR_BUS_CHASSIS,move,8,0,1,false));
  assert(!MotorBus_ModeTestStart(0,5));
  polling=argc>1 && !strcmp(argv[1],"poll");
  assert(!MotorBus_ProbeStart(1) && !MotorBus_RecoveryStart());
  for(unsigned i=0;i<6000 && MotorBus_ModeTestActive();i++)step();
  MotorBus_ModeTestGet(&s);assert(!s.active && s.success && s.restored);
  assert(writes==4 && s.trace_sequence>=12 && MotorBus_ReplySettleMs()==2);
  for(unsigned i=0;i<4;i++)assert(mode[i]==initial[i]);
  if(polling)assert(queries>20);
  /* A stop cancels unsent test writes; restoration yields until stop proof ends. */
  assert(MotorBus_ModeTestStart(0,5));
  for(unsigned i=0;i<300 && writes==4;i++)step();
  position_config_stop_hold=true;MotorBus_ModeTestCancel();
  uint8_t stop[]={1,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,99,true));
  unsigned old_writes=writes;for(unsigned i=0;i<100;i++)step();
  assert(writes==old_writes);position_config_stop_hold=false;
  for(unsigned i=0;i<4000 && MotorBus_ModeTestActive();i++)step();
  MotorBus_ModeTestGet(&s);assert(!s.active && s.cancelled && s.restored && !s.success);
  for(unsigned i=0;i<4;i++)assert(mode[i]==initial[i]);
  assert(MotorBus_ReplySettleMs()==2);
  /* A missing reply keeps the real fault; no motion or configuration replay. */
  assert(MotorBus_ModeTestStart(1,2));drop_config=true;
  for(unsigned i=0;i<500 && MotorBus_ModeTestActive();i++)step();
  MotorBus_ModeTestGet(&s);assert(!s.active && !s.success && !s.restored && MotorBus_IsQuarantined());
  assert(MotorBus_ReplySettleMs()==2);old_writes=writes;
  for(unsigned i=0;i<250;i++)step();
  assert(writes==old_writes);
  puts("motor_bus_mode_test: RAM-only switch/restore, polling, cancel, fault and no-motion gate PASS");
#else
  (void)argc;(void)service;(void)polling;(void)feedback_index;(void)queries;
#endif
}
