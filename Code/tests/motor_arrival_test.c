#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick=1;
static UART_HandleTypeDef uart;
static uint8_t frame[40];
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;memcpy(frame,p,n);return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void reply(const uint8_t *p,unsigned n)
{MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(p,(uint16_t)n);MotorBus_Process();tick+=3;MotorBus_Process();}
int main(void)
{
  MotorBus_Event_t event;
  uint8_t cfg[33]={5,0x42,33,21},ack[]={5,0x48,2,0x6b};
  for(unsigned i=4;i<32;i++)cfg[i]=(uint8_t)i;
  cfg[22]=1;cfg[32]=0x6b;
  assert(MotorBus_Init(&uart)==HAL_OK);
  assert(MotorBus_PositionReplyEnsure(5)==0);MotorBus_Process();
  assert(frame[0]==5 && frame[1]==0x42);
  reply(cfg,33);assert(frame[1]==0x48 && frame[3]==0 && frame[22]==3);
  for(unsigned i=4;i<32;i++)if(i!=22)assert(frame[i]==cfg[i]);
  reply(ack,4);assert(frame[1]==0x42);
  cfg[22]=3;reply(cfg,33);assert(MotorBus_PositionReplyEnsure(5)==1);
  uint32_t accepted=tick;
  uint8_t move[]={5,0xfd,0,0,60,20,0,0,3,32,1,0,0x6b};
  uint8_t received[]={5,0xfd,2,0x6b},reached[]={5,0xfd,0x9f,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_ARM,move,13,4,7,false));MotorBus_Process();
  reply(received,4);assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event));
  assert(!MotorBus_PositionReached(5,accepted)); /* Receive is not completion. */
  uint8_t query[]={6,0x36,0x6b},position[]={6,0x36,0,0,0,0,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,query,3,8,8,false));MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(reached,4);MotorBus_Process();
  assert(MotorBus_PositionReached(5,accepted));
  assert(!MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&event)); /* Async 9F cannot consume a query. */
  MotorBus_RxBytes(position,8);MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&event) && event.length==8);
  tick+=3;accepted=tick;
  assert(MotorBus_Submit(MOTOR_BUS_ARM,move,13,4,9,false));MotorBus_Process();
  assert(!MotorBus_PositionReached(5,accepted)); /* New motion invalidates old completion. */
  reply(reached,4);assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event) && event.data[2]==0x9f);
  uint8_t stop[]={5,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,10,true));tick+=3;MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  MotorBus_RxBytes(reached,4);MotorBus_Process();
  assert(!MotorBus_PositionReached(5,accepted)); /* A stopped command cannot finish late. */
  assert(MotorBus_EventGet(MOTOR_BUS_STOP,&event));
  uint8_t stop_ack[]={5,0xfe,2,0x6b};MotorBus_RxBytes(stop_ack,4);MotorBus_Process();tick+=3;
  /* Rejected/mismatching configuration must not grant movement readiness. */
  assert(MotorBus_PositionReplyEnsure(6)==0);MotorBus_Process();
  cfg[0]=6;cfg[22]=1;ack[0]=6;
  reply(cfg,33);assert(frame[1]==0x48 && frame[22]==3);
  reply(ack,4);assert(frame[1]==0x42);
  reply(cfg,33);assert(MotorBus_PositionReplyEnsure(6)==-1);
  puts("motor_arrival_test: PASS");
}
