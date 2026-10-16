#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static UART_HandleTypeDef uart;
static uint32_t tick=1;
static uint8_t tx[40];
static unsigned transmissions;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;memcpy(tx,p,n);transmissions++;return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void reply(const uint8_t *p,unsigned n)
{MotorBus_TxCpltCallback(&uart);MotorBus_RxBytes(p,n);MotorBus_Process();tick+=3;MotorBus_Process();}
int main(void)
{
  MotorBus_Event_t event;
  assert(MotorBus_Init(&uart)==HAL_OK);
  for(uint8_t id=5;id<=8;id++) {
    uint8_t cfg[33]={0},ack[]={id,0x48,2,0x6b};
    cfg[0]=id;cfg[1]=0x42;cfg[2]=33;cfg[3]=21;cfg[22]=3;cfg[32]=0x6b;
    assert(MotorBus_PositionReplyEnsure(id)==0);MotorBus_Process();
    assert(tx[0]==id && tx[1]==0x42);
    reply(cfg,33);assert(tx[1]==0x48 && tx[3]==0 && tx[22]==1);
    reply(ack,4);assert(tx[1]==0x42);
    cfg[22]=1;reply(cfg,33);assert(MotorBus_PositionReplyEnsure(id)==1);
  }
  unsigned count=transmissions;
  assert(MotorBus_PositionReplyEnsureMode(7,1)==1);
  assert(MotorBus_PositionReplyEnsure(7)==1);
  MotorBus_Process();assert(transmissions==count); /* Dynamic/fixed X share mode. */
  uint8_t move[]={7,0xfd,0,0,60,20,0,0,3,32,1,0,0x6b};
  uint8_t reached[]={7,0xfd,0x9f,0x6b},ack[]={7,0xfd,2,0x6b};
  uint32_t accepted=tick;
  assert(MotorBus_Submit(MOTOR_BUS_ARM,move,13,4,7,false));MotorBus_Process();
  reply(reached,4);
  assert(!MotorBus_PositionReached(7,accepted));
  assert(!MotorBus_EventGet(MOTOR_BUS_ARM,&event)); /* Old 9F is neither ACK nor arrival. */
  MotorBus_RxBytes(ack,4);MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event) && event.data[2]==2);
  assert(!MotorBus_IsQuarantined());
  puts("mechanism_receive_test: RAM-only Receive, shared X mode, late 9F ignored PASS");
}
