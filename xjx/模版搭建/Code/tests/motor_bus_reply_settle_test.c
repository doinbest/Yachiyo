#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
#define MOTOR_BUS_REPLY_SETTLE_MS 2U
static uint32_t tick;
static unsigned sends;
static UART_HandleTypeDef uart;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;sends++;return HAL_OK; }
#include "../template/Hardware/motor_bus.c"
int main(void)
{
  MotorBus_Event_t event;uint8_t read[]={1,0x3a,0x6b},reply[]={1,0x3a,3,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,read,3,4,1,false));MotorBus_Process();assert(sends==1);
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  read[0]=2;assert(MotorBus_Submit(MOTOR_BUS_ARM,read,3,4,2,false));
  tick=1;MotorBus_RxBytes(reply,3);MotorBus_Process();assert(sends==1);
  tick=2;MotorBus_RxBytes(reply+3,1);MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&event) && event.result==MOTOR_BUS_REPLY);
  assert(sends==1);tick=3;MotorBus_Process();assert(sends==1);
  tick=4;MotorBus_Process();assert(sends==2);
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  reply[0]=2;tick=6;MotorBus_RxBytes(reply,4);MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&event));
  /* Urgent delivery keeps its existing TX spacing and does not await this gap. */
  uint8_t stop[]={5,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,3,true));MotorBus_Process();assert(sends==3);
  assert(!MotorBus_IsQuarantined());
  puts("motor_bus_reply_settle_test: PASS (complete reply, 2ms handover, urgent bypass)");
}
