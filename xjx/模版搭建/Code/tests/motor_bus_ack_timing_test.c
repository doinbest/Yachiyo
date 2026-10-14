#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t now;
static UART_HandleTypeDef uart;
static uint8_t sent[32];
static unsigned sends;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;memcpy(sent,p,n);sends++;return HAL_OK; }
int main(void)
{
  MotorBus_Event_t e;
  uint8_t cmd[]={1,0xf3,0xab,1,0,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  for(unsigned id=1;id<=8;id++) {
    cmd[0]=(uint8_t)id;
    assert(MotorBus_Submit(MOTOR_BUS_LEGACY,cmd,6,0,id,false));
    MotorBus_Process();assert(sent[0]==id && sends==id);
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();
    assert(!MotorBus_EventGet(MOTOR_BUS_LEGACY,&e)); /* Receive, not TX */
    now+=id%3==0?8:id%3==1?1:2;
    uint8_t p[]={(uint8_t)id,0xf3,2,0x6b};
    MotorBus_RxBytes(p,3);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_LEGACY,&e));
    MotorBus_RxBytes(p+3,1);MotorBus_Process();
    assert(MotorBus_EventGet(MOTOR_BUS_LEGACY,&e) && e.result==MOTOR_BUS_REPLY);
  }
  /* Wrong source/function/tail and motion-complete notifications cannot ACK FF. */
  uint8_t sync[]={0,0xff,0x66,0x6b};
  assert(MotorBus_SubmitReplyTo(MOTOR_BUS_CHASSIS,sync,4,1,4,40,false));
  MotorBus_Process();MotorBus_TxCpltCallback(&uart);MotorBus_Process();
  const uint8_t wrong[]={2,0xff,2,0x6b,1,0xfd,2,0x6b,1,0xff,0x9f,0x6b,1,0xff,2,0};
  MotorBus_RxBytes(wrong,sizeof wrong);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e));
  const uint8_t correct[]={1,0xff,2,0x6b};
  MotorBus_RxBytes(correct,4);MotorBus_Process();assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e));
  for(unsigned fn=0x35;fn<=0x36;fn++) {
    uint8_t query[]={8,(uint8_t)fn,0x6b}, reject[]={8,(uint8_t)fn,0xe2,0x6b};
    assert(MotorBus_Submit(MOTOR_BUS_TURNTABLE,query,3,0,50,false));MotorBus_Process();
    /* Reply can arrive before foreground processes the TX callback. */
    MotorBus_RxBytes(reject,4);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_TURNTABLE,&e));
    MotorBus_TxCpltCallback(&uart);MotorBus_Process();
    assert(MotorBus_EventGet(MOTOR_BUS_TURNTABLE,&e) && e.length==4 && e.data[2]==0xe2);
    assert(!MotorBus_IsQuarantined());
  }
  /* Ordinary FE waits for Receive; urgent FE/9C advance on TX and drain each ACK. */
  uint8_t hold[]={7,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_ARM,hold,5,0,60,false));MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_ARM,&e));
  uint8_t held[]={7,0xfe,2,0x6b};MotorBus_RxBytes(held,4);MotorBus_Process();
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e) && e.result==MOTOR_BUS_REPLY);
  now+=2;hold[0]=5;
  assert(MotorBus_Submit(MOTOR_BUS_STOP,hold,5,0,61,true));MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();assert(MotorBus_EventGet(MOTOR_BUS_STOP,&e) && e.result==MOTOR_BUS_TX_DONE);
  now+=2;hold[0]=6;
  assert(MotorBus_Submit(MOTOR_BUS_STOP,hold,5,0,62,true));MotorBus_Process();
  assert(sent[0]==6); /* Missing first ACK cannot delay second stop. */
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();assert(MotorBus_EventGet(MOTOR_BUS_STOP,&e));
  const uint8_t stops[]={5,0xfe,2,0x6b,6,0xfe,2,0x6b};
  MotorBus_RxBytes(stops,6);MotorBus_Process();MotorBus_RxBytes(stops+6,2);MotorBus_Process();
  MotorBus_Timing_t stats;MotorBus_TimingGet(&stats);
  assert(stats.drained==2 && stats.id[7].rejected==2 && stats.emergency==2);
  uint8_t query[]={7,0x36,0x6b},refused[]={7,0x36,0xee,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_ARM,query,3,8,63,false));MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);MotorBus_Process();MotorBus_Cancel(MOTOR_BUS_ARM);
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e) && e.result==MOTOR_BUS_CANCELLED);
  assert(MotorBus_Submit(MOTOR_BUS_ARM,query,3,8,64,false));
  unsigned previous=sends;MotorBus_Process();assert(sends==previous);
  MotorBus_RxBytes(refused,4);MotorBus_Process();assert(sends==previous+1 && !MotorBus_IsQuarantined());
  puts("motor bus: IDs1..8 Receive and 1/2/8ms immediate advance PASS");
  return 0;
}
