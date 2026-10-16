#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick=100;
static UART_HandleTypeDef uart;
static uint8_t frame[40],cfg[33];
static unsigned writes;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;memcpy(frame,p,n);if(p[1]==0x48)writes++;return HAL_OK;}
static void later(unsigned dt){tick+=dt;MotorBus_Process();}
static void received(const uint8_t *p,unsigned n){MotorBus_RxBytes(p,(uint16_t)n);MotorBus_Process();}
static void done(void){MotorBus_TxCpltCallback(&uart);MotorBus_Process();}
int main(int argc,char **argv)
{
  bool mismatch=argc>1 && !strcmp(argv[1],"mismatch"),missing=argc>1 && !strcmp(argv[1],"missing");
  cfg[0]=1;cfg[1]=0x42;cfg[2]=33;cfg[3]=21;cfg[22]=1;cfg[32]=0x6b;
  for(unsigned i=4;i<32;i++)if(i!=22)cfg[i]=(uint8_t)(i+7);
  assert(MotorBus_Init(&uart)==HAL_OK);
  assert(MotorBus_WheelReplyEnsure(1,false)==0);MotorBus_Process();assert(frame[1]==0x42);
  done();received(cfg,33);later(3);assert(frame[1]==0x48 && frame[22]==0 && frame[3]==0);
  const uint8_t ack[]={1,0x48,2,0x6b};done();received(ack,sizeof ack);
  later(3);assert(frame[1]==0x42); /* None is applied; full verification is pending. */
  MotorBus_PositionConfigStopHold(true);
  const uint8_t stop[]={1,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,sizeof stop,0,8,true));
  assert(!MotorBus_IsQuarantined());done();later(3);assert(frame[1]==0xfe);done();
  MotorBus_Event_t event;assert(MotorBus_EventGet(MOTOR_BUS_STOP,&event) && event.result==MOTOR_BUS_TX_DONE);
  cfg[22]=0;if(!missing)received(cfg,33); /* Drain the cancelled verification. */
  later(101);
  if(missing){
    MotorBus_Diagnostic_t fault;MotorBus_DiagnosticGet(&fault);
    assert(fault.locked && !strcmp(fault.reason,"drain_timeout"));
  }else{
    assert(!MotorBus_IsQuarantined() && writes==1 && MotorBus_WheelReplyEnsure(1,false)==0);
    MotorBus_PositionConfigStopHold(false);later(3);assert(frame[1]==0x42);
    done();if(mismatch)cfg[8]^=1;received(cfg,33);
    assert(writes==1 && MotorBus_WheelReplyEnsure(1,false)==(mismatch?-1:1));
  }
  puts(missing?"wheel None stop: cancelled query missing remains a real fault PASS":
       mismatch?"wheel None stop: preserved full snapshot rejects mismatch PASS":
       "wheel None stop: no phantom FE ACK, drain and read-only verification resume PASS");
}
