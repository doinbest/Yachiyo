/* Prepared for later host execution. No compilation in this delivery. */
#include "motor_bus.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static unsigned sends;
static uint8_t sent[40];
static bool alter_params;
static UART_HandleTypeDef uart={.Instance=UART5};
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;assert(n<=sizeof sent);memcpy(sent,p,n);sends++;return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void later(unsigned ms){tick+=ms;MotorBus_Process();}
static void tx_done(void){MotorBus_TxCpltCallback(&uart);MotorBus_Process();}
static void receive(const uint8_t *p,unsigned n){MotorBus_RxBytes(p,(uint16_t)n);MotorBus_Process();}
static void config_reply(unsigned mode)
{
  uint8_t p[33]={6,0x42,33,21};
  for(unsigned i=4;i<32;i++)p[i]=(uint8_t)i;
  if(alter_params)p[8]^=1;
  p[22]=(uint8_t)mode;p[32]=0x6b;
  unsigned before=sends;receive(p,17);assert(sends==before);
  receive(p+17,sizeof p-17); /* Partial replies cannot release the drain early. */
}
static void begin_config(void)
{
  assert(MotorBus_PositionReplyEnsure(6)==0);MotorBus_Process();
  assert(sends==1 && sent[0]==6 && sent[1]==0x42);
}
static void cancel_read(bool after_tx,bool missing)
{
  unsigned before=sends;
  MotorBus_Event_t event;const uint8_t stop[]={6,0xfe,0x98,0,0x6b};
  const uint8_t speed[]={1,0x35,0x6b},stop_ack[]={6,0xfe,2,0x6b};
  if(after_tx)tx_done();
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,sizeof stop,0,1,true));
  assert(!MotorBus_IsQuarantined());
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,speed,sizeof speed,6,77,false));
  if(!after_tx){tick++;tx_done();}
  later(3);assert(sends==before+1 && sent[1]==0xfe);
  tick++;tx_done();
  if(!missing)config_reply(3); /* Old cancelled read is drained, not verified. */
  assert(sends==before+1);receive(stop_ack,sizeof stop_ack);
  if(missing){
    later(101);MotorBus_Diagnostic_t fault;MotorBus_DiagnosticGet(&fault);
    assert(fault.locked && !strcmp(fault.reason,"drain_timeout") && fault.address==6 && fault.function==0x42);
    assert(MotorBus_TransientFault(&fault) && sends==before+1);return;
  }
  assert(sends==before+1);later(1);assert(sends==before+1);
  later(1);assert(!MotorBus_IsQuarantined() && sends==before+2 && sent[1]==0x35);
  tx_done();const uint8_t measured[]={1,0x35,0,0,0,0x6b};receive(measured,sizeof measured);
  assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&event) && event.result==MOTOR_BUS_REPLY && event.token==77);
  assert(MotorBus_PositionReplyEnsure(6)==0); /* A new action must prepare again. */
  later(3);assert(sends==before+3 && sent[1]==0x42);
}
int main(int argc,char **argv)
{
  const char *which=argc>1?argv[1]:"before_tx";
  assert(MotorBus_Init(&uart)==HAL_OK);
  if(!strcmp(which,"hold_queued")){
    const uint8_t q[]={1,0x35,0x6b},r[]={1,0x35,0,0,0,0x6b};
    assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,6,11,false));
    assert(MotorBus_PositionReplyEnsure(6)==0);MotorBus_Process();assert(sent[1]==0x35);
    MotorBus_PositionConfigStopHold(true);MotorBus_Process();tx_done();receive(r,sizeof r);
    later(3);assert(sends==1 && !MotorBus_IsQuarantined());
    MotorBus_PositionConfigStopHold(false);later(3);assert(sends==1);
    assert(MotorBus_PositionReplyEnsure(6)==0);later(3);assert(sent[1]==0x42 && sends==2);
  }else if(!strcmp(which,"recovery")){
    assert(MotorBus_PositionReplyEnsure(6)==0);assert(MotorBus_RecoveryStart());
    later(501);assert(sends==1 && sent[1]==0xfe);
  }else{
    begin_config();
    if(!strcmp(which,"before_tx") || !strcmp(which,"after_tx") || !strcmp(which,"missing"))
      cancel_read(!strcmp(which,"after_tx"),!strcmp(which,"missing"));
    else if(!strcmp(which,"verify")){
      tx_done();config_reply(1);later(3);assert(sent[1]==0x48);
      tx_done();const uint8_t ack[]={6,0x48,2,0x6b};receive(ack,sizeof ack);
      later(3);assert(sent[1]==0x42);cancel_read(false,false);
      tx_done();alter_params=true;config_reply(3);
      assert(MotorBus_PositionReplyEnsure(6)==-1); /* Full snapshot must survive cancellation. */
    }else if(!strcmp(which,"write")){
      tx_done();config_reply(1);later(3);assert(sent[1]==0x48);
      const uint8_t stop[]={6,0xfe,0x98,0,0x6b};
      assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,sizeof stop,0,1,true));
      MotorBus_Diagnostic_t fault;MotorBus_DiagnosticGet(&fault);
      assert(fault.locked && !strcmp(fault.reason,"cancelled_reply") && fault.function==0x48);
      assert(!MotorBus_TransientFault(&fault));
    }else if(!strcmp(which,"hold_active")){
      MotorBus_PositionConfigStopHold(true);tx_done();config_reply(1);later(20);
      assert(sends==1 && !MotorBus_IsQuarantined());
      MotorBus_PositionConfigStopHold(false);later(3);assert(sent[1]==0x48 && sends==2);
      tx_done();const uint8_t ack[]={6,0x48,2,0x6b};receive(ack,sizeof ack);
      later(3);assert(sent[1]==0x42 && sends==3);
      tx_done();config_reply(3);assert(MotorBus_PositionReplyEnsure(6)==1);
    }else assert(!"unknown case");
  }
  printf("config/stop case %s PASS\n",which);return 0;
}
