/* Prepared for later execution; this delivery does not compile C. */
#include "motor_bus.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick=10;
static unsigned sends, attempts;
static uint8_t sent[40];
static bool fast_tx, busy_tx;
static bool irq_at_clock_read;
static UART_HandleTypeDef uart={.Instance=UART5};
uint32_t HAL_GetTick(void)
{
  uint32_t value=tick;
  if(irq_at_clock_read){
    irq_at_clock_read=false;tick++;MotorBus_TxCpltCallback(&uart);
  }
  return value;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
  attempts++;
  if(busy_tx)return HAL_BUSY;
  assert(n<=sizeof sent);memcpy(sent,p,n);sends++;
  if(fast_tx)MotorBus_TxCpltCallback(u);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void later(unsigned ms){tick+=ms;MotorBus_Process();}
static void tx_done(void){MotorBus_TxCpltCallback(&uart);MotorBus_Process();}
static void receive(const uint8_t *p,unsigned n){MotorBus_RxBytes(p,(uint16_t)n);MotorBus_Process();}
static uint32_t begin_read(void)
{
  const uint8_t q[]={2,0x3a,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,4,5,false));MotorBus_Process();
  assert(sent[0]==2 && sent[1]==0x3a);tick++;tx_done();return tick;
}
static uint32_t arm_move(void)
{
  const uint8_t move[]={7,0xfd,0,0,40,20,0,0,3,32,1,0,0x6b};
  const uint8_t ack[]={7,0xfd,2,0x6b};MotorBus_Event_t e;
  assert(MotorBus_Submit(MOTOR_BUS_ARM,move,sizeof move,4,7,false));MotorBus_Process();
  tick++;tx_done();uint32_t tx=tick;receive(ack,4);
  assert(MotorBus_EventGet(MOTOR_BUS_ARM,&e) && e.result==MOTOR_BUS_REPLY);
  later(3);return tx;
}
static void excluded(const char *which)
{
  uint8_t q[33]={2,0x35,0x6b};unsigned n=3,r=6;
  MotorBus_Owner_t owner=MOTOR_BUS_FEEDBACK;
  if(!strcmp(which,"position")){q[1]=0x36;r=8;}
  else if(!strcmp(which,"home_flags")){q[1]=0x3b;r=4;}
  else if(!strcmp(which,"arm_state")){q[0]=7;q[1]=0x3a;r=4;owner=MOTOR_BUS_ARM;}
  else if(!strcmp(which,"recovery_state")){q[1]=0x3a;r=4;owner=MOTOR_BUS_RECOVERY;}
  else if(!strcmp(which,"motion")){q[1]=0xf6;n=8;r=4;q[2]=0;q[4]=40;q[7]=0x6b;owner=MOTOR_BUS_CHASSIS;}
  else if(!strcmp(which,"fd")){q[0]=7;q[1]=0xfd;n=13;r=4;q[12]=0x6b;owner=MOTOR_BUS_ARM;}
  else if(!strcmp(which,"home")){q[0]=5;q[1]=0x9a;n=5;r=4;q[4]=0x6b;owner=MOTOR_BUS_ARM;}
  else if(!strcmp(which,"config_read")){q[0]=6;q[1]=0x42;q[2]=0x6c;q[3]=0x6b;n=4;r=33;owner=MOTOR_BUS_POSITION_CONFIG;}
  else if(!strcmp(which,"config_write")){q[0]=6;q[1]=0x48;n=33;r=4;q[32]=0x6b;owner=MOTOR_BUS_LEGACY;}
  else assert(!strcmp(which,"speed"));
  assert(MotorBus_Submit(owner,q,(uint8_t)n,(uint8_t)r,10,false));MotorBus_Process();
  tick++;tx_done();later(100);MotorBus_Event_t e;MotorBus_Diagnostic_t d;
  assert(sends==1 && MotorBus_EventGet(owner,&e) && e.result==MOTOR_BUS_TIMEOUT);
  MotorBus_DiagnosticGet(&d);assert(d.locked && !d.read_retry_count && !e.read_retries);
}
int main(int argc,char **argv)
{
  const char *which=argc>1?argv[1]:"async";
  MotorBus_Event_t e;MotorBus_Diagnostic_t d;MotorBus_Timing_t t;
  const uint8_t status[]={2,0x3a,3,0x6b},reached[]={7,0xfd,0x9f,0x6b};
  assert(MotorBus_Init(&uart)==HAL_OK);
  if(!strcmp(which,"clock_irq")){
    const uint8_t q[]={2,0x35,0x6b},p[]={2,0x35,0,0,0,0x6b};
    assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,6,3,false));MotorBus_Process();
    /* ISR observes the next tick after Process has captured its entry clock. */
    irq_at_clock_read=true;MotorBus_Process();
    assert(!MotorBus_IsQuarantined() && !MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e));
    assert(sends==1);receive(p,6);
    assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && e.result==MOTOR_BUS_REPLY && e.tx_ms==tick);
  }else if(!strcmp(which,"split_handover") || !strcmp(which,"stale_partial") || !strcmp(which,"partial_urgent")){
    const uint8_t q[]={2,0x35,0x6b},stop[]={2,0xfe,0x98,0,0x6b};
    assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,6,3,false));
    receive(reached,2);assert(sends==0); /* First half of 07 FD 9F 6B at DMA boundary. */
    if(!strcmp(which,"partial_urgent")){
      assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,1,true));MotorBus_Process();
      assert(sends==1 && sent[1]==0xfe);
    }else{
      later(1);assert(sends==0);
      if(!strcmp(which,"split_handover")){
        receive(reached+2,2);assert(sends==0);later(1);assert(sends==0);
      }
      later(1);assert(sends==1 && sent[1]==0x35 && !MotorBus_IsQuarantined());
    }
  }else if(!strcmp(which,"handover") || !strcmp(which,"stop_handover")){
    const uint8_t q[]={2,0x35,0x6b},stop[]={2,0xfe,0x98,0,0x6b},ack[]={2,0xfe,2,0x6b};
    unsigned before=0;
    if(!strcmp(which,"stop_handover")){
      assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,1,true));MotorBus_Process();
      tick++;tx_done();assert(MotorBus_EventGet(MOTOR_BUS_STOP,&e));before=sends;
      assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,6,3,false));
      later(5);assert(sends==before);receive(ack,4); /* Last urgent ACK releases its drain. */
    }else{
      assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,q,3,6,3,false));
      receive(reached,4);assert(!MotorBus_PositionReached(7,0)); /* Old motion no longer active. */
    }
    assert(sends==before && !MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e));
    later(1);assert(sends==before);later(1);assert(sends==before+1 && sent[1]==0x35);
    assert(!MotorBus_IsQuarantined());
  }else if(!strcmp(which,"async") || !strcmp(which,"late") || !strcmp(which,"stop")){
    uint32_t arm_tx=arm_move();uint32_t first=begin_read();unsigned before=sends;
    tick=first+100;receive(reached,4); /* Reproduce only X's 9F at the query deadline. */
    assert(MotorBus_PositionReached(7,arm_tx) && sends==before);
    assert(!MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && !MotorBus_IsQuarantined());
    MotorBus_DiagnosticGet(&d);
    assert(d.read_retry_count==1 && d.read_retry_address==2 && d.read_retry_rx_bytes==4);
    assert(d.read_retry_sample_length==4 && !memcmp(d.read_retry_sample,reached,4));
    if(!strcmp(which,"stop")){
      const uint8_t stop[]={2,0xfe,0x98,0,0x6b},ack[]={2,0xfe,2,0x6b};
      assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,5,0,9,true));MotorBus_Process();
      assert(sends==before+1 && sent[1]==0xfe); /* No nonexistent retry DMA wait. */
      assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && e.result==MOTOR_BUS_CANCELLED);
      tick++;tx_done();receive(status,4);receive(ack,4);
      assert(MotorBus_EventGet(MOTOR_BUS_STOP,&e) && e.result==MOTOR_BUS_TX_DONE);
      const uint8_t speed[]={2,0x35,0x6b};
      assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,speed,3,6,3,false));
      later(1);assert(sends==before+1);later(1);assert(sends==before+2 && sent[1]==0x35);
      assert(!MotorBus_IsQuarantined());
    }else{
      later(1);assert(sends==before);
      if(!strcmp(which,"late")){
        receive(status,4);assert(!MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e));
        later(1);assert(sends==before); /* Late first reply extends line quiet only. */
      }
      later(1);assert(sends==before+1 && sent[1]==0x3a);
      tick++;tx_done();receive(status,4);
      assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && e.result==MOTOR_BUS_REPLY && e.token==5);
      assert(e.read_retries==1 && e.first_read_tx_ms==first && e.tx_ms>first && e.tx_completed);
      assert(!MotorBus_IsQuarantined() && MotorBus_PositionReached(7,arm_tx));
      MotorBus_TimingGet(&t);assert(t.id[1].requests==2 && t.id[1].timeouts==1 && t.id[1].success==1);
    }
  }else if(!strcmp(which,"normal") || !strcmp(which,"reject")){
    uint32_t first=begin_read();uint8_t p[]={2,0x3a,3,0x6b};
    if(!strcmp(which,"reject"))p[2]=0xe2;
    tick++;receive(p,4);
    assert(sends==1 && MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && e.result==MOTOR_BUS_REPLY);
    assert(!e.read_retries && e.first_read_tx_ms==first && e.tick_ms==tick);
    assert(e.data[2]==p[2] && !MotorBus_IsQuarantined());
  }else if(!strcmp(which,"missing") || !strcmp(which,"fast") || !strcmp(which,"busy")){
    uint32_t first=begin_read();
    fast_tx=!strcmp(which,"fast");busy_tx=!strcmp(which,"busy");
    later(100);assert(!MotorBus_IsQuarantined());
    if(busy_tx){
      assert(sends==1);later(99);assert(!MotorBus_IsQuarantined());later(1);
      assert(sends==1 && attempts>=2);
    }else{
      assert(sends==2);tick++;if(fast_tx)MotorBus_Process();else tx_done();
      if(fast_tx){receive(status,4);assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && e.tx_completed && e.read_retries==1 && e.first_read_tx_ms==first);}
      else {later(99);assert(!MotorBus_IsQuarantined());later(1);}
    }
    if(!fast_tx){
      MotorBus_DiagnosticGet(&d);assert(d.locked && !strcmp(d.reason,"reply_timeout"));
      assert(MotorBus_EventGet(MOTOR_BUS_FEEDBACK,&e) && e.result==MOTOR_BUS_TIMEOUT && e.read_retries==1);
      unsigned before=sends;later(500);assert(sends==before);
    }
  }else excluded(which);
  printf("status retry case %s PASS\n",which);return 0;
}
