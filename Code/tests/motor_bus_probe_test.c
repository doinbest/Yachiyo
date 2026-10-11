#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static uint8_t frame[32];
static unsigned transmissions;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;assert(n<=sizeof frame);memcpy(frame,p,n);transmissions++;return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{ (void)u;(void)p;(void)n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u) { (void)u;return HAL_OK; }
static void advance(unsigned ms) { tick+=ms;MotorBus_Process(); }
static void tx_done(void) { MotorBus_TxCpltCallback(&uart);MotorBus_Process(); }
static void state_reply(uint8_t id)
{ uint8_t p[]={id,0x3a,3,0x6b};MotorBus_RxBytes(p,sizeof p);MotorBus_Process(); }
static MotorBus_Event_t event(MotorBus_Owner_t owner,MotorBus_Result_t result)
{ MotorBus_Event_t e;assert(MotorBus_EventGet(owner,&e));assert(e.result==result);return e; }
static void start(uint8_t id)
{ assert(MotorBus_ProbeStart(id));advance(101);assert(MotorBus_ProbeReady()); }
static void query(uint8_t id,uint32_t token)
{ uint8_t p[]={id,0x3a,0x6b};assert(MotorBus_Submit(MOTOR_BUS_PROBE,p,3,4,token,false));advance(3);assert(frame[0]==id && frame[1]==0x3a);tx_done(); }
int main(void)
{
  uint8_t read[]={1,0x3a,0x6b},motion[]={1,0xf6,0,0,10,20,0,0x6b};
  MotorBus_Diagnostic_t first,current;
  assert(MotorBus_Init(&uart)==HAL_OK);
  /* A queued poll is preserved, but cannot contaminate the selected session. */
  assert(MotorBus_Submit(MOTOR_BUS_FEEDBACK,read,3,4,1,false));
  start(4);assert(transmissions==0);
  assert(!MotorBus_Submit(MOTOR_BUS_ARM,read,3,4,2,false));
  assert(!MotorBus_Submit(MOTOR_BUS_PROBE,motion,sizeof motion,4,3,false));
  assert(!MotorBus_Submit(MOTOR_BUS_PROBE,read,3,4,4,false));
  query(4,5);state_reply(1);
  MotorBus_Event_t e;assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  advance(2);state_reply(4);e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);
  assert(e.token==5 && e.data[0]==4 && e.tx_completed && e.uart_error==0);
  advance(5);assert(transmissions==1);
  MotorBus_ProbeStop();advance(3);assert(transmissions==2 && frame[0]==1);
  tx_done();state_reply(1);event(MOTOR_BUS_FEEDBACK,MOTOR_BUS_REPLY);
  /* Base diagnostics remain read-only and accept complete configuration frames. */
  start(5);
  uint8_t phase_read[]={5,0x27,0x6b},driver[]={5,0x42,0x6c,0x6b};
  uint8_t phase[]={5,0x27,0x09,0xc4,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_PROBE,phase_read,3,5,30,false));advance(3);tx_done();
  MotorBus_RxBytes(phase,2);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  MotorBus_RxBytes(phase+2,3);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);
  assert(e.length==5 && e.data[2]==9 && e.data[3]==0xc4);
  assert(MotorBus_Submit(MOTOR_BUS_PROBE,driver,4,33,31,false));advance(3);tx_done();
  uint8_t config[33]={5,0x42,0x21,0x15};config[14]=0x0b;config[15]=0xb8;config[32]=0x6b;
  MotorBus_RxBytes(config,16);MotorBus_Process();assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));
  MotorBus_RxBytes(config+16,17);MotorBus_Process();e=event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);
  assert(e.length==33 && !memcmp(e.data,config,33));
  uint8_t write_current[]={5,0x45,0x66,0,0x0b,0xb8,0x6b};
  assert(!MotorBus_Submit(MOTOR_BUS_PROBE,write_current,sizeof write_current,4,32,false));
  MotorBus_ProbeStop();
  /* Never start diagnostic isolation around queued or active motion. */
  assert(MotorBus_Submit(MOTOR_BUS_CHASSIS,motion,sizeof motion,4,6,false));
  assert(!MotorBus_ProbeStart(4));advance(3);tx_done();assert(!MotorBus_ProbeStart(4));
  uart.ErrorCode=HAL_UART_ERROR_FE;MotorBus_ErrorCallback(&uart);MotorBus_Process();
  event(MOTOR_BUS_CHASSIS,MOTOR_BUS_IO_ERROR);MotorBus_DiagnosticGet(&first);
  assert(first.locked && first.uart_error==HAL_UART_ERROR_FE && first.address==1);
  /* A diagnostic can read one healthy ID while motion remains locked. */
  start(4);assert(!MotorBus_RecoverAfterReset());query(4,7);state_reply(4);
  event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);assert(MotorBus_IsQuarantined());
  /* Current packet errors are distinct from the retained first fault. */
  query(4,8);uart.ErrorCode=HAL_UART_ERROR_NE;MotorBus_ErrorCallback(&uart);MotorBus_Process();
  e=event(MOTOR_BUS_PROBE,MOTOR_BUS_IO_ERROR);assert(e.uart_error==HAL_UART_ERROR_NE);
  MotorBus_DiagnosticGet(&current);assert(current.uart_error==first.uart_error && current.tick_ms==first.tick_ms);
  advance(101);assert(MotorBus_ProbeReady());query(4,9);advance(101);
  event(MOTOR_BUS_PROBE,MOTOR_BUS_TIMEOUT);assert(MotorBus_IsQuarantined());
  MotorBus_ProbeStop();start(2);query(2,10);state_reply(4);
  assert(!MotorBus_EventGet(MOTOR_BUS_PROBE,&e));state_reply(2);
  event(MOTOR_BUS_PROBE,MOTOR_BUS_REPLY);assert(MotorBus_IsQuarantined());
  /* Emergency stop preempts reads; ordinary motion cannot use the probe owner. */
  query(2,11);uint8_t stop[]={2,0xfe,0x98,0,0x6b};
  assert(MotorBus_Submit(MOTOR_BUS_STOP,stop,sizeof stop,0,12,true));
  event(MOTOR_BUS_PROBE,MOTOR_BUS_CANCELLED);advance(3);
  assert(frame[0]==2 && frame[1]==0xfe);tx_done();event(MOTOR_BUS_STOP,MOTOR_BUS_TX_DONE);
  uint8_t ack[]={2,0xfe,2,0x6b};MotorBus_RxBytes(ack,sizeof ack);MotorBus_Process();
  state_reply(2);MotorBus_ProbeStop();assert(MotorBus_IsQuarantined());
  puts("motor_bus_probe_test: selected reads, foreign replies, lock retention, current errors, timeout switch and emergency stop OK");
  return 0;
}
