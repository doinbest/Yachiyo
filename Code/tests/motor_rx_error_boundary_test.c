#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_bus.h"
static uint32_t tick;
static UART_HandleTypeDef uart;
static unsigned sends,restarts;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)u;(void)p;(void)n;sends++;return HAL_OK;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{(void)p;(void)n;restarts++;u->ErrorCode=0;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u){(void)u;return HAL_OK;}
static void noise(uint32_t error){uart.ErrorCode=error;MotorBus_ErrorCallback(&uart);}
int main(int argc,char **argv)
{
 const char *mode=argc>1?argv[1]:"ack";
 const uint8_t cmd[]={2,0xf6,0,0,10,0,1,0x6b},ack[]={2,0xf6,2,0x6b};
 MotorBus_Event_t e;bool reject=!strcmp(mode,"partial") || !strcmp(mode,"post_error") ||
   !strcmp(mode,"wrong") || !strcmp(mode,"late") || !strcmp(mode,"bad") || !strcmp(mode,"repeated");
 assert(MotorBus_Init(&uart)==HAL_OK);
 tick=1000;
 if(!strcmp(mode,"ring_wrap")){uint8_t junk[254]={0};MotorBus_RxBytes(junk,sizeof junk);MotorBus_Process();tick+=3;}
 if(!strcmp(mode,"tick_wrap"))tick=UINT32_MAX-30U;
 assert(MotorBus_Submit(MOTOR_BUS_CHASSIS,cmd,sizeof cmd,4,321,false));
 MotorBus_Process();assert(sends==1);MotorBus_TxCpltCallback(&uart);
 if(!strcmp(mode,"missing")) {
   MotorBus_Process();uint32_t sent_ms=tick;
   for(unsigned i=0;i<10;i++){tick+=10;noise(HAL_UART_ERROR_NE);MotorBus_Process();}
   assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e) && e.result==MOTOR_BUS_TIMEOUT);
   MotorBus_Diagnostic_t d;MotorBus_DiagnosticGet(&d);
   assert(tick-sent_ms==100 && sends==1 && d.tx_length==sizeof cmd);
   assert(!memcmp(d.tx_sample,cmd,sizeof cmd) && d.tx_ms==sent_ms);
   puts(mode);return 0;
 }
 if(!strcmp(mode,"cancelled")) {
   MotorBus_Process();MotorBus_RxBytes(ack,4);MotorBus_Cancel(MOTOR_BUS_CHASSIS);
   noise(HAL_UART_ERROR_NE);MotorBus_Process();
   assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e) && e.result==MOTOR_BUS_CANCELLED);
   assert(MotorBus_IsQuarantined() && sends==1);puts(mode);return 0;
 }
 if(!strcmp(mode,"tx_seen"))MotorBus_Process();
 if(!strcmp(mode,"split")){MotorBus_RxBytes(ack,2);MotorBus_Process();MotorBus_RxBytes(ack+2,2);}
 else if(!strcmp(mode,"partial") || !strcmp(mode,"post_error") || !strcmp(mode,"repeated"))MotorBus_RxBytes(ack,2);
 else if(!strcmp(mode,"wrong")){uint8_t other[]={3,0xf6,2,0x6b};MotorBus_RxBytes(other,4);}
 else if(!strcmp(mode,"bad")){uint8_t bad[]={2,0xf6,2,0xeb};MotorBus_RxBytes(bad,4);}
 else {if(!strcmp(mode,"late"))tick+=100;MotorBus_RxBytes(ack,4);}
 if(!strcmp(mode,"partial")) {noise(HAL_UART_ERROR_NE);MotorBus_RxBytes(ack+2,2);}
 else if(!strcmp(mode,"post_error") || !strcmp(mode,"repeated")) {
   noise(HAL_UART_ERROR_NE);MotorBus_RxBytes(ack,4);
   if(!strcmp(mode,"repeated"))noise(HAL_UART_ERROR_FE);
 }else noise(!strcmp(mode,"ore")?HAL_UART_ERROR_ORE:!strcmp(mode,"fe")?HAL_UART_ERROR_FE:HAL_UART_ERROR_NE);
 if(!strcmp(mode,"fatal_later"))noise(HAL_UART_ERROR_DMA);
 MotorBus_Process();
 if(!strcmp(mode,"fatal_later")){
   assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e) && e.result==MOTOR_BUS_IO_ERROR);
   assert(MotorBus_IsQuarantined() && (e.uart_error&HAL_UART_ERROR_DMA) && sends==1);
   puts(mode);return 0;
 }
 if(reject && strcmp(mode,"late")){
  assert(!MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e));
  tick+=2;MotorBus_RxBytes(ack,4);MotorBus_Process();
 }
 assert(MotorBus_EventGet(MOTOR_BUS_CHASSIS,&e));
 if(!strcmp(mode,"late"))assert(e.result==MOTOR_BUS_TIMEOUT && MotorBus_IsQuarantined());
 else assert(e.result==MOTOR_BUS_REPLY && e.token==321 && e.tx_completed && !MotorBus_IsQuarantined());
 assert(sends==1 && restarts==2);
 MotorBus_Diagnostic_t d;MotorBus_DiagnosticGet(&d);
 assert(d.rx_preserved_replies==(reject?0U:1U));
 puts(mode);return 0;
}
