#include "console_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef uart;
static uint32_t tick;
static uint8_t *data;
static uint16_t size;
static char wire[12000];
static unsigned length;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{assert(u==&uart);data=p;size=n;return HAL_OK;}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u){(void)u;data=NULL;return HAL_OK;}
static void pump(unsigned ms)
{
  while(ms--){tick++;if(data){memcpy(wire+length,data,size);length+=size;wire[length]=0;data=NULL;ConsoleTx_TxCpltCallback(&uart);}ConsoleTx_Process();}
}
int main(void)
{
  ConsoleTx_Stats_t stats;
  ConsoleTx_Init(&uart);
  for(unsigned i=0;i<200;i++){
    assert(ConsoleTx_Debug(CONSOLE_DEBUG_IMU,"old imu\r\n",9));
    assert(ConsoleTx_Debug(CONSOLE_DEBUG_VISION,"old vision\r\n",12));
  }
  assert(ConsoleTx_Debug(CONSOLE_DEBUG_IMU,"latest imu\r\n",12));
  assert(ConsoleTx_Debug(CONSOLE_DEBUG_VISION,"latest vision\r\n",15));
  assert(ConsoleTx_Write((const uint8_t *)"reply\r\n",7));
  assert(ConsoleTx_Urgent("STOP\r\n",6));
  assert(ConsoleTx_Telemetry("map\r\n",5));
  ConsoleTx_Process();pump(1000);
  assert(!strcmp(wire,"STOP\r\nreply\r\nmap\r\nlatest imu\r\nlatest vision\r\n"));
  ConsoleTx_GetStats(&stats);
  assert(stats.reply_peak==7 && stats.reply_dropped==0 && stats.debug_dropped==400);
  assert(stats.bytes_sent==length);
  /* An urgent reply cannot splice an in-flight JSON or a long reply line. */
  char line[302];memset(line,'x',300);line[300]='\n';line[301]=0;
  length=0;wire[0]=0;assert(ConsoleTx_Write((uint8_t *)line,301));
  ConsoleTx_Process();pump(31);assert(ConsoleTx_Urgent("STOP\r\n",6));pump(1000);
  assert(!memcmp(wire,line,301) && !strcmp(wire+301,"STOP\r\n"));
  /* Ordinary backlog does not consume the reserved urgent queue. */
  char backlog[8192];memset(backlog,'b',sizeof(backlog));
  assert(ConsoleTx_Write((uint8_t *)backlog,sizeof(backlog)));
  assert(!ConsoleTx_Write((uint8_t *)"x",1));
  assert(ConsoleTx_Urgent("STOP\r\n",6));
  ConsoleTx_GetStats(&stats);assert(stats.reply_peak==8192 && stats.reply_dropped==1);
  puts("console_priority: latest-only debug, reserved stop queue, frame boundaries and statistics PASS");
}
