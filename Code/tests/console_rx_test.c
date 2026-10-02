#include "console_rx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef uart;
static uint8_t *buffer;
static uint16_t capacity;
static uint32_t tick;
static HAL_StatusTypeDef start_result;
static unsigned starts, aborts, bytes, faults, stops;
static uint8_t received[1000];
static DMA_HandleTypeDef dma;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{assert(u==&uart);starts++;buffer=p;capacity=n;dma.counter=n;return start_result;}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u){assert(u==&uart);aborts++;return HAL_OK;}
void ArmConsole_ReceiveData(uint8_t b){received[bytes++]=b;if(b==3)stops++;}
void ArmConsole_ReceiveFault(void){faults++;}
int main(void)
{
  ConsoleRx_Stats_t stats;
  uart.Instance=USART1;uart.hdmarx=&dma;
  assert(ConsoleRx_Init(&uart)==HAL_OK && capacity==128);
  /* Circular HAL may deliver IDLE at NDTR==Size before any new DMA data. */
  ConsoleRx_RxEventCallback(&uart);assert(bytes==0);
  memset(buffer,'a',capacity);buffer[2]=3;
  dma.counter=capacity-5;ConsoleRx_RxEventCallback(&uart);
  assert(bytes==5 && stops==1);
  ConsoleRx_RxEventCallback(&uart);assert(bytes==5);
  dma.counter=capacity/2;ConsoleRx_RxEventCallback(&uart);assert(bytes==64);
  /* IDLE before a delayed HT callback: live position prevents duplicate/wrapped data. */
  dma.counter=capacity-90;ConsoleRx_RxEventCallback(&uart);ConsoleRx_RxEventCallback(&uart);assert(bytes==90);
  dma.counter=capacity;ConsoleRx_RxEventCallback(&uart);assert(bytes==128);
  ConsoleRx_RxEventCallback(&uart);assert(bytes==128);
  buffer[0]='z';dma.counter=capacity-1;ConsoleRx_RxEventCallback(&uart);assert(bytes==129 && received[128]=='z');
  assert(starts==1); /* circular IDLE callbacks never restart DMA */
  uart.ErrorCode=HAL_UART_ERROR_ORE|HAL_UART_ERROR_FE;
  ConsoleRx_ErrorCallback(&uart);assert(faults==1);
  dma.counter=capacity-2;ConsoleRx_RxEventCallback(&uart);assert(bytes==129);
  start_result=HAL_ERROR;ConsoleRx_Process();assert(aborts==1 && starts==2);
  ConsoleRx_Process();assert(starts==2);
  start_result=HAL_OK;tick=100;ConsoleRx_Process();assert(starts==3);
  /* Stale commands left in DMA RAM must never be replayed after recovery. */
  ConsoleRx_RxEventCallback(&uart);assert(bytes==129);
  buffer[0]=3;dma.counter=capacity-1;ConsoleRx_RxEventCallback(&uart);assert(stops==2);
  ConsoleRx_GetStats(&stats);
  assert(stats.bytes_received==130 && stats.uart_errors==1 && stats.overruns==1 && stats.framing_errors==1);
  assert(stats.restarts==1 && stats.restart_failures==1);
  puts("console_rx: circular HT/TC/IDLE, duplicate events, Ctrl+C and RX-only recovery PASS");
}
