#include "console_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static uint8_t *active;
static uint16_t length;
static HAL_StatusTypeDef next_status;
static UART_HandleTypeDef uart;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{ assert(u == &uart); if(next_status != HAL_OK) return next_status; active=p;length=n;return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u) { (void)u;active=NULL;return HAL_OK; }
static void pump(void) { tick+=30;ConsoleTx_Process(); }
#define ConsoleTx_Process pump
static void finish(void) { ConsoleTx_TxCpltCallback(&uart); ConsoleTx_Process(); }
int main(void)
{
  char reply[]="reply";
  ConsoleTx_Init(&uart);
  assert(ConsoleTx_Telemetry("old",3));
  assert(ConsoleTx_Telemetry("new",3));
  assert(ConsoleTx_Write((uint8_t*)reply,5)); reply[0]='X';
  ConsoleTx_Process(); assert(length==5 && !memcmp(active,"reply",5));
  assert(ConsoleTx_Dropped()==1); finish();
  assert(length==3 && !memcmp(active,"new",3));
  assert(ConsoleTx_Telemetry("next",4)); assert(!memcmp(active,"new",3));
  finish(); assert(length==4 && !memcmp(active,"next",4)); finish();
  next_status=HAL_BUSY; assert(ConsoleTx_Write((uint8_t*)"busy",4)); ConsoleTx_Process();
  next_status=HAL_OK; ConsoleTx_Process(); assert(!memcmp(active,"busy",4)); finish();
  assert(!ConsoleTx_Write((uint8_t*)reply,9000));
  assert(ConsoleTx_Telemetry("timeout",7)); ConsoleTx_Process(); tick+=501; ConsoleTx_Process();
  assert(ConsoleTx_Dropped()==2);
  ConsoleTx_Init(&uart); tick=0;
  assert(ConsoleTx_Telemetry("map",3));
  assert(ConsoleTx_Event("old",3)); assert(ConsoleTx_Event("event",5));
  assert(ConsoleTx_Write((uint8_t *)"reply",5));
  ConsoleTx_Process(); assert(length==5 && !memcmp(active,"reply",5));
  finish(); assert(length==5 && !memcmp(active,"event",5));
  assert(ConsoleTx_Event("cancel",6)); ConsoleTx_EventCancel();
  assert(length==5 && !memcmp(active,"event",5)); /* In-flight event remains intact. */
  finish(); assert(length==3 && !memcmp(active,"map",3)); finish();
  assert(ConsoleTx_EventDropped()==2 && ConsoleTx_Dropped()==0);
  next_status=HAL_BUSY; assert(ConsoleTx_Event("stale",5)); ConsoleTx_Process();
  assert(ConsoleTx_Write((uint8_t *)"first",5));
  next_status=HAL_OK; ConsoleTx_Process(); assert(length==5 && !memcmp(active,"first",5));
  ConsoleTx_EventCancel(); finish();
  assert(ConsoleTx_EventDropped()==3);
  assert(ConsoleTx_Event("timeout",7)); ConsoleTx_Process(); tick+=501; ConsoleTx_Process();
  assert(ConsoleTx_EventDropped()==4 && ConsoleTx_Dropped()==0);
  next_status=HAL_BUSY; assert(ConsoleTx_Telemetry("map2",4)); ConsoleTx_Process();
  assert(ConsoleTx_Event("qr",2)); next_status=HAL_OK; ConsoleTx_Process();
  assert(length==2 && !memcmp(active,"qr",2)); finish();
  assert(length==4 && !memcmp(active,"map2",4)); finish();
  next_status=HAL_ERROR; assert(ConsoleTx_Event("error",5)); ConsoleTx_Process();
  assert(ConsoleTx_EventDropped()==5 && ConsoleTx_Dropped()==0);
  next_status=HAL_OK; assert(ConsoleTx_Event("irq",3)); ConsoleTx_Process();
  ConsoleTx_ErrorCallback(&uart); ConsoleTx_Process();
  assert(ConsoleTx_EventDropped()==6);
  assert(!ConsoleTx_Event(NULL,1) && !ConsoleTx_Event("",0));
  assert(ConsoleTx_EventDropped()==8);
  puts("console_tx: lifetime, priority, latest telemetry, busy, overflow, timeout PASS");
  return 0;
}
