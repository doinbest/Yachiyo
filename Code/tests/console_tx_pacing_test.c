#include "console_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static UART_HandleTypeDef uart;
static uint32_t tick, finished_at;
static unsigned starts;
static bool have_finished;
static HAL_StatusTypeDef next_status;
static const uint8_t *sending;
static uint16_t send_size;
static char wire[12000];
static size_t written;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n)
{
  assert(u == &uart && !sending && n > 0 && n <= CONSOLE_TX_CHUNK_BYTES);
#if CONSOLE_TX_GAP_MS > 0U
  if(have_finished) assert((uint32_t)(tick-finished_at)>=CONSOLE_TX_GAP_MS);
#endif
  if(next_status!=HAL_OK) return next_status;
  sending=p;send_size=n;starts++;return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *u)
{ (void)u;sending=NULL;return HAL_OK; }
static void advance(unsigned ms)
{
  while(ms--)
  {
    tick++;
    if(sending)
    {
      assert(written+send_size<sizeof(wire));
      memcpy(wire+written,sending,send_size);written+=send_size;wire[written]=0;
      sending=NULL;finished_at=tick;have_finished=true;
      ConsoleTx_TxCpltCallback(&uart);
    }
    ConsoleTx_Process();
  }
}
int main(void)
{
  char frame[1100], expected[1300];
  /* A completion must admit the next DMA chunk in that same Process call,
   * without advancing the clock by a fixed wireless delay. */
  memset(frame,'n',200U);frame[200]='\0';ConsoleTx_Init(&uart);
  assert(ConsoleTx_Telemetry(frame,200U));ConsoleTx_Process();assert(starts==1U);
  advance(1U);assert(starts==2U && sending && written==CONSOLE_TX_CHUNK_BYTES);
  advance(10U);assert(written==200U && !memcmp(wire,frame,200U));
  starts=0U;written=0U;wire[0]='\0';have_finished=false;
  memset(frame,'x',sizeof(frame));memcpy(frame,"@CHASSIS {\"padding\":\"",21);
  memcpy(frame+sizeof(frame)-5,"\"}\r\n",5);
  ConsoleTx_Init(&uart);tick=0xfffffff0U;
  assert(ConsoleTx_Telemetry(frame,(uint16_t)strlen(frame)));
  ConsoleTx_Process();
  assert(ConsoleTx_Write((const uint8_t*)"OK query\r\n",10));
  assert(ConsoleTx_Event("EVT qr\r\n",8));
  assert(ConsoleTx_Telemetry("stale\r\n",7));
  assert(ConsoleTx_Telemetry("latest\r\n",8));
  snprintf(expected,sizeof(expected),"%sOK query\r\nEVT qr\r\nlatest\r\n",frame);
  advance(2000);
  assert(!strcmp(wire,expected) && starts==(strlen(frame)+CONSOLE_TX_CHUNK_BYTES-1U)/CONSOLE_TX_CHUNK_BYTES+3U);
  assert(ConsoleTx_Dropped()==1);
  /* Cancel drops only a pending frame, even when an active line is between chunks. */
  written=0;wire[0]=0;
  assert(ConsoleTx_Telemetry(frame,(uint16_t)strlen(frame)));
  ConsoleTx_Process();next_status=HAL_BUSY;advance(1);
  assert(ConsoleTx_Write((const uint8_t*)"OK after\r\n",10));
  assert(ConsoleTx_Telemetry("cancel-me\r\n",11));
  ConsoleTx_TelemetryCancel();
  assert(!ConsoleTx_TelemetryReady());
  advance(100);assert(written==CONSOLE_TX_CHUNK_BYTES);
  next_status=HAL_OK;advance(2000);
  snprintf(expected,sizeof(expected),"%sOK after\r\n",frame);
  assert(!strcmp(wire,expected) && ConsoleTx_TelemetryReady());
  assert(ConsoleTx_Dropped()==2);
  puts("console_tx_pacing_test: no fixed gap, bounded chunks, atomic JSON, reply priority, latest pending frame and tick wrap PASS");
}
