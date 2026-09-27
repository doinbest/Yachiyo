#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "turntable_direct.h"
static UART_HandleTypeDef uart;
static uint32_t tick, count, reads;
static uint8_t a,b, last[16];
static uint16_t size;
static uint32_t simulated_target;
uint32_t HAL_GetTick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(void *port,uint16_t pin)
{ (void)pin; reads++; return (port==GPIOA ? a : b) ? GPIO_PIN_RESET : GPIO_PIN_SET; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *p,uint8_t *data,uint16_t n,uint32_t timeout)
{
  uint32_t distance;
  assert(p==&uart && timeout==20 && n<=16); memcpy(last,data,n); size=n; count++;
  if(data[1]==0xfd) {
    distance=((uint32_t)data[6]<<24)|((uint32_t)data[7]<<16)|((uint32_t)data[8]<<8)|data[9];
    /* Reproduce the reported behavior: nonzero mode treated as absolute.
       This is a compatibility fixture, not a confirmed firmware-version claim. */
    if(data[10]==0) simulated_target+=distance; else simulated_target=distance;
  }
  return HAL_OK;
}
/* No UART receive implementation: no reply can be required to operate either key. */
static void advance(unsigned n) { while(n--) { tick++; Turntable_DirectProcess(); } }
static void press(uint8_t *key) { *key=1; advance(30); *key=0; advance(30); }
static unsigned pulses(void)
{ return ((unsigned)last[6]<<24)|((unsigned)last[7]<<16)|((unsigned)last[8]<<8)|last[9]; }
static void reset(void)
{
  tick=count=simulated_target=0; a=b=0; Turntable_DirectInit(&uart); advance(1200);
}
int main(void)
{
  const uint8_t stop[]={1,0xfe,0x98,0,0x6b};
  unsigned guard;
  reset(); assert(count==0);
  press(&a); advance(100);
  assert(count==2 && size==13 && last[1]==0xfd && pulses()==533);
  assert(last[3]==0 && last[4]==8);
  assert(simulated_target==533);
  press(&b); assert(count==2);
  guard=3000;
  while(Turntable_DirectStatus.state==DIRECT_STEP && guard--) advance(1);
  assert(guard>0);
  advance(1999); assert(count==2);
  advance(1); assert(count==3 && pulses()==1067);
  assert(simulated_target==1600);
  advance(6000); assert(count==4 && pulses()==1067);
  assert(simulated_target==2667);
  press(&a); assert(size==5 && memcmp(last,stop,5)==0);
  advance(10000); assert(count==5);

  reset(); press(&b); advance(100); assert(pulses()==533);
  b=1; advance(5000); assert(count==2); b=0; advance(30);
  press(&b); advance(100); assert(pulses()==1067);
  advance(5000); assert(count==4); /* single-step never starts automatic cycle */
  press(&b); advance(100); assert(pulses()==1067);
  advance(5000); press(&b); advance(100); assert(pulses()==1066);
  assert(simulated_target==3733);
  press(&a); assert(last[1]==0xfe);
  /* Stop also cancels a pending enable-to-motion gap. */
  press(&a); press(&a); advance(5000); assert(last[1]==0xfe);
  assert(reads>0);
  reset(); press(&a); advance(2500); press(&a); /* stop during dwell */
  advance(10000); assert(count==3 && last[1]==0xfe);
  tick=0xfffffff0U; a=b=0; count=0;
  Turntable_DirectInit(&uart); advance(1200); press(&a); advance(100);
  assert(count==2 && last[1]==0xfd);
  puts("direct automatic indexing without RX: PASS"); return 0;
}
