#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "turntable_boot.h"
static uint32_t tick;
static unsigned count;
static HAL_StatusTypeDef tx_result;
static UART_HandleTypeDef uart;
uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *p, uint8_t *data, uint16_t n, uint32_t timeout)
{
  const uint8_t enable[]={1,0xf3,0xab,1,0,0x6b};
  const uint8_t velocity[]={1,0xf6,0,0,8,10,0,0x6b};
  assert(p==&uart && timeout==20);
  if(count==0) assert(n==sizeof(enable) && memcmp(data,enable,n)==0);
  else { assert(count==1); assert(n==sizeof(velocity) && memcmp(data,velocity,n)==0); }
  count++;
  return tx_result;
}
/* Intentionally no GPIO or receive APIs: the boot path must not depend on them. */
static void run(uint32_t start, HAL_StatusTypeDef result)
{
  tick=start; count=0; tx_result=result; Turntable_BootInit(&uart);
  tick=start+999; Turntable_BootProcess(); assert(count==0);
  tick=start+1000; Turntable_BootProcess(); assert(count==1);
  tick=start+1099; Turntable_BootProcess(); assert(count==1);
  tick=start+1100; Turntable_BootProcess(); assert(count==2);
  tick=start+10000; Turntable_BootProcess(); assert(count==2);
  assert(Turntable_BootStatus.enable_sent==1 && Turntable_BootStatus.velocity_sent==1);
  assert(Turntable_BootStatus.enable_result==result && Turntable_BootStatus.velocity_result==result);
}
int main(void)
{
  run(0,HAL_OK); run(0xfffffff0U,HAL_OK); run(0,HAL_ERROR);
  puts("boot direct UART: PASS"); return 0;
}
