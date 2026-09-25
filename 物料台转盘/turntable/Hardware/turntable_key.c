#include "turntable_key.h"
#include "turntable_config.h"
#include "stm32f1xx_hal.h"
typedef struct { uint32_t changed; uint8_t raw, stable, armed; } Key_t;
static Key_t keys[2];
static uint8_t Read(unsigned index)
{
  return HAL_GPIO_ReadPin(index == 0 ? GPIOA : GPIOB,
                         index == 0 ? GPIO_PIN_15 : GPIO_PIN_3) == GPIO_PIN_RESET;
}
void Key_Init(void)
{
  unsigned i;
  for(i=0;i<2;i++) {
    keys[i].raw=keys[i].stable=Read(i);
    keys[i].armed=!keys[i].raw;
    keys[i].changed=HAL_GetTick();
  }
}
uint8_t Key_Scan(void)
{
  unsigned i; uint8_t events=0, raw; uint32_t now=HAL_GetTick();
  for(i=0;i<2;i++) {
    raw=Read(i);
    if(raw!=keys[i].raw) { keys[i].raw=raw; keys[i].changed=now; }
    if(raw!=keys[i].stable && (uint32_t)(now-keys[i].changed)>=TURNTABLE_KEY_DEBOUNCE_MS) {
      keys[i].stable=raw;
      if(!raw) keys[i].armed=1;
      else if(keys[i].armed) { events|=(uint8_t)(1U<<i); keys[i].armed=0; }
    }
  }
  return events;
}
