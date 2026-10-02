#include "key.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef test_gpioe;
static uint32_t tick;
static uint16_t pressed;
uint32_t HAL_GetTick(void) { return tick; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{ assert(port == GPIOE); return (pressed & pin) ? GPIO_PIN_RESET : GPIO_PIN_SET; }
int main(void)
{
  unsigned key;
  Key_Init(); assert(Key_Get_Press_Event() == KEY_EVENT_NONE);
  for (key = 2; key <= 5; key++)
  {
    pressed = (uint16_t)(1U << key); Key_Scan();
    tick += 19; Key_Scan(); assert(Key_Get_Press_Event() == KEY_EVENT_NONE);
    tick++; Key_Scan(); assert(Key_Get_Press_Event() == (KeyEvent_t)(key - 1));
    tick += 2000; Key_Scan(); assert(Key_Get_Press_Event() == KEY_EVENT_NONE);
    pressed = 0; Key_Scan(); tick += 20; Key_Scan(); assert(Key_Get_Press_Event() == KEY_EVENT_NONE);
  }
  puts("key_test: PE2..PE5 debounce and single press OK"); return 0;
}
