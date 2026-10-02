#include "led.h"

#define LED_GPIO_PORT    GPIOB
#define LED_GPIO_PIN     GPIO_PIN_2

void led_init(void)
{
    led_off();
}

void led_on(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
}

void led_off(void)
{
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}

void led_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
}

void led_set(led_state_t state)
{
    if (state == LED_ON)
    {
        led_on();
    }
    else
    {
        led_off();
    }
}
