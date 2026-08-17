/**
 * @file    key.c
 * @brief   板载四按键扫描驱动实现。
 */
#include "key.h"

#define KEY_COUNT              4U
#define KEY_DEBOUNCE_TIME_MS   20U
#define KEY_ACTIVE_LEVEL       GPIO_PIN_RESET

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  KeyEvent_t event;
  GPIO_PinState raw_state;
  GPIO_PinState stable_state;
  uint32_t change_tick;
} KeyObject_t;

/* PE2、PE3、PE4、PE5 依次对应前、后、左、右。 */
static KeyObject_t key_objects[KEY_COUNT] =
{
  {GPIOE, GPIO_PIN_2, KEY_EVENT_FORWARD,  GPIO_PIN_SET, GPIO_PIN_SET, 0U},
  {GPIOE, GPIO_PIN_3, KEY_EVENT_BACKWARD, GPIO_PIN_SET, GPIO_PIN_SET, 0U},
  {GPIOE, GPIO_PIN_4, KEY_EVENT_LEFT,     GPIO_PIN_SET, GPIO_PIN_SET, 0U},
  {GPIOE, GPIO_PIN_5, KEY_EVENT_RIGHT,    GPIO_PIN_SET, GPIO_PIN_SET, 0U}
};

static KeyEvent_t pending_event = KEY_EVENT_NONE;

/**********************************************************
*** 按键初始化
**********************************************************/
/**
  * @brief    初始化四个板载按键的原始状态和稳定状态
  * @param    无
  * @retval   无
  * @note     必须在 MX_GPIO_Init() 之后调用；初始化不会产生按键事件
  */
void Key_Init(void)
{
  uint8_t index;
  GPIO_PinState current_state;

  for (index = 0U; index < KEY_COUNT; ++index)
  {
    current_state = HAL_GPIO_ReadPin(key_objects[index].port,
                                    key_objects[index].pin);
    key_objects[index].raw_state = current_state;
    key_objects[index].stable_state = current_state;
    key_objects[index].change_tick = HAL_GetTick();
  }

  pending_event = KEY_EVENT_NONE;
}

/**********************************************************
*** 按键扫描与消抖
**********************************************************/
/**
  * @brief    扫描 PE2、PE3、PE4、PE5，并完成20ms非阻塞消抖
  * @param    无
  * @retval   无
  * @note     本函数应在 while(1) 中持续调用；长按只产生一次按下事件
  */
void Key_Scan(void)
{
  uint8_t index;
  uint32_t current_tick = HAL_GetTick();
  GPIO_PinState current_state;

  for (index = 0U; index < KEY_COUNT; ++index)
  {
    current_state = HAL_GPIO_ReadPin(key_objects[index].port,
                                    key_objects[index].pin);

    /* 原始电平发生变化时，重新开始计算20ms稳定时间。 */
    if (current_state != key_objects[index].raw_state)
    {
      key_objects[index].raw_state = current_state;
      key_objects[index].change_tick = current_tick;
      continue;
    }

    /* 只有电平持续稳定20ms后，才更新按键的有效状态。 */
    if ((current_state != key_objects[index].stable_state) &&
        ((current_tick - key_objects[index].change_tick) >= KEY_DEBOUNCE_TIME_MS))
    {
      key_objects[index].stable_state = current_state;

      /* 仅在松开到按下的边沿产生一次事件，长按不会重复触发。 */
      if ((current_state == KEY_ACTIVE_LEVEL) &&
          (pending_event == KEY_EVENT_NONE))
      {
        pending_event = key_objects[index].event;
      }
    }
  }
}

/**********************************************************
*** 按键事件读取
**********************************************************/
/**
  * @brief    读取并清除一个已经消抖完成的按键按下事件
  * @param    无
  * @retval   KEY_EVENT_NONE      ：当前没有新的按键事件
  * @retval   其他 KeyEvent_t 值  ：对应方向按键被按下
  */
KeyEvent_t Key_Get_Press_Event(void)
{
  KeyEvent_t event = pending_event;

  pending_event = KEY_EVENT_NONE;
  return event;
}
