/**
 * @file    screen_verify.c
 * @brief   陶晶驰串口屏最小功能验证程序实现。
 */
#include "screen_verify.h"

#include "delay.h"
#include "tjc_screen.h"

#define SCREEN_VERIFY_BOOT_DELAY_MS      1000U
#define SCREEN_VERIFY_COMMAND_DELAY_MS   50U
#define SCREEN_VERIFY_REFRESH_TIME_MS    500U
#define SCREEN_VERIFY_PAGE_ID            0U
#define SCREEN_VERIFY_TEXT_OBJECT        "t0"
#define SCREEN_VERIFY_NUMBER_OBJECT      "n0"

typedef enum
{
  SCREEN_VERIFY_WAIT_POWER = 0,
  SCREEN_VERIFY_SHOW_PAGE,
  SCREEN_VERIFY_SHOW_TEXT,
  SCREEN_VERIFY_REFRESH_NUMBER
} ScreenVerifyState_t;

static ScreenVerifyState_t screen_verify_state = SCREEN_VERIFY_WAIT_POWER;
static uint32_t screen_verify_tick = 0U;
static uint32_t screen_refresh_count = 0U;
static uint32_t screen_receive_count = 0U;
static bool screen_receive_message_pending = false;

/**********************************************************
*** 串口屏验证初始化
**********************************************************/
/**
  * @brief    初始化串口屏上电等待和页面验证状态
  * @param    无
  * @retval   无
  */
void Screen_Check_Init(void)
{
  screen_verify_state = SCREEN_VERIFY_WAIT_POWER;
  screen_verify_tick = HAL_GetTick();
  screen_refresh_count = 0U;
  screen_receive_count = 0U;
  screen_receive_message_pending = false;
}

/**********************************************************
*** 串口屏验证主循环
**********************************************************/
/**
  * @brief    发送页面、文本和递增数字，并处理屏幕返回数据
  * @param    无
  * @retval   无
  * @note     t0显示通信状态，n0每500ms递增一次
  */
void Screen_Check_Process(void)
{
  uint8_t rx_frame[TJC_RX_FRAME_MAX_LENGTH];
  uint16_t rx_length;

  TJC_Process();
  if (TJC_Frame_Get(rx_frame, &rx_length, sizeof(rx_frame)))
  {
    ++screen_receive_count;
    screen_receive_message_pending = true;
  }

  switch (screen_verify_state)
  {
    case SCREEN_VERIFY_WAIT_POWER:
      if (Delay_Time_Is_Up(screen_verify_tick, SCREEN_VERIFY_BOOT_DELAY_MS))
      {
        screen_verify_state = SCREEN_VERIFY_SHOW_PAGE;
      }
      break;

    case SCREEN_VERIFY_SHOW_PAGE:
      if (TJC_Page_Show(SCREEN_VERIFY_PAGE_ID) == HAL_OK)
      {
        screen_verify_tick = HAL_GetTick();
        screen_verify_state = SCREEN_VERIFY_SHOW_TEXT;
      }
      break;

    case SCREEN_VERIFY_SHOW_TEXT:
      if (Delay_Time_Is_Up(screen_verify_tick,
                           SCREEN_VERIFY_COMMAND_DELAY_MS) &&
          (TJC_Text_Set(SCREEN_VERIFY_TEXT_OBJECT,
                        "USART3 DMA OK") == HAL_OK))
      {
        screen_verify_tick = HAL_GetTick();
        screen_verify_state = SCREEN_VERIFY_REFRESH_NUMBER;
      }
      break;

    case SCREEN_VERIFY_REFRESH_NUMBER:
      /* 收到屏幕返回帧后，优先在t0上显示双向通信成功。 */
      if (screen_receive_message_pending)
      {
        if (TJC_Text_Set(SCREEN_VERIFY_TEXT_OBJECT,
                         "USART3 RX OK") == HAL_OK)
        {
          screen_receive_message_pending = false;
        }
      }
      else if (Delay_Time_Is_Up(screen_verify_tick,
                                SCREEN_VERIFY_REFRESH_TIME_MS))
      {
        if (TJC_Number_Set(SCREEN_VERIFY_NUMBER_OBJECT,
                           (int32_t)screen_refresh_count) == HAL_OK)
        {
          ++screen_refresh_count;
          screen_verify_tick = HAL_GetTick();
        }
      }
      break;

    default:
      screen_verify_state = SCREEN_VERIFY_WAIT_POWER;
      screen_verify_tick = HAL_GetTick();
      break;
  }
}
