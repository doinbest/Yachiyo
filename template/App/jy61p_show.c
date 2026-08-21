/**
 * @file    jy61p_show.c
 * @brief   JY61P 三轴角度 OLED 显示程序。
 */
#include "jy61p_show.h"
#include "oled.h"

#include <stdio.h>
#include <string.h>

#define JY61P_SHOW_REFRESH_MS  100U
#define JY61P_SHOW_TEXT_LENGTH  22U

static bool jy61p_show_is_ready = false;
static uint32_t jy61p_show_last_tick = 0U;
static uint32_t jy61p_show_last_count = 0U;
static char jy61p_show_last_roll[JY61P_SHOW_TEXT_LENGTH] = {0};
static char jy61p_show_last_pitch[JY61P_SHOW_TEXT_LENGTH] = {0};
static char jy61p_show_last_yaw[JY61P_SHOW_TEXT_LENGTH] = {0};

/**********************************************************
*** 角度文本格式化
**********************************************************/
/**
  * @brief    把浮点角度转换为“R:+123.45 deg”形式
  * @param    axis       ：轴名称字符，使用 R、P 或 Y
  * @param    angle      ：角度，单位为度
  * @param    angle_text ：格式化结果缓存
  * @param    text_size  ：结果缓存长度
  * @retval   无
  * @note     不使用 printf 浮点格式，避免扩大 ARMCC5 固件体积
  */
static void JY61P_Angle_Text_Make(char axis, float angle,
                                  char *angle_text, uint32_t text_size)
{
  int32_t angle_centidegree;
  uint32_t angle_absolute;
  char angle_sign;

  if ((angle_text == NULL) || (text_size == 0U))
  {
    return;
  }

  if (angle >= 0.0f)
  {
    angle_centidegree = (int32_t)((angle * 100.0f) + 0.5f);
    angle_sign = '+';
  }
  else
  {
    angle_centidegree = (int32_t)((angle * 100.0f) - 0.5f);
    angle_sign = '-';
  }

  if (angle_centidegree < 0)
  {
    angle_absolute = (uint32_t)(-angle_centidegree);
  }
  else
  {
    angle_absolute = (uint32_t)angle_centidegree;
  }

  (void)snprintf(angle_text, text_size,
                 "%c:%c%3lu.%02lu deg",
                 axis,
                 angle_sign,
                 (unsigned long)(angle_absolute / 100U),
                 (unsigned long)(angle_absolute % 100U));
}

/**********************************************************
*** 单行文本刷新
**********************************************************/
/**
  * @brief    文本发生变化时，使用整页数据一次刷新指定行
  * @param    page      ：OLED页号
  * @param    text      ：本次显示文本
  * @param    last_text ：上次成功显示的文本缓存
  * @retval   HAL状态
  */
static HAL_StatusTypeDef JY61P_Show_Line_Update(uint8_t page,
                                                const char *text,
                                                char *last_text)
{
  HAL_StatusTypeDef hal_status;

  if ((text == NULL) || (last_text == NULL))
  {
    return HAL_ERROR;
  }

  /* 显示内容没有变化时，不重复占用 I2C 总线。 */
  if (strcmp(text, last_text) == 0)
  {
    return HAL_OK;
  }

  hal_status = OLED_Line_Show(page, text);
  if (hal_status == HAL_OK)
  {
    (void)snprintf(last_text, JY61P_SHOW_TEXT_LENGTH, "%s", text);
  }

  return hal_status;
}

/**********************************************************
*** JY61P角度显示初始化
**********************************************************/
HAL_StatusTypeDef JY61P_Show_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef hal_status;

  jy61p_show_is_ready = false;
  jy61p_show_last_count = 0U;
  jy61p_show_last_tick = HAL_GetTick();
  jy61p_show_last_roll[0] = '\0';
  jy61p_show_last_pitch[0] = '\0';
  jy61p_show_last_yaw[0] = '\0';

  hal_status = OLED_Init(hi2c);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  hal_status = OLED_Line_Show(0U, "JY61P ANGLE");
  if (hal_status == HAL_OK)
  {
    hal_status = OLED_Line_Show(2U, "WAIT UART4...");
  }

  if (hal_status == HAL_OK)
  {
    jy61p_show_is_ready = true;
  }

  return hal_status;
}

/**********************************************************
*** JY61P角度显示刷新
**********************************************************/
HAL_StatusTypeDef JY61P_Show_Process(const JY61P_Angle_t *angle)
{
  char angle_text[JY61P_SHOW_TEXT_LENGTH];
  HAL_StatusTypeDef hal_status;

  if ((!jy61p_show_is_ready) || (angle == NULL))
  {
    return HAL_ERROR;
  }

  if ((angle->update_count == jy61p_show_last_count) ||
      ((HAL_GetTick() - jy61p_show_last_tick) < JY61P_SHOW_REFRESH_MS))
  {
    return HAL_OK;
  }

  JY61P_Angle_Text_Make('R', angle->roll,
                        angle_text, sizeof(angle_text));
  hal_status = JY61P_Show_Line_Update(2U, angle_text,
                                      jy61p_show_last_roll);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  JY61P_Angle_Text_Make('P', angle->pitch,
                        angle_text, sizeof(angle_text));
  hal_status = JY61P_Show_Line_Update(4U, angle_text,
                                      jy61p_show_last_pitch);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  JY61P_Angle_Text_Make('Y', angle->yaw,
                        angle_text, sizeof(angle_text));
  hal_status = JY61P_Show_Line_Update(6U, angle_text,
                                      jy61p_show_last_yaw);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  jy61p_show_last_count = angle->update_count;
  jy61p_show_last_tick = HAL_GetTick();
  return HAL_OK;
}
