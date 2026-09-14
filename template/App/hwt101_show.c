/**
 * @file    hwt101_show.c
 * @brief   HWT101 Z 轴角度 OLED 显示程序。
 */
#include "hwt101_show.h"
#include "oled.h"

#include <stdio.h>
#include <string.h>

#define HWT101_SHOW_REFRESH_MS   100U
/* HWT101 OLED显示刷新间隔。 */

#define HWT101_SHOW_TEXT_LENGTH  22U
/* HWT101 OLED单行文本缓冲区长度。 */

static bool hwt101_show_is_ready = false;
static uint32_t hwt101_show_last_tick = 0U;
static uint32_t hwt101_show_last_count = 0U;
static char hwt101_show_last_yaw[HWT101_SHOW_TEXT_LENGTH] = {0};
static char hwt101_show_last_count_text[HWT101_SHOW_TEXT_LENGTH] = {0};

/**********************************************************
*** 角度文本格式化
**********************************************************/
/**
  * @brief    把浮点角度转换为“Y:+123.45 deg”形式
  * @param    angle      ：角度，单位为度
  * @param    angle_text ：格式化结果缓存
  * @param    text_size  ：结果缓存长度
  * @retval   无
  * @note     不使用 printf 浮点格式，避免扩大 ARMCC5 固件体积
  */
static void HWT101_Angle_Text_Make(float angle,
                                   char *angle_text,
                                   uint32_t text_size)
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
                 "Y:%c%3lu.%02lu deg",
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
static HAL_StatusTypeDef HWT101_Show_Line_Update(uint8_t page,
                                                 const char *text,
                                                 char *last_text)
{
  HAL_StatusTypeDef hal_status;

  if ((text == NULL) || (last_text == NULL))
  {
    return HAL_ERROR;
  }

  if (strcmp(text, last_text) == 0)
  {
    return HAL_OK;
  }

  hal_status = OLED_Line_Show(page, text);
  if (hal_status == HAL_OK)
  {
    (void)snprintf(last_text, HWT101_SHOW_TEXT_LENGTH, "%s", text);
  }

  return hal_status;
}

/**********************************************************
*** HWT101角度显示初始化
**********************************************************/
/**
  * @brief    初始化 OLED，并显示 HWT101 等待界面
  * @param    hi2c ：OLED 使用的 I2C 句柄，当前工程传入 &hi2c1
  * @retval   HAL状态
  */
HAL_StatusTypeDef HWT101_Show_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef hal_status;

  hwt101_show_is_ready = false;
  hwt101_show_last_count = 0U;
  hwt101_show_last_tick = HAL_GetTick();
  hwt101_show_last_yaw[0] = '\0';
  hwt101_show_last_count_text[0] = '\0';

  hal_status = OLED_Init(hi2c);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  hal_status = OLED_Line_Show(0U, "HWT101 YAW");
  if (hal_status == HAL_OK)
  {
    hal_status = OLED_Line_Show(2U, "WAIT I2C2...");
  }

  if (hal_status == HAL_OK)
  {
    hwt101_show_is_ready = true;
  }

  return hal_status;
}

/**********************************************************
*** HWT101角度显示刷新
**********************************************************/
/**
  * @brief    将最新的 Yaw 和有效帧计数刷新到 OLED
  * @param    angle ：HWT101 最新 Z 轴角度
  * @retval   HAL状态
  */
HAL_StatusTypeDef HWT101_Show_Process(const HWT101_Angle_t *angle)
{
  char show_text[HWT101_SHOW_TEXT_LENGTH];
  HAL_StatusTypeDef hal_status;

  if ((!hwt101_show_is_ready) || (angle == NULL))
  {
    return HAL_ERROR;
  }

  if ((angle->update_count == hwt101_show_last_count) ||
      ((HAL_GetTick() - hwt101_show_last_tick) < HWT101_SHOW_REFRESH_MS))
  {
    return HAL_OK;
  }

  HWT101_Angle_Text_Make(angle->yaw, show_text, sizeof(show_text));
  hal_status = HWT101_Show_Line_Update(2U, show_text,
                                       hwt101_show_last_yaw);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  (void)snprintf(show_text, sizeof(show_text), "COUNT:%lu",
                 (unsigned long)angle->update_count);
  hal_status = HWT101_Show_Line_Update(4U, show_text,
                                       hwt101_show_last_count_text);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  hwt101_show_last_count = angle->update_count;
  hwt101_show_last_tick = HAL_GetTick();
  return HAL_OK;
}
