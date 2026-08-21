/**
 * @file    oled.c
 * @brief   SSD1306 128x64 OLED 硬件 I2C 驱动实现。
 *
 * 本文件参考 F103 工程的 SSD1306 初始化流程和6x8字库，底层通信改为
 * STM32F4 HAL_I2C_Master_Transmit()，不再使用 GPIO 模拟 I2C。
 */
#include "oled.h"
#include "oled_font.h"

#define OLED_CONTROL_COMMAND      0x00U
#define OLED_CONTROL_DATA         0x40U
#define OLED_I2C_TIMEOUT_MS       100U
#define OLED_CLEAR_BLOCK_SIZE      16U
#define OLED_CHAR_WIDTH             6U
#define OLED_ASCII_FIRST          0x20U
#define OLED_ASCII_LAST           0x7EU

static I2C_HandleTypeDef *oled_i2c = NULL;

/**********************************************************
*** OLED底层I2C发送
**********************************************************/
/**
  * @brief    向 OLED 发送一组命令
  * @param    command ：命令数组
  * @param    length  ：命令字节数，最大31字节
  * @retval   HAL状态
  */
static HAL_StatusTypeDef OLED_Command_Send(const uint8_t *command,
                                            uint8_t length)
{
  uint8_t tx_data[32];
  uint8_t index;

  if ((oled_i2c == NULL) || (command == NULL) ||
      (length == 0U) || (length > 31U))
  {
    return HAL_ERROR;
  }

  tx_data[0] = OLED_CONTROL_COMMAND;
  for (index = 0U; index < length; ++index)
  {
    tx_data[index + 1U] = command[index];
  }

  return HAL_I2C_Master_Transmit(oled_i2c,
                                 (uint16_t)(OLED_I2C_ADDRESS_7BIT << 1U),
                                 tx_data,
                                 (uint16_t)(length + 1U),
                                 OLED_I2C_TIMEOUT_MS);
}

/**
  * @brief    设置 OLED 当前写入位置
  * @param    x    ：横坐标
  * @param    page ：页号
  * @retval   HAL状态
  */
static HAL_StatusTypeDef OLED_Position_Set(uint8_t x, uint8_t page)
{
  uint8_t command[3];

  if ((x >= OLED_WIDTH) || (page >= OLED_PAGE_COUNT))
  {
    return HAL_ERROR;
  }

  command[0] = (uint8_t)(0xB0U | page);
  command[1] = (uint8_t)(x & 0x0FU);
  command[2] = (uint8_t)(0x10U | ((x >> 4U) & 0x0FU));
  return OLED_Command_Send(command, sizeof(command));
}

/**********************************************************
*** OLED初始化
**********************************************************/
HAL_StatusTypeDef OLED_Init(I2C_HandleTypeDef *hi2c)
{
  static const uint8_t init_command[] =
  {
    0xAEU,             /* 关闭显示。 */
    0x20U, 0x02U,      /* 页寻址模式。 */
    0xB0U,             /* 起始页0。 */
    0xC8U,             /* COM扫描方向反向。 */
    0x00U, 0x10U,      /* 列地址低位和高位。 */
    0x40U,             /* 起始显示行0。 */
    0x81U, 0xFFU,      /* 对比度。 */
    0xA1U,             /* 段重映射。 */
    0xA6U,             /* 正常显示。 */
    0xA8U, 0x3FU,      /* 1/64复用。 */
    0xA4U,             /* 使用显存内容。 */
    0xD3U, 0x00U,      /* 显示偏移0。 */
    0xD5U, 0x80U,      /* 时钟分频。 */
    0xD9U, 0xF1U,      /* 预充电周期。 */
    0xDAU, 0x12U,      /* COM引脚配置。 */
    0xDBU, 0x30U,      /* VCOMH电平。 */
    0x8DU, 0x14U,      /* 开启内部电荷泵。 */
    0xAFU              /* 开启显示。 */
  };
  HAL_StatusTypeDef hal_status;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  oled_i2c = hi2c;
  HAL_Delay(100U);

  hal_status = HAL_I2C_IsDeviceReady(oled_i2c,
                                     (uint16_t)(OLED_I2C_ADDRESS_7BIT << 1U),
                                     2U,
                                     OLED_I2C_TIMEOUT_MS);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  hal_status = OLED_Command_Send(init_command, sizeof(init_command));
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  return OLED_Clear();
}

/**********************************************************
*** OLED单页清除
**********************************************************/
HAL_StatusTypeDef OLED_Page_Clear(uint8_t page)
{
  uint8_t tx_data[OLED_CLEAR_BLOCK_SIZE + 1U] = {0U};
  uint8_t block;
  HAL_StatusTypeDef hal_status;

  if ((oled_i2c == NULL) || (page >= OLED_PAGE_COUNT))
  {
    return HAL_ERROR;
  }

  hal_status = OLED_Position_Set(0U, page);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  tx_data[0] = OLED_CONTROL_DATA;
  for (block = 0U;
       block < (OLED_WIDTH / OLED_CLEAR_BLOCK_SIZE);
       ++block)
  {
    hal_status = HAL_I2C_Master_Transmit(
                   oled_i2c,
                   (uint16_t)(OLED_I2C_ADDRESS_7BIT << 1U),
                   tx_data,
                   sizeof(tx_data),
                   OLED_I2C_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
      return hal_status;
    }
  }

  return HAL_OK;
}

/**********************************************************
*** OLED清屏
**********************************************************/
HAL_StatusTypeDef OLED_Clear(void)
{
  uint8_t page;
  HAL_StatusTypeDef hal_status;

  for (page = 0U; page < OLED_PAGE_COUNT; ++page)
  {
    hal_status = OLED_Page_Clear(page);
    if (hal_status != HAL_OK)
    {
      return hal_status;
    }
  }

  return HAL_OK;
}

/**********************************************************
*** OLED字符和字符串显示
**********************************************************/
/**
  * @brief    显示一个6x8 ASCII字符
  * @param    x         ：起始横坐标
  * @param    page      ：页号
  * @param    character ：ASCII字符
  * @retval   HAL状态
  */
static HAL_StatusTypeDef OLED_Character_Show(uint8_t x, uint8_t page,
                                              char character)
{
  uint8_t tx_data[OLED_CHAR_WIDTH + 1U];
  uint8_t index;
  uint8_t font_index;
  HAL_StatusTypeDef hal_status;

  if ((x > (OLED_WIDTH - OLED_CHAR_WIDTH)) ||
      (page >= OLED_PAGE_COUNT))
  {
    return HAL_ERROR;
  }

  if (((uint8_t)character < OLED_ASCII_FIRST) ||
      ((uint8_t)character > OLED_ASCII_LAST))
  {
    character = '?';
  }

  hal_status = OLED_Position_Set(x, page);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  font_index = (uint8_t)character - OLED_ASCII_FIRST;
  tx_data[0] = OLED_CONTROL_DATA;
  for (index = 0U; index < OLED_CHAR_WIDTH; ++index)
  {
    tx_data[index + 1U] = oled_font_6x8[font_index][index];
  }

  return HAL_I2C_Master_Transmit(oled_i2c,
                                 (uint16_t)(OLED_I2C_ADDRESS_7BIT << 1U),
                                 tx_data,
                                 sizeof(tx_data),
                                 OLED_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef OLED_String_Show(uint8_t x, uint8_t page,
                                   const char *text)
{
  HAL_StatusTypeDef hal_status;

  if ((oled_i2c == NULL) || (text == NULL) ||
      (page >= OLED_PAGE_COUNT))
  {
    return HAL_ERROR;
  }

  while ((*text != '\0') && (x <= (OLED_WIDTH - OLED_CHAR_WIDTH)))
  {
    hal_status = OLED_Character_Show(x, page, *text);
    if (hal_status != HAL_OK)
    {
      return hal_status;
    }

    x = (uint8_t)(x + OLED_CHAR_WIDTH);
    ++text;
  }

  return HAL_OK;
}

/**********************************************************
*** OLED整行无闪烁刷新
**********************************************************/
HAL_StatusTypeDef OLED_Line_Show(uint8_t page, const char *text)
{
  uint8_t tx_data[OLED_WIDTH + 1U] = {0U};
  uint8_t x = 0U;
  uint8_t index;
  uint8_t font_index;
  uint8_t character;
  HAL_StatusTypeDef hal_status;

  if ((oled_i2c == NULL) || (text == NULL) ||
      (page >= OLED_PAGE_COUNT))
  {
    return HAL_ERROR;
  }

  /*
   * 先在 RAM 中生成完整的一页像素。字符串以外的区域保持为0，
   * 随后整页一次写入，避免“清空页面”和“重新画字”之间出现空白帧。
   */
  tx_data[0] = OLED_CONTROL_DATA;
  while ((*text != '\0') && (x <= (OLED_WIDTH - OLED_CHAR_WIDTH)))
  {
    character = (uint8_t)(*text);
    if ((character < OLED_ASCII_FIRST) || (character > OLED_ASCII_LAST))
    {
      character = (uint8_t)'?';
    }

    font_index = character - OLED_ASCII_FIRST;
    for (index = 0U; index < OLED_CHAR_WIDTH; ++index)
    {
      tx_data[x + index + 1U] = oled_font_6x8[font_index][index];
    }

    x = (uint8_t)(x + OLED_CHAR_WIDTH);
    ++text;
  }

  hal_status = OLED_Position_Set(0U, page);
  if (hal_status != HAL_OK)
  {
    return hal_status;
  }

  return HAL_I2C_Master_Transmit(oled_i2c,
                                 (uint16_t)(OLED_I2C_ADDRESS_7BIT << 1U),
                                 tx_data,
                                 sizeof(tx_data),
                                 OLED_I2C_TIMEOUT_MS);
}

/**********************************************************
*** OLED显示开关
**********************************************************/
HAL_StatusTypeDef OLED_Display_On(void)
{
  uint8_t command = 0xAFU;
  return OLED_Command_Send(&command, 1U);
}

HAL_StatusTypeDef OLED_Display_Off(void)
{
  uint8_t command = 0xAEU;
  return OLED_Command_Send(&command, 1U);
}
