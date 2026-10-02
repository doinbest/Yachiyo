#include "hwt101_i2c.h"

#define HWT101_SAMPLE_INTERVAL_MS 10U
#define HWT101_PROBE_RETRY_MS 1000U
#define HWT101_TRANSFER_TIMEOUT_MS 5U

static I2C_HandleTypeDef *hwt101_i2c = NULL;
static bool hwt101_ready = false;
static HWT101_Angle_t hwt101_latest_angle = {0};
static HWT101_Status_t hwt101_status = {0};
static uint32_t hwt101_probe_tick;
static uint32_t hwt101_read_tick;

static int16_t HWT101_Bytes_To_Int16(uint8_t low_byte, uint8_t high_byte)
{
  uint16_t raw_data;

  raw_data = (uint16_t)low_byte | ((uint16_t)high_byte << 8U);
  return (int16_t)raw_data;
}

bool HWT101_Init(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == NULL)
  {
    hwt101_i2c = NULL;
    hwt101_ready = false;
    return false;
  }

  hwt101_i2c = hi2c;
  hwt101_latest_angle.roll = 0.0f;
  hwt101_latest_angle.pitch = 0.0f;
  hwt101_latest_angle.yaw = 0.0f;
  hwt101_latest_angle.update_count = 0U;
  hwt101_latest_angle.last_update_ms = 0U;
  hwt101_status.valid_read_count = 0U;
  hwt101_status.i2c_error_count = 0U;
  hwt101_status.last_update_ms = 0U;
  hwt101_probe_tick = HAL_GetTick();
  hwt101_read_tick = HAL_GetTick() - HWT101_SAMPLE_INTERVAL_MS;
  hwt101_ready = (HAL_I2C_IsDeviceReady(hwt101_i2c,
                                         (uint16_t)(HWT101_I2C_ADDRESS << 1U),
                                         1U, HWT101_TRANSFER_TIMEOUT_MS) == HAL_OK);
  return hwt101_ready;
}

HAL_StatusTypeDef HWT101_ReadRegister(uint8_t reg, uint16_t *value)
{
  uint8_t data[2] = {0};
  HAL_StatusTypeDef hal_status;

  if ((hwt101_i2c == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  hal_status = HAL_I2C_Mem_Read(hwt101_i2c,
                                (uint16_t)(HWT101_I2C_ADDRESS << 1U),
                                reg, I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                                HWT101_TRANSFER_TIMEOUT_MS);
  if (hal_status != HAL_OK)
  {
    ++hwt101_status.i2c_error_count;
    hwt101_ready = false;
    hwt101_probe_tick = HAL_GetTick();
    return hal_status;
  }

  *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
  return HAL_OK;
}

HAL_StatusTypeDef HWT101_WriteRegister(uint8_t reg, uint16_t value)
{
  uint8_t data[2];
  HAL_StatusTypeDef hal_status;

  if (hwt101_i2c == NULL)
  {
    return HAL_ERROR;
  }

  data[0] = (uint8_t)(value & 0xffU);
  data[1] = (uint8_t)(value >> 8U);
  hal_status = HAL_I2C_Mem_Write(hwt101_i2c,
                                 (uint16_t)(HWT101_I2C_ADDRESS << 1U),
                                 reg, I2C_MEMADD_SIZE_8BIT, data, sizeof(data),
                                 HWT101_TRANSFER_TIMEOUT_MS);
  if (hal_status != HAL_OK)
  {
    ++hwt101_status.i2c_error_count;
    hwt101_ready = false;
    hwt101_probe_tick = HAL_GetTick();
  }
  return hal_status;
}

void HWT101_Process(void)
{
  uint8_t raw_angle[2] = {0};
  HAL_StatusTypeDef hal_status;
  int16_t raw_yaw;
  uint32_t now = HAL_GetTick();

  if (hwt101_i2c == NULL)
  {
    return;
  }
  if (!hwt101_ready)
  {
    if ((uint32_t)(now - hwt101_probe_tick) < HWT101_PROBE_RETRY_MS)
    {
      return;
    }
    hwt101_probe_tick = now;
    hwt101_ready = (HAL_I2C_IsDeviceReady(hwt101_i2c,
                        (uint16_t)(HWT101_I2C_ADDRESS << 1U), 1U, HWT101_TRANSFER_TIMEOUT_MS) == HAL_OK);
    if (!hwt101_ready)
    {
      return;
    }
  }
  if ((uint32_t)(now - hwt101_read_tick) < HWT101_SAMPLE_INTERVAL_MS)
  {
    return;
  }
  hwt101_read_tick = now;

  hal_status = HAL_I2C_Mem_Read(hwt101_i2c,
                                (uint16_t)(HWT101_I2C_ADDRESS << 1U),
                                HWT101_ANGLE_REGISTER,
                                I2C_MEMADD_SIZE_8BIT,
                                raw_angle,
                                sizeof(raw_angle),
                                HWT101_TRANSFER_TIMEOUT_MS);
  if (hal_status != HAL_OK)
  {
    ++hwt101_status.i2c_error_count;
    hwt101_ready = false;
    hwt101_probe_tick = now;
    return;
  }

  raw_yaw = HWT101_Bytes_To_Int16(raw_angle[0], raw_angle[1]);

  /* HWT101只测量Z轴；保留的roll/pitch字段不代表有效测量值。 */
  hwt101_latest_angle.yaw = (float)raw_yaw * (180.0f / 32768.0f);
  ++hwt101_latest_angle.update_count;
  hwt101_latest_angle.last_update_ms = HAL_GetTick();
  hwt101_status.valid_read_count = hwt101_latest_angle.update_count;
  hwt101_status.last_update_ms = hwt101_latest_angle.last_update_ms;

}

bool HWT101_Angle_Get(HWT101_Angle_t *angle)
{
  if ((angle == NULL) || (hwt101_latest_angle.update_count == 0U))
  {
    return false;
  }

  *angle = hwt101_latest_angle;
  return true;
}

bool HWT101_Angle_Is_Fresh(const HWT101_Angle_t *angle,
                           uint32_t max_age_ms)
{
  if ((angle == NULL) || (angle->update_count == 0U))
  {
    return false;
  }

  return ((uint32_t)(HAL_GetTick() - angle->last_update_ms) <= max_age_ms);
}

bool HWT101_Status_Get(HWT101_Status_t *status)
{
  if (status == NULL)
  {
    return false;
  }

  *status = hwt101_status;
  return true;
}

bool HWT101_Is_Ready(void)
{
  return (hwt101_i2c != NULL) && hwt101_ready;
}
