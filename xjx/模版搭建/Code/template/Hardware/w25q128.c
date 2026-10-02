#include "w25q128.h"

#include <stddef.h>
#include <string.h>

#define W25Q128_CS_PORT GPIOA
#define W25Q128_CS_PIN GPIO_PIN_4
#define W25Q128_TRANSFER_TIMEOUT_MS 20U

HAL_StatusTypeDef W25Q128_ReadJedecId(SPI_HandleTypeDef *hspi, uint8_t id[3])
{
  uint8_t TxData[4] = {0x9FU, 0xFFU, 0xFFU, 0xFFU};
  uint8_t RxData[4] = {0};
  HAL_StatusTypeDef Status;

  if ((hspi == NULL) || (id == NULL))
  {
    return HAL_ERROR;
  }

  /* 命令和后续三个读字节必须处于同一次片选低电平内。 */
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET);
  Status = HAL_SPI_TransmitReceive(hspi, TxData, RxData, sizeof(TxData),
                                   W25Q128_TRANSFER_TIMEOUT_MS);
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_SET);

  if (Status == HAL_OK)
  {
    /* 发送0x9F时收到的首字节无效，后面才是JEDEC ID。 */
    id[0] = RxData[1];
    id[1] = RxData[2];
    id[2] = RxData[3];
  }
  return Status;
}

HAL_StatusTypeDef W25Q128_ReadDeviceId(SPI_HandleTypeDef *hspi, uint8_t id[2])
{
  uint8_t TxData[6] = {0x90U, 0x00U, 0x00U, 0x00U, 0xFFU, 0xFFU};
  uint8_t RxData[6] = {0};
  HAL_StatusTypeDef Status;

  if ((hspi == NULL) || (id == NULL))
  {
    return HAL_ERROR;
  }

  /* 0x90 后发送三字节地址 000000，再读取两个 ID 字节。 */
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET);
  Status = HAL_SPI_TransmitReceive(hspi, TxData, RxData, sizeof(TxData),
                                  W25Q128_TRANSFER_TIMEOUT_MS);
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_SET);

  if (Status == HAL_OK)
  {
    id[0] = RxData[4];
    id[1] = RxData[5];
  }
  return Status;
}

static HAL_StatusTypeDef W25Q128_Transfer(SPI_HandleTypeDef *hspi, uint8_t *tx, uint8_t *rx, uint16_t size)
{
  HAL_StatusTypeDef status;
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(hspi, tx, rx, size, W25Q128_TRANSFER_TIMEOUT_MS);
  HAL_GPIO_WritePin(W25Q128_CS_PORT, W25Q128_CS_PIN, GPIO_PIN_SET);
  return status;
}

HAL_StatusTypeDef W25Q128_ReadStatus(SPI_HandleTypeDef *hspi, uint8_t *status)
{
  uint8_t tx[2] = {0x05, 0xFF}, rx[2];
  HAL_StatusTypeDef result;
  if (!hspi || !status) return HAL_ERROR;
  result = W25Q128_Transfer(hspi, tx, rx, 2);
  if (result == HAL_OK) *status = rx[1];
  return result;
}

static void W25Q128_Address(uint8_t *tx, uint8_t command, uint32_t address)
{
  tx[0] = command; tx[1] = (uint8_t)(address >> 16);
  tx[2] = (uint8_t)(address >> 8); tx[3] = (uint8_t)address;
}

HAL_StatusTypeDef W25Q128_ReadData(SPI_HandleTypeDef *hspi, uint32_t address, uint8_t *data, uint16_t length)
{
  uint8_t tx[260], rx[260];
  HAL_StatusTypeDef result;
  if (!hspi || !data || !length || length > 256 || address >= 0x1000000UL ||
      length > 0x1000000UL - address) return HAL_ERROR;
  memset(tx, 0xFF, sizeof(tx));
  W25Q128_Address(tx, 0x03, address);
  result = W25Q128_Transfer(hspi, tx, rx, length + 4U);
  if (result == HAL_OK) memcpy(data, rx + 4, length);
  return result;
}

static HAL_StatusTypeDef W25Q128_WriteEnable(SPI_HandleTypeDef *hspi)
{
  uint8_t sr, tx = 0x06, rx;
  HAL_StatusTypeDef result = W25Q128_ReadStatus(hspi, &sr);
  if (result != HAL_OK) return result;
  if (sr & 1U) return HAL_BUSY;
  result = W25Q128_Transfer(hspi, &tx, &rx, 1);
  if (result != HAL_OK) return result;
  result = W25Q128_ReadStatus(hspi, &sr);
  if (result != HAL_OK) return result;
  return ((sr & 3U) == 2U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef W25Q128_EraseSectorStart(SPI_HandleTypeDef *hspi, uint32_t address)
{
  uint8_t tx[4], rx[4];
  HAL_StatusTypeDef result;
  if (!hspi || address >= 0x1000000UL || (address & 0xFFFU)) return HAL_ERROR;
  result = W25Q128_WriteEnable(hspi);
  if (result != HAL_OK) return result;
  W25Q128_Address(tx, 0x20, address);
  return W25Q128_Transfer(hspi, tx, rx, 4);
}

HAL_StatusTypeDef W25Q128_ProgramStart(SPI_HandleTypeDef *hspi, uint32_t address, const uint8_t *data, uint16_t length)
{
  uint8_t tx[260], rx[260];
  HAL_StatusTypeDef result;
  if (!hspi || !data || !length || length > 256 || address >= 0x1000000UL ||
      (address & 255U) + length > 256U) return HAL_ERROR;
  result = W25Q128_WriteEnable(hspi);
  if (result != HAL_OK) return result;
  W25Q128_Address(tx, 0x02, address);
  memcpy(tx + 4, data, length);
  return W25Q128_Transfer(hspi, tx, rx, length + 4U);
}
