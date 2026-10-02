/* Exercise the real driver with a simulated SPI transaction and CS pin. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "w25q128.h"

GPIO_TypeDef test_gpioa;
static SPI_HandleTypeDef bus;
static GPIO_PinState cs = GPIO_PIN_SET;
static unsigned cs_writes, transfers;
static HAL_StatusTypeDef transfer_result = HAL_OK;
static uint8_t reply[6];
static uint8_t command;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  assert(port == GPIOA && pin == GPIO_PIN_4);
  assert(state != cs);
  cs = state;
  cs_writes++;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
    uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout)
{
  const uint8_t jedec[] = {0x9F, 0xFF, 0xFF, 0xFF};
  const uint8_t device[] = {0x90, 0, 0, 0, 0xFF, 0xFF};
  const uint8_t *expected = command == 0x9F ? jedec : device;
  unsigned expected_size = command == 0x9F ? sizeof(jedec) : sizeof(device);
  assert(hspi == &bus && cs == GPIO_PIN_RESET);
  assert(size == expected_size && timeout == 20);
  assert(memcmp(tx, expected, size) == 0);
  memcpy(rx, reply, size); /* Even a failing HAL call may touch RX. */
  transfers++;
  return transfer_result;
}

static void test_reader(HAL_StatusTypeDef (*read_id)(SPI_HandleTypeDef *, uint8_t *),
                        uint8_t cmd, unsigned id_size, unsigned offset)
{
  uint8_t id[3] = {0x11, 0x22, 0x33};
  const uint8_t sentinel[3] = {0x11, 0x22, 0x33};
  const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};

  command = cmd;
  cs_writes = 0;
  transfers = 0;
  transfer_result = HAL_OK;
  memset(reply, 0xA5, sizeof(reply));
  reply[offset] = 0xEF;
  reply[offset + 1] = cmd == 0x9F ? 0x40 : 0x17;
  if (id_size == 3) reply[offset + 2] = 0x18;

  assert(read_id(NULL, id) == HAL_ERROR);
  assert(read_id(&bus, NULL) == HAL_ERROR);
  assert(cs_writes == 0 && transfers == 0);
  assert(memcmp(id, sentinel, sizeof(id)) == 0);

  assert(read_id(&bus, id) == HAL_OK);
  assert(memcmp(id, reply + offset, id_size) == 0);
  if (id_size == 2) assert(id[2] == sentinel[2]);
  assert(cs == GPIO_PIN_SET && cs_writes == 2 && transfers == 1);

  for (unsigned i = 0; i < sizeof(failures) / sizeof(failures[0]); i++)
  {
    memcpy(id, sentinel, sizeof(id));
    transfer_result = failures[i];
    unsigned before = cs_writes;
    assert(read_id(&bus, id) == failures[i]);
    assert(memcmp(id, sentinel, sizeof(id)) == 0);
    assert(cs == GPIO_PIN_SET && cs_writes == before + 2);
  }

  /* Successful transfer preserves raw data; startup code reports invalid IDs. */
  transfer_result = HAL_OK;
  memset(reply, 0xFF, sizeof(reply));
  assert(read_id(&bus, id) == HAL_OK);
  assert(memcmp(id, reply + offset, id_size) == 0);
  memset(reply, 0, sizeof(reply));
  assert(read_id(&bus, id) == HAL_OK);
  assert(memcmp(id, reply + offset, id_size) == 0);
}

int main(void)
{
  test_reader(W25Q128_ReadJedecId, 0x9F, 3, 1);
  test_reader(W25Q128_ReadDeviceId, 0x90, 2, 4);
  puts("w25q128_test: both ID commands, offsets, CS cleanup and HAL errors OK");
  return 0;
}
