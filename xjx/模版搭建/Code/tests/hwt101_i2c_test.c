#include "hwt101_i2c.h"
#include <assert.h>
#include <stdio.h>

static uint32_t tick;
static unsigned int read_count, write_count, probe_count;
static unsigned int reinit_count, deinit_count;
static bool busy_until_reinit, reinit_fails;
static HAL_StatusTypeDef result = HAL_ERROR;
static I2C_HandleTypeDef *expected_bus;
static uint16_t expected_reg = 0x3fU;
static uint8_t read_low = 0U, read_high = 0xc0U;
static uint8_t expected_write_low, expected_write_high;

uint32_t HAL_GetTick(void) { return tick; }
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *bus, uint16_t addr, uint32_t trials, uint32_t timeout)
{
  (void)bus;
  assert(addr == 0xa0 && trials == 1U && timeout == 5U);
  probe_count++;
  return busy_until_reinit ? HAL_BUSY : result;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus)
{ assert(bus==expected_bus); deinit_count++; return HAL_OK; }
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus)
{
  assert(bus==expected_bus); reinit_count++;
  if(reinit_fails)return HAL_ERROR;
  busy_until_reinit=false; return HAL_OK;
}
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *bus)
{ assert(bus==expected_bus); return 0x20U; }
uint32_t HAL_I2C_GetState(I2C_HandleTypeDef *bus)
{ assert(bus==expected_bus); return 0x24U; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                   uint16_t size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == expected_bus);
  assert(addr == 0xa0 && reg == expected_reg && size == 1U && length == 2U && timeout == 5U);
  read_count++;
  /* Even a failed bus transfer can have changed its receive buffer. */
  data[0] = read_low;
  data[1] = read_high;
  return result;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t addr, uint16_t reg,
                                    uint16_t size, uint8_t *data, uint16_t length, uint32_t timeout)
{
  assert(bus == expected_bus);
  assert(addr == 0xa0 && reg == expected_reg && size == 1U && length == 2U && timeout == 5U);
  assert(data[0] == expected_write_low && data[1] == expected_write_high);
  write_count++;
  return result;
}

static void test_uninitialized_register_access(void)
{
  uint16_t value = 0x7654U;

  assert(HWT101_ReadRegister(0x61U, &value) == HAL_ERROR);
  assert(HWT101_ReadRegister(0x61U, NULL) == HAL_ERROR);
  assert(HWT101_WriteRegister(0x69U, 0xb588U) == HAL_ERROR);
  assert(value == 0x7654U);
  assert(read_count == 0U && write_count == 0U && probe_count == 0U);
}

static void test_angle_polling_unchanged(void)
{
  I2C_HandleTypeDef bus = {0};
  HWT101_Angle_t angle = {0};
  HWT101_Status_t status;

  expected_bus = &bus;
  assert(!HWT101_Init(&bus));
  assert(!HWT101_Angle_Get(&angle));
  assert(probe_count == 1U && read_count == 0U);

  result = HAL_OK;
  tick = 999U;
  HWT101_Process();
  assert(probe_count == 1U && read_count == 0U);
  tick = 1000U;
  HWT101_Process();
  assert(probe_count == 2U && read_count == 1U);
  assert(HWT101_Angle_Get(&angle));
  assert(angle.yaw == -90.0f && angle.update_count == 1U && angle.last_update_ms == 1000U);

  assert(HWT101_Angle_Get(&angle));
  assert(probe_count == 2U && read_count == 1U);
  HWT101_Process();
  assert(read_count == 1U);
  tick += 10U;
  result = HAL_ERROR;
  HWT101_Process();
  assert(read_count == 2U);
  assert(HWT101_Angle_Get(&angle));
  assert(angle.update_count == 1U && angle.last_update_ms == 1000U);
  assert(HWT101_Status_Get(&status));
  assert(status.valid_read_count == 1U && status.i2c_error_count == 1U);

  tick = 1300U;
  assert(HWT101_Angle_Is_Fresh(&angle, HWT101_DATA_FRESH_MS));
  tick = 1301U;
  assert(!HWT101_Angle_Is_Fresh(&angle, HWT101_DATA_FRESH_MS));
}

static void test_register_access(void)
{
  I2C_HandleTypeDef bus = {0};
  HWT101_Status_t status;
  HWT101_Angle_t angle = {0};
  uint16_t value = 0U;
  unsigned int reads_before, writes_before, probes_before;
  const HAL_StatusTypeDef failures[] = {HAL_ERROR, HAL_BUSY, HAL_TIMEOUT};
  unsigned int i;

  expected_bus = &bus;
  result = HAL_OK;
  assert(HWT101_Init(&bus));
  expected_reg = 0x61U;
  read_low = 0x34U;
  read_high = 0xabU;
  reads_before = read_count;
  assert(HWT101_ReadRegister(0x61U, &value) == HAL_OK);
  assert(value == 0xab34U && read_count == reads_before + 1U);
  assert(!HWT101_Angle_Get(&angle));
  assert(HWT101_Status_Get(&status));
  assert(status.valid_read_count == 0U && status.i2c_error_count == 0U);
  assert(HWT101_Is_Ready());

  expected_reg = 0x69U;
  expected_write_low = 0x88U;
  expected_write_high = 0xb5U;
  writes_before = write_count;
  assert(HWT101_WriteRegister(0x69U, 0xb588U) == HAL_OK);
  assert(write_count == writes_before + 1U);
  assert(HWT101_Is_Ready());

  reads_before = read_count;
  assert(HWT101_ReadRegister(0x61U, NULL) == HAL_ERROR);
  assert(read_count == reads_before);
  assert(HWT101_Status_Get(&status));
  assert(status.i2c_error_count == 0U && HWT101_Is_Ready());

  for (i = 0U; i < sizeof(failures) / sizeof(failures[0]); ++i)
  {
    result = HAL_OK;
    assert(HWT101_Init(&bus));
    result = failures[i];
    expected_reg = 0x61U;
    value = 0x7654U;
    reads_before = read_count;
    assert(HWT101_ReadRegister(0x61U, &value) == failures[i]);
    assert(value == 0x7654U && read_count == reads_before + 1U);
    assert(HWT101_Status_Get(&status));
    assert(status.i2c_error_count == 1U && status.valid_read_count == 0U);
    assert(!HWT101_Is_Ready());

    result = HAL_OK;
    assert(HWT101_Init(&bus));
    result = failures[i];
    expected_reg = 0x69U;
    writes_before = write_count;
    assert(HWT101_WriteRegister(0x69U, 0xb588U) == failures[i]);
    assert(write_count == writes_before + 1U);
    assert(HWT101_Status_Get(&status));
    assert(status.i2c_error_count == 1U && status.valid_read_count == 0U);
    assert(!HWT101_Is_Ready());
  }

  /* Register transfer errors use the same one-second probe backoff as angle reads. */
  probes_before = probe_count;
  reads_before = read_count;
  result = HAL_OK;
  tick += 999U;
  HWT101_Process();
  assert(probe_count == probes_before && read_count == reads_before);
  tick += 1U;
  expected_reg = 0x3fU;
  read_low = 0U;
  read_high = 0xc0U;
  HWT101_Process();
  assert(probe_count == probes_before + 1U && read_count == reads_before + 1U);
  assert(HWT101_Angle_Get(&angle));
  assert(angle.yaw == -90.0f && angle.update_count == 1U);

  /* Removing the bus handle blocks both APIs without changing the caller's value. */
  assert(!HWT101_Init(NULL));
  reads_before = read_count;
  writes_before = write_count;
  value = 0x7654U;
  assert(HWT101_ReadRegister(0x61U, &value) == HAL_ERROR);
  assert(HWT101_WriteRegister(0x69U, 0xb588U) == HAL_ERROR);
  assert(value == 0x7654U);
  assert(read_count == reads_before && write_count == writes_before);
}

/* Repeated address probes cannot clear a stuck host peripheral by themselves. */
static void test_stuck_busy_recovers_without_restarting_angle_history(void)
{
  I2C_HandleTypeDef bus={0}; HWT101_Angle_t before,after;
  expected_bus=&bus;expected_reg=0x3fU;result=HAL_OK;
  tick=5000; assert(HWT101_Init(&bus)); HWT101_Process();
  assert(HWT101_Angle_Get(&before));
  result=HAL_BUSY; tick+=10;HWT101_Process();assert(!HWT101_Is_Ready());
  busy_until_reinit=true;result=HAL_OK;
  tick+=1000;HWT101_Process();assert(reinit_count==0);
  tick+=1000;HWT101_Process();
  assert(HWT101_Is_Ready());
  assert(reinit_count==1 && deinit_count==1);
  assert(HWT101_Angle_Get(&after) && after.update_count==before.update_count+1);
  assert(after.last_update_ms==tick);
  /* A transient BUSY that clears before the second probe needs no reinit. */
  result=HAL_BUSY;tick+=10;HWT101_Process();
  tick+=1000;HWT101_Process();
  result=HAL_OK;tick+=1000;HWT101_Process();
  assert(HWT101_Is_Ready() && reinit_count==1);
  /* A disconnected/NACK target is not a reason to reset the host peripheral. */
  result=HAL_ERROR;tick+=10;HWT101_Process();
  tick+=1000;HWT101_Process();tick+=1000;HWT101_Process();
  assert(!HWT101_Is_Ready() && reinit_count==1);
  /* Failed host reinit leaves qualification unavailable, and can be retried. */
  result=HAL_OK;busy_until_reinit=true;reinit_fails=true;
  tick+=1000;HWT101_Process();tick+=1000;HWT101_Process();
  assert(!HWT101_Is_Ready() && reinit_count==2);
  reinit_fails=false;tick+=1000;HWT101_Process();tick+=1000;HWT101_Process();
  assert(HWT101_Is_Ready() && reinit_count==3);
  HWT101_Status_t s;assert(HWT101_Status_Get(&s));
  assert(s.reinit_count==3 && s.reinit_hal==HAL_OK && s.probe_fail_count>=6);
  assert(s.hal_state==0x24U && s.hal_error==0x20U);
}

int main(void)
{
  test_uninitialized_register_access();
  test_angle_polling_unchanged();
  test_register_access();
  test_stuck_busy_recovers_without_restarting_angle_history();
  puts("HWT101 bounded register access and angle-cache tests passed");
  return 0;
}
