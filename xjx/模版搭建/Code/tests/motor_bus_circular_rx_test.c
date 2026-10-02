#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mechanical_arm.h"
#include "motor_bus.h"

static UART_HandleTypeDef uart = {.Instance=UART5};
UART_HandleTypeDef huart5;
static uint32_t tick;
static uint8_t *rx_buffer, frame[32];
static uint16_t rx_size, rx_position;
static unsigned rx_starts, stop_calls;
static bool rx_active;

uint32_t HAL_GetTick(void) { return tick; }
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
bool Mecanum_Test_Stop(void) { return true; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  assert(u == &uart && n <= sizeof(frame));
  memcpy(frame, p, n);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  assert(u == &uart);
  rx_starts++;
  /* STM32 HAL keeps RxState BUSY_RX after circular DMA IDLE/HT/TC. */
  if (rx_active)
    return HAL_BUSY;
  rx_active = true;
  rx_buffer = p;
  rx_size = n;
  rx_position = 0;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u)
{
  assert(u == &uart);
  stop_calls++;
  rx_active = false;
  return HAL_OK;
}
static void receive_bytes(const uint8_t *data, unsigned length, bool idle)
{
  assert(rx_active);
  for (unsigned i = 0; i < length; i++)
  {
    rx_buffer[rx_position++] = data[i];
    if (rx_position == rx_size / 2 || rx_position == rx_size)
      MotorBus_RxEventCallback(&uart, rx_position);
    if (rx_position == rx_size)
      rx_position = 0;
  }
  if (idle)
    MotorBus_RxEventCallback(&uart, rx_position ? rx_position : rx_size);
}
static void process(void)
{
  MotorBus_Process();
  MechanicalArm_Process();
}
static void begin(bool enable)
{
  tick += 10;
  assert((enable ? MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE)
                 : MechanicalArm_Disable(MECHANICAL_ARM_AXIS_BASE)) == MECHANICAL_ARM_RESULT_NONE);
  process();
  assert(frame[0] == 5 && frame[1] == 0xf3 && frame[3] == (enable ? 1 : 0));
  MotorBus_TxCpltCallback(&uart);
  process();
}
static void expect_ok(void)
{
  MechanicalArm_EventTypeDef e;
  process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
  assert(!MotorBus_IsQuarantined() && !MechanicalArm_IsBusy());
  assert(!MechanicalArm_ResultGet(&e));
}
int main(void)
{
  MechanicalArm_EventTypeDef e;
  const uint8_t ack[] = {5, 0xf3, 2, 0x6b};
  const uint8_t noise = 0;
  uint8_t burst[80];
  assert(MotorBus_Init(&uart) == HAL_OK);
  assert(MechanicalArm_Init(&uart) == HAL_OK);
  for (unsigned i = 0; i < 30; i++)
  {
    begin((i & 1U) != 0);
    /* Duplicate IDLE must not replay the previous command's ACK. */
    if (i > 0)
    {
      MotorBus_RxEventCallback(&uart, rx_position ? rx_position : rx_size);
      process();
      assert(!MechanicalArm_ResultGet(&e));
    }
    receive_bytes(ack, 2, true);
    process();
    assert(!MechanicalArm_ResultGet(&e));
    receive_bytes(ack + 2, 2, true);
    expect_ok();
  }
  assert(rx_starts == 1 && stop_calls == 0);
  /* ACK split across the physical DMA end, including duplicate TC/IDLE. */
  while (rx_position != rx_size - 2)
  {
    receive_bytes(&noise, 1, true);
    process();
  }
  begin(true);
  receive_bytes(ack, 2, true);
  process();
  assert(!MechanicalArm_ResultGet(&e));
  receive_bytes(ack + 2, 2, true);
  expect_ok();
  /* More than one buffer without IDLE: HT/TC deliver incremental data. */
  memset(burst, 0, sizeof(burst));
  memcpy(burst + 60, ack, sizeof(ack));
  begin(false);
  receive_bytes(burst, sizeof(burst), true);
  expect_ok();
  begin(true);
  MotorBus_RxEventCallback(&uart, rx_position);
  process();
  assert(!MechanicalArm_ResultGet(&e));
  receive_bytes(ack, sizeof(ack), true);
  expect_ok();
  assert(rx_starts == 1 && stop_calls == 0);
  /* A genuine UART error still locks ordinary traffic; restart resets cursor. */
  begin(false);
  MotorBus_ErrorCallback(&uart);
  process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_TX_ERROR);
  assert(MotorBus_IsQuarantined() && rx_starts == 2 && stop_calls == 1);
  assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_TX_ERROR);
  assert(MotorBus_RecoverAfterReset()); /* Host model only, no hardware reset. */
  begin(true);
  receive_bytes(ack, sizeof(ack), true);
  expect_ok();
  begin(false);
  tick += MOTOR_BUS_TIMEOUT_MS;
  process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
  assert(MotorBus_IsQuarantined());
  puts("motor_bus_circular_rx_test: repeated enable/disable, fragments, duplicate events, wrap, continuous RX, error restart and timeout PASS");
}
