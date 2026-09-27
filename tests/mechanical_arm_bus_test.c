#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mechanical_arm.h"
#include "motor_bus.h"
static uint32_t tick;
static bool route_busy, motion_busy;
bool ChassisRoute_IsBusy(void) { return route_busy; }
bool ChassisMotion_IsBusy(void) { return motion_busy; }
static UART_HandleTypeDef uart = {.Instance=UART5};
UART_HandleTypeDef huart5;
static uint8_t frame[32];
uint32_t HAL_GetTick(void)
{
  return tick;
}
bool Mecanum_Test_Stop(void)
{
  return true;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  memcpy(frame, p, n);
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  (void)p;
  (void)n;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *u)
{
  (void)u;
  return HAL_OK;
}
int main(void)
{
  MechanicalArm_EventTypeDef e;
  uint8_t other[] = {1, 0x35, 0x6b}, ack[] = {5, 0xf3, 2, 0x6b};
  assert(MotorBus_Init(&uart) == HAL_OK);
  assert(MechanicalArm_Init(&uart) == HAL_OK);
  for (unsigned owner = 0; owner < 2; owner++)
  {
    route_busy = owner == 0; motion_busy = owner == 1;
    assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Disable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Position(MECHANICAL_ARM_AXIS_BASE, 10) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_BASE, 10, 5, 8,
      MECHANICAL_ARM_POSITION_RELATIVE_CURRENT) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Home(MECHANICAL_ARM_AXIS_BASE, 0) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_OriginSet(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(MechanicalArm_Zero(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_BUSY);
    assert(!MechanicalArm_IsBusy());
    assert(MechanicalArm_PositionRead(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM); MechanicalArm_Process();
    assert(!MechanicalArm_IsBusy()); assert(MechanicalArm_ResultGet(&e));
    assert(MechanicalArm_StateRead(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM); MechanicalArm_Process();
    assert(!MechanicalArm_IsBusy()); assert(MechanicalArm_ResultGet(&e));
    assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
    MotorBus_Cancel(MOTOR_BUS_ARM); MechanicalArm_Process();
    assert(!MechanicalArm_IsBusy()); assert(MechanicalArm_ResultGet(&e));
  }
  route_busy = motion_busy = false;
  assert(MotorBus_Submit(MOTOR_BUS_CHASSIS, other, 3, 0, 0, false));
  assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  tick = 10;
  MotorBus_Process();
  assert(frame[0] == 5 && frame[1] == 0xf3);
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  MotorBus_RxBytes(ack, 2);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(!MechanicalArm_ResultGet(&e));
  MotorBus_RxBytes(ack + 2, 2);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
  /* Installed Z: protocol CCW moves down; positive/up must transmit CW.
   * Exercise console and explicit-speed paths with both signs. */
  for (unsigned explicit_speed = 0; explicit_speed < 2; explicit_speed++)
  {
    for (unsigned negative = 0; negative < 2; negative++)
    {
      const int32_t pulses = negative ? -89 : 89;
      const uint8_t position_ack[] = {6, 0xfd, 2, 0x6b};
      assert((explicit_speed ? MechanicalArm_PositionEx(MECHANICAL_ARM_AXIS_Z,
                 pulses, 10, 20, MECHANICAL_ARM_POSITION_RELATIVE_CURRENT)
               : MechanicalArm_Position(MECHANICAL_ARM_AXIS_Z, pulses)) == MECHANICAL_ARM_RESULT_NONE);
      tick += 10;
      MotorBus_Process();
      assert(frame[0] == 6 && frame[1] == 0xfd && frame[2] == negative);
      /* Default motion must carry the selected speed/acceleration on wire;
       * explicit-speed callers retain their own parameters. */
      assert((((unsigned)frame[3] << 8) | frame[4]) == (explicit_speed ? 10U : 500U));
      assert(frame[5] == (explicit_speed ? 20U : 120U));
      assert(frame[6] == 0 && frame[7] == 0 && frame[8] == 0 && frame[9] == 89);
      MotorBus_TxCpltCallback(&uart);
      MotorBus_RxBytes(position_ack, sizeof(position_ack));
      MotorBus_Process(); MechanicalArm_Process();
      assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
    }
  }
  assert(MechanicalArm_Enable(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10;
  MotorBus_Process();
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  tick += 100;
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
  assert(MechanicalArm_Stop(MECHANICAL_ARM_AXIS_BASE) == MECHANICAL_ARM_RESULT_NONE);
  tick += 10;
  MotorBus_Process();
  assert(frame[1] == 0xfe);
  MotorBus_TxCpltCallback(&uart);
  MotorBus_Process();
  MechanicalArm_Process();
  assert(MechanicalArm_ResultGet(&e) && e.Result == MECHANICAL_ARM_RESULT_OK);
  puts(
      "mechanical_arm_bus_test: unrelated TX cannot complete arm, fragmented own ACK completes OK");
}
