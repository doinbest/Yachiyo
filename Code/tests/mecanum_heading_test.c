#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mecanum_chassis.h"
#include "hwt101_calibration.h"
#include "motor_bus.h"

/* Exercise real heading controller and real asynchronous bus with a HAL boundary. */
static uint32_t tick;
static bool valid, calibration_busy, in_flight;
static HWT101_Angle_t angle;
static unsigned sends, stops;
static HAL_StatusTypeDef tx_result;
static UART_HandleTypeDef uart;
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisMotion_IsBusy(void) { return false; }
uint32_t HAL_GetTick(void)
{
  return tick;
}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *out)
{
  if (!valid || calibration_busy || (uint32_t)(tick - angle.last_update_ms) > 300U)
    return false;
  *out = angle;
  return true;
}
bool HWT101_Cal_IsBusy(void)
{
  return calibration_busy;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
  (void)u;
  (void)n;
  if (p[1] == 0xf6)
    sends++;
  if (p[1] == 0xfe)
    stops++;
  in_flight = tx_result == HAL_OK;
  return tx_result;
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
#include "../template/Hardware/motor_bus.c"
#include "../template/App/mecanum_chassis.c"
static void pump(unsigned ms)
{
  while (ms--)
  {
    tick++;
    if (in_flight)
    {
      in_flight = false;
      MotorBus_TxCpltCallback(&uart);
    }
    MotorBus_Process();
    Mecanum_Process();
  }
}
static void reading(float yaw)
{
  angle.yaw = yaw;
  angle.last_update_ms = tick;
  angle.update_count++;
}
static void stopped(void)
{
  assert(Mecanum_Test_Stop());
  assert(Mecanum_IsBusy());
  pump(60);
  assert(!Mecanum_IsBusy());
  assert(Mecanum_RecoveryAfterReset());
}
int main(void)
{
  Mecanum_HeadingTestStatus_t status;
  Mecanum_HeadingPid_t pid;
  MotorBus_Init(&uart);
  assert(Mecanum_AckProfile_Set(MECANUM_ACK_NONE));
  Mecanum_HeadingPid_Init(&pid, 0.02f, 0, 0, 0.1f, 0, 0.15f);
  calibration_busy = true;
  assert(!Mecanum_Move_Control(10, 0, 0));
  assert(!Mecanum_Polarity_Move(10));
  assert(!Mecanum_Wheel_Test(MECANUM_WHEEL_FRONT_LEFT, 10));
  assert(!Mecanum_Velocity_Control(10, 0, 0));
  assert(!Mecanum_Velocity_Start(10, 0, 0));
  assert(!Mecanum_HeadingTest_Start(10, 1000));
  assert(!Mecanum_Heading_Hold_Step(&pid, 0, 10, 0));
  assert(!pid.initialized && sends == 0);
  stopped();
  calibration_busy = false;
  assert(!Mecanum_HeadingTest_Start(30, 1000));
  valid = true;
  reading(179);
  assert(!Mecanum_HeadingTest_Start(NAN, 1000));
  assert(!Mecanum_HeadingTest_Start(101, 1000));
  assert(!Mecanum_HeadingTest_Start(30, 10001));
  assert(Mecanum_HeadingTest_Start(30, 1000));
  assert(sends == 0 && Mecanum_IsBusy());
  pump(60);
  assert(sends == 4);
  assert(!Mecanum_Velocity_Start(20, 0, 0));
  tick += 50;
  reading(-179);
  Mecanum_Velocity_Process();
  Mecanum_HeadingTest_StatusGet(&status);
  assert(status.active && fabsf(status.output_rad_s + 0.04f) < .0001f);
  unsigned n = sends;
  Mecanum_Velocity_Process();
  assert(sends == n);
  tick += 100;
  reading(-150);
  Mecanum_Velocity_Process();
  Mecanum_HeadingTest_StatusGet(&status);
  assert(fabsf(status.output_rad_s + .15f) < .0001f);
  valid = false;
  Mecanum_Velocity_Process();
  Mecanum_HeadingTest_StatusGet(&status);
  assert(!status.active && !strcmp(status.reason, "imu_invalid"));
  assert(Mecanum_IsBusy());
  pump(60);
  assert(!Mecanum_IsBusy());
  assert(Mecanum_RecoveryAfterReset());
  valid = true;
  tick = UINT32_MAX - 100;
  reading(0);
  assert(Mecanum_HeadingTest_Start(-30, 1000));
  pump(60);
  tick = 900;
  reading(0);
  Mecanum_Velocity_Process();
  Mecanum_HeadingTest_StatusGet(&status);
  assert(!status.active && !strcmp(status.reason, "complete"));
  pump(60);
  assert(!Mecanum_IsBusy());
  assert(Mecanum_RecoveryAfterReset());
  reading(12);
  assert(Mecanum_HeadingTest_Start(0, 1000));
  pump(60);
  tick += 301;
  Mecanum_Velocity_Process();
  Mecanum_HeadingTest_StatusGet(&status);
  assert(!status.active && !strcmp(status.reason, "imu_invalid"));
  pump(60);
  assert(Mecanum_RecoveryAfterReset());
  assert(Mecanum_Velocity_Start(30, 0, 0));
  pump(60);
  Mecanum_VelocityRefresh_Stop();
  assert(Mecanum_IsBusy());
  tx_result = HAL_ERROR;
  assert(Mecanum_Test_Stop());
  pump(60);
  assert(Mecanum_IsBusy());
  Mecanum_Status_t state;
  Mecanum_StatusGet(&state);
  assert(state.stage == MECANUM_STAGE_FAULT);
  tx_result = HAL_OK;
  stopped();
  assert(Mecanum_Move_Control(10, 0, 0));
  pump(60);
  tick += 60000;
  Mecanum_Velocity_Process();
  assert(Mecanum_IsBusy());
  stopped();
  puts("mecanum_heading_test: real async bus, calibration guard, wrap, clamp, freshness, timed "
       "stop and failed stop OK");
}
