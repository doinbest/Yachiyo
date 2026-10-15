#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mecanum_chassis.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "hwt101_calibration.h"
#include "motor_bus.h"

/* Exercise real heading controller and real asynchronous bus with a HAL boundary. */
static uint32_t tick;
static bool valid, calibration_busy, in_flight;
static HWT101_Angle_t angle;
static unsigned sends, stops;
static uint8_t last_acc;
static bool refresh_imu;
static HAL_StatusTypeDef tx_result;
static UART_HandleTypeDef uart;
static uint8_t sent_address,sent_function;
bool ChassisRoute_IsBusy(void) { return false; }
bool ChassisRoute_IsSubmitting(void) { return false; }
bool GrabTask_IsBusy(void) { return false; }
bool GrabTask_CalibrationActive(void) { return false; }
uint8_t MechanicalArm_IsBusy(void) { return false; }
void ChassisLocalization_Get(ChassisLocalization_Status_t *out) { memset(out,0,sizeof *out); }
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
  if (p[1] == 0xf6) { sends++;last_acc=p[5]; }
  if (p[1] == 0xfe)
    stops++;
  in_flight = tx_result == HAL_OK;
  sent_address=p[0]?p[0]:1;sent_function=p[1];
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
    if(refresh_imu && tick%20==0) {angle.last_update_ms=tick;angle.update_count++;}
    if (in_flight)
    {
      in_flight = false;
      MotorBus_TxCpltCallback(&uart);
      const uint8_t ack[]={sent_address,sent_function,2,0x6b};MotorBus_RxBytes(ack,4);
    }
    MotorBus_Process();
    ChassisMotion_Process();
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
  assert(ChassisMotion_Stop(0));
  assert(Mecanum_IsBusy());
  pump(60);
  assert(!Mecanum_IsBusy());
  assert(Mecanum_RecoveryAfterReset());
}
int main(void)
{
  ChassisMotion_HoldStatus_t status;
  MotorBus_Init(&uart);
  ChassisMotion_Init();
  assert(Mecanum_AckProfile_Set(MECANUM_ACK_NONE));
  calibration_busy=true;
  assert(!Mecanum_Move_Control(10,0,0));
  assert(!Mecanum_Polarity_Move(10));
  assert(!Mecanum_Wheel_Test(MECANUM_WHEEL_FRONT_LEFT,10));
  assert(!Mecanum_Velocity_Request(10,0,0,20));
  assert(!ChassisMotion_VelocityStart(10,0,0));
  assert(!ChassisMotion_HoldStart(10,1000));
  assert(sends==0);
  calibration_busy=false;
  assert(!ChassisMotion_HoldStart(30,1000));
  valid=true;refresh_imu=true;reading(179);
  assert(!ChassisMotion_HoldStart(NAN,1000));
  assert(!ChassisMotion_HoldStart(101,1000));
  assert(!ChassisMotion_HoldStart(30,10001));
  assert(ChassisMotion_HoldStart(30,1000));
  assert(sends==0 && Mecanum_IsBusy());
  pump(60);assert(sends==4 && last_acc==20);
  assert(!ChassisMotion_VelocityStart(20,0,0));
  reading(-179);pump(20);ChassisMotion_HoldStatusGet(&status);
  /* Default P=.1, D=0: the wrapped -2 degree error produces -.2 rad/s. */
  assert(status.active && fabsf(status.output_rad_s+.2f)<.0001f);
  reading(-150);pump(20);ChassisMotion_HoldStatusGet(&status);
  assert(fabsf(status.output_rad_s+.5f)<.0001f);
  valid=false;ChassisMotion_Process();ChassisMotion_HoldStatusGet(&status);
  assert(!status.active && !strcmp(status.reason,"imu_invalid"));
  assert(Mecanum_IsBusy());pump(60);assert(!Mecanum_IsBusy());
  assert(Mecanum_RecoveryAfterReset());
  valid=true;reading(12);assert(ChassisMotion_HoldStart(-30,1000));
  pump(1060);ChassisMotion_HoldStatusGet(&status);
  assert(!status.active && !strcmp(status.reason,"complete"));
  assert(!Mecanum_IsBusy());assert(Mecanum_RecoveryAfterReset());
  reading(12);assert(ChassisMotion_HoldStart(0,1000));pump(60);
  refresh_imu=false;pump(301);ChassisMotion_HoldStatusGet(&status);
  assert(!status.active && !strcmp(status.reason,"imu_invalid"));
  pump(60);assert(Mecanum_RecoveryAfterReset());
  valid=false;
  assert(ChassisMotion_VelocityStart(30,0,0));pump(61000);
  assert(ChassisMotion_IsBusy() && Mecanum_IsBusy() && last_acc==20);
  unsigned before=sends;
  assert(ChassisMotion_VelocityUpdate(-30,20,.1f));pump(60);
  assert(sends>before);
  stopped();assert(!ChassisMotion_IsBusy());
  before=sends;pump(200);assert(sends==before);
  assert(ChassisMotion_VelocityStart(30,0,0));pump(60);
  tx_result=HAL_ERROR;assert(ChassisMotion_Stop(0));pump(60);
  assert(Mecanum_IsBusy());Mecanum_Status_t state;Mecanum_StatusGet(&state);
  assert(state.stage==MECANUM_STAGE_FAULT);
  tx_result=HAL_OK;stopped();
  assert(Mecanum_Move_Control(10,0,0));pump(60);
  pump(60000);assert(Mecanum_IsBusy());stopped();
  puts("mecanum_heading_test: shared task + real async bus, capture/wrap/clamp, "
       "freshness, grade20, indefinite >60s, update, stop and failed stop OK");
  return 0;
}
