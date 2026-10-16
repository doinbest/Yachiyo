#define main legacy_mecanum_bus_main
#include "mecanum_bus_test.c"
#undef main

int main(void)
{
  MotorBus_Init(&uart);
  /* A completed zero stream transfers between raw picks without clearing
   * the explicit-stop ownership used by manual motion and IMU calibration. */
  assert(Mecanum_Velocity_Request(0,0,0,20));
  assert(!Mecanum_VelocityHandoffReady()); /* Pending zero is insufficient. */
  advance(80);
  assert(Mecanum_IsBusy() && Mecanum_VelocityHandoffReady());
  assert(Mecanum_Velocity_Request(20,0,0,20));
  assert(!Mecanum_VelocityHandoffReady()); /* Latest queued motion wins. */
  advance(80);assert(!Mecanum_VelocityHandoffReady());
  assert(Mecanum_Velocity_Request(0,0,0,20));advance(80);
  assert(Mecanum_VelocityHandoffReady());
  assert(Mecanum_RequestPlannedStop());assert(!Mecanum_VelocityHandoffReady());
  advance(80);assert(!Mecanum_IsBusy());
  assert(Mecanum_Move_Control(10,0,0));
  assert(!Mecanum_VelocityHandoffReady());
  advance(80);assert(!Mecanum_VelocityHandoffReady());
  assert(Mecanum_Test_Stop());advance(60);
  assert(Mecanum_Velocity_Request(20,0,0,20));advance(1);
  tx_result=HAL_ERROR;advance(80);
  assert(!Mecanum_VelocityHandoffReady());
  puts("mecanum_velocity_handoff_test: committed zero, queued motion, stop, position and fault PASS");
}
