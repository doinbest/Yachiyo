#define main legacy_grab_main
#include "grab_task_test.c"
#undef main
static void setup(void)
{
  reset();configure(true);setting("x_min","0");setting("x_max","130");setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_grab","-50");setting("z_lift","0");setting("z_car_lift","0");setting("z_place","-40");
  setting("z_rough_align","-10");setting("z_proc_place","-40");
  RearBase=37;RearReferenceValid=true;physical[0]=30;physical[2]=165;
}
static void transient_fault(void)
{
  home_bus_diag=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
  fault("motor_feedback");assert(ResumePending && status().state==GRAB_STOPPING);
  home_bus_diag.locked=false;run_to(GRAB_IDLE,200,0);
  assert(GrabTask_Continue());
}
int main(void)
{
  setup();TrackHeld=false;assert(GrabTask_AlignPlaceStart("rough",1));run_to(GRAB_ALIGN,500,0);
  transient_fault();assert(status().state==GRAB_ACQUIRE && !closures && !AlignedPlace.valid);
  setup();assert(GrabTask_CarPickStart(1));run_to(GRAB_CLOSE,500,0);
  unsigned closed=closures;transient_fault();run_to(GRAB_COMPLETE,500,0);
  assert(closures==closed && !openings && TrackHeld && !strcmp(status().result,"picked"));
  setup();TrackHeld=true;assert(GrabTask_HeldPlaceStart("rough",1,1));run_to(GRAB_RELEASE,500,0);
  unsigned opened=openings;transient_fault();run_to(GRAB_COMPLETE,500,0);
  assert(openings==opened && !TrackHeld && !strcmp(status().result,"released"));
  setup();assert(GrabTask_CarPickStart(1));run_to(GRAB_PLACE,500,0);
  home_bus_diag=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
  fault("axis_disabled_or_stall");assert(!ResumePending);
  puts("grab transient faults: fresh realignment, held material, no repeated grasp/release, driver fault excluded PASS");
}
