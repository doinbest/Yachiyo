#define main legacy_route_test_main
#include "chassis_route_test.c"
#undef main
int main(void)
{
  setup();char *automatic[]={"chassis","route","auto"};
  assert(ChassisRoute_Command(3,automatic));finish_leg();
  unsigned before=starts;ChassisRoute_Process();
  assert(status().segment==2 && starts>before); /* no artificial dwell */
#ifdef CHASSIS_TASK_ROUTE
  setup();assert(ChassisRoute_TaskStart(NULL,200));
  for(unsigned i=0;i<3;i++) { finish_leg(); if(i<2) ChassisRoute_Process(); }
  assert(status().station==4 && status().visit==2 && status().target_yaw_deg==0);
  assert(ChassisRoute_WorkBegin());assert(ChassisRoute_StationReserved());
  motion.action_id++;motion.target_map_yaw_deg=0; /* owned local station move */
  ChassisRoute_Process();assert(!strcmp(status().state,"work"));
  loc.generation++; /* brief stationary wheel gap recovered; origin is retained */
  assert(ChassisRoute_WorkResume());assert(last_turn && motion.target_map_yaw_deg==-90);
  expect_cancel();
  setup();assert(ChassisRoute_TaskStart(NULL,200));
  for(unsigned i=0;i<3;i++) { finish_leg();if(i<2)ChassisRoute_Process(); }
  assert(ChassisRoute_WorkBegin());loc.generation++;loc.origin.x_mm++;
  assert(!ChassisRoute_WorkResume() && !strcmp(status().reason,"station_reference_changed"));
#endif
  puts("chassis_task_route_test: PASS (immediate continuation, task handoff)");
}
