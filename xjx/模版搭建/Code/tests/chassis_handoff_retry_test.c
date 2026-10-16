/* Pending route admission retries the resource without losing the target. */
#define main legacy_route_retry_main
#include "chassis_route_test.c"
#undef main
int main(void)
{
 const char *reasons[]={"busy","grab_task_busy","chassis_handoff_wait"};
 for(unsigned i=0;i<3;i++){
  setup();accept=false;reject_reason=reasons[i];
  assert(ChassisRoute_TaskStart(NULL,300));
  for(unsigned retry=0;retry<6;retry++){
   advance_clock(10);ChassisRoute_Process();
   assert(!strcmp(status().state,"running") && !strcmp(status().reason,reasons[i]) && !starts);
  }
  accept=true;ChassisRoute_Process();
  assert(starts==1 && status().action_id==motion.action_id);
  assert(motion.target_x_mm==2100 && motion.target_y_mm==300);
  assert(motion.linear_accel_mm_s2==500 && motion.linear_decel_mm_s2==300);
 }
 setup();accept=false;reject_reason="invalid_target";
 assert(!ChassisRoute_TaskStart(NULL,300));
 assert(!strcmp(status().state,"error") && !strcmp(status().reason,"invalid_target"));
 puts("route handoff retry: finite resource waits preserve target; real invalid request rejects PASS");
 return 0;
}
