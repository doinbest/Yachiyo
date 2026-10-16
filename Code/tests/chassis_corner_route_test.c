#define main original_task_route_main
#include "chassis_route_test.c"
#undef main

int main(void)
{
  setup();ChassisRoute_TaskOptions_t options={true,1200,50,RADAR_START_LOWER};
  assert(ChassisRoute_TaskStartEx(NULL,1000,&options));
  for(unsigned i=0;i<12 && strcmp(status().state,"station");i++){arrive();ChassisRoute_Process();}
  assert(status().station==5 && !strcmp(status().state,"station"));
  assert(ChassisRoute_WorkBegin());
  loc.observer.feedback.yaw_rad=89.8f*CHASSIS_MODEL_PI/180;
  grab_busy=qr_departure=true;clearance=false;
  assert(ChassisRoute_WorkResume());
  assert(!last_turn && motion.target_y_mm==2100 && corner_prepared);
  assert(last_speed==800 && corner_speed==700 && corner_accel==800);
  unsigned before=starts,id=motion.action_id,segment=status().segment;
  loc.observer.feedback.y_mm=2095;corner_passed=true;
  motion.target_x_mm=1200;motion.target_y_mm=2100;
  ChassisRoute_Process();
  assert(starts==before && status().action_id==id && status().segment==segment+1);
  assert(status().station==4 && status().visit==2 && !status().stop_confirmed);
  assert(!strcmp(status().phase,"work_approach"));
  /* Pause after passing cannot replay the old northbound endpoint. */
  assert(ChassisRoute_TaskPause());grab_busy=false;
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  ChassisRoute_Process();ChassisRoute_Process();assert(!strcmp(status().state,"paused"));
  assert(ChassisRoute_TaskContinue());assert(motion.target_x_mm==1200 && !last_turn);
  arrive();assert(last_turn && motion.target_map_yaw_deg==0); /* Actual station turn remains. */
  puts("corner route: QR tiny-turn suppression, running adoption, pause and final turn PASS");
  return 0;
}
