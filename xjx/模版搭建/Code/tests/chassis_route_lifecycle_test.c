/* A completed lower-start match must not become the paused route of the next
 * upper-start match while that match is still preparing its own reference. */
#define main legacy_route_main
#include "chassis_route_test.c"
#undef main

static void confirm_pause(void)
{
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="cancelled";
  ChassisRoute_Process();ChassisRoute_Process();
  assert(!strcmp(status().state,"paused") && status().stop_confirmed);
}

int main(void)
{
  setup();assert(!ChassisRoute_TaskPause() && !stops);
  ChassisRoute_TaskOptions_t lower={false,1200,100,RADAR_START_LOWER};
  RadarPlan_t short_match={0};short_match.valid=true;short_match.count=2;
  short_match.points[0]=(RadarPlan_Point_t){2250,150,1,0};
  short_match.points[1]=(RadarPlan_Point_t){2100,300,0,0};
  assert(ChassisRoute_TaskStartEx(&short_match,300,&lower));
  finish_leg();ChassisRoute_Process();
  assert(!strcmp(status().state,"done") && !ChassisRoute_StationReserved());
  unsigned before_stops=stops,before_starts=starts;
  ChassisRoute_Status_t completed=status();
  assert(!ChassisRoute_TaskPause());
  assert(stops==before_stops && starts==before_starts && !strcmp(status().state,"done"));
  assert(status().target_x_mm==completed.target_x_mm && status().target_y_mm==completed.target_y_mm);

  /* No new route exists during preparation, even when feedback is rebuilding. */
  loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;
  loc.feedback_valid=false;
  assert(!ChassisRoute_TaskPause());
  assert(stops==before_stops && !ChassisRoute_StationReserved());
  loc.feedback_valid=true;
  ChassisRoute_TaskOptions_t upper={true,1200,100,RADAR_START_UPPER};
  assert(ChassisRoute_TaskStartEx(NULL,300,&upper));
  assert(motion.target_x_mm==2100 && motion.target_y_mm==2100 && motion.target_map_yaw_deg==-90);
  assert(ChassisRoute_TaskPause());confirm_pause();
  before_stops=stops;
  assert(ChassisRoute_TaskPause() && stops==before_stops); /* Same valid pause. */
  assert(ChassisRoute_TaskContinue());
  assert(motion.target_y_mm==2100 && motion.target_map_yaw_deg==-90);
  assert(loc.origin.y_mm==2250 && loc.origin.yaw_rad==-CHASSIS_MODEL_PI/2);

  assert(ChassisRoute_Cancel());motion.state=CHASSIS_MOTION_DONE;
  ChassisRoute_Process();before_stops=stops;
  assert(!strcmp(status().state,"cancelled") && !ChassisRoute_TaskPause() && stops==before_stops);
  setup();assert(ChassisRoute_TaskStart(NULL,300));
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="bus_unavailable";ChassisRoute_Process();
  before_stops=stops;
  assert(!ChassisRoute_TaskPause() && stops==before_stops && !strcmp(status().reason,"bus_unavailable"));
  puts("route ownership: completed lower match, upper preparation pause, new origin and terminal rejection PASS");
  return 0;
}
