#define main legacy_route_main
#include "chassis_route_test.c"
#undef main

static ChassisRoute_TaskOptions_t options={true,1200,100,RADAR_START_LOWER};
static void fixed_plan(RadarPlan_t *plan)
{
  memset(plan,0,sizeof *plan);plan->valid=true;plan->count=CHASSIS_ROUTE_TARGET_COUNT+1;
  for(unsigned i=0;i<plan->count;i++)assert(ChassisRouteProfile_FixedPoint(i,&plan->points[i]));
}
static void entrance(bool planned,float scale,float base)
{
  setup();qr_pose_calls=0;
  RadarPlan_t plan;fixed_plan(&plan);
  assert(ChassisRoute_TaskStartConfigured(planned?&plan:NULL,base,&options,scale,true));
  assert(!last_turn && !strcmp(status().phase,"entry_curve"));
  assert(starts==1 && stops==0 && last_speed==fminf(base,800)*scale && path_scale==scale);
  assert(submitted_path.count==26 && submitted_path.point[8].x==2125 && submitted_path.point[8].y==350);
  for(unsigned i=1;i<25;i++)assert(submitted_path.point[i].limit==fminf(base,200));
  assert(submitted_path.point[24].x==2100 && submitted_path.point[24].y==950 && submitted_path.point[24].yaw==90);
  assert(submitted_path.point[25].x==2100 && submitted_path.point[25].y==1200);
  curve_progress=500;ChassisRoute_Process();assert(!qr_pose_calls && starts==1 && !stops);
  assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="cancelled";
  ChassisRoute_Process();ChassisRoute_Process();assert(!strcmp(status().state,"paused"));
  assert(!ChassisRoute_TaskStartConfigured(NULL,base,&options,1,false));
  ChassisRoute_TaskRecoveryHold(true);ChassisRoute_TaskRecoveryHold(false);
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  ChassisRoute_Process();ChassisRoute_Process();
  assert(ChassisRoute_TaskContinue() && curve_progress==500 && path_scale==scale);
  assert(last_speed==fminf(base,800)*scale);
  /* The path remains one motion action. Only its end may hand off to QR. */
  unsigned before=starts,stopped=stops;
  curve_progress=curve_end;loc.observer.feedback.yaw_rad=100*CHASSIS_MODEL_PI/180;
  ChassisRoute_Process();assert(!qr_pose_calls);
  loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;
  qr_pose_delayed=true;ChassisRoute_Process();assert(qr_pose_calls==1 && starts==before && stops==stopped);
  arrive();assert(strcmp(status().state,"station")); /* Arm preparation still running. */
  grab_busy=false;ChassisRoute_Process();
  assert(!strcmp(status().state,"station") && status().station==5 && status().stop_confirmed);
  assert(starts==before && stops==stopped);
  assert(route_command("status") && strstr(reply,"start_smooth=1 start_smooth_active=1"));
}
int main(void)
{
  entrance(false,1,1000);entrance(true,1.5f,1000);entrance(false,.5f,100);
  setup();assert(ChassisRoute_TaskStartScaled(NULL,1000,&options,1));
  assert(!last_turn && !strcmp(status().phase,"move") && motion.target_y_mm==300);
  assert(route_command("status") && strstr(reply,"start_smooth=0 start_smooth_active=0"));
  setup();selected_profile=CHASSIS_PROFILE_COSINE;
  assert(!ChassisRoute_TaskStartConfigured(NULL,1000,&options,1,true));
  assert(ChassisRoute_TaskStartScaled(NULL,1000,&options,1) && !strcmp(status().phase,"entry_curve") && last_speed==1000);
  setup();options.start_id=RADAR_START_UPPER;
  assert(!ChassisRoute_TaskStartConfigured(NULL,1000,&options,1,true));
  loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;
  assert(ChassisRoute_TaskStartScaled(NULL,1000,&options,1));
  assert(strcmp(status().phase,"entry_curve") && motion.target_x_mm==2100 && motion.target_y_mm==2100);
  options.start_id=RADAR_START_LOWER;
  setup();RadarPlan_t plan;fixed_plan(&plan);
  /* A radar detour's first neutral target cannot be replaced by the fixed curve. */
  plan.count=3;plan.points[1]=(RadarPlan_Point_t){2000,500,0,0};
  plan.points[2]=(RadarPlan_Point_t){2100,1200,5,1};
  assert(ChassisRoute_TaskStartConfigured(&plan,1000,&options,1,true));
  assert(strcmp(status().phase,"entry_curve") && motion.target_x_mm==2000 && motion.target_y_mm==500);
  assert(route_command("status") && strstr(reply,"start_smooth=1 start_smooth_active=0"));
  puts("mission start smooth route: geometry, caps, pause/recovery, QR handoff PASS");return 0;
}
