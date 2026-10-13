#define main legacy_route_test_main
#include "chassis_route_test.c"
#undef main
static void expect_qr_heading_retained(bool planned,float scan_y)
{
  setup();
  unsigned poses_before=qr_pose_calls;
  ChassisRoute_TaskOptions_t options={true,scan_y,50,RADAR_START_LOWER};
  RadarPlan_t plan={0};
  if(planned)
  {
    plan.valid=true;plan.count=CHASSIS_ROUTE_TARGET_COUNT+1U;
    for(unsigned i=0;i<plan.count;i++)assert(ChassisRouteProfile_FixedPoint(i,&plan.points[i]));
  }
  assert(ChassisRoute_TaskStartEx(planned?&plan:NULL,200,&options));
  assert(!last_turn && motion.target_x_mm==2100 && motion.target_y_mm==scan_y && motion.target_map_yaw_deg==90);
  assert(!strcmp(status().phase,"entry_curve") && turn_limit==50);
  arrive();assert(qr_pose_calls==poses_before+1);
  ChassisRoute_Process();
  assert(!last_turn && motion.target_y_mm==scan_y && motion.target_map_yaw_deg==90 && last_speed==200);
  arrive();
  assert(starts==1 && !strcmp(status().state,"station") && status().stop_confirmed);
  assert(status().station==5 && status().target_yaw_deg==90);
  assert(ChassisRoute_WorkBegin());assert(ChassisRoute_WorkResume());
  assert(!last_turn && motion.target_x_mm==2100 && motion.target_y_mm==2100 && motion.target_map_yaw_deg==90);
  arrive();ChassisRoute_Process();
  assert(!last_turn && motion.target_x_mm==1200 && motion.target_y_mm==2100 && motion.target_map_yaw_deg==90);
  assert(!strcmp(status().direction,"lateral"));
  motion.state=CHASSIS_MOTION_STOPPING;motion.stop_confirmed=false;
  ChassisRoute_Process();assert(starts==3 && !last_turn);
  arrive();
  assert(starts==4 && last_turn && motion.target_map_yaw_deg==0);
  assert(!strcmp(status().phase,"work_turn"));
  arrive();
  assert(starts==4 && !strcmp(status().state,"station") && status().target_yaw_deg==0);
}
int main(void)
{
  setup();loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;
  ChassisRoute_TaskOptions_t upper={true,1200,30,RADAR_START_UPPER};
  assert(ChassisRoute_TaskStartEx(NULL,300,&upper));
  assert(!last_turn && motion.target_y_mm==2100 && motion.target_map_yaw_deg==-90);
  arrive();assert(!last_turn && motion.target_y_mm==1200 && last_speed==300);
  arrive();assert(!strcmp(status().state,"station") && status().target_yaw_deg==-90);
  expect_cancel();

  setup();ChassisRoute_TaskOptions_t options={true,1250,50,RADAR_START_LOWER};
  assert(ChassisRoute_TaskStartEx(NULL,200,&options));
  assert(motion.target_y_mm==1250 && motion.target_map_yaw_deg==90 && status().phase_speed_mm_s==200);
  finish_leg();assert(status().station==5 && status().target_y_mm==1250);
  assert(status().target_yaw_deg==90 && !strcmp(status().state,"station"));
  assert(qr_pose_calls==1 && turn_limit==50);
  assert(ChassisRoute_WorkBegin());
  assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=false;independent_stop=true;motion.reason="cancelled";ChassisRoute_Process();ChassisRoute_Process();
  assert(!strcmp(status().state,"paused"));
  feedback_enabled=false;tick+=10000;unsigned starts_before_resume=starts;assert(!ChassisRoute_TaskContinue());assert(feedback_enabled && starts==starts_before_resume);
  for(unsigned i=0;i<20;i++){tick+=20;assert(!ChassisRoute_TaskContinue());}
  assert(feedback_selections==1); /* Re-selecting restarts the real wheel poll at wheel 1. */
  loc.feedback_tick=tick;assert(ChassisRoute_TaskContinue());
  independent_stop=false;assert(!ChassisRoute_WorkResume()); /* stop proof must remain valid */
  independent_stop=true;motion.action_id++;assert(!ChassisRoute_WorkResume()); /* old proof cannot cover new local motion */
  motion.action_id--;loc.observer.feedback.y_mm+=100;assert(!ChassisRoute_WorkResume());
  loc.observer.feedback.y_mm-=100;assert(ChassisRoute_WorkResume());independent_stop=false;
  assert(status().segment==2 && motion.target_map_yaw_deg==90);
  assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;ChassisRoute_Process();ChassisRoute_Process();
  feedback_enabled=false;assert(ChassisRoute_TaskContinue());assert(feedback_enabled && feedback_selections==2);
  expect_cancel();
  for(unsigned planned=0;planned<2;planned++)
    for(unsigned scan_y=1100;scan_y<=1300;scan_y+=100)
      expect_qr_heading_retained(planned!=0,(float)scan_y);
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
