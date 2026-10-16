#define main legacy_route_test_main
#include "chassis_route_test.c"
#undef main
static void qr_overlap(bool pause, bool arm_first)
{
  setup();selected_profile=CHASSIS_PROFILE_COSINE;qr_pose_delayed=true;
  ChassisRoute_TaskOptions_t options={true,1200,50,RADAR_START_LOWER};
  assert(ChassisRoute_TaskStartEx(NULL,200,&options));
  unsigned calls=qr_pose_calls;
  curve_progress=curve_end+5;
  loc.observer.feedback.x_mm=2050;loc.observer.feedback.y_mm=500;
  loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;
  unsigned before=starts;ChassisRoute_Process();
  assert(qr_pose_calls==calls+1 && grab_busy && starts==before);
  clearance=true;ChassisRoute_Process();assert(starts==before && !last_turn);
  if(pause){
    assert(ChassisRoute_TaskPause());grab_busy=false;
    motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
    ChassisRoute_Process();ChassisRoute_Process();grab_busy=true;clearance=false;
    before=starts;assert(ChassisRoute_TaskContinue());assert(starts==before);
    ChassisRoute_Process();assert(starts==before);
    clearance=true;ChassisRoute_Process();assert(starts==before+1);
    assert(qr_pose_calls==calls+1); /* No repeated QR pose on continuation. */
  }
  if(arm_first)grab_busy=false;
  arrive();
  if(!arm_first){assert(strcmp(status().state,"station"));grab_busy=false;ChassisRoute_Process();}
  assert(!strcmp(status().state,"station") && status().station==5);
  assert(ChassisRoute_WorkBegin());grab_busy=true;clearance=false;
  assert(!ChassisRoute_WorkResume());clearance=true;
  assert(ChassisRoute_WorkResume());
  assert(!last_turn && motion.target_y_mm==2100 && grab_busy);
  before=starts;arrive();ChassisRoute_Process();
  assert(starts==before+1 && !last_turn && !strcmp(status().phase,"work_approach"));
  assert(motion.target_x_mm==1200 && motion.target_map_yaw_deg==90);
  /* Translation may overlap a clear arm pose, but the turn at the station waits. */
  before=starts;clearance=false;arrive();assert(starts==before);
  grab_busy=false;ChassisRoute_Process();
  assert(starts==before+1 && last_turn && !strcmp(status().phase,"work_turn"));
  assert(motion.target_map_yaw_deg==0);
  grab_busy=true;clearance=false;before=starts;arrive();assert(starts==before);
  grab_busy=false;ChassisRoute_Process();
  assert(starts==before && !strcmp(status().state,"station"));
  expect_cancel();grab_busy=false;
}
static void upper_qr_and_turn_barrier(void)
{
  setup();selected_profile=CHASSIS_PROFILE_COSINE;loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;
  ChassisRoute_TaskOptions_t options={true,1200,50,RADAR_START_UPPER};
  grab_busy=true;clearance=false;
  assert(ChassisRoute_TaskStartEx(NULL,200,&options));assert(!starts);
  clearance=true;ChassisRoute_Process();assert(starts==1 && !last_turn);
  arrive();assert(starts==2 && !last_turn && motion.target_y_mm==1200);
  arrive();assert(strcmp(status().state,"station"));grab_busy=false;
  ChassisRoute_Process();assert(!strcmp(status().state,"station"));expect_cancel();
  /* A real chassis turn cannot start just because X/Z are clear. */
  setup();selected_profile=CHASSIS_PROFILE_COSINE;assert(ChassisRoute_TaskStart(NULL,200));grab_busy=true;clearance=true;
  unsigned before=starts;arrive();assert(starts==before);
  grab_busy=false;ChassisRoute_Process();assert(starts==before && !strcmp(status().state,"waiting"));
  ChassisRoute_Process();assert(starts==before+1 && !last_turn);
  expect_cancel();
}
static void pause_after_turn_before_translation(bool qr)
{
  setup();selected_profile=CHASSIS_PROFILE_COSINE;
  if(qr){
    qr_pose_delayed=true;
    ChassisRoute_TaskOptions_t options={true,1200,50,RADAR_START_LOWER};
    assert(ChassisRoute_TaskStartEx(NULL,200,&options));arrive();arrive();
    assert(grab_busy && !last_turn);
  }else{
    assert(ChassisRoute_TaskStart(NULL,200));finish_leg();ChassisRoute_Process();
    finish_leg();ChassisRoute_Process();finish_leg();
    assert(ChassisRoute_WorkBegin());assert(ChassisRoute_WorkResume());
    arrive();arrive();assert(last_turn); /* Turn at380 precedes the350 observation approach. */
    grab_busy=true;clearance=false;arrive();
  }
  unsigned before=starts,calls=qr_pose_calls;
  assert(ChassisRoute_TaskPause());grab_busy=false;
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  ChassisRoute_Process();ChassisRoute_Process();
  grab_busy=true;clearance=false;assert(ChassisRoute_TaskContinue());
  ChassisRoute_Process();assert(starts==before);
  clearance=true;ChassisRoute_Process();
  if(qr)assert(starts==before+1 && !last_turn && grab_busy && qr_pose_calls==calls);
  else {
    assert(starts==before+1 && grab_busy && !last_turn && motion.target_y_mm==350);
    arrive();assert(strcmp(status().state,"station")); /* Base must also finish. */
    grab_busy=false;ChassisRoute_Process();
    assert(!strcmp(status().state,"station") && status().station==2 && status().target_y_mm==350);
  }
  expect_cancel();grab_busy=false;
}
int main(void)
{
  pause_after_turn_before_translation(false);
  qr_overlap(false,false);qr_overlap(true,false);qr_overlap(false,true);
  upper_qr_and_turn_barrier();
  setup();selected_profile=CHASSIS_PROFILE_COSINE;accept=false;
  assert(ChassisRoute_TaskStart(NULL,200));
  assert(starts==0 && !strcmp(status().reason,"busy"));
  accept=true;ChassisRoute_Process();
  assert(starts==1 && motion.target_x_mm==2100 && motion.target_y_mm==1200);
  arrive();assert(!strcmp(status().state,"waiting"));
  expect_cancel();
  setup();selected_profile=CHASSIS_PROFILE_COSINE;RadarPlan_t p={0};p.valid=true;p.count=4;
  p.points[0]=(RadarPlan_Point_t){2250,150,1,0};
  p.points[1]=(RadarPlan_Point_t){2100,300,0,0};
  p.points[2]=(RadarPlan_Point_t){2100,600,0,0};
  p.points[3]=(RadarPlan_Point_t){2100,900,0,0};
  assert(ChassisRoute_TaskStart(&p,200));arrive();arrive();ChassisRoute_Process();
  assert(motion.target_y_mm==900 && status().segment==3);
  assert(ChassisRoute_TaskPause());motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  ChassisRoute_Process();ChassisRoute_Process();assert(ChassisRoute_TaskContinue());
  assert(motion.target_y_mm==900);expect_cancel();
  /* A lateral corner is never replaced by a diagonal shortcut. */
  setup();selected_profile=CHASSIS_PROFILE_COSINE;p.points[3]=(RadarPlan_Point_t){1800,600,0,0};
  assert(ChassisRoute_TaskStart(&p,200));arrive();arrive();ChassisRoute_Process();
  assert(motion.target_x_mm==2100 && motion.target_y_mm==600);expect_cancel();
  setup();selected_profile=CHASSIS_PROFILE_COSINE;p.points[3]=(RadarPlan_Point_t){2100,900,5,1};
  assert(ChassisRoute_TaskStart(&p,200));arrive();arrive();ChassisRoute_Process();
  assert(motion.target_y_mm==600);expect_cancel();
  /* Busy correction retains its absolute target instead of advancing. */
  setup();selected_profile=CHASSIS_PROFILE_COSINE;assert(ChassisRoute_TaskStart(NULL,200));finish_leg();ChassisRoute_Process();
  accept=false;feedback_arrival(30,0,0);assert(!strcmp(status().reason,"busy"));
  accept=true;ChassisRoute_Process();assert(motion.target_x_mm==2100 && motion.target_y_mm==2100);
  expect_cancel();
  /* Post-turn processing approach can overlap clear Base preparation, but
   * station/vision handoff still waits for the full pose and350 arrival. */
  setup();selected_profile=CHASSIS_PROFILE_COSINE;assert(ChassisRoute_TaskStart(NULL,200));finish_leg();ChassisRoute_Process();
  finish_leg();ChassisRoute_Process();finish_leg();
  assert(ChassisRoute_WorkBegin());assert(ChassisRoute_WorkResume());
  arrive();arrive();assert(last_turn);grab_busy=true;clearance=false;
  unsigned before=starts;arrive();assert(starts==before);
  clearance=true;ChassisRoute_Process();
  assert(starts==before+1 && !last_turn && motion.target_y_mm==350 && strcmp(status().state,"station"));
  arrive();assert(strcmp(status().state,"station"));
  grab_busy=false;ChassisRoute_Process();
  assert(!strcmp(status().state,"station") && status().target_y_mm==350 && starts==before+1);
  expect_cancel();
  puts("chassis_route_continuity_test: PASS");
}
