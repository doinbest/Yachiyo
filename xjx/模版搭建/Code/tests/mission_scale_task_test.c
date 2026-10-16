#define main legacy_mission_main
#include "contest_mission_test.c"
#undef main
#include "../template/App/contest_mission.c"

extern float test_local_scaled_speed,test_local_scaled_accel,test_local_scaled_decel,test_route_scale;
extern bool test_route_start_smooth;
static void setup_scale(void)
{
  route.state="idle";route.phase="idle";route.reason="idle";
  grab.state=GRAB_IDLE;grab.reason="idle";grab.result="none";
  motion.state=CHASSIS_MOTION_IDLE;motion.reason="idle";
  start_fixture_custom=start_imu_missing=start_feedback_missing=false;
  grab_resume_pending=arm_busy=turntable_busy=false;reject_move=NULL;
  for(unsigned i=0;i<3;i++)inventory[i].state=TURNTABLE_EMPTY;
  QR_Init();ContestMission_Init();
}
int main(void)
{
  setup_scale();request.profile=CHASSIS_PROFILE_LINEAR;
  assert(ContestMission_StartConfigured(NULL,&request,1.5f,true));
  ContestMission_Report();assert(strstr(report,"start_smooth=1"));
  for(unsigned i=0;i<30 && !starts;i++)step();
  assert(starts && test_route_start_smooth && test_route_scale==1.5f);
  ContestMission_Stop();step();assert(StartSmooth && MotionScale==1.5f);
  assert(!ContestMission_StartConfigured(NULL,&request,1,false) && StartSmooth);
  assert(ContestMission_Continue());step();assert(StartSmooth && MotionScale==1.5f);
  assert(ContestMission_RequestBusRecovery(false));
  for(unsigned i=0;i<5 && !mission_recovery.active;i++)step();
  mission_recovery.active=false;mission_recovery.success=true;mission_recovery.checked_mask=0xff;mission_recovery.reason="complete";
  for(unsigned i=0;i<10;i++)step();
  assert(StartSmooth && MotionScale==1.5f);
  ContestMission_Report();assert(strstr(report,"start_smooth=1") && strstr(report,"state=paused"));
  setup_scale();request.start_id=RADAR_START_UPPER;
  assert(!ContestMission_StartConfigured(NULL,&request,1,true) && !StartSmooth);
  assert(!strcmp(Reason,"start_smooth_lower_linear_only"));
  request.start_id=RADAR_START_LOWER;request.profile=CHASSIS_PROFILE_COSINE;
  assert(!ContestMission_StartConfigured(NULL,&request,1,true) && !StartSmooth);
  request.profile=CHASSIS_PROFILE_LINEAR;starts=0;
  setup_scale();request.profile=CHASSIS_PROFILE_LINEAR;
  assert(ContestMission_StartScaled(NULL,&request,1.5f));
  assert(!StartSmooth && MotionScale==1.5f && Request.route_speed_mm_s==200 && RouteOptions.turn_speed_deg_s==50);
  ContestMission_Report();assert(strstr(report,"route_speed=300.0") && strstr(report,"route_base_speed=200.0") && strstr(report,"route_scale=1.5"));
  assert(!ContestMission_StartScaled(NULL,&request,2) && MotionScale==1.5f);
  for(unsigned i=0;i<30 && !starts;i++)step();
  assert(starts && test_route_scale==1.5f);
  /* Shared local move covers ring changes, docking and processing departure. */
  State=M_RING_MOVE;allowed_local_speed=75;motion.state=CHASSIS_MOTION_DONE;
  assert(local_move(1050,350,"ring_position"));
  assert(test_local_scaled_speed==75 && test_local_scaled_accel==450 && test_local_scaled_decel==450);
  ContestMission_Stop(); /* Pause cannot replace the accepted scale. */
  assert(MotionScale==1.5f);
  setup_scale();assert(!ContestMission_StartScaled(NULL,&request,NAN));
  assert(!strcmp(Reason,"route_scale_range") && MotionScale==1);
  assert(!ContestMission_StartScaled(NULL,&request,0));
  request.route_speed_mm_s=5000;assert(!ContestMission_StartScaled(NULL,&request,1.2f));
  assert(!strcmp(Reason,"route_scaled_speed_range"));
  request.route_speed_mm_s=200;request.profile=CHASSIS_PROFILE_COSINE;
  assert(!ContestMission_StartScaled(NULL,&request,1.2f) && !strcmp(Reason,"route_scale_linear_only"));
  allowed_local_speed=50;
  puts("mission scale task: locked base/effective, local motion and rejection PASS");return 0;
}
