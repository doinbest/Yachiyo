/* P014/P008 contracts. Source prepared only; no C compilation/execution in this round. */
#define main legacy_contest_main
#include "contest_mission_test.c"
#undef main
#include "../template/App/contest_mission.c"
static void fixture(MissionState phase)
{
  ContestMission_Init();State=phase;HasRun=true;PausedState=M_IDLE;
  ChildPaused=RoutePaused=ResumeAfterStop=false;Single=RingOnly=HeadingTest=false;
  memset(&motion,0,sizeof motion);motion.state=CHASSIS_MOTION_DONE;
  motion.map_anchor_valid=true;motion.reason="feedback_arrived";motion.action_id=7;
  route.state="work";route.reason="station_work";route.phase="work_dock";
  route.action_id=7;route.stop_confirmed=true;route_resume_delayed=false;route_resume_reason=NULL;
  grab.state=GRAB_COMPLETE;grab.reason="idle";grab.result="work_ready";grab.stop_confirmed=true;
  memset(&recovery_diagnostic,0,sizeof recovery_diagnostic);
  memset(&mission_mecanum,0,sizeof mission_mecanum);
  memset(&cancel_stop,0,sizeof cancel_stop);cancel_stop.id=1;
  cancel_stop.tx_complete=cancel_stop.wheels_stopped=true;
  memset(&localization,0,sizeof localization);localization.origin_valid=true;
  localization.feedback_valid=localization.speed_valid=true;
  start_fixture_custom=true;start_imu_missing=start_feedback_missing=false;start_wheel_flags=3;start_wheel_rpm=0;
  arm_busy=turntable_busy=false;
  feedback_pause_allowed=true;feedback_pause_calls=0;grab_resume_pending=false;
  grab_resume_calls=0;cancel_defer=false;Reason="fixture";
  Request.profile=selected_profile=CHASSIS_PROFILE_LINEAR;
  Stored=4;Placed=3;Batch=1;Item=1;strcpy(Code,"156+123+516+231");
}
static void retained_child_requires_explicit_continue(void)
{
  fixture(M_TRANSFER);grab.state=GRAB_PREPARE;grab.operation="store";
  start_imu_missing=true;ContestMission_Process();
  assert(State==M_IMU_WAIT && PausedState==M_TRANSFER && ChildPaused && RoutePaused);
  assert(feedback_pause_calls==1 && grab_resume_pending && !BusAutoResume);
  assert(Stored==4 && Placed==3 && Item==1 && !strcmp(Code,"156+123+516+231"));
  assert(!ContestMission_Continue());
  route.stop_confirmed=false;grab.stop_confirmed=false;
  start_imu_missing=false;ContestMission_Process();
  assert(State==M_IMU_WAIT && !grab_resume_calls); /* Recovered angle is not parking proof. */
  route.stop_confirmed=grab.stop_confirmed=true;start_imu_missing=true;
  ContestMission_Process();assert(State==M_IMU_WAIT && !grab_resume_calls);
  start_imu_missing=false;route_resume_delayed=true;
  ContestMission_Process();assert(State==M_IMU_WAIT && !grab_resume_calls);
  route_resume_delayed=false;route_resume_reason="feedback_moving";ContestMission_Process();
  assert(State==M_IMU_WAIT && !grab_resume_calls);
  route_resume_reason="resume_feedback_wait";ContestMission_Process();
  assert(State==M_IMU_WAIT && !grab_resume_calls);
  route_resume_reason=NULL;ContestMission_Process();
  assert(State==M_PAUSED && !strcmp(Reason,"imu_recovered") && !grab_resume_calls);
  ContestMission_Process();assert(State==M_PAUSED && !grab_resume_calls);
  assert(ContestMission_Continue());ContestMission_Process();
  assert(State==M_TRANSFER && grab_resume_calls==1 && !RoutePaused && !ChildPaused);
  assert(Stored==4 && Placed==3 && Item==1);
}
static void interrupted_route_and_manual_controls(void)
{
  fixture(M_DRIVING);route.state="error";route.reason="feedback_invalid";
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="feedback_invalid";
  start_imu_missing=true;ContestMission_Process();assert(State==M_IMU_WAIT);
  start_imu_missing=false;ContestMission_Process();assert(State==M_PAUSED);
  assert(ContestMission_Continue());ContestMission_Process();assert(State==M_DRIVING);
  ContestMission_Process();assert(State==M_DRIVING && !strcmp(route.state,"running"));
  fixture(M_RING_MOVE);start_imu_missing=true;ContestMission_Process();
  ContestMission_Stop();assert(State==M_PAUSING && PausedState==M_RING_MOVE);
  ContestMission_Process();assert(State==M_PAUSED);
  start_imu_missing=false;assert(ContestMission_Continue());ContestMission_Process();
  assert(State==M_RING_CLEAR);
  fixture(M_CENTER);start_imu_missing=true;ContestMission_Process();
  ContestMission_Cancel();assert(State==M_CANCELLING && !ChildPaused && !RoutePaused);
  fixture(M_DRIVING);start_imu_missing=true;ContestMission_Process();
  assert(ContestMission_RequestBusRecovery(true));
  assert(State==M_BUS_DRAIN && !BusAutoResume && PausedState==M_DRIVING);
}
static void unsafe_or_unowned_errors_are_not_adopted(void)
{
  fixture(M_DRIVING);start_imu_missing=true;motion.map_anchor_valid=false;
  route.state="error";route.reason="feedback_invalid";
  ContestMission_Process();assert(State==M_ERROR && !feedback_pause_calls);
  fixture(M_DRIVING);start_imu_missing=true;motion.state=CHASSIS_MOTION_ERROR;
  motion.reason="stop_unconfirmed";route.state="error";route.reason="stop_unconfirmed";
  ContestMission_Process();assert(State==M_ERROR && !feedback_pause_calls);
  fixture(M_DRIVING);start_imu_missing=true;ContestMission_Process();
  start_imu_missing=false;ContestMission_Process();assert(State==M_PAUSED);
  route_resume_reason="resume_wheel_fault";assert(ContestMission_Continue());ContestMission_Process();
  assert(State==M_RESUMING && RoutePaused && !grab_resume_calls && !strcmp(Reason,"resume_wheel_fault"));
  route_resume_reason=NULL;ContestMission_Process();assert(State==M_DRIVING);
  fixture(M_DRIVING);start_imu_missing=true;feedback_pause_allowed=false;
  route.state="error";route.reason="station_reference_changed";
  ContestMission_Process();assert(State==M_ERROR);
  fixture(M_PREPARE);start_imu_missing=true;ContestMission_Process();
  assert(State==M_PREPARE && !feedback_pause_calls);
  fixture(M_DRIVING);start_imu_missing=true;ContestMission_Process();
  start_imu_missing=false;route_resume_reason="resume_wheel_fault";ContestMission_Process();
  assert(State==M_STOP_WAIT && !strcmp(Reason,"resume_wheel_fault"));
  fixture(M_DRIVING);start_imu_missing=true;ContestMission_Process();
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="stop_unconfirmed";
  ContestMission_Process();assert(State==M_STOP_WAIT && PausedState==M_DRIVING);
}
static void feedback_preparation_needs_live_stop_proof(void)
{
  fixture(M_PAUSED);assert(ContestMission_FeedbackPreparationAllowed());
  cancel_stop.wheels_stopped=false;assert(!ContestMission_FeedbackPreparationAllowed());
  cancel_stop.wheels_stopped=true;arm_busy=true;assert(!ContestMission_FeedbackPreparationAllowed());arm_busy=false;
  turntable_busy=true;assert(!ContestMission_FeedbackPreparationAllowed());turntable_busy=false;
  mission_mecanum.stop_pending=true;assert(!ContestMission_FeedbackPreparationAllowed());mission_mecanum.stop_pending=false;
  mission_mecanum.locked=true;assert(!ContestMission_FeedbackPreparationAllowed());mission_mecanum.locked=false;
  recovery_diagnostic.locked=true;assert(!ContestMission_FeedbackPreparationAllowed());recovery_diagnostic.locked=false;
  motion.state=CHASSIS_MOTION_RUNNING;assert(!ContestMission_FeedbackPreparationAllowed());motion.state=CHASSIS_MOTION_DONE;
  for(MissionState s=M_IDLE;s<=M_IMU_WAIT;s++)if(s!=M_PAUSED){State=s;assert(!ContestMission_FeedbackPreparationAllowed());}
}
int main(void)
{
  retained_child_requires_explicit_continue();interrupted_route_and_manual_controls();
  unsafe_or_unowned_errors_are_not_adopted();feedback_preparation_needs_live_stop_proof();
  puts("contest_imu_pause_test: PASS");return 0;
}
