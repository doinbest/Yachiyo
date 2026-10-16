/* Prepared without compiling: the parent must reach feedback preparation even
 * when the same interrupted action still has its old feedback_invalid error. */
#define main existing_mission_main
#define ChassisRoute_TaskResumeReady FixtureResumeReady
#define ChassisRoute_TaskContinue FixtureContinue
#include "contest_mission_test.c"
#undef ChassisRoute_TaskResumeReady
#undef ChassisRoute_TaskContinue
#undef main

static unsigned ready_calls,continue_calls;
static const char *wait_reason;
bool ChassisRoute_TaskResumeReady(void)
{
  ready_calls++;
  if(wait_reason){route.reason=wait_reason;return false;}
  return FixtureResumeReady();
}
bool ChassisRoute_TaskContinue(void){continue_calls++;return FixtureContinue();}
static void recover_interrupted_action(void)
{
  tick=1000;memset(&grab,0,sizeof grab);memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
  grab.reason="idle";grab.result="none";route.state=route.phase=route.reason="idle";
  memset(&recovery_diagnostic,0,sizeof recovery_diagnostic);memset(&mission_recovery,0,sizeof mission_recovery);
  for(unsigned i=0;i<3;i++)inventory[i].state=TURNTABLE_EMPTY;
  cancel_defer=false;start_fixture_custom=false;grab_resume_pending=false;qr_scanning=false;
  route_resume_delayed=false;wait_reason=NULL;ready_calls=continue_calls=0;
  QR_Init();ContestMission_Init();assert(ContestMission_Start(NULL,&request));step();
  motion.action_id=route.action_id=9;
  assert(ContestMission_RequestBusRecovery(true));
  tick++;ContestMission_Process();assert(mission_recovery.active);
  mission_recovery.active=false;mission_recovery.success=true;mission_recovery.checked_mask=0xff;
  tick++;ContestMission_Process();
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="feedback_invalid";
  route.state="paused";route.stop_confirmed=true;grab.stop_confirmed=true;
  tick++;ContestMission_Process();ContestMission_Report();assert(strstr(report,"state=resuming "));
}
int main(void)
{
  recover_interrupted_action();wait_reason="resume_feedback_wait";
  for(unsigned i=0;i<4;i++){tick++;ContestMission_Process();}
  ContestMission_Report();assert(strstr(report,"state=resuming ") && strstr(report,"reason=resume_feedback_wait"));
  assert(ready_calls==4 && !continue_calls);
  wait_reason=NULL;tick++;ContestMission_Process();ContestMission_Report();
  assert(continue_calls==1 && strstr(report,"reason=resumed"));

  recover_interrupted_action();wait_reason="resume_origin_invalid";
  tick++;ContestMission_Process();ContestMission_Report();
  assert(strstr(report,"reason=resume_origin_invalid") && !continue_calls);

  recover_interrupted_action();motion.action_id=10;
  tick++;ContestMission_Process();ContestMission_Report();
  assert(strstr(report,"state=stop_wait ") && !ready_calls && !continue_calls);

  recover_interrupted_action();motion.reason="stop_failed";
  tick++;ContestMission_Process();ContestMission_Report();
  assert(strstr(report,"state=stop_wait ") && !ready_calls && !continue_calls);

  recover_interrupted_action();ContestMission_Stop();
  for(unsigned i=0;i<3;i++){tick++;ContestMission_Process();}
  ContestMission_Report();assert(strstr(report,"auto_resume=0") && !continue_calls);
  puts("mission resume: same interrupted action waits for current feedback; new errors and pause preserved PASS");
  return 0;
}
