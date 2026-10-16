/* Prepared for later C regression; no new wait timer or origin reset. */
#define main existing_route_main
#include "chassis_route_test.c"
#undef main

static void current_wheel_qualification(void)
{
  for(unsigned wheel=0;wheel<4;wheel++)for(unsigned fault=0;fault<10;fault++)
  {
    setup();assert(ChassisRoute_TaskStart(NULL,200));assert(ChassisRoute_TaskPause());
    motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
    ChassisRoute_Process();ChassisRoute_Process();assert(status().stop_confirmed);
    for(unsigned i=0;i<4;i++)assert(Mecanum_FeedbackGet(i,&resume_feedback[i]));
    resume_feedback_override=true;
    unsigned before=starts;
    Mecanum_Feedback_t *w=&resume_feedback[wheel];
    switch(fault){
      case 0:resume_feedback_missing=true;break;
      case 1:w->speed_valid=false;break;
      case 2:w->position_valid=false;break;
      case 3:w->state_valid=false;break;
      case 4:w->speed_ms=tick-601U;break;
      case 5:w->position_ms=tick-601U;break;
      case 6:w->state_ms=tick-601U;break;
      case 7:w->state_flags=0x05U;break;
      case 8:w->state_flags=0;break;
      case 9:w->speed_rpm=wheel%2?2:-2;break;
    }
    const char *why=fault==7?"resume_wheel_fault":fault==8?"resume_wheels_disabled":
                    fault==9?"feedback_moving":"resume_feedback_wait";
    assert(!ChassisRoute_TaskResumeReady() && !strcmp(status().reason,why));
    assert(!ChassisRoute_TaskContinue() && !strcmp(status().reason,why));
    assert(starts==before && !strcmp(status().state,"paused"));
    resume_feedback_override=resume_feedback_missing=false;
    assert(ChassisRoute_TaskContinue() && starts==before+1);
  }
  /* The existing inclusive 600 ms and +/-1 RPM bounds remain admissible,
   * including unsigned millisecond wrap. No new fixed wait is introduced. */
  setup();assert(ChassisRoute_TaskStart(NULL,200));assert(ChassisRoute_TaskPause());
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  ChassisRoute_Process();ChassisRoute_Process();
  for(unsigned i=0;i<4;i++){
    assert(Mecanum_FeedbackGet(i,&resume_feedback[i]));
    resume_feedback[i].state_ms=resume_feedback[i].speed_ms=resume_feedback[i].position_ms=tick-600U;
    resume_feedback[i].speed_rpm=i%2?1:-1;
  }
  resume_feedback_override=true;assert(ChassisRoute_TaskResumeReady());
  assert(ChassisRoute_TaskContinue());
}
int main(void)
{
  current_wheel_qualification();
  setup();assert(ChassisRoute_TaskStart(NULL,200));
  assert(ChassisRoute_TaskPause());
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  ChassisRoute_Process();ChassisRoute_Process();assert(status().stop_confirmed);
  unsigned selected=feedback_selections,before=starts;
  loc.feedback_valid=false;
  assert(!ChassisRoute_TaskResumeReady());assert(!strcmp(status().reason,"resume_feedback_wait"));
  assert(feedback_selections==selected+1 && feedback_enabled && starts==before);
  assert(!ChassisRoute_TaskResumeReady());assert(feedback_selections==selected+1);

  loc.feedback_valid=true;loc.speed_valid=false;
  assert(!ChassisRoute_TaskResumeReady());assert(!strcmp(status().reason,"resume_speed_wait"));
  loc.speed_valid=true;loc.origin_valid=false;
  assert(!ChassisRoute_TaskResumeReady());assert(!strcmp(status().reason,"resume_origin_invalid"));
  loc.origin_valid=true;float origin_x=loc.origin.x_mm;loc.origin.x_mm+=1;
  assert(!ChassisRoute_TaskResumeReady());assert(!strcmp(status().reason,"station_reference_changed"));
  loc.origin.x_mm=origin_x;
  assert(ChassisRoute_TaskResumeReady());assert(!strcmp(status().reason,"resume_ready"));
  assert(starts==before && feedback_selections==selected+1);
  assert(ChassisRoute_TaskContinue());assert(starts==before+1);
  puts("route resume: fresh feedback reasons, one selection, unchanged reference and original target PASS");
  return 0;
}
