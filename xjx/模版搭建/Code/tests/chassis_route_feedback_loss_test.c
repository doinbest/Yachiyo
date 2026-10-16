/* Feedback-only interruption can preserve a route after motion has already
 * stopped with feedback_invalid; other actions, references and faults cannot. */
#define main legacy_route_main
#include "chassis_route_test.c"
#undef main

static void confirm_feedback_pause(void)
{
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="cancelled";
  ChassisRoute_Process();ChassisRoute_Process();
  assert(!strcmp(status().state,"paused") && status().stop_confirmed);
}
static void restored_feedback(void)
{
  loc.feedback_valid=loc.speed_valid=true;loc.feedback_tick=tick;loc.generation++;
}
static void qr_station(void)
{
  ChassisRoute_TaskOptions_t options={true,1200,50,RADAR_START_LOWER};
  assert(ChassisRoute_TaskStartEx(NULL,1000,&options));
  for(unsigned i=0;i<12 && strcmp(status().state,"station");i++){arrive();ChassisRoute_Process();}
  assert(status().station==5 && !strcmp(status().state,"station"));
}
int main(void)
{
  /* P014: qualification recovered and the route is already paused, then a
   * new real drive fault arrives before explicit Continue. Preserve the
   * target and reject movement until current healthy wheel evidence returns. */
  for(unsigned fault=0;fault<2;fault++){
    setup();assert(ChassisRoute_TaskStart(NULL,1000));loc.feedback_valid=false;
    assert(ChassisRoute_TaskPauseFeedbackLoss());confirm_feedback_pause();restored_feedback();
    assert(ChassisRoute_TaskResumeReady());
    unsigned before=starts,id=status().action_id;
    float x=status().phase_target_x_mm,y=status().phase_target_y_mm;
    for(unsigned i=0;i<4;i++)assert(Mecanum_FeedbackGet(i,&resume_feedback[i]));
    resume_feedback_override=true;resume_feedback[2].state_flags=fault?0:0x09U;
    assert(!ChassisRoute_TaskContinue());
    assert(!strcmp(status().reason,fault?"resume_wheels_disabled":"resume_wheel_fault"));
    assert(starts==before && !strcmp(status().state,"paused") && status().action_id==id);
    assert(status().phase_target_x_mm==x && status().phase_target_y_mm==y);
    resume_feedback_override=false;
    assert(ChassisRoute_TaskContinue() && starts==before+1);
    assert(motion.target_x_mm==x && motion.target_y_mm==y);
  }
  /* Caller-qualified IMU loss can precede localization invalidation in the
   * same control loop; the cached wheel pose must not block the pause. */
  setup();assert(ChassisRoute_TaskStart(NULL,300));
  unsigned same_loop_starts=starts;
  assert(loc.feedback_valid && ChassisRoute_TaskPauseFeedbackLoss() && starts==same_loop_starts);

  /* The motion layer may be running, stopping, or already in ERROR when the
   * mission sees the same temporary loss. Every case retains its endpoint. */
  for(unsigned stage=0;stage<3;stage++){
    setup();assert(ChassisRoute_TaskStart(NULL,1000));
    unsigned id=status().action_id,before=starts;
    loc.feedback_valid=false;
    if(stage){
      motion.state=stage==1?CHASSIS_MOTION_STOPPING:CHASSIS_MOTION_ERROR;
      motion.reason="feedback_invalid";
      if(stage==2){ChassisRoute_Process();motion.state=CHASSIS_MOTION_ERROR;ChassisRoute_Process();}
    }
    assert(ChassisRoute_TaskPauseFeedbackLoss());
    assert(starts==before && status().action_id==id && status().phase_target_x_mm==2100 && status().phase_target_y_mm==300);
    confirm_feedback_pause();assert(!ChassisRoute_TaskContinue());
    assert(!strcmp(status().reason,"resume_feedback_wait") && starts==before);
    restored_feedback();assert(ChassisRoute_TaskContinue());
    assert(starts==before+1 && motion.target_x_mm==2100 && motion.target_y_mm==300);
    assert(motion.linear_accel_mm_s2==500 && motion.linear_decel_mm_s2==300);
  }

  setup();assert(ChassisRoute_TaskStart(NULL,300));
  loc.feedback_valid=false;motion.action_id++;
  unsigned before=starts,before_stops=stops;
  assert(!ChassisRoute_TaskPauseFeedbackLoss());
  assert(starts==before && stops==before_stops && !strcmp(status().reason,"action_mismatch"));
  setup();assert(ChassisRoute_TaskStart(NULL,300));
  loc.feedback_valid=false;loc.origin.x_mm++;
  before_stops=stops;
  assert(!ChassisRoute_TaskPauseFeedbackLoss() && stops==before_stops);
  assert(!strcmp(status().reason,"station_reference_changed") && status().phase_target_x_mm==2100);
  setup();assert(ChassisRoute_TaskStart(NULL,300));
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="stop_unconfirmed";ChassisRoute_Process();
  before_stops=stops;
  assert(!ChassisRoute_TaskPauseFeedbackLoss() && stops==before_stops && !strcmp(status().reason,"stop_unconfirmed"));
  setup();assert(ChassisRoute_TaskStart(NULL,300));
  loc.feedback_valid=false;motion.state=CHASSIS_MOTION_STOPPING;motion.reason="bus_fault";
  before_stops=stops;
  assert(!ChassisRoute_TaskPauseFeedbackLoss() && stops==before_stops && !strcmp(status().reason,"bus_fault"));

  /* A stopped continuous path retains measured progress for its next start. */
  setup();selected_profile=CHASSIS_PROFILE_COSINE;assert(ChassisRoute_TaskStart(NULL,300));
  curve_progress=123;loc.feedback_valid=false;
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="feedback_invalid";
  ChassisRoute_Process();motion.state=CHASSIS_MOTION_ERROR;ChassisRoute_Process();
  assert(ChassisRoute_TaskPauseFeedbackLoss());confirm_feedback_pause();restored_feedback();
  assert(ChassisRoute_TaskContinue() && curve_progress==123 && motion.target_y_mm==1200);

  /* Route work belongs to the mission, whose local motion has another ID. */
  setup();qr_station();assert(ChassisRoute_WorkBegin());
  motion.action_id++;motion.state=CHASSIS_MOTION_ERROR;motion.reason="feedback_invalid";
  unsigned local_action=motion.action_id;
  loc.feedback_valid=false;before=starts;
  assert(ChassisRoute_TaskPauseFeedbackLoss() && status().action_id==local_action);
  confirm_feedback_pause();restored_feedback();
  assert(ChassisRoute_TaskContinue() && !strcmp(status().state,"work") && starts==before);

  /* If motion passed the corner before the route saw the loss, adopt the
   * lateral endpoint in RAM before pausing; never restart the northbound leg. */
  setup();qr_station();assert(ChassisRoute_WorkBegin());assert(ChassisRoute_WorkResume());
  unsigned segment=status().segment;before=starts;
  assert(corner_prepared);corner_passed=true;
  motion.target_x_mm=1200;motion.target_y_mm=2100;
  motion.state=CHASSIS_MOTION_STOPPING;motion.reason="feedback_invalid";loc.feedback_valid=false;
  ChassisRoute_Process();
  assert(ChassisRoute_TaskPauseFeedbackLoss());
  assert(starts==before && status().segment==segment+1 && status().phase_target_x_mm==1200);
  confirm_feedback_pause();restored_feedback();assert(ChassisRoute_TaskContinue());
  assert(!last_turn && motion.target_x_mm==1200 && motion.target_y_mm==2100);
  puts("feedback route pause: running/stopping/error, reference/action/fault gates, path/work/corner preservation PASS");
  return 0;
}
