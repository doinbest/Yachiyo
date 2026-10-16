/* Actual production static controller; inherited fixture supplies hardware IO. */
#define GRAB_MEASURED_MAIN measured_alignment_main
#include "grab_measured_alignment_test.c"

static void braking_tracks_current_batches_and_feedback_age(void)
{
  float dx,dy,v[3];setup_target();Config[C_ACCEL]=300;
  pixel(1010,-10,0);sample(1020,0,0,20,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  v[0]=100;v[1]=v[2]=0;static_alignment_limit(v,dx,dy,clock_ms);
  assert(fabsf(v[0]-55.9116f)<.02f); /* 20ms command opportunity, no fixed250ms. */
  align_control(clock_ms); /* Only this task's requested motion owns the batch clock. */
  bus.sent_velocity_valid=true;bus.stage=MECANUM_STAGE_SENT;
  bus.sent_ms=1020;bus.sent_sequence=1;sample(1020,0,0,20,0);
  bus.sent_ms=1040;bus.sent_sequence=2;sample(1040,0,0,20,0);
  bus.sent_ms=1120;bus.sent_sequence=3;sample(1120,0,0,20,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  v[0]=100;v[1]=v[2]=0;static_alignment_limit(v,dx,dy,clock_ms);
  assert(fabsf(v[0]-42.1178f)<.02f); /* Actual batch interval80ms. */
  assert(static_alignment_residual(1220,&dx,&dy));
  v[0]=100;v[1]=v[2]=0;static_alignment_limit(v,dx,dy,1220);
  assert(fabsf(v[0]-27.9337f)<.02f); /* Add common feedback age100ms once. */
}
static void setup_x(void)
{
  setup_target();Config[C_ACCEL]=1000;Config[C_LAMBDA]=1;
  Config[C_BODY_SPEED]=160;Config[C_X_SPEED]=60;Config[C_X_WEIGHT]=1;
  Config[C_J11]=Config[C_J12]=1;Config[C_X_PPM]=31.746031746f;
  memset(PreviousVelocity,0,sizeof PreviousVelocity);
  LastControl=1000;LastPoll=1020;
}
static void idle_time_and_real_overspeed_are_not_hidden(void)
{
  float dx,dy,v[3];setup_target();Config[C_ACCEL]=300;
  bus.sent_velocity_valid=true;bus.stage=MECANUM_STAGE_SENT;bus.sent_ms=1000;bus.sent_sequence=5;
  sample(1000,0,0,20,0);sample(2020,0,0,20,0);pixel(2020,-10,0);
  align_control(clock_ms);
  bus.sent_ms=2030;bus.sent_sequence=6;sample(2030,0,0,20,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  v[0]=100;v[1]=v[2]=0;static_alignment_limit(v,dx,dy,clock_ms);
  assert(fabsf(v[0]-55.9102f)<.02f); /* First own batch cannot include prior idle. */
  setup_target();Config[C_ACCEL]=300;
  sample(1020,10,0,20,0);pixel(1020,-10,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  PreviousVelocity[0]=20;v[0]=100;v[1]=v[2]=0;
  static_alignment_limit(v,dx,dy,clock_ms);
  assert(v[0]==0); /* Measured500mm/s vs last20 spends all available stopping room. */
}
static void x_matches_correction_speed_and_keeps_short_lead(void)
{
  setup_x();pixel(1010,0,-6);sample(1020,0,0,20,0);align_control(clock_ms);
  assert(positions==1 && x_position_rpm==2); /* Approximately3mm/s, not60. */
  assert(Axes[0].target>20.15f && Axes[0].target<20.23f); /* 60ms lookahead. */
  assert(!Status.stop_requested && !closures);
  unsigned count=positions;align_control(clock_ms);assert(positions==count);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;AlignFeedbackDue=false;
  sample(1040,0,0,20,0);pixel(1040,0,0);LastPoll=clock_ms;align_control(clock_ms);
  assert(PendingAction==MECHANICAL_ARM_ACTION_STOP && PendingAxis==MECHANICAL_ARM_AXIS_X);
  assert(!Status.stop_requested); /* Only cancel this axis's lead; body task survives. */
}
static void busy_x_keeps_only_latest_direction_and_never_replays_after_stop(void)
{
  setup_x();PendingAction=MECHANICAL_ARM_ACTION_STATE;PendingAxis=MECHANICAL_ARM_AXIS_Z;pending_valid=true;
  pixel(1010,0,-6);sample(1020,0,0,20,0);align_control(clock_ms);
  sample(1100,0,0,20,0);pixel(1100,0,6);align_control(clock_ms);
  assert(!positions && velocity_requests==2);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;AlignFeedbackDue=false;LastPoll=clock_ms;
  clock_ms++;align_control(clock_ms);
  assert(positions==1 && Axes[0].target<20 && Axes[0].target>19.75f && x_position_rpm==2);
  unsigned count=positions;stop_begin();pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  clock_ms+=20;align_control(clock_ms);assert(positions==count && Status.stop_requested);
}
static void same_pulse_speed_changes_and_feedback_loss_are_handled(void)
{
  setup_x();pixel(1010,0,-6);sample(1020,0,0,20,0);align_control(clock_ms);
  float target=Axes[0].target;assert(positions==1 && x_position_rpm==2);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;AlignFeedbackDue=false;
  sample(1040,0,0,19.83f,0);pixel(1040,0,-12);LastPoll=clock_ms;align_control(clock_ms);
  assert(positions==2 && fabsf(Axes[0].target-target)<.001f && x_position_rpm==4);
  setup_x();PendingAction=MECHANICAL_ARM_ACTION_STATE;pending_valid=true;
  pixel(1010,0,-6);sample(1020,0,0,20,0);align_control(clock_ms);
  test_static_position.speed_valid=false;clock_ms=1030;static_alignment_observe(clock_ms);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;AlignFeedbackDue=false;LastPoll=clock_ms;
  align_control(clock_ms);assert(!positions && !AlignX.pending);
}
static void real_batch_clock_wraps_without_charging_a_task_gap(void)
{
  float dx,dy,v[3];setup_target();Config[C_ACCEL]=300;static_alignment_reset();
  RequestTick=UINT32_MAX-50U;LastControl=UINT32_MAX-40U;
  sample(UINT32_MAX-40U,0,0,20,0);pixel(UINT32_MAX-20U,-10,0);align_control(clock_ms);
  bus.sent_velocity_valid=true;bus.stage=MECANUM_STAGE_SENT;
  bus.sent_sequence=1;bus.sent_ms=UINT32_MAX-9U;sample(UINT32_MAX-9U,0,0,20,0);
  bus.sent_sequence=2;bus.sent_ms=10;sample(10,0,0,20,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  v[0]=100;v[1]=v[2]=0;static_alignment_limit(v,dx,dy,clock_ms);
  assert(fabsf(v[0]-55.9102f)<.02f);
}
int main(void)
{
  braking_tracks_current_batches_and_feedback_age();
  idle_time_and_real_overspeed_are_not_hidden();
  x_matches_correction_speed_and_keeps_short_lead();
  busy_x_keeps_only_latest_direction_and_never_replays_after_stop();
  same_pulse_speed_changes_and_feedback_loss_are_handled();
  real_batch_clock_wraps_without_charging_a_task_gap();
  puts("static visual motion: measured dispatch/feedback braking, speed-matched bounded X and cancellation PASS");
  return 0;
}
