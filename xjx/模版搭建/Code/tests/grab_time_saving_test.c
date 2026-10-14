#define main legacy_main
#include "grab_task_test.c"
#undef main

static void fresh_axes(uint32_t now)
{
  clock_ms=now;
  memset(Axes,0,sizeof Axes);
  for(unsigned i=0;i<3;i++) {
    Axes[i].position_valid=Axes[i].state_valid=Axes[i].quiet=true;
    Axes[i].position_tick=Axes[i].state_tick=now;
    Axes[i].quiet_since=now-300;
    Axes[i].flags=3;
    Axes[i].mm=physical[i];
  }
}
static void fixed_target_already_held(void)
{
  reset();configure(false);physical[1]=10;fresh_axes(1000);
  Status.state=GRAB_LIFT;Operation=OP_STORE;
  assert(axis_move(1,10));assert(positions==0);
  assert(arrived(1,clock_ms));
  Axes[1].mm=9;
  assert(!arrived(1,clock_ms) && !Axes[1].commanded);
  Axes[1].mm=10;assert(axis_move(1,10));
  stop_begin();assert(!arrived(1,clock_ms));
  reset();configure(false);physical[1]=10;fresh_axes(1000);Status.state=GRAB_LIFT;
  assert(axis_move(1,10));
  clock_ms+=Config[C_FEEDBACK_MS]+1;
  assert(!arrived(1,clock_ms)); /* Cached evidence must expire. */

  const GrabTask_State_t precise[]={GRAB_ALIGN,GRAB_DESCEND,GRAB_PLACE};
  for(unsigned n=0;n<3;n++) {
    reset();configure(false);physical[1]=10;fresh_axes(1000);
    Status.state=precise[n];assert(axis_move(1,10));assert(positions==1);
  }
  reset();configure(false);physical[1]=10;fresh_axes(1000);
  Status.state=GRAB_LIFT;Axes[1].quiet=false;
  assert(axis_move(1,10));assert(positions==1);
  reset();configure(false);physical[1]=10;fresh_axes(1000);
  Status.state=GRAB_LIFT;Axes[1].position_tick=clock_ms-(uint32_t)Config[C_FEEDBACK_MS]-1;
  assert(axis_move(1,10));assert(positions==1);
  reset();configure(false);physical[0]=5;fresh_axes(1000);
  Status.state=GRAB_X_RETRACT;Axes[0].flags=1; /* Missing reached needs its existing fallback. */
  assert(axis_move(0,5));assert(positions==1);
}
static void independent_claw_waits(void)
{
  reset();configure(false);setting("close_ms","700");setting("release_ms","100");
  Operation=OP_CAR_PLACE;Status.state=GRAB_RELEASE;StateTick=clock_ms;
  transfer_process(clock_ms+99);assert(Status.state==GRAB_RELEASE);
  transfer_process(clock_ms+100);assert(Status.state==GRAB_COMPLETE);
  assert(!strcmp(Status.result,"released"));
  Status.state=GRAB_CLOSE;StateTick=clock_ms;
  transfer_process(clock_ms+699);assert(Status.state==GRAB_CLOSE);
  transfer_process(clock_ms+700);assert(Status.state==GRAB_LIFT);
}
static void completed_pose_handoff(void)
{
  for(unsigned stale=0;stale<2;stale++) {
    reset();configure(false);Config[C_Z_CAR_LIFT]=0;Config[C_X_MIN]=0;
    RearReferenceValid=OutsideReferenceValid=RearPose=true;RearBase=0;
    physical[0]=physical[1]=physical[2]=0;fresh_axes(1000);
    Status.state=GRAB_COMPLETE;Status.stop_requested=false;
    if(stale)Axes[0].position_tick=clock_ms-(uint32_t)Config[C_FEEDBACK_MS]-1;
    assert(GrabTask_PoseStart(false));pose_process(clock_ms);
    assert(stale?Status.state==GRAB_ROUTE_POSE:Status.state==GRAB_COMPLETE);
    assert(!positions && !openings);
  }
}
int main(void)
{
  fixed_target_already_held();completed_pose_handoff();independent_claw_waits();
  puts("grab time saving: fresh held target/pose reuse, expiry and precise moves PASS");
  return 0;
}
