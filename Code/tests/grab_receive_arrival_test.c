#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main
static MechanicalArm_AxisTypeDef axis_name(unsigned i)
{return i==0?MECHANICAL_ARM_AXIS_X:i==1?MECHANICAL_ARM_AXIS_Z:MECHANICAL_ARM_AXIS_BASE;}
static void feedback(unsigned i,bool state)
{
  MechanicalArm_EventTypeDef event={0};
  event.Axis=axis_name(i);event.Action=state?MECHANICAL_ARM_ACTION_STATE:MECHANICAL_ARM_ACTION_POSITION_READ;
  event.Result=MECHANICAL_ARM_RESULT_OK;
  event.StateFlags[event.Axis]=3;
  float ppm=i==2?3200.f/360.f:Config[i==0?C_X_PPM:C_Z_PPM];
  int sign=(i==2?MECHANICAL_ARM_BASE_POSITIVE_DIR:i==0?MECHANICAL_ARM_X_POSITIVE_DIR:MECHANICAL_ARM_Z_POSITIVE_DIR)?-1:1;
  event.CurrentPositionRaw=sign*(int64_t)llroundf(Axes[i].target*ppm*65536.f/3200.f);
  PendingAxis=event.Axis;PendingAction=event.Action;
  assert(GrabTask_MotorEventHandle(&event));
}
static void seed(void)
{
  reset();configure(true);clock_ms=1000;Config[C_ARRIVAL_HOLD_MS]=200;
  for(unsigned i=0;i<3;i++) {
    Axes[i]=(GrabAxis_t){.target=i==1?-40.f:0.f,.commanded=true,.command_tick=900};
  }
}
static void observe(unsigned loops)
{
  for(unsigned n=0;n<loops;n++) {
    clock_ms+=25;
    for(unsigned i=0;i<3;i++){feedback(i,false);feedback(i,true);}
  }
}
int main(void)
{
  seed();observe(12); /* Never call arrived while receiving feedback. */
  assert(Axes[0].arrival_accepted && Axes[1].arrival_accepted && Axes[2].arrival_accepted);
  assert(arrived(1,clock_ms) && arrived(0,clock_ms) && arrived(2,clock_ms));
  Status.state=GRAB_LIFT;
  assert(axis_move(0,Axes[0].target));
  assert(!pending_valid && arrived(0,clock_ms));
  clock_ms+=(uint32_t)Config[C_FEEDBACK_MS]+1;
  assert(!arrived(0,clock_ms));
  feedback(0,false);feedback(0,true);assert(!arrived(0,clock_ms));
  observe(12);assert(arrived(0,clock_ms));
  stop_begin();assert(!arrived(0,clock_ms));
  seed();observe(12);GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_X);
  assert(!Axes[0].arrival_accepted && !Axes[0].commanded);
  seed();observe(4);assert(!arrived(1,clock_ms));
  clock_ms+=300;assert(!arrived(1,clock_ms)); /* Cache cannot finish the hold. */
  observe(10);assert(arrived(1,clock_ms));
  clock_ms+=(uint32_t)Config[C_FEEDBACK_MS]+1;assert(!arrived(1,clock_ms));
  observe(4);assert(!arrived(1,clock_ms));observe(8);assert(arrived(1,clock_ms));
  /* Neither drive bit1 nor an old Both notification can bypass measured hold. */
  seed();physical[0]=destination[0]=0;arrival_reply_enabled=true;
  feedback(0,false);feedback(0,true);assert(!arrived(0,clock_ms));
  /* Continuous X can submit the next target after ACK without waiting200ms. */
  seed();Status.state=GRAB_ALIGN;
  assert(axis_move(0,10));pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  clock_ms+=5;assert(axis_move(0,11));assert(!Axes[0].arrival_accepted);
  puts("grab_receive_arrival_test: independent200ms, reuse, invalidation, continuous X PASS");
}
