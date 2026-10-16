#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main

static unsigned reads[6];
static uint32_t last_read[6], max_gap[6];

static void seed_feedback(void)
{
  reset();configure(true);
  Status.state=GRAB_LIFT;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  PollIndex=0;LastPoll=clock_ms;
  memset(reads,0,sizeof reads);memset(max_gap,0,sizeof max_gap);
  for(unsigned i=0;i<3;i++) {
    Axes[i]=(GrabAxis_t){.position_valid=true,.state_valid=true,.flags=1,
      .position_tick=clock_ms,.state_tick=clock_ms,.quiet=true,.quiet_since=clock_ms};
    last_read[i*2]=last_read[i*2+1]=clock_ms;
  }
}

static void finish_read(void)
{
  unsigned i=pending.Axis==MECHANICAL_ARM_AXIS_X?0:pending.Axis==MECHANICAL_ARM_AXIS_Z?1:2;
  unsigned kind=pending.Action==MECHANICAL_ARM_ACTION_STATE;
  assert(pending.Action==MECHANICAL_ARM_ACTION_POSITION_READ || kind);
  unsigned item=i*2+kind;
  uint32_t gap=clock_ms-last_read[item];
  if(gap>max_gap[item])max_gap[item]=gap;
  last_read[item]=clock_ms;reads[item]++;
  /* The device has replied; only the corresponding measured field advances. */
  if(kind)Axes[i].state_tick=clock_ms;else Axes[i].position_tick=clock_ms;
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
}

static void run_poll(unsigned duration)
{
  while(duration--) {
    clock_ms++;poll_axes(clock_ms);
    if(pending_valid)finish_read();
  }
}

int main(void)
{
  seed_feedback();Axes[1].commanded=true;Axes[1].target=10;Axes[1].command_tick=clock_ms;
  run_poll(1000);
  /* The only moving axis must update well before the old 150ms round. */
  assert(reads[2]>=20 && reads[3]>=20);
  assert(max_gap[2]<=60 && max_gap[3]<=60);
  for(unsigned n=0;n<6;n++)assert(reads[n] && max_gap[n]<=220);
  assert(reads[2]>reads[0]*2 && reads[3]>reads[1]*2);
  printf("single active Z: position/state max gap=%lu/%lums, reads=%u/%u; held X=%u/%u\n",
    (unsigned long)max_gap[2],(unsigned long)max_gap[3],reads[2],reads[3],reads[0],reads[1]);

  seed_feedback();
  for(unsigned i=0;i<3;i++){Axes[i].commanded=true;Axes[i].target=10;Axes[i].command_tick=clock_ms;}
  run_poll(1000);
  for(unsigned n=0;n<6;n++)assert(reads[n]>=20 && max_gap[n]<=60);
  printf("parallel X/Z/Base: max field gaps=%lu/%lu/%lu/%lu/%lu/%lums\n",
    (unsigned long)max_gap[0],(unsigned long)max_gap[1],(unsigned long)max_gap[2],
    (unsigned long)max_gap[3],(unsigned long)max_gap[4],(unsigned long)max_gap[5]);

  seed_feedback();clock_ms+=151;
  poll_axes(clock_ms);assert(pending_valid);
  MechanicalArm_AxisTypeDef selected=pending.Axis;
  MechanicalArm_ActionTypeDef action=pending.Action;
  unsigned pointer=PollIndex;uint32_t accepted=LastPoll;
  for(unsigned n=0;n<100;n++){clock_ms++;poll_axes(clock_ms);}
  assert(pending.Axis==selected && pending.Action==action && PollIndex==pointer && LastPoll==accepted);
  finish_read();
  clock_ms++;poll_axes(clock_ms);assert(pending_valid); /* No extra 25ms wait. */

  seed_feedback();Status.state=GRAB_STOPPING;StopEvidenceStarted=false;
  run_poll(100);for(unsigned n=0;n<6;n++)assert(!reads[n]);
  StopEvidenceStarted=true;StopTick=clock_ms;
  run_poll(80);for(unsigned n=0;n<6;n++)assert(reads[n]);

  seed_feedback();AlignFeedbackDue=true;clock_ms+=6;
  poll_axes(clock_ms);assert(!pending_valid && !AlignFeedbackDue);

  seed_feedback();Axes[1].commanded=true;Axes[1].command_tick=clock_ms-1;
  assert(Config[C_ARRIVAL_HOLD_MS]==200 && !arrived(1,clock_ms));
  uint32_t hold_start=clock_ms;bool complete=false;
  for(unsigned n=0;n<280 && !complete;n++) {
    run_poll(1);complete=arrived(1,clock_ms);
    if(clock_ms-hold_start<200)assert(!complete);
  }
  assert(complete && clock_ms-hold_start>=200);

  /* Low configured freshness and tick wrap must not starve held axes. */
  seed_feedback();Config[C_FEEDBACK_MS]=50;
  run_poll(300);for(unsigned n=0;n<6;n++)assert(max_gap[n]<50);
  seed_feedback();clock_ms=UINT32_MAX-20U;LastPoll=clock_ms;
  for(unsigned i=0;i<3;i++)Axes[i].position_tick=Axes[i].state_tick=clock_ms;
  for(unsigned n=0;n<6;n++)last_read[n]=clock_ms;
  Axes[1].commanded=true;Axes[1].target=10;Axes[1].command_tick=clock_ms;
  run_poll(400);for(unsigned n=0;n<6;n++)assert(reads[n] && max_gap[n]<=220);
  puts("grab_feedback_schedule_test: active-axis latency, parallel fairness, busy, stop, freshness and wrap PASS");
}
