/* Production controller exercised without the legacy fixture's 50 ms step.
 * Prepared while C builds are deferred by the user; run via host suite later. */
#define main grab_original_main
#include "grab_task_test.c"
#undef main

static void setup(void)
{
  reset();configure(true);Status.mode="align";Status.state=GRAB_ALIGN;
  memset(PreviousVelocity,0,sizeof PreviousVelocity); /* Bypass normal stop/start setup explicitly. */
  ActiveColor=CAMERA_COLOR_RED;LastControl=clock_ms;
  HaveCapture=FreshVision=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  RequestTick=clock_ms;LastSequence=0;
  Axes[0].mm=Axes[0].target=0;
  for(unsigned i=0;i<3;i++){
    Axes[i].position_valid=Axes[i].state_valid=true;Axes[i].flags=3;
    Axes[i].position_tick=Axes[i].state_tick=clock_ms;
  }
  camera.UsbConfigured=camera.RequestActive=camera.HasFrame=1;
  camera.HasValidData=camera.TargetValid=1;camera.RequestFunction=camera.Data.Function=0xB2;
  camera.RequestTarget=camera.Data.Target=CAMERA_COLOR_RED;
}
static void frame(int dx,int dy)
{
  camera.Data.Sequence++;camera.Data.Tick=clock_ms;
  camera.Data.DX=(int16_t)dx;camera.Data.DY=(int16_t)dy;
  assert(vision_read(clock_ms));
}
int main(void)
{
  setup();clock_ms+=16;frame(40,0);align_control(clock_ms);
  assert(velocity_requests==1 && Status.forward_mm_s<0); /* No 100 ms gate. */
  clock_ms+=16;frame(40,0);align_control(clock_ms);
  assert(velocity_requests==2); /* Equal pixels, different observation. */
  clock_ms++;assert(vision_read(clock_ms));align_control(clock_ms);
  assert(velocity_requests==2); /* Duplicate snapshot does not move again. */

  setup();clock_ms+=16;frame(40,0);bus.stage=MECANUM_STAGE_WHEELS;
  float before=Axes[0].target;align_control(clock_ms);
  assert(velocity_requests==1 && Axes[0].target==before && !AlignControlPending && LastControl==clock_ms);
  clock_ms+=16;frame(-40,0);align_control(clock_ms); /* Latest frame replaces pending. */
  bus.stage=MECANUM_STAGE_SENT;clock_ms++;assert(vision_read(clock_ms));align_control(clock_ms);
  assert(velocity_requests==2 && LastControl==clock_ms-1);
  clock_ms++;assert(vision_read(clock_ms));align_control(clock_ms);assert(velocity_requests==2);

  setup();clock_ms+=16;frame(40,0);pending_valid=true;
  align_control(clock_ms);assert(velocity_requests==1);
  pending_valid=false;clock_ms++;assert(vision_read(clock_ms));align_control(clock_ms);
  assert(velocity_requests==1); /* Arm busy no longer blocks/replays the body observation. */

  setup();Config[C_LOSS_GRACE_MS]=250;clock_ms+=16;frame(40,0);
  vision_loss_begin(clock_ms);assert(Status.state==GRAB_VISION_GRACE);
  Status.state=GRAB_ALIGN;FreshVision=false;align_control(clock_ms);assert(!velocity_requests);
  clock_ms++;frame(40,0);stop_begin();Status.state=GRAB_ALIGN;
  align_control(clock_ms);assert(!velocity_requests); /* Stop cancels pending motion. */

  setup();Config[C_LOSS_GRACE_MS]=250;clock_ms+=16;frame(40,0);align_control(clock_ms);
  clock_ms+=99;assert(vision_read(clock_ms));assert(Status.state==GRAB_ALIGN);
  clock_ms++;assert(!vision_read(clock_ms));assert(Status.state==GRAB_VISION_GRACE);
  align_control(clock_ms);assert(velocity_requests==2 && Status.forward_mm_s==0);

  setup();StaticAlignTest=true;RingApproachStarted=true;RingApproachSequence=0;
  StationAlignRaised=true;RingAlignZ=-5;clock_ms+=16;frame(0,0);
  static_lower_control(clock_ms);assert(positions==1 && RingLowerSent);

  setup();StaticAlignTest=RingApproachStarted=StationAlignRaised=true;
  RingApproachSequence=0;RingAlignZ=-5;clock_ms+=16;
  camera.Data.Sequence++;camera.Data.Tick=clock_ms;camera.Data.DX=0;camera.Data.DY=40;
  /* Frequent X corrections cannot starve a newly eligible Z action. */
  GrabTask_Process();assert(positions==1 && RingLowerSent && velocity_requests==1);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;clock_ms++;
  GrabTask_Process(); /* Due feedback may take its fair turn before XY. */
  if(!velocity_requests){pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;clock_ms++;GrabTask_Process();}
  assert(velocity_requests==1); /* Same pending frame after Z ACK/read. */

  setup();uint32_t started=clock_ms;
  for(unsigned i=0;i<65;i++){
    step(true,0,0,10); /* Every 50 ms loop also carries a new X correction. */
    assert(Status.state==GRAB_ALIGN);
  }
  assert(velocity_requests>10 && Axes[0].position_tick>started+2000U);
  for(unsigned i=0;i<3;i++)assert(axis_fresh(i,clock_ms));

  setup();StaticAlignTest=RingApproachStarted=true;StationAlignRaised=false;
  RingApproachSequence=0;RingAlignZ=-5;Axes[1].mm=physical[1]=20;started=clock_ms;
  for(unsigned i=0;i<65;i++){
    step(true,0,0,235+(int)(i%2)); /* Alternating legal low-plane Z targets. */
    assert(Status.state==GRAB_ALIGN);
  }
  assert(Axes[1].position_tick>started+2000U);
  for(unsigned i=0;i<3;i++)assert(axis_fresh(i,clock_ms));
  puts("grab_static_frame_test: PASS (new frames, busy handoff, duplicates, loss, stop, Z)");
  return 0;
}
