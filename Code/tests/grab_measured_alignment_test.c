/* Actual-motion continuation and finite visual target regressions. */
#define main legacy_grab_main
#include "grab_task_test.c"
#undef main
static void sample(uint32_t time,float forward,float left,float x,float yaw)
{
  clock_ms=time;
  test_static_position.origin_valid=test_static_position.feedback_valid=test_static_position.speed_valid=true;
  test_static_position.units_per_rev=65536;test_static_position.generation=1;
  test_static_position.feedback_tick=time;
  test_static_position.observer.feedback=(ChassisModel_Pose_t){forward,left,yaw};
  Axes[0].mm=x;Axes[0].position_tick=time;
  for(unsigned i=0;i<3;i++){
    Axes[i].position_valid=Axes[i].state_valid=true;Axes[i].flags=3;
    Axes[i].state_tick=time;Axes[i].quiet=true;
    if(i)Axes[i].position_tick=time;
  }
  static_alignment_observe(time);
}
static void setup_target(void)
{
  reset();configure(true);Status.state=GRAB_ALIGN;Status.mode="align";
  Config[C_LOSS_GRACE_MS]=250;
  StaticAlignTest=StationAlignRaised=true;RingTest=true;RingNumber=1;
  Config[C_J00]=640.0f/450;Config[C_J01]=Config[C_J02]=Config[C_J10]=0;
  Config[C_J11]=Config[C_J12]=480.0f/330;
  Config[C_X_MIN]=0;Config[C_X_MAX]=130;Config[C_DX_TOL]=Config[C_DY_TOL]=1;
  Axes[1].mm=0;Axes[2].mm=117.5f;
  Axes[0].target=XIntegral=20;Axes[1].target=0;
  RequestTick=clock_ms;HaveCapture=false;
  camera.UsbConfigured=camera.RequestActive=camera.HasFrame=1;
  camera.RequestFunction=camera.Data.Function=0xB2;
  camera.RequestTarget=camera.Data.Target=10;
  sample(1000,0,0,20,0);
}
static void pixel(uint32_t time,int dx,int dy)
{
  clock_ms=time;camera.HasValidData=camera.TargetValid=true;
  camera.Data.Tick=camera.Data.ReceivedTick=time;camera.Data.HasReceiveTime=1;
  camera.Data.Sequence++;camera.Data.DX=(int16_t)dx;camera.Data.DY=(int16_t)dy;
  assert(vision_read(time));
}
static void measured_remainder_and_new_frame(void)
{
  float dx,dy;setup_target();pixel(1010,-64,-48);
  assert(!static_alignment_residual(clock_ms,&dx,&dy)); /* No extrapolation to frame time. */
  sample(1020,0,0,20,0);assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(fabsf(dx+64)<.001f && fabsf(dy+48)<.001f);
  sample(1040,20,4,26,0);assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(fabsf(dx-(-64+20*640.0f/450))<.001f);
  assert(fabsf(dy-(-48+10*480.0f/330))<.001f); /* Chassis and X both count. */
  float oldx=dx,oldy=dy;
  for(unsigned i=0;i<30;i++)assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(dx==oldx && dy==oldy); /* Repeated reads cannot spend or replenish distance. */
  pixel(1040,-14,-8);assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(fabsf(dx+14)<.001f && fabsf(dy+8)<.001f); /* New measurement replaces prior target. */
  sample(1060,40,14,36,0);assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(dx==0 && dy==0); /* No reverse/repeated move after crossing the old target. */
}
static void loss_continues_only_to_measured_target(void)
{
  float dx,dy;setup_target();pixel(1010,-64,-48);sample(1020,0,0,20,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  unsigned before=stops,requests=camera_requests;
  camera.HasValidData=camera.TargetValid=false;camera.InvalidCount++;
  for(unsigned i=1;i<=8;i++){
    sample(1020+50*i,2*i,0,20+i,0);
    assert(vision_read(clock_ms));assert(Status.state==GRAB_ALIGN && AlignMeasured);
    assert(!FreshVision && !VisualStable && !CaptureCount && stops==before);
  }
  assert(camera_requests==requests && !closures);
  PendingAction=MECHANICAL_ARM_ACTION_NONE;pending_valid=false;
  LastControl=LastXControl=clock_ms-20;align_control(clock_ms);
  assert(velocity_requests>0 && Status.forward_mm_s>=0);
  sample(1500,45,0,53,0);
  assert(!vision_read(clock_ms));assert(Status.state==GRAB_VISION_PAUSE);
  assert(Status.stop_requested && !Status.stop_confirmed && !VisualStable && !closures);
}
static void invalidation_and_observation_geometry(void)
{
  float dx,dy;
  for(unsigned kind=0;kind<6;kind++){
    setup_target();pixel(1010,-64,-48);sample(1020,0,0,20,0);
    assert(static_alignment_residual(clock_ms,&dx,&dy));
    if(kind==0)test_static_position.feedback_valid=false;
    if(kind==1)test_static_position.generation++;
    if(kind==2)Axes[1].mm=5;
    if(kind==3)Axes[2].mm+=5;
    if(kind==4)clock_ms+=601;
    if(kind==5)GrabTask_Stop();
    static_alignment_observe(clock_ms);
    assert(!static_alignment_residual(clock_ms,&dx,&dy));
  }
  setup_target();sample(1020,0,0,20,CHASSIS_MODEL_PI/2);pixel(1020,-64,-48);
  sample(1040,0,20,20,CHASSIS_MODEL_PI/2);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(fabsf(dx-(-64+20*640.0f/450))<.001f && fabsf(dy+48)<.001f);
}
static void receive_time_and_common_measurement(void)
{
  float dx,dy;setup_target();
  pixel(1010,-64,-48);sample(1020,10,0,22,0);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(fabsf(dx-(-64+5*640.0f/450))<.001f);
  assert(fabsf(dy-(-48+1*480.0f/330))<.001f); /* Anchor interpolates inside the bracket. */
  test_static_position.observer.feedback.x_mm=20;test_static_position.feedback_tick=1040;
  clock_ms=1040;static_alignment_observe(clock_ms);
  assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(fabsf(dx-(-64+5*640.0f/450))<.001f); /* X has no1040 sample; do not mix times. */
}
static void delayed_bracket_can_resume_the_budget(void)
{
  setup_target();pixel(1010,-64,-48);
  camera.HasValidData=camera.TargetValid=false;camera.InvalidCount++;
  clock_ms=1015;assert(!vision_read(clock_ms));assert(Status.state==GRAB_VISION_GRACE);
  sample(1020,0,0,20,0);vision_grace_process(clock_ms);
  assert(Status.state==GRAB_ALIGN && AlignMeasured && !FreshVision && !Status.stop_requested);
}
static void timestamp_wrap_and_new_target(void)
{
  float dx,dy;setup_target();static_alignment_reset();RequestTick=UINT32_MAX-50U;
  sample(UINT32_MAX-40U,0,0,20,0);pixel(UINT32_MAX-20U,-64,-48);
  sample(0,10,0,22,0);assert(static_alignment_residual(clock_ms,&dx,&dy));
  assert(dx<0 && dx>-64 && dy<0 && dy>-48);
  RingNumber=2;static_alignment_observe(clock_ms);
  assert(!static_alignment_residual(clock_ms,&dx,&dy));
}
static void no_duplicate_control_before_time_bracket(void)
{
  setup_target();pixel(1010,-64,-48);LastControl=LastXControl=1000;
  align_control(clock_ms);unsigned requests=velocity_requests;float integral=XIntegral;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;pending_valid=false;
  for(unsigned i=1;i<=4;i++){
    clock_ms=1010+10*i;assert(vision_read(clock_ms));align_control(clock_ms);
    assert(velocity_requests==requests && XIntegral==integral);
  }
}
static void one_in_window_axis_does_not_block_the_other(void)
{
  for(unsigned axis=0;axis<2;axis++){
    setup_target();pixel(1010,axis?-64:1,axis?1:-48);sample(1020,0,0,20,0);
    clock_ms=1040;assert(vision_read(clock_ms));LastControl=LastXControl=1020;
    align_control(clock_ms);
    if(axis)assert(Status.forward_mm_s>0 && fabsf(Status.left_mm_s)<.001f);
    else assert(fabsf(Status.forward_mm_s)<.001f && Status.left_mm_s>0);
  }
}
static void new_frame_revokes_wrong_or_spent_x_lead(void)
{
  for(unsigned exhausted=0;exhausted<2;exhausted++){
    setup_target();sample(1020,0,0,40,0);pixel(1020,0,exhausted?0:2);
    Axes[0].target=XIntegral=50;LastControl=LastXControl=1000;
    align_control(clock_ms);
    assert(fabsf(XIntegral-40)<.001f && fabsf(Axes[0].target-40)<.04f);
    assert(PreviousVelocity[2]==0); /* Old forward lead is actively replaced by a hold. */
  }
  /* If a mixed J needed X/body cancellation, holding X reallocates to body. */
  setup_target();Config[C_J02]=.5f;sample(1020,0,0,40,0);pixel(1020,0,2);
  Axes[0].target=XIntegral=50;LastControl=LastXControl=1000;
  align_control(clock_ms);
  assert(fabsf(XIntegral-40)<.001f && PreviousVelocity[2]==0);
  assert(fabsf(Status.forward_mm_s)<.001f && Status.left_mm_s<=0);
}
static void mixed_frame_batches_do_not_throttle_fresh_control(void)
{
  setup_target();Config[C_LAMBDA]=2.5f;Config[C_ACCEL]=300;
  Config[C_BODY_SPEED]=160;Config[C_X_SPEED]=60;
  pixel(1010,-64,-48);static_alignment_reset();
  unsigned stopped=stops,requested=camera_requests;
  LastControl=LastXControl=1010;
  for(unsigned i=1;i<=20;i++){
    clock_ms=1010+30*i;camera.InvalidCount++;camera.Data.Sequence++;
    camera.Data.Tick=camera.Data.ReceivedTick=clock_ms;
    camera.Data.DX=(int16_t)(-64+2*(int)i);camera.Data.DY=(int16_t)(-48+(int)i);
    assert(vision_read(clock_ms) && FreshVision && Status.state==GRAB_ALIGN);
    PendingAction=MECHANICAL_ARM_ACTION_NONE;pending_valid=false;
    AlignFeedbackDue=false;LastPoll=clock_ms; /* A feedback-ready control opportunity. */
    align_control(clock_ms);
    assert(velocity_requests==i && stops==stopped && camera_requests==requested);
    assert(hypotf(Status.forward_mm_s,Status.left_mm_s)<=160.001f);
    assert(fabsf(PreviousVelocity[2])<=60.001f && !closures);
  }
}
int main(void)
{
  /* Geometry history is needed for blind continuation, never for accepting a
   * new valid pixel after an invalid report in the same USB batch. */
  setup_target();pixel(1010,-64,-48);static_alignment_reset();
  LossActive=true;LossTick=1000;
  camera.InvalidCount++;camera.Data.Sequence++;camera.Data.Tick=camera.Data.ReceivedTick=1020;
  camera.Data.DX=-20;camera.Data.DY=-10;clock_ms=1020;
  CaptureCount=5;VisualStable=true;
  assert(vision_read(clock_ms) && FreshVision && Status.state==GRAB_ALIGN);
  assert(Vision.DX==-20 && Vision.DY==-10 && !VisualStable && CaptureCount==0);
  assert(!LossActive); /* A real recovered image ends the old miss even without a map bracket. */
  mixed_frame_batches_do_not_throttle_fresh_control();
  no_duplicate_control_before_time_bracket();one_in_window_axis_does_not_block_the_other();
  new_frame_revokes_wrong_or_spent_x_lead();
  delayed_bracket_can_resume_the_budget();timestamp_wrap_and_new_target();
  measured_remainder_and_new_frame();loss_continues_only_to_measured_target();
  invalidation_and_observation_geometry();receive_time_and_common_measurement();
  puts("grab measured alignment: finite old-frame target, common-time actual displacement, renewal and invalidation PASS");
  return 0;
}
