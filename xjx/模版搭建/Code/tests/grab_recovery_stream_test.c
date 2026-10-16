#define main original_grab_test_main
#include "grab_task_test.c"
#undef main

static void stopped_ring_stream(void)
{
  reset();configure(true);clock_ms=5000;
  Config[C_LOSS_GRACE_MS]=250;
  Status.mode="align";Status.state=GRAB_VISION_PAUSE;StateTick=4500;
  RingTest=true;RingNumber=1;ActiveColor=CAMERA_COLOR_RED;
  StopChassisSent=StopArmSent=StopArmTx=StopEvidenceStarted=true;
  StopTick=4000;WheelQuiet=true;WheelQuietTick=4500;
  bus.stage=MECANUM_STAGE_STOPPED;bus.tx_complete=true;
  for(unsigned i=0;i<3;i++)Axes[i]=(GrabAxis_t){.position_valid=true,.state_valid=true,
    .position_tick=4990,.state_tick=4990,.quiet=true,.quiet_since=4500,.flags=3};
  camera.UsbConfigured=camera.RequestActive=camera.HasFrame=camera.HasValidData=camera.TargetValid=1;
  camera.RequestFunction=camera.Data.Function=0xb2;
  camera.RequestTarget=camera.Data.Target=10;
  camera.Data.Tick=camera.LastFrameTick=clock_ms-31;camera.Data.Sequence=776;
  RequestTick=4000;RecoveryWaiting=false;RecoveryRestartPending=true;RecoveryPassive=false;
}
static void new_coordinate(void)
{
  clock_ms++;camera.Data.Tick=camera.LastFrameTick=clock_ms;camera.Data.Sequence++;
  vision_pause_process(clock_ms);
}
int main(void)
{
  /* A valid frame can arrive after the function's snapshot, while stop proof
   * is being checked. A restart must not erase that just-recovered stream. */
  stopped_ring_stream();RecoveryWaiting=true;RecoveryTick=4999;
  camera.HasValidData=camera.TargetValid=false;camera_late_recovery=true;
  vision_pause_process(clock_ms);
  assert(!camera_requests && camera.HasValidData && !RecoveryRestartPending);
  assert(RecoveryCount==0 && Status.state==GRAB_VISION_PAUSE && !closures);
  stopped_ring_stream();unsigned requests=camera_requests;
  vision_pause_process(clock_ms);
  assert(Status.stop_confirmed && RecoveryWaiting && camera_requests==requests);
  assert(camera.HasValidData && camera.Data.Sequence==776 && !RecoveryRestartPending);
  uint32_t deadline=RecoveryTick;
  for(unsigned n=0;n<10;n++){clock_ms++;vision_pause_process(clock_ms);}
  assert(RecoveryCount==0 && Status.state==GRAB_VISION_PAUSE); /* Cached frame is not recovery. */
  new_coordinate();assert(RecoveryCount==1 && Status.state==GRAB_VISION_PAUSE);
  new_coordinate();assert(RecoveryCount==2 && Status.state==GRAB_VISION_PAUSE);
  new_coordinate();assert(Status.state==GRAB_ALIGN && camera_requests==requests && !CaptureCount && !closures);

  /* A stream may recover during deceleration, before physical rest is proved. */
  stopped_ring_stream();bus.stage=MECANUM_STAGE_WHEELS;StopChassisSent=StopEvidenceStarted=false;
  PendingAction=MECHANICAL_ARM_ACTION_STATE;pending_valid=true;
  vision_pause_process(clock_ms);assert(!RecoveryRestartPending && !RecoveryWaiting && !Status.stop_confirmed);
  assert(!camera_requests && Status.state==GRAB_VISION_PAUSE && !CaptureCount);

  /* A real renewed absence still restarts exactly once, retaining the window. */
  stopped_ring_stream();vision_pause_process(clock_ms);deadline=RecoveryTick;
  camera.HasValidData=camera.TargetValid=false;camera.InvalidCount++;
  clock_ms+=251;vision_pause_process(clock_ms);
  assert(camera_requests==1 && !RecoveryRestartPending && RecoveryTick==deadline);
  clock_ms++;vision_pause_process(clock_ms);assert(camera_requests==1);

  /* Stale, pre-pause or wrong-target snapshots cannot suppress reacquisition. */
  for(unsigned kind=0;kind<5;kind++) {
    stopped_ring_stream();RecoveryWaiting=true;RecoveryTick=4999;
    if(kind==0)camera.Data.Tick=clock_ms-251;
    if(kind==1)StateTick=clock_ms-10;
    if(kind==2)camera.Data.Target=11;
    if(kind==3){camera.Data.HasReceiveTime=1;camera.Data.ReceivedTick=StateTick-1;}
    if(kind==4){camera.Data.HasReceiveTime=1;camera.Data.ReceivedTick=clock_ms-251;}
    vision_pause_process(clock_ms);assert(camera_requests==1 && RecoveryCount==0);
  }

  /* A BUSY retry is cancelled when the original stream has recovered. */
  stopped_ring_stream();RecoveryWaiting=true;RecoveryTick=4999;
  camera.HasValidData=camera.TargetValid=false;camera_tx_result=HAL_BUSY;
  vision_pause_process(clock_ms);assert(RecoveryRestartPending && camera_attempts==1);
  camera.HasValidData=camera.TargetValid=true;camera.Data.Tick=clock_ms;camera.Data.Sequence++;
  vision_pause_process(clock_ms);assert(!RecoveryRestartPending && camera_attempts==1 && !camera_requests);

  /* Explicit no-grace stops can still preserve a recovered live stream. */
  stopped_ring_stream();Config[C_LOSS_GRACE_MS]=0;vision_pause_process(clock_ms);
  assert(!RecoveryRestartPending && !camera_requests);

  stopped_ring_stream();
  uint32_t shift=(UINT32_MAX-20U)-clock_ms;
  clock_ms+=shift;StateTick+=shift;StopTick+=shift;WheelQuietTick+=shift;RequestTick+=shift;
  for(unsigned i=0;i<3;i++){Axes[i].position_tick+=shift;Axes[i].state_tick+=shift;Axes[i].quiet_since+=shift;}
  camera.Data.Tick+=shift;camera.LastFrameTick+=shift;
  vision_pause_process(clock_ms);assert(!RecoveryRestartPending && !camera_requests);
  clock_ms+=40;camera.Data.Tick=camera.LastFrameTick=clock_ms;camera.Data.Sequence++;
  vision_pause_process(clock_ms);assert(!camera_requests && RecoveryCount==1);

  /* Parsing an old USB packet after rest cannot make it a post-rest sample. */
  stopped_ring_stream();vision_pause_process(clock_ms);
  clock_ms++;camera.Data.HasReceiveTime=1;camera.Data.Tick=clock_ms;
  camera.Data.ReceivedTick=RecoveryTick-1;camera.Data.Sequence++;
  vision_pause_process(clock_ms);assert(RecoveryCount==0 && !camera_requests);
  clock_ms++;camera.Data.Tick=camera.Data.ReceivedTick=clock_ms;camera.Data.Sequence++;
  vision_pause_process(clock_ms);assert(RecoveryCount==1 && !camera_requests);

  /* Bounded recovery remains bounded even after cancelling the restart. */
  stopped_ring_stream();vision_pause_process(clock_ms);deadline=RecoveryTick;
  clock_ms=deadline+(uint32_t)Config[C_VISION_WAIT_MS]+1;
  vision_pause_process(clock_ms);assert(Status.state==GRAB_STOPPING && !strcmp(Status.reason,"vision_recover_timeout"));

  /* Preserve stop decisions; distinguish them from merely missing a frame. */
  stopped_ring_stream();Status.state=GRAB_ALIGN;Status.dx=-90;Status.dy=-80;Vision.Tick=clock_ms-300;
  vision_loss_begin(clock_ms);assert(!strcmp(Status.reason,"vision_gap_expired"));
  stopped_ring_stream();Status.state=GRAB_ALIGN;Status.dx=Status.dy=1;Vision.Tick=clock_ms;
  vision_loss_begin(clock_ms);assert(!strcmp(Status.reason,"vision_pause_near_window"));
  stopped_ring_stream();Status.state=GRAB_VISION_GRACE;Status.dx=-13;Status.dy=-26;
  LossTick=clock_ms-20;GraceSequence=775;camera.Data.Tick=clock_ms;
  camera.Data.DX=10;camera.Data.DY=20;
  vision_grace_process(clock_ms);
  if(strcmp(Status.reason,"vision_pause_direction_change"))fprintf(stderr,"direction diagnostic: state=%u reason=%s count=%u\n",Status.state,Status.reason,GraceCount);
  assert(!strcmp(Status.reason,"vision_pause_direction_change"));
  puts("grab_recovery_stream_test: retain recovered stream, three NEW frames, stop proof, real restart, BUSY, age/epoch/target and deadline PASS");
}
