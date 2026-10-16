#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void start_settle(void)
{
  reset();configure(true);assert(GrabTask_Start("align"));
  run_to(GRAB_ALIGN,180,1);run_to(GRAB_SETTLE,300,0);Config[C_AGE_MS]=5000;
}

static void fresh_axes(void)
{
  pending_valid=false;
  for(unsigned i=0;i<3;i++) {
    MechanicalArm_AxisTypeDef axis=i==2?MECHANICAL_ARM_AXIS_BASE:i==0?MECHANICAL_ARM_AXIS_X:MECHANICAL_ARM_AXIS_Z;
    MechanicalArm_EventTypeDef e={0};e.Axis=axis;e.Result=MECHANICAL_ARM_RESULT_OK;
    e.Action=PendingAction=MECHANICAL_ARM_ACTION_POSITION_READ;PendingAxis=axis;
    e.CurrentPositionRaw=Axes[i].raw;GrabTask_MotorEventHandle(&e);
    e.Action=PendingAction=MECHANICAL_ARM_ACTION_STATE;PendingAxis=axis;
    e.StateFlags[axis]=3;GrabTask_MotorEventHandle(&e);
  }
}
static void continuous_rest_needs_new_samples_not_another_hold(void)
{
  start_settle();
  for(unsigned i=0;!StopEvidenceStarted && i<100;i++)step(false,0,0,0);
  assert(StopEvidenceStarted && VisualStable);
  clock_ms++;fresh_axes();assert(wheels(clock_ms));stop_process(clock_ms,false);
  assert(clock_ms-StopTick<Config[C_STABLE_MS]);
  assert(Status.stop_confirmed); /* New post-stop samples bridge continuous rest. */
  assert(Status.state==GRAB_SETTLE); /* No new image: cannot complete. */
  camera.Data.Sequence++;camera.Data.Tick=++clock_ms;
  camera.Data.HasReceiveTime=1;camera.Data.ReceivedTick=StopTick-1;
  GrabTask_Process();assert(Status.state==GRAB_SETTLE); /* Late parsing is not new receipt. */
  camera.Data.Sequence++;camera.Data.Tick=camera.Data.ReceivedTick=++clock_ms;
  GrabTask_Process();assert(Status.state==GRAB_HOLD);
}
static void station_setup(void)
{
  reset();configure(true);setting("x_min","0");setting("x_max","130");
  setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
  setting("z_observe","0");setting("z_proc_place","-120");setting("z_stack_place","-110");
  RearReferenceValid=true;RearBase=0;physical[2]=GRAB_BASE_WORK_OFFSET_DEG;
}
static void station_recognition_overlaps_preparation(void)
{
  const char *scenes[]={"rough","temp1","temp2"};
  for(unsigned i=0;i<3;i++) {
    station_setup();assert(GrabTask_AlignPlaceStartForSlot(scenes[i],2,true,1));
    assert(Status.state==GRAB_PREPARE && camera_requests==1);
    assert(requested_color==(i==2?2:11));
    unsigned requests=camera_requests;
    run_to(GRAB_ACQUIRE,300,0);assert(camera_requests==requests);
    assert(!HaveCapture && !VisualStable); /* Preparation images cannot complete. */
    run_to(GRAB_HOLD,300,0);assert(camera_requests==requests);
  }
  station_setup();assert(GrabTask_ProcessedStoreStartEx(2,1,1,true));
  assert(Status.state==GRAB_PREPARE && camera_requests==1 && requested_color==2);
  run_to(GRAB_ACQUIRE,300,0);assert(camera_requests==1 && !HaveCapture);
}
static void preparation_pixels_are_not_acquisition(void)
{
  station_setup();assert(GrabTask_StationVisionPrepare(11));
  unsigned requests=camera_requests;
  for(unsigned i=0;i<5;i++)assert(GrabTask_StationVisionPrepare(11));
  assert(camera_requests==requests);
  assert(GrabTask_AlignPlaceStartForSlot("rough",2,true,1));
  run_to(GRAB_ACQUIRE,300,0);assert(camera_requests==requests);
  step(false,0,0,0);assert(Status.state==GRAB_ACQUIRE && !HaveCapture);
  /* A USB frame parsed after pose-ready but received before it is also old. */
  camera.Data.HasReceiveTime=1;camera.Data.ReceivedTick=RequestTick-1;
  camera.Data.Sequence++;camera.Data.Tick=++clock_ms;
  GrabTask_Process();assert(Status.state==GRAB_ACQUIRE && !HaveCapture);
  camera.Data.Sequence++;camera.Data.Tick=camera.Data.ReceivedTick=++clock_ms;
  GrabTask_Process();assert(Status.state==GRAB_ALIGN && HaveCapture && !VisualStable);
  assert(camera_requests==requests && !closures);
  assert(!GrabTask_StationVisionPrepare(10)); /* Active alignment owns its stream. */
}
static void cancelled_preparation_is_not_adopted(void)
{
  station_setup();assert(GrabTask_StationVisionPrepare(10));
  unsigned requests=camera_requests,stopped=camera_stops;
  GrabTask_StationVisionCancel();assert(camera_stops==stopped+1);
  assert(GrabTask_AlignPlaceStartForSlot("rough",1,true,1));
  assert(camera_requests==requests+1);
  assert(GrabTask_Pause());requests=camera_requests;
  assert(!GrabTask_StationVisionPrepare(10) && camera_requests==requests);
}
static void interrupted_rest_must_accumulate_again(unsigned kind)
{
  start_settle();
  for(unsigned i=0;!StopEvidenceStarted && i<100;i++)step(false,0,0,0);
  assert(StopEvidenceStarted);clock_ms++;
  if(kind==0){wheel_moving=true;(void)wheels(clock_ms);wheel_moving=false;}
  if(kind==1){bus.motion_sequence++;stop_process(clock_ms,false);}
  if(kind==2){stable_reset();}
  fresh_axes();(void)wheels(clock_ms);stop_process(clock_ms,false);
  assert(!Status.stop_confirmed && !closures);
  for(unsigned i=0;i<30 && Status.state==GRAB_SETTLE;i++)step(true,0,0,0);
  assert(Status.state==GRAB_HOLD);
}
int main(int argc,char **argv)
{
  if(argc>1 && !strcmp(argv[1],"prefetch"))station_recognition_overlaps_preparation();
  else {
    continuous_rest_needs_new_samples_not_another_hold();
    station_recognition_overlaps_preparation();preparation_pixels_are_not_acquisition();
    cancelled_preparation_is_not_adopted();
    for(unsigned i=0;i<3;i++)interrupted_rest_must_accumulate_again(i);
  }
  puts("static preparation: PASS");return 0;
}
