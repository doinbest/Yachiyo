#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void wheel_sample(float left_mm,uint32_t stamp)
{
  ChassisModel_Velocity_t motion={0,left_mm,0};
  ChassisModel_Wheels_t wheels;
  assert(ChassisModel_Inverse(&TrackGeometry,&motion,3000,&wheels));
  for(unsigned i=0;i<4;i++) {
    int64_t logical=(int64_t)llround(wheels.rpm[i]*65536.0/60.0);
    wheel_position[i]=i<2?logical:-logical;
    wheel_position_time[i]=stamp;
  }
}
static void start_motion(GrabTracking_Mode_t mode)
{
  reset();configure(true);TrackMode=mode;
  clock_ms=1000;wheel_sample(0,clock_ms);track_observer(clock_ms);
  clock_ms=1020;wheel_sample(0,clock_ms);track_observer(clock_ms);
  assert(TrackObserver.feedback_valid);
  GrabTracking_Reference(&Tracking,0);
  ChassisModel_Velocity_t command={0,60,0};ChassisModel_Wheels_t wheels;
  assert(ChassisModel_Inverse(&TrackGeometry,&command,3000,&wheels));
  bus.sent_velocity_valid=true;bus.stage=MECANUM_STAGE_SENT;bus.sent_ms=clock_ms;
  for(unsigned i=0;i<4;i++)bus.sent_rpm[i]=wheels.command[i];
  /* Seed an actual post-command feedback sample so predict can establish its
   * encoder-to-command anchor before the following feedback gap. */
  clock_ms=1040;wheel_sample(0,clock_ms);track_observer(clock_ms);
}
static float budget(void)
{
  TrackInput.now_ms=TrackInput.sample_ms=clock_ms;TrackInput.sequence++;
  TrackInput.feedback_age_ms=clock_ms-TrackObserver.feedback_ms;
  TrackInput.valid=true;TrackInput.x_mm=50;TrackInput.x_min_mm=0;TrackInput.x_max_mm=100;
  TrackInput.j[0][0]=2;TrackInput.j[1][1]=TrackInput.j[1][2]=2;
  GrabTracking_Update(&Tracking,track_config(TrackMode),&TrackInput,&TrackOutput);
  return TrackOutput.body_room_mm;
}
int main(void)
{
  const GrabTracking_Mode_t modes[]={GRAB_TRACK_FEEDBACK,GRAB_TRACK_VISUAL,GRAB_TRACK_PREDICT};
  for(unsigned mode=0;mode<3;mode++) {
    start_motion(modes[mode]);
    for(clock_ms=1060;clock_ms<=1300;clock_ms+=20)track_observer(clock_ms);
    assert(TrackObserver.feedback_ms==1040); /* Repeated old samples are not new measurements. */
    assert(fabsf(TrackInput.body_left_mm)<.01f && fabsf(budget()-50)<.01f);
    if(TrackMode==GRAB_TRACK_PREDICT)assert(TrackInput.motion_mm[1]>1);
    else {
      assert(!TrackCommandCount); /* feedback/visual need no command-position predictor. */
      assert(fabsf(TrackInput.motion_mm[1])<.01f);
    }
    clock_ms=1400;wheel_sample(12,clock_ms);track_observer(clock_ms);
    assert(fabsf(budget()-38)<.1f); /* Only measured travel spends the budget. */
    uint32_t measured=TrackObserver.feedback_ms;
    for(clock_ms=1420;clock_ms<=1600;clock_ms+=20)track_observer(clock_ms);
    assert(TrackObserver.feedback_ms==measured && fabsf(budget()-38)<.1f);
  }
  puts("tracking motion: measured budget isolated from command prediction PASS");
  return 0;
}
