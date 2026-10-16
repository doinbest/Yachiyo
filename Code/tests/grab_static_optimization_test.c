#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void setup_control(void)
{
  reset();configure(true);Status.mode="align";Status.state=GRAB_ALIGN;
  Config[C_ACCEL]=300;Config[C_BODY_SPEED]=160;Config[C_X_SPEED]=60;
  Config[C_LAMBDA]=2.5f;Config[C_X_MIN]=0;Config[C_X_MAX]=130;
  Config[C_LEAD]=12;Config[C_X_WEIGHT]=2.333333f;
  LastControl=clock_ms;LastPoll=clock_ms;RequestTick=clock_ms;
  ActiveColor=CAMERA_COLOR_RED;HaveCapture=false;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;
  memset(PreviousVelocity,0,sizeof PreviousVelocity);
  Axes[0].mm=Axes[0].target=XIntegral=65;
  for(unsigned i=0;i<3;i++) {
    Axes[i].position_valid=Axes[i].state_valid=true;Axes[i].flags=3;
    Axes[i].position_tick=Axes[i].state_tick=clock_ms;
  }
  camera.UsbConfigured=camera.RequestActive=camera.HasFrame=1;
  camera.HasValidData=camera.TargetValid=1;camera.RequestFunction=camera.Data.Function=0xB2;
  camera.RequestTarget=camera.Data.Target=CAMERA_COLOR_RED;
}
static void pixel(int dx,int dy)
{
  camera.Data.Sequence++;camera.Data.Tick=clock_ms;
  camera.Data.DX=(int16_t)dx;camera.Data.DY=(int16_t)dy;
  assert(vision_read(clock_ms));
}
static void busy_acceleration(bool read)
{
  setup_control();clock_ms+=20;pixel(100,100);align_control(clock_ms);
  assert(velocity_requests==1);
  float f=Status.forward_mm_s,l=Status.left_mm_s,x=PreviousVelocity[2],target=XIntegral;
  PendingAction=MECHANICAL_ARM_ACTION_NONE;pending_valid=false;
  clock_ms+=19;pixel(100,100);
  if(read) {AlignFeedbackDue=true;LastPoll=clock_ms-25;}
  else bus.stage=MECANUM_STAGE_WHEELS;
  align_control(clock_ms);assert(velocity_requests==1 && XIntegral==target);
  PendingAction=MECHANICAL_ARM_ACTION_NONE;pending_valid=false;
  AlignFeedbackDue=false;bus.stage=MECANUM_STAGE_SENT;
  clock_ms++;align_control(clock_ms);
  float delta=sqrtf((Status.forward_mm_s-f)*(Status.forward_mm_s-f)+
    (Status.left_mm_s-l)*(Status.left_mm_s-l)+(PreviousVelocity[2]-x)*(PreviousVelocity[2]-x));
  assert(delta>5.99f && delta<6.01f); /* 300 mm/s2 over 20 ms, not 1 ms. */
  assert(XIntegral-target<.02f); /* Never integrate the 19 ms busy interval. */
}
static void saturated_x(void)
{
  float v[3];setup_control();
  assert(solve(-40,-60,65,v));
  assert(fabsf(v[0]-100)<.01f && fabsf(v[1]-90)<.01f && fabsf(v[2]-60)<.01f);
  assert(solve(40,60,65,v));
  assert(fabsf(v[0]+100)<.01f && fabsf(v[1]+90)<.01f && fabsf(v[2]+60)<.01f);
  Config[C_J01]=.25f;Config[C_J02]=.5f;Config[C_J10]=-.2f;
  assert(solve(-40,-60,65,v));
  assert(fabsf(v[0]-45.238095f)<.01f && fabsf(v[1]-99.047619f)<.01f && fabsf(v[2]-60)<.01f);
  Config[C_BODY_SPEED]=50;
  assert(solve(-40,-60,65,v));
  assert(hypotf(v[0],v[1])<=50.001f && fabsf(v[2])<=60.001f);
  assert(fabsf((v[0]+.25f*v[1]+.5f*v[2])*1.5f-(-.2f*v[0]+v[1]+v[2]))<.01f);
  setup_control();Config[C_J11]=0; /* X alone supplies the second image dimension. */
  assert(solve(-40,-60,65,v));
  assert(fabsf(v[0]-40)<.01f && fabsf(v[1])<.01f && fabsf(v[2]-60)<.01f);
  setup_control();assert(solve(-4,20,0,v));
  assert(v[2]==0 && fabsf(v[0]-10)<.01f && fabsf(v[1]+50)<.01f);
  assert(solve(-4,-20,130,v));assert(v[2]==0 && fabsf(v[1]-50)<.01f);
  Config[C_J00]=640.0f/450;Config[C_J11]=Config[C_J12]=480.0f/330;
  assert(solve(-40,-100,65,v));
  assert(fabsf(v[0]-70.3125f)<.01f && fabsf(v[1]-111.875f)<.01f && fabsf(v[2]-60)<.01f);
}
static void start_settle(void)
{
  reset();configure(true);assert(GrabTask_Start("align"));
  run_to(GRAB_ALIGN,180,1);run_to(GRAB_SETTLE,300,0);Config[C_AGE_MS]=5000;
}
static void settled_window(void)
{
  start_settle();assert(VisualStable); /* Stop must keep the completed image window. */
  unsigned i=0;
  while(!StopEvidenceStarted && i++<100)step(false,0,0,0);
  assert(StopEvidenceStarted);
  /* Real stop evidence can mature without a new camera frame. It cannot finish. */
  for(i=0;i<35;i++)step(false,0,0,0);
  assert(Status.state==GRAB_SETTLE && Status.stop_confirmed && VisualStable);
  step(true,0,0,0);
  assert(Status.state==GRAB_HOLD); /* One new in-window frame; no second image hold. */
}
static void drift_discards_window(void)
{
  start_settle();step(true,0,10,0);assert(!VisualStable);
  for(unsigned i=0;i<100 && Status.state==GRAB_SETTLE;i++)step(true,0,10,0);
  assert(Status.state==GRAB_ALIGN && !closures);
}
static void newer_motion_blocks_old_stop(void)
{
  start_settle();
  for(unsigned i=0;i<40;i++)step(false,0,0,0);
  assert(Status.state==GRAB_SETTLE && Status.stop_confirmed);
  wheel_moving=true;step(true,0,0,0);
  assert(Status.state==GRAB_SETTLE && !closures); /* A latched stop is not current rest. */
  wheel_moving=false;
  for(unsigned i=0;i<30 && Status.state==GRAB_SETTLE;i++)step(true,0,0,0);
  assert(Status.state==GRAB_HOLD);
}
static void changing_z_leaves_xy_opportunities(void)
{
  setup_control();StaticAlignTest=RingApproachStarted=true;
  RingApproachSequence=0;StationAlignRaised=false;RingAlignZ=-5;
  Axes[1].mm=physical[1]=20;
  for(unsigned i=0;i<30;i++)step(true,0,40,235-(int)i*3);
  assert(Status.state==GRAB_ALIGN && positions>5 && velocity_requests>5);
}
int main(int argc,char **argv)
{
  if(argc==1) {
    busy_acceleration(false);busy_acceleration(true);saturated_x();
    settled_window();drift_discards_window();newer_motion_blocks_old_stop();
    changing_z_leaves_xy_opportunities();
    puts("static optimization: timing, allocation, stop and drift PASS");return 0;
  }
  assert(argc==2);
  if(!strcmp(argv[1],"busy"))busy_acceleration(false);
  else if(!strcmp(argv[1],"read"))busy_acceleration(true);
  else if(!strcmp(argv[1],"speed"))saturated_x();
  else if(!strcmp(argv[1],"settle"))settled_window();
  else if(!strcmp(argv[1],"drift"))drift_discards_window();
  else if(!strcmp(argv[1],"z_fair"))changing_z_leaves_xy_opportunities();
  else if(!strcmp(argv[1],"rest"))newer_motion_blocks_old_stop();
  else assert(false);
  printf("static optimization %s: PASS\n",argv[1]);return 0;
}
