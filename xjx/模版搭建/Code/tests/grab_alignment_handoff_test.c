#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void setup_alignment(const char *scene,bool raised)
{
  reset();configure(true);
  setting("x_min","0");setting("x_max","130");
  setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
  setting("z_observe","0");setting("z_rough_align","-100");setting("z_temp_align","-100");
  setting("z_proc_place","-140");setting("z_stack_place","-110");
  RearBase=37;RearReferenceValid=true;physical[0]=42;
  physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
  assert(GrabTask_AlignPlaceStartEx(scene,!strcmp(scene,"temp2")?5:2,raised));
  run_to(GRAB_HOLD,600,0);
  assert(Status.stop_confirmed && VisualStable && !closures);
}
static void new_pixel(int dx)
{
  clock_ms++;
  camera.Data.Sequence++;camera.Data.Tick=camera.LastFrameTick=clock_ms;
  camera.Data.DX=(int16_t)dx;camera.Data.DY=0;
}
static void capture(void)
{
  assert(HaveCapture); /* HOLD must retain the completed stability window. */
  assert(!GrabTask_AlignPlaceCapture()); /* A repeated snapshot cannot save it. */
  uint32_t started=clock_ms;
  new_pixel(0);
  assert(GrabTask_AlignPlaceCapture());
  assert(clock_ms-started<Config[C_STABLE_MS] && AlignedPlace.valid);
  assert(Status.state==GRAB_COMPLETE && Status.stop_confirmed);
  /* A read already submitted in HOLD may reject pickup once as busy. Its
   * reply can drain without losing the single completed-stop handoff. */
  if(pending_valid) {
    assert(!GrabTask_CarPickStart(2));
    assert(Status.state==GRAB_COMPLETE && AlignedPlace.stop_handoff);
    step(false,0,0,0);
  }
  assert(!pending_valid && PendingAction==MECHANICAL_ARM_ACTION_NONE);
}
static void immediate_pickup(const char *scene,bool raised)
{
  setup_alignment(scene,raised);capture();
  float saved_x=AlignedPlace.x;
  unsigned stopped=stops,requested=camera_requests;
  assert(GrabTask_CarPickStart(2));
  assert(Status.stop_confirmed && Axes[0].position_valid && WheelQuiet);
  GrabTask_Process();
  assert(Status.state==GRAB_RETRACT && stops==stopped);
  run_to(GRAB_COMPLETE,600,0);
  assert(closures==1 && TrackHeld && AlignedPlace.picked);
  assert(fabsf(physical[0]-30)<.11f);
  unsigned opened=openings;
  assert(GrabTask_AlignedPlaceStart(scene,!strcmp(scene,"temp2")?5:2,5));
  camera_valid=false;
  run_to(GRAB_RELEASE,600,0);
  assert(fabsf(physical[0]-saved_x)<.11f && openings==opened+1);
  assert(fabsf(physical[1]-(!strcmp(scene,"temp2")?-110:-140))<.11f);
  assert(camera_requests==requested); /* Holding material never re-aligns visually. */
}
static void changed_evidence_rebuilds_stop(unsigned change)
{
  setup_alignment("rough",true);capture();
  if(change==0)Axes[0].position_tick=clock_ms-(uint32_t)Config[C_FEEDBACK_MS]-1U;
  else if(change==1)wheel_moving=true;
  else if(change==2)bus.motion_sequence++;
  else if(change==3)bus.stop_sequence++;
  else if(change==4) {
    MechanicalArm_EventTypeDef event={0};
    event.Action=MECHANICAL_ARM_ACTION_POSITION;event.Axis=MECHANICAL_ARM_AXIS_X;
    event.Transmitted=1;event.Result=MECHANICAL_ARM_RESULT_OK;
    assert(!GrabTask_MotorEventHandle(&event));
  } else if(change==5)GrabTask_Stop();
  else assert(!GrabTask_Pause()); /* Stop/pause at COMPLETE still cancel the handoff. */
  assert(GrabTask_CarPickStart(1));
  assert(Status.state==GRAB_PREPARE && !Status.stop_confirmed && !Axes[0].position_valid);
}
static void drift_loss_and_pause_do_not_reuse_stability(void)
{
  setup_alignment("rough",true);new_pixel(10);
  assert(!GrabTask_AlignPlaceCapture() && Status.state==GRAB_ALIGN && !VisualStable);
  setup_alignment("rough",true);
  camera.InvalidCount++;camera.TargetValid=false;
  assert(!GrabTask_AlignPlaceCapture() && Status.state==GRAB_VISION_PAUSE && !VisualStable);
  setup_alignment("rough",true);
  assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
  assert(GrabTask_Continue());run_to(GRAB_ALIGN,300,0);
  assert(!VisualStable && !GrabTask_AlignPlaceCapture());
}
int main(void)
{
  const char *scenes[]={"rough","temp1","temp2"};
  for(unsigned i=0;i<3;i++)for(unsigned raised=0;raised<2;raised++)
    immediate_pickup(scenes[i],raised!=0);
  for(unsigned change=0;change<7;change++)changed_evidence_rebuilds_stop(change);
  drift_loss_and_pause_do_not_reuse_stability();
  puts("alignment handoff: fresh capture, inherited stop, changed evidence and placement PASS");
  return 0;
}
