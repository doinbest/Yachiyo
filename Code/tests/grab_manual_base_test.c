/* Source-only regression prepared while compilation is deferred by the user. */
#define main legacy_grab_task_main
#include "grab_task_test.c"
#undef main

static void setup_work(void)
{
  reset();configure(true);
  RearBase=397;RearReferenceValid=true;OutsideReferenceValid=OutsidePose=true;
  OutsideBase=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
  physical[0]=Config[C_X_MIN];physical[1]=Config[C_Z_CAR_LIFT];physical[2]=OutsideBase;
  assert(GrabTask_VisualResponseSet(3,1,2,4));
  CalibrationReferenceConfirmed=true;
  Config[C_HEAD_PROC_GAIN]=1.25f;Config[C_HEAD_TEMP_GAIN]=-1;Config[C_HEAD_LINE_GAIN]=.75f;
  tray_inventory[0]=(Turntable_Inventory_t){TURNTABLE_OCCUPIED,3};
}
static void manual_position(bool sent,MechanicalArm_ResultTypeDef result)
{
  MechanicalArm_EventTypeDef event={0};
  event.Action=MECHANICAL_ARM_ACTION_POSITION;event.Axis=MECHANICAL_ARM_AXIS_BASE;
  event.Transmitted=sent;event.Result=result;event.Acknowledged=result==MECHANICAL_ARM_RESULT_OK;
  assert(!GrabTask_MotorEventHandle(&event));
}
static void saved_parameters_survive_and_old_measurements_do_not(void)
{
  setup_work();
  float saved[6];memcpy(saved,&Config[C_J00],sizeof saved);
  AlignedPlace.valid=AlignedPlace.stop_handoff=true;ProcClearance.active=true;
  HaveCapture=FreshVision=VisualStable=AlignControlPending=true;
  Tracking.reference_valid=Tracking.have_sample=true;Tracking.samples=Tracking.motions=1;
  ResumePending=NextTrack.pending=true;
  physical[2]=RearBase+30;
  unsigned opened=openings,closed=closures;
  manual_position(true,MECHANICAL_ARM_RESULT_OK);
  assert(RearReferenceValid && RearBase==397 && !RearPose && !OutsidePose);
  assert(!OutsideReferenceValid && !AlignedPlace.valid && !AlignedPlace.stop_handoff);
  assert(!ProcClearance.active && !HaveCapture && !FreshVision && !VisualStable && !AlignControlPending);
  assert(!Tracking.reference_valid && !Tracking.have_sample && !Tracking.samples && !Tracking.motions);
  assert(!ResumePending && !NextTrack.pending);
  assert(!memcmp(saved,&Config[C_J00],sizeof saved) && VisualModelCustom);
  assert(Config[C_HEAD_PROC_GAIN]==1.25f && Config[C_HEAD_TEMP_GAIN]==-1 && Config[C_HEAD_LINE_GAIN]==.75f);
  assert(!GrabTask_CalibrationReferenceValid() && !GrabTask_CalibrationReferenceSet());
  assert(!GrabTask_Start("align") && !strcmp(status().reason,"work_pose_required"));
  assert(!tracking_start(CAMERA_COLOR_RED,0,GRAB_TRACK_FEEDBACK,false,false,true,0,0,50));
  assert(!strcmp(status().reason,"work_pose_required") && !camera_requests && !positions);
  assert(openings==opened && closures==closed && tray_inventory[0].state==TURNTABLE_OCCUPIED);

  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,250,0);
  assert(fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && !homes);
  assert(GrabTask_CalibrationReferenceValid() && !memcmp(saved,&Config[C_J00],sizeof saved));
  assert(RearReferenceValid && RearBase==397 && OutsidePose && VisualModelCustom);
  assert(GrabTask_Start("align"));
  assert(!HaveCapture); /* A new request still needs a new valid observation. */
  run_to(GRAB_ALIGN,250,0);
}
static void unknown_ack_and_stop_cannot_restore_pose(void)
{
  setup_work();physical[2]=RearBase+20;
  manual_position(true,MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
  assert(RearReferenceValid && isfinite(Config[C_J00]));
  assert(!GrabTask_Start("align") && !strcmp(status().reason,"work_pose_required"));
  GrabTask_Stop();
  assert(!GrabTask_Start("align") && !strcmp(status().reason,"work_pose_required"));
  assert(GrabTask_PoseStart(false));run_to(GRAB_COMPLETE,250,0);
  assert(fabsf(physical[2]-397)<.15f);
  assert(!GrabTask_Start("align") && !strcmp(status().reason,"work_pose_required"));
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,250,0);
  assert(tracking_start(CAMERA_COLOR_RED,0,GRAB_TRACK_FEEDBACK,false,false,true,0,0,50));
  assert(Status.state==GRAB_TRACK_OBSERVE && !Tracking.reference_valid && !Tracking.samples);
}
static void preparation_requires_new_stationary_feedback(void)
{
  setup_work();physical[2]=RearBase+20;manual_position(true,MECHANICAL_ARM_RESULT_OK);
  assert(!Axes[2].position_valid && !Axes[2].state_valid);
  base_arrives=false;assert(GrabTask_PoseStart(true));
  assert(Status.state==GRAB_ROUTE_POSE && !positions && !GrabTask_CalibrationReferenceValid());
  for(unsigned i=0;i<150 && !base_moves;i++)step(true,0,0,0);
  assert(base_moves && Status.state==GRAB_ROUTE_POSE && !GrabTask_CalibrationReferenceValid());
  GrabTask_Stop();run_to(GRAB_IDLE,150,0);
  assert(!GrabTask_Start("align") && !strcmp(status().reason,"work_pose_required"));
  base_arrives=true;assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,250,0);
  assert(GrabTask_CalibrationReferenceValid());
}
static void unsent_moves_and_real_origin_changes_are_distinct(void)
{
  setup_work();manual_position(false,MECHANICAL_ARM_RESULT_TX_ERROR);
  assert(RearReferenceValid && OutsidePose && GrabTask_CalibrationReferenceValid());
  const MechanicalArm_ActionTypeDef changes[]={MECHANICAL_ARM_ACTION_ZERO,
    MECHANICAL_ARM_ACTION_HOME,MECHANICAL_ARM_ACTION_ORIGIN_SET};
  for(unsigned i=0;i<sizeof changes/sizeof changes[0];i++){
    setup_work();
    MechanicalArm_EventTypeDef event={0};
    event.Axis=MECHANICAL_ARM_AXIS_BASE;event.Transmitted=1;event.Result=MECHANICAL_ARM_RESULT_OK;
    event.Action=changes[i];event.HomeMode=0;
    assert(!GrabTask_MotorEventHandle(&event));
    assert(!RearReferenceValid && !isfinite(Config[C_J00]) && !GrabTask_CalibrationReferenceValid());
  }
}
int main(void)
{
  saved_parameters_survive_and_old_measurements_do_not();
  unknown_ack_and_stop_cannot_restore_pose();
  preparation_requires_new_stationary_feedback();
  unsent_moves_and_real_origin_changes_are_distinct();
  puts("manual Base: retained model/anchor, invalid old observations, verified work pose and true origin changes PASS");
  return 0;
}
