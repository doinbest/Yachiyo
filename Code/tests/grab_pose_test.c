#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main
#ifndef GRAB_TASK_POSE
#define GrabTask_PoseStart(work) GrabTask_Start("fixed")
#endif
static void pose_stop_can_restart_without_homing(void)
{
  const unsigned stages[]={0,2,4,5};
  for(unsigned trial=0;trial<4;trial++)
  {
    reset();configure(true);RearReferenceValid=RearPose=true;RearBase=37;
    physical[2]=37;physical[0]=5;physical[1]=0;
    setting("j00","2.25");
    assert(GrabTask_PoseStart(true));
    if(stages[trial]==5)base_arrives=false;
    for(unsigned n=0;n<100 && PoseStep!=stages[trial];n++)step(true,0,0,0);
    assert(PoseStep==stages[trial]);
    if(stages[trial]==5)physical[2]=80; /* Stop partway through the owned turn. */
    assert(!openings); /* The claw stays unchanged until Base arrival is verified. */
    GrabTask_Stop();assert(!GrabTask_PoseStart(true));
    run_to(GRAB_IDLE,150,0);
    assert(Status.stop_confirmed && RearReferenceValid && RearBase==37 && Config[C_J00]==2.25f);
    unsigned before=positions;base_arrives=true;
    assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,150,0);
    assert(fabsf(physical[2]-(37+117.5f))<.15f && homes==0 && !closures && openings==1);
    if(stages[trial]==5)assert(positions==before+1); /* Already lifted/retracted: resume Base only. */
    before=positions;
    assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,150,0);
    assert(positions==before); /* Preparing an already verified pose performs reads only. */
  }
}

static void manual_base_home_event(MechanicalArm_ResultTypeDef result)
{
  MechanicalArm_EventTypeDef event={0};
  event.Action=MECHANICAL_ARM_ACTION_HOME;event.Axis=MECHANICAL_ARM_AXIS_BASE;
  event.HomeMode=0;event.Result=result;event.Transmitted=1;event.Acknowledged=result==MECHANICAL_ARM_RESULT_OK;
  test_homed_mask|=1U<<MECHANICAL_ARM_AXIS_BASE;
  assert(!GrabTask_MotorEventHandle(&event));
}

static void pose_recovers_manual_base_home_locally(void)
{
  reset();configure(true);physical[2]=397;
  physical[0]=Config[C_X_MIN];physical[1]=Config[C_Z_CAR_LIFT];
  float x_min=Config[C_X_MIN],z_grab=Config[C_Z_GRAB];
  manual_base_home_event(MECHANICAL_ARM_RESULT_OK);
  assert(!RearReferenceValid);
  assert(GrabTask_PoseStart(true));
  char *status_cmd[]={"grab","status"};assert(GrabTask_Command(2,status_cmd));
  assert(strstr(output,"operation=pose") && strstr(output,"home_scope=base") &&
         strstr(output,"home_started=4 home_verified=0"));
  home_flags=7;
  for(unsigned n=0;n<15;n++)step(true,0,0,0);
  assert(!positions && !homes && !RearReferenceValid); /* ACK and running HOME are not rear proof. */
  home_flags=3;run_to(GRAB_COMPLETE,150,0);
  assert(RearReferenceValid && fabsf(RearBase-397)<.15f && fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f);
  assert(homes==0 && positions==1 && !boot_verify_calls && openings==1 && !closures);
  assert(Config[C_X_MIN]==x_min && Config[C_Z_GRAB]==z_grab && GrabTask_ConfigReady("pick"));

  reset();configure(true);manual_base_home_event(MECHANICAL_ARM_RESULT_ACK_TIMEOUT);
  assert(!GrabTask_PoseStart(true)); /* Unknown command completion cannot manufacture an anchor. */
  reset();configure(true);manual_base_home_event(MECHANICAL_ARM_RESULT_OK);
  MechanicalArm_EventTypeDef abort_event={0};
  abort_event.Action=MECHANICAL_ARM_ACTION_HOME_ABORT;abort_event.Axis=MECHANICAL_ARM_AXIS_BASE;
  abort_event.Transmitted=1;abort_event.Result=MECHANICAL_ARM_RESULT_OK;
  assert(!GrabTask_MotorEventHandle(&abort_event));assert(!GrabTask_PoseStart(true));
  reset();configure(true);setting("base_home_degrade","0");manual_base_home_event(MECHANICAL_ARM_RESULT_OK);home_flags=11;
  assert(GrabTask_PoseStart(true));run_to(GRAB_ERROR,150,0);
  assert(!RearReferenceValid && !positions && !homes);

  reset();configure(true);manual_base_home_event(MECHANICAL_ARM_RESULT_OK);home_flags=7;
  assert(GrabTask_PoseStart(true));
  for(unsigned n=0;n<5;n++)step(true,0,0,0);
  GrabTask_Stop();run_to(GRAB_IDLE,150,0);
  assert(!RearReferenceValid && !positions && !homes && aborts==1);
  for(unsigned n=0;n<20;n++)step(true,0,0,0);
  assert(!positions && !GrabTask_IsBusy()); /* Cancelled verification must not resume the queued pose. */

  /* A later full rehome must identify itself even if the last operation was pose. */
  reset();configure(true);Status.operation="pose";
  GrabTask_BootHomeStart();assert(GrabTask_Command(2,status_cmd));
  assert(strstr(output,"operation=home") && strstr(output,"home_scope=all"));
}
static void rear_home_direct_car_release(float rear)
{
  char *cmd[]={"grab","place","car"};
  reset();configure(true);physical[2]=rear;
  assert(GrabTask_Command(3,cmd));assert(!GrabTask_IsBusy());
  assert(!strcmp(status().reason,"rear_reference_missing"));
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);configure(true);
  setting("x_max","120");
  setting("x_transfer_speed","55");
  Config[C_Z_GRAB]=Config[C_Z_LIFT]=Config[C_Z_OBSERVE]=NAN;
  tray_ready=false;tray_unset_slot=2; /* Direct arm test does not need indexing. */
  for(unsigned i=0;i<3;i++)tray_inventory[i].state=TURNTABLE_UNKNOWN;
  assert(GrabTask_Command(3,cmd));assert(GrabTask_IsBusy());
  /* A stationary Base within the temporary 2-degree task tolerance must not wait for
   * the driver's tighter position-reached bit. X/Z still require it. */
  base_state_flags=1;
  run_to(GRAB_PLACE,600,0);
  assert(openings==1 && !closures && !camera_requests && !tray_indexes);
  assert(fabsf(physical[2]-rear)<.15f);
  assert(base_position_mode==MECHANICAL_ARM_POSITION_ABSOLUTE);
  /* The relaxed driver bit must not allow stale, moving, faulty or off-target Base. */
  {
    GrabAxis_t base=Axes[2], z=Axes[1];
    assert(arrived(2,HAL_GetTick()));
    /* Probe target tolerance against one unchanged, already stable measurement;
     * moving the measured position would correctly restart the drift window. */
    Axes[2].target=base.mm-2;assert(arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].target=base.mm+2;assert(arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].target=base.mm-2.01f;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].quiet=false;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].flags=5;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].position_tick=HAL_GetTick()-3000;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[1]=base;arrival_reset(1);assert(!arrived(1,HAL_GetTick()));Axes[1]=z; /* Z needs its own stable window. */
    /* Real upper-stop return: quiet -0.062 mm, target 0, driver flag=1. */
    Axes[1]=base;Axes[1].target=Config[C_Z_MAX];Axes[1].mm=Config[C_Z_MAX]-.062f;
    Axes[1].arrival_anchor=Axes[1].mm; /* Independently measured stable upper-stop residue. */
    assert(arrived(1,HAL_GetTick()));
    Axes[1].target=Axes[1].mm=-5;assert(!arrived(1,HAL_GetTick()));
    Axes[1].target=Config[C_Z_MAX];Axes[1].mm=Config[C_Z_MAX]-2;assert(!arrived(1,HAL_GetTick()));
    Axes[1].mm=Config[C_Z_MAX];Axes[1].flags=5;assert(!arrived(1,HAL_GetTick()));
    Axes[1]=z;
  }

  run_to(GRAB_RELEASE,600,0);
  assert(x_position_rpm==103 && x_position_acc==configured_x_acc);
  assert(fabsf(physical[0]-Config[C_X_MIN])<.01f && fabsf(physical[1]+5)<.01f);
  run_to(GRAB_COMPLETE,600,0);
  assert(openings==2 && !closures && !tray_indexes && homes==3);
  assert(!strcmp(status().reason,"car_place_complete") && !strcmp(status().result,"released"));
  assert(fabsf(physical[2]-rear)<.15f && fabsf(physical[0]-Config[C_X_MIN])<.01f && fabsf(physical[1]+5)<.01f);
  for(unsigned i=0;i<3;i++)assert(tray_inventory[i].state==TURNTABLE_UNKNOWN);
}
static void base_home_stability_configured_window(void)
{
  const int64_t base_window=(int64_t)floor(GRAB_BASE_HOME_STABLE_TOL_DEG/360.0*65536.0);
  for(unsigned axis=0;axis<3;axis++) {
    reset();GrabTask_BootHomeStart();BootAxisIndex=axis;
    BootPositionValid=true;BootPositionRaw=0;
    MechanicalArm_EventTypeDef event={0};
    event.Action=MECHANICAL_ARM_ACTION_POSITION_READ;
    event.Axis=BootAxes[axis];event.Result=MECHANICAL_ARM_RESULT_OK;
    event.CurrentPositionRaw=event.Axis==MECHANICAL_ARM_AXIS_BASE?base_window:364;
    PendingAction=event.Action;PendingAxis=event.Axis;
    assert(boot_event(&event));
    assert(BootPositionStable==(event.Axis==MECHANICAL_ARM_AXIS_BASE));
    event.CurrentPositionRaw+=event.Axis==MECHANICAL_ARM_AXIS_BASE?base_window+1:365;
    PendingAction=event.Action;PendingAxis=event.Axis;
    assert(boot_event(&event));assert(!BootPositionStable);
  }
}
static void base_home_degraded_continuation(void)
{
  /* Driver HOME failure alone must not discard a stationary usable Base. */
  reset();physical[2]=397;home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=0x0b;
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);
  assert(RearReferenceValid && fabsf(RearBase-397)<.15f && boot_verify_calls==1);
  assert(!strcmp(status().reason,"home_complete_degraded"));
  assert(status().base_home_degraded && speed_reads>=2 && !stops && !aborts);
  char *get[]={"grab","status"};GrabTask_Command(2,get);
  assert(strstr(output,"base_home_degraded=1"));
  configure(true);assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,180,0);
  assert(fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && status().base_home_degraded);
  GrabTask_Stop();assert(status().base_home_degraded);
  home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=3;
  homes=0;test_homed_mask=0;
  physical[0]=physical[1]=0;
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);
  assert(!status().base_home_degraded && !strcmp(status().reason,"home_complete"));

  /* Motor reached=0 is allowed, but speed must be measured, not inferred. */
  reset();physical[2]=37;measured_base_rpm=12;base_state_flags=1;
  home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250;i++)step(false,0,0,0);
  assert(speed_reads && !RearReferenceValid && !boot_verify_calls && !stops);
  measured_base_rpm=-1;
  for(unsigned i=0;i<20;i++)step(false,0,0,0);
  assert(!RearReferenceValid);
  measured_base_rpm=0;run_to(GRAB_IDLE,100,0);
  assert(status().base_home_degraded && fabsf(RearBase-37)<.15f);

  /* HOME still running (even with fail set) is not a quiet reference. */
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=15;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250;i++)step(false,0,0,0);
  assert(!RearReferenceValid && !speed_reads && !boot_verify_calls);
  home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;run_to(GRAB_IDLE,100,0);

  /* Temporary position drift waits and then recovers; no repeated HOME. */
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250;i++) { physical[2]+=6;step(false,0,0,0); }
  assert(!RearReferenceValid && !boot_verify_calls && !stops);
  run_to(GRAB_IDLE,100,0);assert(homes==3 && status().base_home_degraded);

  /* Hard faults and missing feedback cannot be accepted as HOME success. */
  const uint8_t faults[]={0x1b,0x2b,0x09};
  for(unsigned i=0;i<sizeof faults;i++) {
    reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=faults[i];GrabTask_BootHomeStart();
    run_to(GRAB_ERROR,300,0);assert(!RearReferenceValid && !boot_verify_calls);
  }
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;base_state_flags=9;
  GrabTask_BootHomeStart();run_to(GRAB_ERROR,300,0);
  assert(!RearReferenceValid && !boot_verify_calls);
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;GrabTask_BootHomeStart();
  while(!speed_reads)step(false,0,0,0);
  pending.Result=MECHANICAL_ARM_RESULT_ACK_TIMEOUT;
  run_to(GRAB_ERROR,100,0);assert(!RearReferenceValid && !boot_verify_calls);

  /* A cancelled quiet check never starts the queued pose or IMU. */
  reset();home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;GrabTask_BootHomeStart();
  while(!speed_reads)step(false,0,0,0);
  GrabTask_Stop();run_to(GRAB_IDLE,100,0);
  for(unsigned i=0;i<30;i++)step(false,0,0,0);
  assert(!RearReferenceValid && !boot_verify_calls && homes==3);

  /* Strict mode remains selectable; explicit acceptance retries Base only. */
  char *accept[]={"grab","home","accept"};
  reset();GrabTask_Command(3,accept);assert(!GrabTask_IsBusy() && !RearReferenceValid);
  setting("base_home_degrade","0");home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;
  GrabTask_BootHomeStart();run_to(GRAB_ERROR,300,0);
  assert(!RearReferenceValid && !boot_verify_calls && homes==3);
  GrabTask_Command(3,accept);run_to(GRAB_IDLE,150,0);
  assert(RearReferenceValid && status().base_home_degraded && homes==3 && boot_verify_calls==1);
  assert(Config[C_BASE_HOME_DEGRADE]==0); /* Acceptance does not rewrite policy. */

  reset();home_disabled_axis=2;home_disabled_reads=-1;measured_base_rpm=1;
  home_axis_flags[MECHANICAL_ARM_AXIS_BASE]=11;GrabTask_BootHomeStart();
  for(unsigned i=0;i<250;i++)step(false,0,0,0);
  assert(!recovery_enables && !RearReferenceValid);
  measured_base_rpm=0;run_to(GRAB_IDLE,150,0);
  assert(recovery_enables==1 && status().base_home_degraded && boot_verify_calls==1);

  /* Accepted manual near HOME can recover the reference and queued work pose. */
  reset();configure(true);physical[2]=397;home_flags=11;
  manual_base_home_event(MECHANICAL_ARM_RESULT_OK);
  assert(GrabTask_PoseStart(true));status_write(true);run_to(GRAB_COMPLETE,180,0);
  assert(status().base_home_degraded && fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && !homes);
}
static void heading_accepts_higher_pose_without_rebasing_z(void)
{
  reset();configure(true);RearReferenceValid=RearPose=true;RearBase=37;
  setting("z_min","-150");setting("z_car_lift","-10");
  setting("head_proc_gain","1");
  physical[0]=Config[C_X_MIN];physical[1]=0;physical[2]=37;
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,200,0);
  for(unsigned n=0;n<100;n++){(void)GrabTask_HeadingReady();step(true,0,0,0);}
  assert(GrabTask_HeadingReady() && fabsf(Axes[1].mm)<.01f);
  StationHeading_Config_t heading;GrabTask_HeadingConfigGet(2,&heading);
  assert(heading.observation_z_mm==Axes[1].mm && !heading.height_matches);
  assert(heading.gain==1 && Config[C_HEAD_PROC_GAIN]==1);
  assert(heading.wait_ms==5000);
  /* Fixed descent sends the absolute -130 target, never -120 from an assumed -10 start. */
  unsigned retries=20;
  while(!axis_move(1,-130)){assert(retries--);step(true,0,0,0);}
  assert(Status.z_target_mm==-130 && Axes[1].target==-130);
  /* Relative tray depth has an explicit z_max reference, saved once; it never uses z_car_lift. */
  reset();configure(true);RearReferenceValid=RearPose=true;RearBase=37;
  setting("z_min","-150");setting("z_max","0");setting("z_car_lift","-10");
  physical[0]=Config[C_X_MIN];physical[1]=0;physical[2]=37;
  assert(GrabTask_CarPickStart(3));assert(CarPickLiftTarget==0);
  assert(CarPickTarget==-Config[C_CAR_PICK_DROP_MM]);
}
int main(void)
{
  heading_accepts_higher_pose_without_rebasing_z();
  reset();GrabTask_BootHomeStart();
  while((BootVerifiedMask&3U)!=3U) {
    assert(!openings && homes<=2);step(true,0,0,0);
  }
  assert(openings==1 && homes==2);
  run_to(GRAB_IDLE,300,0);assert(homes==3 && openings==1);

  reset();configure(true);manual_base_home_event(MECHANICAL_ARM_RESULT_OK);
  assert(GrabTask_PoseStart(true));
  BootPositionRaw=-4294967295LL;BootPositionValid=BootHomeFlagsValid=BootMotorFlagsValid=true;
  clock_ms+=59000;status_write(true); /* Large multi-turn diagnostics still fit debug slot. */
  assert(strstr(progress_log,"base_home_degraded=0") && strstr(progress_log,"elapsed="));
  base_home_degraded_continuation();
  base_home_stability_configured_window();
  pose_stop_can_restart_without_homing();
  pose_recovers_manual_base_home_locally();
  /* Real driver symptom: mode 2 goes to the small correction value rather
   * than adding it. All owned Base corrections must retain the absolute target. */
  reset();configure(true);physical[2]=397;
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);configure(true);
  assert(GrabTask_PoseStart(true));
  while(PoseStep!=5)step(true,0,0,0);
  step(true,0,0,0); /* Consume the coarse turn. */
  physical[2]=Axes[2].target-2.5f;
  base_mode2_absolute=true;
  run_to(GRAB_COMPLETE,150,0);
  assert(base_position_mode==MECHANICAL_ARM_POSITION_ABSOLUTE);
  assert(fabsf(physical[2]-Axes[2].target)<=1 && base_moves==2);
  rear_home_direct_car_release(37);
  rear_home_direct_car_release(397);
  rear_home_direct_car_release(-323);
  reset();configure(true);physical[2]=397;
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);
  /* Synthetic X/Z plant has 100 pulses/mm; retain that test scale. */
  configure(true);
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,600,0);
  assert(PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy());
  assert(!closures && openings==2 && !camera_requests && homes==3);
  assert(fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && !strcmp(status().result,"work_ready"));
  assert(GrabTask_ConfigReady("pick"));
  assert(Config[C_X_SPEED]==5 && Config[C_X_TRANSFER_SPEED]==60);
  assert(GrabTask_PoseStart(false));run_to(GRAB_COMPLETE,600,0);
  assert(PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy());
  assert(fabsf(physical[2]-397)<.15f && !strcmp(status().result,"travel_ready"));
  assert(physical[0]==Config[C_X_MIN] && physical[1]>=Config[C_Z_CAR_LIFT]);
  unsigned qr_openings=openings;
  chassis_pose_motion=true; /* QR Base preparation on post-curve straight. */
  assert(GrabTask_QrPoseStart());run_to(GRAB_COMPLETE,600,0);
  chassis_pose_motion=false;
  assert(fabsf(physical[2]-432)<.15f && !strcmp(status().result,"qr_pose_ready"));
  assert(openings==qr_openings && !RearPose && !OutsidePose);
  assert(GrabTask_PoseStart(false));run_to(GRAB_COMPLETE,600,0);
  assert(fabsf(physical[2]-397)<.15f);
  chassis_pose_motion=true;
  unsigned before_positions=positions, before_base=base_moves;
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,600,0);
  assert(positions==before_positions+1 && base_moves==before_base+1);
  chassis_pose_motion=false;
  assert(fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && homes==3);
  /* The chassis turn can finish before the first six arm reads. The route
   * remains running: still use the verified rear pose, without X/Z commands. */
  assert(GrabTask_PoseStart(false));run_to(GRAB_COMPLETE,600,0);
  chassis_pose_route=true;before_positions=positions;before_base=base_moves;
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,600,0);
  assert(positions==before_positions+1 && base_moves==before_base+1);
  chassis_pose_route=false;
  assert(fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && homes==3);
  assert(isfinite(Config[C_J00]) && isfinite(Config[C_J11]));
  /* All ID8 slots share the same Base rear-home loading direction; repeated
   * store/take cycles must return to the same 117.5-degree outside anchor. */
  setting("z_proc_place","-6");

  for(unsigned pass=0;pass<4;pass++) for(unsigned slot=1;slot<=3;slot++)
  {
    const char *op=(pass==0 || pass==2)?"store":"take";
    const char *scene=pass==0?"raw":pass==1?"rough":pass==2?"proc":"temp1";
    unsigned retry=30;
    while(!GrabTask_TransferStart(op,scene,(uint8_t)slot,(uint8_t[]){1,5,6}[slot-1]))
    { assert(retry-- && !strcmp(status().reason,"busy"));step(true,0,0,0); }
    run_to(GRAB_PLACE,600,0);assert(fabsf(physical[2]-397)<.15f);
    run_to(GRAB_COMPLETE,600,0);
    assert(x_position_rpm==112 && x_position_acc==configured_x_acc);
    assert(fabsf(physical[2]-(397+GRAB_BASE_WORK_OFFSET_DEG))<.15f && homes==3 && GrabTask_ConfigReady("pick"));
    assert(tray_inventory[slot-1].state==(!strcmp(op,"store")?TURNTABLE_OCCUPIED:TURNTABLE_EMPTY));
  }
  assert(closures==12 && camera_requests==3);
  unsigned retry=30;
  while(!GrabTask_PoseStart(false)) { assert(retry--);step(true,0,0,0); }
  run_to(GRAB_COMPLETE,600,0);assert(fabsf(physical[2]-397)<.15f && homes==3);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(!GrabTask_PoseStart(false));
  puts("grab_pose_test: PASS (rear-home car direction, +117.5 work geometry, preserved model)");
}
