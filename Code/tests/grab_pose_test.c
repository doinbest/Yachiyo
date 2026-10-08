#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main
#ifndef GRAB_TASK_POSE
#define GrabTask_PoseStart(work) GrabTask_Start("fixed")
#endif
static void rear_home_direct_car_release(void)
{
  char *cmd[]={"grab","place","car"};
  reset();configure(true);physical[2]=397;
  assert(GrabTask_Command(3,cmd));assert(!GrabTask_IsBusy());
  assert(!strcmp(status().reason,"rear_reference_missing"));
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);configure(true);
  setting("x_max","120");setting("x_car","100");
  setting("x_transfer_speed","55");
  Config[C_Z_GRAB]=Config[C_Z_LIFT]=Config[C_Z_OBSERVE]=NAN;
  tray_ready=false;tray_unset_slot=2; /* Direct arm test does not need indexing. */
  for(unsigned i=0;i<3;i++)tray_inventory[i].state=TURNTABLE_UNKNOWN;
  assert(GrabTask_Command(3,cmd));assert(GrabTask_IsBusy());
  /* A stationary Base within the temporary 2-degree task tolerance must not wait for
   * the driver's tighter position-reached bit. X/Z still require it. */
  base_state_flags=1;
  run_to(GRAB_CAR_EXTEND,600,0);
  assert(!openings && !closures && !camera_requests && !tray_indexes);
  assert(fabsf(physical[2]-307)<.15f);
  assert(base_position_mode==MECHANICAL_ARM_POSITION_ABSOLUTE);
  /* The relaxed driver bit must not allow stale, moving, faulty or off-target Base. */
  {
    GrabAxis_t base=Axes[2], z=Axes[1];
    assert(arrived(2,HAL_GetTick()));
    Axes[2].mm=base.target+2;assert(arrived(2,HAL_GetTick()));
    Axes[2].mm=base.target-2;assert(arrived(2,HAL_GetTick()));
    Axes[2].mm=base.target+2.01f;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].quiet=false;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].flags=5;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[2].position_tick=HAL_GetTick()-3000;assert(!arrived(2,HAL_GetTick()));Axes[2]=base;
    Axes[1]=base;assert(!arrived(1,HAL_GetTick()));Axes[1]=z; /* Release Z still needs reached. */
    /* Real upper-stop return: quiet -0.062 mm, target 0, driver flag=1. */
    Axes[1]=base;Axes[1].target=Config[C_Z_MAX];Axes[1].mm=Config[C_Z_MAX]-.062f;
    assert(arrived(1,HAL_GetTick()));
    Axes[1].target=Axes[1].mm=-5;assert(!arrived(1,HAL_GetTick()));
    Axes[1].target=Config[C_Z_MAX];Axes[1].mm=Config[C_Z_MAX]-2;assert(!arrived(1,HAL_GetTick()));
    Axes[1].mm=Config[C_Z_MAX];Axes[1].flags=5;assert(!arrived(1,HAL_GetTick()));
    Axes[1]=z;
  }

  run_to(GRAB_RELEASE,600,0);
  assert(x_position_rpm==(uint16_t)floorf(55*Config[C_X_PPM]*60/3200));
  assert(fabsf(physical[0]-100)<.01f && fabsf(physical[1]+5)<.01f);
  run_to(GRAB_COMPLETE,600,0);
  assert(openings==1 && !closures && !tray_indexes && homes==3);
  assert(!strcmp(status().reason,"car_place_complete") && !strcmp(status().result,"released"));
  assert(fabsf(physical[2]-307)<.15f && fabsf(physical[0]-100)<.01f && fabsf(physical[1]+5)<.01f);
  for(unsigned i=0;i<3;i++)assert(tray_inventory[i].state==TURNTABLE_UNKNOWN);
}
int main(void)
{
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
  rear_home_direct_car_release();
  reset();configure(true);physical[2]=397;
  GrabTask_BootHomeStart();run_to(GRAB_IDLE,300,0);
  /* Synthetic X/Z plant has 100 pulses/mm; retain that test scale. */
  configure(true);
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,600,0);
  assert(PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy());
  assert(!closures && !openings && !camera_requests && homes==3);
  assert(fabsf(physical[2]-487)<.15f && !strcmp(status().result,"work_ready"));
  assert(GrabTask_ConfigReady("pick"));
  assert(Config[C_X_SPEED]==5 && Config[C_X_TRANSFER_SPEED]==60);
  assert(GrabTask_PoseStart(false));run_to(GRAB_COMPLETE,600,0);
  assert(PendingAction==MECHANICAL_ARM_ACTION_NONE && !MechanicalArm_IsBusy());
  assert(fabsf(physical[2]-397)<.15f && !strcmp(status().result,"travel_ready"));
  assert(physical[0]==Config[C_X_MIN] && physical[1]>=Config[C_Z_CAR_LIFT]);
  chassis_pose_motion=true;
  unsigned before_positions=positions, before_base=base_moves;
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,600,0);
  assert(positions==before_positions+1 && base_moves==before_base+1);
  chassis_pose_motion=false;
  assert(fabsf(physical[2]-487)<.15f && homes==3);
  /* The chassis turn can finish before the first six arm reads. The route
   * remains running: still use the verified rear pose, without X/Z commands. */
  assert(GrabTask_PoseStart(false));run_to(GRAB_COMPLETE,600,0);
  chassis_pose_route=true;before_positions=positions;before_base=base_moves;
  assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,600,0);
  assert(positions==before_positions+1 && base_moves==before_base+1);
  chassis_pose_route=false;
  assert(fabsf(physical[2]-487)<.15f && homes==3);
  assert(isfinite(Config[C_REF_U]) && isfinite(Config[C_J11]));
  /* Exercise real single-piece control after rear-home, including the onboard
   * -90 degree direction and repeated recovery of the same outside anchor. */
  setting("z_proc_place","-6");setting("z_proc_grab","-8");setting("z_proc_lift","5");
  setting("z_temp_place","-8");
  for(unsigned pass=0;pass<4;pass++) for(unsigned slot=1;slot<=3;slot++)
  {
    const char *op=(pass==0 || pass==2)?"store":"take";
    const char *scene=pass==0?"raw":pass==1?"rough":pass==2?"proc":"temp1";
    unsigned retry=30;
    while(!GrabTask_TransferStart(op,scene,(uint8_t)slot,(uint8_t[]){1,5,6}[slot-1]))
    { assert(retry-- && !strcmp(status().reason,"busy"));step(true,0,0,0); }
    run_to(GRAB_CAR_EXTEND,600,0);assert(fabsf(physical[2]-307)<.15f);
    run_to(GRAB_COMPLETE,600,0);
    assert(x_position_rpm==(uint16_t)floorf(60*Config[C_X_PPM]*60/3200));
    assert(fabsf(physical[2]-487)<.15f && homes==3 && GrabTask_ConfigReady("pick"));
    assert(tray_inventory[slot-1].state==(!strcmp(op,"store")?TURNTABLE_OCCUPIED:TURNTABLE_EMPTY));
  }
  assert(closures==12 && camera_requests==3);
  unsigned retry=30;
  while(!GrabTask_PoseStart(false)) { assert(retry--);step(true,0,0,0); }
  run_to(GRAB_COMPLETE,600,0);assert(fabsf(physical[2]-397)<.15f && homes==3);
  GrabTask_ReferenceInvalidate(MECHANICAL_ARM_AXIS_BASE);
  assert(!GrabTask_PoseStart(false));
  puts("grab_pose_test: PASS (rear-home anchor, +/-90 work/car geometry, preserved model)");
}
