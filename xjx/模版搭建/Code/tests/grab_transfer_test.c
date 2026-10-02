/* Single-piece transfer regressions use production GrabTask and a synthetic plant. */
#define main grab_legacy_regression_main
#include "grab_task_test.c"
#undef main

static void completed_single_piece_releases_task(void)
{
  reset(); configure(true);
  assert(GrabTask_Start("fixed"));
  run_to(GRAB_COMPLETE,600,0);
  assert(!GrabTask_IsBusy());
  assert(!strcmp(status().result,"stored") && status().slot==1);
  GrabTask_Stop();
  assert(status().state==GRAB_COMPLETE && !strcmp(status().result,"stored"));
  assert(tray_inventory[0].state==TURNTABLE_OCCUPIED);
}

static void two_pieces_return_to_actual_base_anchor(void)
{
  char *second[]={"grab","start","fixed","2","3"};
  reset(); configure(true); physical[2]=397; physical[0]=3;
  assert(GrabTask_Start("fixed")); run_to(GRAB_COMPLETE,600,0);
  assert(fabsf(physical[2]-397)<.12f && fabsf(physical[0]-3)<.01f);
  assert(tray_indexes==1 && base_moves==2 && closures==1);
  assert(tray_inventory[0].state==TURNTABLE_OCCUPIED && tray_inventory[0].color==1);
  assert(isfinite(Config[C_REF_U]) && GrabTask_ConfigReady("pick"));
  assert(!strcmp(status().reference_cause,"transfer_return_verified"));
  assert(GrabTask_Command(5,second)); run_to(GRAB_COMPLETE,600,0);
  assert(fabsf(physical[2]-397)<.12f && base_moves==4 && tray_indexes==2 && closures==2);
  assert(tray_inventory[0].state==TURNTABLE_OCCUPIED && tray_inventory[0].color==1);
  assert(tray_inventory[1].state==TURNTABLE_OCCUPIED && tray_inventory[1].color==3);
  assert(!GrabTask_IsBusy() && GrabTask_ConfigReady("align"));
}

static void scene_parameters_only_block_the_requested_scene(void)
{
  char *proc[]={"grab","store","proc","1","2"};
  char *rough[]={"grab","take","1","rough"};
  char *temp2[]={"grab","take","1","temp2"};
  reset(); configure(false);
  assert(isnan(Config[C_Z_PROC_GRAB]) && isnan(Config[C_Z_STACK_PLACE]));
  assert(GrabTask_ConfigReady("fixed"));
  assert(GrabTask_Command(5,proc)); assert(!GrabTask_IsBusy());
  assert(!strcmp(status().missing,"z_proc_grab"));
  setting("z_proc_grab","-10"); setting("z_proc_lift","5");
  assert(GrabTask_Command(5,proc)); run_to(GRAB_COMPLETE,600,0);
  assert(tray_inventory[0].color==2 && !strcmp(status().scene,"proc"));
  assert(GrabTask_Command(4,rough)); assert(!GrabTask_IsBusy());
  assert(!strcmp(status().missing,"z_proc_place"));
  setting("z_proc_place","-12");
  assert(GrabTask_Command(4,rough)); run_to(GRAB_COMPLETE,600,0);
  assert(tray_inventory[0].state==TURNTABLE_EMPTY && !strcmp(status().result,"released"));
  assert(physical[2]==0 && physical[0]==0 && physical[1]==0);
  (void)Turntable_InventorySet(1,TURNTABLE_OCCUPIED,4);
  assert(GrabTask_Command(4,temp2)); assert(!GrabTask_IsBusy());
  assert(!strcmp(status().missing,"z_stack_place"));
  setting("z_stack_place","-6");
  assert(GrabTask_Command(4,temp2)); run_to(GRAB_COMPLETE,600,0);
  assert(!strcmp(status().scene,"temp2") && tray_inventory[0].state==TURNTABLE_EMPTY);
  assert(isnan(Config[C_Z_TEMP_PLACE])); /* First-layer calibration was not borrowed. */
}

static void unconfirmed_inventory_and_unset_angles_do_not_move(void)
{
  char *fixed[]={"grab","start","fixed","1","1"};
  reset(); configure(false);
  for(unsigned i=0;i<3;i++) tray_inventory[i].state=TURNTABLE_UNKNOWN;
  assert(!GrabTask_Start("fixed") && !positions && !stops);
  assert(!strcmp(status().reason,"no_confirmed_empty_slot"));
  assert(GrabTask_Command(5,fixed)); assert(!GrabTask_IsBusy());
  assert(!strcmp(status().reason,"inventory_unconfirmed"));
  tray_inventory[0].state=TURNTABLE_EMPTY; tray_unset_slot=2;
  assert(GrabTask_Start("fixed")); run_to(GRAB_COMPLETE,600,0);
  assert(tray_indexes==1 && status().slot==1); /* Other angles/heights do not block slot one. */
}

static void cancellation_only_marks_a_touched_slot_and_return_is_local(void)
{
  char *back[]={"grab","return"};
  unsigned before;
  reset(); configure(false); physical[2]=37;
  assert(GrabTask_Start("fixed")); run_to(GRAB_PLACE,600,0);
  while(!Axes[1].commanded) step(true,0,0,0);
  assert(InventoryTouched && !OutsidePose);
  before=openings; GrabTask_Stop(); run_to(GRAB_IDLE,180,0);
  assert(openings==before && closures==1 && homes==0);
  assert(tray_inventory[0].state==TURNTABLE_UNKNOWN);
  assert(tray_inventory[1].state==TURNTABLE_EMPTY && tray_inventory[2].state==TURNTABLE_EMPTY);
  assert(isfinite(Config[C_REF_U]) && !GrabTask_ConfigReady("pick"));
  assert(GrabTask_Command(2,back)); run_to(GRAB_COMPLETE,600,0);
  assert(fabsf(physical[2]-37)<.12f && !strcmp(status().result,"returned"));
  assert(GrabTask_ConfigReady("pick") && homes==0 && tray_inventory[0].state==TURNTABLE_UNKNOWN);
  assert(openings==before); /* Stop and local return retain the claw output. */

  reset(); configure(false); tray_arrives=false;
  assert(GrabTask_Start("fixed")); run_to(GRAB_INDEX,600,0);
  while(!IndexSubmitted) step(true,0,0,0);
  assert(!InventoryTouched); before=openings; GrabTask_Stop(); run_to(GRAB_IDLE,180,0);
  assert(tray_stops && tray_inventory[0].state==TURNTABLE_EMPTY && openings==before);
}

static void pick_and_align_share_gap_control(void)
{
  float speed[2]; unsigned requests[2];
  for(unsigned mode=0;mode<2;mode++)
  {
    reset(); configure(true); setting("loss_grace_ms","250");
    assert(GrabTask_Start(mode?"pick":"align")); run_to(GRAB_ALIGN,180,1);
    for(unsigned i=0;i<20;i++) step(true,1,-95,0);
    camera_valid=false; step(true,1,-95,0);
    assert(status().state==GRAB_VISION_GRACE && !closures);
    speed[mode]=status().forward_mm_s; requests[mode]=camera_requests;
    camera_valid=true;
    step(true,1,-94,0); step(true,1,-93,0); step(true,1,-92,0);
    assert(status().state==GRAB_ALIGN && !closures);
    camera_valid=false;
    for(unsigned i=0;i<5;i++) step(true,1,-95,0);
    assert(status().state==GRAB_VISION_PAUSE);
    camera_valid=true; run_to(GRAB_ALIGN,80,1);
    assert(camera_requests==requests[mode]+1 && RecoveryUsed==1 && !closures);
  }
  assert(fabsf(speed[0]-speed[1])<1e-5f);
}

static void take_never_lowers_to_a_clearance_target_or_reuses_car_exit_for_release(void)
{
  char *take[]={"grab","take","1","rough"};
  reset(); configure(false); setting("z_car_lift","1"); setting("z_proc_place","10");
  physical[1]=15; (void)Turntable_InventorySet(1,TURNTABLE_OCCUPIED,2);
  assert(GrabTask_Command(4,take)); run_to(GRAB_RETRACT,200,0);
  while(!Axes[1].commanded) step(true,0,0,0);
  assert(Axes[1].target==15); /* Higher initial Z is retained before tray indexing. */
  run_to(GRAB_RELEASE,600,0);
  assert(physical[1]==10 && physical[0]==0); /* Vertical release at the fixed external target. */
  run_to(GRAB_RETRACT,100,0);
  while(!Axes[1].commanded) step(true,0,0,0);
  assert(Axes[1].target==Config[C_Z_MAX] && Axes[1].target>10);
  run_to(GRAB_COMPLETE,300,0); assert(physical[1]==15);
}

static void unconfirmed_stop_exits_for_explicit_recovery_without_forgetting_parameters(void)
{
  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  tray_status.feedback_valid=false;
  GrabTask_Stop(); run_to(GRAB_ERROR,200,1);
  assert(!GrabTask_IsBusy() && !status().stop_confirmed && !strcmp(status().reason,"stop_unconfirmed"));
  assert(GrabTask_ConfigReady("align") && tray_inventory[0].state==TURNTABLE_EMPTY && !closures);
  tray_status.feedback_valid=true; /* Explicit bus recovery supplies fresh stopped evidence. */
  assert(GrabTask_Start("align")); run_to(GRAB_HOLD,300,1);
  assert(!closures && !base_moves);

  reset(); configure(true); assert(GrabTask_Start("align")); run_to(GRAB_ALIGN,180,1);
  bus.locked=true; GrabTask_Stop(); run_to(GRAB_ERROR,80,1);
  assert(!status().stop_confirmed && !GrabTask_IsBusy() && GrabTask_ConfigReady("align"));
}

static void explicit_rehome_remains_available_after_a_completed_piece(void)
{
  char *home[]={"grab","rehome"};
  reset(); configure(false); assert(GrabTask_Start("fixed")); run_to(GRAB_COMPLETE,600,0);
  assert(GrabTask_Command(2,home));
  assert(status().state==GRAB_HOMING);
}

static void all_existing_b2_colors_remain_recordable(void)
{
  char color[2]={'1',0}; char *fixed[]={"grab","start","fixed","1",color};
  for(unsigned i=1;i<=6;i++)
  {
    reset(); configure(false); color[0]=(char)('0'+i);
    assert(GrabTask_Command(5,fixed)); run_to(GRAB_COMPLETE,600,0);
    assert(tray_inventory[0].state==TURNTABLE_OCCUPIED && tray_inventory[0].color==i);
    assert(!strcmp(status().result,"stored"));
  }
}

static void stale_turntable_arrival_cannot_start_the_base(void)
{
  reset(); configure(false); tray_arrives=false;
  assert(GrabTask_Start("fixed")); run_to(GRAB_INDEX,500,0);
  while(!IndexSubmitted) step(true,0,0,0);
  tray_moving=false; tray_status.arrived=true; tray_status.feedback_valid=false; tray_status.reason="arrived";
  step(true,0,0,0);
  assert(status().state==GRAB_STOPPING && !strcmp(status().reason,"turntable_feedback_stale"));
  assert(!base_moves && tray_inventory[0].state==TURNTABLE_EMPTY);
}

int main(void)
{
  completed_single_piece_releases_task();
  two_pieces_return_to_actual_base_anchor();
  scene_parameters_only_block_the_requested_scene();
  unconfirmed_inventory_and_unset_angles_do_not_move();
  cancellation_only_marks_a_touched_slot_and_return_is_local();
  pick_and_align_share_gap_control();
  take_never_lowers_to_a_clearance_target_or_reuses_car_exit_for_release();
  unconfirmed_stop_exits_for_explicit_recovery_without_forgetting_parameters();
  explicit_rehome_remains_available_after_a_completed_piece();
  all_existing_b2_colors_remain_recordable();
  stale_turntable_arrival_cannot_start_the_base();
  puts("grab_transfer_test: PASS");
  return 0;
}
