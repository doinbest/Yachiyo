#define main legacy_grab_main
#include "grab_task_test.c"
#undef main
static void prepare_pick(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_proc_place","-120");setting("z_stack_place","-80");
 setting("z_observe","0");
 RearBase=37;RearReferenceValid=true;physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
 AlignedPlace.valid=true;AlignedPlace.picked=false;AlignedPlace.scene=SCENE_ROUGH;AlignedPlace.ring=1;
 AlignedPlace.x=40;AlignedPlace.z=0;AlignedPlace.release_z=-120;
 for(unsigned i=0;i<3;i++)tray_inventory[i]=(Turntable_Inventory_t){TURNTABLE_OCCUPIED,(uint8_t)(i+1)};
 assert(GrabTask_CarPickStart(1));run_to(GRAB_LIFT,600,0);
}
static void first_clearance_overlaps(void)
{
 prepare_pick();unsigned before=base_moves;
 z_arrives=base_arrives=tray_arrives=false;
 for(unsigned i=0;i<45;i++)step(true,0,0,0);
 assert(physical[0]==30 && physical[1]<0);
 assert(base_moves>before && tray_angle_moves==1);
 assert(!AlignedPlace.picked && !openings && tray_inventory[0].state!=TURNTABLE_EMPTY);
 physical[1]=destination[1];z_arrives=true;
 physical[2]=destination[2];base_arrives=true;
 run_to(GRAB_COMPLETE,250,0);
 assert(AlignedPlace.picked && tray_moving && CarPickTrayClearance.active);
 assert(GrabTask_AlignedPlaceStart("rough",1,1));
 for(unsigned i=0;i<100;i++)step(false,0,0,0);
 assert(physical[0]==40 && physical[1]==0 && !openings);
 tray_arrives=true;run_to(GRAB_RELEASE,250,0);
 assert(physical[1]==-120 && openings==1 && tray_angle_moves==1);
}
static void early_alignment_index(const char *scene,unsigned slot)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_proc_place","-120");setting("z_stack_place","-80");
 setting("z_observe","0");
 RearBase=37;RearReferenceValid=true;physical[0]=42;physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
 uint8_t target=!strcmp(scene,"temp2")?5:2;
 if(!GrabTask_AlignPlaceStartForSlot(scene,target,true,(uint8_t)slot)){fprintf(stderr,"start %s slot%u reason=%s missing=%s\n",scene,slot,Status.reason,Status.missing);assert(0);}
 tray_arrives=false;run_to(GRAB_ALIGN,600,20);
 assert(tray_indexes==1 && tray_moving && !AlignedPlace.valid && !VisualStable);
 run_to(GRAB_HOLD,600,0);assert(tray_moving && !tray_stops);
 for(unsigned i=0;i<50 && !AlignedPlace.valid;i++){
   step(true,0,0,0);(void)GrabTask_AlignPlaceCapture();
 }
 assert(AlignedPlace.valid);
 if(pending_valid)step(false,0,0,0);
 assert(!GrabTask_CarPickStart(slot==1?2:1)); /* An unrelated target cannot steal it. */
 assert(GrabTask_CarPickStart((uint8_t)slot));
 run_to(GRAB_INDEX,600,0);assert(!closures && tray_indexes==1);
 tray_arrives=true;run_to(GRAB_COMPLETE,600,0);
 assert(AlignedPlace.picked && tray_indexes==1 && closures==1);
}
static void pause_parallel_lift(void)
{
 prepare_pick();z_arrives=base_arrives=tray_arrives=false;
 for(unsigned i=0;i<150 && !tray_angle_moves;i++)step(true,0,0,0);
 assert(Status.state==GRAB_RETURN_TURN && tray_angle_moves==1 && physical[1]<0);
 tray_status.angle_deg=35;
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 z_arrives=base_arrives=tray_arrives=true;
 unsigned closed=closures;assert(GrabTask_Continue());run_to(GRAB_COMPLETE,600,0);
 assert(physical[1]==0 && physical[0]==30 && closures==closed && !openings);
 assert(CarPickTrayClearance.target_angle==90 && tray_status.angle_deg==90);
}
static void prefetched_third(bool pause,bool cancel)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_observe","0");setting("z_proc_place","-120");
 RearBase=37;RearReferenceValid=true;physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
 assert(GrabTask_PoseStart(true));run_to(GRAB_COMPLETE,300,0);
 Status.result="stored";Status.slot=2;tray_status.angle_deg=90;tray_arrives=false;
 assert(GrabTask_PrepareProcessedTray3());
 assert(ProcClearance.prefetched && tray_angle_moves==1 && tray_status.target_deg==270);
 assert(Status.slot==2 && !strcmp(Status.result,"stored"));
 chassis_pose_motion=true;for(unsigned i=0;i<10;i++)step(false,0,0,0);
 assert(!closures && physical[1]==0 && tray_angle_moves==1);chassis_pose_motion=false;
 if(cancel){
   GrabTask_Cancel();assert(!tray_moving && !GrabTask_OwnsTrayGoal());
   run_to(GRAB_IDLE,200,0);assert(!GrabTask_Continue());return;
 }
 if(pause){
   assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
   unsigned moves=tray_angle_moves;for(unsigned i=0;i<20;i++)step(false,0,0,0);
   assert(tray_angle_moves==moves && !tray_moving);
   tray_status.angle_deg=180;assert(GrabTask_Continue());
   for(unsigned i=0;i<100 && !tray_moving;i++)step(false,0,0,0);
   assert(tray_angle_moves==moves+1 && tray_status.target_deg==270);
 }
 unsigned before=tray_angle_moves,opened=openings;
 assert(GrabTask_ProcessedStoreStartEx(4,3,3,true));
 for(unsigned i=0;i<70;i++)step(true,0,0,0);
 assert(physical[1]==0 && !closures && tray_angle_moves==before);
 tray_arrives=true;run_to(GRAB_COMPLETE,600,0);
 assert(closures==1 && openings==opened+1 && tray_inventory[2].color==4);
 assert(tray_angle_moves==before+1 && tray_status.target_deg==180);
}
static void servo_wait_boundaries(void)
{
 reset();assert(Config[C_CLOSE_MS]==100 && Config[C_RELEASE_MS]==100 && Config[C_CAR_PICK_WAIT_MS]==100);
 configure(true);setting("close_ms","100");Operation=OP_ALIGN;ToCar=false;
 enter(GRAB_CLOSE);clock_ms+=99;transfer_process(clock_ms);assert(Status.state==GRAB_CLOSE);
 clock_ms++;transfer_process(clock_ms);assert(Status.state==GRAB_LIFT);
 enter(GRAB_RELEASE);clock_ms+=99;transfer_process(clock_ms);assert(Status.state==GRAB_RELEASE);
 clock_ms++;transfer_process(clock_ms);assert(Status.state==GRAB_RETRACT);
 enter(GRAB_COMPLETE);setting("close_ms","300");enter(GRAB_CLOSE);clock_ms+=100;transfer_process(clock_ms);
 assert(Status.state==GRAB_CLOSE); /* Explicit custom settings remain respected. */
 enter(GRAB_TURN);Operation=OP_CAR_PICK;CarPickClosed=false;CarPickWaiting=true;CarPickWaitTick=clock_ms;
 RearBase=0;Axes[2]=(GrabAxis_t){.commanded=true,.target_reused=true,.position_valid=true,.state_valid=true,
   .position_tick=clock_ms,.state_tick=clock_ms,.quiet=true,.quiet_since=clock_ms-300,.flags=3};
 clock_ms+=99;transfer_process(clock_ms);assert(Status.state==GRAB_TURN);
 clock_ms++;transfer_process(clock_ms);assert(Status.state==GRAB_PLACE);
}
static void cancel_pick_handoff(void)
{
 prepare_pick();tray_arrives=false;run_to(GRAB_COMPLETE,600,0);
 assert(!GrabTask_IsBusy() && GrabTask_OwnsTrayGoal() && tray_moving);
 GrabTask_Cancel();assert(!tray_moving && !GrabTask_OwnsTrayGoal());
 run_to(GRAB_IDLE,200,0);assert(!GrabTask_Continue() && !AlignedPlace.valid);
}
static void paused_completed_preindex_uses_reads(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_observe","0");setting("z_proc_place","-120");
 RearBase=37;RearReferenceValid=true;physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
 assert(GrabTask_AlignPlaceStartForSlot("rough",1,true,1));run_to(GRAB_HOLD,600,0);
 assert(StationPreindex.verified && tray_indexes==1);
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 assert(GrabTask_Continue());run_to(GRAB_HOLD,600,0);
 assert(tray_indexes==1 && tray_verifies==1);
}
int main(void)
{
 first_clearance_overlaps();
 const char *scenes[]={"rough","temp1","temp2"};
 for(unsigned i=0;i<3;i++)for(unsigned slot=1;slot<=3;slot++)early_alignment_index(scenes[i],slot);
 pause_parallel_lift();
 prefetched_third(false,false);prefetched_third(true,false);prefetched_third(false,true);
 servo_wait_boundaries();
 cancel_pick_handoff();
 paused_completed_preindex_uses_reads();
 puts("parallel transfer: X clearance starts Base/tray before Z, saved X overlaps tray and descent joins PASS");
 return 0;
}
