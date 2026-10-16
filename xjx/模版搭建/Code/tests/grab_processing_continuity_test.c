#define main legacy_main
#include "grab_task_test.c"
#undef main

static void setup(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_observe","0");setting("z_rough_align","-100");setting("z_proc_place","-130");
 physical[0]=25;physical[1]=0;physical[2]=37+GRAB_BASE_WORK_OFFSET_DEG;
 RearBase=37;RearReferenceValid=true;OutsidePose=true;
}
static void continuous_empty_ring_descent(void)
{
 setup();assert(GrabTask_AlignPlaceStartEx("rough",1,false));run_to(GRAB_ALIGN,200,0);
 move_arrives=false;
 for(unsigned i=0;i<30;i++)step(true,0,5,0);
 assert(RingLowerSent && destination[1]==-100 && physical[1]==0);
 assert(!closures && status().state==GRAB_ALIGN); /* ACK cannot confirm alignment. */
 unsigned moves=positions;
 for(unsigned i=0;i<20;i++)step(true,0,5,0);
 assert(positions==moves && destination[1]==-100); /* No repeated short Z targets. */
 physical[1]=-40;assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 assert(GrabTask_Continue());run_to(GRAB_ALIGN,200,0);
 for(unsigned i=0;i<20;i++)step(true,0,5,0);
 assert(destination[1]==-100 && physical[1]==-40 && !closures);
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_HOLD,300,0);
 assert(physical[1]==-100 && !closures);
}
static void car_index_during_lift(bool pause)
{
 setup();physical[1]=-100;assert(GrabTask_CarPickStart(2));
 move_arrives=false;tray_arrives=false;run_to(GRAB_RETRACT,200,0);
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(tray_indexes==1 && tray_moving && physical[1]==-100 && destination[1]==0);
 assert(!base_moves && !closures);
 if(pause) {
  assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
  unsigned count=tray_indexes;
  for(unsigned i=0;i<10;i++)step(true,0,0,0);
  assert(tray_indexes==count && !tray_moving);
  assert(GrabTask_Continue());
 }
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_INDEX,200,0);
 assert(!base_moves);unsigned count=tray_indexes;
 for(unsigned i=0;i<10;i++)step(true,0,0,0);
 assert(!base_moves && tray_indexes==count); /* Wait for the same in-flight index. */
 tray_arrives=true;run_to(GRAB_COMPLETE,400,0);
 assert(closures==1 && !openings && tray_indexes==count && physical[0]==30);
}
static void processed_pick_open_and_index(void)
{
 setup();tray_arrives=false;assert(GrabTask_ProcessedStoreStartEx(4,2,2,false));
 assert(grip_custom_calls==1 && fabsf(grip_custom_duty-12.25f)<.001f);
 run_to(GRAB_CLOSE,500,0);assert(!openings);
 move_arrives=false;tray_arrives=false;run_to(GRAB_LIFT,200,0);
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(tray_indexes==1 && tray_moving && physical[1]==-130 && destination[1]==0);
 assert(!base_moves && closures==1 && grip_custom_calls==1);
 /* Early completion may age during the arm transfer: verify without re-HOME. */
 tray_arrives=true;step(true,0,0,0);assert(!base_moves);
 tray_status.feedback_valid=false;
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_INDEX,200,0);
 assert(!base_moves);
 run_to(GRAB_COMPLETE,500,0);
 assert(tray_indexes==1 && tray_verifies==1 && closures==1 && openings==1);
 assert(tray_inventory[1].state==TURNTABLE_OCCUPIED && tray_inventory[1].color==4);
}
static void no_early_index_under_claw(void)
{
 setup();physical[2]=RearBase;physical[1]=-40;
 assert(GrabTask_CarPickStart(1));move_arrives=false;run_to(GRAB_RETRACT,200,0);
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(!tray_indexes && !base_moves);
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_COMPLETE,500,0);
 assert(tray_indexes==1 && closures==1);
}
static void processed_pause_keeps_claw_and_target(void)
{
 setup();tray_arrives=false;assert(GrabTask_ProcessedStoreStartEx(4,2,2,false));run_to(GRAB_CLOSE,500,0);
 move_arrives=false;tray_arrives=false;run_to(GRAB_LIFT,200,0);
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(tray_indexes==1);assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 assert(!tray_moving && closures==1 && !openings && grip_custom_calls==1);
 assert(GrabTask_Continue());
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(tray_indexes==2 && tray_status.slot==2 && !base_moves && grip_custom_calls==1);
 move_arrives=true;physical[1]=destination[1];tray_arrives=true;
 run_to(GRAB_COMPLETE,500,0);
 assert(closures==1 && openings==1 && tray_inventory[1].color==4);
}
static void third_processed_clearance(bool pause_out, bool pause_return)
{
 setup();tray_status.angle_deg=90;
 assert(GrabTask_ProcessedStoreStartEx(4,3,3,false));tray_arrives=false;
 for(unsigned n=0;n<100 && !tray_angle_moves;n++)step(true,0,0,0);
 assert(tray_angle_moves==1 && tray_status.target_deg==270 && !tray_indexes);
 assert(physical[1]==0 && !closures); /* Never lower at the intermediate slot3 direction. */
 if(pause_out){
   assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);tray_status.angle_deg=180;
   assert(GrabTask_Continue());
   for(unsigned n=0;n<100 && tray_angle_moves<2;n++)step(true,0,0,0);
   assert(tray_angle_moves==2 && tray_status.target_deg==270 && physical[1]==0);
 }
 tray_arrives=true;run_to(GRAB_CLOSE,500,0);assert(closures==1 && physical[1]==-130);
 unsigned out_moves=tray_angle_moves;
 move_arrives=false;tray_arrives=false;run_to(GRAB_LIFT,200,0);
 for(unsigned n=0;n<20;n++)step(true,0,0,0);
 assert(tray_angle_moves==out_moves && physical[1]==-130 && !base_moves);
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_TURN,250,0);
 for(unsigned n=0;n<30;n++)step(true,0,0,0);
 assert(tray_angle_moves==out_moves+1 && tray_status.target_deg==180 && tray_moving);
 assert(base_moves && !openings && Status.state==GRAB_TURN); /* Base may finish first, release cannot. */
 if(pause_return){
   assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);tray_status.angle_deg=225;
   assert(GrabTask_Continue());
   for(unsigned n=0;n<100 && tray_angle_moves<out_moves+2;n++)step(true,0,0,0);
   assert(tray_angle_moves==out_moves+2 && tray_status.target_deg==180 && !openings);
 }
 tray_arrives=true;run_to(GRAB_COMPLETE,600,0);
 assert(closures==1 && openings==1 && tray_inventory[2].color==4 && !tray_indexes);
}
static void third_clearance_pause_low(bool raised)
{
 setup();tray_status.angle_deg=90;
 if(raised){setting("z_car_lift","-10");setting("z_place","-40");} /* Actual Z=0 is already above the saved lift target. */
 assert(GrabTask_ProcessedStoreStartEx(4,3,3,raised));run_to(GRAB_DESCEND,500,0);
 move_arrives=false;
 for(unsigned n=0;n<8;n++)step(true,0,0,0);
 physical[1]=-115;assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 unsigned moves=tray_angle_moves, reads=tray_verifies, requests=camera_requests;
 assert(GrabTask_Continue());
 for(unsigned n=0;n<25;n++)step(false,0,0,0);
 assert(tray_verifies==reads+1 && tray_angle_moves==moves);
 assert(!closures && !openings && camera_requests==requests && destination[1]==-130);
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_COMPLETE,600,0);
 assert(closures==1 && openings==1 && tray_angle_moves==moves+1);
}
int main(void)
{
 third_clearance_pause_low(false);third_clearance_pause_low(true);
 third_processed_clearance(false,false);third_processed_clearance(true,false);third_processed_clearance(false,true);
 continuous_empty_ring_descent();car_index_during_lift(false);car_index_during_lift(true);
 processed_pick_open_and_index();no_early_index_under_claw();processed_pause_keeps_claw_and_target();
 puts("processing continuity: continuous Z, parallel tray index and full-open station pickup PASS");
}
