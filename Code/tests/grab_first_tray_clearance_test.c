/* User confirmed first placed object: after Base reaches work, ID8+90.
 * Prepared source only; no C compiler/executable or hardware in this round. */
#define main legacy_grab_main
#include "grab_task_test.c"
#undef main
static void setup(GrabScene_t scene,unsigned slot)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_proc_place","-120");setting("z_stack_place","-80");
 RearBase=37;RearReferenceValid=true;physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
 for(unsigned i=0;i<3;i++)tray_inventory[i]=(Turntable_Inventory_t){TURNTABLE_OCCUPIED,(uint8_t)(i+1)};
 AlignedPlace.valid=true;AlignedPlace.picked=false;AlignedPlace.scene=scene;
 AlignedPlace.ring=scene==SCENE_TEMP2?3:1;AlignedPlace.x=40;AlignedPlace.z=0;
 AlignedPlace.release_z=scene==SCENE_TEMP2?-80:-120;
 assert(GrabTask_CarPickStart((uint8_t)slot));
}
static void first_pick_waits_for_table(GrabScene_t scene)
{
 setup(scene,1);run_to(GRAB_RETURN_TURN,600,0);
 assert(physical[0]==30 && physical[1]==0 && !tray_angle_moves);
 /* Base may still be turning: tray movement starts only once outside. */
 base_arrives=false;unsigned closed=closures;
 for(unsigned i=0;i<4;i++)step(true,0,0,0);
 assert(!tray_angle_moves);base_arrives=true;physical[2]=destination[2];tray_arrives=false;
 for(unsigned i=0;i<40 && !tray_angle_moves;i++)step(true,0,0,0);
 assert(tray_angle_moves==1 && tray_status.target_deg==90 && tray_status.slot==0);
 assert(fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f);
 assert(physical[1]==0 && status().state==GRAB_RETURN_TURN && !AlignedPlace.picked);
 assert(!strcmp(status().reason,"car_pick_tray_clearance"));
 assert(tray_inventory[0].state==TURNTABLE_EMPTY);
 assert(tray_inventory[1].state==TURNTABLE_OCCUPIED && tray_inventory[2].state==TURNTABLE_OCCUPIED);
 assert(!GrabTask_AlignedPlaceStart(SceneNames[scene],AlignedPlace.ring,1));
 for(unsigned i=0;i<10;i++)step(true,0,0,0);
 assert(tray_angle_moves==1 && !openings && closures==closed && physical[1]==0);
 tray_arrives=true;run_to(GRAB_COMPLETE,200,0);
 assert(AlignedPlace.picked && !strcmp(status().result,"picked"));
 assert(tray_status.angle_deg==90 && !CarPickTrayClearance.active);
 assert(GrabTask_AlignedPlaceStart(SceneNames[scene],AlignedPlace.ring,1));
 run_to(GRAB_RELEASE,600,0);assert(tray_angle_moves==1 && physical[1]==AlignedPlace.release_z);
 assert(tray_status.angle_deg==90 && tray_inventory[1].state==TURNTABLE_OCCUPIED);
}
static void pause_keeps_target(void)
{
 setup(SCENE_ROUGH,1);run_to(GRAB_RETURN_TURN,600,0);tray_arrives=false;
 for(unsigned i=0;i<40 && !tray_angle_moves;i++)step(true,0,0,0);
 assert(tray_angle_moves==1);tray_status.angle_deg=35;
 assert(GrabTask_Pause());tray_arrives=true;run_to(GRAB_IDLE,200,0);
 unsigned closed=closures;assert(GrabTask_Continue());
 run_to(GRAB_COMPLETE,400,0);
 assert(CarPickTrayClearance.target_angle==90 && tray_status.angle_deg==90);
 assert(tray_angle_moves==2 && closures==closed && !openings); /* Remaining55, not another90. */
}
static void stale_arrival_is_read_only(void)
{
 setup(SCENE_TEMP2,1);run_to(GRAB_RETURN_TURN,600,0);tray_arrives=false;
 for(unsigned i=0;i<40 && !tray_angle_moves;i++)step(true,0,0,0);
 assert(tray_angle_moves==1);tray_moving=false;tray_status.arrived=true;
 tray_status.angle_deg=90;tray_status.feedback_valid=false;
 step(true,0,0,0);assert(CarPickTrayClearance.verified && !CarPickTrayClearance.submitted);
 unsigned verified=tray_verifies;run_to(GRAB_COMPLETE,200,0);
 assert(tray_verifies==verified+1 && tray_angle_moves==1 && !openings);
}
static void cancel_drops_goal(void)
{
 setup(SCENE_TEMP1,1);run_to(GRAB_RETURN_TURN,600,0);tray_arrives=false;
 for(unsigned i=0;i<40 && !tray_angle_moves;i++)step(true,0,0,0);
 GrabTask_Cancel();tray_arrives=true;run_to(GRAB_IDLE,200,0);
 assert(!CarPickTrayClearance.active && !CarPickActive && !GrabTask_Continue());
 unsigned before=tray_angle_moves;for(unsigned i=0;i<15;i++)step(true,0,0,0);
 assert(tray_angle_moves==before && !openings);
}
int main(void)
{
 first_pick_waits_for_table(SCENE_ROUGH);first_pick_waits_for_table(SCENE_TEMP1);first_pick_waits_for_table(SCENE_TEMP2);
 pause_keeps_target();stale_arrival_is_read_only();cancel_drops_goal();
 for(unsigned slot=2;slot<=3;slot++){
  setup(SCENE_TEMP1,slot);run_to(GRAB_COMPLETE,600,0);
  assert(!tray_angle_moves && !CarPickTrayClearance.active);
 }
 setup(SCENE_ROUGH,1);AlignedPlace.valid=false; /* Manual work pickup keeps its own old scope. */
 GrabTask_Cancel();run_to(GRAB_IDLE,200,0);assert(GrabTask_CarPickStart(1));
 run_to(GRAB_COMPLETE,600,0);assert(!tray_angle_moves);
 puts("first tray clearance: three station scenes, Base/Z sequence, absolute+90, real arrival, stock and pause/cancel PASS");
 return 0;
}
