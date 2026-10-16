/* Rejected retries must retain the completed producer and its stock/context. */
#define main legacy_raw_main
#include "grab_task_test.c"
#undef main
static void setup(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","100");setting("z_min","-150");setting("z_max","0");
 setting("z_observe","0");setting("z_grab","-50");setting("z_lift","0");setting("z_car_lift","0");setting("z_place","-40");
 RearBase=37;RearReferenceValid=true;physical[0]=30;physical[2]=165;
}
static void stored_raw(void)
{
 setup();assert(GrabTask_TrackedStoreStart(3,1,GRAB_TRACK_FEEDBACK));
 run_to(GRAB_COMPLETE,1000,0);assert(!strcmp(status().result,"stored"));
 chassis_bus_busy=true;chassis_velocity_handoff_ready=false;
}
int main(void)
{
 stored_raw();
 for(unsigned i=0;i<3;i++){
  assert(!GrabTask_Start("align",CAMERA_COLOR_BLUE));
  assert(status().state==GRAB_COMPLETE && !strcmp(status().result,"stored"));
  assert(Operation==OP_STORE && Scene==SCENE_RAW);
  assert(tray_inventory[0].state==TURNTABLE_OCCUPIED);
 }
 chassis_velocity_handoff_ready=true;
 assert(GrabTask_Start("align",CAMERA_COLOR_BLUE));
 assert(status().state==GRAB_PREPARE && !strcmp(status().result,"pending"));
 stored_raw();chassis_velocity_handoff_ready=true;
 pending_valid=true;assert(!GrabTask_Start("align",CAMERA_COLOR_BLUE));pending_valid=false;
 assert(status().state==GRAB_COMPLETE && !strcmp(status().result,"stored"));
 assert(GrabTask_Start("align",CAMERA_COLOR_BLUE));
 setup();chassis_bus_busy=true;chassis_velocity_handoff_ready=false;
 assert(!GrabTask_Start("align",CAMERA_COLOR_BLUE)); /* Unproven zero retains the veto. */
 chassis_velocity_handoff_ready=true;
 assert(GrabTask_Start("align",CAMERA_COLOR_BLUE)); /* PREPARE still proves real stopping. */
 puts("raw handoff: repeated busy preserves result/inventory, committed zero admits next owner PASS");
 return 0;
}
