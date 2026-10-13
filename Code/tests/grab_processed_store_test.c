#define main legacy_main
#include "grab_task_test.c"
#undef main
static void setup(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","100");setting("z_min","-150");setting("z_max","0");
 setting("z_car_lift","0");setting("z_observe","0");setting("z_rough_align","-100");setting("z_proc_place","-130");
 setting("dx_tol","1");setting("dy_tol","1");
 RearBase=37;RearReferenceValid=true;physical[2]=165;
}
static void finish_store(void)
{
 run_to(GRAB_COMPLETE,600,0);
 assert(closures==1 && !strcmp(status().result,"stored") && tray_inventory[0].color==5);
 assert(fabsf(physical[1])<.1f);
}
static void align_then_descend_without_vision(bool missing)
{
 setup();assert(GrabTask_ProcessedStoreStartEx(5,2,1,false));
 assert(grip_custom_calls==1 && fabsf(grip_custom_duty-10.6f)<.001f);
 run_to(GRAB_ACQUIRE,200,0);
 assert(fabsf(physical[1]+100)<.1f && !velocity_requests && !closures);
 run_to(GRAB_ALIGN,200,0);
 pixel_center_override=true;pixel_center_x=480;pixel_center_y=200;
 for(unsigned i=0;i<40;i++)step(true,0,2,0);
 assert(!closures && status().state==GRAB_ALIGN && fabsf(physical[1]+100)<.1f);
 run_to(GRAB_DESCEND,400,0);
 assert(!closures && requested_color==5 && status().dx==0 && status().dy==0);
 unsigned xy=velocity_requests,requests=camera_requests;
 assert(camera_stops>0);
 move_arrives=false;camera_valid=!missing;
 for(unsigned i=0;i<30;i++)step(!missing,0,120,-80);
 assert(status().state==GRAB_DESCEND && !closures && fabsf(physical[1]+100)<.1f);
 assert(destination[1]==-130 && velocity_requests==xy && camera_requests==requests);
 move_arrives=true;physical[1]=destination[1];
 for(unsigned i=0;i<80 && !closures;i++)step(!missing,0,120,-80);
 assert(closures==1 && status().state==GRAB_CLOSE && physical[1]==-130);
 assert(velocity_requests==xy && camera_requests==requests && grip_custom_calls==1);
 finish_store();
}
int main(void)
{
 align_then_descend_without_vision(false);align_then_descend_without_vision(true);
 setup();Config[C_Z_ROUGH_ALIGN]=NAN;assert(!GrabTask_ProcessedStoreStartEx(5,2,1,false));
 assert(!positions && !grip_custom_calls);
 setup();setting("z_proc_place","-90");assert(!GrabTask_ProcessedStoreStartEx(5,2,1,false));
 assert(!positions && !grip_custom_calls);

 /* Vision still matters before the descent commitment. */
 setup();assert(GrabTask_ProcessedStoreStartEx(5,2,1,false));run_to(GRAB_ALIGN,200,0);
 camera_valid=false;
 for(unsigned i=0;i<20;i++) {step(true,0,0,0);assert(!closures && physical[1]==-100);}
 assert(status().state==GRAB_VISION_PAUSE);
 camera_valid=true;run_to(GRAB_ALIGN,200,0);assert(RecoveryUsed==1 && !closures);
 run_to(GRAB_CLOSE,600,0);finish_store();

 setup();move_arrives=false;assert(GrabTask_ProcessedStoreStartEx(5,2,1,false));
 for(unsigned i=0;i<100;i++)step(true,0,0,0);
 assert(status().state==GRAB_PREPARE && !camera_requests && !velocity_requests && !closures);
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_ALIGN,200,0);
 for(unsigned i=0;i<200 && !closures;i++)step(true,0,1,-1);
 assert(closures==1 && physical[1]==-130);finish_store();

 const GrabTask_State_t phases[]={GRAB_PREPARE,GRAB_ACQUIRE,GRAB_ALIGN,GRAB_SETTLE,GRAB_DESCEND};
 for(unsigned i=0;i<sizeof phases/sizeof *phases;i++) {
  setup();assert(GrabTask_ProcessedStoreStartEx(5,2,1,false));run_to(phases[i],400,0);
  assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(!closures);
  unsigned requests=camera_requests;
  if(phases[i]==GRAB_DESCEND){camera_valid=false;camera_tx_result=HAL_ERROR;}
  assert(GrabTask_Continue());run_to(GRAB_CLOSE,600,0);
  assert(physical[1]==-130);
  if(phases[i]==GRAB_DESCEND)assert(camera_requests==requests);
  finish_store();
 }
 puts("processed retrieval: align -100, committed descent -130 without vision, pause and store PASS");
}
