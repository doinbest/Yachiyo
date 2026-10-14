#define main legacy_main
#include "grab_task_test.c"
#undef main
static void setup(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","130");
 setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
 setting("z_observe","0");setting("z_rough_align","-100");setting("z_temp_align","-100");
 setting("z_proc_place","-130");setting("z_stack_place","-100");
 RearBase=37;RearReferenceValid=true;physical[2]=165;
}
static void processed_pick(bool raised)
{
 setup();assert(GrabTask_ProcessedStoreStartEx(5,2,1,raised));
 run_to(GRAB_ACQUIRE,200,0);assert(physical[1]==(raised?0:-100));
 run_to(GRAB_ALIGN,200,0);
 for(unsigned i=0;i<20;i++){step(true,0,10,0);assert(physical[1]==(raised?0:-100) && !closures);}
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(GrabTask_Continue());
 run_to(GRAB_DESCEND,400,0);camera_valid=false;camera_tx_result=HAL_ERROR;
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);unsigned requests=camera_requests;
 assert(GrabTask_Continue());run_to(GRAB_CLOSE,400,0);
 assert(physical[1]==-130 && closures==1 && camera_requests==requests);
 run_to(GRAB_COMPLETE,600,0);assert(tray_inventory[0].color==5);
}
static void empty_ring(bool raised)
{
 setup();physical[0]=42;assert(GrabTask_AlignPlaceStartEx("rough",2,raised));
 run_to(GRAB_HOLD,500,0);assert(physical[1]==(raised?0:-100) && !closures);
 for(unsigned i=0;i<80 && !AlignedPlace.valid;i++){step(true,0,0,0);(void)GrabTask_AlignPlaceCapture();}
 assert(AlignedPlace.valid && AlignedPlace.z==(raised?0:-100));
 bool accepted=false;
 for(unsigned i=0;i<20 && !accepted;i++){accepted=GrabTask_CarPickStart(2);if(!accepted)step(true,0,0,0);}
 assert(accepted);run_to(GRAB_COMPLETE,600,0);
 unsigned opened=openings;assert(GrabTask_AlignedPlaceStart("rough",2,5));camera_valid=false;
 run_to(GRAB_RELEASE,600,0);assert(physical[1]==-130 && openings==opened+1);
 run_to(GRAB_COMPLETE,300,0);
}
static void held_raised(const char *scene,bool stack)
{
 setup();TrackHeld=true;assert(GrabTask_HeldPlaceStartEx(scene,stack?5:2,5,true));
 run_to(GRAB_ALIGN,200,0);
 for(unsigned i=0;i<25;i++){step(true,0,10,0);assert(physical[1]==0 && !openings);}
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(GrabTask_Continue());
 for(unsigned i=0;i<700 && status().state!=GRAB_DESCEND;i++){
  step(true,0,0,0);
  assert(physical[1]==0 && !openings);
 }
 assert(status().state==GRAB_DESCEND);
 move_arrives=false;camera_valid=false;camera_tx_result=HAL_ERROR;
 for(unsigned i=0;i<20;i++)step(false,0,100,-80);
 assert(physical[1]==0 && destination[1]==(stack?-100:-130) && !openings);
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);unsigned requests=camera_requests;
 assert(GrabTask_Continue());
 for(unsigned i=0;i<20;i++)step(false,0,100,-80);
 assert(!openings && camera_requests==requests);
 move_arrives=true;physical[1]=destination[1];run_to(GRAB_RELEASE,400,0);
 assert(openings==1 && physical[1]==(stack?-100:-130));
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(GrabTask_Continue());
 run_to(GRAB_COMPLETE,300,0);assert(openings==1);
 /* A later standalone station task defaults to the same high observation. */
 camera_valid=true;camera_tx_result=HAL_OK;TrackHeld=true;
 assert(GrabTask_HeldPlaceStart("temp1",2,5));run_to(GRAB_DESCEND,500,0);
 assert(physical[1]==0);
}
int main(void)
{
 setup();Config[C_Z_ROUGH_ALIGN]=NAN;
 assert(GrabTask_ProcessedStoreStart(5,2,1));run_to(GRAB_ACQUIRE,200,0);assert(physical[1]==0);
 setup();Config[C_Z_ROUGH_ALIGN]=NAN;
 assert(GrabTask_AlignPlaceStart("rough",2));run_to(GRAB_HOLD,500,0);assert(physical[1]==0);
 processed_pick(true);processed_pick(false);empty_ring(true);empty_ring(false);
 held_raised("rough",false);held_raised("temp1",false);held_raised("temp2",true);
 setup();Config[C_Z_ROUGH_ALIGN]=NAN;
 assert(GrabTask_ProcessedStoreStartEx(5,2,1,true));run_to(GRAB_ACQUIRE,200,0);assert(physical[1]==0);
 setup();Config[C_Z_TEMP_ALIGN]=NAN;TrackHeld=true;
 assert(GrabTask_HeldPlaceStartEx("temp1",2,5,true));
 setup();setting("z_max","-1");assert(!GrabTask_AlignPlaceStartEx("rough",2,true));assert(!positions);
 puts("station alignment: raised/lowered pickup and rings, high stacking, final descent without vision and pause PASS");
}
