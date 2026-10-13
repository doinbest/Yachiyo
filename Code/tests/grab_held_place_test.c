#define main legacy_main
#include "grab_task_test.c"
#undef main
static void setup_place(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","100");setting("z_min","-150");setting("z_max","0");
 setting("z_car_lift","0");setting("z_observe","0");setting("z_rough_align","-10");setting("z_temp_align","-10");setting("z_proc_place","-40");setting("z_stack_place","-20");
 RearBase=37;RearReferenceValid=true;physical[2]=165;TrackHeld=true;
}
static void align_then_pick_and_place_continuously(const char *scene,bool raised,const char *release_height)
{
 setup_place();TrackHeld=false;physical[0]=42;
 setting("z_rough_align","-100");setting("z_temp_align","-80");setting("z_proc_place",release_height);
 float release_z=Config[C_Z_PROC_PLACE];
 assert(GrabTask_AlignPlaceStartEx(scene,2,raised));
 assert(!GrabTask_AlignPlaceCapture() && !AlignedPlace.valid && !closures);
 run_to(GRAB_HOLD,500,0);
 /* Alignment alone cannot skip the fresh stopped capture contract. */
 unsigned closed=closures;wheel_moving=true;step(true,0,0,0);
 assert(!GrabTask_AlignPlaceCapture() && !closures);
 wheel_moving=false;
 for(unsigned i=0;i<80 && !AlignedPlace.valid;i++){step(true,0,0,0);(void)GrabTask_AlignPlaceCapture();}
 assert(AlignedPlace.valid && !AlignedPlace.picked && status().state==GRAB_COMPLETE);
 float aligned_x=AlignedPlace.x,aligned_z=AlignedPlace.z;
 assert(fabsf(aligned_x-42)<.1f && fabsf(aligned_z-(raised?0:!strcmp(scene,"temp1")?-80:-100))<.1f);
 assert(!GrabTask_AlignedPlaceStart(scene,2,5));
 if(!strcmp(scene,"temp1"))setting("z_proc_place","-140"); /* Preserve this item's captured release target. */
 bool accepted=false;
 for(unsigned i=0;i<20 && !accepted;i++)
 {
  accepted=GrabTask_CarPickStart(2);
  if(!accepted){assert(!strcmp(status().reason,"busy"));step(true,0,0,0);}
 }
 assert(accepted);
 if(!strcmp(scene,"temp1")){
  run_to(GRAB_CLOSE,400,0);assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
  assert(AlignedPlace.valid && AlignedPlace.x==aligned_x);assert(GrabTask_Continue());
 }
 run_to(GRAB_COMPLETE,600,0);
 assert(closures==closed+1 && TrackHeld && physical[0]==30 && AlignedPlace.picked);
 unsigned opened=openings;
 unsigned saved_requests=camera_requests;
 assert(!GrabTask_AlignedPlaceStart(!strcmp(scene,"temp1")?"rough":"temp1",2,5));
 assert(GrabTask_AlignedPlaceStart(scene,2,5));
 camera_valid=false;run_to(GRAB_EXTERNAL_EXTEND,300,0);move_arrives=false;
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(status().state==GRAB_EXTERNAL_EXTEND && physical[0]==30);
 assert(physical[1]==0 && openings==opened); /* Restore X before lowering the held item. */
 move_arrives=true;physical[0]=destination[0];run_to(GRAB_DESCEND,300,0);
 assert(fabsf(physical[0]-aligned_x)<.1f && openings==opened);
 move_arrives=false;
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(destination[1]==release_z && physical[1]==0 && openings==opened);
 unsigned requests=camera_requests;
 /* Speed updates retain the final destination across the saved alignment height. */
 physical[1]=aligned_z;
 for(unsigned i=0;i<20;i++) {
  step(true,0,100,-80);
  assert(status().state==GRAB_DESCEND && destination[1]==release_z);
 }
 assert(openings==opened && camera_requests==requests && requests==saved_requests);
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(AlignedPlace.valid);
 camera_tx_result=HAL_ERROR;assert(GrabTask_Continue());
 for(unsigned i=0;i<20;i++)step(true,0,100,-80);
 assert(destination[1]==release_z && physical[1]==aligned_z && openings==opened);
 assert(camera_requests==requests); /* A saved placement resumes without vision. */
 move_arrives=true;physical[1]=release_z;run_to(GRAB_RELEASE,300,0);
 assert(fabsf(physical[1]-release_z)<.1f && openings==opened+1);
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(GrabTask_Continue());
 run_to(GRAB_COMPLETE,300,0);assert(openings==opened+1 && !TrackHeld && !AlignedPlace.valid);
 setup_place();TrackHeld=false;assert(GrabTask_AlignPlaceStartEx(scene,1,raised));run_to(GRAB_HOLD,500,0);
 /* A new out-of-window frame must return to alignment instead of capture. */
 step(true,0,20,0);assert(!GrabTask_AlignPlaceCapture() && status().state==GRAB_ALIGN && !AlignedPlace.valid);
 GrabTask_Cancel();assert(!AlignedPlace.valid);
}
int main(void)
{
 setup_place();assert(GrabTask_HeldPlaceStartEx("rough",2,5,false));run_to(GRAB_DESCEND,400,0);
 assert(!openings && !closures && requested_color==11);
 camera_valid=false;for(unsigned i=0;i<20;i++)step(true,0,0,0);assert(!openings);
 camera_valid=true;run_to(GRAB_RELEASE,200,0);assert(openings==1);
 run_to(GRAB_COMPLETE,200,0);assert(!strcmp(status().result,"released") && openings==1 && !TrackHeld);
 setup_place();assert(GrabTask_HeldPlaceStartEx("rough",1,6,false));run_to(GRAB_DESCEND,400,0);
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(!openings);camera_tx_result=HAL_BUSY;assert(!GrabTask_Continue());
 assert(status().stop_confirmed);camera_tx_result=HAL_OK;assert(GrabTask_Continue());
 run_to(GRAB_RELEASE,300,0);assert(openings==1);assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 assert(GrabTask_Continue());run_to(GRAB_COMPLETE,300,0);assert(openings==1);
 setup_place();assert(GrabTask_HeldPlaceStartEx("rough",1,6,false));run_to(GRAB_DESCEND,400,0);
 for(unsigned i=0;i<100;i++)step(true,0,20,0);
 assert(!openings && status().state!=GRAB_DESCEND && physical[1]>=-10.1f);
 run_to(GRAB_COMPLETE,500,0);assert(openings==1);
 /* First-layer release follows the shared target, including user edits. */
 for(unsigned i=0;i<2;i++){
  setup_place();setting("z_proc_place",i?"-60":"-40");
  assert(GrabTask_HeldPlaceStartEx("temp1",2,5,false));run_to(GRAB_RELEASE,600,0);
  assert(fabsf(physical[1]-(i?-60:-40))<.1f && openings==1);
  run_to(GRAB_COMPLETE,300,0);assert(!TrackHeld);
 }
 setup_place();setting("z_temp_align","-100");setting("z_stack_place","-100");
 assert(GrabTask_HeldPlaceStartEx("temp2",5,5,false));
 for(unsigned i=0;i<900 && status().state!=GRAB_RELEASE;i++)step(true,0,0,0);
 assert(status().state==GRAB_RELEASE);
 assert(fabsf(physical[1]+100)<.1f && openings==1 && requested_color==5);
 run_to(GRAB_COMPLETE,300,0);assert(!TrackHeld);
 align_then_pick_and_place_continuously("rough",false,"-130");
 align_then_pick_and_place_continuously("temp1",false,"-130");
 align_then_pick_and_place_continuously("temp1",true,"-130");
 align_then_pick_and_place_continuously("rough",false,"-120");
 puts("held placement: align before pickup, restore X then continuous Z release, pause without duplicate release PASS");
}
