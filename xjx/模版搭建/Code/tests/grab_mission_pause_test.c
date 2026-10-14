#define main legacy_main
#include "grab_task_test.c"
#undef main
static void setup(void)
{
 reset();configure(true);setting("x_min","0");setting("x_max","100");setting("z_min","-150");setting("z_max","0");
 setting("z_observe","0");setting("z_grab","-50");setting("z_lift","0");setting("z_car_lift","0");setting("z_place","-40");
 setting("z_rough_align","-10");setting("z_proc_place","-40");
 RearBase=37;RearReferenceValid=true;physical[0]=30;physical[2]=165;
}
static void pause_resume(void)
{
 unsigned closed=closures,opened=openings;GrabTask_Pause();GrabTask_Pause();run_to(GRAB_IDLE,200,0);
 assert(closures==closed && openings==opened);static unsigned pauses;clock_ms+=(pauses++%2)?60000:10000;
 bool resumed=false;for(unsigned n=0;n<200 && !resumed;n++){resumed=GrabTask_Continue();if(!resumed)step(true,0,0,0);}
 assert(resumed);
 run_to(GRAB_COMPLETE,1000,0);assert(closures<=closed+1);
}
int main(void)
{
 setup();assert(GrabTask_QrPoseStart());
 while(PoseStep!=5)step(true,0,0,0);
 pause_resume();
 assert(fabsf(physical[2]-72)<.15f && !strcmp(status().result,"qr_pose_ready") && !openings);
 const GrabTask_State_t store[]={GRAB_TRACK_OBSERVE,GRAB_DESCEND,GRAB_SETTLE,GRAB_CLOSE,GRAB_LIFT,GRAB_INDEX,GRAB_TURN,GRAB_PLACE,GRAB_RELEASE,GRAB_RETRACT,GRAB_RETURN_TURN};
 for(unsigned i=0;i<sizeof store/sizeof *store;i++){
  setup();assert(GrabTask_TrackedStoreStart(3,2,GRAB_TRACK_FEEDBACK));run_to(store[i],1000,0);
  fprintf(stderr,"pause tracking phase=%u\n",store[i]);pause_resume();
  assert(closures==1 && !strcmp(status().result,"stored") && tray_inventory[1].state==TURNTABLE_OCCUPIED);
 }
 const GrabTask_State_t pick[]={GRAB_PREPARE,GRAB_INDEX,GRAB_TURN,GRAB_PLACE,GRAB_CLOSE,GRAB_LIFT,GRAB_RETURN_TURN};
 for(unsigned i=0;i<sizeof pick/sizeof *pick;i++){
  setup();assert(GrabTask_CarPickStart(1));run_to(pick[i],1000,0);pause_resume();
  assert(closures==1 && !openings && !strcmp(status().result,"picked") && tray_inventory[0].state==TURNTABLE_EMPTY);
 }
 const GrabTask_State_t held[]={GRAB_PREPARE,GRAB_ACQUIRE,GRAB_ALIGN,GRAB_SETTLE,GRAB_DESCEND,GRAB_RELEASE,GRAB_RETRACT};
 for(unsigned i=0;i<sizeof held/sizeof *held;i++){
  setup();TrackHeld=true;assert(GrabTask_HeldPlaceStart("rough",2,5));run_to(held[i],1000,0);pause_resume();
  assert(!closures && openings==1 && !strcmp(status().result,"released"));
 }
 setup();assert(GrabTask_QrScanStart(0));run_to(GRAB_QR_SCAN,10,0);
 for(unsigned i=0;i<30;i++)step(true,0,0,0);
 assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);assert(GrabTask_Continue());
 qr_snapshot.Valid=1;qr_snapshot.Source=QR_SOURCE_UART;qr_snapshot.Sequence=1;run_to(GRAB_IDLE,200,0);
 assert(!strcmp(status().result,"qr_scanned"));
 setup();assert(GrabTask_CarPickStart(1));run_to(GRAB_PLACE,1000,0);
 unsigned closed=closures,opened=openings;assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
 GrabTask_Cancel();assert(!GrabTask_Continue());assert(closures==closed && openings==opened);
 puts("mission child pause: tracking, storage, pickup, held release and QR resume PASS");
}
