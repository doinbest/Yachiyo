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
static void queued_next_pose(void)
{
 setup();assert(GrabTask_TrackedStoreStart(3,1,GRAB_TRACK_FEEDBACK));
 run_to(GRAB_RETRACT,1000,0);
 GrabTask_Stop();run_to(GRAB_IDLE,200,0);
 assert(TrackStoreReleased && TrackStoreRecorded && tray_inventory[0].state==TURNTABLE_OCCUPIED);
 assert(GrabTask_TrackedStoreStart(5,2,GRAB_TRACK_FEEDBACK));
 assert(status().state==GRAB_ROUTE_POSE && NextTrack.pending);
}
static void queued_next_pose_survives_pause(void)
{
 for(unsigned transport=0;transport<2;transport++){
  queued_next_pose();unsigned closed=closures,indexed=tray_indexes;
  if(transport)assert(GrabTask_StoreFinishTravel());
  if(transport){
   /* Use the same recoverable UART-noise seam as grab_bus_recovery_test. */
   home_bus_diag=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
   fault("motor_feedback");
   home_bus_diag.locked=false;
  }else assert(GrabTask_Pause());
  assert(ResumePending && NextTrack.pending && status().state==GRAB_STOPPING && !FinishError);
  assert(NextTrack.color==5 && NextTrack.slot==2 && NextTrack.mode==GRAB_TRACK_FEEDBACK);
  assert(NextTrack.automatic && NextTrack.pick && NextTrack.observe && NextTrack.budget==50);
  assert(NextTrack.finish_travel==(transport!=0));
  assert(GrabTask_Pause());GrabTask_Stop(); /* An existing mission pause remains resumable. */
  run_to(GRAB_IDLE,200,0);clock_ms+=10000;
  for(unsigned i=0;i<10;i++)step(true,0,0,0);
  assert(NextTrack.pending && closures==closed && tray_indexes==indexed && !GrabTask_IsBusy());
  bool resumed=false;
  for(unsigned i=0;i<200 && !resumed;i++){resumed=GrabTask_Continue();if(!resumed)step(true,0,0,0);}
  assert(resumed && NextTrack.pending && status().state==GRAB_ROUTE_POSE);
  /* Completing only the pose would report work_ready and fail the parent's
   * M_TRANSFER contract. The retained next pickup must instead finish stored. */
  run_to(GRAB_COMPLETE,1500,0);
  assert(!NextTrack.pending && !strcmp(status().result,"stored") && status().slot==2);
  assert(closures==closed+1 && tray_indexes==indexed+1);
  assert(tray_inventory[0].state==TURNTABLE_OCCUPIED && tray_inventory[0].color==3);
  assert(tray_inventory[1].state==TURNTABLE_OCCUPIED && tray_inventory[1].color==5);
  if(transport)assert(RearPose && fabsf(physical[2]-RearBase)<.15f);
 }
}
static void queued_next_pose_cancels_without_resume(void)
{
 for(unsigned cancel=0;cancel<3;cancel++){
  queued_next_pose();unsigned closed=closures,indexed=tray_indexes;
  if(cancel){
   assert(GrabTask_Pause() && NextTrack.pending);
   if(cancel==2)run_to(GRAB_IDLE,200,0);
   GrabTask_Cancel();
  }else GrabTask_Stop();
  assert(!ResumePending && !NextTrack.pending);
  run_to(GRAB_IDLE,200,0);assert(!GrabTask_Continue());
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(closures==closed && tray_indexes==indexed && !GrabTask_IsBusy());
  assert(tray_inventory[0].state==TURNTABLE_OCCUPIED && tray_inventory[0].color==3);
  assert(tray_inventory[1].state==TURNTABLE_EMPTY);
 }
 queued_next_pose();
 home_bus_diag=(MotorBus_Diagnostic_t){.locked=true,.reason="uart_error",.uart_error=HAL_UART_ERROR_NE};
 fault("axis_disabled_or_stall");
 assert(!ResumePending && !NextTrack.pending && FinishError);
 home_bus_diag.locked=false;run_to(GRAB_ERROR,200,0);assert(!GrabTask_Continue());
}
int main(void)
{
 queued_next_pose_survives_pause();queued_next_pose_cancels_without_resume();
 setup();assert(GrabTask_QrPoseStart());
 while(PoseStep!=5)step(true,0,0,0);
 pause_resume();
 assert(fabsf(physical[2]-72)<.15f && !strcmp(status().result,"qr_pose_ready") && !openings);
 const GrabTask_State_t store[]={GRAB_TRACK_OBSERVE,GRAB_DESCEND,GRAB_CLOSE,GRAB_LIFT,GRAB_INDEX,GRAB_TURN,GRAB_PLACE,GRAB_RELEASE,GRAB_RETRACT,GRAB_RETURN_TURN};
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
 puts("mission child pause: queued next pickup, cancel, tracking, storage, pickup, held release and QR resume PASS");
}
