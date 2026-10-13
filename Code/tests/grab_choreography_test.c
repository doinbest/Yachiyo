#define main legacy_main
#include "grab_task_test.c"
#undef main

static void setup(void)
{
  reset();configure(true);setting("x_min","0");setting("x_max","130");
  setting("z_min","-150");setting("z_max","0");setting("z_grab","-50");
  setting("z_lift","-10");setting("z_car_lift","0");
  setting("z_observe","0");
  setting("z_rough_align","-100");setting("z_proc_place","-130");
  RearBase=37;RearReferenceValid=true;OutsidePose=true;
  physical[0]=30;physical[1]=-5;physical[2]=37+GRAB_BASE_WORK_OFFSET_DEG;
}
static void finish_store(bool processed, bool pause)
{
  setup();
  assert(processed?GrabTask_ProcessedStoreStart(2,1,3):GrabTask_TrackedStoreStart(2,3,GRAB_TRACK_FEEDBACK));
  assert(GrabTask_StoreFinishTravel());
  run_to(GRAB_RELEASE,1000,0);
  if(pause)
  {
    assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
    assert(GrabTask_Continue());
  }
  unsigned closed=closures,opened=openings;
  for(unsigned n=0;n<700 && status().state!=GRAB_COMPLETE;n++)
  {
    step(true,0,0,0);
    assert(status().state!=GRAB_OBSERVE_X && status().state!=GRAB_OBSERVE_Z);
    assert(fabsf(destination[2]-(37+GRAB_BASE_WORK_OFFSET_DEG))>.15f); /* Never return to the observation heading. */
  }
  assert(status().state==GRAB_COMPLETE && !strcmp(status().result,"stored"));
  assert(tray_inventory[2].state==TURNTABLE_OCCUPIED && tray_inventory[2].color==2);
  assert(closures==closed && openings==opened && RearPose && !OutsidePose);
  assert(fabsf(physical[2]-37)<.15f && physical[0]==0 && physical[1]==0);
}
static void clearance_is_live_and_pose_only(bool qr)
{
  setup();assert(!GrabTask_PoseClearanceReady());
  base_arrives=false;
  assert(qr?GrabTask_QrPoseStart():GrabTask_PoseStart(false));assert(!GrabTask_PoseClearanceReady());
  for(unsigned n=0;n<300 && !GrabTask_PoseClearanceReady();n++)step(true,0,0,0);
  assert(GrabTask_PoseClearanceReady() && status().state==GRAB_ROUTE_POSE);
  GrabAxis_t x=Axes[0],z=Axes[1],base=Axes[2];
  Axes[0].mm=4;assert(!GrabTask_PoseClearanceReady());Axes[0]=x;
  Axes[1].mm=-10;assert(!GrabTask_PoseClearanceReady());Axes[1]=z;
  Axes[0].quiet=false;assert(!GrabTask_PoseClearanceReady());Axes[0]=x;
  Axes[1].position_tick=clock_ms-3000;assert(!GrabTask_PoseClearanceReady());Axes[1]=z;
  Axes[2].flags=9;assert(!GrabTask_PoseClearanceReady());Axes[2]=base;
  assert(GrabTask_Pause());assert(!GrabTask_PoseClearanceReady());run_to(GRAB_IDLE,200,0);
  assert(GrabTask_Continue());assert(!GrabTask_PoseClearanceReady());
  for(unsigned n=0;n<300 && !GrabTask_PoseClearanceReady();n++)step(true,0,0,0);
  assert(GrabTask_PoseClearanceReady());
  base_arrives=true;physical[2]=qr?72:37;run_to(GRAB_COMPLETE,200,0);
  assert(!strcmp(status().result,qr?"qr_pose_ready":"travel_ready"));
  assert(!GrabTask_PoseClearanceReady());
}
int main(void)
{
  finish_store(false,false);finish_store(false,true);finish_store(true,false);finish_store(true,true);
  clearance_is_live_and_pose_only(false);clearance_is_live_and_pose_only(true);
  setup();assert(GrabTask_TrackedStoreStart(1,1,GRAB_TRACK_FEEDBACK));run_to(GRAB_COMPLETE,1000,0);
  assert(OutsidePose && fabsf(physical[2]-(37+GRAB_BASE_WORK_OFFSET_DEG))<.15f);
  puts("grab choreography: terminal storage, pause, default return and fresh pose clearance PASS");
}
