#define main legacy_mission_main
#include "contest_mission_test.c"
#undef main
#include "../template/App/contest_mission.c"

extern unsigned test_heading_starts,test_heading_competition_starts;
extern bool test_heading_done;
extern uint8_t test_heading_source;

static void departure_fixture(unsigned batch,bool enabled)
{
  ContestMission_Init();State=M_TRANSFER_REQUEST;Single=RingOnly=false;
  Batch=batch;Station=2;Visit=batch?6:3;Item=2;Pass=1;
  Stored=batch?11:5;Placed=batch?9:3;
  place=Placed;
  WorkYaw=180;Speed=200;CenterX=1200;CenterY=350;
  Request.visual_heading_enabled=enabled;Request.align_raised=true;
  RouteOptions.turn_speed_deg_s=100;Colors[batch][2]=6;Rings[batch][2]=3;
  route.state="work";route.reason="station_work";route.phase="work_dock";
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
  grab.state=GRAB_COMPLETE;grab.result="work_ready";grab.stop_confirmed=true;
  grab_resume_pending=pose_clearance=false;reject_transfer=false;reject_pose=0;reject_move=NULL;
  start_imu_missing=start_feedback_missing=false;
  start_fixture_custom=true;memset(&localization,0,sizeof localization);
  localization.origin_valid=localization.feedback_valid=localization.speed_valid=true;
  localization.observer.feedback=(ChassisModel_Pose_t){1050,350,CHASSIS_MODEL_PI};
  tick+=100;tx=ty=-1;
}
static void processing_departure(unsigned batch,bool enabled)
{
  departure_fixture(batch,enabled);
  unsigned travel=store_travel_calls,poses=pose_calls;
  ContestMission_Process();assert(State==M_TRANSFER);
  assert(store_travel_calls==travel+!enabled);
  grab.state=GRAB_COMPLETE;grab.result="stored";grab.slot=3;
  ContestMission_Process();
  assert(Stored==(batch?12U:6U) && Placed==(batch?9U:3U) && Item==3);
  if(!enabled){assert(State==M_REAR_REQUEST && !test_heading_starts);return;}
  assert(State==M_CLEAR_REQUEST);
  ContestMission_Process();assert(State==M_RING_CLEAR && pose_work && pose_calls==poses+1);
  /* Arm clearance alone must not start a ring move or a yaw observation. */
  pose_clearance=true;ContestMission_Process();
  assert(State==M_RING_CLEAR && !test_heading_starts && tx==-1);
  grab.state=GRAB_COMPLETE;grab.result="work_ready";
  ContestMission_Process();
  assert(State==M_HEADING && HeadingNext==M_REAR_REQUEST);
  assert(test_heading_competition_starts==1 && test_heading_source==STATION_HEADING_RING);
  ContestMission_Report();assert(strstr(report,"state=departure_heading"));
  for(unsigned i=0;i<3;i++)ContestMission_Process();
  assert(State==M_HEADING && pose_calls==poses+1 && pose_work && tx==-1);
  assert(Stored==(batch?12U:6U) && Item==3);
  test_heading_done=true;ContestMission_Process();assert(State==M_REAR_REQUEST);
  ContestMission_Process();assert(State==M_LEAVE && !pose_work && pose_calls==poses+2);
  assert(tx==-1 && test_heading_starts==1);
}
static void pause_departure(bool during_heading)
{
  departure_fixture(0,true);ContestMission_Process();
  grab.state=GRAB_COMPLETE;grab.result="stored";grab.slot=3;
  ContestMission_Process();ContestMission_Process();assert(State==M_RING_CLEAR);
  if(during_heading){grab.state=GRAB_COMPLETE;grab.result="work_ready";ContestMission_Process();assert(State==M_HEADING);}
  unsigned headings=test_heading_starts;
  ContestMission_Stop();ContestMission_Process();assert(State==M_PAUSED);
  assert(Stored==6 && Item==3 && tx==-1);
  assert(ContestMission_Continue());
  for(unsigned i=0;i<6 && State==M_RESUMING;i++)ContestMission_Process();
  assert(State==(during_heading?M_HEADING:M_RING_CLEAR));
  assert(test_heading_starts==headings && Stored==6 && tx==-1);
  if(!during_heading){grab.state=GRAB_COMPLETE;grab.result="work_ready";ContestMission_Process();}
  assert(State==M_HEADING && HeadingNext==M_REAR_REQUEST);
  test_heading_done=true;ContestMission_Process();assert(State==M_REAR_REQUEST && Stored==6);
}
int main(void)
{
  processing_departure(0,true);processing_departure(1,true);
  processing_departure(0,false);processing_departure(1,false);
  pause_departure(false);pause_departure(true);
  puts("processing departure heading: stored first, observe at work pose, rear pose afterwards PASS");
  return 0;
}
