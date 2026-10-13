#define main legacy_mission_main
#include "contest_mission_test.c"
#undef main
#include "../template/App/contest_mission.c"

static void fixture(MissionState stage)
{
  ContestMission_Init();State=stage;Single=false;Speed=200;Station=2;Visit=3;Item=0;Pass=0;
  CenterX=1200;CenterY=350;WorkYaw=180;Rings[0][0]=1;Colors[0][0]=1;
  route.state="work";route.reason="station_work";route.phase="work_dock";
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
  grab.state=GRAB_ROUTE_POSE;grab.reason="preparing_travel";grab.result="pending";
  grab_resume_pending=pose_clearance=false;reject_move=NULL;route_resume_delayed=false;
  tick+=100;tx=ty=-1;
}
static void overlap_and_rendezvous(bool work)
{
  fixture(work?M_RING_CLEAR:M_LEAVE);
  ContestMission_Process();assert(tx==-1); /* Low/extended arm cannot admit chassis. */
  pose_clearance=true;reject_move="busy";
  ContestMission_Process();assert(tx==-1 && State==(work?M_RING_CLEAR:M_LEAVE));
  reject_move=NULL;ContestMission_Process();
  assert(tx==(work?1350:1200) && State==(work?M_RING_MOVE:M_CENTER));
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
  unsigned old=resumes;
  ContestMission_Process();assert(State==(work?M_RING_MOVE:M_CENTER) && resumes==old);
  grab.state=GRAB_COMPLETE;grab.result=work?"work_ready":"travel_ready";
  ContestMission_Process();assert(work?State==M_TRANSFER_REQUEST:State==M_DRIVING);
  assert(resumes==old+!work);
}
static void overlap_pause(void)
{
  fixture(M_LEAVE);pose_clearance=true;ContestMission_Process();assert(State==M_CENTER);
  ContestMission_Stop();assert(grab_resume_pending);ContestMission_Process();assert(State==M_PAUSED);
  assert(ContestMission_Continue());pose_clearance=false;ContestMission_Process();
  assert(State==M_LEAVE && grab.state==GRAB_ROUTE_POSE);
  tx=-1;ContestMission_Process();assert(tx==-1);
  pose_clearance=true;ContestMission_Process();assert(State==M_CENTER && tx==1200);
}
static void qr_departure_and_upper_start(void)
{
  fixture(M_QR_LEAVE);unsigned old=resumes;
  ContestMission_Process();assert(State==M_QR_LEAVE && resumes==old);
  pose_clearance=true;ContestMission_Process();
  assert(State==M_DRIVING && resumes==old+1 && grab.state==GRAB_ROUTE_POSE);
  /* Upper start can leave with a moving QR Base after live clearance, while
   * retaining the usual initial localization and feedback checks. */
  fixture(M_PREPARE);UpperQrPreparing=true;RouteOptions.start_id=RADAR_START_UPPER;
  RouteOptions.scan_qr=true;RouteOptions.turn_speed_deg_s=50;
  start_fixture_custom=true;memset(&localization,0,sizeof localization);
  localization.origin=(ChassisModel_Pose_t){2250,2250,-CHASSIS_MODEL_PI/2};
  localization.observer.feedback=localization.origin;
  localization.origin_valid=localization.feedback_valid=localization.speed_valid=true;
  Request.radar_enabled=false;unsigned before=starts;
  ContestMission_Process();assert(starts==before && State==M_PREPARE);
  pose_clearance=true;ContestMission_Process();
  assert(starts==before+1 && State==M_DRIVING && grab.state==GRAB_ROUTE_POSE);
  start_fixture_custom=false;
}
static void late_work_pose(unsigned station,bool pause,bool completing_during_correction)
{
  fixture(M_DRIVING);WorkStarted=WorkReady=false;single_test=false;raw=0;
  route.state="running";route.phase="work_dock";route.station=station;route.visit=2;
  route.stop_confirmed=false;route.target_x_mm=1200;route.target_y_mm=2100;
  grab.state=GRAB_COMPLETE;grab.result="travel_ready";
  unsigned old_pose=pose_calls,old_transfers=transfers;
  ContestMission_Process();assert(WorkStarted && !WorkReady && pose_calls==old_pose+1);
  /* Position repair replaces work_dock with move before pose completion. */
  route.phase="move";route.state=completing_during_correction?"running":"station";
  route.stop_confirmed=!completing_during_correction;
  ContestMission_Process();assert(State==M_DRIVING && !WorkReady && transfers==old_transfers);
  if(pause){
    ContestMission_Stop();ContestMission_Process();assert(State==M_PAUSED);
    assert(ContestMission_Continue());ContestMission_Process();assert(State==M_DRIVING && WorkStarted);
  }
  grab.state=GRAB_COMPLETE;grab.result="work_ready";
  if(completing_during_correction){
    ContestMission_Process();assert(WorkReady && State==M_DRIVING && transfers==old_transfers);
    route.state="station";route.stop_confirmed=false;
    ContestMission_Process();assert(State==M_DRIVING && transfers==old_transfers);
  }
  route.stop_confirmed=true;ContestMission_Process();
  assert(WorkReady && State==(station==4?M_TRANSFER_REQUEST:M_RING_CLEAR));
  assert(pose_calls==old_pose+1); /* Completion must not restart the pose. */
  if(station==4){ContestMission_Process();assert(transfers==old_transfers+1 && State==M_TRANSFER);}
}
static void stale_pose_does_not_start_work(void)
{
  fixture(M_DRIVING);WorkStarted=WorkReady=false;single_test=false;raw=0;
  route.state="station";route.phase="move";route.station=4;route.visit=2;route.stop_confirmed=true;
  grab.state=GRAB_COMPLETE;grab.result="work_ready";
  unsigned old=pose_calls;ContestMission_Process();
  assert(State==M_DRIVING && !WorkStarted && !WorkReady && pose_calls==old);
}
int main(void)
{
  legacy_mission_main();assert(store_travel_calls==5); /* four batches + single-piece route */
  overlap_and_rendezvous(false);overlap_and_rendezvous(true);overlap_pause();
  qr_departure_and_upper_start();
  late_work_pose(4,false,false);late_work_pose(4,true,true);
  late_work_pose(2,false,true);late_work_pose(3,true,false);stale_pose_does_not_start_work();
  fixture(M_DRIVING);WorkReady=true;WorkStarted=true;
  route.state="station";route.station=2;route.visit=3;route.stop_confirmed=true;
  route.target_x_mm=1200;route.target_y_mm=350;
  grab.state=GRAB_COMPLETE;grab.result="work_ready";
  unsigned old=pose_calls;ContestMission_Process();ContestMission_Process();
  assert(pose_calls==old && State==M_RING_MOVE);
  puts("contest choreography: terminal policy, pose reuse, overlapping motion and pause rendezvous PASS");
}
