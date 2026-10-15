#define ChassisLocalization_Origin Legacy_Origin
#define main legacy_mission_main
#include "contest_mission_test.c"
#undef main
#undef ChassisLocalization_Origin
static float test_origin_x,test_origin_y,test_origin_yaw;
bool ChassisLocalization_Origin(float x,float y,float yaw){origin_calls++;test_origin_x=x;test_origin_y=y;test_origin_yaw=yaw;localization.origin_valid=true;return true;}
static unsigned probe_turn_calls;
static float probe_target;
static bool probe_accept=true;
bool ChassisMotion_TurnToPrecision(float yaw,float speed,float tol,uint32_t timeout){assert(speed==5 && tol==.1f && timeout);if(!probe_accept){motion.reason="busy";return false;}probe_turn_calls++;probe_target=yaw;motion.state=CHASSIS_MOTION_RUNNING;motion.stop_confirmed=false;return true;}
#include "../template/App/contest_mission.c"
static void fixture(MissionState s){ContestMission_Init();State=s;Single=false;Station=2;WorkYaw=180;RouteOptions.turn_speed_deg_s=10;motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";route.state="work";route.reason="station_work";grab.state=GRAB_COMPLETE;grab.result="work_ready";}
extern unsigned test_heading_starts,test_heading_processes,test_heading_pauses;
extern uint8_t test_heading_source;
extern bool test_heading_done;
extern float test_heading_image;
extern const char *test_heading_result;
static void aligned_station_anchor(void){
  fixture(M_EMPTY_ALIGN);start_fixture_custom=true;tick=1000;
  CenterX=1200;CenterY=380;Batch=Item=Pass=0;
  Rings[0][0]=1;Rings[0][1]=2;Rings[0][2]=3;
  grab.state=GRAB_HOLD;grab.result="aligned";grab.stop_confirmed=true;
  localization.origin_valid=localization.feedback_valid=true;
  localization.observer.feedback=(ChassisModel_Pose_t){1362,377,CHASSIS_MODEL_PI};
  localization.feedback_valid=false;ContestMission_Process();
  assert(State==M_EMPTY_ALIGN && !RingAnchorValid);
  localization.feedback_valid=true;ContestMission_Process();assert(State==M_TRANSFER_REQUEST);
  float x,y;Item=1;ring_target(&x,&y);assert(x==1212 && y==377);
  Item=2;ring_target(&x,&y);assert(x==1062 && y==377);
  Pass=1;Item=0;ring_target(&x,&y);assert(x==1362 && y==377);
  /* A new station visit discards the prior visit's position correction. */
  State=M_DRIVING;WorkReady=true;route.state="station";route.stop_confirmed=true;
  route.station=2;route.visit=6;route.target_x_mm=1200;route.target_y_mm=380;
  Rings[1][0]=2;ContestMission_Process();ring_target(&x,&y);assert(x==1200 && y==380);
  /* Temporary ring progression preserves distance on map X, including stacking. */
  fixture(M_TRANSFER);Station=3;Batch=1;Item=0;Rings[1][0]=1;Colors[1][0]=5;FirstRing[5]=2;
  Colors[1][1]=1;FirstRing[1]=3;
  localization.origin_valid=localization.feedback_valid=true;
  localization.observer.feedback=(ChassisModel_Pose_t){382,1184,CHASSIS_MODEL_PI/2};
  grab.state=GRAB_COMPLETE;grab.result="released";
  ContestMission_Process();assert(Item==1 && RingAnchorValid);
  ring_target(&x,&y);assert(x==382 && y==1334);
  ContestMission_Stop();ring_target(&x,&y);assert(x==382 && y==1334);
  /* Matching arrival yaw can omit work_turn and work_dock entirely. */
  fixture(M_DRIVING);WorkReady=WorkStarted=false;route.state="station";
  route.station=2;route.phase="work_approach";route.stop_confirmed=true;
  unsigned prepared=pose_calls;ContestMission_Process();assert(pose_calls==prepared+1 && WorkStarted);
  start_fixture_custom=false;
}
int main(void){
  aligned_station_anchor();
  /* One heading operation on each arrival, never on intra-station ring moves. */
  for(unsigned visit=3;visit<=7;visit++) {
    if(visit==5)continue; /* Raw station has no heading observation. */
    fixture(M_DRIVING);Request.visual_heading_enabled=true;WorkReady=true;
    route.state="station";route.stop_confirmed=true;route.station=(visit==3 || visit==6)?2:3;
    route.visit=visit;route.target_x_mm=1200;route.target_y_mm=300;
    ContestMission_Process();assert(State==M_HEADING && test_heading_starts==1);
    assert(test_heading_source==(visit==7?0x0e:0x0d));
    ContestMission_Process();assert(State==M_HEADING && test_heading_processes==1);
    test_heading_done=true;ContestMission_Process();assert(State==M_RING_CLEAR);
    for(unsigned pass=0;pass<2;pass++)for(unsigned item=0;item<3;item++) {
      State=M_RING_MOVE;Pass=pass;Item=item;
      grab.state=GRAB_COMPLETE;grab.result="work_ready";
      motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
      ContestMission_Process();assert(State==M_TRANSFER_REQUEST && test_heading_starts==1);
    }
  }
  /* Disabled arrival goes directly to per-item positioning/alignment. */
  fixture(M_DRIVING);WorkReady=true;route.state="station";route.stop_confirmed=true;
  route.station=2;route.visit=3;
  ContestMission_Process();assert(State==M_RING_CLEAR && test_heading_starts==0);
  fixture(M_RING_MOVE);grab.state=GRAB_COMPLETE;grab.result="work_ready";
  ContestMission_Process();assert(State==M_TRANSFER_REQUEST && test_heading_starts==0);
  fixture(M_IDLE);route.state="idle";start_fixture_custom=true;
  memset(&localization,0,sizeof localization);origin_calls=0;starts=transfers=pose_calls=0;
  inventory[0].state=TURNTABLE_OCCUPIED;inventory[0].color=3;
  grab.result="travel_ready";
  assert(!ContestMission_StartHeadingTest(2,5));assert(!strcmp(ContestMission_Reason(),"heading_prepare_required"));
  grab.result="work_ready";
  assert(ContestMission_StartHeadingTest(2,5));
  start_feedback_missing=true;ContestMission_Process();assert(origin_calls==0);
  start_feedback_missing=false;ContestMission_Process();assert(origin_calls==1);
  assert(test_origin_x==0 && test_origin_y==0 && fabsf(test_origin_yaw-CHASSIS_MODEL_PI)<.001f);
  localization.feedback_valid=localization.speed_valid=true;
  for(unsigned i=0;i<5;i++)ContestMission_Process();
  assert(State==M_HEADING && test_heading_starts==1);
  motion.stop_confirmed=false;ContestMission_Report();assert(strstr(report,"stop_confirmed=1"));
  test_heading_done=true;ContestMission_Process();assert(State==M_DONE);
  assert(starts==0 && transfers==0 && pose_calls==0 && inventory[0].state==TURNTABLE_OCCUPIED);
  assert(strstr(report,"mode=heading") && strstr(report,"heading_uncalibrated"));
  unsigned origins=origin_calls;
  assert(ContestMission_StartHeadingTest(3,5));
  for(unsigned i=0;i<5;i++)ContestMission_Process();
  assert(origin_calls==origins && WorkYaw==90);
  cancel_defer=true;ContestMission_Stop();assert(State==M_CANCELLING);
  ContestMission_Process();assert(State==M_CANCELLING);
  assert(!ContestMission_StartHeadingTest(2,5));
  cancel_stop.tx_complete=cancel_stop.wheels_stopped=true;motion.state=CHASSIS_MOTION_DONE;
  ContestMission_Process();assert(State==M_CANCELLED);
  assert(!ContestMission_Continue());
  /* Two separate observations surround exactly one signed, absolute turn. */
  for(int sample=-1;sample<=1;sample++)for(int sign=-1;sign<=1;sign+=2){
    const float angle=sample*9.51f;
    fixture(M_IDLE);route.state="idle";cancel_defer=false;probe_turn_calls=0;motion.map_yaw_rad=CHASSIS_MODEL_PI;
    assert(ContestMission_StartHeadingProbe(2,(float)sign,5));
    for(unsigned i=0;i<5;i++)ContestMission_Process();
    test_heading_image=angle;test_heading_done=true;ContestMission_Process();
    assert(probe_turn_calls==0);
    probe_accept=false;ContestMission_Process();assert(probe_turn_calls==0);
    probe_accept=true;ContestMission_Process();assert(probe_turn_calls==1);
    assert(fabsf(probe_target-remainderf(180+sign*angle,360))<.001f);
    ContestMission_Process();assert(test_heading_starts==1); /* Still moving. */
    motion.state=CHASSIS_MOTION_DONE;motion.map_yaw_rad=probe_target*CHASSIS_MODEL_PI/180;
    ContestMission_Process();assert(test_heading_starts==1); /* Stop not confirmed. */
    motion.stop_confirmed=true;ContestMission_Process();assert(test_heading_starts==2);
    test_heading_image=sign>0?.1f:19.02f;test_heading_done=true;ContestMission_Process();
    assert(State==M_DONE && probe_turn_calls==1 && fabsf(ProbeYawDelta-sign*angle)<.001f);
    assert(strstr(report,"heading_probe_done") && fabsf(ProbeBefore-angle)<.001f);
  }
  fixture(M_IDLE);route.state="idle";probe_turn_calls=0;
  assert(ContestMission_StartHeadingProbe(2,1,5));for(unsigned i=0;i<5;i++)ContestMission_Process();
  test_heading_result="heading_no_observation";test_heading_done=true;ContestMission_Process();
  assert(State==M_DONE && probe_turn_calls==0); /* No valid before frame, no motion. */
  /* Manual stop during the physical turn ends the test and never schedules a second turn. */
  fixture(M_IDLE);route.state="idle";probe_turn_calls=0;cancel_defer=false;
  assert(ContestMission_StartHeadingProbe(2,1,5));for(unsigned i=0;i<5;i++)ContestMission_Process();
  test_heading_image=9.51f;test_heading_done=true;ContestMission_Process();ContestMission_Process();
  assert(State==M_PROBE_MOVING && probe_turn_calls==1);
  cancel_defer=true;ContestMission_Stop();assert(State==M_CANCELLING);
  ContestMission_Process();assert(State==M_CANCELLING);
  cancel_stop.tx_complete=cancel_stop.wheels_stopped=true;motion.state=CHASSIS_MOTION_DONE;
  ContestMission_Process();assert(State==M_CANCELLED && probe_turn_calls==1 && !ContestMission_Continue());
  assert(!ContestMission_StartHeadingProbe(2,0,5));assert(!ContestMission_StartHeadingProbe(2,1,2));
  fixture(M_IDLE);route.state="idle";cancel_defer=false;
  assert(ContestMission_StartHeadingTestSource(3,5,0x0e));
  for(unsigned i=0;i<5;i++)ContestMission_Process();
  assert(test_heading_source==0x0e && strstr(report,"heading_source=line"));
  test_heading_done=true;ContestMission_Process();assert(State==M_DONE);
  assert(!ContestMission_StartHeadingTestSource(3,5,0x01));
  puts("mission heading: once per station arrival, no per-ring repeat, independent probes PASS");
}
