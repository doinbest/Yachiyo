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
extern bool test_heading_done;
extern float test_heading_image;
extern const char *test_heading_result;
int main(void){
  fixture(M_RING_MOVE);Request.visual_heading_enabled=true;
  grab.state=GRAB_COMPLETE;grab.result="work_ready";
  ContestMission_Process();assert(State==M_HEADING && test_heading_starts==1);
  ContestMission_Process();assert(State==M_HEADING && test_heading_processes==1);
  test_heading_done=true;ContestMission_Process();assert(State==M_TRANSFER_REQUEST);
  /* Captured empty-ring placement cannot be intercepted by the new phase. */
  RingAlignmentSaved=true;assert(State==M_TRANSFER_REQUEST);
  Request.visual_heading_enabled=false;fixture(M_RING_MOVE);
  grab.state=GRAB_COMPLETE;grab.result="work_ready";ContestMission_Process();
  assert(State==M_TRANSFER_REQUEST && test_heading_starts==0);
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
  puts("mission heading: optional rendezvous before transfer, old disabled flow PASS");
}
