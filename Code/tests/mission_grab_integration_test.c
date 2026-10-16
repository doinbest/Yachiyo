#include "mission_navigation.h"
void MissionNavigation_Reset(void){}
void MissionNavigation_Pause(void){}
int MissionNavigation_Process(RadarStart_Id_t s,uint32_t g,RadarPlan_t *p,const char **r){(void)s;(void)g;(void)p;*r="radar_scanning";return 0;}
/* Production mission + production GrabTask/tracking; emulate only hardware and
 * station travel. This catches state/result contracts hidden by child mocks. */
static void mission_before_grab(void);
#define TEST_GRAB_BEFORE_PROCESS mission_before_grab
#define GRAB_TEST_EXTERNAL_LOCALIZATION
#define main legacy_main
#include "grab_task_test.c"
#undef main
#include "contest_mission.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
static ChassisRoute_Status_t route;
static ChassisMotion_Status_t motion;
static ChassisMotion_Profile_t selected_profile;
ChassisMotion_Profile_t ChassisMotion_ProfileGet(void){return selected_profile;}
bool ChassisMotion_ProfileSet(ChassisMotion_Profile_t p)
{if(ChassisMotion_IsBusy() || (p!=CHASSIS_PROFILE_LINEAR && p!=CHASSIS_PROFILE_COSINE))return false;selected_profile=p;return true;}
static char displayed[16];
static unsigned resumed,stored,placed,pause_count;
static unsigned overlapping_moves,busy_moves,last_base_moves;
static unsigned overlap_pauses;
static bool overlap_in_flight;
static uint32_t base_due,motion_due;
static bool integration_mission_enabled,standalone_without_origin;
static bool expected_raised;
static unsigned raised_ring,raised_pick,raised_stack,stack_camera_requests;
static const char *paused_route;
static const GrabTask_State_t pause_stages[]={GRAB_TRACK_OBSERVE,GRAB_DESCEND,GRAB_RELEASE,GRAB_CLOSE,GRAB_ALIGN,GRAB_RETRACT};
bool ChassisRoute_StationReserved(void){return !strcmp(route.state,"work") || !strcmp(route.state,"station");}
bool ChassisRoute_TaskStart(const RadarPlan_t *p,float s){(void)p;(void)s;route.state="running";return true;}
bool ChassisRoute_TaskStartEx(const RadarPlan_t *p,float s,const ChassisRoute_TaskOptions_t *o){assert(o->scan_qr);return ChassisRoute_TaskStart(p,s);}
bool ChassisRoute_TaskPause(void){paused_route=route.state;route.state="paused";route.stop_confirmed=true;chassis_pose_motion=false;motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="cancelled";return true;}
bool ChassisRoute_TaskPauseFeedbackLoss(void){return ChassisRoute_TaskPause();}
bool ChassisRoute_TaskContinue(void){route.state=paused_route;return true;}
bool ChassisRoute_TaskResumeReady(void){return true;}
void ChassisRoute_TaskRecoveryHold(bool hold){(void)hold;}
void Mecanum_Feedback_StopMonitor(bool on){(void)on;}
void Mecanum_Feedback_Arrival(bool on){(void)on;}
bool MotorBus_RecoveryStart(void){return false;}
void MotorBus_RecoveryGet(MotorBus_Recovery_t *s){memset(s,0,sizeof *s);}
void MotorBus_RecoveryCancel(void){}
bool Mecanum_RecoveryAfterBusCheck(void){return false;}
bool ChassisRoute_WorkBegin(void){route.state="work";return true;}
bool ChassisRoute_WorkResume(void){route.state="running";route.phase="move";resumed++;return true;}
bool ChassisRoute_StationFinish(void){route.state="done";return true;}
bool ChassisRoute_Cancel(void){route.state="cancelled";return true;}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out){*out=route;}
bool ChassisMotion_MoveTo(float x,float y,float a,float s,uint32_t t){(void)x;(void)y;(void)a;(void)s;(void)t;
 if(MechanicalArm_IsBusy()){busy_moves++;motion.reason="busy";return false;}
 overlap_in_flight=status().state==GRAB_ROUTE_POSE;
 if(overlap_in_flight){assert(GrabTask_PoseClearanceReady());overlapping_moves++;}
 /* Exercise both completion orders, including an interrupted paired move. */
 assert(!chassis_pose_motion);chassis_pose_motion=true;motion_due=clock_ms+(overlap_pauses==2?2500:250);
 motion.state=CHASSIS_MOTION_RUNNING;motion.stop_confirmed=false;motion.reason="running";return true;}
bool ChassisMotion_Stop(uint32_t ms){assert(!ms);motion.stop_confirmed=true;return true;}
bool ChassisMotion_StopRequest(uint32_t token){assert(!token);return true;}
bool ChassisMotion_StopPending(void){return false;}
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *s){memset(s,0,sizeof *s);s->tx_complete=s->wheels_stopped=true;s->requested_ms=clock_ms;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out){*out=motion;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *out)
{memset(out,0,sizeof *out);out->origin_valid=out->feedback_valid=out->speed_valid=true;
 out->feedback_tick=clock_ms;out->origin=(ChassisModel_Pose_t){2250,150,CHASSIS_MODEL_PI};
 out->observer.feedback=out->origin;if(standalone_without_origin)out->origin_valid=false;}
bool ChassisLocalization_Origin(float x,float y,float yaw)
{(void)x;(void)y;(void)yaw;assert(false);return false;}
void ContestScreen_TaskCodeClear(void){displayed[0]=0;}
uint8_t ContestScreen_TaskCodeRefresh(const char *code){strcpy(displayed,code);return 1;}
uint8_t ContestScreen_TaskCodeSent(void){return 1;}
uint8_t ContestScreen_HasError(void){return 0;}
uint8_t ContestScreen_ProgressSet(uint8_t a,uint8_t b){stored=a;placed=b;return 1;}
static void mission_before_grab(void){if(integration_mission_enabled)ContestMission_Process();}
static void cycle(void){
 if(last_base_moves!=base_moves){last_base_moves=base_moves;base_due=clock_ms+800;}
 if(base_due && clock_ms>=base_due){physical[2]=destination[2];base_due=0;}
 if(chassis_pose_motion && clock_ms>=motion_due){chassis_pose_motion=false;motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";}
 /* One loop with an outstanding reply, then a completed motor event before
  * the parent and GrabTask, matching the production ownership order. */
 ContestMission_Process();step(true,0,0,0);
  if(route.station==3 && route.visit==4 && !strcmp(status().operation,"car_pick")){
    assert(AlignedPlace.valid && AlignedPlace.scene==SCENE_TEMP1);
  }
 if(route.station==3 && route.visit==7 && !strcmp(status().operation,"car_pick")){
   assert(AlignedPlace.valid && AlignedPlace.scene==SCENE_TEMP2 && AlignedPlace.ring==(uint8_t)ActiveColor);
   stack_camera_requests=camera_requests;
 }
 if(route.station==3 && route.visit==7 && PrealignedPlaceActive)assert(camera_requests==stack_camera_requests);
 if(status().state==GRAB_ALIGN && (EmptyPlaceAlignActive || ProcessedStoreActive || HeldPlaceActive)){
   assert(StationAlignRaised==expected_raised);
   if(expected_raised){assert(fabsf(physical[1])<0.01f);
     if(EmptyPlaceAlignActive)raised_ring++;
     if(ProcessedStoreActive)raised_pick++;
     if(EmptyPlaceAlignActive && Scene==SCENE_TEMP2){assert(!TrackHeld);raised_stack++;}
   }
 }
 if(overlap_in_flight && overlap_pauses<3 &&
    ((overlap_pauses==0 && chassis_pose_motion && status().state==GRAB_ROUTE_POSE) ||
     (overlap_pauses==1 && !chassis_pose_motion && status().state==GRAB_ROUTE_POSE) ||
     (overlap_pauses==2 && chassis_pose_motion && status().state==GRAB_COMPLETE))){
   /* Stop cancels the delayed physical Base movement too; resume must issue
    * the remaining absolute goal rather than relying on old completion. */
   ContestMission_Stop();assert(!chassis_pose_motion);
   base_due=0;last_base_moves=base_moves;overlap_in_flight=false;overlap_pauses++;
 }
 if(pause_count<sizeof pause_stages/sizeof *pause_stages && status().state==pause_stages[pause_count]){
   ContestMission_Stop();ContestMission_Stop();pause_count++;
 }
 if(ContestMission_CanConfigure())assert(ContestMission_Continue());
}
static void visit(unsigned id,unsigned number)
{
 route.phase="work_turn";
 for(unsigned i=0;i<200;i++)cycle();
 route.state="station";route.station=id;route.visit=number;route.stop_confirmed=true;
 route.target_x_mm=id==3?350:1200;route.target_y_mm=id==4?2100:id==2?350:1200;
 unsigned old=resumed;
 for(unsigned i=0;i<6000 && resumed==old;i++)cycle();
 if(resumed==old){ContestMission_Report();fprintf(stderr,"mission stalled: %s child=%s/%s\n",output,status().state_name,status().reason);}
 assert(resumed==old+1);
}
static void run_match(bool raised)
{
 integration_mission_enabled=false;standalone_without_origin=false;expected_raised=raised;
 resumed=stored=placed=pause_count=overlapping_moves=busy_moves=last_base_moves=overlap_pauses=0;
 raised_ring=raised_pick=raised_stack=0;overlap_in_flight=false;base_due=motion_due=0;paused_route=NULL;
 memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
 reset();configure(true);setting("x_min","0");setting("x_max","100");setting("z_min","-150");setting("z_max","0");
 setting("z_observe","0");setting("z_grab","-50");setting("z_lift","0");setting("z_car_lift","0");setting("z_place","-40");
 setting("z_rough_align","-10");setting("z_temp_align","-10");setting("z_proc_place","-40");setting("z_stack_place","-20");
 RearBase=37;RearReferenceValid=true;physical[2]=37;base_arrives=false;
 route.state="idle";route.phase="idle";route.reason="idle";
 motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
 motion.heading_valid=true;
 ContestMission_Init();ContestMission_Request_t request={MISSION_CODE_FIXED,"156+123+516+231",200,50,RADAR_START_LOWER,false,false,false,CHASSIS_PROFILE_LINEAR};
 request.align_raised=raised;
 request.visual_heading_enabled=true;
 assert(ContestMission_Start(NULL,&request));
 request.align_raised=!raised; /* Caller edits cannot change this accepted match. */
 ContestMission_Report();assert(strstr(output,raised?"align=raised":"align=lowered"));
 integration_mission_enabled=true;
 for(unsigned i=0;i<200;i++)cycle();
 route.state="station";route.station=5;route.visit=1;route.stop_confirmed=true;
 for(unsigned i=0;i<20;i++)cycle();
 qr_snapshot.Valid=1;qr_snapshot.Source=QR_SOURCE_UART;qr_snapshot.Sequence++;strcpy(qr_snapshot.Code,"356+123+536+231");
 for(unsigned i=0;i<200 && !resumed;i++)cycle();
 assert(resumed==1 && !strcmp(displayed,"356+123+536+231"));
 visit(4,2);assert(stored==3 && !placed);
 visit(2,3);assert(stored==6 && placed==3);
 visit(3,4);assert(stored==6 && placed==6);
 visit(4,5);visit(2,6);visit(3,7);
 fprintf(stderr,"integration stored=%u placed=%u paused=%u overlap=%u overlap_pauses=%u busy=%u\n",stored,placed,pause_count,overlapping_moves,overlap_pauses,busy_moves);
 assert(stored==12 && placed==12 && pause_count==6 && overlap_pauses==3 && overlapping_moves>0 && busy_moves>0);
 extern unsigned test_heading_competition_starts;
 assert(test_heading_competition_starts==7); /* Four arrivals, two processing departures and one first-layer temporary departure. */
 route.state="done";cycle();assert(!ContestMission_IsBusy());
 if(raised)assert(raised_ring && raised_pick && raised_stack);
 ContestMission_Report();assert(strstr(output,raised?"align=raised":"align=lowered"));
 puts("mission + production grab/tracking integration: two batches, real action states, 12 stored/12 placed PASS");
}
static void standalone_ring_tray_place(void)
{
 for(uint8_t slot=1;slot<=3;slot++){
  integration_mission_enabled=false;standalone_without_origin=true;expected_raised=true;
  memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
  route.state="cancelled";route.reason="cancelled";motion.state=CHASSIS_MOTION_ERROR;motion.stop_confirmed=true;
  motion.heading_valid=true; /* The fixture's IMU is healthy throughout this pause/resume test. */
  reset();configure(true);setting("x_min","0");setting("x_max","100");setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_car_lift","0");setting("z_proc_place","-140");
  RearBase=37;RearReferenceValid=true;physical[2]=RearBase+GRAB_BASE_WORK_OFFSET_DEG;
  physical[0]=42;tray_inventory[slot-1]=(Turntable_Inventory_t){TURNTABLE_UNKNOWN,0};
  ContestMission_Init();assert(!ContestMission_StartRingTake(slot)); /* An aligned ring is required. */
  assert(GrabTask_AlignStart(11,0,true));run_to(GRAB_HOLD,600,0);
  float release_drop=slot==2?130:140;
  assert(!closures);assert(slot==2?ContestMission_StartRingTakeEx(slot,release_drop,35):ContestMission_StartRingTake(slot));
  unsigned requests=0,closed=closures,opened=0;bool pickup_paused=false,descent_paused=false,release_paused=false,lift_checked=false;
  for(unsigned i=0;i<3000 && ContestMission_IsBusy();i++){
   ContestMission_Process();step(true,0,0,0);
   if(!strcmp(status().operation,"car_pick")){assert(AlignedPlace.valid && AlignedPlace.ring==2);requests=camera_requests;}
   if(PrealignedPlaceActive){assert(ActiveColor==0 && camera_requests==requests);if(!opened)opened=openings;
    if(status().state==GRAB_DESCEND && !descent_paused){ContestMission_Stop();descent_paused=true;}}
   if(status().state==GRAB_CLOSE && !pickup_paused){ContestMission_Stop();pickup_paused=true;}
   if(status().state==GRAB_LIFT && CarPickClosed && !lift_checked){
    lift_checked=true;z_arrives=false;unsigned turns=base_moves;
    for(unsigned wait=0;wait<25;wait++){
     ContestMission_Process();step(true,0,0,0);
     assert(base_moves==turns && physical[1]<0); /* Standalone ring pickup must lift first. */
    }
    z_arrives=true;physical[1]=destination[1];
   }
   if(status().state==GRAB_RELEASE && !release_paused){assert(physical[1]==-release_drop);ContestMission_Stop();release_paused=true;}
   if(ContestMission_CanConfigure())assert(ContestMission_Continue());
  }
  if(ContestMission_IsBusy() || TrackHeld || closures!=closed+1 || AlignedPlace.valid){
    ContestMission_Report();fprintf(stderr,"ring slot=%u held=%u closed=%u expected=%u saved=%u child=%s reason=%s parent=%s\n",
      slot,TrackHeld,closures,closed+1,AlignedPlace.valid,States[status().state],status().reason,output);
  }
  assert(!ContestMission_IsBusy() && !TrackHeld && closures==closed+1 && !AlignedPlace.valid);
  assert(pickup_paused && descent_paused && release_paused && lift_checked && openings==opened+1);
  assert(tray_inventory[slot-1].state==TURNTABLE_EMPTY && physical[1]==0);
  assert(fabsf(physical[0]-42)<.1f && fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.1f);
  ContestMission_Report();assert(strstr(output,"mode=ring_place") && strstr(output,"ring_pickplace_complete"));
 }
 puts("standalone ring tray place: selected tray without inventory, fresh capture, no held vision, pauses and one release PASS");
}
int main(void){run_match(false);run_match(true);standalone_ring_tray_place();return 0;}
