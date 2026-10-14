#include "mission_navigation.h"
static int navigation_result;
static unsigned navigation_calls,qr_pose_calls;
static RadarStart_Id_t navigation_start,route_start;
static bool route_planned;
void MissionNavigation_Reset(void){}
void MissionNavigation_Pause(void){}
int MissionNavigation_Process(RadarStart_Id_t s,uint32_t g,RadarPlan_t *p,const char **r){navigation_calls++;navigation_start=s;(void)g;(void)p;*r=navigation_result<0?"radar_no_route":"radar_scanning";return navigation_result;}
#include "contest_mission.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "QR.h"
#include "turntable.h"
#include "motor_bus.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick,qr_baseline;
static char screen_code[16];
static bool screen_sent=true,screen_error;
static unsigned screen_refreshes;
static bool qr_scanning;
static ContestMission_Request_t request={MISSION_CODE_FIXED,"156+123+516+231",200,50,RADAR_START_LOWER,false,false,false,CHASSIS_PROFILE_LINEAR};
static ChassisRoute_Status_t route;
static ChassisMotion_Status_t motion;
static ChassisMotion_Profile_t selected_profile;
ChassisMotion_Profile_t ChassisMotion_ProfileGet(void){return selected_profile;}
bool ChassisMotion_ProfileSet(ChassisMotion_Profile_t p)
{if(ChassisMotion_IsBusy() || (p!=CHASSIS_PROFILE_LINEAR && p!=CHASSIS_PROFILE_COSINE))return false;selected_profile=p;return true;}
static GrabTask_Status_t grab;
static Turntable_Inventory_t inventory[3];
static unsigned transfers,raw,place,proc,resumes,starts,pose_calls,cancels;
static unsigned store_travel_calls;
static bool pose_clearance;
static bool pose_work,reject_transfer,single_test;
static bool ring_captured,alignment_blocked;
static unsigned ring_aligns,ring_captures,rough_picks;
static unsigned reject_route,reject_pose;
static const char *reject_move;
static char report[700];
static bool cancel_defer;
static bool stop_request_tick_advance;
static bool route_resume_delayed;
static unsigned grab_resume_calls;
static ChassisStop_Status_t cancel_stop;
static bool cancel_stop_pending;
static float tx,ty;
static ChassisLocalization_Status_t localization;
static bool start_fixture_custom, feedback_enabled;
static unsigned feedback_selections, origin_calls;
static MotorBus_Diagnostic_t recovery_diagnostic;
static MotorBus_Recovery_t mission_recovery;
static unsigned recovery_starts;
static const char *paused_route_state;
static GrabTask_Status_t paused_grab;
static bool grab_resume_pending;
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *s){*s=recovery_diagnostic;}
bool MotorBus_RecoveryStart(void){recovery_starts++;mission_recovery.active=true;mission_recovery.success=false;mission_recovery.reason="checking";return true;}
void MotorBus_RecoveryGet(MotorBus_Recovery_t *s){*s=mission_recovery;}
void MotorBus_RecoveryCancel(void){mission_recovery.active=mission_recovery.success=false;mission_recovery.reason="cancelled";}
bool Mecanum_RecoveryAfterBusCheck(void){return mission_recovery.success;}
uint8_t MechanicalArm_IsBusy(void){return 0;}
void Mecanum_Feedback_StopMonitor(bool on){(void)on;}
static uint8_t start_wheel_flags=3;
static int start_wheel_rpm;
static bool start_feedback_missing, start_imu_missing, origin_rejected;
bool Mecanum_Feedback_Select(uint8_t address){assert(address==0);feedback_selections++;return true;}
void Mecanum_Feedback_Enable(bool enabled){feedback_enabled=enabled;}
bool Mecanum_FeedbackGet(unsigned wheel,Mecanum_Feedback_t *out)
{assert(wheel<4);memset(out,0,sizeof *out);out->speed_valid=out->position_valid=out->state_valid=!start_feedback_missing;
 out->speed_ms=out->position_ms=out->state_ms=tick;out->state_flags=start_wheel_flags;out->speed_rpm=start_wheel_rpm;return !start_feedback_missing;}
void Mecanum_StatusGet(Mecanum_Status_t *out){memset(out,0,sizeof *out);out->ack_profile=MECANUM_ACK_RECEIVE;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *out)
{if(!start_fixture_custom){memset(&localization,0,sizeof localization);localization.origin_valid=localization.feedback_valid=localization.speed_valid=true;
 localization.origin.x_mm=localization.observer.feedback.x_mm=2250;localization.origin.y_mm=localization.observer.feedback.y_mm=150;
 localization.origin.yaw_rad=localization.observer.feedback.yaw_rad=CHASSIS_MODEL_PI;}
 if(!start_fixture_custom && !strcmp(route.state,"work")) {
   localization.observer.feedback.x_mm=tx;localization.observer.feedback.y_mm=ty;
 }
 localization.feedback_tick=tick;*out=localization;}
bool ChassisLocalization_Origin(float x,float y,float yaw)
{origin_calls++;assert(x==2250 && y==150 && yaw==CHASSIS_MODEL_PI);if(origin_rejected)return false;
 localization.origin=(ChassisModel_Pose_t){x,y,yaw};localization.observer.feedback=localization.origin;
 localization.origin_valid=true;localization.feedback_valid=false;return true;}
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<sizeof report);memcpy(report,d,n);report[n]=0;return true;}
bool ChassisRoute_IsBusy(void){return !strcmp(route.state,"running");}
bool ChassisRoute_StationReserved(void){return !strcmp(route.state,"work") || !strcmp(route.state,"station");}
bool ChassisRoute_TaskStart(const RadarPlan_t *p,float s){(void)p;assert(s==200);if(reject_route){reject_route--;route.state="error";route.reason="busy";return false;}starts++;route.state="running";route.reason="move";route.phase="move";return true;}
bool ChassisRoute_TaskStartEx(const RadarPlan_t *p,float speed,const ChassisRoute_TaskOptions_t *o)
{route_start=o->start_id;route_planned=p!=NULL;assert(o->scan_qr && o->turn_speed_deg_s==50);return ChassisRoute_TaskStart(p,speed);}
bool ChassisRoute_TaskPause(void){if(strcmp(route.state,"paused"))paused_route_state=route.state;route.state="paused";route.stop_confirmed=true;return true;}
bool ChassisRoute_TaskContinue(void){route.state=paused_route_state?paused_route_state:"running";return true;}
void ChassisRoute_TaskRecoveryHold(bool hold){(void)hold;}
bool ChassisRoute_TaskResumeReady(void){return !route_resume_delayed;}
bool ChassisRoute_StationFinish(void){assert(!strcmp(route.state,"station") && route.station==4 && route.visit==2 && route.stop_confirmed);route.state="done";return true;}
bool ChassisRoute_WorkBegin(void){assert(!strcmp(route.state,"station") && route.stop_confirmed);route.state="work";return true;}
bool ChassisRoute_WorkResume(void){assert(!strcmp(route.state,"work"));route.state="running";route.phase="move";resumes++;return true;}
bool ChassisRoute_Cancel(void){cancels++;route.state="cancelled";return true;}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out){*out=route;}
bool ChassisMotion_IsBusy(void){return motion.state==CHASSIS_MOTION_RUNNING;}
bool ChassisMotion_MoveTo(float x,float y,float a,float s,uint32_t t){assert(s<=50 && t);(void)a;if(reject_move){motion.reason=reject_move;return false;}tx=x;ty=y;motion.state=CHASSIS_MOTION_RUNNING;return true;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out){*out=motion;out->heading_valid=!start_imu_missing;out->heading_sample_tick=tick;}
bool GrabTask_IsBusy(void){return grab.state==GRAB_ROUTE_POSE || grab.state==GRAB_PREPARE || grab.state==GRAB_HOLD;}
void GrabTask_StatusGet(GrabTask_Status_t *out){*out=grab;}
void GrabTask_Stop(void){grab.state=GRAB_IDLE;grab.result="cancelled";grab.stop_confirmed=true;}
void GrabTask_Cancel(void){grab_resume_pending=false;GrabTask_Stop();}
bool ChassisMotion_Stop(uint32_t ms){assert(!ms);motion.state=cancel_defer?CHASSIS_MOTION_STOPPING:CHASSIS_MOTION_DONE;motion.stop_confirmed=!cancel_defer;return true;}
bool ChassisMotion_StopRequest(uint32_t token){assert(!token);cancel_stop.id++;cancel_stop.requested_ms=tick;
 cancel_stop.tx_complete=cancel_stop.wheels_stopped=!cancel_defer;cancel_stop.reason=cancel_defer?"monitoring":"stopped";
 cancel_stop_pending=cancel_defer;if(stop_request_tick_advance)tick++;return ChassisMotion_Stop(0);}
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *out){*out=cancel_stop;}
bool ChassisMotion_StopPending(void){return cancel_stop_pending;}
bool GrabTask_QrPoseStart(void){qr_pose_calls++;grab.state=GRAB_COMPLETE;grab.result="qr_pose_ready";grab.stop_confirmed=true;return true;}
bool GrabTask_PoseStart(bool work){assert(!GrabTask_IsBusy());if(reject_pose){reject_pose--;grab.reason="busy";return false;}pose_work=work;pose_calls++;grab.state=GRAB_ROUTE_POSE;grab.result="pending";return true;}
bool GrabTask_PoseClearanceReady(void){return pose_clearance && grab.state==GRAB_ROUTE_POSE && !grab_resume_pending;}
bool GrabTask_StoreFinishTravel(void){assert(grab.state==GRAB_PREPARE && !strcmp(grab.operation,"store"));store_travel_calls++;return true;}
bool GrabTask_TransferStart(const char *op,const char *scene,uint8_t slot,uint8_t color)
{
  assert(!GrabTask_IsBusy() && slot>=1 && slot<=3);
  if(reject_transfer){grab.reason="config_missing";grab.missing="z_proc_place";return false;}
  if(!strcmp(scene,"raw")) {raw++;assert(color==(single_test?3:(raw<=3?(uint8_t[]){1,5,6}:(uint8_t[]){5,1,6})[(raw-1)%3]));}
  else if(!strcmp(scene,"proc")) proc++;
  else {place++;if(!strcmp(scene,"rough")) assert((tx==(float[]){1350,1200,1050,1200,1050,1350}[(place<=3?place-1:place-4)%6]));
    if(!strcmp(scene,"temp2")) assert((ty==(float[]){1200,1050,1350}[place-10]));}
  if(!strcmp(scene,"proc")) assert(place==3 || place==9); /* whole batch already released */
  grab.state=GRAB_PREPARE;grab.slot=slot;grab.operation=op;grab.scene=scene;grab.result="pending";
  inventory[slot-1].color=color;transfers++;return true;
}
bool Turntable_ReferenceValid(void){return true;}
bool Turntable_IsBusy(void){return false;}
bool Turntable_SlotConfigured(uint8_t s){return s>=1 && s<=3;}
Turntable_Inventory_t Turntable_InventoryGet(uint8_t s){return inventory[s-1];}
bool Turntable_InventorySet(uint8_t s,Turntable_InventoryState_t state,uint8_t color){inventory[s-1]=(Turntable_Inventory_t){state,color};return true;}
uint8_t ContestScreen_TaskCodeRefresh(const char *s){assert(strlen(s)==15);strcpy(screen_code,s);screen_refreshes++;return 1;}
void ContestScreen_TaskCodeClear(void){screen_code[0]=0;}
uint8_t ContestScreen_TaskCodeSent(void){return screen_sent;}
uint8_t ContestScreen_HasError(void){return screen_error;}
bool GrabTask_QrReady(float *y){*y=1250;return true;}
bool GrabTask_QrScanStart(uint32_t baseline){qr_baseline=baseline;qr_scanning=true;grab.state=GRAB_QR_SCAN;return true;}
bool GrabTask_Pause(void){if(grab_resume_pending){if(grab.state==GRAB_ERROR)GrabTask_Stop();return true;}
 if(!GrabTask_IsBusy())return false;
 paused_grab=grab;grab_resume_pending=true;GrabTask_Stop();return true;}
bool GrabTask_Continue(void){grab_resume_calls++;grab=paused_grab;grab_resume_pending=false;return true;}
bool GrabTask_BusRecoveryVerify(void){grab.state=GRAB_IDLE;grab.stop_confirmed=true;return true;}
bool GrabTask_TrackedStoreStart(uint8_t color,uint8_t slot,GrabTracking_Mode_t mode)
{assert(mode==GRAB_TRACK_FEEDBACK);return GrabTask_TransferStart("store","raw",slot,color);}
bool GrabTask_ProcessedStoreStart(uint8_t color,uint8_t ring,uint8_t slot)
{assert(ring>=1 && ring<=3);return GrabTask_TransferStart("store","proc",slot,color);}
bool GrabTask_CarPickStart(uint8_t slot)
{if(route.station==2 || route.station==3){assert(ring_captured);if(route.station==2)rough_picks++;}
 grab.state=GRAB_COMPLETE;grab.slot=slot;grab.result="picked";return true;}
bool GrabTask_AlignPlaceStart(const char *scene,uint8_t ring)
{assert((!strcmp(scene,"rough") || !strcmp(scene,"temp1") || !strcmp(scene,"temp2")) && ring>=1 && ring<=(!strcmp(scene,"temp2")?6:3));
 if(reject_transfer){grab.reason="config_missing";grab.missing="z_proc_place";return false;}
 ring_captured=false;ring_aligns++;grab.state=GRAB_PREPARE;grab.operation=!strcmp(scene,"temp2")?"empty_material_align":"empty_ring_align";grab.result="pending";return true;}
bool GrabTask_AlignPlaceCapture(void)
{assert(grab.state==GRAB_HOLD && !strcmp(grab.result,"aligned"));if(alignment_blocked)return false;
 ring_captured=true;ring_captures++;grab.state=GRAB_COMPLETE;return true;}
bool GrabTask_AlignedPlaceStart(const char *scene,uint8_t ring,uint8_t color)
{assert(ring_captured && ring>=1 && ring<=(!strcmp(scene,"temp2")?6:3));return GrabTask_TransferStart("take",scene,grab.slot,color);}
bool GrabTask_HeldPlaceStart(const char *scene,uint8_t target,uint8_t color)
{assert(target>=1 && target<=6);return GrabTask_TransferStart("take",scene,grab.slot,color);}
bool GrabTask_ProcessedStoreStartEx(uint8_t color,uint8_t ring,uint8_t slot,bool raised)
{(void)raised;return GrabTask_ProcessedStoreStart(color,ring,slot);}
bool GrabTask_AlignPlaceStartEx(const char *scene,uint8_t ring,bool raised)
{(void)raised;return GrabTask_AlignPlaceStart(scene,ring);}
bool GrabTask_HeldPlaceStartEx(const char *scene,uint8_t target,uint8_t color,bool raised)
{(void)raised;return GrabTask_HeldPlaceStart(scene,target,color);}
uint8_t ContestScreen_ProgressSet(uint8_t a,uint8_t b){assert(a<=12 && b<=12);return 1;}
static void step(void)
{
  tick+=100;
  if(qr_scanning){QR_SnapshotTypeDef q;QR_SnapshotGet(&q);if(q.Source==QR_SOURCE_UART && q.Sequence!=qr_baseline){qr_scanning=false;grab.state=GRAB_IDLE;grab.result="qr_scanned";grab.stop_confirmed=true;}}
  if(grab.state==GRAB_ROUTE_POSE){grab.state=GRAB_COMPLETE;grab.result=pose_work?"work_ready":"travel_ready";}
  else if(grab.state==GRAB_PREPARE){
    if(!strcmp(grab.operation,"empty_ring_align") || !strcmp(grab.operation,"empty_material_align")){grab.state=GRAB_HOLD;grab.result="aligned";grab.stop_confirmed=true;}
    else {grab.state=GRAB_COMPLETE;grab.result=!strcmp(grab.operation,"store")?"stored":"released";inventory[grab.slot-1].state=!strcmp(grab.operation,"store")?TURNTABLE_OCCUPIED:TURNTABLE_EMPTY;}
  }
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
  ContestMission_Process();
}
static void scan_station(void)
{
 route.state="station";route.station=5;route.visit=1;route.stop_confirmed=true;route.target_x_mm=2100;route.target_y_mm=1250;
 for(unsigned i=0;i<5;i++)step();
 assert(qr_scanning && !raw);
 assert(QR_SimulatedSet("356+123+536+231"));step();assert(qr_scanning);
 const char *text="356+123+536+231\r";for(;*text;text++)QR_ReceiveData((uint8_t)*text);
 unsigned old=resumes;for(unsigned i=0;i<15 && resumes==old;i++)step();assert(resumes==old+1);
 assert(!strcmp(screen_code,"356+123+536+231"));ContestMission_Report();
 assert(strstr(report,"exec_code=156+123+516+231") && strstr(report,"code_source=fixed"));
}
static void station(unsigned id,unsigned visit)
{
  route.state="running";route.phase="work_turn";ContestMission_Process();step();
  route.state="station";route.station=id;route.visit=visit;route.stop_confirmed=true;
  route.target_x_mm=id==3?350:1200;route.target_y_mm=id==4?2100:id==2?350:1200;
  unsigned before=resumes;
  for(unsigned i=0;i<100 && resumes==before;i++) step();
  assert(resumes==before+1);
}
int main(void)
{
  route.state="idle";route.phase="idle";route.reason="idle";grab.reason="idle";grab.result="none";
  for(unsigned i=0;i<3;i++)inventory[i].state=TURNTABLE_EMPTY;
  ContestMission_Init();request.profile=(ChassisMotion_Profile_t)2;
  unsigned poses_before=pose_calls;
  assert(!ContestMission_Start(NULL,&request) && !strcmp(ContestMission_Reason(),"profile_required") && pose_calls==poses_before);
  request.profile=CHASSIS_PROFILE_COSINE;
  QR_Init();ContestMission_Init();assert(ContestMission_Start(NULL,&request));assert(!starts);
  assert(selected_profile==CHASSIS_PROFILE_COSINE && strstr(report,"profile=cosine"));
  request.profile=CHASSIS_PROFILE_LINEAR;ContestMission_Report();assert(strstr(report,"profile=cosine"));
  step();assert(starts==1);scan_station();
  station(4,2);assert(raw==3 && transfers==3);
  /* Missing scene height waits at the same item. Transient reads/feedback gaps
   * retry without consuming a transfer or repeating an already stored piece. */
  route.phase="work_turn";ContestMission_Process();step();
  route.state="station";route.station=2;route.visit=3;route.stop_confirmed=true;
  route.target_x_mm=1200;route.target_y_mm=350;
  reject_transfer=true;reject_pose=2;reject_move="feedback_invalid";
  for(unsigned i=0;i<10;i++)step();
  assert(transfers==3 && ContestMission_IsBusy());
  reject_move=NULL;
  for(unsigned i=0;i<10 && !ContestMission_CanConfigure();i++)step();
  assert(ContestMission_CanConfigure() && transfers==3);
  ContestMission_Report();assert(strstr(report,"item=1") && strstr(report,"reason=z_proc_place"));
  reject_transfer=false;selected_profile=CHASSIS_PROFILE_LINEAR;
  assert(ContestMission_Continue() && selected_profile==CHASSIS_PROFILE_COSINE);
  alignment_blocked=true;
  for(unsigned i=0;i<12;i++)step();
  assert(ring_aligns==1 && !ring_captures && !rough_picks && !place && transfers==3);
  alignment_blocked=false;
  unsigned before=resumes;
  for(unsigned i=0;i<100 && resumes==before;i++)step();
  assert(resumes==before+1);
  assert(proc==3 && place==3 && transfers==9);
  station(3,4);assert(place==6);
  station(4,5);assert(raw==6);
  station(2,6);assert(proc==6);
  assert(ring_aligns==9 && ring_captures==9 && rough_picks==6);
  station(3,7);assert(place==12 && transfers==24);
  assert(ring_aligns==12 && ring_captures==12 && rough_picks==6);
  route.state="done";ContestMission_Process();assert(!ContestMission_IsBusy());
  ContestMission_Report();assert(strstr(report,"mission_complete") && strstr(report,"stored=12") && strstr(report,"placed=12"));
  request.profile=CHASSIS_PROFILE_COSINE;
  ContestMission_Init();route.state="idle";assert(ContestMission_Start(NULL,&request));
  ContestMission_Stop();grab.state=GRAB_ERROR;grab.reason="stop_unconfirmed";step();
  assert(ContestMission_CanConfigure());assert(ContestMission_HasPendingWork());assert(ContestMission_Continue());ContestMission_Stop();step();
  ContestMission_Report();assert(strstr(report,"state=paused"));
  assert(ContestMission_HasPendingWork());
  /* Reset only for an explicitly new test fixture. */
  grab.state=GRAB_IDLE;route.state="idle";
  /* Single test ignores QR ordering, requires only slot1, and must never
   * resume towards processing or start a second pickup. */
  ContestMission_Init();route.state="idle";grab.state=GRAB_IDLE;
  inventory[0].state=TURNTABLE_EMPTY;inventory[1].state=inventory[2].state=TURNTABLE_UNKNOWN;
  single_test=true;raw=transfers=0;unsigned old_resumes=resumes;
  assert(QR_SimulatedSet("356+123+536+231"));
  assert(ContestMission_StartSingle(200));step();
  ContestMission_Report();assert(selected_profile==CHASSIS_PROFILE_LINEAR && strstr(report,"profile=linear"));
  route.phase="work_turn";ContestMission_Process();step();
  route.state="station";route.station=4;route.visit=2;route.stop_confirmed=true;
  route.target_x_mm=1200;route.target_y_mm=2100;
  for(unsigned i=0;i<100 && ContestMission_IsBusy();i++)step();
  assert(!ContestMission_IsBusy() && transfers==1 && raw==1 && resumes==old_resumes);
  assert(!pose_work && grab.state==GRAB_COMPLETE && !strcmp(grab.result,"travel_ready"));
  assert(inventory[0].state==TURNTABLE_OCCUPIED && inventory[0].color==3);
  QR_SnapshotTypeDef kept;QR_SnapshotGet(&kept);assert(!strcmp(kept.Code,"356+123+536+231"));
  ContestMission_Report();assert(strstr(report,"mode=single") && strstr(report,"stored=1"));
  assert(!ContestMission_StartSingle(200));assert(!strcmp(ContestMission_Reason(),"first_slot_occupied"));
  inventory[0].state=TURNTABLE_UNKNOWN;
  assert(!ContestMission_StartSingle(200));assert(!strcmp(ContestMission_Reason(),"first_slot_inventory_unknown"));
  puts("contest_mission_test: PASS (two batches, all placed before takeback, color-matched stack, stop)");
  return 0;
}
