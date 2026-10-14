#include "mission_navigation.h"
void MissionNavigation_Reset(void){}
void MissionNavigation_Pause(void){}
int MissionNavigation_Process(RadarStart_Id_t s,uint32_t g,RadarPlan_t *p,const char **r){(void)s;(void)g;(void)p;*r="radar_scanning";return 0;}
#define main legacy_motion_main
#include "chassis_motion_test.c"
#undef main
#include "contest_mission.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "turntable.h"
#include "QR.h"
#include "motor_bus.h"
static GrabTask_Status_t child;
static ChassisRoute_Status_t travel;
static char report[512];
bool Mecanum_Feedback_Select(uint8_t n){assert(!n);return true;}
void Mecanum_Feedback_Enable(bool on){(void)on;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<sizeof report);memcpy(report,d,n);report[n]=0;return true;}
bool ChassisLocalization_Origin(float x,float y,float yaw){location.origin=(ChassisModel_Pose_t){x,y,yaw};location.origin_valid=true;return ChassisMotion_AnchorSet(yaw);}
bool ChassisRoute_StationReserved(void){return route_busy;}
bool ChassisRoute_TaskStartEx(const RadarPlan_t *p,float s,const ChassisRoute_TaskOptions_t *o)
{(void)p;(void)s;(void)o;expected_acc=20;bool ok=ChassisMotion_VelocityStart(20,0,0);route_busy=ok;travel.state="running";return ok;}
bool ChassisRoute_TaskStart(const RadarPlan_t *p,float s){return ChassisRoute_TaskStartEx(p,s,NULL);}
bool ChassisRoute_Cancel(void){route_busy=false;travel.state="cancelled";return ChassisMotion_Stop(0);}
bool ChassisRoute_TaskPause(void){return false;}
bool ChassisRoute_TaskContinue(void){return false;}
bool ChassisRoute_TaskResumeReady(void){return false;}
void ChassisRoute_TaskRecoveryHold(bool hold){(void)hold;}
bool Turntable_IsBusy(void){return false;}
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *s){memset(s,0,sizeof *s);}
bool MotorBus_RecoveryStart(void){return false;}
void MotorBus_RecoveryGet(MotorBus_Recovery_t *s){memset(s,0,sizeof *s);}
void MotorBus_RecoveryCancel(void){}
bool Mecanum_RecoveryAfterBusCheck(void){return false;}
bool ChassisRoute_StationFinish(void){return false;}
bool ChassisRoute_WorkBegin(void){return false;}
bool ChassisRoute_WorkResume(void){return false;}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out){*out=travel;}
void GrabTask_StatusGet(GrabTask_Status_t *out){*out=child;}
bool GrabTask_QrPoseStart(void){return false;}
bool GrabTask_PoseClearanceReady(void){return false;}
bool GrabTask_StoreFinishTravel(void){return false;}
bool GrabTask_PoseStart(bool work){(void)work;child.state=GRAB_COMPLETE;child.result="travel_ready";return true;}
void GrabTask_Stop(void){child.state=GRAB_IDLE;child.stop_confirmed=true;}
void GrabTask_Cancel(void){GrabTask_Stop();}
bool GrabTask_Pause(void){return false;}
bool GrabTask_Continue(void){return false;}
bool GrabTask_BusRecoveryVerify(void){return false;}
bool GrabTask_QrReady(float *y){*y=1250;return true;}
bool GrabTask_QrScanStart(uint32_t n){(void)n;return false;}
bool GrabTask_TransferStart(const char *a,const char *b,uint8_t c,uint8_t d){(void)a;(void)b;(void)c;(void)d;return false;}
bool GrabTask_TrackedStoreStart(uint8_t a,uint8_t b,GrabTracking_Mode_t c){(void)a;(void)b;(void)c;return false;}
bool GrabTask_ProcessedStoreStart(uint8_t a,uint8_t b,uint8_t c){(void)a;(void)b;(void)c;return false;}
bool GrabTask_CarPickStart(uint8_t s){(void)s;return false;}
bool GrabTask_HeldPlaceStart(const char *s,uint8_t a,uint8_t b){(void)s;(void)a;(void)b;return false;}
bool GrabTask_AlignPlaceStart(const char *s,uint8_t a){(void)s;(void)a;return false;}
bool GrabTask_ProcessedStoreStartEx(uint8_t a,uint8_t b,uint8_t c,bool raised)
{(void)raised;return GrabTask_ProcessedStoreStart(a,b,c);}
bool GrabTask_HeldPlaceStartEx(const char *s,uint8_t a,uint8_t b,bool raised)
{(void)raised;return GrabTask_HeldPlaceStart(s,a,b);}
bool GrabTask_AlignPlaceStartEx(const char *s,uint8_t a,bool raised)
{(void)raised;return GrabTask_AlignPlaceStart(s,a);}
bool GrabTask_AlignPlaceCapture(void){return false;}
bool GrabTask_AlignedPlaceStart(const char *s,uint8_t a,uint8_t b){(void)s;(void)a;(void)b;return false;}
bool Turntable_ReferenceValid(void){return true;}
bool Turntable_SlotConfigured(uint8_t s){(void)s;return true;}
Turntable_Inventory_t Turntable_InventoryGet(uint8_t s){(void)s;return (Turntable_Inventory_t){TURNTABLE_EMPTY,0};}
bool Turntable_InventorySet(uint8_t s,Turntable_InventoryState_t t,uint8_t c){(void)s;(void)t;(void)c;return true;}
void ContestScreen_TaskCodeClear(void){}
uint8_t ContestScreen_TaskCodeRefresh(const char *s){(void)s;return 1;}
uint8_t ContestScreen_TaskCodeSent(void){return 1;}
uint8_t ContestScreen_HasError(void){return 0;}
uint8_t ContestScreen_ProgressSet(uint8_t a,uint8_t b){(void)a;(void)b;return 1;}
static void sample(uint32_t now,bool tx)
{
 tick=now;if(tx){bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;bus.tx_complete=true;bus.sent_ms=now;}
 else for(unsigned i=0;i<4;i++){location.wheels[i].speed_ms=location.wheels[i].position_ms=now;location.wheels[i].speed_rpm=0;}
 ChassisMotion_Process();ContestMission_Process();ContestMission_Report();
}
int main(void)
{
 for(unsigned prep=0;prep<2;prep++){
  reset();valid=true;reading(100,0);ChassisMotion_HeadingProcess();assert(ChassisMotion_AnchorSet(CHASSIS_MODEL_PI));
  location.origin_valid=location.feedback_valid=location.speed_valid=true;location.feedback_tick=tick;
  location.origin=(ChassisModel_Pose_t){2250,150,CHASSIS_MODEL_PI};location.observer.feedback=location.origin;
  for(unsigned i=0;i<4;i++){location.wheels[i]=(Mecanum_Feedback_t){0};location.wheels[i].speed_valid=location.wheels[i].position_valid=location.wheels[i].state_valid=true;
   location.wheels[i].speed_ms=location.wheels[i].position_ms=location.wheels[i].state_ms=tick;location.wheels[i].state_flags=3;location.wheels[i].speed_rpm=prep?3:0;}
  memset(&child,0,sizeof child);child.reason="idle";travel.state=travel.phase=travel.reason="idle";
  QR_Init();ContestMission_Init();ContestMission_Request_t r={MISSION_CODE_QR,"",200,50,RADAR_START_LOWER,false,false,false,CHASSIS_PROFILE_LINEAR};assert(ContestMission_Start(NULL,&r));ContestMission_Process();
  if(prep)assert(!strcmp(ContestMission_Reason(),"start_wheels_moving"));else assert(ChassisMotion_IsBusy());
  ContestMission_Cancel();ContestMission_Process();assert(ChassisMotion_StopPending());
  sample(110,true);assert(!status().stop_confirmed && ContestMission_HasPendingWork());
  sample(120,false);sample(370,false);assert(ContestMission_HasPendingWork());sample(620,false);
  assert(!status().stop_confirmed && !ContestMission_HasPendingWork() && !ContestMission_Continue());
  assert(strstr(report,"state=cancelled") && strstr(report,"stop_confirmed=1"));
 }
 puts("production motion + mission: cancellation and pre-departure stop await physical feedback PASS");
}
