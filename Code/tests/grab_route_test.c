#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "GrabRoute.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_config.h"
#include "chassis_localization.h"
#include "contest_screen.h"
#include "QR.h"
#include "console_tx.h"
static uint32_t tick;
static ChassisRoute_Status_t route;
static unsigned starts,resumes,finishes,picks,cancels;
static int ready=1,screen_done,screen_error,task_busy;
static GrabTask_Status_t grab_status;
static char reply[600];
static float route_speed;
static ChassisLocalization_Status_t localization;
static bool feedback_enabled;
static unsigned feedback_selections;
void ChassisLocalization_Get(ChassisLocalization_Status_t *s){*s=localization;}
bool Mecanum_Feedback_Select(uint8_t address){assert(address==0);feedback_selections++;feedback_enabled=false;return true;}
void Mecanum_Feedback_Enable(bool on){feedback_enabled=on;}
bool GrabTask_QrReady(float *y){*y=1200;return ready;}
bool GrabTask_QrScanStart(uint32_t seq){(void)seq;task_busy=1;grab_status.state=GRAB_QR_SCAN;grab_status.reason="qr_wait";return true;}
bool ChassisRoute_QrTestStart(float speed,float y){assert(y==1200);route_speed=speed;route.state="running";starts++;return true;}
static unsigned single_starts;
static bool single_accept=true;
bool ContestMission_StartSingle(float speed){single_starts++;route_speed=speed;return single_accept;}
static bool legacy(void){RadarPlan_t p={0};p.valid=true;return GrabRoute_StartPlanned(&p,50);}
void ContestMission_Init(void){}
bool ContestMission_Start(const RadarPlan_t *p,float s){(void)p;(void)s;return false;}
void ContestMission_Process(void){}
void ContestMission_Stop(void){}
bool ContestMission_IsBusy(void){return false;}
bool ContestMission_HasRun(void){return false;}
const char *ContestMission_Reason(void){return "first_slot_inventory_unknown";}
bool ContestMission_Continue(void){return false;}
bool ContestMission_CanConfigure(void){return false;}
void ContestMission_Report(void){}
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<sizeof(reply));memcpy(reply,d,n);reply[n]=0;return true;}
bool GrabTask_ConfigReady(const char *mode){assert(!strcmp(mode,"pick"));return ready;}
bool GrabTask_IsBusy(void){return task_busy;}
bool GrabTask_Start(const char *mode){assert(!strcmp(mode,"pick"));picks++;task_busy=1;grab_status.state=GRAB_PREPARE;grab_status.result="pending";grab_status.reason="running";return true;}
void GrabTask_Stop(void){task_busy=0;grab_status.state=GRAB_IDLE;grab_status.result="cancelled";grab_status.reason="cancelled";}
void GrabTask_StatusGet(GrabTask_Status_t *s){*s=grab_status;}
bool ChassisRoute_PlanStart(const RadarPlan_t *p,float speed,bool station_mode){assert(p->valid && speed==50 && station_mode);starts++;route.planned=true;route.segment=1;route.state="running";return true;}
bool ChassisRoute_StationResume(void){resumes++;route.segment=3;route.state="running";return true;}
bool ChassisRoute_StationFinish(void){finishes++;route.state="done";return true;}
bool ChassisRoute_Cancel(void){cancels++;route.state="cancelled";return true;}
bool ChassisRoute_IsBusy(void){return !strcmp(route.state,"running");}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *s){*s=route;}
uint8_t ContestScreen_TaskCodeRefresh(const char *s){assert(!strcmp(s,"156+123+516+231"));return 1;}
uint8_t ContestScreen_ProgressSet(uint8_t a,uint8_t b){assert(!a&&!b);return 1;}
uint8_t ContestScreen_TaskCodeSent(void){return screen_done;}
uint8_t ContestScreen_HasError(void){return screen_error;}
static void setup(void){memset(&route,0,sizeof(route));memset(&grab_status,0,sizeof(grab_status));route.state="idle";route.reason="none";grab_status.reason="idle";grab_status.result="none";tick=0;starts=resumes=finishes=picks=cancels=single_starts=0;ready=1;screen_done=screen_error=task_busy=0;QR_Init();GrabRoute_Init();}
int main(void)
{
  setup();char *qrstart[]={"grab","qr","begin"};
  localization.origin_valid=true;localization.origin.x_mm=2250;localization.origin.y_mm=2250;
  localization.origin.yaw_rad=-CHASSIS_MODEL_PI/2;
  assert(GrabRoute_Command(3,qrstart));assert(!starts && GrabRoute_IsBusy());
  assert(strstr(reply,"state=preparing"));
  GrabRoute_Process();assert(feedback_enabled && feedback_selections==1 && !starts);
  for(unsigned i=0;i<20;i++){tick+=100;GrabRoute_Process();}
  assert(!starts && !task_busy); /* Never move on missing position/speed feedback. */
  localization.feedback_valid=localization.speed_valid=true;localization.feedback_tick=tick;
  GrabRoute_Process();assert(starts==1 && route_speed==50);
  GrabRoute_Process();assert(!task_busy);
  route.state="done";route.stop_confirmed=false;GrabRoute_Process();assert(!task_busy);
  route.stop_confirmed=true;GrabRoute_Process();assert(task_busy && GrabRoute_IsBusy());
  char *qrquery[]={"grab","qr","status"};assert(GrabRoute_Command(3,qrquery));assert(strstr(reply,"state=scan"));
  task_busy=0;grab_status.state=GRAB_IDLE;grab_status.stop_confirmed=true;grab_status.result="qr_scanned";
  GrabRoute_Process();assert(!GrabRoute_IsBusy());assert(strstr(reply,"state=done"));
  setup();assert(GrabRoute_Command(3,qrstart));GrabRoute_Stop();GrabRoute_Process();
  assert(!GrabRoute_IsBusy() && strstr(reply,"state=cancelled") && !picks && !single_starts);
  setup();localization.origin_valid=false;assert(GrabRoute_Command(3,qrstart));
  assert(!GrabRoute_IsBusy() && !starts && strstr(reply,"origin_required"));
  setup();char *lowerstart[]={"grab","qr","begin","lower","60"};
  localization.origin_valid=true;localization.origin.x_mm=2250;localization.origin.y_mm=150;
  localization.origin.yaw_rad=CHASSIS_MODEL_PI;
  localization.feedback_valid=localization.speed_valid=true;localization.feedback_tick=tick;
  assert(GrabRoute_Command(5,lowerstart));
  assert(strstr(reply,"state=preparing") && !starts);
  GrabRoute_Process();assert(starts==1 && route_speed==60);
  route.reason="qr_scan_turn";route.phase="qr_scan_turn";
  GrabRoute_Process();assert(!task_busy && strstr(reply,"reason=qr_scan_turn"));
  for(unsigned i=0;i<10;i++)GrabRoute_Process();
  assert(!task_busy && starts==1); /* Z waits for the chassis half-turn and stop. */
  route.state="done";route.stop_confirmed=false;GrabRoute_Process();assert(!task_busy);
  route.stop_confirmed=true;GrabRoute_Process();assert(task_busy);
  GrabRoute_Stop();GrabRoute_Process();assert(!GrabRoute_IsBusy());
  setup();localization.origin.y_mm=2250;localization.origin.yaw_rad=-CHASSIS_MODEL_PI/2;
  assert(GrabRoute_Command(5,lowerstart));
  assert(!starts && !GrabRoute_IsBusy() && strstr(reply,"origin_required"));
  setup();task_busy=1;route.state="running";
  char *query[]={"grab","route","status"};
  assert(GrabRoute_Command(3,query));assert(strstr(reply,"OK grab route state=idle"));
  assert(!single_starts && !starts && !cancels && task_busy);

  QR_SnapshotTypeDef qr;
  setup();char *default_start[]={"grab","route"};assert(GrabRoute_Command(2,default_start));
  assert(single_starts==1 && !starts && !picks && route_speed==CHASSIS_ROUTE_DEFAULT_SPEED_MM_S);
  char *blue_test[]={"grab","route","150"};assert(GrabRoute_Command(3,blue_test));assert(single_starts==2 && route_speed==150);
  single_accept=false;assert(GrabRoute_Command(3,blue_test));assert(strstr(reply,"mode=single") && strstr(reply,"source=test") && strstr(reply,"state=error") && strstr(reply,"first_slot_inventory_unknown"));single_accept=true;
  setup();assert(legacy());route.state="station";route.segment=2;route.stop_confirmed=true;
  GrabRoute_Process();tick=1000;GrabRoute_Process();QR_SnapshotGet(&qr);assert(!qr.Valid && !resumes);
  setup();ready=0;assert(!legacy());assert(!starts);
  setup();assert(legacy());assert(starts==1&&GrabRoute_IsBusy());
  route.state="stopping";route.reason="position_reached";GrabRoute_Process();
  assert(GrabRoute_IsBusy() && !cancels); /* normal arrival must await stop feedback */
  route.state="station";route.segment=1;route.station=5;route.visit=1;route.stop_confirmed=true;GrabRoute_Process();
  tick=999;GrabRoute_Process();QR_SnapshotGet(&qr);assert(!qr.Valid&&!resumes);
  tick=1000;GrabRoute_Process();QR_SnapshotGet(&qr);assert(qr.Valid&&qr.Source==QR_SOURCE_SIMULATED&&!qr.Received&&!qr.Accepted);
  assert(QR_ColorGet(0,0)==1&&!resumes);tick+=1000;GrabRoute_Process();assert(!resumes);
  screen_done=1;GrabRoute_Process();assert(resumes==1);
  route.state="station";route.segment=3;route.station=4;route.visit=2;GrabRoute_Process();assert(finishes==1&&picks==1&&GrabRoute_IsBusy());
  GrabRoute_Process();assert(picks==1&&GrabRoute_IsBusy());
  grab_status.state=GRAB_COMPLETE;grab_status.result="stored";grab_status.reason="stored";task_busy=0;
  GrabRoute_Process();assert(!GrabRoute_IsBusy()&&picks==1&&strstr(reply,"state=done")&&strstr(reply,"single_piece_stored"));
  setup();legacy();route.state="station";route.segment=1;route.station=5;route.visit=1;route.stop_confirmed=true;GrabRoute_Process();tick=1000;GrabRoute_Process();screen_error=1;GrabRoute_Process();assert(!resumes&&!picks&&!GrabRoute_IsBusy()&&cancels);
  setup();legacy();GrabRoute_Stop();tick=9999;GrabRoute_Process();assert(!picks&&!resumes&&!GrabRoute_IsBusy());
  setup();RadarPlan_t plan={0};plan.valid=true;
  assert(GrabRoute_StartPlanned(&plan,50));
  route.state="station";route.segment=7;route.station=5;route.visit=1;route.stop_confirmed=true;
  GrabRoute_Process();tick=1000;GrabRoute_Process();screen_done=1;GrabRoute_Process();
  assert(resumes==1);route.state="station";route.segment=12;route.station=4;route.visit=2;
  GrabRoute_Process();assert(finishes==1&&picks==1);
  puts("grab_route_test: PASS (fixed/planned semantic visits, 1000ms, screen TX, pick, cancel)");
}
