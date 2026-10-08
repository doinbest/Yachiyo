#include "contest_mission.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "QR.h"
#include "turntable.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static ChassisRoute_Status_t route;
static ChassisMotion_Status_t motion;
static GrabTask_Status_t grab;
static Turntable_Inventory_t inventory[3];
static unsigned transfers,raw,place,proc,resumes,starts,pose_calls,cancels;
static bool pose_work,reject_transfer,single_test;
static unsigned reject_route,reject_pose;
static const char *reject_move;
static char report[512];
static float tx,ty;
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<sizeof report);memcpy(report,d,n);report[n]=0;return true;}
bool ChassisRoute_IsBusy(void){return !strcmp(route.state,"running");}
bool ChassisRoute_StationReserved(void){return !strcmp(route.state,"work") || !strcmp(route.state,"station");}
bool ChassisRoute_TaskStart(const RadarPlan_t *p,float s){(void)p;assert(s==200);if(reject_route){reject_route--;route.state="error";route.reason="busy";return false;}starts++;route.state="running";route.reason="move";route.phase="move";return true;}
bool ChassisRoute_StationFinish(void){assert(!strcmp(route.state,"station") && route.station==4 && route.visit==2 && route.stop_confirmed);route.state="done";return true;}
bool ChassisRoute_WorkBegin(void){assert(!strcmp(route.state,"station") && route.stop_confirmed);route.state="work";return true;}
bool ChassisRoute_WorkResume(void){assert(!strcmp(route.state,"work"));route.state="running";route.phase="move";resumes++;return true;}
bool ChassisRoute_Cancel(void){cancels++;route.state="cancelled";return true;}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out){*out=route;}
bool ChassisMotion_IsBusy(void){return motion.state==CHASSIS_MOTION_RUNNING;}
bool ChassisMotion_MoveTo(float x,float y,float a,float s,uint32_t t){assert(s<=50 && t);(void)a;if(reject_move){motion.reason=reject_move;return false;}tx=x;ty=y;motion.state=CHASSIS_MOTION_RUNNING;return true;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out){*out=motion;}
bool GrabTask_IsBusy(void){return grab.state==GRAB_ROUTE_POSE || grab.state==GRAB_PREPARE;}
void GrabTask_StatusGet(GrabTask_Status_t *out){*out=grab;}
void GrabTask_Stop(void){grab.state=GRAB_IDLE;grab.result="cancelled";}
bool GrabTask_PoseStart(bool work){assert(!GrabTask_IsBusy());if(reject_pose){reject_pose--;grab.reason="busy";return false;}pose_work=work;pose_calls++;grab.state=GRAB_ROUTE_POSE;grab.result="pending";return true;}
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
bool Turntable_SlotConfigured(uint8_t s){return s>=1 && s<=3;}
Turntable_Inventory_t Turntable_InventoryGet(uint8_t s){return inventory[s-1];}
uint8_t ContestScreen_TaskCodeRefresh(const char *s){assert(strlen(s)==15);return 1;}
uint8_t ContestScreen_ProgressSet(uint8_t a,uint8_t b){assert(a<=12 && b<=12);return 1;}
static void step(void)
{
  tick+=100;
  if(grab.state==GRAB_ROUTE_POSE){grab.state=GRAB_COMPLETE;grab.result=pose_work?"work_ready":"travel_ready";}
  else if(grab.state==GRAB_PREPARE){grab.state=GRAB_COMPLETE;grab.result=!strcmp(grab.operation,"store")?"stored":"released";inventory[grab.slot-1].state=!strcmp(grab.operation,"store")?TURNTABLE_OCCUPIED:TURNTABLE_EMPTY;}
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
  ContestMission_Process();
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
  QR_Init();ContestMission_Init();assert(ContestMission_Start(NULL,200));assert(!starts);step();assert(starts==1);
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
  reject_transfer=false;assert(ContestMission_Continue());
  unsigned before=resumes;
  for(unsigned i=0;i<100 && resumes==before;i++)step();
  assert(resumes==before+1);
  assert(proc==3 && place==3 && transfers==9);
  station(3,4);assert(place==6);
  station(4,5);assert(raw==6);
  station(2,6);assert(proc==6);
  station(3,7);assert(place==12 && transfers==24);
  route.state="done";ContestMission_Process();assert(!ContestMission_IsBusy());
  ContestMission_Report();assert(strstr(report,"mission_complete") && strstr(report,"stored=12") && strstr(report,"placed=12"));
  ContestMission_Init();route.state="idle";assert(ContestMission_Start(NULL,200));step();
  ContestMission_Stop();step();assert(cancels && !ContestMission_IsBusy() && raw==6);
  /* Restart from a cancelled route, with a bus read still draining. */
  assert(ContestMission_Start(NULL,200));reject_route=1;step();
  assert(ContestMission_IsBusy() && !strcmp(ContestMission_Reason(),"route_bus_wait"));
  step();assert(ContestMission_IsBusy());
  route.state="done";ContestMission_Process();assert(!ContestMission_IsBusy());
  assert(!strcmp(ContestMission_Reason(),"mission_incomplete"));
  /* Single test ignores QR ordering, requires only slot1, and must never
   * resume towards processing or start a second pickup. */
  ContestMission_Init();route.state="idle";grab.state=GRAB_IDLE;
  inventory[0].state=TURNTABLE_EMPTY;inventory[1].state=inventory[2].state=TURNTABLE_UNKNOWN;
  single_test=true;raw=transfers=0;unsigned old_resumes=resumes;
  assert(QR_SimulatedSet("356+123+536+231"));
  assert(ContestMission_StartSingle(200));step();
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
}
