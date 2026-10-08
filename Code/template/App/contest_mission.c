#include "contest_mission.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include "turntable.h"
#include "QR.h"
#include "contest_screen.h"
#include "console_tx.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum {
  M_IDLE,M_PREPARE,M_DRIVING,M_RING_CLEAR,M_RING_MOVE,M_TRANSFER,
  M_LEAVE,M_CENTER,M_WAIT_CONFIG,M_DONE,M_ERROR,M_CANCELLED,
  M_CLEAR_REQUEST,M_REAR_REQUEST,M_TRANSFER_REQUEST
} MissionState;
static MissionState State;
static const char *Reason;
static uint32_t StartTick;
static char Code[QR_TASK_CODE_BUFFER_SIZE];
static uint8_t Colors[2][3],Rings[2][3],FirstRing[7];
static unsigned Batch,Item,Pass,Station,Visit,Stored,Placed;
static float Speed,CenterX,CenterY,WorkYaw;
static bool Planned,WorkReady,WorkStarted,HasRun,Single;
static uint8_t Source;
static RadarPlan_t Plan;

void ContestMission_Init(void)
{ State=M_IDLE;Reason="idle";HasRun=false;Single=false;Stored=Placed=Batch=Item=Pass=0; }
bool ContestMission_IsBusy(void)
{ return State!=M_IDLE && State!=M_DONE && State!=M_ERROR && State!=M_CANCELLED; }
bool ContestMission_HasRun(void) { return HasRun; }
const char *ContestMission_Reason(void) { return Reason; }
bool ContestMission_CanConfigure(void) { return State==M_WAIT_CONFIG; }
static void fail(const char *why)
{
  Reason=why;State=M_ERROR;(void)ChassisRoute_Cancel();GrabTask_Stop();ContestMission_Report();
}
void ContestMission_Stop(void)
{
  if(!ContestMission_IsBusy()) return;
  State=M_CANCELLED;Reason="operator_stop";
  (void)ChassisRoute_Cancel();GrabTask_Stop();ContestMission_Report();
}
void ContestMission_Report(void)
{
  static const char *names[]={"idle","preparing_travel","driving","ring_clear","ring_move",
    "transfer","leaving","return_center","config_wait","done","error","cancelled",
    "prepare_ring","prepare_departure","prepare_transfer"};
  char line[320];
  int n=snprintf(line,sizeof line,"%s grab route state=%s mode=%s source=%s code=%s color=%u slot=%u batch=%u item=%u station=%u visit=%u stored=%u placed=%u elapsed_ms=%lu reason=%s\r\n",
    State==M_ERROR?"ERR":"OK",names[State],Single?"single":"mission",Single?"test":Source==QR_SOURCE_UART?"uart":"simulated",HasRun?Code:"none",HasRun?(Single?CAMERA_COLOR_BLUE:Colors[Batch][Item<3U?Item:2U]):0U,Single?1U:Item<3U?Item+1:3U,
    Batch+1,Item<3U?Item+1:3U,Station,Visit,Stored,Placed,(unsigned long)(HAL_GetTick()-StartTick),Reason);
  if(n>0 && n<(int)sizeof line) (void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)n);
}
static bool decode(const char *code)
{
  unsigned b,i;
  memset(FirstRing,0,sizeof FirstRing);
  for(b=0;b<2;b++)
  {
    unsigned colors=0,rings=0;
    for(i=0;i<3;i++)
    {
      unsigned c=(unsigned)(code[b*8+i]-'0'),r=(unsigned)(code[b*8+4+i]-'0');
      if(c<1 || c>6 || r<1 || r>3 || (colors&(1U<<c)) || (rings&(1U<<r))) return false;
      colors|=1U<<c;rings|=1U<<r;Colors[b][i]=(uint8_t)c;Rings[b][i]=(uint8_t)r;
      if(!b) FirstRing[c]=(uint8_t)r;
    }
    if(b && colors!=( (1U<<Colors[0][0]) | (1U<<Colors[0][1]) | (1U<<Colors[0][2]) )) return false;
  }
  return true;
}
static bool start(const RadarPlan_t *plan,float speed,bool single)
{
  QR_SnapshotTypeDef qr;
  unsigned i;
  if(ContestMission_IsBusy() || GrabTask_IsBusy() || ChassisRoute_IsBusy() ||
     ChassisRoute_StationReserved() || ChassisMotion_IsBusy())
  { Reason="busy";return false; }
  if(!isfinite(speed) || speed<10 || speed>CHASSIS_MOTION_MAX_LINEAR_MM_S)
  { Reason="speed_range";return false; }
  QR_SnapshotGet(&qr);
  if(single)
  {
    /* Test selection is local; preserve any UART/simulated task code. */
    if(!Turntable_ReferenceValid()) { Reason="turntable_reference_missing";return false; }
    if(!Turntable_SlotConfigured(1)) { Reason="first_slot_angle_unset";return false; }
    if(Turntable_InventoryGet(1).state==TURNTABLE_UNKNOWN)
    { Reason="first_slot_inventory_unknown";return false; }
    if(Turntable_InventoryGet(1).state!=TURNTABLE_EMPTY)
    { Reason="first_slot_occupied";return false; }
  }
  else
  {
    if(!qr.Valid) { if(!QR_SimulatedSet("156+123+516+231")) return false;QR_SnapshotGet(&qr); }
    if(!decode(qr.Code)) { Reason="task_code_invalid";return false; }
    if(!Turntable_ReferenceValid()) { Reason="turntable_reference_missing";return false; }
    for(i=1;i<=3;i++)
      if(!Turntable_SlotConfigured((uint8_t)i) || Turntable_InventoryGet((uint8_t)i).state!=TURNTABLE_EMPTY)
      { Reason="three_empty_configured_slots_required";return false; }
  }
  if(plan && (!plan->valid || plan->count<2 || plan->count>RADAR_PLAN_MAX_POINTS ||
              plan->points[0].x_mm!=2250 || plan->points[0].y_mm!=150))
  { Reason="lower_plan_required";return false; }
  if(!GrabTask_PoseStart(false)) { GrabTask_Status_t s;GrabTask_StatusGet(&s);Reason=s.reason;return false; }
  Planned=plan!=NULL;if(plan) Plan=*plan;
  Single=single;
  if(single) { Colors[0][0]=CAMERA_COLOR_BLUE;memset(Code,0,sizeof Code);strcpy(Code,"none"); }
  else memcpy(Code,qr.Code,sizeof Code);
  Source=qr.Source;Speed=speed;
  Stored=Placed=Batch=Item=Pass=Station=Visit=0;StartTick=HAL_GetTick();HasRun=true;
  WorkReady=WorkStarted=false;State=M_PREPARE;Reason="rear_start_pose";
  (void)ContestScreen_ProgressSet(0,0);
  if(!single) (void)ContestScreen_TaskCodeRefresh(Code);
  ContestMission_Report();return true;
}
bool ContestMission_Start(const RadarPlan_t *plan,float speed)
{ return start(plan,speed,false); }
bool ContestMission_StartSingle(float speed)
{ return start(NULL,speed,true); }
static void ring_target(float *x,float *y)
{
  unsigned ring=Station==3U && Batch?FirstRing[Colors[Batch][Item]]:Rings[Batch][Item];
  *x=CenterX;*y=CenterY;
  /* Official Figure1/3: rough1 right,3 left; temporary1 below,3 above. */
  if(Station==2U) *x=1200.0f+(2-(int)ring)*150.0f;
  else *y=1200.0f+((int)ring-2)*150.0f;
}
static bool transfer_start(void)
{
  const char *operation=Station==4U || (Station==2U && Pass) ?"store":"take";
  const char *scene=Station==4U?"raw":Station==2U?(Pass?"proc":"rough"):(Batch?"temp2":"temp1");
  GrabTask_Status_t s;
  if(GrabTask_TransferStart(operation,scene,(uint8_t)(Item+1),(uint8_t)Colors[Batch][Item]))
  { State=M_TRANSFER;Reason=scene;ContestMission_Report();return true; }
  GrabTask_StatusGet(&s);
  if(!strcmp(s.reason,"busy")) return false;
  if(!strcmp(s.reason,"config_missing"))
  { State=M_WAIT_CONFIG;Reason=s.missing;ContestMission_Report(); }
  else fail(s.reason);
  return false;
}
bool ContestMission_Continue(void)
{
  if(State!=M_WAIT_CONFIG || GrabTask_IsBusy()) return false;
  return transfer_start();
}
static bool pose_complete(const GrabTask_Status_t *s,const char *result)
{ return s->state==GRAB_COMPLETE && !strcmp(s->result,result); }
static bool pose_start(bool work)
{
  GrabTask_Status_t grab;
  if(GrabTask_PoseStart(work)) return true;
  GrabTask_StatusGet(&grab);
  if(strcmp(grab.reason,"busy")) fail(grab.reason);
  return false;
}
static bool local_move(float x,float y,const char *next_reason)
{
  ChassisMotion_Status_t motion;
  if(ChassisMotion_MoveTo(x,y,WorkYaw,fminf(Speed,50),CHASSIS_DISTANCE_TIMEOUT_MS))
  { Reason=next_reason;return true; }
  ChassisMotion_StatusGet(&motion);
  /* Pending bus reads and a short feedback gap keep the current item. The
   * motion controller still requires fresh stopped feedback before admission. */
  if(ChassisMotion_IsBusy() || !strcmp(motion.reason,"busy") ||
     !strcmp(motion.reason,"feedback_moving") || !strcmp(motion.reason,"feedback_invalid") ||
     !strcmp(motion.reason,"imu_invalid"))
  {
    if(strcmp(Reason,motion.reason)) { Reason=motion.reason;ContestMission_Report(); }
  }
  else fail(motion.reason);
  return false;
}
void ContestMission_Process(void)
{
  GrabTask_Status_t grab;
  ChassisRoute_Status_t route;
  ChassisMotion_Status_t motion;
  float x,y;
  if(!ContestMission_IsBusy()) return;
  GrabTask_StatusGet(&grab);ChassisRoute_StatusGet(&route);ChassisMotion_StatusGet(&motion);
  if(grab.state==GRAB_ERROR) { fail(grab.reason);return; }
  if(State!=M_PREPARE && (!strcmp(route.state,"error") || !strcmp(route.state,"cancelled")))
  { fail(route.reason);return; }
  if(State==M_WAIT_CONFIG) return;
  if(State==M_PREPARE)
  {
    if(!pose_complete(&grab,"travel_ready")) return;
    if(!ChassisRoute_TaskStart(Planned?&Plan:NULL,Speed))
    {
      ChassisRoute_StatusGet(&route);
      if(!strcmp(route.reason,"busy")) { Reason="route_bus_wait";return; }
      fail(route.reason);return;
    }
    State=M_DRIVING;Reason="qr_pass_through";ContestMission_Report();return;
  }
  if(State==M_DRIVING)
  {
    if(!WorkReady && (!strcmp(route.phase,"work_turn") || !strcmp(route.phase,"work_dock")))
    {
      if(!WorkStarted) { if(!pose_start(true)) return;WorkStarted=true; }
      else if(pose_complete(&grab,"work_ready")) WorkReady=true;
    }
    if(!strcmp(route.state,"done"))
    {
      if(Stored!=12U || Placed!=12U) { fail("mission_incomplete");return; }
      State=M_DONE;Reason="mission_complete";ContestMission_Report();return;
    }
    if(strcmp(route.state,"station") || !route.stop_confirmed || !WorkReady) return;
    Station=(unsigned)route.station;Visit=(unsigned)route.visit;
    Batch=Visit>=5U?1U:0U;Item=Pass=0;
    CenterX=route.target_x_mm;CenterY=route.target_y_mm;
    WorkYaw=Station==4U?0:Station==2U?180:90;
    if(Single)
    {
      if(Station!=4U || Visit!=2U || !ChassisRoute_StationFinish())
      { fail("single_station_handoff_failed");return; }
    }
    else if(!ChassisRoute_WorkBegin()) { fail("station_handoff_failed");return; }
    State=Station==4U?M_TRANSFER_REQUEST:M_CLEAR_REQUEST;
    return;
  }
  if(State==M_TRANSFER_REQUEST) { (void)transfer_start();return; }
  if(State==M_CLEAR_REQUEST || State==M_REAR_REQUEST)
  {
    bool work=State==M_CLEAR_REQUEST;
    if(pose_start(work))
    { State=work?M_RING_CLEAR:M_LEAVE;Reason=work?"ring_clearance":"rear_departure_pose";ContestMission_Report(); }
    return;
  }
  if(State==M_RING_CLEAR)
  {
    if(!pose_complete(&grab,"work_ready")) return;
    ring_target(&x,&y);
    if(!local_move(x,y,"ring_position")) return;
    State=M_RING_MOVE;ContestMission_Report();return;
  }
  if(State==M_RING_MOVE || State==M_CENTER)
  {
    if(motion.state==CHASSIS_MOTION_ERROR) { fail(motion.reason);return; }
    if(motion.state!=CHASSIS_MOTION_DONE || !motion.stop_confirmed || strcmp(motion.reason,"feedback_arrived")) return;
    if(State==M_CENTER)
    {
      if(!ChassisRoute_WorkResume())
      {
        ChassisRoute_StatusGet(&route);
        if(!strcmp(route.state,"error")) fail(route.reason);
        else if(strcmp(Reason,route.reason)) { Reason=route.reason;ContestMission_Report(); }
        return;
      }
      WorkReady=WorkStarted=false;State=M_DRIVING;Reason="next_station";ContestMission_Report();
    }
    else State=M_TRANSFER_REQUEST;
    return;
  }
  if(State==M_TRANSFER)
  {
    bool storing=Station==4U || (Station==2U && Pass);
    if(grab.state!=GRAB_COMPLETE) return;
    if(grab.slot!=Item+1 || strcmp(grab.result,storing?"stored":"released")) { fail("transfer_result_mismatch");return; }
    if(storing) Stored++;else Placed++;
    (void)ContestScreen_ProgressSet((uint8_t)Stored,(uint8_t)Placed);
    Item++;
    if(Single) { State=M_REAR_REQUEST;return; }
    if(Item==3U && Station==2U && Pass==0U) { Item=0;Pass=1; } /* all3 placed before any takeback */
    State=Item==3U?M_REAR_REQUEST:Station==4U?M_TRANSFER_REQUEST:M_CLEAR_REQUEST;
    return;
  }
  if(State==M_LEAVE)
  {
    if(!pose_complete(&grab,"travel_ready")) return;
    if(Single) { State=M_DONE;Reason="single_piece_stored";ContestMission_Report();return; }
    if(!local_move(CenterX,CenterY,"return_station_center")) return;
    State=M_CENTER;ContestMission_Report();
  }
}
