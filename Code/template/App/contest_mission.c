#include "contest_mission.h"
#include "mission_navigation.h"
#include "station_heading.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_config.h"
#include "turntable.h"
#include "QR.h"
#include "contest_screen.h"
#include "console_tx.h"
#include "motor_bus.h"
#include "mechanical_arm.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum {
  M_IDLE,M_PREPARE,M_DRIVING,M_RING_CLEAR,M_RING_MOVE,M_TRANSFER,
  M_LEAVE,M_CENTER,M_WAIT_CONFIG,M_DONE,M_ERROR,M_CANCELLED,
  M_CLEAR_REQUEST,M_REAR_REQUEST,M_TRANSFER_REQUEST,
  M_QR_REQUEST,M_QR_SCAN,M_QR_DISPLAY,M_QR_LEAVE,M_PICK,M_PLACE_REQUEST,M_PAUSING,M_PAUSED,M_RESUMING,M_STOP_WAIT,
  M_CANCELLING,M_CANCEL_WAIT,M_EMPTY_ALIGN,M_BUS_DRAIN,M_BUS_RECOVER,M_BUS_VERIFY,M_BUS_WAIT,M_BUS_RETRY,M_HEADING,M_HEADING_PREP,M_PROBE_TURN,M_PROBE_MOVING
} MissionState;
static MissionState State,WaitState,PausedState,HeadingNext;
static bool ChildPaused,RoutePaused,ResumeAfterStop;
static bool CancelChild,CancelMonitoring;
static uint32_t CancelStopTick;
static bool BusAutoResume,BusStopMonitoring;
static uint32_t BusRecoveryTick,BusStopTick,BusRecoveries;
static uint32_t BusEpisodeTick,BusRetryTick;
static MotorBus_Recovery_t BusFailure;
static bool BusFailureValid;
#define BUS_RECOVERY_RETRY_DELAY_MS 750U
#define BUS_RECOVERY_EPISODE_MS 15000U
static uint32_t QrBaseline,ScreenRetry;
static ChassisRoute_TaskOptions_t RouteOptions;
static ContestMission_Request_t Request;
static char ScannedCode[16];
static bool ScanCaptured;
static const char *Reason;
static uint32_t StartTick;
static char Code[QR_TASK_CODE_BUFFER_SIZE];
static uint8_t Colors[2][3],Rings[2][3],FirstRing[7];
static unsigned Batch,Item,Pass,Station,Visit,Stored,Placed;
static float Speed,CenterX,CenterY,WorkYaw;
static bool Planned,WorkReady,WorkStarted,HasRun,Single;
static bool HeadingTest,HeadingStopRequested;
static uint32_t HeadingStopTick;
static float HeadingBefore;
static bool HeadingLocalOrigin;
static bool HeadingProbe;
static uint8_t HeadingSource=STATION_HEADING_RING;
static unsigned ProbePhase;
static float ProbeSign,ProbeDelta,ProbeTarget,ProbeBefore,ProbeAfter,ProbeYawBefore,ProbeYawDelta;
static bool StartFeedbackSelected,StartOriginPrepared;
static bool RingAlignmentSaved,RingOnly;
/* One measured station anchor per visit; pause/recovery retain it. */
static bool RingAnchorValid;
static float RingAnchorX,RingAnchorY;
static unsigned RingAnchorNumber;
static bool NavigationReady,UpperQrPreparing;
static uint32_t NavigationGeneration;
static ChassisLocalization_Status_t StartPosition;

static RadarPlan_t Plan;
static bool restore_profile(void)
{
  if(ChassisMotion_ProfileGet()==Request.profile || ChassisMotion_ProfileSet(Request.profile))return true;
  Reason="profile_busy";return false;
}
bool ContestMission_BusRecoveryEvidenceGet(MotorBus_Recovery_t *out)
{
  if(!out || !BusFailureValid || Single || !ContestMission_HasPendingWork())return false;
  *out=BusFailure;return true;
}

void ContestMission_Init(void)
{ HeadingSource=STATION_HEADING_RING;StationHeading_Reset();memset(&Request,0,sizeof Request);State=M_IDLE;Reason="idle";HasRun=false;RingAnchorValid=false;Single=HeadingTest=HeadingProbe=RingOnly=false;Stored=Placed=Batch=Item=Pass=0;ChildPaused=RoutePaused=BusAutoResume=false;BusRecoveries=0;BusFailureValid=false;Code[0]=ScannedCode[0]=0; }
bool ContestMission_IsBusy(void)
{ return State!=M_IDLE && State!=M_DONE && State!=M_ERROR && State!=M_CANCELLED && State!=M_PAUSED; }
bool ContestMission_HasRun(void) { return HasRun; }
bool ContestMission_HasPendingWork(void) { return ContestMission_IsBusy() || State==M_PAUSED; }
bool ContestMission_OwnsDisplay(void) { return HasRun && !Single && !HeadingTest && !RingOnly; }
const char *ContestMission_Reason(void) { return Reason; }
bool ContestMission_CanConfigure(void) { return State==M_WAIT_CONFIG || State==M_PAUSED || State==M_STOP_WAIT || State==M_BUS_WAIT; }
static void fail(const char *why)
{
  if(State==M_HEADING)StationHeading_Pause();
  Reason=why;State=M_ERROR;(void)ChassisRoute_Cancel();GrabTask_Stop();ContestMission_Report();
}
void ContestMission_Stop(void)
{
  if(HeadingTest){ContestMission_Cancel();return;}
  if(State==M_CANCELLING || State==M_CANCEL_WAIT)return;
  if(State==M_BUS_DRAIN || State==M_BUS_RECOVER || State==M_BUS_VERIFY || State==M_BUS_WAIT || State==M_BUS_RETRY)
  {BusAutoResume=false;Reason="bus_recovery_manual_pause";ContestMission_Report();return;}
  ResumeAfterStop=false;
  if(!ContestMission_HasPendingWork() || State==M_PAUSING || State==M_PAUSED || State==M_STOP_WAIT) return;
  if(Single){State=M_CANCELLED;Reason="operator_stop";(void)ChassisRoute_Cancel();GrabTask_Stop();ContestMission_Report();return;}
  if(State!=M_RESUMING)PausedState=State;
  if(State==M_HEADING)StationHeading_Pause();
  MissionNavigation_Pause();
  ChildPaused=GrabTask_Pause();
  RoutePaused=ChassisRoute_TaskPause();
  State=M_PAUSING;Reason="operator_stop";ContestMission_Report();
}
void ContestMission_Cancel(void)
{
  if(State==M_HEADING)StationHeading_Pause();
  MissionNavigation_Pause();
  if((!ContestMission_HasPendingWork() && State!=M_ERROR) || State==M_CANCELLING)return;
  if(State!=M_CANCEL_WAIT)
  {
    GrabTask_Status_t grab;GrabTask_StatusGet(&grab);
    CancelChild=ChildPaused || GrabTask_IsBusy() || grab.state==GRAB_ERROR;
  }
  ResumeAfterStop=false;ChildPaused=RoutePaused=false;
  if(State==M_BUS_DRAIN || State==M_BUS_RECOVER || State==M_BUS_VERIFY || State==M_BUS_WAIT || State==M_BUS_RETRY)
    MotorBus_RecoveryCancel();
  BusAutoResume=false;
  GrabTask_Cancel();(void)ChassisRoute_Cancel();
  CancelMonitoring=false;
  State=M_CANCELLING;Reason="operator_cancelled";ContestMission_Report();
}
void ContestMission_Report(void)
{
  static const char *names[]={"idle","preparing_travel","driving","ring_clear","ring_move",
    "transfer","leaving","return_center","config_wait","done","error","cancelled",
    "prepare_ring","prepare_departure","prepare_transfer",
    "qr_prepare","qr_scan","qr_display","qr_departure","car_pick","held_place","stopping","paused","resuming","stop_wait",
    "cancelling","cancel_wait","empty_ring_align","bus_recovery_stopping","bus_recovering","bus_recovery_verify","bus_recovery_wait","bus_recovery_retry","station_heading","heading_preparing","heading_probe_turn","heading_probe_moving"};
  char line[760];
  GrabTask_Status_t grab;ChassisRoute_Status_t route;
  GrabTask_StatusGet(&grab);ChassisRoute_StatusGet(&route);
  if(HeadingTest){
    ChassisMotion_Status_t motion;ChassisMotion_StatusGet(&motion);
    ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
    bool stopped=motion.stop_confirmed || (stop.tx_complete && stop.wheels_stopped);
    int n=snprintf(line,sizeof line,"%s grab route state=%s mode=heading station=%u reason=%s stop_confirmed=%u elapsed_ms=%lu heading_result=%s heading_image=%.2f heading_error=%.2f heading_z=%.2f heading_before=%.2f map_yaw=%.2f local_origin=%u probe=%u probe_before=%.2f probe_after=%.2f probe_delta=%.2f probe_yaw_delta=%.3f heading_source=%s\r\n",
      State==M_ERROR?"ERR":"OK",names[State],Station,Reason,stopped?1U:0U,
      (unsigned long)(HAL_GetTick()-StartTick),StationHeading_Reason(),
      (double)StationHeading_ImageAngle(),(double)StationHeading_BodyError(),(double)StationHeading_ObservationZ(),
      (double)HeadingBefore,(double)(motion.map_yaw_rad*180/CHASSIS_MODEL_PI),HeadingLocalOrigin?1U:0U,HeadingProbe?1U:0U,(double)ProbeBefore,(double)ProbeAfter,(double)ProbeDelta,(double)ProbeYawDelta,HeadingSource==STATION_HEADING_LINE?"line":"ring");
    if(n>0 && n<(int)sizeof line)(void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)n);
    return;
  }
  unsigned item=Item<3U?Item:2U,color=HasRun && !RingOnly?(Single?CAMERA_COLOR_BLUE:Colors[Batch][item]):0U;
  unsigned ring=Station==3U && Batch?FirstRing[color]:Rings[Batch][item];
  const char *source=Single?"test":Request.code_source==MISSION_CODE_FIXED?"fixed":"qr";
  int n=snprintf(line,sizeof line,"%s grab route state=%s mode=%s source=%s code=%s code_source=%s scan_code=%s exec_code=%s color=%u slot=%u ring=%u batch=%u item=%u station=%u visit=%u stored=%u placed=%u stop_confirmed=%u elapsed_ms=%lu reason=%s bus_recoveries=%lu auto_resume=%u start=%s radar=%u route_speed=%.1f turn_speed=%.1f align=%s profile=%s visual_heading=%s heading_result=%s heading_image=%.2f heading_error=%.2f heading_z=%.2f heading_source=%s\r\n",
    State==M_ERROR?"ERR":"OK",State==M_EMPTY_ALIGN && Station==3U && Batch?"empty_material_align":names[State],HeadingTest?"heading":RingOnly?"ring_place":Single?"single":"mission",source,Code[0]?Code:"none",source,ScannedCode[0]?ScannedCode:"none",Code[0]?Code:"none",color,Single?1U:item+1,ring,
    Batch+1,item+1,Station,Visit,Stored,Placed,(State==M_PAUSED || (State==M_CANCELLED && !Single) || (grab.stop_confirmed && route.stop_confirmed))?1U:0U,(unsigned long)(HAL_GetTick()-StartTick),Reason,(unsigned long)BusRecoveries,BusAutoResume?1U:0U,RouteOptions.start_id==RADAR_START_UPPER?"upper":"lower",Request.radar_enabled?1U:0U,Speed,RouteOptions.turn_speed_deg_s,(!Single && Request.align_raised)?"raised":"lowered",ChassisMotion_ProfileName(Request.profile),(!Single && Request.visual_heading_enabled)?"on":"off",StationHeading_Reason(),(double)StationHeading_ImageAngle(),(double)StationHeading_BodyError(),(double)StationHeading_ObservationZ(),HeadingSource==STATION_HEADING_LINE?"line":"ring");
  if(n>0 && n<(int)sizeof line) (void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)n);
}
static bool decode(const char *code)
{
  unsigned b,i;
  if(!code || strlen(code)!=15 || code[3]!='+' || code[7]!='+' || code[11]!='+')return false;
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
static bool start(const RadarPlan_t *plan,const ContestMission_Request_t *request,bool single)
{
  QR_SnapshotTypeDef qr;
  unsigned i;
  float speed=request?request->route_speed_mm_s:NAN;
  if(request && request->profile!=CHASSIS_PROFILE_LINEAR && request->profile!=CHASSIS_PROFILE_COSINE)
  { Reason="profile_required";return false; }
  if(ContestMission_HasPendingWork() || GrabTask_IsBusy() || ChassisRoute_IsBusy() ||
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
    if(!request || (request->code_source!=MISSION_CODE_QR && request->code_source!=MISSION_CODE_FIXED))
    {Reason="source_required";return false;}
    if(!isfinite(request->turn_speed_deg_s) || request->turn_speed_deg_s<5 || request->turn_speed_deg_s>CHASSIS_MOTION_MAX_OMEGA_DEG_S)
    {Reason="turn_speed_range";return false;}
    if(request->code_source==MISSION_CODE_FIXED &&
       (request->fixed_code[15]!=0 || !decode(request->fixed_code)))
    {Reason="task_code_invalid";return false;}
    if(!GrabTask_QrReady(&RouteOptions.qr_scan_y_mm))
    {GrabTask_Status_t g;GrabTask_StatusGet(&g);Reason=g.reason;return false;}
    if(!Turntable_ReferenceValid()) { Reason="turntable_reference_missing";return false; }
    for(i=1;i<=3;i++)
      if(!Turntable_SlotConfigured((uint8_t)i))
      { Reason="slot_angle_unset";return false; }
  }
  if(plan && (!plan->valid || plan->count<2 || plan->count>RADAR_PLAN_MAX_POINTS ||
              plan->points[0].x_mm!=2250 || plan->points[0].y_mm!=(request->start_id==RADAR_START_UPPER?2250:150)))
  { Reason="lower_plan_required";return false; }
  ChassisMotion_Profile_t previous_profile=ChassisMotion_ProfileGet();
  if(!ChassisMotion_ProfileSet(request->profile)){Reason="profile_busy";return false;}
  if(!GrabTask_PoseStart(false))
  { GrabTask_Status_t s;GrabTask_StatusGet(&s);Reason=s.reason;(void)ChassisMotion_ProfileSet(previous_profile);return false; }
  /* Starting a new match declares the physical trays empty; resume never enters here. */
  if(!single) for(i=1;i<=3;i++)
    (void)Turntable_InventorySet((uint8_t)i,TURNTABLE_EMPTY,0);
  Planned=plan!=NULL;if(plan) Plan=*plan;
  Single=single;HeadingTest=false;
  Request=*request;
  if(single) { Colors[0][0]=CAMERA_COLOR_BLUE;memset(Code,0,sizeof Code);strcpy(Code,"none"); }
  else {Code[0]=0;ScannedCode[0]=0;QrBaseline=qr.Sequence;}
  RouteOptions.start_id=request->start_id==RADAR_START_UPPER?RADAR_START_UPPER:RADAR_START_LOWER;
  NavigationReady=false;UpperQrPreparing=false;MissionNavigation_Reset();
  RouteOptions.scan_qr=!single;RouteOptions.turn_speed_deg_s=request->turn_speed_deg_s;
  RingOnly=false;Speed=speed;ChildPaused=RoutePaused=ResumeAfterStop=false;
  Stored=Placed=Batch=Item=Pass=Station=Visit=0;StartTick=HAL_GetTick();HasRun=true;
  StationHeading_Reset();
  WorkReady=WorkStarted=false;State=M_PREPARE;Reason="rear_start_pose";
  StartFeedbackSelected=StartOriginPrepared=false;
  RingAlignmentSaved=false;
  BusAutoResume=BusStopMonitoring=false;BusRecoveries=0;BusFailureValid=false;
  (void)ContestScreen_ProgressSet(0,0);
  if(!single) ContestScreen_TaskCodeClear();
  ContestMission_Report();return true;
}
bool ContestMission_Start(const RadarPlan_t *plan,const ContestMission_Request_t *request)
{ return start(plan,request,false); }
bool ContestMission_StartSingle(float speed)
{ const ContestMission_Request_t request={MISSION_CODE_QR,"",speed,100,RADAR_START_LOWER,false,false,false,CHASSIS_PROFILE_LINEAR};return start(NULL,&request,true); }
bool ContestMission_StartRingTake(uint8_t slot)
{
  uint8_t ring;GrabTask_Status_t grab;
  if(slot<1 || slot>3){Reason="invalid_slot";return false;}
  if(ContestMission_HasPendingWork() || ChassisRoute_IsBusy() || ChassisRoute_StationReserved() || ChassisMotion_IsBusy())
  {Reason="busy";return false;}
  if(!Turntable_ReferenceValid() || !Turntable_SlotConfigured(slot))
  {Reason="turntable_reference_missing";return false;}
  if(!GrabTask_RingTakePrepare(&ring)){GrabTask_StatusGet(&grab);Reason=grab.reason;return false;}
  ContestMission_Init();RingOnly=HasRun=true;Station=2;Visit=0;Item=slot-1;
  Request.profile=ChassisMotion_ProfileGet();Request.align_raised=true;
  Rings[0][Item]=ring;
  Turntable_Inventory_t inventory=Turntable_InventoryGet(slot);
  /* Selected-tray placement needs no color registration; zero stays unknown. */
  Colors[0][Item]=inventory.state==TURNTABLE_OCCUPIED && inventory.color>=1 && inventory.color<=6?
    inventory.color:0;
  StartTick=HAL_GetTick();State=M_EMPTY_ALIGN;Reason="ring_pickplace_verify";
  ContestMission_Report();return true;
}
bool ContestMission_StartHeadingTest(unsigned station,float speed_deg_s)
{return ContestMission_StartHeadingTestSource(station,speed_deg_s,STATION_HEADING_RING);}
bool ContestMission_StartHeadingTestSource(unsigned station,float speed_deg_s,uint8_t source)
{
  GrabTask_Status_t grab;
  if(ContestMission_HasPendingWork() || GrabTask_IsBusy() || ChassisRoute_IsBusy() ||
     ChassisRoute_StationReserved() || ChassisMotion_IsBusy())
  {Reason="busy";return false;}
  if(station!=2 && station!=3){Reason="heading_station_required";return false;}
  if(source!=STATION_HEADING_RING && source!=STATION_HEADING_LINE)
  {Reason="heading_source_required";return false;}
  if(source==STATION_HEADING_LINE && station!=3)
  {Reason="heading_line_station_required";return false;}
  if(!isfinite(speed_deg_s) || speed_deg_s<1 || speed_deg_s>10)
  {Reason="heading_speed_range";return false;}
  GrabTask_StatusGet(&grab);
  if(grab.state!=GRAB_COMPLETE || strcmp(grab.result,"work_ready"))
  {Reason="heading_prepare_required";return false;}
  /* No QR, inventory, route preparation or arm motion belongs to this test. */
  ContestMission_Init();HeadingTest=HasRun=true;Station=station;Visit=0;
  HeadingSource=source;
  WorkYaw=station==2?180:90;Speed=0;
  memset(&RouteOptions,0,sizeof RouteOptions);RouteOptions.turn_speed_deg_s=speed_deg_s;
  Request.visual_heading_enabled=true;Request.align_raised=true;
  StartFeedbackSelected=StartOriginPrepared=HeadingStopRequested=false;
  HeadingBefore=ProbeBefore=ProbeAfter=ProbeYawDelta=ProbeDelta=NAN;HeadingLocalOrigin=false;
  StartTick=HAL_GetTick();State=M_HEADING_PREP;Reason="heading_preparing";
  ContestMission_Report();return true;
}
bool ContestMission_StartHeadingProbe(unsigned station,float polarity,float speed_deg_s)
{return ContestMission_StartHeadingProbeSource(station,polarity,speed_deg_s,STATION_HEADING_RING);}
bool ContestMission_StartHeadingProbeSource(unsigned station,float polarity,float speed_deg_s,uint8_t source)
{
  if(!isfinite(polarity) || (polarity!=1 && polarity!=-1))
  {Reason="heading_probe_polarity";return false;}
  if(!isfinite(speed_deg_s) || speed_deg_s<5 || speed_deg_s>10)
  {Reason="heading_probe_speed";return false;}
  if(!ContestMission_StartHeadingTestSource(station,speed_deg_s,source))return false;
  HeadingProbe=true;ProbePhase=0;ProbeSign=polarity;
  ContestMission_Report();return true;
}
static unsigned target_ring(void)
{ return Station==3U && Batch?FirstRing[Colors[Batch][Item]]:Rings[Batch][Item]; }
static bool station_anchor_capture(void)
{
  ChassisLocalization_Status_t p;
  if(RingAnchorValid)return true;
  ChassisLocalization_Get(&p);
  if(!p.origin_valid || !p.feedback_valid || HAL_GetTick()-p.feedback_tick>600U ||
     !isfinite(p.observer.feedback.x_mm) || !isfinite(p.observer.feedback.y_mm))
  {Reason="feedback_invalid";return false;}
  RingAnchorX=p.observer.feedback.x_mm;RingAnchorY=p.observer.feedback.y_mm;
  RingAnchorNumber=target_ring();RingAnchorValid=true;return true;
}
static void ring_target(float *x,float *y)
{
  unsigned ring=target_ring();
  if(RingAnchorValid) {
    *x=RingAnchorX;*y=RingAnchorY;
    if(Station==2U)*x+=((int)RingAnchorNumber-(int)ring)*150.0f;
    else *y+=((int)ring-(int)RingAnchorNumber)*150.0f;
    return;
  }
  *x=CenterX;*y=CenterY;
  /* Official Figure1/3: rough1 right,3 left; temporary1 below,3 above. */
  if(Station==2U) *x=1200.0f+(2-(int)ring)*150.0f;
  else *y=1200.0f+((int)ring-2)*150.0f;
}
static bool child_rejected(MissionState retry)
{
  GrabTask_Status_t g;GrabTask_StatusGet(&g);
  if(!strcmp(g.reason,"busy") || !strcmp(g.reason,"bus_unavailable") || !strcmp(g.reason,"imu_invalid"))
  {Reason=g.reason;return false;}
  if(!strcmp(g.reason,"config_missing"))
  {WaitState=retry;State=M_WAIT_CONFIG;Reason=g.missing;ContestMission_Report();}
  else fail(g.reason);
  return false;
}
static bool transfer_start(void)
{
  uint8_t color=Colors[Batch][Item],slot=(uint8_t)(Item+1);
  bool accepted;
  if(Single)accepted=GrabTask_TransferStart("store","raw",slot,color);
  else if(Station==4U)accepted=GrabTask_TrackedStoreStart(color,slot,GRAB_TRACK_FEEDBACK);
  else if(Station==2U && Pass)accepted=GrabTask_ProcessedStoreStartEx(color,Rings[Batch][Item],slot,Request.align_raised);
  else if((Station==2U || Station==3U) && !RingAlignmentSaved)
  {
    if(!GrabTask_AlignPlaceStartEx(Station==2U?"rough":Batch?"temp2":"temp1",
        Station==3U && Batch?color:Rings[Batch][Item],Request.align_raised))return child_rejected(M_TRANSFER_REQUEST);
    State=M_EMPTY_ALIGN;Reason=Station==3U && Batch?"empty_material_align":"empty_ring_align";ContestMission_Report();return true;
  }
  else
  {
    if(!GrabTask_CarPickStart(slot))return child_rejected(M_TRANSFER_REQUEST);
    State=M_PICK;Reason="car_pick";ContestMission_Report();return true;
  }
  if(!accepted)return child_rejected(M_TRANSFER_REQUEST);
  if(!Single && Station==4U)GrabTask_NextRawColorSet(Item<2U?Colors[Batch][Item+1U]:0);
  if((Single || Item==2U) && !GrabTask_StoreFinishTravel())
  {fail("store_finish_policy");return false;}
  State=M_TRANSFER;Reason=Station==4U?"raw":"proc";ContestMission_Report();return true;
}
static bool place_start(void)
{
  const char *scene=Station==2U?"rough":Batch?"temp2":"temp1";
  uint8_t color=Colors[Batch][Item];
  uint8_t target=Station==3U && Batch?color:Rings[Batch][Item];
  bool accepted=GrabTask_AlignedPlaceStart(scene,target,color);
  if(!accepted)return child_rejected(M_PLACE_REQUEST);
  State=M_TRANSFER;Reason=scene;ContestMission_Report();return true;
}
bool ContestMission_Continue(void)
{
  if(State==M_BUS_WAIT)return ContestMission_RequestBusRecovery(true);
  if(State==M_WAIT_CONFIG && !GrabTask_IsBusy())
  {if(!restore_profile())return false;if(WaitState==M_PREPARE && !NavigationReady)MissionNavigation_Reset();State=WaitState;Reason="configuration_updated";return true;}
  if(State==M_STOP_WAIT)
  {
    if(ChildPaused)(void)GrabTask_Pause();
    if(RoutePaused)(void)ChassisRoute_TaskPause();
    ResumeAfterStop=true;State=M_PAUSING;Reason="verifying_stop";ContestMission_Report();return true;
  }
  if(State!=M_PAUSED)return false;
  State=M_RESUMING;Reason="resume_requested";ContestMission_Report();return true;
}
bool ContestMission_RequestBusRecovery(bool auto_resume)
{
  if(Single || HeadingTest || !ContestMission_HasPendingWork() || State==M_CANCELLING || State==M_CANCEL_WAIT)return false;
  if(State==M_BUS_DRAIN || State==M_BUS_RECOVER || State==M_BUS_VERIFY || State==M_BUS_RETRY)
  {if(!auto_resume)BusAutoResume=false;return true;}
  if(State!=M_BUS_WAIT && State!=M_PAUSED && State!=M_PAUSING && State!=M_STOP_WAIT && State!=M_RESUMING)
    PausedState=State;
  if(State==M_HEADING)StationHeading_Pause();
  BusAutoResume=auto_resume && !RingOnly;ResumeAfterStop=false;BusEpisodeTick=HAL_GetTick();BusFailureValid=false;
  ChildPaused=GrabTask_Pause() || ChildPaused;
  RoutePaused=ChassisRoute_TaskPause() || RoutePaused;
  ChassisRoute_TaskRecoveryHold(true);
  /* Send cancellation stops once, then let all producer requests drain. The
   * recovery owner obtains its own fresh speed/state proof before unlocking. */
  if(!RoutePaused)(void)ChassisMotion_Stop(0);
  BusRecoveryTick=HAL_GetTick();State=M_BUS_DRAIN;Reason="bus_recovery_stopping";
  ContestMission_Report();return true;
}
static void bus_recovery_wait(const char *reason)
{State=M_BUS_WAIT;BusAutoResume=false;Reason=reason;ContestMission_Report();}
static void bus_recovery_failed(const MotorBus_Recovery_t *recovery)
{
  BusFailure=*recovery;BusFailureValid=true;
  char line[192];
  int n=snprintf(line,sizeof line,"OK bus attempt stage=%s reason=%s addr=%u func=0x%02X uart_error=0x%08lX tx_sent=%u elapsed_ms=%lu\r\n",
    recovery->stage?recovery->stage:"unknown",recovery->reason?recovery->reason:"unknown",recovery->address,recovery->function,
    (unsigned long)recovery->uart_error,recovery->tx_sent?1U:0U,(unsigned long)recovery->elapsed_ms);
  if(n>0 && n<(int)sizeof line)(void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)n);
  char hex[33];unsigned i;
  for(i=0;i<recovery->rx_sample_length;i++)(void)snprintf(hex+2*i,sizeof hex-2*i,"%02X",recovery->rx_sample[i]);
  if(!i)strcpy(hex,"none");
  n=snprintf(line,sizeof line,"OK bus attempt_rx bytes=%lu pending=%u partial=%u gap_ms=%lu ndtr=%u cursor=%u hex=%s\r\n",
    (unsigned long)recovery->rx_bytes,recovery->rx_pending,recovery->parse_bytes,(unsigned long)recovery->process_gap_ms,
    recovery->dma_remaining,recovery->dma_position,hex);
  if(n>0 && n<(int)sizeof line)(void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)n);
  if(MotorBus_RecoveryRetryable(recovery))
  {
    if(HAL_GetTick()-BusEpisodeTick<BUS_RECOVERY_EPISODE_MS)
    {
      ChassisRoute_TaskRecoveryHold(true);BusRetryTick=HAL_GetTick();State=M_BUS_RETRY;
      Reason="bus_recovery_retry";ContestMission_Report();return;
    }
    bus_recovery_wait("bus_recovery_retry_exhausted");return;
  }
  bus_recovery_wait(recovery->reason?recovery->reason:"bus_recovery_failed");
}
static void bus_recovery_process(const GrabTask_Status_t *grab,const ChassisRoute_Status_t *route)
{
  uint32_t now=HAL_GetTick();
  if(State==M_BUS_WAIT)return;
  if(State==M_BUS_VERIFY)
  {
    MotorBus_Diagnostic_t fault;MotorBus_DiagnosticGet(&fault);
    if(fault.locked)
    {
      /* This is a new fault after low-level recovery unlocked the bus. Keep
       * the original paused stage and episode while fresh stop polling drains. */
      MotorBus_Recovery_t current;memset(&current,0,sizeof current);
      current.reason=fault.reason;current.stage="stop_verify";current.address=fault.address;
      current.function=fault.function;current.uart_error=fault.uart_error;
      current.elapsed_ms=now-BusRecoveryTick;current.tx_sent=fault.waiting_reply;
      current.rx_bytes=fault.rx_bytes;current.last_rx_ms=fault.last_rx_ms;
      current.process_gap_ms=fault.process_gap_ms;current.rx_pending=fault.rx_pending;
      current.parse_bytes=fault.parse_bytes;current.rx_sample_length=fault.rx_sample_length;
      memcpy(current.rx_sample,fault.rx_sample,sizeof current.rx_sample);current.dma_remaining=0xffffU;
      ChildPaused=GrabTask_Pause() || ChildPaused;ChassisRoute_TaskRecoveryHold(true);
      bus_recovery_failed(&current);return;
    }
  }
  if(State==M_BUS_RETRY)
  {
    if(now-BusEpisodeTick>=BUS_RECOVERY_EPISODE_MS){bus_recovery_wait("bus_recovery_retry_exhausted");return;}
    if(now-BusRetryTick<BUS_RECOVERY_RETRY_DELAY_MS)return;
    BusRecoveryTick=now;State=M_BUS_DRAIN;Reason="bus_recovery_stopping";ContestMission_Report();return;
  }
  if(State==M_BUS_DRAIN)
  {
    if(now-BusRecoveryTick>10000U){bus_recovery_wait("bus_recovery_drain_wait");return;}
    if(GrabTask_IsBusy() || Turntable_IsBusy() || MechanicalArm_IsBusy() ||
       ChassisMotion_IsBusy() || ChassisRoute_IsBusy())return;
    Mecanum_Feedback_Enable(false);Mecanum_Feedback_StopMonitor(false);
    if(!MotorBus_RecoveryStart())return;
    BusFailureValid=false;
    BusRecoveries++;State=M_BUS_RECOVER;Reason="bus_recovering";ContestMission_Report();return;
  }
  if(State==M_BUS_RECOVER)
  {
    MotorBus_Recovery_t recovery;MotorBus_RecoveryGet(&recovery);
    if(recovery.active)return;
    if(!recovery.success){bus_recovery_failed(&recovery);return;}
    if(recovery.checked_mask!=0xffU || !Mecanum_RecoveryAfterBusCheck())
    {bus_recovery_wait("bus_recovery_failed");return;}
    /* Old at-target bits are not parking proof. Rebuild both arm and wheel
     * stopping after recovery, keeping original resume states untouched. */
    if(ChildPaused && !GrabTask_BusRecoveryVerify())
    {bus_recovery_wait("bus_recovery_child_wait");return;}
    ChassisRoute_TaskRecoveryHold(false);
    if(!RoutePaused)(void)ChassisMotion_Stop(0);
    BusRecoveryTick=now;BusStopMonitoring=false;
    State=M_BUS_VERIFY;Reason="bus_recovery_verify";ContestMission_Report();return;
  }
  if(now-BusRecoveryTick>10000U){bus_recovery_wait("bus_recovery_stop_wait");return;}
  if(GrabTask_IsBusy() || (ChildPaused && !grab->stop_confirmed))return;
  if(RoutePaused)
  {
    if(strcmp(route->state,"paused") || !route->stop_confirmed)return;
  }
  else
  {
    if(!BusStopMonitoring)
    {
      BusStopTick=now;
      if(!ChassisMotion_StopRequest(0)){bus_recovery_wait("bus_recovery_stop_wait");return;}
      BusStopMonitoring=true;return;
    }
    ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
    if(!stop.tx_complete || !stop.wheels_stopped || (uint32_t)(stop.requested_ms-BusStopTick)>=0x80000000U)return;
  }
  if(ChassisMotion_IsBusy())return;
  State=BusAutoResume?M_RESUMING:M_PAUSED;Reason="bus_recovered";
  ContestMission_Report();
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
static bool prepare_wait(const char *why)
{
  if(strcmp(Reason,why)) {Reason=why;ContestMission_Report();}
  return false;
}
static bool at_lower_start(const ChassisModel_Pose_t *pose,float mm,float deg)
{
  float yaw=pose->yaw_rad*180.0f/CHASSIS_MODEL_PI;
  return isfinite(pose->x_mm) && isfinite(pose->y_mm) && isfinite(yaw) &&
    hypotf(pose->x_mm-2250.0f,pose->y_mm-(RouteOptions.start_id==RADAR_START_UPPER?2250.0f:150.0f))<=mm &&
    fabsf(remainderf(yaw-(RouteOptions.start_id==RADAR_START_UPPER?-90.0f:180.0f),360.0f))<=deg;
}
static bool prepare_chassis(void)
{
  ChassisMotion_Status_t motion;
  Mecanum_Status_t bus;
  uint32_t now=HAL_GetTick();
  if(!StartFeedbackSelected)
  {
    if(!Mecanum_Feedback_Select(0))return prepare_wait("start_feedback_wait");
    StartFeedbackSelected=true;
  }
  /* Pose completion/ordinary stop can disable polling. Re-enable on resume,
   * without resetting the in-flight read schedule each main-loop pass. */
  Mecanum_Feedback_Enable(true);
  Mecanum_StatusGet(&bus);
  if(bus.locked || bus.stop_pending || bus.stage==MECANUM_STAGE_FAULT)
    return prepare_wait("bus_unavailable");
  for(unsigned i=0;i<4;i++)
  {
    Mecanum_Feedback_t wheel;
    if(!Mecanum_FeedbackGet(i,&wheel) || !wheel.speed_valid || !wheel.position_valid || !wheel.state_valid ||
       now-wheel.speed_ms>600U || now-wheel.position_ms>600U || now-wheel.state_ms>600U)
      return prepare_wait("start_feedback_wait");
    if(wheel.state_flags&0x0cU)return prepare_wait("start_wheel_fault");
    if(!(wheel.state_flags&1U))return prepare_wait("start_wheels_disabled");
    if(wheel.speed_rpm>1 || wheel.speed_rpm<-1)return prepare_wait("start_wheels_moving");
  }
  ChassisMotion_StatusGet(&motion);
  /* HeadingProcess runs before localization/mission and applies the existing
   * verified-IMU/freshness gate, without zeroing or recalibrating the IMU. */
  if(!motion.heading_valid)return prepare_wait("start_imu_wait");
  ChassisLocalization_Get(&StartPosition);
  if(HeadingTest){
    /* No known field XY at a bench test. Keep any valid field origin; create
     * only a local test origin when absent, without resetting the IMU. */
    if(!StartPosition.origin_valid){
      if(!ChassisLocalization_Origin(0,0,WorkYaw*CHASSIS_MODEL_PI/180))return prepare_wait("start_origin_wait");
      HeadingLocalOrigin=true;
      return prepare_wait("start_localization_wait");
    }
    if(!StartPosition.feedback_valid || !StartPosition.speed_valid || now-StartPosition.feedback_tick>600U)
      return prepare_wait("start_localization_wait");
    return true;
  }
  if(!StartOriginPrepared || !StartPosition.origin_valid)
  {
    bool matching=StartPosition.origin_valid && at_lower_start(&StartPosition.origin,0.1f,0.1f);
    if(matching && !StartPosition.feedback_valid)return prepare_wait("start_localization_wait");
    if(matching && at_lower_start(&StartPosition.observer.feedback,
                                 CHASSIS_DISTANCE_TOLERANCE_MM,CHASSIS_DISTANCE_HEADING_DEG))
      StartOriginPrepared=true;
    else
    {
      /* A new match declares placement. Keep a valid anchor across pause;
       * if IMU loss revoked it BEFORE departure, revalidate and restore it. */
      if(!ChassisLocalization_Origin(2250,RouteOptions.start_id==RADAR_START_UPPER?2250:150,RouteOptions.start_id==RADAR_START_UPPER?-CHASSIS_MODEL_PI/2:CHASSIS_MODEL_PI))return prepare_wait("start_origin_wait");
      StartOriginPrepared=true;
      return prepare_wait("start_localization_wait");
    }
  }
  if(!StartPosition.origin_valid || !StartPosition.feedback_valid || !StartPosition.speed_valid ||
     now-StartPosition.feedback_tick>600U)
    return prepare_wait("start_localization_wait");
  return true;
}
static void heading_begin(MissionState next)
{
  HeadingNext=next;
  if(!HeadingTest)HeadingSource=Station==3U && Batch?STATION_HEADING_LINE:STATION_HEADING_RING;
  if(HeadingTest && HeadingProbe)StationHeading_ObserveBeginSource(Station,WorkYaw,RouteOptions.turn_speed_deg_s,HeadingSource);
  else StationHeading_BeginSource(Station,WorkYaw,RouteOptions.turn_speed_deg_s,HeadingSource);
  State=M_HEADING;Reason="heading_observing";ContestMission_Report();
}
void ContestMission_Process(void)
{
  GrabTask_Status_t grab;
  ChassisRoute_Status_t route;
  ChassisMotion_Status_t motion;
  float x,y;
  if(!ContestMission_IsBusy()) return;
  GrabTask_StatusGet(&grab);ChassisRoute_StatusGet(&route);ChassisMotion_StatusGet(&motion);
  if(ScannedCode[0] && ContestScreen_HasError() && HAL_GetTick()-ScreenRetry>=1000U) {
    (void)ContestScreen_TaskCodeRefresh(ScannedCode);ScreenRetry=HAL_GetTick();
  }
  if(State==M_BUS_DRAIN || State==M_BUS_RECOVER || State==M_BUS_VERIFY || State==M_BUS_WAIT || State==M_BUS_RETRY)
  {bus_recovery_process(&grab,&route);return;}
  /* Detect only a CURRENT eligible transport fault, before treating child
   * stop_unconfirmed or route bus_fault as a terminal match error. */
  MotorBus_Diagnostic_t bus_fault;MotorBus_DiagnosticGet(&bus_fault);
  if(!Single && State!=M_CANCELLING && State!=M_CANCEL_WAIT && State!=M_PAUSED &&
     State!=M_PAUSING && State!=M_STOP_WAIT && MotorBus_TransientFault(&bus_fault))
  {if(HeadingTest)fail(bus_fault.reason);else (void)ContestMission_RequestBusRecovery(true);return;}
  if(State==M_CANCELLING || State==M_CANCEL_WAIT)
  {
    if(CancelChild && (GrabTask_IsBusy() || !grab.stop_confirmed))
    {
      if(State==M_CANCELLING && grab.state==GRAB_ERROR)
      {State=M_CANCEL_WAIT;Reason="cancel_stop_unconfirmed";ContestMission_Report();}
      return;
    }
    if(State==M_CANCEL_WAIT)return;
    /* The child may send its own wheel stop while cancelling. Start the final
     * monitor after that chain drains, so later stop TX cannot invalidate it. */
    if(!CancelMonitoring)
    {
      CancelStopTick=HAL_GetTick();
      if(!ChassisMotion_StopRequest(0))
      {State=M_CANCEL_WAIT;Reason="cancel_stop_unconfirmed";ContestMission_Report();return;}
      CancelMonitoring=true;return;
    }
    ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
    if(stop.tx_complete && stop.wheels_stopped &&
       (uint32_t)(stop.requested_ms-CancelStopTick)<0x80000000U &&
       !ChassisMotion_IsBusy() && !ChassisRoute_IsBusy())
    {State=M_CANCELLED;Reason="operator_cancelled";ContestMission_Report();}
    else if(!ChassisMotion_StopPending())
    {State=M_CANCEL_WAIT;Reason="cancel_stop_unconfirmed";ContestMission_Report();}
    return;
  }
  if(State==M_RESUMING)
  {
    if(grab.state==GRAB_ERROR || (!RingOnly && motion.state==CHASSIS_MOTION_ERROR))
    {State=M_STOP_WAIT;Reason=grab.state==GRAB_ERROR?grab.reason:motion.reason;ContestMission_Report();return;}
    if(RoutePaused && !strcmp(route.state,"error"))
    {if(GrabTask_Pause())ChildPaused=true;State=M_STOP_WAIT;Reason=route.reason;ContestMission_Report();return;}
    if(RoutePaused && !ChassisRoute_TaskResumeReady())return;
    if(!restore_profile())return;
    if(ChildPaused){if(!GrabTask_Continue())return;ChildPaused=false;}
    if(RoutePaused){if(!ChassisRoute_TaskContinue())return;RoutePaused=false;}
    State=PausedState;Reason="resumed";
    if(State==M_RING_MOVE)State=M_RING_CLEAR;
    else if(State==M_CENTER)State=M_LEAVE;
    ContestMission_Report();return;
  }
  if(State==M_PAUSING)
  {
    if(grab.state==GRAB_ERROR || (!RingOnly && (motion.state==CHASSIS_MOTION_ERROR || !strcmp(route.state,"error"))))
    {State=M_STOP_WAIT;Reason=grab.state==GRAB_ERROR?grab.reason:motion.state==CHASSIS_MOTION_ERROR?motion.reason:route.reason;ContestMission_Report();return;}
    if(!GrabTask_IsBusy() && (!ChildPaused || grab.stop_confirmed) &&
       (!RoutePaused || (!strcmp(route.state,"paused") && route.stop_confirmed)))
    {State=ResumeAfterStop?M_RESUMING:M_PAUSED;Reason=ResumeAfterStop?"resume_requested":"operator_stop";ContestMission_Report();}
    return;
  }
  if(State==M_STOP_WAIT)return;
  if(grab.state==GRAB_ERROR) { fail(grab.reason);return; }
  if(!HeadingTest && !RingOnly && State!=M_PREPARE && (!strcmp(route.state,"error") || !strcmp(route.state,"cancelled")))
  { fail(route.reason);return; }
  if(State==M_WAIT_CONFIG) return;
  if(State==M_HEADING_PREP){
    if(!pose_complete(&grab,"work_ready")){(void)prepare_wait("heading_prepare_required");return;}
    if(!prepare_chassis())return;
    if(!HeadingStopRequested){
      HeadingStopTick=HAL_GetTick();
      if(!ChassisMotion_StopRequest(0)){(void)prepare_wait("heading_pose_wait");return;}
      HeadingStopRequested=true;return;
    }
    ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
    if(!stop.tx_complete || !stop.wheels_stopped || ChassisMotion_IsBusy() ||
       (uint32_t)(stop.requested_ms-HeadingStopTick)>=0x80000000U){
      if(!ChassisMotion_StopPending())HeadingStopRequested=false;
      (void)prepare_wait("heading_pose_wait");return;
    }
    HeadingBefore=motion.map_yaw_rad*180/CHASSIS_MODEL_PI;
    heading_begin(M_DONE);return;
  }
  if(State==M_PREPARE)
  {
    if(!pose_complete(&grab,UpperQrPreparing?"qr_pose_ready":"travel_ready") &&
       !(UpperQrPreparing && GrabTask_PoseClearanceReady())) return;
    if(!Single && !prepare_chassis())return;
    if(!Single && Request.radar_enabled && NavigationReady && NavigationGeneration!=StartPosition.generation){
      NavigationReady=false;MissionNavigation_Reset();
    }
    if(!Single && Request.radar_enabled && !NavigationReady){
      const char *why;int ready=MissionNavigation_Process(RouteOptions.start_id,StartPosition.generation,&Plan,&why);
      if(ready<0){WaitState=M_PREPARE;State=M_WAIT_CONFIG;Reason=why;ContestMission_Report();return;}
      if(!ready){(void)prepare_wait(why);return;}
      Planned=NavigationReady=true;NavigationGeneration=StartPosition.generation;
    }
    if(!Single && RouteOptions.start_id==RADAR_START_UPPER && !UpperQrPreparing){
      if(!GrabTask_QrPoseStart())return;
      UpperQrPreparing=true;Reason="preparing_qr_pose";return;
    }

    if(!(Single?ChassisRoute_TaskStart(Planned?&Plan:NULL,Speed):ChassisRoute_TaskStartEx(Planned?&Plan:NULL,Speed,&RouteOptions)))
    {
      ChassisRoute_StatusGet(&route);
      if(!strcmp(route.reason,"busy")) { (void)prepare_wait("route_bus_wait");return; }
      if(!Single && !strcmp(route.reason,"start_origin_or_feedback"))
      { (void)prepare_wait("start_localization_wait");return; }
      fail(route.reason);return;
    }
    State=M_DRIVING;Reason=Single?"single_raw_route":"to_qr";ContestMission_Report();return;
  }
  if(State==M_QR_REQUEST)
  {
    if(!GrabTask_QrScanCompetitionStart(QrBaseline)){(void)child_rejected(M_QR_REQUEST);return;}
    ScanCaptured=false;State=M_QR_SCAN;Reason="scanning_actual_code";ContestMission_Report();return;
  }
  if(State==M_QR_SCAN)
  {
    QR_SnapshotTypeDef qr;QR_SnapshotGet(&qr);
    if(!ScanCaptured && qr.Valid && qr.Source==QR_SOURCE_UART && qr.Sequence!=QrBaseline)
    {memcpy(ScannedCode,qr.Code,sizeof ScannedCode);QrBaseline=qr.Sequence;ScanCaptured=true;}
    if(!ScanCaptured)return;
    (void)ContestScreen_TaskCodeRefresh(ScannedCode);ScreenRetry=HAL_GetTick();
    bool valid=decode(Request.code_source==MISSION_CODE_FIXED?Request.fixed_code:ScannedCode);
    if(!valid){State=M_QR_REQUEST;Reason="task_code_invalid";ContestMission_Report();return;}
    memcpy(Code,Request.code_source==MISSION_CODE_FIXED?Request.fixed_code:ScannedCode,sizeof Code);
    State=M_QR_LEAVE;Reason="qr_departure";ContestMission_Report();
  }
  if(State==M_QR_LEAVE)
  {
    if(!GrabTask_QrDepartStart())return;
    if(!ChassisRoute_WorkResume())return;
    WorkReady=WorkStarted=false;State=M_DRIVING;Reason="to_raw";ContestMission_Report();return;
  }
  if(State==M_DRIVING)
  {
    if(!Single && !strcmp(route.state,"station") && route.station==5U && route.visit==1U && route.stop_confirmed)
    {
      if(!ChassisRoute_WorkBegin())return;
      Station=5;Visit=1;State=M_QR_REQUEST;Reason="qr_station";ContestMission_Report();return;
    }
    if(!WorkReady)
    {
      /* Once admitted, the pose belongs to this station even if position
       * correction changes the route phase to move before Base finishes. */
      if(WorkStarted) { if(pose_complete(&grab,"work_ready")) WorkReady=true; }
      else if(!strcmp(route.phase,"work_turn") || !strcmp(route.phase,"work_dock") ||
              (!strcmp(route.state,"station") && route.station==2U && route.stop_confirmed))
      { if(!pose_start(true)) return;WorkStarted=true; }
    }
    if(!strcmp(route.state,"done"))
    {
      if(Stored!=12U || Placed!=12U) { fail("mission_incomplete");return; }
      State=M_DONE;Reason="mission_complete";ContestMission_Report();return;
    }
    if(strcmp(route.state,"station") || !route.stop_confirmed || !WorkReady) return;
    Station=(unsigned)route.station;Visit=(unsigned)route.visit;
    Batch=Visit>=5U?1U:0U;Item=Pass=0;
    CenterX=route.target_x_mm;CenterY=route.target_y_mm;RingAnchorValid=false;
    WorkYaw=Station==4U?0:Station==2U?180:90;
    if(Single)
    {
      if(Station!=4U || Visit!=2U || !ChassisRoute_StationFinish())
      { fail("single_station_handoff_failed");return; }
    }
    else if(!ChassisRoute_WorkBegin()) { fail("station_handoff_failed");return; }
    /* WorkReady already certifies the arrival preparation for the first ring. */
    if(!Single && Request.visual_heading_enabled && Station!=4U)heading_begin(M_RING_CLEAR);
    else State=Station==4U?M_TRANSFER_REQUEST:M_RING_CLEAR;
    return;
  }
  if(State==M_PROBE_TURN){
    if(!GrabTask_HeadingReady()){(void)prepare_wait("heading_pose_wait");return;}
    /* The absolute target is captured once; transport retries never add a turn. */
    if(!ChassisMotion_TurnToPrecision(ProbeTarget,RouteOptions.turn_speed_deg_s,.1f,CHASSIS_DISTANCE_TIMEOUT_MS)){
      ChassisMotion_StatusGet(&motion);
      if(motion.state==CHASSIS_MOTION_ERROR){fail(motion.reason);return;}
      (void)prepare_wait(motion.reason);return;
    }
    State=M_PROBE_MOVING;Reason="heading_probe_moving";ContestMission_Report();return;
  }
  if(State==M_PROBE_MOVING){
    if(motion.state==CHASSIS_MOTION_ERROR){fail(motion.reason);return;}
    if(motion.state!=CHASSIS_MOTION_DONE || !motion.stop_confirmed)return;
    ProbeYawDelta=remainderf(motion.map_yaw_rad*180/CHASSIS_MODEL_PI-ProbeYawBefore,360);
    ProbePhase=1;heading_begin(M_DONE);return;
  }
  if(State==M_HEADING) {
    if(motion.state==CHASSIS_MOTION_ERROR){fail(motion.reason);return;}
    bool done=StationHeading_Process();
    const char *why=StationHeading_Reason();
    bool changed=strcmp(Reason,why)!=0;Reason=why;
    if(done){
      if(HeadingTest && HeadingProbe && !strcmp(why,"heading_observed")){
        if(!ProbePhase){
          ProbeBefore=StationHeading_ImageAngle();ProbeYawBefore=motion.map_yaw_rad*180/CHASSIS_MODEL_PI;
          if(!isfinite(ProbeBefore) || fabsf(ProbeBefore)>90){fail("heading_angle_mismatch");return;}
          ProbeDelta=ProbeSign*ProbeBefore;
          ProbeTarget=remainderf(ProbeYawBefore+ProbeDelta,360);
          State=M_PROBE_TURN;Reason="heading_probe_turn";
        }else{
          ProbeAfter=StationHeading_ImageAngle();
          ProbeYawDelta=remainderf(motion.map_yaw_rad*180/CHASSIS_MODEL_PI-ProbeYawBefore,360);
          State=M_DONE;Reason="heading_probe_done";
        }
      }else{State=HeadingNext;}
      ContestMission_Report();
    }
    else if(changed)ContestMission_Report();
    return;
  }
  if(State==M_TRANSFER_REQUEST) { (void)transfer_start();return; }
  if(State==M_EMPTY_ALIGN)
  {
    if(grab.state!=GRAB_HOLD || strcmp(grab.result,"aligned"))return;
    bool had_anchor=RingAnchorValid;
    if(!RingOnly && !station_anchor_capture())return;
    if(!GrabTask_AlignPlaceCapture()){if(!had_anchor)RingAnchorValid=false;return;}
    RingAlignmentSaved=true;State=M_TRANSFER_REQUEST;Reason="aligned_position_saved";
    ContestMission_Report();return;
  }
  if(State==M_PICK)
  {
    if(!pose_complete(&grab,"picked"))return;
    if(grab.slot!=Item+1){fail("pickup_result_mismatch");return;}
    State=M_PLACE_REQUEST;return;
  }
  if(State==M_PLACE_REQUEST){(void)place_start();return;}
  if(State==M_CLEAR_REQUEST || State==M_REAR_REQUEST)
  {
    bool work=State==M_CLEAR_REQUEST;
    if(pose_start(work))
    { State=work?M_RING_CLEAR:M_LEAVE;Reason=work?"ring_clearance":"rear_departure_pose";ContestMission_Report(); }
    return;
  }
  if(State==M_RING_CLEAR)
  {
    if(!pose_complete(&grab,"work_ready") && !GrabTask_PoseClearanceReady()) return;
    ring_target(&x,&y);
    if(!local_move(x,y,"ring_position")) return;
    State=M_RING_MOVE;ContestMission_Report();return;
  }
  if(State==M_RING_MOVE || State==M_CENTER)
  {
    if(motion.state==CHASSIS_MOTION_ERROR) { fail(motion.reason);return; }
    if(motion.state!=CHASSIS_MOTION_DONE || !motion.stop_confirmed || strcmp(motion.reason,"feedback_arrived")) return;
    /* Translation may finish before Base. Both owners must finish before the
     * next visual operation or departure route may take their resources. */
    if(!pose_complete(&grab,State==M_CENTER?"travel_ready":"work_ready"))return;
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
    /* Heading is observed once on station arrival. Ring changes and the
     * processing takeback pass retain it; per-item visual alignment follows. */
    else State=M_TRANSFER_REQUEST;
    return;
  }
  if(State==M_TRANSFER)
  {
    bool storing=Station==4U || (Station==2U && Pass);
    if(grab.state!=GRAB_COMPLETE) return;
    if((storing && grab.slot!=Item+1) || strcmp(grab.result,storing?"stored":"released")) { fail("transfer_result_mismatch");return; }
    if(RingOnly){State=M_DONE;Reason="ring_pickplace_complete";ContestMission_Report();return;}
    /* All station placements retain the first successful prealignment anchor. */
    if(Station!=4U && !station_anchor_capture())return;
    if(storing) Stored++;else Placed++;
    (void)ContestScreen_ProgressSet((uint8_t)Stored,(uint8_t)Placed);
    Item++;
    RingAlignmentSaved=false;
    if(Single) { State=M_REAR_REQUEST;return; }
    if(Item==3U && Station==2U && Pass==0U) { Item=0;Pass=1; } /* all3 placed before any takeback */
    State=Item==3U?M_REAR_REQUEST:Station==4U?M_TRANSFER_REQUEST:M_CLEAR_REQUEST;
    return;
  }
  if(State==M_LEAVE)
  {
    bool complete=pose_complete(&grab,"travel_ready");
    if(Single) {
      if(complete){State=M_DONE;Reason="single_piece_stored";ContestMission_Report();}
      return;
    }
    if(!complete && !GrabTask_PoseClearanceReady()) return;
    if(!local_move(CenterX,CenterY,"return_station_center")) return;
    State=M_CENTER;ContestMission_Report();
  }
}
