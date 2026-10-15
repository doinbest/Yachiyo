#include "GrabRoute.h"
#include "contest_mission.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_config.h"
#include "chassis_localization.h"
#include "contest_screen.h"
#include "QR.h"
#include "console_tx.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { GR_IDLE, GR_DRIVING, GR_SCAN_WAIT, GR_DISPLAY_WAIT, GR_HANDOFF, GR_ERROR, GR_CANCELLED, GR_DONE } GrabRoute_State;
static GrabRoute_State State;
static uint32_t StartTick, WaitTick, CodeSequence;
static const char *Reason;
static bool QrTest,QrLowerStart;
static unsigned QrPhase,QrFinalPhase;
static uint32_t QrSequence;
static char QrCode[QR_TASK_CODE_BUFFER_SIZE];
static const char *QrReason;
static float QrDriveSpeed,QrScanY;
static bool QrFeedbackStarted;
static bool QrOriginReady(const ChassisLocalization_Status_t *p)
{
  float angle=p->origin.yaw_rad-(QrLowerStart?CHASSIS_MODEL_PI:-CHASSIS_MODEL_PI/2);
  return p->origin_valid && fabsf(p->origin.x_mm-2250)<0.1f &&
    fabsf(p->origin.y_mm-(QrLowerStart?150:2250))<0.1f &&
    fabsf(atan2f(sinf(angle),cosf(angle)))<0.001f;
}
static void QrReport(void)
{
  static const char *names[]={"idle","driving","scan","stopping","done","cancelled","error","preparing"};
  char text[220];int n=snprintf(text,sizeof text,
    "%s grab qr state=%s code=%s reason=%s elapsed_ms=%lu\r\n",
    QrPhase==6?"ERR":"OK",names[QrPhase],QrCode[0]?QrCode:"none",QrReason,
    (unsigned long)(HAL_GetTick()-StartTick));
  if(n>0 && n<(int)sizeof text)(void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)n);
}
static const char TestCode[] = "156+123+516+231";
bool GrabRoute_StartPlanned(const RadarPlan_t *plan,float speed)
{
  if(GrabRoute_IsBusy() || GrabTask_IsBusy() || !GrabTask_ConfigReady("pick")) return false;
  if(!ChassisRoute_PlanStart(plan,speed,true)) return false;
  ContestMission_Init();QrTest=false;
  StartTick=HAL_GetTick();CodeSequence=0U;State=GR_DRIVING;Reason="to_scan";
  return true;
}

static void Report(uint8_t Error)
{
  static const char *Names[]={"idle","driving","scan_wait","display_wait","handoff","error","cancelled","done"};
  ChassisRoute_Status_t Route;
  char Text[240];
  int Length;
  ChassisRoute_StatusGet(&Route);
  Length=snprintf(Text,sizeof(Text),"%s grab route state=%s segment=%lu source=simulated code=%s elapsed_ms=%lu reason=%s\r\n",
      Error?"ERR":"OK",Names[State],(unsigned long)Route.segment,TestCode,
      (unsigned long)(HAL_GetTick()-StartTick),Reason);
  if(Length>0 && Length<(int)sizeof(Text)) (void)ConsoleTx_Write((const uint8_t*)Text,(uint16_t)Length);
}

void GrabRoute_Init(void)
{
  QrTest=false;QrPhase=0;QrReason="idle";QrCode[0]=0;
  State=GR_IDLE;StartTick=WaitTick=CodeSequence=0U;Reason="idle";
  ContestMission_Init();
}

bool GrabRoute_IsBusy(void)
{
  return (QrTest && ((QrPhase>=1 && QrPhase<=3) || QrPhase==7)) || ContestMission_HasPendingWork() || State==GR_DRIVING || State==GR_SCAN_WAIT || State==GR_DISPLAY_WAIT || State==GR_HANDOFF;
}
bool GrabRoute_CanConfigure(void) { return ContestMission_CanConfigure(); }

void GrabRoute_Stop(void)
{
  if(QrTest && ((QrPhase>=1 && QrPhase<=3) || QrPhase==7))
  {
    (void)ChassisRoute_Cancel();GrabTask_Stop();
    QrFinalPhase=5;QrPhase=3;QrReason="operator_stop";QrReport();return;
  }
  if(ContestMission_HasPendingWork()){ContestMission_Stop();return;}
  if(GrabRoute_IsBusy())
  {
    if(State!=GR_HANDOFF) (void)ChassisRoute_Cancel();
    State=GR_CANCELLED;Reason="operator_stop";
  }
  if(GrabTask_IsBusy()) GrabTask_Stop();
}

static void Fail(const char *Why)
{
  State=GR_ERROR;Reason=Why;
  (void)ChassisRoute_Cancel();
  Report(1U);
}

void GrabRoute_Process(void)
{
  ChassisRoute_Status_t Route;
  QR_SnapshotTypeDef Code;
  if(QrTest)
  {
    if(QrPhase==7)
    {
      ChassisLocalization_Status_t p;ChassisLocalization_Get(&p);
      if(!QrOriginReady(&p)) { QrPhase=6;QrReason="origin_required";QrReport();return; }
      if(!QrFeedbackStarted)
      {
        if(!Mecanum_Feedback_Select(0))return;
        Mecanum_Feedback_Enable(true);QrFeedbackStarted=true;
      }
      if(!p.feedback_valid || !p.speed_valid || HAL_GetTick()-p.feedback_tick>600U)return;
      if(!ChassisRoute_QrTestStart(QrDriveSpeed,QrScanY))
      { ChassisRoute_StatusGet(&Route);QrPhase=6;QrReason=Route.reason; }
      else { QrPhase=1;QrReason="to_scan"; }
      QrReport();return;
    }
    GrabTask_Status_t grab;GrabTask_StatusGet(&grab);ChassisRoute_StatusGet(&Route);
    if(QrPhase==3)
    {
      if(!ChassisRoute_IsBusy() && !GrabTask_IsBusy()) { QrPhase=QrFinalPhase;QrReport(); }
      return;
    }
    if(QrPhase!=1 && QrPhase!=2)return;
    QR_SnapshotGet(&Code);
    if(Code.Valid && Code.Source==QR_SOURCE_UART && Code.Sequence!=QrSequence)
      memcpy(QrCode,Code.Code,sizeof QrCode);
    if(!strcmp(Route.state,"error") || !strcmp(Route.state,"cancelled") || grab.state==GRAB_ERROR)
    {
      QrReason=grab.state==GRAB_ERROR?grab.reason:Route.reason;
      (void)ChassisRoute_Cancel();GrabTask_Stop();QrFinalPhase=6;QrPhase=3;QrReport();return;
    }
    if(QrPhase==1 && !strcmp(Route.state,"done") && Route.stop_confirmed)
    {
      if(!GrabTask_QrScanStart(QrSequence))return; /* Outstanding reads drain before admission. */
      QrPhase=2;QrReason="qr_wait";QrReport();return;
    }
    if(QrPhase==1 && !strcmp(Route.reason,"qr_scan_turn") && strcmp(QrReason,"qr_scan_turn"))
    { QrReason="qr_scan_turn";QrReport(); }
    if(QrPhase==2)
    {
      if(!GrabTask_IsBusy())
      {
        QrPhase=grab.stop_confirmed && !strcmp(grab.result,"qr_scanned")?4:6;
        QrReason=grab.reason;QrReport();return;
      }
      if(strcmp(QrReason,grab.reason)) { QrReason=grab.reason;QrReport(); }
    }
    return;
  }
  if(ContestMission_HasPendingWork()) { ContestMission_Process();return; }
  if(!GrabRoute_IsBusy()) return;
  if(State==GR_HANDOFF)
  {
    GrabTask_Status_t Grab;
    GrabTask_StatusGet(&Grab);
    if(Grab.state==GRAB_COMPLETE && !strcmp(Grab.result,"stored"))
    { State=GR_DONE;Reason="single_piece_stored";Report(0U); }
    else if(Grab.state==GRAB_ERROR || (Grab.state==GRAB_IDLE && !strcmp(Grab.result,"cancelled")))
    { State=GR_ERROR;Reason=Grab.reason;Report(1U); }
    return; /* This scope ends after one storage action; it never starts a second piece. */
  }
  ChassisRoute_StatusGet(&Route);
  if(!strcmp(Route.state,"error") || !strcmp(Route.state,"cancelled"))
  { Fail(Route.reason);return; }
  /* A normal arrival includes deceleration and feedback-confirmed stopping. */
  if(!strcmp(Route.state,"stopping")) return;
  if(CodeSequence)
  {
    QR_SnapshotGet(&Code);
    if(Code.Sequence!=CodeSequence || Code.Source!=QR_SOURCE_SIMULATED || strcmp(Code.Code,TestCode))
    { Fail("task_code_changed");return; }
  }
  if(State==GR_DRIVING)
  {
    if(strcmp(Route.state,"station")) return;
    if(!Route.stop_confirmed) { Fail("station_stop_unconfirmed");return; }
    if(Route.station==5U && Route.visit==1U)
    { State=GR_SCAN_WAIT;WaitTick=HAL_GetTick();Reason="simulated_scan_wait";Report(0U); }
    else if(Route.station==4U && Route.visit==2U)
    {
      if(!ChassisRoute_StationFinish()) { Fail("handoff_failed");return; }
      if(!GrabTask_Start("pick")) { Fail("grab_start_failed");return; }
      State=GR_HANDOFF;Reason="single_pick_started";Report(0U);
    }
    return;
  }
  if(strcmp(Route.state,"station") ||
     Route.station!=5U || Route.visit!=1U || !Route.stop_confirmed)
  { Fail("station_lost");return; }
  if(State==GR_SCAN_WAIT && HAL_GetTick()-WaitTick>=1000U)
  {
    if(!QR_SimulatedSet(TestCode) || QR_ColorGet(0U,0U)!=1U ||
       !ContestScreen_ProgressSet(0U,0U) || !ContestScreen_TaskCodeRefresh(TestCode))
    { Fail("task_code_invalid");return; }
    QR_SnapshotGet(&Code);CodeSequence=Code.Sequence;
    State=GR_DISPLAY_WAIT;WaitTick=HAL_GetTick();Reason="screen_tx_pending";Report(0U);
  }
  else if(State==GR_DISPLAY_WAIT)
  {
    if(ContestScreen_HasError()) { Fail("screen_tx_error");return; }
    if(ContestScreen_TaskCodeSent())
    {
      if(!ChassisRoute_StationResume()) { Fail("route_resume_failed");return; }
      State=GR_DRIVING;Reason="to_material";Report(0U);
    }
    else if(HAL_GetTick()-WaitTick>=5000U) Fail("screen_tx_timeout");
  }
}

bool GrabRoute_Command(unsigned Count,char *Tokens[])
{
  char *End;
  float Speed;
  if(Count<2U || strcmp(Tokens[0],"grab")) return false;
  if(Count>=3U && !strcmp(Tokens[1],"ring") && !strcmp(Tokens[2],"take"))
  {
    if(Count!=4U || Tokens[3][0]<'1' || Tokens[3][0]>'3' || Tokens[3][1])
    {static const char error[]="ERR grab ring_take reason=invalid_slot\r\n";(void)ConsoleTx_Write((const uint8_t*)error,sizeof error-1);return true;}
    if(!ContestMission_StartRingTake((uint8_t)(Tokens[3][0]-'0'))){
      char line[160];snprintf(line,sizeof line,"ERR grab ring_take reason=%s\r\n",ContestMission_Reason());(void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)strlen(line));
    }
    return true;
  }
  if(!strcmp(Tokens[1],"heading"))
  {
    const char *error=NULL;unsigned station=0;Speed=5;float polarity=-1;uint8_t source=STATION_HEADING_RING;
    bool probe=Count>2 && !strcmp(Tokens[2],"angle");
    unsigned station_index=probe?3U:2U,speed_index=probe?5U:3U;
    if(Count<(probe?4U:3U) || Count>speed_index+2)error="format";
    else {
      station=!strcmp(Tokens[station_index],"proc")?2:!strcmp(Tokens[station_index],"temp")?3:0;
      if(!station)error="heading_station_required";
      if(probe && Count>4){polarity=strtof(Tokens[4],&End);if(End==Tokens[4] || *End)error="heading_probe_polarity";}
      if(Count>speed_index){Speed=strtof(Tokens[speed_index],&End);if(End==Tokens[speed_index] || *End)error="heading_speed_range";}
      if(Count>speed_index+1){
        if(!strcmp(Tokens[speed_index+1],"line"))source=STATION_HEADING_LINE;
        else if(strcmp(Tokens[speed_index+1],"ring"))error="heading_source_required";
      }
    }
    if(!error && GrabRoute_IsBusy())error="busy";
    if(!error){
      bool accepted=probe?ContestMission_StartHeadingProbeSource(station,polarity,Speed,source):ContestMission_StartHeadingTestSource(station,Speed,source);
      if(accepted){QrTest=false;return true;}
      error=ContestMission_Reason();
    }
    char line[160];int n=snprintf(line,sizeof line,"ERR grab heading reason=%s\r\n",error);
    if(n>0 && n<(int)sizeof line)(void)ConsoleTx_Write((const uint8_t*)line,(uint16_t)n);
    return true;
  }
  if(!strcmp(Tokens[1],"qr"))
  {
    if(Count==3 && !strcmp(Tokens[2],"status")) { QrReport();return true; }
    if(Count==3 && !strcmp(Tokens[2],"stop")) { GrabRoute_Stop();return true; }
    if(Count>=3 && Count<=5 && !strcmp(Tokens[2],"begin"))
    {
      float y;QR_SnapshotTypeDef qr;Speed=50;
      bool lower=false;unsigned speed_index=3U;
      if(Count>3 && (!strcmp(Tokens[3],"lower") || !strcmp(Tokens[3],"upper")))
      { lower=!strcmp(Tokens[3],"lower");speed_index=4U; }
      if(Count>speed_index+1U)goto qr_format;
      if(Count>speed_index)
      { Speed=strtof(Tokens[speed_index],&End);if(End==Tokens[speed_index] || *End)Speed=NAN; }
      if(GrabRoute_IsBusy() || GrabTask_IsBusy() || ChassisRoute_IsBusy())
      { static const char busy[]="ERR grab qr reason=busy\r\n";(void)ConsoleTx_Write((const uint8_t*)busy,sizeof busy-1);return true; }
      QrTest=true;QrLowerStart=lower;QrPhase=6;QrCode[0]=0;StartTick=HAL_GetTick();
      if(!GrabTask_QrReady(&y))QrReason="home_or_scan_config_required";
      else if(!isfinite(Speed) || Speed<10 || Speed>200)QrReason="speed_range";
      else
      {
        ChassisLocalization_Status_t p;ChassisLocalization_Get(&p);
        if(!QrOriginReady(&p))QrReason="origin_required";
        else
        {
          QR_SnapshotGet(&qr);QrSequence=qr.Sequence;ContestMission_Init();
          QrDriveSpeed=Speed;QrScanY=y;QrFeedbackStarted=false;
          QrPhase=7;QrReason="feedback_wait";
        }
      }
      QrReport();return true;
    }
qr_format:
    ;
    static const char format[]="ERR grab qr format: begin [upper|lower] [10..200 mm/s] | status | stop\r\n";
    (void)ConsoleTx_Write((const uint8_t*)format,sizeof format-1);return true;
  }
  if(Count==2U && !strcmp(Tokens[1],"status"))
  { if(QrTest)QrReport();else if(ContestMission_HasRun()) ContestMission_Report();else Report(0U);return false; }
  if(Count==3U && !strcmp(Tokens[1],"mission") && !strcmp(Tokens[2],"status"))
  {ContestMission_Report();return true;}
  if(Count==3U && !strcmp(Tokens[1],"route") && !strcmp(Tokens[2],"status"))
  { if(ContestMission_HasRun()) ContestMission_Report();else Report(0U);return true; }
  if(!strcmp(Tokens[1],"mission"))
  {
    if(Count==3U && !strcmp(Tokens[2],"pause"))
    {ContestMission_Stop();ContestMission_Report();return true;}
    if(Count==3U && !strcmp(Tokens[2],"stop"))
    {ContestMission_Cancel();ContestMission_Report();return true;}
    if(Count==3U && !strcmp(Tokens[2],"continue"))
    { (void)ContestMission_Continue();ContestMission_Report();return true; }
    ContestMission_Request_t request={MISSION_CODE_QR,"",CHASSIS_ROUTE_DEFAULT_SPEED_MM_S,100,RADAR_START_LOWER,false,true,false,CHASSIS_PROFILE_LINEAR};
    const char *error=NULL;unsigned first=3;
    if(Count<3 || (strcmp(Tokens[2],"qr") && strcmp(Tokens[2],"fixed")))error="source_required";
    else if(!strcmp(Tokens[2],"fixed"))
    {
      request.code_source=MISSION_CODE_FIXED;first=4;
      if(Count<4 || strlen(Tokens[3])!=15)error="task_code_invalid";
      else memcpy(request.fixed_code,Tokens[3],16);
    }
    if(!error && Count>first+6)error="format";
    if(!error && Count>first){request.route_speed_mm_s=strtof(Tokens[first],&End);if(End==Tokens[first] || *End)error="speed_range";}
    if(!error && Count>first+1){request.turn_speed_deg_s=strtof(Tokens[first+1],&End);if(End==Tokens[first+1] || *End)error="turn_speed_range";}
    if(!error && Count>first+2){
      if(!strcmp(Tokens[first+2],"upper"))request.start_id=RADAR_START_UPPER;
      else if(strcmp(Tokens[first+2],"lower"))error="start_required";
    }
    if(!error && Count>first+3){
      if(!strcmp(Tokens[first+3],"radar"))request.radar_enabled=true;
      else if(strcmp(Tokens[first+3],"fixed"))error="route_mode_required";
    }
    if(!error && Count>first+4){
      if(!strcmp(Tokens[first+4],"vhigh")){request.align_raised=true;request.visual_heading_enabled=true;}
      else if(!strcmp(Tokens[first+4],"vlow")){request.align_raised=false;request.visual_heading_enabled=true;}
      else if(!strcmp(Tokens[first+4],"raised"))request.align_raised=true;
      else if(!strcmp(Tokens[first+4],"lowered"))request.align_raised=false;
      else error="alignment_mode_required";
    }
    if(!error && Count>first+5){
      if(!strcmp(Tokens[first+5],"cosine"))request.profile=CHASSIS_PROFILE_COSINE;
      else if(strcmp(Tokens[first+5],"linear"))error="profile_required";
    }
    if(!error && (GrabRoute_IsBusy() || GrabTask_IsBusy()))error="busy";
    if(!error && !ContestMission_Start(NULL,&request))error=ContestMission_Reason();
    if(error)
    {
      char text[180];int n=snprintf(text,sizeof text,"ERR grab mission reason=%s\r\n",error);
      if(n>0 && n<(int)sizeof text)(void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)n);
    }
    else QrTest=false;
    return true;
  }
  if(strcmp(Tokens[1],"route")) return false;
  if(GrabRoute_IsBusy() || GrabTask_IsBusy() || ChassisRoute_IsBusy())
  { static const char Busy[]="ERR grab route busy\r\n";(void)ConsoleTx_Write((const uint8_t*)Busy,sizeof(Busy)-1U);return true; }
  if(Count!=2U && Count!=3U) { Reason="format";Report(1U);return true; }
  Speed=CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
  if(Count==3U)
  {
    Speed=strtof(Tokens[2],&End);
    if(End==Tokens[2] || *End || !isfinite(Speed) || Speed<10.0f || Speed>CHASSIS_MOTION_MAX_LINEAR_MM_S)
    { Reason="speed_range";Report(1U);return true; }
  }
  QrTest=false;
  if(!ContestMission_StartSingle(Speed))
  {
    char error[220];
    int length;
    Reason=ContestMission_Reason();
    length=snprintf(error,sizeof error,"ERR grab route state=error mode=single source=test code=none color=3 slot=1 elapsed_ms=0 reason=%s\r\n",Reason);
    if(length>0 && length<(int)sizeof error) (void)ConsoleTx_Write((const uint8_t*)error,(uint16_t)length);
  }
  return true;
}
