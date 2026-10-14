#include "chassis_route.h"
#include "chassis_route_profile.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_config.h"
#include "console_tx.h"
#include "mecanum_chassis.h"
#include "GrabTask.h"
#include "contest_mission.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
static ChassisRoute_Status_t Route;
static bool Submitting, Automatic, Stations, FirstScanLeg;
static bool TaskRoute, RequestTask, QrRoute, RequestQr;
static bool QrLowerStart, RequestQrLower, QrPoseStarted;
static ChassisRoute_TaskOptions_t TaskOptions;
static const char *PauseState;
static bool ResumeFeedbackStarted;
static bool RecoveryHold;
static bool PauseMonitoring;
static uint32_t PauseStopTick;
static uint32_t Generation, LegStartTick, LegBudget;
static float RouteSpeed = CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
static float AcceptedYaw;
static ChassisModel_Pose_t StartOrigin;
static uint32_t StartUnits;
static RadarPlan_Point_t Planned[RADAR_PLAN_MAX_POINTS-1U];
static unsigned PlannedCount, PhaseIndex;
static ChassisRoute_Profile_t Profile;
static bool Pending;
static unsigned PendingIndex;
static ChassisRoute_Phase_t PendingPhase;
static ChassisLocalization_Status_t Position;
static ChassisMotion_Status_t Motion;
static ChassisPath Curve;
static bool CurveActive;
static float CurveProgress;
static float angle_error(float a,float b)
{
  float e=fmodf(a-b+180.0f,360.0f);
  return (e<0?e+360.0f:e)-180.0f;
}
void ChassisRoute_Init(void)
{
  memset(&Route,0,sizeof(Route));Route.state="idle";Route.reason="idle";
  Route.phase="idle";Route.direction="none";Route.total=CHASSIS_ROUTE_TARGET_COUNT;
  Submitting=Automatic=Stations=FirstScanLeg=false;PlannedCount=PhaseIndex=0;
  Pending=false;TaskRoute=RequestTask=QrRoute=RequestQr=false;
  memset(&TaskOptions,0,sizeof TaskOptions);PauseState=NULL;ResumeFeedbackStarted=RecoveryHold=PauseMonitoring=false;
  QrLowerStart=RequestQrLower=false;
  CurveActive=false;CurveProgress=0;
  RouteSpeed=CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;AcceptedYaw=90;
}
bool ChassisRoute_IsBusy(void)
{
  return Route.state && (!strcmp(Route.state,"running") || !strcmp(Route.state,"stopping") || !strcmp(Route.state,"waiting"));
}
bool ChassisRoute_IsSubmitting(void) { return Submitting; }
bool ChassisRoute_StationReserved(void)
{
  return Stations && (ChassisRoute_IsBusy() || (Route.state &&
      (!strcmp(Route.state,"station") || !strcmp(Route.state,"work") || !strcmp(Route.state,"paused") || !strcmp(Route.state,"pausing"))));
}
static void stop_route(const char *reason)
{
  (void)ChassisMotion_Stop(0);Route.state="stopping";Route.reason=reason;Route.stop_confirmed=false;
}
static uint32_t remaining(void)
{
  uint32_t elapsed=(uint32_t)(HAL_GetTick()-LegStartTick);
  return elapsed<LegBudget?LegBudget-elapsed:0U;
}
static bool pose_can_overlap(const ChassisRoute_Phase_t *phase)
{
  /* Only the pose task can qualify, with live retracted X/lifted Z evidence.
   * A new chassis turn still waits for the whole arm pose. */
  return !phase->turn && (strcmp(phase->direction,"curve") ||
    (CurveProgress>=Curve.turn_end && fabsf(Route.error_heading_deg)<=CHASSIS_DISTANCE_HEADING_DEG)) &&
    GrabTask_PoseClearanceReady();
}
static bool submit_phase(const ChassisRoute_Phase_t *phase)
{
  bool ok;
  uint32_t timeout=remaining();
  if(!timeout) { stop_route("route_timeout");return false; }
  if(TaskRoute && GrabTask_IsBusy() && !pose_can_overlap(phase))
  { Route.state="running";Route.reason="grab_task_busy";return true; }
  Submitting=true;
  if(!strcmp(phase->direction,"curve"))
    ok=ChassisMotion_PathStart(&Curve,phase->speed_mm_s,TaskOptions.turn_speed_deg_s,timeout,CurveProgress);
  else if(phase->turn) ok=TaskRoute?ChassisMotion_TurnToSpeed(phase->yaw_deg,TaskOptions.turn_speed_deg_s,timeout):ChassisMotion_TurnTo(phase->yaw_deg,timeout);
  else ok=ChassisMotion_MoveTo(phase->x_mm,phase->y_mm,phase->yaw_deg,phase->speed_mm_s,timeout);
  Submitting=false;
  ChassisMotion_StatusGet(&Motion);
  if(!ok) {
    if(TaskRoute && Motion.reason && (!strcmp(Motion.reason,"busy") || !strcmp(Motion.reason,"grab_task_busy")))
    { Route.state="running";Route.reason=Motion.reason;return true; }
    Pending=false;Route.state="error";Route.reason=Motion.reason;return false;
  }
  PhaseIndex=PendingIndex;Pending=false;
  CurveActive=!strcmp(phase->direction,"curve");
  Route.action_id=Motion.action_id;Route.phase=phase->phase;Route.direction=phase->direction;
  Route.phase_speed_mm_s=phase->speed_mm_s;
  Route.phase_target_x_mm=phase->x_mm;Route.phase_target_y_mm=phase->y_mm;Route.phase_target_yaw_deg=phase->yaw_deg;
  PauseMonitoring=false;
  Route.state="running";Route.reason=phase->phase;Route.stop_confirmed=false;
  return true;
}
static void defer_phase(const ChassisRoute_Phase_t *phase,unsigned index)
{
  PendingPhase=*phase;PendingIndex=index;Pending=true;
}
static bool queue_phase(const ChassisRoute_Phase_t *phase,unsigned index)
{
  defer_phase(phase,index);
  return submit_phase(&PendingPhase);
}
static bool repair_position(void)
{
  ChassisRoute_Phase_t phase={false,"move","diagonal",Route.target_x_mm,Route.target_y_mm,
                             Route.target_yaw_deg,0};
  float dx=Route.target_x_mm-Position.observer.feedback.x_mm;
  float dy=Route.target_y_mm-Position.observer.feedback.y_mm;
  float yaw=Route.target_yaw_deg*CHASSIS_MODEL_PI/180;
  float forward=dx*cosf(yaw)+dy*sinf(yaw),lateral=-dx*sinf(yaw)+dy*cosf(yaw);
  phase.speed_mm_s=fminf(RouteSpeed,CHASSIS_DISTANCE_CORRECTION_SPEED_MM_S);
  if(fabsf(lateral)<0.01f) phase.direction=forward>=0?"forward":"reverse";
  else if(fabsf(forward)<0.01f) phase.direction="lateral";
  return queue_phase(&phase,Profile.count);
}
static bool submit(unsigned index)
{
  RadarPlan_Point_t from,to;
  if(Route.planned)
  {
    to=Planned[index];
    if(index) from=Planned[index-1U];
    else { (void)ChassisRouteProfile_FixedPoint(0,&from);if(TaskRoute && TaskOptions.start_id==RADAR_START_UPPER)from.y_mm=2250; }
  }
  else
  {
    (void)ChassisRouteProfile_FixedPoint(index,&from);
    (void)ChassisRouteProfile_FixedPoint(index+1U,&to);
  }
  if(TaskRoute && TaskOptions.scan_qr)
  {
    if(to.station==5U && to.visit==1U)to.y_mm=(int16_t)lroundf(TaskOptions.qr_scan_y_mm);
    if(from.station==5U && from.visit==1U)from.y_mm=(int16_t)lroundf(TaskOptions.qr_scan_y_mm);
  }
  if(QrRoute)
  {
    if(!index)from=(RadarPlan_Point_t){2250,QrLowerStart?150:2250,1,0};
    unsigned move_phase=QrLowerStart && index?1U:0U;
    float heading=QrLowerStart?(index?90:180):-90;
    memset(&Profile,0,sizeof Profile);Profile.count=move_phase+1U;Profile.final_yaw_deg=heading;
    if(move_phase)
      Profile.phases[0]=(ChassisRoute_Phase_t){true,"qr_turn","turn",from.x_mm,from.y_mm,90,0};
    Profile.phases[move_phase]=(ChassisRoute_Phase_t){false,"qr_move",index?"forward":"diagonal",to.x_mm,to.y_mm,heading,RouteSpeed};
    Profile.nominal_time_s=hypotf(to.x_mm-from.x_mm,to.y_mm-from.y_mm)/RouteSpeed;
    if(move_phase)Profile.nominal_time_s+=90.0f/CHASSIS_MOTION_MAX_OMEGA_DEG_S;
    if(index+1U==PlannedCount)
    {
      Profile.final_yaw_deg=angle_error(heading+180.0f,0);
      Profile.phases[Profile.count++]=(ChassisRoute_Phase_t){true,"qr_scan_turn","turn",
        to.x_mm,to.y_mm,Profile.final_yaw_deg,0};
      Profile.nominal_time_s+=180.0f/CHASSIS_MOTION_MAX_OMEGA_DEG_S;
    }
  }
  else if(!(TaskRoute?ChassisRouteProfile_BuildTaskEntry(&from,&to,AcceptedYaw,RouteSpeed,FirstScanLeg,
      ChassisMotion_ProfileGet()==CHASSIS_PROFILE_COSINE,&Profile):
      ChassisRouteProfile_Build(&from,&to,AcceptedYaw,RouteSpeed,FirstScanLeg,&Profile)))
  { Route.state="error";Route.reason="route_profile_invalid";return false; }
  /* Only remove a neutral stop when both generated legs are single plain
   * translations on exactly the same line and heading. Dock/turn profiles
   * remain barriers, including the confirmed QR entrance. */
  while(TaskRoute && !from.station && !to.station && index+1U<Route.total &&
        Profile.count==1U && !Profile.phases[0].turn && !strcmp(Profile.phases[0].phase,"move"))
  {
    RadarPlan_Point_t next;ChassisRoute_Profile_t extension;
    if(Route.planned)next=Planned[index+1U];
    else if(!ChassisRouteProfile_FixedPoint(index+2U,&next))break;
    if(next.station)break;
    float ax=to.x_mm-from.x_mm,ay=to.y_mm-from.y_mm;
    float bx=next.x_mm-to.x_mm,by=next.y_mm-to.y_mm;
    if(fabsf(ax*by-ay*bx)>0.01f || ax*bx+ay*by<=0 ||
       !ChassisRouteProfile_BuildTaskEntry(&to,&next,Profile.final_yaw_deg,RouteSpeed,FirstScanLeg,
         ChassisMotion_ProfileGet()==CHASSIS_PROFILE_COSINE,&extension) ||
       extension.count!=1U || extension.phases[0].turn || strcmp(extension.phases[0].phase,"move") ||
       fabsf(angle_error(Profile.phases[0].yaw_deg,extension.phases[0].yaw_deg))>0.01f)break;
    float speed=fminf(Profile.phases[0].speed_mm_s,extension.phases[0].speed_mm_s);
    Profile.nominal_time_s=(hypotf(next.x_mm-from.x_mm,next.y_mm-from.y_mm))/speed;
    Profile.phases[0]=extension.phases[0];Profile.phases[0].speed_mm_s=speed;
    Profile.final_yaw_deg=extension.final_yaw_deg;to=next;index++;
  }
  if(TaskRoute && TaskOptions.scan_qr && FirstScanLeg)
  {
    /* Formal scanning keeps the lane heading; scanner orientation belongs to
     * the Base QR pose. Standalone QrRoute tests retain their scan half-turn. */
  }
  if(TaskRoute) for(unsigned j=0;j<Profile.count;j++)if(Profile.phases[j].turn)
    Profile.nominal_time_s+=180.0f/TaskOptions.turn_speed_deg_s;
  bool has_curve=Profile.count && !strcmp(Profile.phases[Profile.count-1].direction,"curve");
  CurveActive=false;
  CurveProgress=0;
  if(has_curve){
    unsigned kind=!strcmp(Profile.phases[Profile.count-1].phase,"entry_curve")?1:from.x_mm==2100?2:3;
    if(!ChassisPath_Build(&Curve,kind,from.x_mm,from.y_mm,to.x_mm,to.y_mm,RouteSpeed))return false;
    Profile.nominal_time_s=Curve.length/RouteSpeed+180/TaskOptions.turn_speed_deg_s+5;
  }
  LegStartTick=HAL_GetTick();
  LegBudget=(uint32_t)fminf(CHASSIS_DISTANCE_MAX_TIMEOUT_MS,
               fmaxf(CHASSIS_DISTANCE_TIMEOUT_MS,2000.0f*Profile.nominal_time_s+10000.0f));
  PhaseIndex=0;
  Route.segment=index+1U;Route.target_x_mm=to.x_mm;Route.target_y_mm=to.y_mm;
  Route.target_yaw_deg=Profile.final_yaw_deg;Route.station=to.station;Route.visit=to.visit;
  Route.speed_mm_s=RouteSpeed;
  ChassisLocalization_Get(&Position);
  return Profile.count?queue_phase(&Profile.phases[0],0):repair_position();
}
static bool start_at_speed(float speed, bool automatic, bool planned)
{
  unsigned i;
  if (!isfinite(speed) || speed < 10.0f || speed > CHASSIS_MOTION_MAX_LINEAR_MM_S) return false;
  if(ChassisRoute_IsBusy() || ChassisRoute_StationReserved() || ChassisMotion_IsBusy()) return false;
  ChassisLocalization_Get(&Position);
  float origin_y=(RequestQr && !RequestQrLower) || (RequestTask && TaskOptions.start_id==RADAR_START_UPPER)?2250:150;
  float origin_yaw=RequestQr?(RequestQrLower?180:-90):(RequestTask && TaskOptions.start_id==RADAR_START_UPPER?-90:180);
  if(!Position.origin_valid || !Position.feedback_valid || !Position.speed_valid ||
     (uint32_t)(HAL_GetTick()-Position.feedback_tick)>600U ||
      fabsf(Position.origin.x_mm-2250)>0.1f || fabsf(Position.origin.y_mm-origin_y)>0.1f ||
      fabsf(angle_error(Position.origin.yaw_rad*180/CHASSIS_MODEL_PI,origin_yaw))>0.1f ||
      hypotf(Position.observer.feedback.x_mm-2250,Position.observer.feedback.y_mm-origin_y)>CHASSIS_DISTANCE_TOLERANCE_MM ||
      fabsf(angle_error(Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI,origin_yaw))>CHASSIS_DISTANCE_HEADING_DEG)
  { Route.state="error";Route.reason="start_origin_or_feedback";return false; }
  TaskRoute=RequestTask;QrRoute=RequestQr;
  QrLowerStart=RequestQr && RequestQrLower;
  QrPoseStarted=TaskRoute && TaskOptions.start_id==RADAR_START_UPPER;RouteSpeed=speed;Automatic=automatic;Stations=false;AcceptedYaw=origin_yaw;FirstScanLeg=!planned;
  if(planned) for(i=0;i<PlannedCount;i++)
    if(Planned[i].station==5U && Planned[i].visit==1U) FirstScanLeg=true;
  Route.planned=planned;Route.total=planned?PlannedCount:CHASSIS_ROUTE_TARGET_COUNT;
  Generation=Position.generation;
  StartOrigin=Position.origin;StartUnits=Position.units_per_rev;
  return submit(0);
}
bool ChassisRoute_QrTestStart(float speed,float scan_y)
{
  if(ChassisRoute_IsBusy() || ChassisRoute_StationReserved() || ChassisMotion_IsBusy())return false;
  if(!isfinite(scan_y) || scan_y<1100 || scan_y>1300 ||
     !isfinite(speed) || speed<10 || speed>200)return false;
  ChassisLocalization_Get(&Position);
  RequestQrLower=fabsf(Position.origin.y_mm-150)<0.1f;
  PlannedCount=2;Planned[0]=(RadarPlan_Point_t){2100,RequestQrLower?300:2100,0,0};
  Planned[1]=(RadarPlan_Point_t){2100,(int16_t)lroundf(scan_y),5,1};
  RequestQr=true;
  bool accepted=start_at_speed(speed,true,true);
  RequestQr=RequestQrLower=false;return accepted;
}
bool ChassisRoute_PlanStart(const RadarPlan_t *plan, float speed, bool station_mode)
{
  unsigned i;
  if(!plan || !plan->valid || plan->count<2U || plan->count>RADAR_PLAN_MAX_POINTS ||
     ChassisRoute_IsBusy() || ChassisRoute_StationReserved() || ChassisMotion_IsBusy()) return false;
  if(plan->points[0].x_mm!=2250 || plan->points[0].y_mm!=(RequestTask && TaskOptions.start_id==RADAR_START_UPPER?2250:150)) return false;
  for(i=1;i<plan->count;i++)
    if(plan->points[i].x_mm<0 || plan->points[i].x_mm>2400 ||
       plan->points[i].y_mm<0 || plan->points[i].y_mm>2400) return false;
  PlannedCount=plan->count-1U;
  memcpy(Planned,plan->points+1,PlannedCount*sizeof(Planned[0]));
  if(!start_at_speed(speed,true,true)) return false;
  Stations=station_mode;
  return true;
}
static bool station_start(float speed)
{
  if (!start_at_speed(speed,true,false)) return false;
  Stations=true;
  return true;
}
bool ChassisRoute_TaskStart(const RadarPlan_t *plan,float speed)
{ const ChassisRoute_TaskOptions_t options={false,1200,100,RADAR_START_LOWER};return ChassisRoute_TaskStartEx(plan,speed,&options); }
bool ChassisRoute_TaskStartEx(const RadarPlan_t *plan,float speed,const ChassisRoute_TaskOptions_t *options)
{
  if(ChassisRoute_IsBusy() || ChassisRoute_StationReserved())return false;
  if(!options || !isfinite(options->qr_scan_y_mm) || options->qr_scan_y_mm<1100 || options->qr_scan_y_mm>1300 ||
     !isfinite(options->turn_speed_deg_s) || options->turn_speed_deg_s<5 || options->turn_speed_deg_s>CHASSIS_MOTION_MAX_OMEGA_DEG_S)return false;
  TaskOptions=*options;PauseState=NULL;ResumeFeedbackStarted=RecoveryHold=false;RequestTask=true;
  bool ok;
  if(!plan && options->start_id==RADAR_START_UPPER){
    PlannedCount=CHASSIS_ROUTE_TARGET_COUNT;
    for(unsigned i=0;i<PlannedCount;i++)ChassisRouteProfile_FixedPoint(i+1,&Planned[i]);
    Planned[PlannedCount-2].y_mm=2100;Planned[PlannedCount-1].y_mm=2250;
    ok=start_at_speed(speed,true,true);if(ok)Stations=true;
  }else ok=plan?ChassisRoute_PlanStart(plan,speed,true):station_start(speed);
  RequestTask=false;return ok;
}
bool ChassisRoute_TaskPause(void)
{
  if(!TaskRoute || !Stations)return false;
  if(CurveActive && !Pending)CurveProgress=ChassisMotion_PathProgress();
  ResumeFeedbackStarted=false;
  if(PauseState){ChassisMotion_StatusGet(&Motion);if(Motion.state==CHASSIS_MOTION_ERROR || !strcmp(Route.state,"error"))
    {PauseMonitoring=false;Route.state="pausing";Route.reason="operator_stop";(void)ChassisMotion_Stop(0);}return true;}
  PauseState=Route.state;Route.state="pausing";Route.reason="operator_stop";
  PauseMonitoring=false;
  Route.stop_confirmed=false;(void)ChassisMotion_Stop(0);return true;
}
bool ChassisRoute_TaskResumeReady(void)
{
  if(RecoveryHold || !PauseState || strcmp(Route.state,"paused") || !Route.stop_confirmed || ChassisMotion_IsBusy())return false;
  /* Explicit continue restores reads disabled by bus recovery; it never resets
   * the map origin or admits motion against the pre-recovery cached pose. */
  /* Select resets the four-wheel polling index, so do it once per resume.
   * Repeating it while waiting for fresh positions starves wheels 2..4. */
  if(!ResumeFeedbackStarted)
  {
    if(!Mecanum_Feedback_Select(0))return false;
    ResumeFeedbackStarted=true;
  }
  Mecanum_Feedback_Enable(true);
  ChassisLocalization_Get(&Position);
  Mecanum_Status_t bus;Mecanum_StatusGet(&bus);
  if(!Position.origin_valid || !Position.feedback_valid || !Position.speed_valid || HAL_GetTick()-Position.feedback_tick>600 ||
     bus.locked || bus.stop_pending || bus.error!=MECANUM_ERROR_NONE)return false;
  if(Position.units_per_rev!=StartUnits || fabsf(Position.origin.x_mm-StartOrigin.x_mm)>.1f ||
     fabsf(Position.origin.y_mm-StartOrigin.y_mm)>.1f || fabsf(angle_error(Position.origin.yaw_rad*180/CHASSIS_MODEL_PI,StartOrigin.yaw_rad*180/CHASSIS_MODEL_PI))>.1f)
  {Route.reason="station_reference_changed";return false;}
  return true;
}
bool ChassisRoute_TaskContinue(void)
{
  if(!ChassisRoute_TaskResumeReady())return false;
  Generation=Position.generation;
  const char *resume=PauseState;
  ChassisMotion_StatusGet(&Motion);
  if(!strcmp(resume,"work"))
  {PauseState=NULL;ResumeFeedbackStarted=false;Route.state=resume;Route.reason="resumed";Route.action_id=Motion.action_id;return true;}
  LegStartTick=HAL_GetTick();
  bool ok=Pending?submit_phase(&PendingPhase):(PhaseIndex<Profile.count?queue_phase(&Profile.phases[PhaseIndex],PhaseIndex):repair_position());
  if(ok){PauseState=NULL;ResumeFeedbackStarted=false;}
  else {Route.state="paused";Route.stop_confirmed=true;}
  return ok;
}
void ChassisRoute_TaskRecoveryHold(bool hold)
{
  RecoveryHold=hold;
  if(!hold && PauseState)
  {
    PauseMonitoring=false;Route.stop_confirmed=false;
    Route.state="pausing";Route.reason="bus_recovery_verify";
    (void)ChassisMotion_Stop(0);
  }
}
bool ChassisRoute_WorkBegin(void)
{
  if(!TaskRoute || !Stations || strcmp(Route.state,"station") || !Route.stop_confirmed) return false;
  PauseMonitoring=false;Route.state="work";Route.reason="station_work";return true;
}
bool ChassisRoute_WorkResume(void)
{
  if(!TaskRoute || !Stations || strcmp(Route.state,"work") ||
     (GrabTask_IsBusy() && !GrabTask_PoseClearanceReady()) || ChassisMotion_IsBusy()) return false;
  ChassisLocalization_Get(&Position);
  if(!Position.origin_valid || !Position.feedback_valid || !Position.speed_valid ||
     HAL_GetTick()-Position.feedback_tick>600U)
  { Route.reason="station_feedback_wait";return false; }
  if(Position.generation!=Generation &&
     (Position.units_per_rev!=StartUnits || fabsf(Position.origin.x_mm-StartOrigin.x_mm)>0.1f ||
      fabsf(Position.origin.y_mm-StartOrigin.y_mm)>0.1f ||
      fabsf(angle_error(Position.origin.yaw_rad*180/CHASSIS_MODEL_PI,
                       StartOrigin.yaw_rad*180/CHASSIS_MODEL_PI))>0.1f))
  { Route.state="error";Route.reason="station_reference_changed";return false; }
  ChassisMotion_StatusGet(&Motion);
  /* A resumed station has independent post-stop wheel evidence, even though
   * cancelling a motion deliberately does not claim positional arrival. Do
   * not reuse that evidence after a new local positioning action. */
  bool resumed_stop=false;
  if(PauseMonitoring && Route.stop_confirmed && Motion.action_id==Route.action_id)
  {
    ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
    resumed_stop=stop.tx_complete && stop.wheels_stopped &&
      (uint32_t)(stop.requested_ms-PauseStopTick)<0x80000000U;
  }
  if((!resumed_stop && (Motion.state!=CHASSIS_MOTION_DONE || !Motion.stop_confirmed)) ||
     hypotf(Route.target_x_mm-Position.observer.feedback.x_mm,Route.target_y_mm-Position.observer.feedback.y_mm)>CHASSIS_DISTANCE_TOLERANCE_MM ||
     fabsf(angle_error(Route.target_yaw_deg,Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI))>CHASSIS_DISTANCE_HEADING_DEG) return false;
  /* A recovered stationary feedback gap resets the observer baseline, not its
   * origin. The completed local return has fresh stop/arrival evidence. */
  Generation=Position.generation;
  AcceptedYaw=Position.observer.feedback.yaw_rad*180.0f/CHASSIS_MODEL_PI;
  return submit(Route.segment);
}
bool ChassisRoute_StationResume(void)
{
  if (!Stations || !Route.state || strcmp(Route.state,"station") || Route.station!=5U || Route.visit!=1U) return false;
  ChassisRoute_Process();
  if (strcmp(Route.state,"station")) return false;
  return submit(Route.segment);
}
bool ChassisRoute_StationFinish(void)
{
  if (!Stations || !Route.state || strcmp(Route.state,"station") || Route.station!=4U || Route.visit!=2U) return false;
  ChassisRoute_Process();
  if (strcmp(Route.state,"station")) return false;
  Route.state="done";Route.reason="station_handoff";Stations=false;
  return true;
}
static bool route_next(void)
{
  if(Automatic || !Route.state || strcmp(Route.state,"waiting")) return false;
  ChassisRoute_Process();
  if(strcmp(Route.state,"waiting")) return false;
  return submit(Route.segment);
}
bool ChassisRoute_Cancel(void)
{
  bool active=ChassisRoute_IsBusy() || ChassisRoute_StationReserved();
  bool ok=true;
  Stations=false;PauseState=NULL;ResumeFeedbackStarted=RecoveryHold=PauseMonitoring=false;
  Pending=false;
  if(active || ChassisMotion_IsBusy()) ok=ChassisMotion_Stop(0);
  if(active) { Route.state=ChassisMotion_IsBusy()?"stopping":"cancelled";Route.reason="cancelled";Route.stop_confirmed=false; }
  return ok;
}
static bool stopping_reason(void)
{
  return !strcmp(Route.reason,"cancelled") || !strcmp(Route.reason,"feedback_invalid") || !strcmp(Route.reason,"route_timeout");
}
void ChassisRoute_Process(void)
{
  if(!ChassisRoute_IsBusy() && !ChassisRoute_StationReserved()) return;
  if(PauseState)
  {
    if(RecoveryHold)return;
    ChassisMotion_StatusGet(&Motion);
    if(!PauseMonitoring)
    {
      if(ChassisMotion_IsBusy() || GrabTask_IsBusy())return;
      PauseStopTick=HAL_GetTick();
      if(!ChassisMotion_StopRequest(0)){Route.state="error";Route.reason="stop_unconfirmed";return;}
      PauseMonitoring=true;return;
    }
    ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
    Route.stop_confirmed=stop.tx_complete && stop.wheels_stopped &&
      (uint32_t)(stop.requested_ms-PauseStopTick)<0x80000000U;
    if(!ChassisMotion_IsBusy() && Route.stop_confirmed){Route.state="paused";Route.reason="operator_stop";}
    else if(!ChassisMotion_StopPending()){Route.state="error";Route.reason="stop_unconfirmed";}
    return;
  }
  if(TaskRoute && !strcmp(Route.state,"work")) return; /* local producer owns this station */
  ChassisLocalization_Get(&Position);ChassisMotion_StatusGet(&Motion);
  Route.feedback_valid=Position.feedback_valid;
  Route.error_x_mm=Route.target_x_mm-Position.observer.feedback.x_mm;
  Route.error_y_mm=Route.target_y_mm-Position.observer.feedback.y_mm;
  Route.error_heading_deg=angle_error(Route.target_yaw_deg,Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI);
  if(!stopping_reason() && (!Position.feedback_valid || Position.generation!=Generation || !Position.origin_valid ||
      (uint32_t)(HAL_GetTick()-Position.feedback_tick)>600U)) stop_route("feedback_invalid");
  if(stopping_reason())
  {
    if(!ChassisMotion_IsBusy())
    {
      Route.state=(!strcmp(Route.reason,"cancelled") && Motion.state!=CHASSIS_MOTION_ERROR)?"cancelled":"error";
      Route.stop_confirmed=Motion.stop_confirmed;
      if(Motion.state==CHASSIS_MOTION_ERROR) Route.reason=Motion.reason;
    }
    return;
  }
  if(!strcmp(Route.state,"waiting") || !strcmp(Route.state,"station"))
  {
    Mecanum_Status_t bus;
    Mecanum_StatusGet(&bus);
    if(Motion.action_id!=Route.action_id || Motion.state!=CHASSIS_MOTION_DONE ||
       !Motion.stop_confirmed || strcmp(Motion.reason,"feedback_arrived") || bus.locked || bus.error!=MECANUM_ERROR_NONE ||
       bus.stage==MECANUM_STAGE_FAULT || !Position.speed_valid)
      stop_route("feedback_invalid");
    else if(strcmp(Route.state,"station") && Automatic)
    {
      if(Route.segment==Route.total){Route.state="done";Route.reason="route_complete";}
      else (void)submit(Route.segment);
    }
    return;
  }
  if(Pending)
  {
    (void)submit_phase(&PendingPhase);return;
  }
  if(Motion.action_id!=Route.action_id)
  { (void)ChassisMotion_Stop(0);Route.state="error";Route.reason="action_mismatch";return; }
  if(Motion.state==CHASSIS_MOTION_ERROR)
  { Route.state="error";Route.reason=Motion.reason;Route.stop_confirmed=Motion.stop_confirmed;return; }
  if(!remaining()) { stop_route("route_timeout");return; }
  if(CurveActive){
    CurveProgress=ChassisMotion_PathProgress();
    if(CurveProgress>=Curve.turn_end && fabsf(Route.error_heading_deg)<=CHASSIS_DISTANCE_HEADING_DEG){
      if(!strcmp(Profile.phases[PhaseIndex].phase,"entry_curve")){
        if(TaskOptions.scan_qr && !QrPoseStarted && GrabTask_QrPoseStart())QrPoseStarted=true;
      }else Route.phase="work_dock"; /* Admit Base-only preparation on the final straight. */
    }
    /* Endpoint cannot hand off before the QR pose has been started/completed. */
    if(Motion.state==CHASSIS_MOTION_DONE && TaskOptions.scan_qr && FirstScanLeg &&
       !strcmp(Profile.phases[PhaseIndex].phase,"entry_curve")){
      if(!QrPoseStarted){if(GrabTask_QrPoseStart())QrPoseStarted=true;return;}
      GrabTask_Status_t arm;GrabTask_StatusGet(&arm);
      if(arm.state!=GRAB_COMPLETE || strcmp(arm.result,"qr_pose_ready"))return;
    }
  }
  if(Motion.state==CHASSIS_MOTION_DONE)
  {
    Route.stop_confirmed=Motion.stop_confirmed;
    if(!Motion.stop_confirmed || strcmp(Motion.reason,"feedback_arrived"))
    { Route.state="error";Route.reason="arrival_unconfirmed";return; }
    bool next_translation=PhaseIndex+1U<Profile.count &&
      pose_can_overlap(&Profile.phases[PhaseIndex+1U]);
    bool neutral_handoff=PhaseIndex+1U>=Profile.count && !Route.station &&
      GrabTask_PoseClearanceReady();
    if(TaskRoute && GrabTask_IsBusy() && !next_translation && !neutral_handoff)
    {
      /* The turn has already completed. Preserve its following dock during
       * an interrupted clearance wait instead of replaying the turn. */
      if(PhaseIndex+1U<Profile.count && !strcmp(Profile.phases[PhaseIndex+1U].phase,"work_dock"))
        defer_phase(&Profile.phases[PhaseIndex+1U],PhaseIndex+1U);
      return;
    }
    /* Start the QR pose after the entrance turn has stopped, then overlap its
     * Base rotation with the straight once X/Z clearance is fresh. */
    if(TaskRoute && TaskOptions.scan_qr && TaskOptions.start_id!=RADAR_START_UPPER && FirstScanLeg && PhaseIndex+1U<Profile.count &&
       !Profile.phases[PhaseIndex+1U].turn &&
       fabsf(angle_error(Profile.phases[PhaseIndex+1U].yaw_deg,90))<0.01f)
    {
      if(!QrPoseStarted)
      {
        if(GrabTask_QrPoseStart())
        {
          QrPoseStarted=true;Route.reason="preparing_qr_pose";
          defer_phase(&Profile.phases[PhaseIndex+1U],PhaseIndex+1U);
        }
        return;
      }
      GrabTask_Status_t arm;GrabTask_StatusGet(&arm);
      if((arm.state!=GRAB_COMPLETE || strcmp(arm.result,"qr_pose_ready")) &&
         !GrabTask_PoseClearanceReady())return;
    }

    if(PhaseIndex+1U<Profile.count) { (void)queue_phase(&Profile.phases[PhaseIndex+1U],PhaseIndex+1U);return; }
    if(hypotf(Route.error_x_mm,Route.error_y_mm)>CHASSIS_DISTANCE_TOLERANCE_MM ||
       fabsf(Route.error_heading_deg)>CHASSIS_DISTANCE_HEADING_DEG)
    { (void)repair_position();return; }
    AcceptedYaw=Profile.final_yaw_deg;
    if(Route.station==5U && Route.visit==1U) FirstScanLeg=false;
    Route.state=Route.segment==Route.total&&!Automatic?"done":"waiting";
    Route.reason=Automatic?"auto_continue":Route.segment==Route.total?"route_complete":"operator_next_required";
    if (Stations && (TaskRoute?((TaskOptions.scan_qr && Route.station==5U && Route.visit==1U) || Route.station==4U || Route.station==2U || Route.station==3U):
        ((Route.station==5U && Route.visit==1U) || (Route.station==4U && Route.visit==2U))))
    { Route.state="station";Route.reason="station_handoff"; }
  }
  else
  {
    Route.state=Motion.state==CHASSIS_MOTION_STOPPING?"stopping":"running";
    Route.reason=Route.phase;
  }
}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out)
{
  if(!out) return;
  *out=Route;
  if(!out->state) {out->state="idle";out->reason="idle";out->phase="idle";out->direction="none";}
  ChassisLocalization_Get(&Position);
  out->feedback_valid=Position.feedback_valid;
  if(out->segment)
  {
    out->error_x_mm=out->target_x_mm-Position.observer.feedback.x_mm;
    out->error_y_mm=out->target_y_mm-Position.observer.feedback.y_mm;
    out->error_heading_deg=angle_error(out->target_yaw_deg,Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI);
  }
}
static bool number(const char *s,float *n)
{
  char *end;errno=0;*n=strtof(s,&end);
  return end!=s && !*end && !errno && isfinite(*n);
}
bool ChassisRoute_Command(unsigned n,char *t[])
{
  bool ok=false;char text[640];int length;float dx,dy;const char *reject=NULL;
  if(n<2 || strcmp(t[0],"chassis")) return false;
  if(!strcmp(t[1],"stop") && n==2 && ChassisRoute_IsBusy()) ok=ChassisRoute_Cancel();
  else if(!strcmp(t[1],"test"))
  {
    float speed=0.0f,distance=0.0f;
    ChassisMotion_Profile_t profile=CHASSIS_PROFILE_LINEAR,previous=ChassisMotion_ProfileGet();
    reject="invalid_displacement";
    if(n==6 && !strcmp(t[5],"cosine"))profile=CHASSIS_PROFILE_COSINE;
    if(ContestMission_HasPendingWork())reject="mission_reserved";
    else if(ChassisRoute_IsBusy() || ChassisRoute_StationReserved()) reject="route_reserved";
    else if(ChassisMotion_IsBusy()) reject="motion_busy";
    else if(n==6 && strcmp(t[5],"linear") && strcmp(t[5],"cosine"))reject="profile_required";
    else if((n==5 || n==6) && number(t[2],&dx) && number(t[3],&dy) && number(t[4],&speed) &&
            (distance=hypotf(dx,dy))>0.0f && distance<=3000.0f)
    {
      reject="speed_range";
      if(speed>=10.0f && speed<=CHASSIS_MOTION_MAX_LINEAR_MM_S)
      {
        float yaw,c,s;
        uint32_t timeout=(uint32_t)(distance/speed*1250.0f)+10000U;
        if(timeout<CHASSIS_DISTANCE_TIMEOUT_MS) timeout=CHASSIS_DISTANCE_TIMEOUT_MS;
        ChassisLocalization_Get(&Position);
        yaw=Position.observer.feedback.yaw_rad;c=cosf(yaw);s=sinf(yaw);
        /* Same selected straight-line controller as a route leg; retain anchor. */
        ok=ChassisMotion_ProfileSet(profile) && ChassisMotion_MoveTo(Position.observer.feedback.x_mm+c*dx-s*dy,
                               Position.observer.feedback.y_mm+s*dx+c*dy,
                               yaw*180.0f/CHASSIS_MODEL_PI,speed,timeout);
        ChassisMotion_StatusGet(&Motion);reject=Motion.reason;
        if(ok) ChassisRoute_Init();
        else (void)ChassisMotion_ProfileSet(previous);
      }
    }
    ChassisMotion_StatusGet(&Motion);
    length=snprintf(text,sizeof(text),"%s chassis test reason=%s id=%lu profile=%s distance_mm=%.3f speed_mm_s=%.1f; request only, arrival requires feedback\r\n",
                    ok?"OK":"ERR",ok?"accepted":reject,(unsigned long)Motion.action_id,
                    ChassisMotion_ProfileName(profile),(double)distance,(double)speed);
    if(length>0 && (size_t)length<sizeof(text)) (void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)length);
    return true;
  }
  else if(!strcmp(t[1],"move"))
  {
    reject="invalid_displacement";
    if(ChassisRoute_IsBusy()) reject="route_reserved";
    else if(ChassisMotion_IsBusy()) reject="motion_busy";
    else if(n==4 && number(t[2],&dx) && number(t[3],&dy) && hypotf(dx,dy)>0 && hypotf(dx,dy)<=300 && !ChassisRoute_IsBusy())
    {
      ChassisLocalization_Get(&Position);
      ok=ChassisMotion_MoveTo(Position.observer.feedback.x_mm+dx,Position.observer.feedback.y_mm+dy,
                             Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI,
                             CHASSIS_DISTANCE_SPEED_MM_S,CHASSIS_DISTANCE_TIMEOUT_MS);
      ChassisMotion_StatusGet(&Motion);reject=Motion.reason;
      if(ok) ChassisRoute_Init();
    }
    ChassisMotion_StatusGet(&Motion);
    length=snprintf(text,sizeof(text),"%s chassis move reason=%s id=%lu; request only, arrival requires feedback\r\n",ok?"OK":"ERR",ok?"accepted":reject,(unsigned long)Motion.action_id);
    if(length>0 && (size_t)length<sizeof(text)) (void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)length);
    return true;
  }
  else if(!strcmp(t[1],"route"))
  {
    reject="format";
    if(n>=3 && (!strcmp(t[2],"start") || !strcmp(t[2],"auto")))
    {
      dx=CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
      if((n==3 || (n==4 && number(t[3],&dx))))
      {
        if(dx<10.0f || dx>CHASSIS_MOTION_MAX_LINEAR_MM_S) reject="speed_range";
        else if(ChassisRoute_IsBusy() || ChassisRoute_StationReserved()) reject="route_reserved";
        else if(ChassisMotion_IsBusy()) reject="motion_busy";
        else {reject=NULL;ok=start_at_speed(dx,!strcmp(t[2],"auto"),false);}
      }
    }
    else if(n==3)
    {
      if(!strcmp(t[2],"next"))
      {
        if(Automatic || !Route.state || strcmp(Route.state,"waiting")) reject="next_requires_waiting";
        else {reject=NULL;ok=route_next();}
      }
      else if(!strcmp(t[2],"cancel")) {ok=ChassisRoute_Cancel();reject="stop_failed";}
      else if(!strcmp(t[2],"status")) ok=true;
    }
  }
  else return false;
  ChassisRoute_StatusGet(&Route);
  length=snprintf(text,sizeof(text),"%s chassis route state=%s segment=%lu id=%lu reason=%s error_mm=%.1f,%.1f heading_error_deg=%.2f feedback_valid=%u stop_confirmed=%u speed_mm_s=%.1f auto=%u total=%lu dwell_ms=%lu phase=%s direction=%s phase_speed_mm_s=%.1f phase_target=%.1f,%.1f,%.1f station=%lu visit=%lu\r\n",
      ok?"OK":"ERR",Route.state,(unsigned long)Route.segment,(unsigned long)Route.action_id,(!ok && reject)?reject:Route.reason,
      (double)Route.error_x_mm,(double)Route.error_y_mm,(double)Route.error_heading_deg,Route.feedback_valid?1:0,Route.stop_confirmed?1:0,(double)RouteSpeed,Automatic?1:0,(unsigned long)Route.total,(unsigned long)CHASSIS_ROUTE_AUTO_DWELL_MS,
      Route.phase,Route.direction,(double)Route.phase_speed_mm_s,(double)Route.phase_target_x_mm,(double)Route.phase_target_y_mm,(double)Route.phase_target_yaw_deg,(unsigned long)Route.station,(unsigned long)Route.visit);
  if(length>0 && (size_t)length<sizeof(text)) (void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)length);
  return true;
}
