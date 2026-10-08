#include "chassis_route.h"
#include "chassis_route_profile.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_config.h"
#include "console_tx.h"
#include "mecanum_chassis.h"
#include "GrabTask.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
static ChassisRoute_Status_t Route;
static bool Submitting, Automatic, Stations, FirstScanLeg;
static bool TaskRoute, RequestTask;
static uint32_t Generation, LegStartTick, LegBudget;
static float RouteSpeed = CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
static float AcceptedYaw;
static ChassisModel_Pose_t StartOrigin;
static uint32_t StartUnits;
static RadarPlan_Point_t Planned[RADAR_PLAN_MAX_POINTS-1U];
static unsigned PlannedCount, PhaseIndex;
static ChassisRoute_Profile_t Profile;
static ChassisLocalization_Status_t Position;
static ChassisMotion_Status_t Motion;
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
  TaskRoute=RequestTask=false;
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
      (!strcmp(Route.state,"station") || !strcmp(Route.state,"work"))));
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
static bool submit_phase(const ChassisRoute_Phase_t *phase)
{
  bool ok;
  uint32_t timeout=remaining();
  if(!timeout) { stop_route("route_timeout");return false; }
  Submitting=true;
  if(phase->turn) ok=ChassisMotion_TurnTo(phase->yaw_deg,timeout);
  else ok=ChassisMotion_MoveTo(phase->x_mm,phase->y_mm,phase->yaw_deg,phase->speed_mm_s,timeout);
  Submitting=false;
  ChassisMotion_StatusGet(&Motion);
  if(!ok) { Route.state="error";Route.reason=Motion.reason;return false; }
  Route.action_id=Motion.action_id;Route.phase=phase->phase;Route.direction=phase->direction;
  Route.phase_speed_mm_s=phase->speed_mm_s;
  Route.phase_target_x_mm=phase->x_mm;Route.phase_target_y_mm=phase->y_mm;Route.phase_target_yaw_deg=phase->yaw_deg;
  Route.state="running";Route.reason=phase->phase;Route.stop_confirmed=false;
  return true;
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
  return submit_phase(&phase);
}
static bool submit(unsigned index)
{
  RadarPlan_Point_t from,to;
  if(Route.planned)
  {
    to=Planned[index];
    if(index) from=Planned[index-1U];
    else (void)ChassisRouteProfile_FixedPoint(0,&from);
  }
  else
  {
    (void)ChassisRouteProfile_FixedPoint(index,&from);
    (void)ChassisRouteProfile_FixedPoint(index+1U,&to);
  }
  if(!(TaskRoute?ChassisRouteProfile_BuildTask(&from,&to,AcceptedYaw,RouteSpeed,FirstScanLeg,&Profile):
      ChassisRouteProfile_Build(&from,&to,AcceptedYaw,RouteSpeed,FirstScanLeg,&Profile)))
  { Route.state="error";Route.reason="route_profile_invalid";return false; }
  LegStartTick=HAL_GetTick();
  LegBudget=(uint32_t)fminf(CHASSIS_DISTANCE_MAX_TIMEOUT_MS,
               fmaxf(CHASSIS_DISTANCE_TIMEOUT_MS,2000.0f*Profile.nominal_time_s+10000.0f));
  PhaseIndex=0;
  Route.segment=index+1U;Route.target_x_mm=to.x_mm;Route.target_y_mm=to.y_mm;
  Route.target_yaw_deg=Profile.final_yaw_deg;Route.station=to.station;Route.visit=to.visit;
  Route.speed_mm_s=RouteSpeed;
  ChassisLocalization_Get(&Position);
  return Profile.count?submit_phase(&Profile.phases[0]):repair_position();
}
static bool start_at_speed(float speed, bool automatic, bool planned)
{
  unsigned i;
  if (!isfinite(speed) || speed < 10.0f || speed > CHASSIS_MOTION_MAX_LINEAR_MM_S) return false;
  if(ChassisRoute_IsBusy() || ChassisRoute_StationReserved() || ChassisMotion_IsBusy()) return false;
  ChassisLocalization_Get(&Position);
  if(!Position.origin_valid || !Position.feedback_valid || !Position.speed_valid ||
     (uint32_t)(HAL_GetTick()-Position.feedback_tick)>600U ||
     fabsf(Position.origin.x_mm-2250)>0.1f || fabsf(Position.origin.y_mm-150)>0.1f ||
     fabsf(angle_error(Position.origin.yaw_rad*180/CHASSIS_MODEL_PI,90))>0.1f ||
     hypotf(Position.observer.feedback.x_mm-2250,Position.observer.feedback.y_mm-150)>CHASSIS_DISTANCE_TOLERANCE_MM ||
     fabsf(angle_error(Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI,90))>CHASSIS_DISTANCE_HEADING_DEG)
  { Route.state="error";Route.reason="start_origin_or_feedback";return false; }
  TaskRoute=RequestTask;
  RouteSpeed=speed;Automatic=automatic;Stations=false;AcceptedYaw=90;FirstScanLeg=!planned;
  if(planned) for(i=0;i<PlannedCount;i++)
    if(Planned[i].station==5U && Planned[i].visit==1U) FirstScanLeg=true;
  Route.planned=planned;Route.total=planned?PlannedCount:CHASSIS_ROUTE_TARGET_COUNT;
  Generation=Position.generation;
  StartOrigin=Position.origin;StartUnits=Position.units_per_rev;
  return submit(0);
}
bool ChassisRoute_PlanStart(const RadarPlan_t *plan, float speed, bool station_mode)
{
  unsigned i;
  if(!plan || !plan->valid || plan->count<2U || plan->count>RADAR_PLAN_MAX_POINTS ||
     ChassisRoute_IsBusy() || ChassisRoute_StationReserved() || ChassisMotion_IsBusy()) return false;
  if(plan->points[0].x_mm!=2250 || plan->points[0].y_mm!=150) return false;
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
{
  bool ok;
  RequestTask=true;
  ok=plan?ChassisRoute_PlanStart(plan,speed,true):station_start(speed);
  RequestTask=false;
  return ok;
}
bool ChassisRoute_WorkBegin(void)
{
  if(!TaskRoute || !Stations || strcmp(Route.state,"station") || !Route.stop_confirmed) return false;
  Route.state="work";Route.reason="station_work";return true;
}
bool ChassisRoute_WorkResume(void)
{
  if(!TaskRoute || !Stations || strcmp(Route.state,"work") || GrabTask_IsBusy() || ChassisMotion_IsBusy()) return false;
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
  if(Motion.state!=CHASSIS_MOTION_DONE || !Motion.stop_confirmed || strcmp(Motion.reason,"feedback_arrived")) return false;
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
  Stations=false;
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
  if(Motion.action_id!=Route.action_id)
  { (void)ChassisMotion_Stop(0);Route.state="error";Route.reason="action_mismatch";return; }
  if(Motion.state==CHASSIS_MOTION_ERROR)
  { Route.state="error";Route.reason=Motion.reason;Route.stop_confirmed=Motion.stop_confirmed;return; }
  if(!remaining()) { stop_route("route_timeout");return; }
  if(Motion.state==CHASSIS_MOTION_DONE)
  {
    Route.stop_confirmed=Motion.stop_confirmed;
    if(!Motion.stop_confirmed || strcmp(Motion.reason,"feedback_arrived"))
    { Route.state="error";Route.reason="arrival_unconfirmed";return; }
    if(TaskRoute && GrabTask_IsBusy()) return; /* owned work-pose preparation, no elapsed dwell */
    if(PhaseIndex<Profile.count) PhaseIndex++;
    if(PhaseIndex<Profile.count) { (void)submit_phase(&Profile.phases[PhaseIndex]);return; }
    if(hypotf(Route.error_x_mm,Route.error_y_mm)>CHASSIS_DISTANCE_TOLERANCE_MM ||
       fabsf(Route.error_heading_deg)>CHASSIS_DISTANCE_HEADING_DEG)
    { (void)repair_position();return; }
    AcceptedYaw=Profile.final_yaw_deg;
    if(Route.station==5U && Route.visit==1U) FirstScanLeg=false;
    Route.state=Route.segment==Route.total&&!Automatic?"done":"waiting";
    Route.reason=Automatic?"auto_continue":Route.segment==Route.total?"route_complete":"operator_next_required";
    if (Stations && (TaskRoute?(Route.station==4U || Route.station==2U || Route.station==3U):
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
