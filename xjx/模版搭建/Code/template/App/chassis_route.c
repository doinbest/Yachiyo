#include "chassis_route.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_config.h"
#include "console_tx.h"
#include "mecanum_chassis.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
static ChassisRoute_Status_t Route;
static bool Submitting, Automatic;
static bool Stations;
static uint32_t DwellTick;
static uint32_t Generation;
static float RouteSpeed = CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
#define ROUTE_COUNT 16U
static RadarPlan_Point_t Planned[RADAR_PLAN_MAX_POINTS-1U];
static unsigned PlannedCount;
static const float Points[ROUTE_COUNT][2]={
  {2100,300},{2100,1200},{2100,2100},{1200,2100},
  {1200,350},{1200,1200},{350,1200},{1200,1200},
  {1200,2100},{1200,350},{1200,1200},{350,1200},
  {1200,1200},{2100,1200},{2100,300},{2250,150}};
static ChassisLocalization_Status_t Position;
static ChassisMotion_Status_t Motion;
static float angle_error(float a,float b)
{
  float e=fmodf(a-b+180.0f,360.0f);
  return (e<0?e+360.0f:e)-180.0f;
}
void ChassisRoute_Init(void)
{
  memset(&Route,0,sizeof(Route));Route.state="idle";Route.reason="idle";Route.total=ROUTE_COUNT;Submitting=false;Automatic=false;Stations=false;DwellTick=0;PlannedCount=0;
}
bool ChassisRoute_IsBusy(void)
{
  return Route.state && (!strcmp(Route.state,"running") || !strcmp(Route.state,"stopping") || !strcmp(Route.state,"waiting"));
}
bool ChassisRoute_IsSubmitting(void) { return Submitting; }
bool ChassisRoute_StationReserved(void)
{
  return Stations && (ChassisRoute_IsBusy() || (Route.state && !strcmp(Route.state,"station")));
}
static bool submit(unsigned index)
{
  bool ok;
  uint32_t timeout;
  float distance;
  float x=Route.planned?Planned[index].x_mm:Points[index][0];
  float y=Route.planned?Planned[index].y_mm:Points[index][1];
  ChassisLocalization_Get(&Position);
  distance=hypotf(x-Position.observer.feedback.x_mm,y-Position.observer.feedback.y_mm);
  /* Include integer-RPM quantization and final settling at low speed. */
  timeout=(uint32_t)fminf(CHASSIS_DISTANCE_MAX_TIMEOUT_MS,
      fmaxf(CHASSIS_DISTANCE_TIMEOUT_MS,2000.0f*distance/RouteSpeed+10000.0f));
  Submitting=true;
  ok=ChassisMotion_MoveTo(x,y,90.0f,
                          RouteSpeed,timeout);
  Submitting=false;
  ChassisMotion_StatusGet(&Motion);
  if(!ok) { Route.state="error";Route.reason=Motion.reason;return false; }
  Route.segment=index+1;Route.action_id=Motion.action_id;
  Route.target_x_mm=x;Route.target_y_mm=y;Route.target_yaw_deg=90;
  Route.station=Route.planned?Planned[index].station:0U;
  Route.visit=Route.planned?Planned[index].visit:0U;
  Route.state="running";Route.reason="running";Route.stop_confirmed=false;
  return true;
}
static bool start_at_speed(float speed, bool automatic, bool planned)
{
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
  RouteSpeed=speed;Automatic=automatic;Stations=false;
  Route.planned=planned;Route.total=planned?PlannedCount:ROUTE_COUNT;
  Generation=Position.generation;
  return submit(0);
}
bool ChassisRoute_Start(void) { return start_at_speed(CHASSIS_ROUTE_DEFAULT_SPEED_MM_S,false,false); }
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
bool ChassisRoute_StationStart(float speed)
{
  if (!start_at_speed(speed,true,false)) return false;
  Stations=true;
  return true;
}
bool ChassisRoute_StationResume(void)
{
  if (!Stations || !Route.state || strcmp(Route.state,"station") ||
      (Route.planned?(Route.station!=5U || Route.visit!=1U):Route.segment!=2U)) return false;
  ChassisRoute_Process();
  if (strcmp(Route.state,"station")) return false;
  return submit(Route.segment);
}
bool ChassisRoute_StationFinish(void)
{
  if (!Stations || !Route.state || strcmp(Route.state,"station") ||
      (Route.planned?(Route.station!=4U || Route.visit!=2U):Route.segment!=4U)) return false;
  ChassisRoute_Process();
  if (strcmp(Route.state,"station")) return false;
  Route.state="done";Route.reason="station_handoff";Stations=false;
  return true;
}
bool ChassisRoute_Next(void)
{
  if(Automatic || !Route.state || strcmp(Route.state,"waiting")) return false;
  ChassisRoute_Process();
  if(strcmp(Route.state,"waiting")) return false;
  return submit(Route.segment);
}
bool ChassisRoute_Cancel(void)
{
  bool active=ChassisRoute_IsBusy() || ChassisRoute_StationReserved();
  Stations=false;
  bool ok=true;
  if(active || ChassisMotion_IsBusy()) ok=ChassisMotion_Stop(0);
  if(active) { Route.state=ChassisMotion_IsBusy()?"stopping":"cancelled";Route.reason="cancelled";Route.stop_confirmed=false; }
  return ok;
}
void ChassisRoute_Process(void)
{
  if(!ChassisRoute_IsBusy() && !ChassisRoute_StationReserved()) return;
  ChassisLocalization_Get(&Position);ChassisMotion_StatusGet(&Motion);
  Route.feedback_valid=Position.feedback_valid;
  Route.error_x_mm=Route.target_x_mm-Position.observer.feedback.x_mm;
  Route.error_y_mm=Route.target_y_mm-Position.observer.feedback.y_mm;
  Route.error_heading_deg=angle_error(Route.target_yaw_deg,Position.observer.feedback.yaw_rad*180/CHASSIS_MODEL_PI);
  if(strcmp(Route.reason,"cancelled") && strcmp(Route.reason,"feedback_invalid") &&
     (!Position.feedback_valid || Position.generation!=Generation || !Position.origin_valid ||
      (uint32_t)(HAL_GetTick()-Position.feedback_tick)>600U))
  {
    (void)ChassisMotion_Stop(0);Route.state="stopping";Route.reason="feedback_invalid";Route.stop_confirmed=false;
  }
  if(!strcmp(Route.reason,"cancelled") || !strcmp(Route.reason,"feedback_invalid"))
  {
    if(!ChassisMotion_IsBusy())
    {
      Route.state=(!strcmp(Route.reason,"cancelled") && Motion.state!=CHASSIS_MOTION_ERROR)?"cancelled":"error";
      if(Motion.state==CHASSIS_MOTION_ERROR) Route.reason=Motion.reason;
    }
    return;
  }
  if(!strcmp(Route.state,"waiting") || !strcmp(Route.state,"station"))
  {
    Mecanum_Status_t bus;
    Mecanum_StatusGet(&bus);
    if(Motion.action_id!=Route.action_id || Motion.state!=CHASSIS_MOTION_DONE ||
       !Motion.stop_confirmed || bus.locked || bus.error!=MECANUM_ERROR_NONE ||
       bus.stage==MECANUM_STAGE_FAULT || !Position.speed_valid)
    {
      (void)ChassisMotion_Stop(0);Route.state="stopping";Route.reason="feedback_invalid";Route.stop_confirmed=false;
      Route.stop_confirmed=false;
    }
    else if(strcmp(Route.state,"station") && Automatic && (uint32_t)(HAL_GetTick()-DwellTick)>=CHASSIS_ROUTE_AUTO_DWELL_MS)
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
  if(Motion.state==CHASSIS_MOTION_DONE)
  {
    Route.stop_confirmed=Motion.stop_confirmed;
    if(!Motion.stop_confirmed || strcmp(Motion.reason,"feedback_arrived"))
    { Route.state="error";Route.reason="arrival_unconfirmed";return; }
    DwellTick=HAL_GetTick();
    Route.state=Route.segment==Route.total&&!Automatic?"done":"waiting";
    Route.reason=Automatic?"auto_dwell":Route.segment==Route.total?"route_complete":"operator_next_required";
    if (Stations && (Route.planned?((Route.station==5U && Route.visit==1U) ||
        (Route.station==4U && Route.visit==2U)):(Route.segment==2U || Route.segment==4U)))
    { Route.state="station";Route.reason="station_handoff"; }
  }
  else
  {
    Route.state=Motion.state==CHASSIS_MOTION_STOPPING?"stopping":"running";
    if(!strcmp(Motion.reason,"position_correcting")) Route.reason="position_correcting";
  }
}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out)
{
  if(!out) return;
  *out=Route;
  if(!out->state) {out->state="idle";out->reason="idle";}
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
  bool ok=false;char text[320];int length;float dx,dy;const char *reject="invalid_displacement";
  if(n<2 || strcmp(t[0],"chassis")) return false;
  if(!strcmp(t[1],"stop") && n==2 && ChassisRoute_IsBusy()) ok=ChassisRoute_Cancel();
  else if(!strcmp(t[1],"move"))
  {
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
    if(n==4 && (!strcmp(t[2],"start") || !strcmp(t[2],"auto")) && number(t[3],&dx)) ok=start_at_speed(dx,!strcmp(t[2],"auto"),false);
    else if(n==3)
    {
      if(!strcmp(t[2],"start")) ok=ChassisRoute_Start();
      else if(!strcmp(t[2],"auto")) ok=start_at_speed(CHASSIS_ROUTE_DEFAULT_SPEED_MM_S,true,false);
      else if(!strcmp(t[2],"next")) ok=ChassisRoute_Next();
      else if(!strcmp(t[2],"cancel")) ok=ChassisRoute_Cancel();
      else if(!strcmp(t[2],"status")) ok=true;
    }
  }
  else return false;
  ChassisRoute_StatusGet(&Route);
  length=snprintf(text,sizeof(text),"%s chassis route state=%s segment=%lu id=%lu reason=%s error_mm=%.1f,%.1f heading_error_deg=%.2f feedback_valid=%u stop_confirmed=%u speed_mm_s=%.1f auto=%u total=%lu dwell_ms=%lu\r\n",
      ok?"OK":"ERR",Route.state,(unsigned long)Route.segment,(unsigned long)Route.action_id,Route.reason,
      (double)Route.error_x_mm,(double)Route.error_y_mm,(double)Route.error_heading_deg,Route.feedback_valid?1:0,Route.stop_confirmed?1:0,(double)RouteSpeed,Automatic?1:0,(unsigned long)Route.total,(unsigned long)CHASSIS_ROUTE_AUTO_DWELL_MS);
  if(length>0 && (size_t)length<sizeof(text)) (void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)length);
  return true;
}
