#include "radar_console.h"
#include "radar_scan.h"
#include "chassis_route.h"
#include "chassis_localization.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include "GrabRoute.h"
#include "GrabTask.h"
#include "mechanical_arm.h"
#include "ArmVision.h"
#include "MaterialVision.h"
#include "hwt101_calibration.h"
#include "console_tx.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

/* A requested page has its own low-priority slot. Chassis telemetry and replies
 * cannot overwrite it; pages consume at most 256 wire bytes per second. */
static RadarPlan_t Plan;
static RadarScan_Status_t Scan;
static ChassisLocalization_Status_t Location;
static ChassisRoute_Status_t Route;
static uint32_t Session,PlanId,PlanMap,LastBulk,LastStatus,LastNav,ParamsRevision;
static uint32_t SeenMap;
static uint32_t CaptureOrigin;
static bool Watching,Pending,FormatOk;
static char Kind[8],Line[257];
static uint32_t RequestedId;
static unsigned RequestedPage,Length;
static const char *const ParamNames[]={"lidar_x_mm","lidar_y_mm","zero_deg",
  "distance_min_mm","distance_max_mm","angle_min_tenths","angle_max_tenths",
  "threshold","energy_min","energy_max"};
static const char *const States[]={"idle","preparing","wait_wrap","collecting","ready","stopped","error"};

static void refresh(void)
{
  RadarScan_StatusGet(&Scan);ChassisLocalization_Get(&Location);ChassisRoute_StatusGet(&Route);
}
static bool applicable(void)
{
  return Scan.map_valid && Scan.map_applicable && Scan.origin_generation==Location.generation;
}
bool RadarConsole_ScanBusy(void)
{
  RadarScan_Status_t s;RadarScan_StatusGet(&s);
  return s.state==RADAR_SCAN_PREPARING || s.state==RADAR_SCAN_WAIT_WRAP || s.state==RADAR_SCAN_COLLECTING;
}
static bool actuators_busy(void)
{
  return ChassisMotion_IsBusy() || ChassisRoute_IsBusy() || ChassisRoute_StationReserved() ||
    GrabRoute_IsBusy() || GrabTask_IsBusy() || Mecanum_IsBusy() || MechanicalArm_IsBusy() ||
    ArmVision_IsBusy() || MaterialVision_IsBusy() || HWT101_Cal_IsBusy();
}
static void reply(bool ok,const char *reason)
{
  char text[110];int n=snprintf(text,sizeof(text),"%s radar %s\r\n",ok?"OK":"ERR",reason);
  if(n>0 && n<(int)sizeof(text)) (void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)n);
}
static void append(const char *format,...)
{
  int n;va_list ap;if(!FormatOk) return;
  va_start(ap,format);n=vsnprintf(Line+Length,sizeof(Line)-Length,format,ap);va_end(ap);
  if(n<0 || (unsigned)n>=sizeof(Line)-Length) {FormatOk=false;return;}Length+=(unsigned)n;
}
static void begin(const char *kind,unsigned page,unsigned pages)
{
  Length=0;FormatOk=true;
  append("@RADAR {\"v\":1,\"k\":\"%s\",\"session\":%lu,\"origin\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":%lu,\"page\":%u,\"pages\":%u,",
    kind,(unsigned long)Session,(unsigned long)Scan.origin_generation,(unsigned long)Scan.scan_id,
    (unsigned long)Scan.map_id,(unsigned long)PlanId,page,pages);
}
static bool finish(bool foreground)
{
  append("}\r\n");if(!FormatOk || Length>256U) {reply(false,"page_size");return false;}
  return foreground?ConsoleTx_Write((const uint8_t*)Line,(uint16_t)Length):ConsoleTx_Bulk(Line,(uint16_t)Length);
}
static float param_value(unsigned i)
{
  const RadarMap_Params_t *p=RadarScan_Params();
  switch(i){case 0:return p->lidar_x_mm;case 1:return p->lidar_y_mm;case 2:return p->zero_deg;
  case 3:return p->distance_min_mm;case 4:return p->distance_max_mm;case 5:return p->angle_min_tenths;
  case 6:return p->angle_max_tenths;case 7:return p->threshold;case 8:return p->energy_min;default:return p->energy_max;}
}
static bool build_plan(void)
{
  if(!Scan.map_valid) return false;
  PlanMap=Scan.map_id;PlanId++;if(!PlanId) PlanId=1U;
  return RadarPlan_Build(RadarScan_Map(),&Plan);
}
static void status_page(bool foreground)
{
  begin("status",0,1);
  append("\"state\":\"%s\",\"reason\":\"%s\",\"elapsed\":%lu,\"applicable\":%s,\"points_valid\":%s",
    States[Scan.state],Scan.reason?Scan.reason:"none",(unsigned long)Scan.elapsed_ms,
    applicable()?"true":"false",Scan.points_valid?"true":"false");
  (void)finish(foreground);
}
static void nav_page(bool foreground)
{
  begin("nav",0,1);
  append("\"state\":\"%s\",\"reason\":\"%s\",\"index\":%lu,\"total\":%lu,\"station\":%lu,\"visit\":%lu,\"stopped\":%s",
    Route.planned?Route.state:"idle",Route.planned?Route.reason:"idle",
    (unsigned long)(Route.planned?Route.segment:0U),(unsigned long)(Route.planned?Route.total:0U),
    (unsigned long)Route.station,(unsigned long)Route.visit,Route.stop_confirmed?"true":"false");
  (void)finish(foreground);
}
static bool fetch_page(void)
{
  unsigned i,offset,total,pages;uint16_t count=0;const RadarSample_t *points;
  if(!strcmp(Kind,"map"))
  {
    const RadarMap_t *m=RadarScan_Map();const RadarMap_Params_t *p=RadarScan_MapParams();
    if(!Scan.map_valid || RequestedId!=Scan.map_id || RequestedPage>=6U) return false;
    begin("map",RequestedPage,6);
    if(!RequestedPage) append("\"mask\":%lu,\"pose\":[%.1f,%.1f,%.1f],\"coverage\":%u,\"age_ms\":%lu,\"applicable\":%s",
      (unsigned long)m->mask,(double)p->lidar_x_mm,(double)p->lidar_y_mm,(double)p->zero_deg,
      3600U,(unsigned long)(HAL_GetTick()-Scan.completed_tick),applicable()?"true":"false");
    else {offset=(RequestedPage-1U)*5U;append("\"offset\":%u,\"counts\":[",offset);
      for(i=0;i<5U;i++) append("%s%u",i?",":"",m->counts[offset+i]);
      append("]");}
  }
  else if(!strcmp(Kind,"path"))
  {
    if(!PlanId || RequestedId!=PlanId) return false;
    pages=Plan.count?(Plan.count+1U)/2U:1U;if(RequestedPage>=pages) return false;
    begin("path",RequestedPage,pages);offset=RequestedPage*2U;
    append("\"offset\":%u,\"valid\":%s,\"failed_leg\":%u,\"points\":[",offset,Plan.valid?"true":"false",Plan.failed_leg);
    for(i=offset;i<Plan.count && i<offset+2U;i++) append("%s[%d,%d,%u,%u]",i==offset?"":",",
      Plan.points[i].x_mm,Plan.points[i].y_mm,Plan.points[i].station,Plan.points[i].visit);
    append("]");
  }
  else if(!strcmp(Kind,"points") || !strcmp(Kind,"cloud"))
  {
    points=RadarScan_Points(&count);if(!points || !Scan.points_valid || RequestedId!=Scan.scan_id) return false;
    total=!strcmp(Kind,"points") && count>360U?360U:count;
    pages=total?(total+2U)/3U:1U;if(RequestedPage>=pages) return false;
    begin(Kind,RequestedPage,pages);offset=RequestedPage*3U;
    append("\"offset\":%u,\"total\":%u,\"truncated\":%s,\"points\":[",offset,total,Scan.truncated?"true":"false");
    for(i=offset;i<total && i<offset+3U;i++) {
      unsigned at=total?(uint32_t)i*count/total:0U;
      append("%s[%u,%u,%u]",i==offset?"":",",points[at].angle_tenths,points[at].distance_mm,points[at].energy);
    }append("]");
  }
  else if(!strcmp(Kind,"params"))
  {
    if((RequestedId && RequestedId!=ParamsRevision) || RequestedPage>=10U) return false;
    begin("params",RequestedPage,10);append("\"rev\":%lu,\"key\":\"%s\",\"value\":%.1f",
      (unsigned long)ParamsRevision,ParamNames[RequestedPage],(double)param_value(RequestedPage));
  }
  else return false;
  return finish(false);
}
void RadarConsole_Init(void)
{
  memset(&Plan,0,sizeof(Plan));Session=HAL_GetTick()+1U;if(!Session) Session=1;
  PlanId=PlanMap=SeenMap=LastBulk=LastStatus=LastNav=CaptureOrigin=0;ParamsRevision=1;
  Watching=Pending=false;Kind[0]=0;
}
void RadarConsole_Process(void)
{
  uint32_t now=HAL_GetTick();refresh();
  if(RadarConsole_ScanBusy() && (ChassisMotion_IsBusy() || Mecanum_IsBusy() ||
      CaptureOrigin!=Location.generation)) RadarScan_Stop();
  if(Scan.map_valid && Scan.map_id!=SeenMap){SeenMap=Scan.map_id;(void)build_plan();}
  if(!Watching || !ConsoleTx_BulkReady() || (uint32_t)(now-LastBulk)<1000U) return;
  if(Pending){if(!fetch_page()) reply(false,"snapshot_changed_or_page_range");Pending=false;}
  else if(Route.planned && (uint32_t)(now-LastNav)>=1500U){nav_page(false);LastNav=now;}
  else if((uint32_t)(now-LastStatus)>=2000U){status_page(false);LastStatus=now;}
  else return;
  LastBulk=now;
}
static bool number(const char *s,float *value)
{
  char *end;errno=0;*value=strtof(s,&end);return end!=s && !*end && !errno && isfinite(*value);
}
static bool integer(const char *s,uint32_t *value)
{
  char *end;unsigned long n;errno=0;if(!*s || *s=='-') return false;
  n=strtoul(s,&end,10);if(end==s || *end || errno || n>0xFFFFFFFFUL) return false;*value=(uint32_t)n;return true;
}
bool RadarConsole_Command(unsigned n,char *t[])
{
  float value;uint32_t id,page;bool ok=false;const char *reason="format";
  if(n<2U || strcmp(t[0],"radar")) return false;
  refresh();Watching=true;
  if(n==2U && !strcmp(t[1],"status")){
    status_page(true);begin("diag",0,1);
    append("\"bytes\":%lu,\"packets\":%lu,\"bad\":%lu,\"overflow\":%lu,\"alarm\":%lu,\"unit\":\"%s\"",
      (unsigned long)Scan.bytes,(unsigned long)Scan.packets,(unsigned long)Scan.bad,(unsigned long)Scan.overflow,
      (unsigned long)Scan.alarm,Scan.unit==RADAR_UNIT_CM?"cm":Scan.unit==RADAR_UNIT_MM?"mm":"unknown");
    (void)finish(true);return true;
  }
  if(n==2U && !strcmp(t[1],"stop")){RadarScan_Stop();reply(true,"scan_stop_requested");return true;}
  if(n==2U && !strcmp(t[1],"scan")){
    if(actuators_busy()) reason="stationary_capture_required";
    else{
      RadarMap_Params_t p=*RadarScan_Params();
      /* Registered reference pose is expressed at the validated chassis origin.
       * Only the sensor pose follows actual feedback; IMU/motor axes stay intact. */
      if(Location.origin_valid && Location.feedback_valid){
        float yaw=Location.observer.feedback.yaw_rad-CHASSIS_MODEL_PI/2.0f;
        float dx=p.lidar_x_mm-2250.0f,dy=p.lidar_y_mm-150.0f;
        p.lidar_x_mm=Location.observer.feedback.x_mm+cosf(yaw)*dx-sinf(yaw)*dy;
        p.lidar_y_mm=Location.observer.feedback.y_mm+sinf(yaw)*dx+cosf(yaw)*dy;
        p.zero_deg+=yaw*180.0f/CHASSIS_MODEL_PI;
      }
      ok=RadarScan_Start(&p,Location.generation);reason=ok?"scan_accepted":"scan_busy_or_parameters";
      if(ok) CaptureOrigin=Location.generation;
    }
  }
  else if(n==2U && !strcmp(t[1],"plan")){
    ok=build_plan();reason=ok?"plan_ready":Scan.map_valid?"leg_unreachable":"no_complete_map";
  }
  else if(n==2U && !strcmp(t[1],"get")){
    if(Pending) reason="page_pending";
    else {strcpy(Kind,"params");RequestedId=0;RequestedPage=0;Pending=true;ok=true;reason="fetch_params";}
  }
  else if(n==4U && !strcmp(t[1],"set") && number(t[3],&value)){
    ok=RadarScan_SetParam(t[2],value);if(ok) ParamsRevision++;
    reason=ok?"parameter_updated_ram":"parameter_range_or_key";
  }
  else if(n==5U && !strcmp(t[1],"fetch") && integer(t[3],&id) && integer(t[4],&page)){
    if(strcmp(t[2],"map") && strcmp(t[2],"path") && strcmp(t[2],"points") && strcmp(t[2],"cloud") && strcmp(t[2],"params")) reason="fetch_kind";
    else if(Pending) reason="page_pending";
    else if(page>65535U) reason="page_range";
    else {strcpy(Kind,t[2]);RequestedId=id;RequestedPage=(unsigned)page;Pending=true;return true;}
  }
  else if(n>=3U && (!strcmp(t[1],"nav") || !strcmp(t[1],"task"))){
    if(n==3U && !strcmp(t[1],"nav") && !strcmp(t[2],"status")){nav_page(true);return true;}
    if(n==3U && !strcmp(t[1],"nav") && !strcmp(t[2],"cancel")){
      if(Route.planned){GrabRoute_Stop();ok=ChassisRoute_Cancel();}else ok=true;
      reason="navigation_stop_requested";
    }
    else if((n==3U || n==4U) && !strcmp(t[2],"start") && (n==3U || number(t[3],&value))){
      if(n==3U) value=CHASSIS_DISTANCE_SPEED_MM_S;
      if(RadarConsole_ScanBusy()) reason="scan_in_progress";
      else if(!applicable() || PlanMap!=Scan.map_id) reason="map_origin_or_parameters_changed";
      else if(!Plan.valid) reason="leg_unreachable";
      else if(actuators_busy()) reason="actuator_busy";
      else {
        ok=!strcmp(t[1],"task")?GrabRoute_StartPlanned(&Plan,value):ChassisRoute_PlanStart(&Plan,value,false);
        reason=ok?"navigation_accepted":"navigation_or_grab_not_ready";
      }
    }
  }
  reply(ok,reason);return true;
}
