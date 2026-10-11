#include "chassis_route_profile.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

#define ROUTE_PI 3.14159265358979323846f
static const RadarPlan_Point_t FixedPoints[CHASSIS_ROUTE_TARGET_COUNT+1U]={
  {2250,150,1,0},{2100,1200,5,1},{2100,2100,0,0},{1200,2100,4,2},
  {1200,350,2,3},{1200,1200,0,0},{350,1200,3,4},{1200,1200,0,0},
  {1200,2100,4,5},{1200,350,2,6},{1200,1200,0,0},{350,1200,3,7},
  {1200,1200,0,0},{2100,1200,0,0},{2100,300,0,0},{2250,150,1,8}};
static float normalized(float yaw)
{
  yaw=fmodf(yaw,360.0f);
  return yaw>180.0f?yaw-360.0f:(yaw<=-180.0f?yaw+360.0f:yaw);
}
static float delta(float a,float b)
{
  float difference=fmodf(a-b+180.0f,360.0f);
  return (difference<0?difference+360.0f:difference)-180.0f;
}
static bool at(const RadarPlan_Point_t *point,int x,int y)
{ return point->x_mm==x && point->y_mm==y; }
static const char *direction(float dx,float dy,float yaw)
{
  float radians=yaw*ROUTE_PI/180.0f;
  float forward=dx*cosf(radians)+dy*sinf(radians);
  float lateral=-dx*sinf(radians)+dy*cosf(radians);
  if(fabsf(lateral)<0.01f) return forward>=0?"forward":"reverse";
  return fabsf(forward)<0.01f?"lateral":"diagonal";
}
static void add(ChassisRoute_Profile_t *out,bool turn,const char *name,
                float *x,float *y,float *yaw,float tx,float ty,float heading,float speed)
{
  ChassisRoute_Phase_t *phase;
  float distance=hypotf(tx-*x,ty-*y),rotation=fabsf(delta(heading,*yaw));
  if((turn && rotation<0.01f) || (!turn && distance<0.01f)) return;
  phase=&out->phases[out->count++];
  phase->turn=turn;phase->phase=name;
  phase->direction=turn?"turn":direction(tx-*x,ty-*y,heading);
  phase->x_mm=tx;phase->y_mm=ty;phase->yaw_deg=heading;phase->speed_mm_s=turn?0:speed;
  out->nominal_time_s+=turn?rotation/CHASSIS_MOTION_MAX_OMEGA_DEG_S:distance/speed;
  *x=tx;*y=ty;*yaw=heading;
}

bool ChassisRouteProfile_FixedPoint(unsigned index,RadarPlan_Point_t *out)
{
  if(!out || index>CHASSIS_ROUTE_TARGET_COUNT) return false;
  *out=FixedPoints[index];return true;
}
bool ChassisRouteProfile_Build(const RadarPlan_Point_t *from,const RadarPlan_Point_t *to,
                              float previous_yaw_deg,float speed_mm_s,bool first_scan_leg,
                              ChassisRoute_Profile_t *out)
{
  float x,y,yaw,dx,dy,travel,opposite,first_turn,other_turn,final_yaw,dock_speed;
  bool east_west,temp_arrival,temp_departure;
  if(!from || !to || !out || !isfinite(previous_yaw_deg) ||
     !isfinite(speed_mm_s) || speed_mm_s<=0) return false;
  memset(out,0,sizeof(*out));
  x=from->x_mm;y=from->y_mm;yaw=normalized(previous_yaw_deg);
  dx=to->x_mm-from->x_mm;dy=to->y_mm-from->y_mm;
  travel=yaw;
  if(first_scan_leg || at(to,2100,300) || at(to,2250,150)) travel=90;
  else if(hypotf(dx,dy)>300.0f)
  {
    travel=normalized(atan2f(dy,dx)*180.0f/ROUTE_PI);
    opposite=normalized(travel+180);
    first_turn=fabsf(delta(travel,yaw));other_turn=fabsf(delta(opposite,yaw));
    if(other_turn+0.01f<first_turn) travel=opposite;
    else if(fabsf(first_turn-other_turn)<0.01f)
    {
      if(fabsf(dx)<0.01f) travel=90;
      else if(fabsf(dy)<0.01f) travel=180;
    }
  }
  east_west=fabsf(dy)<0.01f && fabsf(dx)>0.01f;
  if(!first_scan_leg && east_west && hypotf(dx,dy)>300.0f &&
     (at(from,350,1200) || at(to,350,1200))) travel=180;
  final_yaw=to->station || at(to,2100,1200) || at(to,2100,300) || at(to,350,1200)?90:travel;
  /* Detours can approach at a short retained heading other than90. Restore
   * station yaw at the absolute staging point before entering the dock. */
  temp_arrival=at(to,350,1200) &&
      ((!first_scan_leg && east_west) || fabsf(delta(travel,90))>0.01f);
  temp_departure=at(from,350,1200) &&
      ((!first_scan_leg && east_west) || fabsf(delta(travel,90))>0.01f || fabsf(delta(travel,yaw))>0.01f);
  dock_speed=fminf(speed_mm_s,50.0f);
  if(temp_departure)
    add(out,false,"undock",&x,&y,&yaw,425,1200,90,dock_speed);
  /* Match the confirmed QR entrance: retain the start heading while leaving
   * the lower dock, then turn at the lane entrance. Detour coordinates stay intact. */
  if(first_scan_leg && at(from,2250,150))
  {
    add(out,false,"move",&x,&y,&yaw,at(to,2100,1200)?2100:to->x_mm,
        at(to,2100,1200)?300:to->y_mm,yaw,speed_mm_s);
    add(out,true,"turn",&x,&y,&yaw,x,y,90,0);
  }
  add(out,true,"turn",&x,&y,&yaw,x,y,travel,0);
  if(temp_arrival)
  {
    add(out,false,"dock_approach",&x,&y,&yaw,425,1200,travel,speed_mm_s);
    add(out,true,"arrival_turn",&x,&y,&yaw,x,y,90,0);
    add(out,false,"dock",&x,&y,&yaw,350,1200,90,dock_speed);
  }
  else
  {
    add(out,false,"move",&x,&y,&yaw,to->x_mm,to->y_mm,travel,speed_mm_s);
    add(out,true,"arrival_turn",&x,&y,&yaw,x,y,final_yaw,0);
  }
  out->final_yaw_deg=final_yaw;
  return true;
}

bool ChassisRouteProfile_BuildTask(const RadarPlan_Point_t *from,const RadarPlan_Point_t *to,
                                 float previous_yaw_deg,float speed_mm_s,bool first_scan_leg,
                                 ChassisRoute_Profile_t *out)
{
  float x,y,yaw,dx,dy,length,travel,work,stage_x,stage_y,dock;
  ChassisRoute_Profile_t transit;
  RadarPlan_Point_t neutral;
  if(!from || !to || !out || !isfinite(previous_yaw_deg) ||
     !isfinite(speed_mm_s) || speed_mm_s<=0) return false;
  if(to->station!=4U && to->station!=2U && to->station!=3U)
    return ChassisRouteProfile_Build(from,to,previous_yaw_deg,speed_mm_s,first_scan_leg,out);
  memset(out,0,sizeof(*out));
  x=from->x_mm;y=from->y_mm;yaw=normalized(previous_yaw_deg);
  dx=to->x_mm-x;dy=to->y_mm-y;length=hypotf(dx,dy);
  work=to->station==4U?0:(to->station==2U?180:90);
  travel=yaw;
  if(to->station==4U && fabsf(dy)<0.01f) travel=90; /* left translation before raw */
  else if(to->station==2U && fabsf(dx)<0.01f && dy<0) travel=-90;
  else if(to->station==3U) travel=90;
  else if(length>300)
  {
    neutral=*to;neutral.station=0;
    if(!ChassisRouteProfile_Build(from,&neutral,yaw,speed_mm_s,false,&transit)) return false;
    travel=transit.phases[0].yaw_deg;
  }
  dock=fminf(length,75.0f);
  stage_x=length>0?to->x_mm-dx*dock/length:x;
  stage_y=length>0?to->y_mm-dy*dock/length:y;
  add(out,true,"turn",&x,&y,&yaw,x,y,travel,0);
  add(out,false,"work_approach",&x,&y,&yaw,stage_x,stage_y,travel,speed_mm_s);
  add(out,true,"work_turn",&x,&y,&yaw,x,y,work,0);
  add(out,false,"work_dock",&x,&y,&yaw,to->x_mm,to->y_mm,work,fminf(speed_mm_s,50));
  out->final_yaw_deg=work;
  return true;
}
