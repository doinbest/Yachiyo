#include "chassis_route_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Until the task API exists, exercise the old route to show the wrong approach. */
#ifndef CHASSIS_TASK_PROFILE
#define ChassisRouteProfile_BuildTask ChassisRouteProfile_Build
#endif
static ChassisRoute_Profile_t build(RadarPlan_Point_t a,RadarPlan_Point_t b,float yaw)
{
  ChassisRoute_Profile_t p;
  assert(ChassisRouteProfile_BuildTask(&a,&b,yaw,200,false,&p));
  return p;
}
int main(void)
{
  RadarPlan_Point_t lower={2250,150,1,0},qr={2100,1250,5,1};
  ChassisRoute_Profile_t entry;
  assert(ChassisRouteProfile_BuildTaskEntry(&lower,&qr,180,1000,true,false,&entry));
  assert(entry.count==3 && !entry.phases[0].turn);
  assert(entry.phases[0].x_mm==2100 && entry.phases[0].y_mm==300 && entry.phases[0].yaw_deg==180);
  assert(entry.phases[1].turn && entry.phases[1].yaw_deg==90);
  assert(entry.phases[2].x_mm==2100 && entry.phases[2].y_mm==1250 && entry.phases[2].yaw_deg==90);
  assert(entry.phases[0].speed_mm_s==1000 && entry.phases[2].speed_mm_s==1000);
  assert(ChassisRouteProfile_BuildTaskEntry(&lower,&qr,180,1000,true,true,&entry));
  assert(entry.count==1 && !strcmp(entry.phases[0].phase,"entry_curve"));
  RadarPlan_Point_t proc;
  assert(ChassisRouteProfile_FixedPoint(4,&proc) && proc.y_mm==380);
  ChassisRoute_Profile_t approach=build((RadarPlan_Point_t){1200,1200,0,0},proc,-90);
  assert(approach.phases[0].y_mm==380 && !approach.phases[0].turn);
  assert(approach.phases[1].turn && approach.phases[1].y_mm==380);
  assert(approach.count==2 && approach.final_yaw_deg==180);
  ChassisRoute_Profile_t leaving=build(proc,(RadarPlan_Point_t){1200,1200,0,0},180);
  assert(!leaving.phases[0].turn && leaving.phases[0].y_mm==425 && leaving.phases[0].yaw_deg==180);
  RadarPlan_Point_t corner={2100,2100,0,0},raw={1200,2100,4,2};
  ChassisRoute_Profile_t north=build((RadarPlan_Point_t){2100,1250,5,1},corner,90);
  assert(north.count==1 && !north.phases[0].turn && north.final_yaw_deg==90);
  assert(north.phases[0].x_mm==2100 && north.phases[0].y_mm==2100);
  assert(!strcmp(north.phases[0].direction,"forward"));
  ChassisRoute_Profile_t p=build(corner,raw,90);
  assert(p.count==2 && !p.phases[0].turn);
  assert(p.phases[0].x_mm==1200 && p.phases[0].yaw_deg==90);
  assert(!strcmp(p.phases[0].direction,"lateral"));
  assert(!strcmp(p.phases[0].phase,"work_approach"));
  assert(p.phases[0].y_mm==2100 && p.final_yaw_deg==0);
  assert(p.phases[0].speed_mm_s==200);
  assert(p.phases[1].turn && !strcmp(p.phases[1].phase,"work_turn"));
  assert(p.phases[1].x_mm==1200 && p.phases[1].y_mm==2100 && p.phases[1].yaw_deg==0);
  p=build(raw,(RadarPlan_Point_t){1200,380,2,3},0);
  assert(p.phases[0].turn && p.phases[0].yaw_deg==-90);
  assert(p.phases[1].y_mm==380 && p.phases[1].yaw_deg==-90);
  assert(p.phases[2].turn && p.phases[2].yaw_deg==180);
  assert(p.final_yaw_deg==180 && p.phases[p.count-1].y_mm==380);
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},90);
  assert(p.count==4 && p.phases[0].turn && p.phases[0].yaw_deg==0);
  assert(p.phases[1].x_mm==425 && !strcmp(p.phases[1].direction,"reverse"));
  assert(p.phases[2].turn && p.phases[2].yaw_deg==90 && p.phases[2].x_mm==425);
  assert(p.phases[3].x_mm==350 && p.final_yaw_deg==90);
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},180);
  assert(p.count==3 && !p.phases[0].turn && !strcmp(p.phases[0].direction,"forward"));
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},0);
  assert(p.count==3 && !p.phases[0].turn && !strcmp(p.phases[0].direction,"reverse"));
  p=build((RadarPlan_Point_t){1200,2050,0,0},raw,180);
  assert(p.final_yaw_deg==0 && p.phases[p.count-1].x_mm==1200 && p.phases[p.count-1].y_mm==2100);
  for(unsigned i=0;i<p.count;i++) assert(p.phases[i].x_mm==1200); /* no artificial sideways detour */
  puts("chassis_task_profile_test: PASS (arm-facing stations, lateral corner, retained detours)");
}
