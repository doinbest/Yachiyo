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
  RadarPlan_Point_t corner={2100,2100,0,0},raw={1200,2100,4,2};
  ChassisRoute_Profile_t p=build(corner,raw,90);
  assert(p.count==3 && !p.phases[0].turn);
  assert(p.phases[0].x_mm==1275 && p.phases[0].yaw_deg==90);
  assert(!strcmp(p.phases[0].direction,"lateral"));
  assert(p.phases[1].turn && p.phases[1].yaw_deg==0);
  assert(!strcmp(p.phases[1].phase,"work_turn"));
  assert(p.phases[2].x_mm==1200 && p.phases[2].yaw_deg==0);
  assert(!strcmp(p.phases[2].direction,"reverse") && p.final_yaw_deg==0);
  p=build(raw,(RadarPlan_Point_t){1200,350,2,3},0);
  assert(p.phases[0].turn && p.phases[0].yaw_deg==-90);
  assert(p.phases[1].y_mm==425 && p.phases[1].yaw_deg==-90);
  assert(p.phases[2].turn && p.phases[2].yaw_deg==180);
  assert(p.final_yaw_deg==180 && p.phases[p.count-1].y_mm==350);
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},90);
  assert(p.count==2 && !p.phases[0].turn && p.phases[0].yaw_deg==90);
  assert(p.phases[0].x_mm==425 && p.phases[1].x_mm==350 && p.final_yaw_deg==90);
  p=build((RadarPlan_Point_t){1200,2050,0,0},raw,180);
  assert(p.final_yaw_deg==0 && p.phases[p.count-1].x_mm==1200 && p.phases[p.count-1].y_mm==2100);
  for(unsigned i=0;i<p.count;i++) assert(p.phases[i].x_mm==1200); /* no artificial sideways detour */
  puts("chassis_task_profile_test: PASS (arm-facing stations, lateral corner, retained detours)");
}
