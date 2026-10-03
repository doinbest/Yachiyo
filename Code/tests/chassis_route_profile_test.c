#include "chassis_route_profile.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void close_to(float a,float b) { assert(fabsf(a-b)<0.01f); }
static ChassisRoute_Profile_t build(RadarPlan_Point_t from,RadarPlan_Point_t to,float yaw,bool first)
{
  ChassisRoute_Profile_t p;
  assert(ChassisRouteProfile_Build(&from,&to,yaw,200,first,&p));
  assert(p.count<=CHASSIS_ROUTE_PROFILE_MAX_PHASES);
  return p;
}
int main(void)
{
  const RadarPlan_Point_t expected[]={
    {2250,150,1,0},{2100,1200,5,1},{2100,2100,0,0},{1200,2100,4,2},
    {1200,350,2,3},{1200,1200,0,0},{350,1200,3,4},{1200,1200,0,0},
    {1200,2100,4,5},{1200,350,2,6},{1200,1200,0,0},{350,1200,3,7},
    {1200,1200,0,0},{2100,1200,0,0},{2100,300,0,0},{2250,150,1,8}};
  RadarPlan_Point_t point;
  assert(CHASSIS_ROUTE_TARGET_COUNT==15);
  for(unsigned i=0;i<=15;i++) {
    assert(ChassisRouteProfile_FixedPoint(i,&point));
    assert(point.x_mm==expected[i].x_mm && point.y_mm==expected[i].y_mm);
    assert(point.station==expected[i].station && point.visit==expected[i].visit);
  }
  assert(!ChassisRouteProfile_FixedPoint(16,&point));
  assert(!ChassisRouteProfile_FixedPoint(0,NULL));
  ChassisRoute_Profile_t first=build(expected[0],expected[1],90,true);
  assert(first.count==2);
  close_to(first.phases[0].x_mm,2100);close_to(first.phases[0].y_mm,300);
  assert(!first.phases[0].turn && !strcmp(first.phases[0].direction,"diagonal"));
  close_to(first.phases[1].x_mm,2100);close_to(first.phases[1].y_mm,1200);
  assert(!first.phases[1].turn && !strcmp(first.phases[1].direction,"forward"));
  close_to(first.nominal_time_s,(hypotf(150,150)+900)/200);
  /* A later pass and a planned detour must keep their own geometry. */
  first=build(expected[0],expected[1],90,false);assert(first.count==3);
  for(unsigned j=0;j<first.count;j++) assert(first.phases[j].y_mm!=300);
  first=build((RadarPlan_Point_t){1800,800,0,0},expected[1],90,true);
  assert(first.count==1);close_to(first.phases[0].y_mm,1200);
  float yaw=90;
  for(unsigned i=1;i<=15;i++) {
    ChassisRoute_Profile_t p=build(expected[i-1],expected[i],yaw,i==1);
    if(expected[i].station || i>=13) close_to(p.final_yaw_deg,90);
    if(i==1 || i==15) for(unsigned j=0;j<p.count;j++) close_to(p.phases[j].yaw_deg,90);
    for(unsigned j=0;j<p.count;j++)
      if(p.phases[j].turn) assert(p.phases[j].x_mm!=350 || p.phases[j].y_mm!=1200);
    assert(p.nominal_time_s>0);
    yaw=p.final_yaw_deg;
  }
  ChassisRoute_Profile_t p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},90,false);
  assert(p.count==4);
  assert(p.phases[0].turn && !strcmp(p.phases[0].phase,"turn"));close_to(p.phases[0].yaw_deg,180);
  assert(!strcmp(p.phases[1].phase,"dock_approach") && !strcmp(p.phases[1].direction,"forward"));
  close_to(p.phases[1].x_mm,425);close_to(p.phases[1].yaw_deg,180);
  assert(!strcmp(p.phases[2].phase,"arrival_turn"));close_to(p.phases[2].x_mm,425);close_to(p.phases[2].yaw_deg,90);
  assert(!strcmp(p.phases[3].phase,"dock") && !strcmp(p.phases[3].direction,"lateral"));
  close_to(p.phases[3].x_mm,350);close_to(p.phases[3].speed_mm_s,50);
  close_to(p.nominal_time_s,775.0f/200+75.0f/50+180.0f/100.0f);
  p=build((RadarPlan_Point_t){350,1200,3,4},(RadarPlan_Point_t){1200,1200,0,0},90,false);
  assert(p.count==3 && !strcmp(p.phases[0].phase,"undock"));close_to(p.phases[0].x_mm,425);close_to(p.phases[0].yaw_deg,90);close_to(p.phases[0].speed_mm_s,50);
  assert(p.phases[1].turn);close_to(p.phases[1].x_mm,425);close_to(p.phases[1].yaw_deg,180);
  assert(!strcmp(p.phases[2].direction,"reverse"));close_to(p.final_yaw_deg,180);
  p=build((RadarPlan_Point_t){350,600,0,0},(RadarPlan_Point_t){350,1200,3,4},90,false);
  assert(p.count==1 && !strcmp(p.phases[0].phase,"move"));
  /* Planned detours may bring a different accepted yaw near the dock. */
  p=build((RadarPlan_Point_t){900,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},0,false);
  assert(p.phases[0].turn);close_to(p.phases[0].yaw_deg,180);
  assert(p.phases[1].x_mm==425 && !strcmp(p.phases[1].phase,"dock_approach"));
  p=build((RadarPlan_Point_t){600,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},0,false);
  close_to(p.phases[0].yaw_deg,0);assert(p.phases[0].x_mm==425);
  assert(p.phases[1].turn && p.phases[1].x_mm==425);
  p=build((RadarPlan_Point_t){350,1000,0,0},(RadarPlan_Point_t){350,1200,3,4},180,false);
  for(unsigned j=0;j<p.count;j++)
    if(p.phases[j].turn) assert(p.phases[j].x_mm!=350 || p.phases[j].y_mm!=1200);
  p=build((RadarPlan_Point_t){600,1200,0,0},(RadarPlan_Point_t){350,1200,0,0},0,false);
  close_to(p.final_yaw_deg,90);assert(p.phases[p.count-1].yaw_deg==90);
  p=build((RadarPlan_Point_t){900,1200,0,0},(RadarPlan_Point_t){350,1200,0,0},90,true);
  assert(p.count==1);close_to(p.phases[0].yaw_deg,90);
  p=build((RadarPlan_Point_t){600,1200,0,0},(RadarPlan_Point_t){350,1200,3,4},90,false);
  assert(p.count==2 && !p.phases[0].turn && !p.phases[1].turn);
  assert(!strcmp(p.phases[0].phase,"dock_approach") && !strcmp(p.phases[1].phase,"dock"));
  close_to(p.phases[0].x_mm,425);close_to(p.phases[1].speed_mm_s,50);
  p=build((RadarPlan_Point_t){350,1200,3,4},(RadarPlan_Point_t){600,1200,0,0},90,false);
  assert(p.count==2 && !p.phases[0].turn && !p.phases[1].turn);
  assert(!strcmp(p.phases[0].phase,"undock"));close_to(p.phases[0].x_mm,425);
  close_to(p.phases[0].speed_mm_s,50);close_to(p.final_yaw_deg,90);
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){2100,1200,0,0},180,false);
  assert(p.count==2 && !strcmp(p.phases[0].direction,"reverse") && !strcmp(p.phases[1].phase,"arrival_turn"));
  /* QR geometry restores90 even if no business label is attached. */
  close_to(p.final_yaw_deg,90);
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){1200,1500,0,0},180,false);
  assert(p.count==1);close_to(p.final_yaw_deg,180);assert(!strcmp(p.phases[0].direction,"lateral"));
  p=build((RadarPlan_Point_t){1200,1200,0,0},(RadarPlan_Point_t){1200,1501,0,0},180,false);
  assert(p.phases[0].turn);close_to(p.final_yaw_deg,90);
  p=build((RadarPlan_Point_t){2250,150,1,0},(RadarPlan_Point_t){1600,150,0,0},90,true);
  assert(p.count==1);close_to(p.final_yaw_deg,90);assert(!strcmp(p.phases[0].direction,"lateral"));
  p=build((RadarPlan_Point_t){1200,2100,0,0},(RadarPlan_Point_t){800,2000,0,0},180,false);
  assert(p.phases[0].turn && p.phases[0].yaw_deg>=-180 && p.phases[0].yaw_deg<=180);
  close_to(p.final_yaw_deg,-165.96376f);
  for(unsigned i=0;i<p.count;i++)assert(p.phases[i].yaw_deg>=-180 && p.phases[i].yaw_deg<=180);
  RadarPlan_Point_t a={1200,1200,0,0},b={350,1200,3,4};
  assert(ChassisRouteProfile_Build(&a,&b,90,30,false,&p));close_to(p.phases[p.count-1].speed_mm_s,30);
  assert(!ChassisRouteProfile_Build(&a,&b,NAN,200,false,&p));
  assert(!ChassisRouteProfile_Build(&a,&b,90,0,false,&p));
  puts("chassis_route_profile_test: PASS (15 targets, nominal headings, absolute dock phases)");
}
