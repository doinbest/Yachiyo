#include "chassis_route_profile.h"
#include "chassis_path.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
int main(void)
{
  ChassisPath path;
  assert(ChassisPath_Build(&path,1,2250,150,2100,1250,1000));
  assert(path.count<=CHASSIS_PATH_CAPACITY);
  /* Approach keeps the initial heading; turn is spread through the corridor,
   * with >=45 mm lateral clearance for the complete 300 mm square. */
  assert(path.point[8].x==2125 && path.point[8].y==350 && path.point[8].yaw==180);
  for(unsigned i=1;i<=8;i++)assert(path.point[i].yaw==180 && path.point[i].limit==200);
  assert(path.point[path.count-2].x==2100 && path.point[path.count-2].y==950);
  assert(path.point[path.count-2].yaw==90);
  for(float s=path.point[8].s;s<=path.turn_end;s+=.25f){
    ChassisPath_Point p=ChassisPath_Sample(&path,s);
    float a=p.yaw*3.14159265358979323846f/180;
    float half=150*(fabsf(cosf(a))+fabsf(sinf(a)));
    assert(p.x-half>=1850+45 && p.x+half<=2400-45);
  }
  for(unsigned kind=2;kind<=3;kind++){
    assert(ChassisPath_Build(&path,kind,kind==2?2100:1200,kind==2?2100:1200,1200,2100,1000));
    assert(path.count==23 && path.point[1].limit==1000);
    for(unsigned i=2;i+1<path.count;i++)assert(path.point[i].limit==60);
    assert(ChassisPath_Sample(&path,path.point[1].s*.5f).limit==1000);
    assert(ChassisPath_Build(&path,kind,kind==2?2100:1200,kind==2?2100:1200,1200,2100,40));
    for(unsigned i=1;i+1<path.count;i++)assert(path.point[i].limit==40);
  }
  ChassisRoute_Profile_t p;
  RadarPlan_Point_t start={2250,150,1,0},qr={2100,1250,5,1};
  assert(ChassisRouteProfile_BuildTask(&start,&qr,180,1000,true,&p));
  assert(p.count==1 && !p.phases[0].turn && !strcmp(p.phases[0].phase,"entry_curve"));
  assert(p.phases[0].x_mm==2100 && p.phases[0].y_mm==1250 && p.final_yaw_deg==90);
  RadarPlan_Point_t corner={2100,2100,0,0},raw={1200,2100,4,2};
  assert(ChassisRouteProfile_BuildTask(&corner,&raw,90,1000,false,&p));
  assert(p.count==2 && !strcmp(p.phases[0].phase,"work_approach"));
  assert(p.phases[0].yaw_deg==90 && !p.phases[0].turn);
  assert(p.phases[1].turn && p.phases[1].yaw_deg==0);
  corner.x_mm=1200;corner.y_mm=1200;
  assert(ChassisRouteProfile_BuildTask(&corner,&raw,90,1000,false,&p));
  assert(p.count==1 && !strcmp(p.phases[0].phase,"work_curve"));
  float accels[CHASSIS_ROUTE_PROFILE_MAX_PHASES];
  ChassisRouteProfile_ApplyTaskLinear(&corner,&raw,&p,true,accels);
  assert(p.phases[0].speed_mm_s==800 && accels[0]==300);
  /* A planner's nonstandard entry remains its entry, not a fixed shortcut. */
  corner.x_mm=1500;corner.y_mm=1800;
  assert(ChassisRouteProfile_BuildTask(&corner,&raw,90,1000,false,&p));
  assert(strcmp(p.phases[0].phase,"work_curve"));
  assert(ChassisRouteProfile_Build(&start,&qr,180,1000,true,&p));
  assert(p.count==3 && p.phases[1].turn); /* standalone route unchanged */
  puts("chassis_curve_profile_test: PASS");
}
