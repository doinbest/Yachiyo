#include "chassis_route_profile.h"
#include "chassis_path.h"
#include <stdio.h>
#include <string.h>
int main(void) {
  RadarPlan_Point_t a,b;
  float yaw=180;
  ChassisRouteProfile_FixedPoint(0,&a);
  puts("leg,phase,kind,x0,y0,x1,y1,yaw0,yaw1,speed,length");
  for(unsigned leg=1;leg<=15;leg++) {
    ChassisRoute_Profile_t p;
    ChassisRouteProfile_FixedPoint(leg,&b);
    if(!ChassisRouteProfile_BuildTask(&a,&b,yaw,1000,leg==1,&p))return 1;
    float x=a.x_mm,y=a.y_mm;
    for(unsigned j=0;j<p.count;j++) {
      ChassisRoute_Phase_t q=p.phases[j];
      unsigned kind=0;
      float length=hypotf(q.x_mm-x,q.y_mm-y);
      if(!strcmp(q.direction,"curve")) {
        kind=!strcmp(q.phase,"entry_curve")?1:a.x_mm==2100?2:3;
        ChassisPath path;ChassisPath_Build(&path,kind,x,y,q.x_mm,q.y_mm,1000);
        length=path.length;
      }
      printf("%u,%s,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",leg,q.phase,kind,x,y,q.x_mm,q.y_mm,yaw,q.yaw_deg,q.speed_mm_s,length);
      x=q.x_mm;y=q.y_mm;yaw=q.yaw_deg;
    }
    a=b;
  }
}
