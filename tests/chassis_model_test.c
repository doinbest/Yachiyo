#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "chassis_model.h"
#define NEAR(a,b,e) assert(fabsf((a)-(b))<(e))
int main(void)
{
  ChassisModel_Geometry_t g={210,240,80,1};
  ChassisModel_Velocity_t v={100,0,0}, back;
  ChassisModel_Wheels_t w;
  ChassisModel_Pose_t pose={0,0,0};
  float x,y;
  assert(ChassisModel_Inverse(&g,&v,100,&w));
  for(unsigned i=0;i<4;i++){NEAR(w.rpm[i],23.87324f,.0001f);assert(w.command[i]==24);}
  assert(ChassisModel_Forward(&g,w.rpm,&back));NEAR(back.vx_mm_s,100,.001f);
  v.vx_mm_s=0;v.vy_mm_s=100;
  assert(ChassisModel_Inverse(&g,&v,100,&w));assert(w.command[0]==-24&&w.command[1]==24&&w.command[2]==-24&&w.command[3]==24);
  v.vy_mm_s=0;v.omega_rad_s=.1f;
  assert(ChassisModel_Inverse(&g,&v,100,&w));NEAR(w.rpm[0],-5.371479f,.0001f);NEAR(w.rpm[3],5.371479f,.0001f);
  assert(ChassisModel_BodyToMap(100,0,CHASSIS_MODEL_PI/2,&x,&y));NEAR(x,0,.0001f);NEAR(y,100,.001f);
  assert(ChassisModel_MapToBody(100,0,CHASSIS_MODEL_PI/2,&x,&y));NEAR(x,0,.0001f);NEAR(y,-100,.001f);
  v.vx_mm_s=400;v.vy_mm_s=300;v.omega_rad_s=.8f;
  assert(ChassisModel_Inverse(&g,&v,100,&w));assert(w.scale<1);NEAR(w.rpm[3],100,.0001f);
  assert(ChassisModel_Forward(&g,w.rpm,&back));NEAR(back.vx_mm_s,v.vx_mm_s*w.scale,.001f);NEAR(back.vy_mm_s,v.vy_mm_s*w.scale,.001f);NEAR(back.omega_rad_s,v.omega_rad_s*w.scale,.00001f);
  v.vx_mm_s=8;v.vy_mm_s=0;v.omega_rad_s=0;assert(ChassisModel_Inverse(&g,&v,100,&w));assert(w.command[0]==2);
  v.vx_mm_s=15;assert(ChassisModel_Inverse(&g,&v,100,&w));assert(w.command[0]==4);
  v.vx_mm_s=-15;assert(ChassisModel_Inverse(&g,&v,100,&w));assert(w.command[0]==-4);
  v.vx_mm_s=NAN;assert(!ChassisModel_Inverse(&g,&v,100,&w));
  v.vx_mm_s=100;g.wheel_diameter_mm=0;assert(!ChassisModel_Inverse(&g,&v,100,&w));g.wheel_diameter_mm=80;
  assert(!ChassisModel_Inverse(&g,&v,0,&w));assert(!ChassisModel_Inverse(&g,&v,INFINITY,&w));
  v.omega_rad_s=.1f;
  for(unsigned i=0;i<1000;i++)assert(ChassisModel_IntegrateMidpoint(&pose,&v,.01f));
  NEAR(pose.x_mm,1000*sinf(1),.1f);NEAR(pose.y_mm,1000*(1-cosf(1)),.1f);NEAR(pose.yaw_rad,1,.0001f);
  assert(!ChassisModel_IntegrateMidpoint(&pose,&v,NAN));assert(!ChassisModel_BodyToMap(1,2,INFINITY,&x,&y));
  puts("chassis_model_test: PASS (baseline, transform, scaling, quantization, guards, analytic circle)");
}
