#include "chassis_observer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
  ChassisObserver_t o;
  ChassisObserver_Feedback_t f={0};
  ChassisModel_Geometry_t g={210,240,80,1};
  float rpm[4]={60,60,60,60};
  unsigned i;
  ChassisObserver_Init(&o,0,0,0);
  ChassisObserver_Command(&o,&g,rpm,100,100,true);
  ChassisObserver_Command(&o,&g,rpm,120,100,true);
  assert(fabsf(o.command.x_mm-5.026548f)<0.001f);
  ChassisObserver_Command(&o,&g,rpm,1000,100,true);
  assert(!o.command_valid && o.command.x_mm<6);
  {float zero[4]={0};
    ChassisObserver_t step;
    ChassisObserver_Init(&step,0,0,0);
    ChassisObserver_Command(&step,&g,zero,100,100,true);
    ChassisObserver_Command(&step,&g,rpm,120,110,true);
    assert(fabsf(step.command.x_mm-2.513274f)<0.001f);
    ChassisObserver_Command(&step,&g,zero,140,130,true);
    assert(fabsf(step.command.x_mm-5.026548f)<0.001f);
  }
  f.yaw_valid=true; f.units_per_rev=65536; f.now_ms=1000;
  for(i=0;i<4;i++){ f.valid[i]=true; f.time_ms[i]=1000; }
  ChassisObserver_Feedback(&o,&g,&f); assert(!o.feedback_valid);
  f.now_ms=1200;
  for(i=0;i<4;i++){f.position[i]=65536;f.time_ms[i]=1200;}
  ChassisObserver_Feedback(&o,&g,&f);
  assert(o.feedback_valid && fabsf(o.feedback.x_mm-251.3274f)<0.01f);
  f.now_ms=2000; ChassisObserver_Feedback(&o,&g,&f); assert(!o.feedback_valid);
  for(i=0;i<4;i++) {f.position[i]=655360;f.time_ms[i]=2000;}
  ChassisObserver_Feedback(&o,&g,&f); assert(!o.feedback_valid);
  assert(o.feedback.x_mm<252); /* stale recovery never integrates the gap */
  f.units_per_rev=0; ChassisObserver_Feedback(&o,&g,&f); assert(!o.feedback_valid);
  puts("chassis observer: command integration, stale gaps, two sets, units guard PASS");
}
