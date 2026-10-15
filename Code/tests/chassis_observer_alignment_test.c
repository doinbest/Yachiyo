#include "chassis_observer.h"
static ChassisObserver_History_t history;
#include <assert.h>
#include <math.h>
#include <stdio.h>

static double travel(uint32_t tick)
{
  double t=((double)tick-1300)/1000, pi=3.141592653589793;
  if(t<=0) return 0;
  if(t<.4) return 500*(t-.4/pi*sin(pi*t/.4));
  if(t<.8) return 200+1000*(t-.4);
  if(t<1.2) return 600+500*(t-.8+.4/pi*sin(pi*(t-.8)/.4));
  return 800;
}
static void straight(int direction, uint32_t epoch)
{
  ChassisObserver_t observer;
  ChassisObserver_Feedback_t sample={0};
  ChassisModel_Geometry_t geometry={210,240,80,1};
  float maximum=0;
  unsigned valid=0;
  ChassisObserver_Init(&observer,0,0,0);
  sample.yaw_valid=true; sample.units_per_rev=65536;
  for(uint32_t base=1000;base<=3000;base+=100) {
    sample.now_ms=epoch+base+30;
    for(unsigned w=0;w<4;w++) {
      uint32_t t=base+w*10;
      sample.time_ms[w]=epoch+t;sample.valid[w]=true;
      sample.position[w]=(int64_t)llround(direction*travel(t)*65536/(80*3.141592653589793));
    }
    ChassisObserver_Feedback(&observer,&geometry,&sample,&history);
    if(observer.feedback_valid) {
      valid++;
      assert(observer.feedback_ms==epoch+base);
      if(fabsf(observer.feedback.y_mm)>maximum) maximum=fabsf(observer.feedback.y_mm);
    }
  }
  printf("aligned straight direction=%d epoch=%lu max false Y=%.6f final X=%.3f Y=%.6f valid=%u\n",
      direction,(unsigned long)epoch,maximum,observer.feedback.x_mm,observer.feedback.y_mm,valid);
  fflush(stdout);
  assert(valid>=15);
  assert(maximum<2.0f);
  assert(fabsf(observer.feedback.y_mm)<.02f);
  assert(fabsf(observer.feedback.x_mm-direction*800)<.03f);
}
static void arc(float forward)
{
  ChassisObserver_t observer;
  ChassisObserver_Feedback_t sample={0};
  ChassisModel_Geometry_t geometry={210,240,80,1};
  const float k=225, yaw_start=3;
  ChassisObserver_Init(&observer,0,0,yaw_start);
  sample.yaw_valid=true;sample.units_per_rev=65536;
  for(uint32_t base=1000;base<=3000;base+=100) {
    sample.now_ms=base+30;
    sample.yaw_time_valid=true;sample.yaw_ms=base;
    sample.yaw_rad=remainderf(yaw_start+(base-1000)/1000.0f,2*CHASSIS_MODEL_PI);
    for(unsigned w=0;w<4;w++) {
      uint32_t t=base+w*10;
      double speed=forward+(w<2?-k:k);
      sample.position[w]=(int64_t)llround(speed*(t-1000)/1000*65536/(80*3.141592653589793));
      sample.time_ms[w]=t;sample.valid[w]=true;
    }
    ChassisObserver_Feedback(&observer,&geometry,&sample,&history);
  }
  assert(observer.feedback_valid && observer.feedback_ms==3000);
  assert(fabsf(remainderf(observer.feedback.yaw_rad-5,2*CHASSIS_MODEL_PI))<.0001f);
  assert(fabsf(observer.feedback.x_mm-forward*(sinf(5)-sinf(3.1f)))<.2f);
  assert(fabsf(observer.feedback.y_mm-forward*(cosf(3.1f)-cosf(5)))<.2f);
  float x=observer.feedback.x_mm,y=observer.feedback.y_mm;
  uint32_t time=observer.feedback_ms;
  sample.now_ms+=20;sample.time_ms[0]+=20;sample.position[0]+=100;
  ChassisObserver_Feedback(&observer,&geometry,&sample,&history);
  assert(observer.feedback_ms==time && observer.feedback.x_mm==x && observer.feedback.y_mm==y);
  sample.now_ms+=700;
  ChassisObserver_Feedback(&observer,&geometry,&sample,&history);
  assert(!observer.feedback_valid && observer.feedback.x_mm==x && observer.feedback.y_mm==y);
  printf("aligned arc vx=%.0f: common-time yaw, +/-pi wrap, partial group and stale gap PASS\n",forward);
}
static void slower_heading(void)
{
  ChassisObserver_t observer;
  ChassisObserver_Feedback_t sample={0};
  ChassisModel_Geometry_t geometry={210,240,80,1};
  ChassisObserver_Init(&observer,0,0,3);
  sample.yaw_valid=sample.yaw_time_valid=true;sample.units_per_rev=65536;
  unsigned updates=0;
  uint32_t previous=0;
  for(uint32_t base=1000;base<=3000;base+=20) {
    sample.now_ms=base+9;sample.yaw_ms=(sample.now_ms/100)*100;
    float seconds=sample.yaw_ms>1300?(sample.yaw_ms-1300)/1000.f:0;
    sample.yaw_rad=remainderf(3+seconds,2*CHASSIS_MODEL_PI);
    for(unsigned w=0;w<4;w++) {
      uint32_t time=base+w*3;
      double t=time>1300?(time-1300)/1000.0:0;
      sample.time_ms[w]=time;sample.valid[w]=true;
      sample.position[w]=(int64_t)llround((200+(w<2?-225:225))*t*65536/(80*3.141592653589793));
    }
    ChassisObserver_Feedback(&observer,&geometry,&sample,&history);
    if(observer.feedback_valid && observer.feedback_ms!=previous) {
      updates++;previous=observer.feedback_ms;
      assert(observer.feedback_ms<=sample.yaw_ms);
    }
  }
  assert(updates>=17 && observer.feedback_valid && observer.feedback_ms==3000);
  assert(fabsf(observer.feedback.x_mm-200*(sinf(4.7f)-sinf(3)))<.2f);
  assert(fabsf(observer.feedback.y_mm-200*(cosf(3)-cosf(4.7f)))<.2f);
  puts("observer alignment: 20ms wheel groups / 100ms IMU, cached yaw never relabeled PASS");
}
int main(void)
{
  straight(1,0);straight(-1,0);straight(1,UINT32_MAX-1800U);
  arc(200);arc(0);
  slower_heading();
  puts("observer alignment: cosine start/stop, forward/backward, tick wrap PASS");
}
