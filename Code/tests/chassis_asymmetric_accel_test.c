#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#define CHASSIS_DISTANCE_TEST_ENABLE_PLANT
#include "chassis_distance_test.c"
static void near(float a,float b){assert(fabsf(a-b)<.001f);}
static void start(float accel,float decel,float direction)
{
  setup();ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);
  p.linear_accel_mm_s2=accel;p.linear_decel_mm_s2=decel;
  assert(ChassisMotion_MoveToTuned(1200,1200+direction*2000,90,1000,&p,60000));
}
int main(void)
{
  for(unsigned reverse=0;reverse<2;reverse++){
    float direction=reverse?-1:1;start(500,250,direction);
    for(unsigned now=101;now<=300;now++){sample(now);ChassisMotion_Process();}
    near(sent_vx,direction*100);assert(status().linear_decel_mm_s2==250);
    location.observer.feedback.y_mm=1200+direction*1995;
    sample(301);ChassisMotion_Process();
    assert(status().state==CHASSIS_MOTION_RUNNING);near(sent_vx,direction*99.75f);
    for(unsigned now=302;now<=700;now++){sample(now);ChassisMotion_Process();}
    sample(701);ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_STOPPING);
  }
  start(500,250,1);for(unsigned now=101;now<=300;now++){sample(now);ChassisMotion_Process();}
  assert(ChassisMotion_Stop(0));assert(status().state==CHASSIS_MOTION_STOPPING);
  setup();assert(ChassisMotion_MoveToAccel(1200,3200,90,1000,800,60000));
  assert(status().linear_accel_mm_s2==800 && status().linear_decel_mm_s2==800);
  setup();assert(ChassisMotion_MoveTo(1200,3200,90,1000,60000));assert(status().linear_decel_mm_s2==300);
  const float invalid[]={0,9,5001,NAN,INFINITY};
  for(unsigned i=0;i<sizeof invalid/sizeof *invalid;i++){
    setup();ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);p.linear_decel_mm_s2=invalid[i];
    assert(!ChassisMotion_MoveToTuned(1200,3200,90,1000,&p,60000));
  }
  plant_profile=CHASSIS_PROFILE_LINEAR;
  delayed_plant_tuned(1000,1000,10,false,500,300);
  delayed_plant_tuned(2000,1000,10,false,500,300);
  delayed_plant_tuned(3000,1000,10,false,500,250);
  (void)delayed_plant;(void)zero_dispatched;puts("asymmetric ramps, soft linear arrival, reverse and legacy defaults passed");return 0;
}
