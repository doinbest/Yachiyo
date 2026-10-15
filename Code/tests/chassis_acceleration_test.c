#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
int main(void)
{
  setup();
  assert(ChassisMotion_MoveToAccel(1200,3200,90,1000,800,60000));
  assert(status().linear_accel_mm_s2==800);
  sample(120);ChassisMotion_Process();
  assert(fabsf(status().body_target.vx_mm_s-16)<.01f);
  assert(!ChassisMotion_MoveToAccel(1200,3200,90,1000,2000,60000));
  assert(status().linear_accel_mm_s2==800);
  /* Same acceleration controls the feedback-age-aware braking envelope. */
  location.observer.feedback.y_mm=3150;
  for(unsigned now=140;now<=2140;now+=20){sample(now);ChassisMotion_Process();}
  float fast=status().body_target.vx_mm_s;
  setup();assert(ChassisMotion_MoveToAccel(1200,3200,90,1000,300,60000));
  location.observer.feedback.y_mm=3150;
  for(unsigned now=120;now<=2140;now+=20){sample(now);ChassisMotion_Process();}
  assert(fast>status().body_target.vx_mm_s+10);
  setup();assert(ChassisMotion_MoveTo(1200,3200,90,1000,60000));
  assert(status().linear_accel_mm_s2==300);
  sample(120);ChassisMotion_Process();assert(fabsf(status().body_target.vx_mm_s-6)<.01f);
  const float invalid[]={0,9,5001,NAN,INFINITY};
  for(unsigned i=0;i<sizeof invalid/sizeof invalid[0];i++){
    setup();assert(!ChassisMotion_MoveToAccel(1200,3200,90,1000,invalid[i],60000));
    assert(!ChassisMotion_IsBusy());
  }
  /* Retain helpers used only by the included distance fixture. */
  (void)zero_dispatched;
  puts("chassis acceleration tests passed");return 0;
}
