#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"

int main(void)
{
  (void)zero_dispatched;
  setup();
  ChassisPath path;assert(ChassisPath_Build(&path,3,1200,1200,1200,2100,1000));
  assert(ChassisMotion_PathStartScaled(&path,1200,100,200000,0,1.2f));
  assert(fabsf(status().linear_accel_mm_s2-360)<.01f);
  assert(fabsf(status().linear_decel_mm_s2-360)<.01f);
  sample(120);ChassisMotion_Process();
  assert(fabsf(hypotf(status().body_target.vx_mm_s,status().body_target.vy_mm_s)-7.2f)<.01f);
  /* At the original60mm/s section, use its scaled72 cap, not the old60. */
  location.observer.feedback.y_mm=1950;
  for(unsigned t=140;t<=1140;t+=20){sample(t);ChassisMotion_Process();}
  float speed=hypotf(status().body_target.vx_mm_s,status().body_target.vy_mm_s);
  assert(speed>71 && speed<73);
  unsigned id=status().action_id;
  assert(!ChassisMotion_PathStartScaled(&path,1200,100,200000,0,NAN) && status().action_id==id);
  assert(!ChassisMotion_PathStartScaled(&path,1200,100,200000,0,6) && status().action_id==id);
  setup();assert(ChassisMotion_PathStart(&path,1000,100,200000,0));
  sample(120);ChassisMotion_Process();
  assert(fabsf(hypotf(status().body_target.vx_mm_s,status().body_target.vy_mm_s)-6)<.01f);
  assert(status().linear_accel_mm_s2==300 && status().linear_decel_mm_s2==300);
  puts("mission scale motion: PASS");return 0;
}
