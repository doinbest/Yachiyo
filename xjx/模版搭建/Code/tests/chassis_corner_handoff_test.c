/* Prepared production-motion regressions; C execution is deferred by request. */
#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"

static void corner_start(void)
{
  setup();location.observer.feedback.x_mm=2100;
  ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);
  p.linear_accel_mm_s2=1000;p.linear_decel_mm_s2=300;
  assert(ChassisMotion_MoveToTuned(2100,2100,90,1000,&p,60000));
  ChassisMotion_UseRouteArrival();
  assert(ChassisMotion_PrepareCorner(1200,2100,700,800,120000));
}
int main(void)
{
  corner_start();unsigned id=status().action_id;
  for(unsigned t=120;t<=200;t+=20){sample(t);ChassisMotion_Process();}
  float previous_x=-status().body_target.vy_mm_s,previous_y=status().body_target.vx_mm_s;
  location.observer.feedback.y_mm=2095;sample(220);ChassisMotion_Process();
  assert(ChassisMotion_CornerPassed() && status().action_id==id && !stops);
  assert(status().state==CHASSIS_MOTION_RUNNING && !status().stop_confirmed);
  assert(status().target_x_mm==1200 && status().target_y_mm==2100);
  assert(status().linear_accel_mm_s2==800 && status().linear_decel_mm_s2==300);
  for(unsigned t=240;t<=400;t+=20){
    sample(t);ChassisMotion_Process();
    float x=-status().body_target.vy_mm_s,y=status().body_target.vx_mm_s;
    assert(hypotf(x-previous_x,y-previous_y)<=800*.02f+.01f);
    assert(x<0 && !stops && status().action_id==id);
    previous_x=x;previous_y=y;
  }
  assert(ChassisMotion_Stop(0));assert(ChassisMotion_CornerPassed());

  corner_start();assert(!ChassisMotion_CornerPassed());assert(ChassisMotion_Stop(0));
  location.observer.feedback.y_mm=2100;sample(120);ChassisMotion_Process();
  assert(!ChassisMotion_CornerPassed()); /* Stop cancels the pending handoff. */

  /* Two actual post-TX groups finish a normal route; duplicate data cannot. */
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));ChassisMotion_UseRouteArrival();
  sample(120);ChassisMotion_Process();zero_dispatched(140);
  sample(200);ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_STOPPING);
  ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_STOPPING);
  sample(220);location.wheels[3].position_ms=200;ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_STOPPING); /* One old wheel position cannot form group 2. */
  sample(240);ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_DONE && status().stop_confirmed);
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));
  sample(120);ChassisMotion_Process();zero_dispatched(140);
  sample(200);ChassisMotion_Process();sample(220);ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_STOPPING); /* Ordinary standalone stays at 3. */
  sample(240);ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_DONE);
  puts("corner motion: 10mm handoff, retained velocity/action, stop cancellation and route proof PASS");
  return 0;
}
