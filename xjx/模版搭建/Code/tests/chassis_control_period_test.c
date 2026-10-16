#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
static void near(float a,float b){assert(fabsf(a-b)<.0001f);}
int main(void)
{
  setup();ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);
  assert(p.curve_period_ms==1);
  assert(ChassisMotion_MoveToTuned(1200,3200,90,1000,&p,60000));
  sample(101);ChassisMotion_Process();near(sent_vx,.3f);assert(status().control_dt_ms==1);
  sample(102);ChassisMotion_Process();near(sent_vx,.6f);
  setup();ChassisMotion_TestParametersDefaults(&p);p.curve_period_ms=5;p.heading_kp=.02f;
  assert(ChassisMotion_MoveToTuned(1200,3200,91,1000,&p,60000));
  for(unsigned now=101;now<105;now++){sample(now);ChassisMotion_Process();near(sent_vx,0);}
  sample(105);ChassisMotion_Process();near(sent_vx,1.5f);
  sample(106);reading(106,30.5f);ChassisMotion_Process();near(status().body_target.omega_rad_s,.01f);
  near(hypotf(sent_vx,sent_vy),1.5f); /* Heading changes while translation reference holds. */
  sample(110);ChassisMotion_Process();near(sent_vx,3);
  sample(117);ChassisMotion_Process();near(sent_vx,5.1f); /* Actual 7ms elapsed, no replay loop. */
  assert(status().curve_period_ms==5);
  assert(ChassisMotion_Stop(0));complete_stop();sample(tick+1);
  assert(ChassisMotion_MoveTo(1200,3200,90,1000,60000));assert(status().curve_period_ms==1);
  setup();ChassisMotion_TestParametersDefaults(&p);p.curve_period_ms=5;
  assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));
  assert(ChassisMotion_MoveToTuned(1200,3200,90,1000,&p,60000));
  for(unsigned now=101;now<105;now++){sample(now);ChassisMotion_Process();near(sent_vx,0);}
  sample(105);ChassisMotion_Process();float held=sent_vx;assert(held>0);
  sample(106);ChassisMotion_Process();near(sent_vx,held);
  /* Arrival/stop checks run at 1ms even with a long profile period. */
  location.observer.feedback.y_mm=3200;sample(107);ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_STOPPING);
  for(unsigned i=0;i<2;i++){
    setup();ChassisMotion_TestParametersDefaults(&p);p.curve_period_ms=i?101:0;
    assert(!ChassisMotion_MoveToTuned(1200,3200,90,1000,&p,60000));
  }
  (void)zero_dispatched;puts("1ms control / independent linear+cosine profile period passed");return 0;
}
