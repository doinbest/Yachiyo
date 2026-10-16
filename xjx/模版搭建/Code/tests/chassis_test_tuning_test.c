#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
static void near(float a,float b){assert(fabsf(a-b)<.0001f);}
static float cosine_speed(unsigned k)
{
  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));
  ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);p.cosine_k=(uint16_t)k;
  assert(ChassisMotion_MoveToTuned(1200,4200,90,500,&p,60000));
  assert(status().cosine_k==k);
  for(unsigned now=120;now<=200;now+=20){sample(now);ChassisMotion_Process();}
  return status().body_target.vx_mm_s;
}
int main(void)
{
  assert(cosine_speed(100)>cosine_speed(400)+100);
  setup();ChassisMotion_TestParameters_t p;ChassisMotion_TestParametersDefaults(&p);
  p.heading_kp=.1f;p.heading_kd=.01f;
  assert(ChassisMotion_MoveToTuned(1200,4200,91,1000,&p,60000));
  sample(120);ChassisMotion_Process();near(sent_omega,.1f);
  sample(140);reading(140,30.2f);ChassisMotion_Process();near(sent_omega,.04666667f);
  near(status().heading_kp,.1f);near(status().heading_kd,.01f);
  p.heading_kp=.8f;assert(!ChassisMotion_MoveToTuned(1200,4200,91,1000,&p,60000));
  near(status().heading_kp,.1f);
  /* A later regular task gets current source defaults, not the test override. */
  assert(ChassisMotion_Stop(0));complete_stop();sample(tick+20);
  assert(ChassisMotion_MoveTo(1200,4200,90,1000,60000));
  near(status().heading_kp,CHASSIS_HEADING_TEST_KP);near(status().heading_kd,CHASSIS_HEADING_TEST_KD);
  assert(status().cosine_k==CHASSIS_DISTANCE_COSINE_K);
  setup();ChassisMotion_TestParametersDefaults(&p);p.heading_kp=1;p.heading_kd=0;
  assert(ChassisMotion_MoveToTuned(1200,4200,91,1000,&p,60000));
  sample(120);ChassisMotion_Process();near(sent_omega,.5f);
  setup();ChassisMotion_TestParametersDefaults(&p);p.heading_kp=p.heading_kd=0;
  assert(ChassisMotion_MoveToTuned(1200,4200,91,1000,&p,60000));
  sample(120);ChassisMotion_Process();near(sent_omega,0);
  const float invalid[]={-1,NAN,INFINITY,1.01f};
  for(unsigned i=0;i<4;i++){
    setup();ChassisMotion_TestParametersDefaults(&p);p.heading_kp=invalid[i];
    assert(!ChassisMotion_MoveToTuned(1200,4200,90,1000,&p,60000));
    ChassisMotion_TestParametersDefaults(&p);p.heading_kd=invalid[i];
    assert(!ChassisMotion_MoveToTuned(1200,4200,90,1000,&p,60000));
  }
  setup();ChassisMotion_TestParametersDefaults(&p);p.cosine_k=0;
  assert(!ChassisMotion_MoveToTuned(1200,4200,90,1000,&p,60000));
  (void)zero_dispatched;puts("per-test cosine K and heading PD passed");return 0;
}
