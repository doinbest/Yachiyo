/* Regression source only this round: user requested no compilation or flashing. */
#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
static void begin(float p,float d)
{
  setup();reading(100,0);assert(ChassisMotion_AnchorSet(CHASSIS_MODEL_PI/2));
  ChassisMotion_TestParameters_t tuning;ChassisMotion_TestParametersDefaults(&tuning);
  tuning.heading_kp=p;tuning.heading_kd=d;
  assert(ChassisMotion_MoveToTuned(1200,4200,90,1000,&tuning,60000));
}
static float observe(uint32_t now,float yaw)
{
  sample(now);reading(now,yaw);ChassisMotion_Process();
  return status().body_target.omega_rad_s;
}
int main(void)
{
  /* Both P and D are quiet inside the inclusive +/-0.1 degree window. */
  begin(.1f,.01f);
  assert(observe(120,.05f)==0);
  assert(observe(140,-.05f)==0);
  assert(observe(160,.1f)==0);
  assert(observe(180,-.1f)==0);
  tick=200;ChassisMotion_Process();assert(status().body_target.omega_rad_s==0);
  /* Beyond the deadband, use the original error without subtracting its width. */
  begin(.1f,0);assert(fabsf(observe(120,.11f)+.011f)<.0001f);
  begin(.1f,0);assert(fabsf(observe(120,-.11f)-.011f)<.0001f);
  begin(.1f,0);assert(observe(120,.1002f)<0);
  begin(.1f,0);assert(observe(120,-.1002f)>0);
  /* Old D history is cleared in the deadband, not replayed on the next exit. */
  begin(.1f,.01f);(void)observe(120,.5f);assert(observe(140,.05f)==0);
  assert(fabsf(observe(160,.2f)+.045f)<.0001f);
  /* Visual precision turns must still command correction below 0.1 degree. */
  setup();reading(100,0);assert(ChassisMotion_AnchorSet(CHASSIS_MODEL_PI/2));
  assert(ChassisMotion_TurnToPrecision(90.08f,10,.01f,60000));
  for(unsigned now=120;now<=640;now+=20)(void)observe(now,0);
  assert(status().turn_only && status().state==CHASSIS_MOTION_RUNNING);
  assert(status().body_target.omega_rad_s>0);
  (void)zero_dispatched;puts("heading deadband and visual precision bypass passed");return 0;
}
