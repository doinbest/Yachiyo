/* Regressions for route acceleration, not timed/open-loop arrival. */
#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
#include <stdlib.h>
/* Report expected red-stage failures without a Windows CRT dialog. */
#undef assert
#define assert(x) do { if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);} } while(0)

static float speed(void)
{
  ChassisMotion_Status_t s=status();
  return hypotf(s.body_target.vx_mm_s,s.body_target.vy_mm_s);
}
static float brake_trigger_with_age(unsigned age)
{
  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));
  assert(ChassisMotion_MoveTo(1200,6000,90,1000,60000));
  for(unsigned now=120;now<=500;now+=20){sample(now);ChassisMotion_Process();}
  location.observer.feedback.y_mm=5500;
  for(unsigned now=520;now<=800;now+=20){
    sample(now);location.feedback_tick=now-age;
    for(unsigned i=0;i<4;i++)location.wheels[i].position_ms=location.wheels[i].speed_ms=now-age;
    ChassisMotion_Process();
  }
  return speed();
}
int main(void)
{
  (void)zero_dispatched;
  /* Default planner is the original 300 mm/s² vector ramp. */
  setup();assert(ChassisMotion_ProfileGet()==CHASSIS_PROFILE_LINEAR);
  assert(!ChassisMotion_ProfileSet((ChassisMotion_Profile_t)2));
  assert(ChassisMotion_MoveTo(1200,6000,90,1000,60000));
  sample(120);ChassisMotion_Process();assert(fabsf(speed()-6)<.01f);
  assert(!ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));
  sample(200);ChassisMotion_Process();assert(fabsf(speed()-30)<.01f);
  assert(status().profile==CHASSIS_PROFILE_LINEAR && status().peak_mm_s==0);
  /* Independent quarter/half/full points of a 400 ms half-cosine ramp.
   * A long remaining distance keeps braking out of this observation. */
  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));assert(ChassisMotion_MoveTo(1200,6000,90,1000,60000));
  for(unsigned now=120;now<=500;now+=20){
    sample(now);ChassisMotion_Process();
    if(now==200)assert(fabsf(speed()-146.4466f)<.05f);
    if(now==300)assert(fabsf(speed()-500)<.05f);
    if(now==500)assert(fabsf(speed()-1000)<.05f);
  }
  assert(status().state==CHASSIS_MOTION_RUNNING); /* elapsed ramp never means arrived */

  /* A 100 mm leg must not accelerate to the requested 1 m/s. */
  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));assert(ChassisMotion_MoveTo(1200,1300,90,1000,60000));
  for(unsigned now=120;now<=500;now+=20){sample(now);ChassisMotion_Process();assert(speed()<250);}
  assert(speed()>50);

  /* Deceleration must be distance-aware, and react to old feedback too. */
  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));assert(ChassisMotion_MoveTo(1200,6000,90,1000,60000));
  for(unsigned now=120;now<=500;now+=20){sample(now);ChassisMotion_Process();}
  location.observer.feedback.y_mm=5900;sample(520);ChassisMotion_Process();
  /* A new position starts continuous braking, not a one-tick speed collapse. */
  float fresh=speed();assert(fresh<1000 && fresh>900);
  sample(540);
  for(unsigned i=0;i<4;i++)location.wheels[i].position_ms=location.wheels[i].speed_ms=340;
  ChassisMotion_Process();assert(speed()<fresh);
  /* Observer group age matters even when individual wheel caches are newer. */
  sample(560);location.feedback_tick=360;
  ChassisMotion_Process();assert(speed()<fresh);

  /* A new action starts its own ramp, not the preceding cruise speed. */
  ChassisMotion_Stop(0);complete_stop();
  sample(580);assert(ChassisMotion_MoveTo(1200,9000,90,1000,60000));
  sample(600);ChassisMotion_Process();assert(speed()<10);

  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));sample(0xFFFFFFF0U);
  assert(ChassisMotion_MoveTo(1200,6000,90,1000,60000));
  for(unsigned elapsed=20;elapsed<=100;elapsed+=20){
    sample(0xFFFFFFF0U+elapsed);ChassisMotion_Process();
  }
  assert(fabsf(speed()-146.4466f)<.05f);
  /* Old feedback must constrain convergence; one fresh packet cannot release it. */
  float fresh_limit=brake_trigger_with_age(0),old_limit=brake_trigger_with_age(200);
  assert(fresh_limit>500 && old_limit<fresh_limit-100);
  sample(820);ChassisMotion_Process();assert(speed()<=old_limit+.01f);
  puts("chassis_cosine_distance_test: PASS");
}
