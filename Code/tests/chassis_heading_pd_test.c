/* Exercise the real controller through the fake IMU/bus boundary. */
#define main legacy_motion_regressions_main
#include "chassis_motion_test.c"
#undef main

static void near(float actual, float expected)
{
  if(fabsf(actual-expected)>=0.00002f)fprintf(stderr,"actual=%f expected=%f\n",actual,expected);
  assert(fabsf(actual-expected)<0.00002f);
}

static void begin_heading(float yaw, float target, uint32_t now)
{
  ChassisMotion_Target_t t;
  reset();valid=true;reading(now,yaw);
  assert(ChassisMotion_AnchorSet(0));
  ChassisMotion_TargetDefaults(&t);
  t.mode=CHASSIS_MOTION_HEADING;t.heading_deg=target;
  t.vx_mm_s=1000;t.transition_ms=0;t.hold_ms=5000;
  assert(ChassisMotion_Start(&t));
}

int main(void)
{
  /* Restored pure P: one degree produces .02 rad/s, without D damping. */
  begin_heading(0,1,100);
  reading(120,0);ChassisMotion_Process();near(sent_omega,.02f);
  near(sent_vx,1000);

  /* Measured yaw rate does not contribute with D disabled. */
  reading(140,.2f);ChassisMotion_Process();
  near(sent_omega,.016f);
  float held=sent_omega;
  tick=160;ChassisMotion_Process();near(sent_omega,held);
  /* New sample interval is 60ms; repeated control ticks did not filter it. */
  reading(200,.2f);ChassisMotion_Process();
  near(sent_omega,.016f);

  /* Use sample timestamps, even if foreground consumption is delayed. */
  begin_heading(0,1,100);reading(120,0);ChassisMotion_Process();
  reading(140,.2f);tick=160;ChassisMotion_Process();
  near(sent_omega,.016f);

  /* Only angle error changes correction; output still clips at .5. */
  begin_heading(0,1,100);reading(120,-.2f);ChassisMotion_Process();
  near(sent_omega,.024f);
  begin_heading(0,10,100);reading(120,0);ChassisMotion_Process();near(sent_omega,.2f);
  begin_heading(0,30,100);reading(120,0);ChassisMotion_Process();near(sent_omega,.5f);
  begin_heading(0,-30,100);reading(120,0);ChassisMotion_Process();near(sent_omega,-.5f);

  /* Wrapped yaw and millisecond counter wrap do not create false derivatives. */
  begin_heading(179,179,100);reading(200,-179);ChassisMotion_Process();
  near(sent_omega,-.04f);
  begin_heading(0,1,0xfffffff0U);reading(4,.2f);ChassisMotion_Process();
  near(sent_omega,.016f);

  /* Cancelling and starting another task must not retain the old D state. */
  assert(ChassisMotion_Stop(0));complete_stop();reading(24,.2f);
  ChassisMotion_Target_t t;ChassisMotion_TargetDefaults(&t);
  t.mode=CHASSIS_MOTION_HEADING;t.heading_deg=1.2f;t.transition_ms=0;
  assert(ChassisMotion_Start(&t));reading(44,.2f);ChassisMotion_Process();near(sent_omega,.02f);

  /* A zero-time new sample cannot divide by zero or overwrite the held output. */
  held=sent_omega;angle.update_count++;tick=64;
  ChassisMotion_Process();near(sent_omega,held);

  /* Pure turns retain the original .02 P and their independent speed ceiling. */
  reset();valid=true;reading(100,0);assert(ChassisMotion_AnchorSet(0));
  location.origin_valid=location.feedback_valid=location.speed_valid=true;
  location.units_per_rev=65536;
  for(unsigned i=0;i<4;i++) {
    location.wheels[i].position_valid=location.wheels[i].speed_valid=true;
    location.wheels[i].position_ms=location.wheels[i].speed_ms=100;
  }
  assert(ChassisMotion_TurnToSpeed(5,100,10000));
  for(unsigned now=120;now<=600;now+=20) {
    reading(now,0);location.feedback_tick=now;location.feedback_sequence++;
    for(unsigned i=0;i<4;i++)location.wheels[i].position_ms=location.wheels[i].speed_ms=now;
    ChassisMotion_Process();
  }
  assert(status().state==CHASSIS_MOTION_RUNNING);near(sent_omega,.1f);
  puts("heading pure P .02: zero D contribution, cached samples, wrap, reset, .5 ceiling and independent turns PASS");
  return 0;
}
