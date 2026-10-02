/* Independent stop confirmation reuses the fake device boundary, not arrival data. */
#define main timed_regressions_main
#include "chassis_motion_test.c"
#undef main
static ChassisStop_Status_t stop_get(void)
{ ChassisStop_Status_t s;ChassisMotion_StopStatusGet(&s);return s; }
static void stop_sample(uint32_t now)
{
  tick=now; /* No map setup: units, origin and IMU stay unset. */
  for(unsigned i=0;i<4;i++)
  {
    location.wheels[i].position_valid=location.wheels[i].speed_valid=true;
    location.wheels[i].position_ms=location.wheels[i].speed_ms=now;
  }
  ChassisMotion_Process();
}
static void stop_tx(uint32_t now)
{
  tick=now;bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;
  bus.tx_complete=true;bus.sent_ms=now;ChassisMotion_Process();
}
static void begin_stop(void)
{ reset();tick=100;assert(ChassisMotion_StopRequest(123));assert(stop_polling); }
int main(void)
{
  ChassisStop_Status_t s;
  ChassisMotion_Target_t t;
  begin_stop();s=stop_get();assert(s.id==1 && s.token==123 && s.requested_ms==100);
  stop_sample(110);stop_sample(400);stop_sample(650);
  assert(!stop_get().tx_complete && !stop_get().wheels_stopped); /* zero feedback is not TX */
  stop_tx(700);assert(stop_get().tx_complete && !stop_get().wheels_stopped);
  stop_sample(710);assert(!stop_get().wheels_stopped);
  stop_sample(960);assert(!stop_get().wheels_stopped);
  tick=1200;assert(ChassisMotion_StopRequest(123));assert(stops==1 && stop_get().id==1);
  stop_sample(1210);assert(stop_get().wheels_stopped);
  assert(!location.origin_valid && !location.feedback_valid && !status().heading_valid);
  assert(!status().stop_confirmed); /* Physical stop must never claim arrival. */
  tick=1811;assert(stop_get().wheels_stopped && !stop_polling); /* Request result is retained after releasing temporary polling. */

  begin_stop();stop_tx(110);stop_sample(120);
  /* One reused speed/position makes the whole group ineligible. */
  tick=400;for(unsigned i=1;i<4;i++)location.wheels[i].speed_ms=location.wheels[i].position_ms=tick;
  ChassisMotion_Process();tick=620;ChassisMotion_Process();assert(!stop_get().wheels_stopped);
  stop_sample(630);assert(!stop_get().wheels_stopped);stop_sample(880);assert(!stop_get().wheels_stopped);
  stop_sample(1130);assert(stop_get().wheels_stopped);
  location.wheels[2].speed_rpm=2;assert(stop_get().wheels_stopped); /* Historical confirmation; new motion invalidates below. */

  begin_stop();stop_tx(110);stop_sample(120);stop_sample(370);
  location.wheels[0].position_raw=1000;stop_sample(620);assert(!stop_get().wheels_stopped);
  stop_sample(870);stop_sample(1120);assert(!stop_get().wheels_stopped);
  stop_sample(1370);assert(stop_get().wheels_stopped);
  /* Newly accepted direct motion invalidates tokens even before a TX completes. */
  bus.motion_sequence++;assert(!stop_get().wheels_stopped && !strcmp(stop_get().reason,"new_motion"));
  assert(ChassisMotion_StopRequest(123));assert(stop_get().id==2 && stops==2);
  stop_tx(1380);stop_sample(1390);stop_sample(1640);stop_sample(1890);assert(stop_get().wheels_stopped);
  ChassisMotion_TargetDefaults(&t);assert(ChassisMotion_Start(&t));
  assert(!stop_get().wheels_stopped && !strcmp(stop_get().reason,"new_motion"));
  assert(ChassisMotion_StopRequest(123));assert(stop_get().id==3 && stops==3);

  begin_stop();stop_tx(110);stop_sample(120);location.wheels[0].speed_valid=false;
  tick=620;assert(!stop_get().wheels_stopped && !strcmp(stop_get().reason,"missing_feedback"));
  tick=6100;assert(!stop_get().wheels_stopped && !strcmp(stop_get().reason,"missing_feedback"));
  assert(ChassisMotion_StopRequest(123));assert(stop_get().requested_ms==100); /* no retry reset */
  assert(ChassisMotion_StopRequest(0));assert(stop_get().id==2 && stop_get().token==0);

  begin_stop();stop_tx(110);stop_sample(120);stop_sample(370);
  tick=700;location.wheels[0].speed_ms=121;ChassisMotion_Process();
  assert(!stop_get().wheels_stopped); /* 579ms fresh individually, excessive group skew */
  begin_stop();stop_tx(110);location.units_per_rev=65536;
  tick=700;ChassisMotion_Process();assert(!stop_get().wheels_stopped); /* absent flags/zero storage */

  reset();tick=UINT32_MAX-100;assert(ChassisMotion_StopRequest(UINT32_MAX));
  stop_tx(UINT32_MAX-50);stop_sample(UINT32_MAX-40);stop_sample(209);stop_sample(459);
  assert(stop_get().wheels_stopped && stop_get().token==UINT32_MAX);
  begin_stop();bus.stage=MECANUM_STAGE_FAULT;assert(!stop_get().tx_complete && !stop_get().wheels_stopped);
  assert(!strcmp(stop_get().reason,"tx_failed"));
  s=stop_get();tick+=200;
  assert(ChassisMotion_StopRequest(123));
  assert(stop_get().id==s.id && stop_get().requested_ms==s.requested_ms && stops==1);
  assert(!strcmp(stop_get().reason,"tx_failed")); /* Retry cannot rewrite a failed request. */
  begin_stop();
  bus.locked=true;bus.error=MECANUM_ERROR_CANCELLED;
  stop_tx(110);stop_sample(120);stop_sample(370);stop_sample(620);
  assert(stop_get().wheels_stopped && bus.locked);
  ChassisMotion_TargetDefaults(&t);
  assert(!ChassisMotion_Start(&t)); /* Confirmed stationary never clears motion lock. */
  puts("chassis_stop_test: PASS (TX correlation, independent fresh groups, retry tokens, stale/new motion, wrap)");
  return 0;
}
