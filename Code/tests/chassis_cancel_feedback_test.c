#define main existing_motion_main
#include "chassis_motion_test.c"
#undef main

static void ready(void)
{
  reset();valid=true;reading(1000,0);ChassisMotion_HeadingProcess();
  assert(ChassisMotion_AnchorSet(0));
  location.origin_valid=location.feedback_valid=location.speed_valid=true;
  location.units_per_rev=65536;location.feedback_tick=tick;
  for(unsigned i=0;i<4;i++){
    location.wheels[i].speed_valid=location.wheels[i].position_valid=location.wheels[i].state_valid=true;
    location.wheels[i].speed_ms=location.wheels[i].position_ms=location.wheels[i].state_ms=tick;
    location.wheels[i].state_flags=3;
  }
  assert(ChassisMotion_MoveTo(500,0,0,200,10000));
}
static void stopped_tx(uint32_t now)
{
  reading(now,0);bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;
  bus.tx_complete=true;bus.sent_ms=now;
}
int main(void)
{
  /* Recovered transport can confirm real parking before localization has a
   * complete aligned group. Cancellation is not a position-arrival claim. */
  ready();assert(ChassisMotion_StopRequest(0));
  location.feedback_valid=location.speed_valid=false;
  stopped_tx(1010);ChassisMotion_Process();
  assert(status().state!=CHASSIS_MOTION_ERROR);
  assert(!status().stop_confirmed && !strcmp(status().reason,"cancelled"));
  for(unsigned n=0;n<3;n++){
    reading(1020+20*n,0);
    for(unsigned i=0;i<4;i++)location.wheels[i].speed_ms=location.wheels[i].position_ms=tick;
    ChassisMotion_Process();
  }
  ChassisStop_Status_t stop;ChassisMotion_StopStatusGet(&stop);
  assert(stop.tx_complete && stop.wheels_stopped && !sends);
  assert(!status().stop_confirmed); /* Never relabel the cancelled target as arrived. */

  ready();location.feedback_valid=false;reading(1010,0);ChassisMotion_Process();
  stopped_tx(1020);ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_ERROR && !strcmp(status().reason,"feedback_invalid"));

  ready();assert(ChassisMotion_StopRequest(0));bus.stage=MECANUM_STAGE_FAULT;
  reading(1010,0);ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_ERROR); /* Real bus/stop failure still blocks. */
  puts("cancelled position task: stop proof survives localization gap; active/fault checks preserved PASS");
  return 0;
}
