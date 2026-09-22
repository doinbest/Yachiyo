/* Reuse fake bus/IMU boundary; exercise the production distance implementation. */
#define main timed_regressions_main
#include "chassis_motion_test.c"
#undef main
static void sample(uint32_t now)
{
  unsigned i;
  reading(now,30);location.feedback_tick=now;location.feedback_sequence++;
  for(i=0;i<4;i++) {location.wheels[i].speed_valid=true;location.wheels[i].position_valid=true;
    location.wheels[i].speed_ms=location.wheels[i].position_ms=now;}
}
static void setup(void)
{
  reset();valid=true;reading(100,30);assert(ChassisMotion_AnchorSet(CHASSIS_MODEL_PI/2));
  location.origin_valid=location.feedback_valid=location.speed_valid=true;location.units_per_rev=65536;
  location.generation=7;location.origin.x_mm=location.observer.feedback.x_mm=1200;
  location.origin.y_mm=location.observer.feedback.y_mm=1200;
  location.origin.yaw_rad=location.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;sample(100);
}
static void zero_dispatched(uint32_t now)
{
  sample(now);bus.stage=MECANUM_STAGE_SENT;bus.tx_complete=true;bus.sent_velocity_valid=true;
  bus.sent_ms=now;memset(bus.sent_rpm,0,sizeof(bus.sent_rpm));ChassisMotion_Process();
  assert(stops==1);bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;
}
int main(void)
{
  setup();assert(ChassisMotion_MoveTo(1100,1300,90,50,60000));
  assert(fabsf(status().requested.heading_deg-30)<.01f); /* map90 = currentmodule30 */
  sample(120);ChassisMotion_Process();
  assert(fabsf(status().body_target.vx_mm_s-1.414214f)<.001f);
  assert(fabsf(status().body_target.vy_mm_s-1.414214f)<.001f);
  assert(fabsf(status().body_target.omega_rad_s+.04f)<.001f);
  assert(!status().stop_confirmed); /* numerical position never implies physical stop */
  route_busy=true;ChassisMotion_Stop(0);complete_stop();assert(!ChassisMotion_MoveTo(1100,1300,90,50,60000));
  setup();location.feedback_valid=false;assert(!ChassisMotion_MoveTo(1100,1300,90,50,60000));
  setup();assert(ChassisMotion_MoveTo(1100,1300,90,10,190000));
  setup();assert(!ChassisMotion_MoveTo(1100,1300,90,10,190001));
  setup();assert(!ChassisMotion_MoveTo(NAN,1300,90,50,60000));assert(!ChassisMotion_MoveTo(1100,1300,90,101,60000));
  setup();assert(ChassisMotion_MoveTo(1300,1200,90,50,60000));location.generation++;
  sample(120);ChassisMotion_Process();assert(stops==1);complete_stop();assert(status().state==CHASSIS_MOTION_ERROR);
  setup();assert(ChassisMotion_MoveTo(1300,1200,90,50,60000));sample(201);ChassisMotion_Process();assert(stops==1);complete_stop();assert(!strcmp(status().reason,"control_gap"));
  setup();assert(ChassisMotion_MoveTo(1300,1200,90,50,100));
  for(unsigned j=120;j<=200;j+=20){sample(j);ChassisMotion_Process();}complete_stop();assert(!strcmp(status().reason,"distance_timeout"));
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));sample(120);ChassisMotion_Process();
  assert(stops==0&&status().state==CHASSIS_MOTION_STOPPING); /* drain zero batch */
  zero_dispatched(140);ChassisMotion_Process();assert(!status().stop_confirmed);
  sample(200);ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_STOPPING);
  reading(500,30);ChassisMotion_Process();assert(!status().stop_confirmed); /* no new group */
  sample(550);ChassisMotion_Process();assert(!status().stop_confirmed);
  sample(750);ChassisMotion_Process();assert(status().stop_confirmed&&status().state==CHASSIS_MOTION_DONE);
  assert(!strcmp(status().reason,"feedback_arrived"));
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));sample(120);ChassisMotion_Process();zero_dispatched(140);
  for(unsigned j=200;j<=3200;j+=100){sample(j);location.wheels[0].speed_rpm=2;ChassisMotion_Process();}
  assert(status().state==CHASSIS_MOTION_ERROR&&!status().stop_confirmed);assert(!strcmp(status().reason,"stop_unconfirmed"));
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));sample(120);ChassisMotion_Process();zero_dispatched(140);
  sample(200);ChassisMotion_Process();sample(500);location.wheels[0].position_raw=1000;ChassisMotion_Process();
  sample(800);ChassisMotion_Process();assert(!status().stop_confirmed);sample(1000);ChassisMotion_Process();sample(1400);ChassisMotion_Process();assert(status().stop_confirmed);
  setup();tick=0xFFFFFFA0U;sample(tick);assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));sample(0xFFFFFFB4U);ChassisMotion_Process();zero_dispatched(0xFFFFFFC8U);
  sample(20);ChassisMotion_Process();sample(220);ChassisMotion_Process();sample(620);ChassisMotion_Process();assert(status().stop_confirmed);
  /* Closed-loop convergence under an ideal low-speed plant; no forced endpoint snap. */
  setup();assert(ChassisMotion_MoveTo(1100,1300,90,50,60000));
  for(unsigned j=120;j<20000 && ChassisMotion_IsBusy();j+=20)
  {
    ChassisMotion_Status_t m=status();
    location.observer.feedback.x_mm-=m.body_target.vy_mm_s*.02f;
    location.observer.feedback.y_mm+=m.body_target.vx_mm_s*.02f;
    sample(j);ChassisMotion_Process();
    if(status().state==CHASSIS_MOTION_STOPPING && !stops)zero_dispatched(j);
    if(stops){bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;}
  }
  assert(status().stop_confirmed);assert(hypotf(status().error_x_mm,status().error_y_mm)<=10);
  puts("chassis_distance_test: PASS (coordinate conversion, arrival, fresh groups, timeout, loss, wrap, convergence)");
  return 0;
}
