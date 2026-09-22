/* Reuse fake bus/IMU boundary; exercise the production distance implementation. */
#define main timed_regressions_main
#include "chassis_motion_test.c"
#undef main
#include "chassis_config.h"
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
  unsigned previous_stops=stops;
  sample(now);bus.stage=MECANUM_STAGE_SENT;bus.tx_complete=true;bus.sent_velocity_valid=true;
  bus.sent_ms=now;memset(bus.sent_rpm,0,sizeof(bus.sent_rpm));ChassisMotion_Process();
  assert(stops==previous_stops+1);bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;
}
/* Delayed wheel feedback and first-order motor response, no endpoint snapping.
   A long software-only move is needed to actually reach 1 m/s at 100 mm/s^2. */
static void delayed_plant(float length, float speed, unsigned delay_steps, bool disturb)
{
  const unsigned directions[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,
      CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,
      CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  float positions[32]={0}, velocities[32]={0};
  float position=0, velocity=0, peak=0, overshoot=0, reverse=0;
  unsigned step_index=0;bool pushed=false;
  setup();location.observer.feedback.y_mm=-4000;
  assert(ChassisMotion_MoveTo(1200,-4000+length,90,speed,200000));
  for(unsigned now=120;now<190000 && ChassisMotion_IsBusy();now+=20)
  {
    float wanted=status().body_target.vx_mm_s;
    velocity+=(wanted-velocity)*(.02f/.12f);
    position+=velocity*.02f;
    if(disturb && !pushed && bus.stage==MECANUM_STAGE_STOPPED && fabsf(velocity)<1)
    {position+=25;pushed=true;} /* A small physical displacement after stopping. */
    if(velocity>peak)peak=velocity;
    if(-velocity>reverse)reverse=-velocity;
    if(position-length>overshoot)overshoot=position-length;
    positions[step_index%32]=position;velocities[step_index%32]=velocity;
    reading(now,30);
    if(step_index%5==0)
    {
      unsigned old=step_index>=delay_steps?(step_index-delay_steps)%32:0;
      sample(now);location.observer.feedback.y_mm=-4000+positions[old];
      for(unsigned w=0;w<4;w++)
      {
        location.wheels[w].speed_ms=location.wheels[w].position_ms=now-delay_steps*20;
        location.wheels[w].speed_rpm=(directions[w]?-1:1)*(int16_t)lroundf(velocities[old]*60/(CHASSIS_MODEL_PI*80));
        location.wheels[w].position_raw=(directions[w]?-1:1)*(int64_t)(positions[old]*65536/(CHASSIS_MODEL_PI*80));
      }
    }
    if(status().state==CHASSIS_MOTION_STOPPING)
    {
      if(bus.stop_pending || bus.stage==MECANUM_STAGE_STOPPED)
      {bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;}
      else
      {
        bus.stage=MECANUM_STAGE_SENT;bus.tx_complete=true;bus.sent_velocity_valid=true;
        bus.sent_ms=now;memset(bus.sent_rpm,0,sizeof(bus.sent_rpm));
      }
    }
    else bus.stage=MECANUM_STAGE_SENT;
    ChassisMotion_Process();step_index++;
  }
  printf("delayed plant length=%.0f speed=%.0f peak=%.1f overshoot=%.2f reverse=%.2f final=%.2f reason=%s\n",length,speed,peak,overshoot,reverse,position-length,status().reason);
  fflush(stdout);
  assert(status().state==CHASSIS_MOTION_DONE && status().stop_confirmed);
  assert(fabsf(position-length)<=10 && overshoot<=(disturb?30:10) && reverse<=20);
  if(disturb)assert(pushed && stops>=2);
  if(length>10000)assert(peak>=990); /* actual modeled speed, not just command acceptance */
}
int main(void)
{
  delayed_plant(14000,1000,5,false);
  delayed_plant(1750,1000,10,false);
  delayed_plant(900,500,10,false);
  delayed_plant(212,1000,10,false);
  delayed_plant(212,1000,10,true);

  setup();assert(ChassisMotion_MoveTo(1100,1300,90,50,60000));
  assert(fabsf(status().requested.heading_deg-30)<.01f); /* map90 = currentmodule30 */
  sample(120);ChassisMotion_Process();
  assert(fabsf(status().body_target.vx_mm_s-1.414214f)<.001f);
  assert(fabsf(status().body_target.vy_mm_s-1.414214f)<.001f);
  assert(fabsf(status().body_target.omega_rad_s+.04f)<.001f);
  assert(!status().stop_confirmed); /* numerical position never implies physical stop */
  route_busy=true;ChassisMotion_Stop(0);complete_stop();assert(!ChassisMotion_MoveTo(1100,1300,90,50,60000));
  setup();location.feedback_valid=false;assert(!ChassisMotion_MoveTo(1100,1300,90,50,60000));
  setup();assert(ChassisMotion_MoveTo(1100,1300,90,10,400000));
  setup();assert(!ChassisMotion_MoveTo(1100,1300,90,10,400001));
  setup();assert(!ChassisMotion_MoveTo(NAN,1300,90,50,60000));assert(!ChassisMotion_MoveTo(1100,1300,90,1001,60000));
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
  /* Stopped outside the target is a distinct outcome, not a stop timeout. */
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));sample(120);ChassisMotion_Process();zero_dispatched(140);
  location.observer.feedback.x_mm=1215;
  sample(200);ChassisMotion_Process();sample(500);ChassisMotion_Process();sample(750);ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_RUNNING && !strcmp(status().reason,"position_correcting"));
  assert(!status().stop_confirmed); /* stop evidence cannot survive resumed movement */
  sample(770);ChassisMotion_Process();assert(hypotf(status().body_target.vx_mm_s,status().body_target.vy_mm_s)<=20);
  ChassisMotion_Stop(0);complete_stop();assert(!ChassisMotion_IsBusy());
  /* Repeated creep failures stop after two attempts, keeping the original ID. */
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));
  unsigned action=status().action_id;
  for(unsigned attempt=0;attempt<3;attempt++)
  {
    unsigned now=120+attempt*650;
    location.observer.feedback.x_mm=1200;
    sample(now);ChassisMotion_Process();zero_dispatched(now+20);
    location.observer.feedback.x_mm=1215;
    sample(now+80);ChassisMotion_Process();sample(now+380);ChassisMotion_Process();sample(now+630);ChassisMotion_Process();
    assert(status().action_id==action);
    if(attempt<2)assert(status().state==CHASSIS_MOTION_RUNNING);
  }
  assert(status().state==CHASSIS_MOTION_ERROR && status().stop_confirmed);
  assert(!strcmp(status().reason,"position_not_reached"));
  /* Large residual after a confirmed stop must not launch an unbounded correction. */
  setup();assert(ChassisMotion_MoveTo(1200,1200,90,50,60000));sample(120);ChassisMotion_Process();zero_dispatched(140);
  location.observer.feedback.x_mm=1400;
  sample(200);ChassisMotion_Process();sample(500);ChassisMotion_Process();sample(750);ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_ERROR && status().stop_confirmed);
  assert(!strcmp(status().reason,"position_not_reached"));
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
