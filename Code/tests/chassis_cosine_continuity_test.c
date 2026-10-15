/* Catch a premature zero/restart during a normal, delayed-feedback approach. */
#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
#include <stdlib.h>
#undef assert
#define assert(x) do {if(!(x)){fprintf(stderr,"line %d: %s\n",__LINE__,#x);exit(1);}} while(0)

static void approach(float length,float requested,unsigned delay_steps,float response,float acceleration)
{
  const unsigned dirs[4]={CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR,CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR,CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
  float positions[64]={0},velocities[64]={0};
  float position=0,velocity=0,peak=0,overshoot=0,minimum=1e9f,rebound=0;
  unsigned step=0,early_zero=0;bool braking=false;
  setup();assert(ChassisMotion_ProfileSet(CHASSIS_PROFILE_COSINE));
  assert(ChassisMotion_MoveTo(1200,1200+length,90,requested,60000));
  for(unsigned now=120;now<59000 && ChassisMotion_IsBusy();now+=20){
    ChassisMotion_Status_t s=status();
    float dv=(s.body_target.vx_mm_s-velocity)*(.02f/response);
    velocity+=fmaxf(-acceleration*.02f,fminf(acceleration*.02f,dv));
    position+=velocity*.02f;peak=fmaxf(peak,velocity);overshoot=fmaxf(overshoot,position-length);
    positions[step%64]=position;velocities[step%64]=velocity;
    reading(now,30);
    if(step%5==0){
      unsigned old=step>=delay_steps?(step-delay_steps)%64:0;
      sample(now);location.feedback_tick=now-delay_steps*20;
      location.observer.feedback.y_mm=1200+positions[old];
      for(unsigned w=0;w<4;w++){
        location.wheels[w].speed_ms=location.wheels[w].position_ms=now-delay_steps*20;
        location.wheels[w].speed_rpm=(dirs[w]?-1:1)*(int16_t)lroundf(velocities[old]*60/(CHASSIS_MODEL_PI*80));
        location.wheels[w].position_raw=(dirs[w]?-1:1)*(int64_t)(positions[old]*65536/(CHASSIS_MODEL_PI*80));
      }
    }
    if(s.state==CHASSIS_MOTION_STOPPING){
      if(bus.stop_pending || bus.stage==MECANUM_STAGE_STOPPED){
        if(bus.stop_pending)bus.sent_ms=now;
        bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;bus.tx_complete=true;
      }else{
        bus.stage=MECANUM_STAGE_SENT;bus.tx_complete=true;bus.sent_velocity_valid=true;
        bus.sent_ms=now;memset(bus.sent_rpm,0,sizeof(bus.sent_rpm));
      }
    }else bus.stage=MECANUM_STAGE_SENT;
    ChassisMotion_Process();step++;s=status();
    float command=fabsf(s.body_target.vx_mm_s);
    if(peak>requested*.8f && command<requested*.6f)braking=true;
    if(braking){minimum=fminf(minimum,command);rebound=fmaxf(rebound,command-minimum);}
    if(peak>requested*.8f && length-position>25 && s.state==CHASSIS_MOTION_RUNNING &&
       fabsf(s.body_target.vx_mm_s)<.5f)early_zero++;
  }
  printf("cosine continuity D=%.0f V=%.0f delay=%ums early_zero=%u rebound=%.2f final=%.2f overshoot=%.2f state=%u\n",
    length,requested,delay_steps*20,early_zero,rebound,position-length,overshoot,status().state);fflush(stdout);
  assert(early_zero==0);
  assert(rebound<=5);
  assert(status().state==CHASSIS_MOTION_DONE && status().stop_confirmed);
  assert(fabsf(position-length)<=10 && overshoot<=10);
}
int main(void)
{
  (void)zero_dispatched;
  approach(1000,300,10,.12f,100000);approach(1000,500,10,.12f,100000);
  approach(500,300,5,.12f,100000);approach(1500,1000,10,.12f,100000);
  approach(1000,500,10,.20f,600);
  puts("chassis_cosine_continuity_test: PASS");return 0;
}
