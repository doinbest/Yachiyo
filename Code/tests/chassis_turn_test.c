/* Production closed-loop yaw with integer RPM batches and delayed wheel feedback. */
#define CHASSIS_DISTANCE_TEST_HELPERS_ONLY
#include "chassis_distance_test.c"
#undef CHASSIS_DISTANCE_TEST_HELPERS_ONLY

static float deg_error(float a,float b)
{
  return fmodf(a-b+540,360)-180;
}
static void turn_plant(float initial,float target,float limit,float tolerance)
{
  const unsigned dirs[4]={0,0,1,1};
  const ChassisModel_Geometry_t geometry={210,240,80,1};
  float rpm[4]={0},wanted[4]={0},positions[4]={0};
  float old_rpm[32][4]={{0}},old_positions[32][4]={{0}};
  float yaw=initial,max_omega=0;
  unsigned steps=0,batches=0;
  setup();
  assert(ChassisMotion_AnchorSet(initial*CHASSIS_MODEL_PI/180));
  assert(tolerance==2?ChassisMotion_TurnToSpeed(target,limit,60000):ChassisMotion_TurnToPrecision(target,limit,tolerance,60000));
  unsigned id=status().action_id;
  for(unsigned now=120;now<59000 && ChassisMotion_IsBusy();now+=20)
  {
    ChassisModel_Velocity_t velocity;
    if(steps%5==0)
    {
      for(unsigned w=0;w<4;w++)wanted[w]=status().rpm_command[w];
      batches++;
    }
    for(unsigned w=0;w<4;w++)
    {
      rpm[w]+=(wanted[w]-rpm[w])*(.02f/.12f);
      positions[w]+=rpm[w]*65536/60*.02f;
      old_rpm[steps%32][w]=rpm[w];old_positions[steps%32][w]=positions[w];
    }
    assert(ChassisModel_Forward(&geometry,rpm,&velocity));
    yaw+=velocity.omega_rad_s*.02f*180/CHASSIS_MODEL_PI;
    if(steps%15==0)
    {
      unsigned old=steps>=5?(steps-5)%32:0;
      sample(now);
      /* Physical centre drift must stay visible; a pure turn must not fix XY. */
      location.observer.feedback.x_mm=1215;location.observer.feedback.y_mm=1180;
      for(unsigned w=0;w<4;w++)
      {
        location.wheels[w].speed_ms=location.wheels[w].position_ms=now-100;
        location.wheels[w].speed_rpm=(dirs[w]?-1:1)*(int16_t)lroundf(old_rpm[old][w]);
        location.wheels[w].position_raw=(dirs[w]?-1:1)*(int64_t)old_positions[old][w];
      }
    }
    reading(now,30+yaw-initial);
    if(status().state==CHASSIS_MOTION_STOPPING)
    {
      if(bus.stop_pending || bus.stage==MECANUM_STAGE_STOPPED)
      {if(bus.stop_pending)bus.sent_ms=now;bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;bus.tx_complete=true;}
      else
      {bus.stage=MECANUM_STAGE_SENT;bus.tx_complete=true;bus.sent_velocity_valid=true;
       bus.sent_ms=now;memset(bus.sent_rpm,0,sizeof(bus.sent_rpm));}
    }
    else bus.stage=MECANUM_STAGE_SENT;
    ChassisMotion_Process();
    assert(status().body_target.vx_mm_s==0 && status().body_target.vy_mm_s==0);
    max_omega=fmaxf(max_omega,fabsf(status().body_target.omega_rad_s));
    assert(status().action_id==id && location.generation==7);
    steps++;
  }
  printf("turn plant %.0f->%.0f time=%.2fs batches=%u error=%.2f reason=%s\n",
      initial,target,steps*.02f,batches,deg_error(target,yaw),status().reason);
  fflush(stdout);
  assert(status().state==CHASSIS_MOTION_DONE && status().stop_confirmed);
  assert(fabsf(deg_error(target,yaw))<=tolerance && max_omega<=limit*CHASSIS_MODEL_PI/180+0.00001f);
  assert(location.observer.feedback.x_mm==1215 && location.observer.feedback.y_mm==1180);
  assert(status().map_anchor_valid);
  if(fabsf(deg_error(target,initial))>=90) {assert(steps*.02f<6);assert(max_omega>fminf(.8f,limit*CHASSIS_MODEL_PI/180*.8f));}
}
int main(void)
{
  turn_plant(.5f,0,10,.1f);turn_plant(-.5f,0,10,.1f);
  /* Formal ring correction uses the proven inverse relative angle with a
   * separate10deg/s cap; small/large errors share measured-yaw P control. */
  turn_plant(.45f,0,10,.1f);turn_plant(-.45f,0,10,.1f);
  turn_plant(10.92f,0,10,.1f);turn_plant(-10.92f,0,10,.1f);
  turn_plant(0,90,30,2);
  turn_plant(0,-90,50,2);
  turn_plant(90,180,100,2);turn_plant(180,90,100,2);turn_plant(170,-170,100,2);turn_plant(0,180,100,2);turn_plant(90,95,100,2);
  setup();grab_busy=true;assert(!ChassisMotion_TurnTo(180,60000));
  assert(!strcmp(status().reason,"grab_task_busy"));
  setup();assert(!ChassisMotion_TurnTo(NAN,60000));
  assert(!ChassisMotion_TurnTo(181,60000));assert(!ChassisMotion_TurnTo(90,400001));
  setup();assert(ChassisMotion_TurnTo(180,60000));sample(120);ChassisMotion_Process();
  assert(status().body_target.omega_rad_s>0 && status().body_target.omega_rad_s<.01f); /* 500ms startup ramp */
  assert(ChassisMotion_Stop(0));complete_stop();assert(!strcmp(status().reason,"cancelled"));
  setup();assert(ChassisMotion_TurnTo(90,60000));sample(120);ChassisMotion_Process();zero_dispatched(140);
  sample(200);ChassisMotion_Process();reading(500,30);ChassisMotion_Process();
  assert(!status().stop_confirmed);sample(550);ChassisMotion_Process();sample(750);ChassisMotion_Process();
  assert(status().stop_confirmed && status().state==CHASSIS_MOTION_DONE);
  /* More than two post-stop yaw disturbances may recover before original deadline. */
  setup();assert(ChassisMotion_TurnTo(90,10000));unsigned id=status().action_id;
  for(unsigned attempt=0;attempt<4;attempt++)
  {
    unsigned now=120+attempt*700;
    sample(now);ChassisMotion_Process();zero_dispatched(now+20);
    sample(now+80);ChassisMotion_Process();sample(now+380);ChassisMotion_Process();
    sample(now+630);reading(now+630,33);ChassisMotion_Process();
    assert(status().state==CHASSIS_MOTION_RUNNING && status().action_id==id);
    assert(!status().stop_confirmed && status().start_tick==100);
  }
  ChassisMotion_Stop(0);complete_stop();
  setup();assert(ChassisMotion_TurnTo(180,100));
  for(unsigned now=120;now<=200;now+=20){sample(now);ChassisMotion_Process();}
  complete_stop();assert(status().state==CHASSIS_MOTION_ERROR);
  assert(!strcmp(status().reason,"turn_timeout"));
  puts("chassis_turn_test: PASS (quantized delayed plant, pure yaw, fresh stop, wrap, recovery, cancel)");
  return 0;
}
