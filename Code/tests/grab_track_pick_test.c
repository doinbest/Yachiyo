#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void command(unsigned n,char *a[])
{ output[0]=0;assert(GrabTask_Command(n,a));assert(!strstr(output,"ERR")); }

static void configurable_pickup_hold(void)
{
  char *pick[]={"grab","track","pick","3","-8","110"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  setting("track_pick_tol_px","5");setting("track_pick_hold_ms","200");
  physical[0]=35;physical[1]=-8;move_arrives=false;
  command(6,pick);
  for(unsigned i=0;i<60;i++)step(true,0,20,20);
  step(true,0,5,-5);
  for(unsigned i=0;i<3;i++)step(true,0,5,-5);
  assert(status().state==GRAB_DESCEND && !closures); /* Configured 200 ms, not immediate. */
  step(true,0,6,0); /* One out-of-window measurement restarts the hold. */
  step(true,0,5,5);
  for(unsigned i=0;i<3;i++)step(true,0,5,5);
  assert(status().state==GRAB_DESCEND && !closures);
  camera_valid=false;step(true,0,0,0);camera_valid=true;
  step(true,0,5,5);
  for(unsigned i=0;i<4;i++)step(false,0,5,5);
  assert(status().state==GRAB_DESCEND && !closures); /* Cached pixels do not finish hold. */
  step(true,0,5,5);
  assert(status().state==GRAB_SETTLE && !closures);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  setting("track_pick_tol_px","10");setting("track_pick_hold_ms","100");
  command(6,pick);
  for(unsigned i=0;i<60;i++)step(true,0,20,20);
  step(true,0,10,-10);step(true,0,10,-10);
  assert(status().state==GRAB_DESCEND && !closures);
  step(true,0,10,-10);
  assert(status().state==GRAB_SETTLE && !closures);
  for(unsigned i=0;i<80 && !closures;i++)step(true,0,10,-10);
  assert(closures==1 && status().state==GRAB_CLOSE);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  char *bad[]={"grab","set","track_pick_hold_ms","1.5"};
  output[0]=0;assert(GrabTask_Command(4,bad));assert(strstr(output,"ERR"));
  puts("pickup: configured 5px/200ms, configurable window/hold, interrupted hold and fresh samples PASS");
}

/* Catch a descent gated by fine alignment, or a descent which blocks XY tracking.
 * Device replies are fake; the production task and controller remain real. */
static void coarse_descent_and_fine_close(void)
{
  char *pick[]={"grab","track","pick","3","-8","110"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  setting("track_pick_hold_ms","0");setting("track_pick_tol_px","5");
  physical[0]=35;physical[1]=10;move_arrives=false;
  command(6,pick);
  for(unsigned i=0;i<50;i++)step(true,0,51,-50);
  assert(!z_position_rpm && !closures);
  for(unsigned i=0;i<20;i++)step(true,0,-50,51);
  assert(!z_position_rpm && !closures);
  for(unsigned i=0;i<20;i++)step(true,0,50,-50);
  assert(z_position_pulses==-800 && z_position_rpm==206 && z_position_acc==configured_z_acc);
  assert(status().state==GRAB_DESCEND && !status().stop_requested && !closures);
  unsigned requests=velocity_requests;
  for(unsigned i=0;i<20;i++)step(true,0,20,-20);
  assert(velocity_requests>requests && physical[1]==10 && !closures);
  /* Fine pixels alone must not close the gripper while Z has not arrived. */
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(!closures && status().state==GRAB_DESCEND);
  physical[1]=-8;
  for(unsigned i=0;i<30;i++)step(true,0,6,-5);
  assert(!closures && status().state==GRAB_DESCEND);
  step(true,0,5,-5); /* Inclusive fine window: no 200 ms visual dwell. */
  assert(status().state==GRAB_SETTLE && !closures);
  for(unsigned i=0;i<80 && !closures;i++)step(true,0,5,-5);
  assert(closures==1 && status().state==GRAB_CLOSE && status().stop_confirmed);
  move_arrives=true;run_to(GRAB_COMPLETE,150,0);
  assert(closures==1 && fabsf(physical[1]-10)<.01f && !base_moves && !tray_indexes);
  assert(!strcmp(status().result,"picked"));
  puts("pickup: coarse window lowers while tracking; fine window closes after Z/stop feedback PASS");
}

static void descent_pause_recovery_and_stop(void)
{
  char *pick[]={"grab","track","pick","3","-8","110"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=35;physical[1]=10;move_arrives=false;
  command(6,pick);
  for(unsigned i=0;i<60;i++)step(true,0,20,20);
  assert(status().state==GRAB_DESCEND && Axes[1].commanded && !closures);
  unsigned requests=velocity_requests;
  for(unsigned i=0;i<5 && Axes[1].commanded;i++)step(true,0,51,0);
  assert(!Axes[1].commanded && last_stop_axis==MECHANICAL_ARM_AXIS_Z && !closures);
  for(unsigned i=0;i<10;i++)step(true,0,51,0);
  assert(!Axes[1].commanded && velocity_requests>requests && !status().stop_requested);
  for(unsigned i=0;i<10;i++)step(true,0,-50,-50);
  assert(Axes[1].commanded && z_position_pulses==-800 && !closures);
  /* Two lost frames fit the existing grace; a sustained gap stops Z as well. */
  camera_valid=false;
  for(unsigned i=0;i<2;i++)step(true,0,0,0);
  assert(status().state==GRAB_DESCEND && Axes[1].commanded && !closures);
  for(unsigned i=0;i<60;i++)step(true,0,0,0);
  assert(status().state==GRAB_TRACK_PAUSE && status().stop_confirmed);
  assert(last_stop_axis==MECHANICAL_ARM_AXIS_ALL && !closures);
  float anchor=Tracking.reference_left_mm;
  camera_valid=true;
  for(unsigned i=0;i<20;i++)step(true,0,20,20);
  assert(status().state==GRAB_DESCEND && Axes[1].commanded && !closures);
  assert(Tracking.reference_left_mm==anchor);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  assert(last_stop_axis==MECHANICAL_ARM_AXIS_ALL && status().stop_confirmed && !closures);
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(status().state==GRAB_IDLE && !closures && physical[1]==10);
  assert(Config[C_Z_GRAB]==-8 && Config[C_Z_PICK_SPEED]==110 && Tracking.reference_left_mm==anchor);
  puts("pickup: coarse exit pauses Z only; brief loss, reacquisition and ALL cancellation PASS");
}

static void fresh_coordinate_and_z_fault(void)
{
  char *pick[]={"grab","track","pick","3","-8","110"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  setting("track_pick_hold_ms","0");setting("track_pick_tol_px","5");
  physical[0]=35;physical[1]=-8;move_arrives=false;
  command(6,pick);
  for(unsigned i=0;i<60;i++)step(true,0,20,20);
  for(unsigned i=0;i<3;i++)step(false,0,0,0);
  assert(status().state==GRAB_DESCEND && !closures); /* Repeated sample cannot close. */
  step(true,0,5,5);
  assert(status().state==GRAB_SETTLE && !closures);
  /* Fine pixels during stopping must be newer than stop transmission. */
  for(unsigned i=0;i<20;i++)step(false,0,5,5);
  assert(!closures);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  physical[1]=10;
  command(6,pick);
  for(unsigned i=0;i<60;i++)step(true,0,20,20);
  linear_state_flags[1]=9;
  for(unsigned i=0;i<60 && status().state!=GRAB_ERROR;i++)step(true,0,0,0);
  assert((status().state==GRAB_ERROR || status().state==GRAB_STOPPING) && !closures);
  assert(!strcmp(status().reason,"axis_disabled_or_stall"));
  assert(last_stop_axis==MECHANICAL_ARM_AXIS_ALL);
  puts("pickup: fresh fine/parking observation required; Z protection never closes PASS");
}

static void descent_bus_feedback_fairness(void)
{
  const unsigned delays[]={20,25,80};
  for(unsigned trial=0;trial<3;trial++)
  {
    uint32_t completion=0,max_x_age=0,max_z_age=0,max_slow_age=0;
    unsigned writes=0,delay=delays[trial];
    reset();configure(true);setting("x_min","0");setting("x_max","100");
    Status.mode="track";Status.state=GRAB_DESCEND;TrackMoving=TrackPickLowering=true;
    TrackInput.valid=true;TrackInput.error_px[0]=TrackInput.error_px[1]=20;
    Axes[1].commanded=true;
    for(unsigned a=0;a<3;a++)
    { Axes[a].position_valid=Axes[a].state_valid=true;Axes[a].position_tick=Axes[a].state_tick=clock_ms; }
    for(unsigned ms=0;ms<5000;ms++)
    {
      clock_ms++;
      if(ms%20==0)
      {
        TrackControlSequence++;TrackOutput.velocity_mm_s[2]=20+(ms%100)/10.f;
        TrackOutput.x_target_mm=20+(ms%1000)/100.f;
      }
      if(pending_valid && clock_ms>=completion)
      {
        MechanicalArm_EventTypeDef event=pending;pending_valid=false;completion=0;
        if(event.Action==MECHANICAL_ARM_ACTION_STATE)event.StateFlags[event.Axis]=3;
        if(event.Action==MECHANICAL_ARM_ACTION_POSITION)writes++;
        (void)GrabTask_MotorEventHandle(&event);
      }
      track_io(clock_ms,true);
      if(pending_valid && !completion)completion=clock_ms+delay;
      if(clock_ms-Axes[0].position_tick>max_x_age)max_x_age=clock_ms-Axes[0].position_tick;
      for(unsigned a=0;a<3;a++)
      {
        uint32_t age=clock_ms-Axes[a].position_tick;
        if(a==1 && age>max_z_age)max_z_age=age;
        if(age>max_slow_age)max_slow_age=age;
        age=clock_ms-Axes[a].state_tick;if(age>max_slow_age)max_slow_age=age;
      }
    }
    assert(writes>=10 && max_x_age<=4*delay+50 && max_slow_age<2000);
    printf("pickup IO: transaction=%u X max age=%lu Z max age=%lu all max age=%lu writes=%u PASS\n",
      delay,(unsigned long)max_x_age,(unsigned long)max_z_age,(unsigned long)max_slow_age,writes);
  }
}

static void pickup_drift_and_cancel(void)
{
  char *pick[]={"grab","track","pick","3","-8","110"};
  char *begin[]={"grab","track","begin","3","20"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  setting("track_pick_tol_px","5");
  physical[0]=35;physical[1]=10;camera_valid=false;
  command(5,begin);
  for(unsigned i=0;i<50;i++)step(true,0,0,0);
  assert(!Tracking.reference_valid && !velocity_requests && !positions && !closures);
  camera_valid=true;
  for(unsigned i=0;i<50;i++)step(true,0,0,0);
  assert(Tracking.reference_valid && fabsf(TrackOutput.body_room_mm-20)<.1f);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  command(6,pick);run_to(GRAB_SETTLE,200,0);
  for(unsigned i=0;i<50;i++)step(true,0,25,0); /* Target leaves during stop. */
  assert(!closures && status().state!=GRAB_CLOSE && !TrackPickParking);
  camera_valid=false;
  for(unsigned i=0;i<50;i++)step(true,0,25,0);
  assert(!closures && status().state==GRAB_TRACK_PAUSE);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  camera_valid=true;physical[1]=10;move_arrives=false;
  command(6,pick);run_to(GRAB_DESCEND,200,0);
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(!closures);GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  assert(!closures && Tracking.reference_valid);
  float speed=Config[C_Z_PICK_SPEED],height=Config[C_Z_GRAB];
  char *invalid[]={"grab","track","pick","3","-500","110"};
  output[0]=0;assert(GrabTask_Command(6,invalid));assert(strstr(output,"ERR"));
  assert(Config[C_Z_PICK_SPEED]==speed && Config[C_Z_GRAB]==height);
  puts("tracking startup waits for vision, drift reacquires, loss/cancel never closes PASS");
}

int main(void)
{
  configurable_pickup_hold();
  coarse_descent_and_fine_close();
  descent_pause_recovery_and_stop();
  fresh_coordinate_and_z_fault();
  descent_bus_feedback_fairness();
  reset(); configure(true);
  float old_speed=Config[C_Z_PICK_SPEED];
  char *outside_range[]={"grab","track","pick","3","-8","334"};
  output[0]=0; assert(GrabTask_Command(6,outside_range));
  assert(strstr(output,"ERR grab track pickup") && Config[C_Z_PICK_SPEED]==old_speed);
  /* Range acceptance does not bypass conversion to the drive's RPM limit. */
  setting("z_ppm","1000");
  outside_range[5]="200"; output[0]=0; assert(GrabTask_Command(6,outside_range));
  assert(strstr(output,"ERR grab track pickup") && Config[C_Z_PICK_SPEED]==old_speed);
  char *begin[]={"grab","track","begin","3"};
  char *pick[]={"grab","track","pick","3","-8","110"};
  char *stop[]={"grab","track","stop"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=35;physical[1]=10;
  output[0]=0;command(4,begin);
  assert(status().state==GRAB_TRACK_OBSERVE && velocity_requests==0);
  for(unsigned i=0;i<50;i++)step(true,0,0,0);
  assert(Tracking.reference_valid && status().state==GRAB_TRACK_FOLLOW);
  assert(!closures && !openings);
  command(3,stop);run_to(GRAB_IDLE,200,0);
  float anchor=Tracking.reference_left_mm;
  command(4,begin);
  for(unsigned i=0;i<50;i++)step(true,0,0,0);
  assert(Tracking.reference_left_mm==anchor);
  command(3,stop);run_to(GRAB_IDLE,200,0);
  output[0]=0;command(6,pick);
  run_to(GRAB_DESCEND,200,0);
  assert(!status().stop_requested && !closures);
  run_to(GRAB_CLOSE,100,0);
  assert(z_position_rpm==206 && z_position_acc==configured_z_acc);
  assert(closures==1 && fabsf(physical[1]+8)<.01f);
  run_to(GRAB_COMPLETE,150,0);
  assert(fabsf(physical[1]-10)<.01f && !tray_indexes && !base_moves);
  assert(!strcmp(status().result,"picked"));
  assert(Tracking.reference_left_mm==anchor);
  pickup_drift_and_cancel();
  puts("tracking one-click begin, retained budget, fast pickup then lift PASS");
}
