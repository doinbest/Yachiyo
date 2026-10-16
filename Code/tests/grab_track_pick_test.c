#define main legacy_grab_main
#include "grab_task_test.c"
#undef main
static void pick_start(void)
{
 char *pick[]={"grab","track","pick","3","-8","110"};
 reset();configure(true);setting("x_min","0");setting("x_max","100");
 setting("track_pick_tol_px","100");setting("track_pick_hold_ms","0");
 physical[0]=35;physical[1]=10;move_arrives=false;
 output[0]=0;assert(GrabTask_Command(6,pick));assert(!strstr(output,"ERR"));
}
static void high_window_continuous_descent(void)
{
 pick_start();
 for(unsigned i=0;i<60;i++)step(true,0,101,0);
 assert(!z_position_rpm && !closures);
 for(unsigned i=0;i<10 && status().state!=GRAB_DESCEND;i++)step(true,0,100,-100);
 assert(status().state==GRAB_DESCEND);
 for(unsigned i=0;i<10 && !Axes[1].commanded;i++)step(true,0,100,-100);
 assert(Axes[1].commanded && z_position_pulses==-800);
 unsigned requests=velocity_requests,stop_count=stops;
 for(unsigned i=0;i<10;i++)step(true,0,140,70);
 assert(Axes[1].commanded && velocity_requests>requests && stops==stop_count && !closures);
 /* Z completion closes even while X/wheels are moving, with no low-plane
  * pixel gate and no GRAB_SETTLE/all-axis standstill round. */
 physical[1]=-8;arrival_reply_enabled=true;wheel_moving=true;Axes[0].quiet=false;
 step(true,0,140,70);
 assert(closures==1 && status().state==GRAB_CLOSE && !status().stop_requested);
 assert(TrackCloseStopping);
}
static void hold_only_at_high_plane(void)
{
 pick_start();GrabTask_Stop();run_to(GRAB_IDLE,200,0);
 setting("track_pick_hold_ms","200");
 char *pick[]={"grab","track","pick","3","-8","110"};
 assert(GrabTask_Command(6,pick));
 for(unsigned i=0;i<60;i++)step(true,0,101,0);
 step(true,0,90,90);step(true,0,90,90);
 assert(status().state!=GRAB_DESCEND);
 step(true,0,101,0); /* Out of window resets high-plane hold. */
 for(unsigned i=0;i<3;i++)step(true,0,90,90);
 assert(status().state!=GRAB_DESCEND);
 for(unsigned i=0;i<8 && status().state!=GRAB_DESCEND;i++)step(true,0,90,90);
 assert(status().state==GRAB_DESCEND && !closures);
}
static void interrupted_descent_remains_cancellable(void)
{
 pick_start();run_to(GRAB_DESCEND,200,0);
 for(unsigned i=0;i<10;i++)step(true,0,0,0);
 assert(!closures);camera_valid=false;
 for(unsigned i=0;i<60;i++)step(true,0,0,0);
 assert(status().state==GRAB_TRACK_PAUSE && !closures);
 GrabTask_Stop();run_to(GRAB_IDLE,200,0);
 for(unsigned i=0;i<20;i++)step(true,0,0,0);
 assert(!closures && status().state==GRAB_IDLE);
 pick_start();run_to(GRAB_DESCEND,200,0);linear_state_flags[1]=9;
 for(unsigned i=0;i<80 && status().state!=GRAB_ERROR;i++)step(true,0,0,0);
 assert(!closures && (status().state==GRAB_ERROR || status().state==GRAB_STOPPING));
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

static void next_raw_pick_after_zero_velocity(void)
{
  /* Real Mecanum retains its explicit-stop busy latch after a committed zero
   * batch. A completed raw store must still hand over to the next color. */
  for(unsigned blocked=0;blocked<5;blocked++)
  {
    reset();configure(true);RearReferenceValid=true;
    Status.state=GRAB_COMPLETE;Status.mode="fixed";Status.result="stored";
    Operation=OP_STORE;Scene=SCENE_RAW;Status.slot=1;
    ActiveColor=CAMERA_COLOR_RED;OutsidePose=OutsideReferenceValid=true;
    chassis_bus_busy=true;chassis_velocity_handoff_ready=blocked!=1;
    bus.tx_complete=bus.sent_velocity_valid=true;
    if(blocked==2)bus.sent_rpm[0]=1;
    if(blocked==3)bus.stop_pending=true;
    if(blocked==4)Status.result="returned";
    bool accepted=GrabTask_TrackedStoreStart(CAMERA_COLOR_YELLOW,2,GRAB_TRACK_FEEDBACK);
    assert(accepted==(blocked==0));
    if(!accepted){assert(!strcmp(Status.reason,"bus_unavailable"));continue;}
    assert(Status.state==GRAB_TRACK_OBSERVE && ActiveColor==CAMERA_COLOR_YELLOW && Status.slot==2);
    assert(!stops && !positions); /* No FE chain, added pose move or waiting stage. */
    unsigned before=velocity_requests;
    for(unsigned i=0;i<80 && velocity_requests==before;i++)step(true,0,120,20);
    assert(velocity_requests>before && Status.state==GRAB_TRACK_MATCH);
    assert(!closures); /* Outside the high-plane pickup window. */
  }
}
int main(void)
{
 high_window_continuous_descent();hold_only_at_high_plane();
 interrupted_descent_remains_cancellable();descent_bus_feedback_fairness();
 next_raw_pick_after_zero_velocity();
 puts("grab_track_pick_test: high-window continuous descent, completion-driven closure, cancel/fault coverage");
}
