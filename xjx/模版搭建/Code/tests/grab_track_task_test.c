#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void command(unsigned n,char *a[])
{ assert(GrabTask_Command(n,a)); assert(!strstr(output,"ERR")); }

static void delayed_self_motion(void)
{
  reset(); configure(true); setting("x_max","100");
  wheel_feedback_delay=80; TrackXZero=0;
  for(unsigned i=0;i<20;i++)
  {
    clock_ms=1000+i*20;
    float mm=30*((float)clock_ms-1080)/1000;
    int64_t raw=(int64_t)(mm*65536/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO);
    wheel_position[0]=wheel_position[1]=raw; wheel_position[2]=wheel_position[3]=-raw;
    Axes[0].position_valid=true; Axes[0].position_tick=clock_ms-120;
    Axes[0].mm=20*(clock_ms-120)/1000.f; Axes[0].signed_speed=20;
    track_observer(clock_ms);
  }
  assert(fabsf(TrackInput.actual_mm_s[0]-30)<.1f);
  assert(fabsf(TrackInput.motion_mm[2]-20*clock_ms/1000.f)<.01f);
  puts("tracking observer: delayed wheel position + X timestamp self-motion PASS");
}

/* UART5 returns one wheel position between 100 ms motion batches. A valid
 * 300 ms stagger must not erase camera self-motion or restart its origin. */
static void staggered_self_motion(void)
{
  reset(); configure(true);
  for(unsigned w=0;w<4;w++)wheel_position_time[w]=1000;
  for(unsigned ms=0;ms<=4000;ms+=20)
  {
    clock_ms=1000+ms;
    if(ms && ms%100==0)
    {
      unsigned w=(ms/100-1)%4;
      float mm=.020f*ms;
      int64_t raw=(int64_t)llroundf(mm*65536/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO);
      wheel_position[w]=w<2?raw:-raw; wheel_position_time[w]=clock_ms;
    }
    track_observer(clock_ms);
  }
  fprintf(stderr,"staggered camera motion: q=%.3f velocity=%.3f valid=%d\n",
    TrackInput.motion_mm[0],TrackInput.actual_mm_s[0],TrackObserver.feedback_valid);
  assert(TrackObserver.feedback_valid && TrackEstimateObserver.feedback_valid);
  assert(TrackObserver.feedback_ms<clock_ms);
  assert(fabsf(TrackObserver.feedback.x_mm-.020f*(TrackObserver.feedback_ms-1000))<.2f);
  assert(fabsf(TrackInput.body_left_mm-TrackObserver.feedback.y_mm)<.001f);
  assert(fabsf(TrackInput.motion_mm[0]-80)<2);
  assert(fabsf(TrackInput.motion_mm[1])<.2f);
  assert(fabsf(TrackInput.actual_mm_s[0]-20)<.2f);
  /* Stop at the same pose: stale reads must not keep inventing travel, and
   * a truly stale wheel must still invalidate the motion estimate. */
  /* The measured velocity needs two wholly post-stop common endpoints;
   * four staggered 100 ms reads take up to three groups to supply them. */
  for(unsigned ms=20;ms<=1200;ms+=20)
  {
    clock_ms=5000+ms;
    if(ms%100==0)
    {
      unsigned w=(ms/100-1)%4;
      int64_t raw=(int64_t)llroundf(80*65536/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO);
      wheel_position[w]=w<2?raw:-raw; wheel_position_time[w]=clock_ms;
    }
    track_observer(clock_ms);
  }
  assert(fabsf(TrackInput.motion_mm[0]-80)<.1f);
  assert(fabsf(TrackInput.actual_mm_s[0])<.1f);
  wheel_stale=true;clock_ms+=20;track_observer(clock_ms);
  assert(!TrackObserver.feedback_valid);
}

/* A 400 ms encoder average is not the current speed after a reversal. The
 * camera compensation must follow actual displacement without read-time
 * steps. Sent commands below lag the request; the plant itself lags 80 ms. */
static void reversing_staggered_motion(void)
{
  float actual_mm=0,max_error=0,max_step=0,previous=0;
  reset(); configure(true);
  for(unsigned w=0;w<4;w++)wheel_position_time[w]=1000;
  bus.sent_velocity_valid=true; bus.stage=MECANUM_STAGE_SENT;
  for(unsigned ms=0;ms<=6000;ms+=20)
  {
    clock_ms=1000+ms;
    float sent=(ms/800)%2?-20:20;
    float actual=ms<80?0:(((ms-80)/800)%2?-20:20);
    if(ms)actual_mm+=actual*.02f;
    if(ms%100==0)
    {
      bus.sent_ms=clock_ms;
      float rpm=sent*60/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO;
      for(unsigned w=0;w<4;w++)bus.sent_rpm[w]=(int16_t)lroundf(rpm);
      if(ms)
      {
        unsigned w=(ms/100-1)%4;
        int64_t raw=(int64_t)llroundf(actual_mm*65536/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO);
        wheel_position[w]=w<2?raw:-raw; wheel_position_time[w]=clock_ms;
      }
    }
    track_observer(clock_ms);
    if(ms>=1000)
    {
      max_error=fmaxf(max_error,fabsf(TrackInput.motion_mm[0]-actual_mm));
      max_step=fmaxf(max_step,fabsf(TrackInput.motion_mm[0]-previous));
    }
    previous=TrackInput.motion_mm[0];
  }
  fprintf(stderr,"reversal camera compensation: peak error=%.3f step=%.3f mm\n",max_error,max_step);
  assert(max_error<3.5f && max_step<1.0f);
}

/* Stationary material, quantized 30 Hz pixels delayed by 50 ms, 100 ms
 * synchronous commands, 80 ms motor response, staggered physical encoders. */
static void stationary_closed_loop(bool pi_filter,float body_speed,float initial_error)
{
  float truth[1001]={0},sent[1001]={0},sum=0,peak=0; unsigned count=0;
  float history[3]={initial_error,initial_error,initial_error},filtered=initial_error;
  unsigned frames=0;
  GrabTracking_t controller; GrabTracking_Config_t config;
  GrabTracking_Input_t in={0}; GrabTracking_Output_t out={0};
  reset(); configure(true); GrabTracking_Init(&controller); GrabTracking_Defaults(&config);
  config.body_speed_mm_s=body_speed;
  GrabTracking_Reference(&controller,0);GrabTracking_Begin(&controller);
  in.valid=in.actuating=true;in.x_mm=65;in.x_min_mm=0;in.x_max_mm=100;
  in.j[0][0]=2;in.j[1][1]=in.j[1][2]=2;
  for(unsigned w=0;w<4;w++)wheel_position_time[w]=1000;
  bus.sent_velocity_valid=true;bus.stage=MECANUM_STAGE_SENT;
  for(unsigned ms=0;ms<=10000;ms+=10)
  {
    unsigned n=ms/10; clock_ms=1000+ms;
    sent[n]=n?sent[n-1]:0;
    if(ms%100==0)
    {
      float rpm=out.velocity_mm_s[0]*60/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO;
      for(unsigned w=0;w<4;w++)bus.sent_rpm[w]=(int16_t)lroundf(rpm);
      sent[n]=bus.sent_rpm[0]*(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)/(60*CHASSIS_MOTOR_TO_WHEEL_RATIO);
      bus.sent_ms=clock_ms;
    }
    if(n)truth[n]=truth[n-1]+(n>=8?sent[n-8]:0)*.01f;
    if(ms && ms%100==0)
    {
      unsigned w=(ms/100-1)%4;
      int64_t raw=(int64_t)llroundf(truth[n]*65536/(CHASSIS_MODEL_PI*CHASSIS_WHEEL_DIAMETER_MM)*CHASSIS_MOTOR_TO_WHEEL_RATIO);
      wheel_position[w]=w<2?raw:-raw;wheel_position_time[w]=clock_ms;
    }
    track_observer(clock_ms);
    if(ms%30==0)
    {
      float raw=initial_error+2*truth[n>=5?n-5:0];
      if(pi_filter)
      {
        float target=raw,alpha=.35f;
        if(fabsf(raw-filtered)>=12)
        { alpha=.85f; for(unsigned h=0;h<3;h++)history[h]=raw; }
        else
        {
          history[frames%3]=raw;
          target=history[0]+history[1]+history[2]-fminf(history[0],fminf(history[1],history[2]))-
            fmaxf(history[0],fmaxf(history[1],history[2]));
        }
        filtered+=alpha*(target-filtered); frames++;
      }
      in.sequence++;in.sample_ms=clock_ms;in.error_px[0]=roundf(pi_filter?filtered:raw);
    }
    if(ms%50==0)
    {
      in.now_ms=clock_ms;memcpy(in.motion_mm,TrackInput.motion_mm,sizeof in.motion_mm);
      in.body_left_mm=TrackInput.body_left_mm;
      in.feedback_age_ms=clock_ms-TrackObserver.feedback_ms;
      memcpy(in.actual_mm_s,TrackInput.actual_mm_s,sizeof in.actual_mm_s);
      GrabTracking_Update(&controller,&config,&in,&out);
      /* Real common-time velocity can lag by the accepted 600 ms feedback
       * window; include it in startup settling, retain the steady error bound. */
      if(ms>=2600) {sum+=in.error_px[0]*in.error_px[0];count++;peak=fmaxf(peak,fabsf(in.error_px[0]));}
    }
  }
  fprintf(stderr,"staggered closed loop%s speed=%.0f start=%.0f: stationary RMS=%.3f peak=%.3f px\n",pi_filter?" with Pi adaptive filter":"",body_speed,initial_error,sqrtf(sum/count),peak);
  assert(sqrtf(sum/count)<=5 && peak<=10);
}

/* A real tracking load includes X position writes AND reads. UART5 wheel
 * batches can postpone an arm transaction; quiet Z/Base must not starve. */
static void tracking_io_keeps_quiet_axis_feedback(void)
{
  const unsigned delays[]={20,25,80};
  for(unsigned trial=0;trial<3;trial++)
  {
    uint32_t completion=0,max_x_age=0,max_slow_age=0;
    unsigned writes=0,slow_replies=0,delay=delays[trial];
    reset(); configure(true); setting("x_min","0"); setting("x_max","100");
    TrackMoving=true;
    for(unsigned a=0;a<3;a++)
    { Axes[a].position_valid=Axes[a].state_valid=true; Axes[a].position_tick=Axes[a].state_tick=clock_ms; }
    for(unsigned ms=0;ms<5000;ms++)
    {
      clock_ms++;
      /* Real control changes at every new frame/prediction, not one fixed write. */
      if(ms%20==0)
      {
        TrackControlSequence++;
        TrackOutput.velocity_mm_s[2]=20+(ms%100)/10.f;
        TrackOutput.x_target_mm=20+(ms%1000)/100.f;
      }
      if(pending_valid && clock_ms>=completion)
      {
        MechanicalArm_EventTypeDef event=pending;
        pending_valid=false; completion=0;
        if(event.Action==MECHANICAL_ARM_ACTION_STATE)event.StateFlags[event.Axis]=3;
        if(event.Action==MECHANICAL_ARM_ACTION_POSITION)writes++;
        if(event.Axis!=MECHANICAL_ARM_AXIS_X || event.Action==MECHANICAL_ARM_ACTION_STATE)slow_replies++;
        (void)GrabTask_MotorEventHandle(&event);
      }
      track_io(clock_ms,true);
      if(pending_valid && !completion)completion=clock_ms+delay;
      uint32_t age=clock_ms-Axes[0].position_tick;
      if(age>max_x_age)max_x_age=age;
      for(unsigned a=0;a<3;a++)
      {
        age=clock_ms-Axes[a].state_tick;
        if(age>max_slow_age)max_slow_age=age;
        if(a && clock_ms-Axes[a].position_tick>max_slow_age)max_slow_age=clock_ms-Axes[a].position_tick;
      }
    }
    fprintf(stderr,"tracking changing target: transaction=%u X max age=%lu slow max age=%lu writes=%u slow replies=%u\n",
      delay,(unsigned long)max_x_age,(unsigned long)max_slow_age,writes,slow_replies);
    /* The three IO opportunities include one genuine motion update each round. */
    assert(writes>=10 && slow_replies>=10);
    assert(max_x_age<=3*delay+50 && max_slow_age<2000);
  }
}

static void feedback_fault_keeps_trigger_evidence(void)
{
  reset(); configure(true); clock_ms=5000; TrackReady=true;
  memset(&TrackFeedbackFault,0,sizeof TrackFeedbackFault);
  for(unsigned a=0;a<3;a++)
  { Axes[a].position_valid=Axes[a].state_valid=true; Axes[a].position_tick=Axes[a].state_tick=clock_ms; }
  Axes[1].state_tick=clock_ms-2001;
  assert(!track_feedback(clock_ms));
  assert(TrackFeedbackFault.address==MOTOR_ID_ARM_Z && !strcmp(TrackFeedbackFault.kind,"state"));
  assert(TrackFeedbackFault.age_ms==2001 && TrackFeedbackFault.limit_ms==2000);
  Axes[1].state_tick=clock_ms;
  assert(track_feedback(clock_ms));
  assert(TrackFeedbackFault.age_ms==2001); /* returning stop evidence is a different event */
  tracking_fault_log[0]=0;track_status_write(false);
  assert(strstr(tracking_fault_log,"feedback_fault id=6 kind=state age_ms=2001 limit_ms=2000 valid=1"));
  char *observe[]={"grab","track","observe","3"};command(4,observe);
  assert(!TrackFeedbackFault.address);
  puts("tracking feedback fault: exact ID/type/age frozen until next segment PASS");
}

static void batch_timing_excludes_idle_and_previous_motion(void)
{
  reset(); configure(true);
  clock_ms=2000; StartTick=2000; TrackMoving=true;
  bus.sent_velocity_valid=bus.tx_complete=true; bus.stage=MECANUM_STAGE_SENT;
  bus.sent_ms=500; TrackLastBatch=500; TrackBatchInterval=99;
  track_observer(clock_ms);
  assert(TrackInput.batch_interval_ms==0);
  bus.sent_ms=2010; clock_ms=2020; track_observer(clock_ms);
  assert(TrackInput.batch_interval_ms==0); /* First batch is not an interval. */
  bus.sent_ms=2100; clock_ms=2140; track_observer(clock_ms);
  assert(TrackInput.batch_interval_ms==90);
  TrackMoving=false; clock_ms=2200; track_observer(clock_ms);
  assert(TrackInput.batch_interval_ms==0);
  puts("tracking timing: idle/previous task excluded; current completed batches measured PASS");
}

/* Cached feedback just below its age limit must not qualify a new moving
 * segment: its first batch could make it expire before the polling round. */
static void restart_waits_for_current_feedback(void)
{
  char *observe[]={"grab","track","observe"};
  char *reference[]={"grab","track","reference"};
  char *start[]={"grab","track","start"};
  char *stop[]={"grab","track","stop"};
  reset(); configure(true); setting("x_min","0"); setting("x_max","100");
  physical[0]=63;
  command(3,observe);
  for(unsigned i=0;i<45;i++)step(true,0,0,0);
  command(3,reference); command(3,stop); run_to(GRAB_IDLE,200,0);
  clock_ms+=3000;
  for(unsigned a=0;a<3;a++)
  {
    Axes[a].position_valid=Axes[a].state_valid=true;
    Axes[a].position_tick=Axes[a].state_tick=clock_ms-1900;
  }
  for(unsigned w=0;w<4;w++)wheel_position_time[w]=clock_ms-500;
  unsigned requests=velocity_requests;
  float anchor=Tracking.reference_left_mm;
  command(3,start); clock_ms+=50; track_observer(clock_ms);
  clock_ms+=20; track_observer(clock_ms);
  clock_ms+=20; track_observer(clock_ms);
  track_process(clock_ms);
  assert(velocity_requests==requests); /* No motion on pre-start cached proof. */
  assert(!TrackReady && Tracking.reference_left_mm==anchor);
  memset(wheel_position_time,0,sizeof wheel_position_time);
  for(unsigned i=0;i<75;i++)step(true,0,0,0);
  assert(TrackReady && status().state==GRAB_TRACK_FOLLOW);
  assert(velocity_requests>requests && Tracking.reference_left_mm==anchor);
}

static void dispatch_snapshot_is_not_requested_velocity(void)
{
  GrabTracking_Record_t r={0};TrackIo_t io={0};char line[640];
  reset();configure(true);clock_ms=1000;
  bus.sent_velocity_valid=bus.tx_complete=bus.acknowledged=true;bus.sent_ms=920;
  bus.sent_rpm[0]=10;bus.sent_rpm[1]=11;bus.sent_rpm[2]=-12;bus.sent_rpm[3]=-13;
  wheel_position[0]=9007199254740993LL;wheel_feedback_delay=60;
  Axes[0].position_valid=true;Axes[0].raw=12345;
  TrackXDispatch.requested=true;TrackXDispatch.requested_pulses=7000;TrackXDispatch.requested_rpm=17;TrackXDispatch.request_ms=990;
  TrackXDispatch.transmitted=TrackXDispatch.acknowledged=true;TrackXDispatch.sent_ms=950;
  TrackXDispatch.sent_pulses=6500;TrackXDispatch.sent_rpm=15;TrackXDispatch.action=3;
  r.command_mm_s[0]=38;r.command_mm_s[2]=13;track_record_io(&io,clock_ms);
  assert(io.sent_rpm[0]==10 && r.actual_mm_s[0]==0 && r.command_mm_s[0]==38);
  assert(io.wheel_raw[0]==9007199254740993LL && io.wheel_age_ms[0]==60 && io.sent_age_ms==80);
  assert(io.x_requested_pulses==7000 && io.x_sent_pulses==6500 && io.x_sent_age_ms==50);
  unsigned length=track_record_format(line,sizeof line,&r,&io,true,0);
  assert(length>0 && length<sizeof line && strstr(line,"9007199254740993"));
  assert(strstr(line,"6500,15,50,12345,3,") && strstr(line,"7000,17,10,"));
  assert(!track_record_format(line,100,&r,&io,true,0)); /* Never transmit a truncated row. */
  assert(track_record_format(line,sizeof line,&r,&io,false,0) && !strstr(line," io="));
  assert(sizeof r<=80); /* Keep IO out of all 128 frozen ring entries. */
  wheel_stale=true;track_record_io(&io,clock_ms);assert(io.wheel_age_ms[0]==3000);
}

static void static_alignment_then_tracking(void)
{
  char *observe[]={"grab","track","observe"};
  char *reference[]={"grab","track","reference"};
  char *start[]={"grab","track","start"};
  char *stop[]={"grab","track","stop"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=35;
  command(3,observe);
  for(unsigned i=0;i<45;i++)step(true,0,0,0);
  command(3,reference);command(3,stop);run_to(GRAB_IDLE,200,0);
  float anchor=Tracking.reference_left_mm;
  assert(GrabTask_Start("align"));run_to(GRAB_HOLD,250,0);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  unsigned previous_records=Tracking.record_count;
  command(3,start);
  for(unsigned i=0;i<75;i++)step(true,0,6,0);
  assert(TrackReady && FeedbackSelected && feedback_selected==0);
  assert(Tracking.record_count>previous_records && Tracking.last_sequence>0);
  assert(Tracking.reference_valid && Tracking.reference_left_mm==anchor);
  assert(fabsf(Status.forward_mm_s)>.1f);
  command(3,stop);run_to(GRAB_IDLE,200,0);
  assert(status().stop_confirmed);
  puts("tracking task: stationary align -> track -> confirmed stop, reference retained PASS");
}

static void hybrid_visual_timer_schedule(void)
{
  char *observe[]={"grab","track","observe"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=35;command(3,observe);
  for(unsigned i=0;i<45;i++)step(true,0,0,0);
  uint32_t previous=LastControl,updates=Tracking.control_sequence;
  clock_ms=previous+7;camera.Data.Sequence++;camera.Data.Tick=clock_ms;
  GrabTask_Process();assert(LastControl==clock_ms && Tracking.control_sequence==updates+1);
  updates=Tracking.control_sequence;unsigned samples=Tracking.error_count;
  GrabTask_Process();assert(Tracking.control_sequence==updates && Tracking.error_count==samples);
  camera.Data.Sequence++; /* Same receive timestamp is coalesced, not fitted twice. */
  GrabTask_Process();assert(Tracking.control_sequence==updates);
  clock_ms+=20;GrabTask_Process();assert(Tracking.control_sequence==updates+1 && Tracking.error_count==samples);
  updates=Tracking.control_sequence;clock_ms+=20;camera.Data.Sequence++;camera.Data.Tick=clock_ms;
  GrabTask_Process();assert(Tracking.control_sequence==updates+1 && Tracking.error_count==samples+1);
  updates=Tracking.control_sequence;clock_ms+=67;GrabTask_Process();assert(Tracking.control_sequence==updates+1);
  assert(Tracking.control_timing.maximum_ms>=67);
  puts("tracking hybrid: +7ms new frame, duplicates, same stamp, timer, coincidence and late loop PASS");
}

static void stationary_then_moving_keeps_following(void)
{
  char *observe[]={"grab","track","observe"};
  char *reference[]={"grab","track","reference"};
  char *start[]={"grab","track","start"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=50;command(3,observe);
  for(unsigned i=0;i<45;i++)step(true,0,0,0);
  command(3,reference);command(3,start);
  for(unsigned i=0;i<80;i++)step(true,0,0,0);
  assert(status().state==GRAB_TRACK_FOLLOW && TrackMoving);
  unsigned before=velocity_requests;
  for(unsigned i=0;i<12;i++)step(true,0,12,0);
  assert(status().state==GRAB_TRACK_MATCH && TrackMoving);
  assert(velocity_requests>before && fabsf(Status.forward_mm_s)>.1f);
  assert(!closures && !openings && !base_moves);
  puts("tracking: stationary 4s -> moving target remains active PASS");
}

/* X42 0x3A: bit2 is instantaneous stall, bit3 is timed stall protection. */
static void motor_state_distinguishes_stall_from_protection(void)
{
  const uint8_t healthy[]={3,5,7,0x83,0x85,0x87};
  const uint8_t faulty[]={0,2,4,8,9,0x0D};
  MechanicalArm_EventTypeDef event={0};
  event.Action=MECHANICAL_ARM_ACTION_STATE;event.Axis=MECHANICAL_ARM_AXIS_X;
  event.Result=MECHANICAL_ARM_RESULT_OK;
  for(unsigned n=0;n<sizeof healthy+sizeof faulty;n++)
  {
    bool bad=n>=sizeof healthy;
    uint8_t flags=bad?faulty[n-sizeof healthy]:healthy[n];
    reset();configure(true);clock_ms=5000;TrackReady=true;
    Status.mode="track";Status.state=GRAB_TRACK_MATCH;
    event.StateFlags[event.Axis]=flags;PendingAction=event.Action;PendingAxis=event.Axis;
    assert(GrabTask_MotorEventHandle(&event));
    assert((Status.state==GRAB_STOPPING)==bad);
    assert((TrackMotorFault.address!=0)==bad);
    reset();configure(true);clock_ms=6000;TrackReady=true;
    Status.mode="track";Status.state=GRAB_TRACK_MATCH;
    for(unsigned a=0;a<3;a++)
    { Axes[a].position_valid=Axes[a].state_valid=true;Axes[a].position_tick=Axes[a].state_tick=clock_ms; }
    wheel_state_flags[2]=flags;(void)track_feedback(clock_ms);
    assert((Status.state==GRAB_STOPPING)==bad);
    assert((TrackMotorFault.address!=0)==bad);
    reset();configure(true);clock_ms=7000;TrackReady=true;
    Status.mode="track";Status.state=GRAB_TRACK_MATCH;
    wheel_state_flags[2]=flags;(void)wheels(clock_ms);
    assert((Status.state==GRAB_STOPPING)==bad);
    assert((TrackMotorFault.address!=0)==bad);
  }
  puts("motor state: transient stall continues; protection/disabled stops PASS");
}

static void motor_fault_keeps_first_axis_state(void)
{
  reset(); configure(true); clock_ms=5000; TrackReady=true;
  Status.mode="track";Status.state=GRAB_TRACK_MATCH;
  MechanicalArm_EventTypeDef event={0};event.Action=MECHANICAL_ARM_ACTION_STATE;
  event.Axis=MECHANICAL_ARM_AXIS_X;event.Result=MECHANICAL_ARM_RESULT_OK;
  event.StateFlags[event.Axis]=0x0D;PendingAction=event.Action;PendingAxis=event.Axis;
  assert(GrabTask_MotorEventHandle(&event));
  clock_ms=5100;event.StateFlags[event.Axis]=3;PendingAction=event.Action;PendingAxis=event.Axis;
  assert(GrabTask_MotorEventHandle(&event));
  tracking_motor_fault_log[0]=0;track_status_write(false);
  assert(strstr(tracking_motor_fault_log,"motor_fault id=7 kind=axis flags=0x0D"));
  assert(strstr(tracking_motor_fault_log,"received_ms=5000 detected_ms=5000"));
  reset(); configure(true);clock_ms=6000;TrackReady=true;Status.mode="track";
  Status.state=GRAB_TRACK_MATCH;
  for(unsigned a=0;a<3;a++)
  { Axes[a].position_valid=Axes[a].state_valid=true;Axes[a].position_tick=Axes[a].state_tick=clock_ms; }
  wheel_state_flags[2]=0x0D;(void)track_feedback(clock_ms);
  wheel_state_flags[2]=3;clock_ms=6100;(void)track_feedback(clock_ms);
  tracking_motor_fault_log[0]=0;track_status_write(false);
  assert(strstr(tracking_motor_fault_log,"motor_fault id=3 kind=wheel flags=0x0D"));
  assert(strstr(tracking_motor_fault_log,"received_ms=6000 detected_ms=6000"));
  reset();tracking_motor_fault_log[0]=0;track_status_write(false);
  assert(!tracking_motor_fault_log[0]);
}

static void travel_edge_waits_and_resumes_on_return(GrabTracking_Mode_t mode)
{
  char *observe[]={"grab","track","observe"};
  char *reference[]={"grab","track","reference","0"};
  char *start[]={"grab","track","start"};
  char *stop[]={"grab","track","stop"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=100;
  if(mode==GRAB_TRACK_PREDICT)
  {
    command(3,observe);
    for(unsigned i=0;i<45;i++)step(true,0,0,-20);
    command(4,reference);command(3,start);
  }
  else
  {
    char *begin[]={"grab","track",mode==GRAB_TRACK_VISUAL?"visual":"feedback","3","0"};
    command(5,begin);
    for(unsigned i=0;i<65 && !TrackMoving;i++)step(true,0,0,0);
    assert(TrackMoving);
  }
  float anchor=Tracking.reference_left_mm;
  for(unsigned i=0;i<80 && status().state!=GRAB_TRACK_PAUSE;i++)step(true,0,0,-20);
  assert(status().state==GRAB_TRACK_PAUSE && !TrackVisualPause);

  /* Returning target cannot override an unfinished physical stop. */
  wheel_moving=true;
  unsigned requests=velocity_requests;
  for(unsigned i=0;i<8;i++)step(true,0,0,20);
  assert(!status().stop_confirmed && !TrackMoving && velocity_requests==requests);
  wheel_moving=false;
  for(unsigned i=0;i<80 && !status().stop_confirmed;i++)step(true,0,0,-20);
  assert(status().stop_confirmed && status().state==GRAB_TRACK_PAUSE);
  unsigned samples=Tracking.error_count,moves=positions;
  requests=velocity_requests;
  /* Target remains unreachable for 3 seconds: observe without movement. */
  for(unsigned i=0;i<60;i++)step(true,0,0,-20);
  assert(status().state==GRAB_TRACK_PAUSE && !TrackMoving);
  assert(positions==moves && velocity_requests==requests);
  assert(Tracking.error_count>samples && !Tracking.frozen);
  assert(Tracking.reference_valid && Tracking.reference_left_mm==anchor);
  assert(TrackOutput.body_room_mm==0);

  camera_valid=false;
  for(unsigned i=0;i<10;i++)step(true,0,0,20);
  assert(status().state==GRAB_TRACK_PAUSE && velocity_requests==requests);
  camera_valid=true;
  camera.Data.Sequence=Tracking.last_sequence;camera.Data.Tick=Tracking.last_sample_ms;
  step(false,0,0,20); /* Repeated snapshot is not a new returning target. */
  assert(status().state==GRAB_TRACK_PAUSE && !TrackMoving);
  step(true,0,0,20);
  assert(status().state==GRAB_TRACK_MATCH && TrackMoving);
  assert(!status().stop_requested && !status().stop_confirmed);
  assert(velocity_requests>requests && Status.left_mm_s<0);
  assert(hypotf(Status.left_mm_s,TrackOutput.velocity_mm_s[2])<=TrackConfig.accel_mm_s2*.05f+.01f);
  assert(Tracking.reference_left_mm==anchor && TrackOutput.body_room_mm==0);

  /* Do not freeze permanently on a second pass through the same travel edge. */
  physical[0]=100;
  for(unsigned i=0;i<80 && status().state!=GRAB_TRACK_PAUSE;i++)step(true,0,0,-20);
  assert(status().state==GRAB_TRACK_PAUSE);
  for(unsigned i=0;i<80 && !status().stop_confirmed;i++)step(true,0,0,-20);
  command(3,stop);run_to(GRAB_IDLE,200,0);
  requests=velocity_requests;
  for(unsigned i=0;i<20;i++)step(true,0,0,20);
  assert(status().state==GRAB_IDLE && !TrackMoving && velocity_requests==requests);
  assert(Tracking.reference_left_mm==anchor && Tracking.frozen);
  assert(!closures && !openings && !base_moves);
  puts("tracking: edge wait, fresh return, retained zero budget, manual stop PASS");
}

static void visual_pause_still_recovers(GrabTracking_Mode_t mode)
{
  char *observe[]={"grab","track","observe"};
  char *reference[]={"grab","track","reference"};
  char *start[]={"grab","track","start"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  physical[0]=50;
  if(mode==GRAB_TRACK_PREDICT)
  {
    command(3,observe);
    for(unsigned i=0;i<45;i++)step(true,0,0,0);
    command(3,reference);command(3,start);
  }
  else
  {
    char *begin[]={"grab","track",mode==GRAB_TRACK_VISUAL?"visual":"feedback","3","50"};
    command(5,begin);
  }
  for(unsigned i=0;i<30;i++)step(true,0,0,0);
  float anchor=Tracking.reference_left_mm;
  camera_valid=false;
  for(unsigned i=0;i<40;i++)step(true,0,0,0);
  assert(status().state==GRAB_TRACK_PAUSE && TrackVisualPause && status().stop_confirmed);
  camera_valid=true;step(true,0,0,0);
  assert(TrackMoving && status().state==GRAB_TRACK_MATCH);
  assert(Tracking.reference_left_mm==anchor);
  assert(!closures && !openings && !base_moves);
  puts("tracking: existing visual pause and fresh observation recovery PASS");
}

int main(void)
{
  reset();
  assert(TrackConfig.body_speed_mm_s==160 && TrackConfig.x_speed_mm_s==60);
  assert(TrackConfig.lambda_s==2.5f && TrackConfig.horizon_ms==120);
  for(unsigned i=0;i<2;i++)
  {
    assert(TrackComparison[i].body_speed_mm_s==160 && TrackComparison[i].x_speed_mm_s==60);
    assert(TrackComparison[i].lambda_s==2 && TrackComparison[i].horizon_ms==60);
    assert(TrackComparison[i].accel_mm_s2==300 && TrackComparison[i].body_accel_mm_s2==300);
    assert(TrackComparison[i].x_accel_mm_s2==120 && TrackComparison[i].tolerance_px==5);
    assert(TrackComparison[i].lead_mm==12 && TrackComparison[i].grace_ms==250);
    assert(TrackComparison[i].control_ms==20 && TrackComparison[i].batch_ms==0);
  }
  for(unsigned mode=GRAB_TRACK_PREDICT;mode<=GRAB_TRACK_FEEDBACK;mode++) {
    reset();configure(true);setting("x_max","100");
    char *observe[]={"grab","track","observe","3"};command(4,observe);
    TrackMode=(GrabTracking_Mode_t)mode;
    pixel_center_override=true;pixel_center_x=334;pixel_center_y=338;
    for(unsigned i=0;i<40;i++)step(true,0,17,-9);
    assert(TrackReady && TrackInput.error_px[0]==17 && TrackInput.error_px[1]==-9);
    assert(status().dx==17 && status().dy==-9 && !velocity_requests);
    pixel_center_x=372;pixel_center_y=345;
    step(true,0,0,0);
    assert(TrackInput.error_px[0]==0 && TrackInput.error_px[1]==0);
    assert(status().dx==0 && status().dy==0);
    pixel_center_x=430;pixel_center_y=300;step(true,0,-25,12);
    assert(TrackInput.error_px[0]==-25 && TrackInput.error_px[1]==12);
  }
  reset(); configure(true);
  char *set_visual[]={"grab","track","set","x_speed","25","visual"};
  command(6,set_visual);assert(TrackComparison[0].x_speed_mm_s==25 && TrackConfig.x_speed_mm_s==60);
  char *set_feedback[]={"grab","track","set","x_accel","110","feedback"};
  command(6,set_feedback);assert(TrackComparison[1].x_accel_mm_s2==110 && TrackComparison[0].x_accel_mm_s2==120);
  char *visual[]={"grab","track","visual","3","30"};
  command(5,visual);
  assert(TrackAutoStart && Status.state==GRAB_TRACK_OBSERVE);
  assert(!TrackPickArmed);
  for(unsigned i=0;i<65;i++)step(true,0,0,0);
  assert(TrackMode==GRAB_TRACK_VISUAL && TrackMoving && Tracking.reference_valid);
  assert(openings==0 && closures==0 && base_moves==0);
  float original_reference=Tracking.reference_left_mm;
  char *feedback[]={"grab","track","feedback","3","50"};
  output[0]=0;assert(GrabTask_Command(5,feedback));
  assert(strstr(output,"ERR") && TrackMode==GRAB_TRACK_VISUAL);
  char *comparison_stop[]={"grab","track","stop"};command(3,comparison_stop);
  for(unsigned i=0;i<90 && status().state!=GRAB_IDLE;i++)step(true,0,0,0);
  assert(Status.stop_confirmed);
  command(5,feedback);assert(TrackMode==GRAB_TRACK_FEEDBACK && !TrackPickArmed);
  assert(!Tracking.have_sample && Tracking.samples==0 && Tracking.motions==0);
  assert(Tracking.reference_valid && Tracking.reference_left_mm==original_reference);
  for(unsigned i=0;i<65;i++)step(true,0,0,0);
  assert(TrackMoving && Tracking.reference_left_mm==original_reference);
  assert(TrackComparison[0].x_speed_mm_s==25 && TrackConfig.x_speed_mm_s==60);
  const GrabTracking_Record_t *comparison_row=GrabTracking_Record(&Tracking,Tracking.record_count-1);
  char formatted[640];assert(track_record_format(formatted,sizeof formatted,comparison_row,NULL,false,0));
  assert(strstr(formatted,"mode=feedback") && strstr(formatted," crc="));
  reset(); configure(true); configured_x_acc=47;
  TrackControlSequence=1; TrackXControlSequence=0; TrackIoSlot=0;
  TrackOutput.velocity_mm_s[2]=20; TrackOutput.x_target_mm=1;
  track_io(clock_ms,true); assert(x_position_acc==47 && positions==1);
  char *observe[]={"grab","track","observe"};
  char *reference[]={"grab","track","reference"};
  char *start[]={"grab","track","start"};
  char *stop[]={"grab","track","stop"};
  reset(); configure(true); setting("x_min","0"); setting("x_max","100");
  physical[0]=35; physical[1]=7; physical[2]=18;
  char *log_start[]={"grab","track","log","start"};
  char *log_stop[]={"grab","track","log","stop"};
  command(4,log_start);
  assert(strstr(output,"OK grab track log_begin"));
  assert(positions==0 && velocity_requests==0 && camera_requests==0);
  command(3,observe);
  for(unsigned i=0;i<45;i++)step(true,0,0,0);
  assert(positions==0 && openings==0 && closures==0 && velocity_requests==0 && base_moves==0);
  assert(camera.RequestTarget==CAMERA_COLOR_BLUE);
  assert(tracking_live_rows>0 && strstr(tracking_live_log,"TRACK live id=1 i=") && strstr(tracking_live_log," io="));
  assert(strstr(tracking_live_log," crc="));
  command(4,log_stop);
  assert(strstr(output,"OK grab track log_end") && status().state==GRAB_TRACK_OBSERVE);
  unsigned sent=camera_requests;
  command(3,observe); /* Pi was restarted; an explicit observe must resend B2. */
  assert(camera_requests==sent+1);
  command(3,reference); command(3,start);
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(status().state==GRAB_TRACK_FOLLOW && positions==0);
  assert(physical[0]==35 && physical[1]==7 && physical[2]==18);
  command(3,stop); run_to(GRAB_IDLE,200,0);
  assert(status().stop_confirmed && Tracking.reference_valid);
  assert(openings==0 && closures==0 && base_moves==0);
  output[0]=0; char *trace[]={"grab","track","trace","0"}; command(4,trace);
  clock_ms+=250;track_trace_process();
  assert(strstr(output,"TRACK row i=0") && strstr(output," data=") && strstr(output," crc="));
  assert(!strstr(output," io="));
  output[0]=0; track_status_write(false);
  Status.base_home_degraded=true;track_status_write(true);
  assert(strstr(progress_log,"OK grab track state=") && strstr(progress_log,"base_home_degraded=1"));
  Status.base_home_degraded=false;
  assert(strstr(tracking_estimate_log,"OK grab track estimate fit=") && strstr(tracking_estimate_log," comp=") && strstr(tracking_estimate_log," confidence="));
  float x_zero=TrackXZero,table_anchor=Tracking.reference_left_mm;
  clock_ms+=10000; /* No polling while idle. This is not an active drive failure. */
  command(3,observe);
  assert(!TrackReady);
  for(unsigned i=0;i<45;i++)step(true,0,0,0);
  assert(status().state==GRAB_TRACK_OBSERVE && Tracking.reference_valid);
  assert(TrackXZero==x_zero && Tracking.reference_left_mm==table_anchor);
  /* After an MCU reset the physical table gap is not reset. Re-establishing
   * a smaller known remaining budget must not silently credit 50 mm. */
  char *remaining[]={"grab","track","reference","37"};
  command(4,remaining);
  assert(fabsf(50-(TrackInput.body_left_mm-Tracking.reference_left_mm)-37)<.01f);
  table_anchor=Tracking.reference_left_mm;
  char *excess[]={"grab","track","reference","51"};
  output[0]=0; assert(GrabTask_Command(4,excess));
  assert(strstr(output,"ERR grab track reference budget"));
  assert(Tracking.reference_left_mm==table_anchor);
  command(3,start);
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(fabsf(TrackOutput.body_room_mm-37)<.1f);
  command(3,stop); run_to(GRAB_IDLE,200,0);
  assert(Tracking.reference_left_mm==table_anchor);
  delayed_self_motion();
  staggered_self_motion();
  reversing_staggered_motion();
  stationary_closed_loop(false,20,-14); stationary_closed_loop(true,20,-14);
  stationary_closed_loop(true,160,20); stationary_closed_loop(true,160,75);
  tracking_io_keeps_quiet_axis_feedback();
  feedback_fault_keeps_trigger_evidence();
  motor_state_distinguishes_stall_from_protection();
  motor_fault_keeps_first_axis_state();
  batch_timing_excludes_idle_and_previous_motion();
  restart_waits_for_current_feedback();
  static_alignment_then_tracking();
  dispatch_snapshot_is_not_requested_velocity();
  hybrid_visual_timer_schedule();
  stationary_then_moving_keeps_following();
  for(unsigned mode=GRAB_TRACK_PREDICT;mode<=GRAB_TRACK_FEEDBACK;mode++)
  {
    travel_edge_waits_and_resumes_on_return((GrabTracking_Mode_t)mode);
    visual_pause_still_recovers((GrabTracking_Mode_t)mode);
  }
  puts("tracking task: observe is read-only, no retract/Z/Base/claw, stop feedback PASS");
}
