#define main legacy_feedback_main
#include "mecanum_feedback_test.c"
#undef main
#include <stdlib.h>

int main(int argc,char **argv)
{
  uint32_t last_batch=0,last_x=0,x_request=0,x_write=0,max_batch=0,max_age=0,last_sent=0;
  unsigned batches=0,x_replies=0,x_holds=0;
  uint32_t last_start=0,last_sequence=0,min_start=UINT32_MAX;
  MotorBus_Init(&uart); all_wheels=true; acknowledge_motion=true; reply_delay=argc>1?(uint32_t)atoi(argv[1]):8;
  assert(Mecanum_Feedback_Select(0)); Mecanum_Feedback_Enable(true);
  for(unsigned i=0;i<700;i++)step();
  Mecanum_TrackingTiming(true,argc>2?80:0);
  for(unsigned i=0;i<4000;i++)
  {
    MotorBus_Event_t event; Mecanum_Status_t status;
    if(MotorBus_EventGet(MOTOR_BUS_ARM,&event))
    { assert(event.result==MOTOR_BUS_REPLY || (event.result==MOTOR_BUS_REPLY && event.token==99));
      if(event.token==99)x_holds++;
      else if(event.data[1]==0x36){last_x=tick; x_replies++;} }
    if(tick-x_request>=80 && !MotorBus_OwnerBusy(MOTOR_BUS_ARM))
    {
      uint8_t request[]={5,0x36,0x6b};
      if(MotorBus_Submit(MOTOR_BUS_ARM,request,3,8,tick,false))x_request=tick;
    }
    else if(tick-x_write>=80 && !MotorBus_OwnerBusy(MOTOR_BUS_ARM))
    {
      /* Include the X position stream, not only a read-only observer. */
      uint8_t request[]={5,0xfd,0,0,20,8,0,0,2,10,1,0,0x6b};
      uint8_t hold[]={7,0xfe,0x98,0,0x6b};
      /* Alternate routine FE holds with position updates. A priority FE here
       * cancels a wheel ACK and must fail the unlocked assertion below. */
      bool stopping=(i/250)%2!=0;
      if(MotorBus_Submit(MOTOR_BUS_ARM,stopping?hold:request,stopping?5:sizeof request,
                         stopping?0:4,stopping?99:tick,false))x_write=tick;
    }
    if(i%20==0 || i%47==0)assert(Mecanum_Velocity_Request(60+(i%200)/2.f,20,0,20));
    step(); Mecanum_StatusGet(&status);
    if(status.sequence!=last_sequence) {
      if(last_start && tick-last_start<min_start)min_start=tick-last_start;
      last_start=tick;last_sequence=status.sequence;
    }
    assert(!status.locked);
    if(status.sent_ms!=last_sent && status.tx_complete)
    {
      if(last_batch && tick-last_batch>max_batch)max_batch=tick-last_batch;
      last_batch=tick; last_sent=status.sent_ms; batches++;
    }
    if(last_x && tick-last_x>max_age)max_age=tick-last_x;
    for(unsigned w=0;w<4;w++)
    {
      Mecanum_Feedback_t f; assert(Mecanum_FeedbackGet(w,&f));
      if(tick-f.position_ms>600 || tick-f.state_ms>2000 || tick-f.speed_ms>2000)
        fprintf(stderr,"tracking stale wheel=%u time=%lu ages=%lu,%lu,%lu\n",w,(unsigned long)tick,
          (unsigned long)(tick-f.position_ms),(unsigned long)(tick-f.state_ms),(unsigned long)(tick-f.speed_ms));
      assert(tick-f.position_ms<=600 && tick-f.state_ms<=2000 && tick-f.speed_ms<=2000);
    }
  }
  Mecanum_Timing_t timing;Mecanum_TimingGet(&timing);
  uint32_t sum=0,p95=0;
  for(unsigned i=0;i<=200;i++){sum+=timing.interval_bins[i];if(sum>=(timing.intervals*95+99)/100){p95=i;break;}}
  printf("tracking bus simulated %lums reply: %u batches, batch p95 %lu max %lu ms, X replies %u max age %lu ms\n",(unsigned long)reply_delay,batches,(unsigned long)p95,(unsigned long)max_batch,x_replies,(unsigned long)max_age);
  fflush(stdout);
  if(reply_delay==2 && argc==2)assert(p95<=60 && max_age<=120);
  if(argc>2)assert(min_start>=80); /* Limit is batch start; ACK duration may vary. */
  assert(batches>=25 && max_batch<=120 && x_replies>=30 && max_age<=200 && x_holds>=3);
  assert(Mecanum_Test_Stop()); for(unsigned i=0;i<700;i++)step();
  Mecanum_TrackingTiming(false,0);
  Mecanum_Feedback_Enable(false);
  /* The X client must consume its last terminal reply before resetting the
   * test fixture; the production GrabTask keeps consuming this owner. */
  { MotorBus_Event_t event; (void)MotorBus_EventGet(MOTOR_BUS_ARM,&event); }
  for(unsigned i=0;i<30;i++)step();
  assert(Mecanum_RecoveryAfterReset());
  /* Receive-mode interruptions at different points in an all-zero batch
   * must permit restarting, without driver reset or a late sync broadcast. */
  for(unsigned trial=0;trial<5;trial++)
  {
    Mecanum_Status_t status; unsigned baseline=count;
    assert(Mecanum_Velocity_Request(0,0,0,20));
    for(unsigned i=0;i<200 && count==baseline;i++)step();
    assert(count>baseline && frame[1]==0xf6);
    for(unsigned i=0;i<1+trial*12;i++)step();
    unsigned stops=stop_count;
    assert(Mecanum_Test_Stop());
    for(unsigned i=0;i<150;i++)step();
    Mecanum_StatusGet(&status);
    if(stop_count!=stops+4 || !status.tx_complete || status.locked || status.error!=MECANUM_ERROR_NONE || status.stage!=MECANUM_STAGE_STOPPED)
      fprintf(stderr,"zero stop trial=%u stops=%u stage=%u error=%u locked=%u tx=%u quarantine=%u\n",trial,stop_count-stops,status.stage,status.error,status.locked,status.tx_complete,MotorBus_IsQuarantined());
    assert(stop_count==stops+4 && status.tx_complete && !status.locked &&
           status.error==MECANUM_ERROR_NONE && status.stage==MECANUM_STAGE_STOPPED);
  }
  puts("tracking bus: Receive ACK, fair feedback/X read, latest command and stop PASS");
}
