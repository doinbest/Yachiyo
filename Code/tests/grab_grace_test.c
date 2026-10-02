/* Production state/control code with the same synthetic hardware endpoints. */
#define main grab_legacy_regression_main
#include "grab_task_test.c"
#undef main

static void moving_blue(void)
{
  unsigned i;
  reset(); configure(true); setting("loss_grace_ms","250"); assert(GrabTask_Start("align"));
  run_to(GRAB_ALIGN,180,1);
  for(i=0;i<30;i++) step(true,1,-95,0);
  assert(status().forward_mm_s>5 && status().state==GRAB_ALIGN);
}
static void one_missed_sample_does_not_stop(void)
{
  unsigned before, stop_before; float speed;
  moving_blue(); before=camera_requests; stop_before=stops;
  speed=status().forward_mm_s;
  camera_valid=false; step(true,1,-95,0);
  assert(!strcmp(status().state_name,"vision_grace") && !status().stop_requested);
  assert(status().forward_mm_s<=speed && status().forward_mm_s>=0);
  camera_valid=true;
  step(true,1,-94,0); step(true,1,-93,0);
  assert(!strcmp(status().state_name,"vision_grace"));
  step(true,1,-92,0);
  assert(status().state==GRAB_ALIGN && RecoveryUsed==0 && !CaptureCount);
  assert(camera_requests==before && stops==stop_before && !closures);
  assert(status().loss_ms==200 && status().loss_max_ms==200 && !status().grace_count);
}
static void frozen_deadline_and_deceleration(void)
{
  unsigned i; uint32_t anchor; float previous;
  moving_blue(); anchor=camera.Data.Tick; previous=status().forward_mm_s;
  camera_valid=false;
  for(i=1;i<5;i++) {
    step(true,1,-200,0);
    assert(status().state==GRAB_VISION_GRACE && LossTick==anchor && status().loss_ms==50*i);
    assert(status().forward_mm_s<=previous && status().forward_mm_s>=0);
    previous=status().forward_mm_s;
    assert(!CaptureCount && !VisualStable && !closures);
  }
  assert(previous<10); /* At least one deceleration command reached the synthetic bus. */
  step(true,1,-200,0);
  assert(status().state==GRAB_VISION_PAUSE && status().stop_requested && !status().stop_confirmed);
  assert(status().loss_ms==250 && !RecoveryUsed);

  moving_blue(); anchor=camera.Data.Tick;
  camera_valid=false; step(true,1,-95,0);
  camera_valid=true; step(true,1,-94,0); assert(status().grace_count==1);
  camera_valid=false; step(true,1,-94,0); assert(!status().grace_count);
  camera_valid=true; step(true,1,-93,0); assert(status().grace_count==1);
  step(true,1,-92,0); /* Valid again at the deadline cannot renew the anchor. */
  assert(status().state==GRAB_VISION_PAUSE && LossTick==anchor && !RecoveryUsed);
}
static void silence_duplicates_and_observation_count(void)
{
  unsigned i;
  moving_blue(); step(false,1,-95,0); assert(status().state==GRAB_ALIGN);
  step(false,1,-95,0); assert(status().state==GRAB_VISION_GRACE && status().loss_ms==100);
  step(false,1,-95,0); assert(!status().grace_count);
  step(false,1,-95,0); assert(!status().grace_count);
  step(false,1,-95,0); assert(status().state==GRAB_VISION_PAUSE);

  moving_blue(); camera_valid=false; step(true,1,-95,0);
  camera_valid=true; step(true,1,-95,0); assert(status().grace_count==1);
  for(i=0;i<10;i++) GrabTask_Process(); /* Same packet is never 3 observations. */
  assert(status().grace_count==1 && status().state==GRAB_VISION_GRACE);
  camera.Data.Sequence+=10; step(false,1,-95,0);
  assert(status().grace_count==2); /* A USB batch is only one observation. */
  camera.InvalidCount++; step(true,1,-95,0); assert(status().grace_count==1);
  step(true,1,-95,0); assert(status().state==GRAB_VISION_PAUSE);
}
static void short_episodes_preserve_quota(void)
{
  unsigned episode;
  moving_blue();
  for(episode=0;episode<4;episode++) {
    camera_valid=false; step(true,1,-95,0);
    camera_valid=true; step(true,1,-94,0); step(true,1,-93,0); step(true,1,-92,0);
    assert(status().state==GRAB_ALIGN && !RecoveryUsed && !CaptureCount && !closures);
    step(true,1,-95,0);
  }
  assert(camera_requests==1 && status().loss_max_ms==200);
}
static void near_target_reverse_and_pick_gap(void)
{
  unsigned i;
  moving_blue(); step(true,1,3,3); camera_valid=false; step(true,1,3,3);
  assert(status().state==GRAB_VISION_PAUSE && !CaptureCount);
  for(i=0;i<2;i++) {
    moving_blue(); camera_valid=false; step(true,1,-95,0);
    camera_valid=true; step(true,1,i?20:0,0);
    assert(status().state==GRAB_VISION_PAUSE && !RecoveryUsed && !closures);
  }
  reset(); configure(true); setting("loss_grace_ms","250"); assert(GrabTask_Start("pick"));
  run_to(GRAB_ALIGN,180,1); step(true,1,-95,0); camera_valid=false; step(true,1,-95,0);
  assert(status().state==GRAB_VISION_GRACE); /* Pick now shares align's short gap. */
  moving_blue(); run_to(GRAB_SETTLE,200,1); camera_valid=false; step(true,1,0,0);
  assert(status().state==GRAB_VISION_PAUSE && !closures);
}
static void extended_recovery_after_standstill(void)
{
  unsigned i; uint32_t stopped_at;
  moving_blue(); camera_valid=false;
  for(i=0;i<5;i++) step(true,1,-95,0);
  assert(status().state==GRAB_VISION_PAUSE);
  wheel_moving=true;
  for(i=0;i<12;i++) step(true,1,-95,0);
  assert(!RecoveryWaiting && status().recovery_left_ms==-1);
  wheel_moving=false;
  for(i=0;i<80 && !RecoveryWaiting;i++) step(true,1,-95,0);
  assert(RecoveryWaiting && status().stop_confirmed && status().recovery_left_ms==1500);
  stopped_at=clock_ms;
  for(i=0;i<20;i++) step(true,1,-95,0);
  assert(status().state==GRAB_VISION_PAUSE && status().recovery_left_ms==500 && camera_requests==2);
  camera_valid=true; step(true,1,-94,0); step(true,1,-93,0); step(true,1,-92,0);
  assert(clock_ms-stopped_at==1150 && status().state==GRAB_ALIGN && RecoveryUsed==1);
  assert(status().loss_ms>1500 && status().loss_ms==status().loss_max_ms && !CaptureCount && !closures);
}
static void faults_and_manual_stop_bypass_grace(void)
{
  unsigned scenario;
  const char *reasons[]={"cancelled","vision_usb_off","vision_request_lost","feedback_stale","chassis_tx","travel_limit","imu_invalid"};
  for(scenario=0;scenario<7;scenario++) {
    moving_blue(); camera_valid=false; step(true,1,-95,0);
    switch(scenario) {
      case 0: GrabTask_Stop(); break;
      case 1: camera.UsbConfigured=0; GrabTask_Process(); break;
      case 2: camera.RequestTarget=CAMERA_COLOR_RED; GrabTask_Process(); break;
      case 3: wheel_stale=true; step(false,1,-95,0); break;
      case 4: bus.locked=true; GrabTask_Process(); break;
      case 5: wheel_position[0]=65536; GrabTask_Process(); break;
      default:
        imu_valid=false;
        for(unsigned i=0;i<3 && status().state==GRAB_VISION_GRACE;i++) step(false,1,-95,0);
    }
    assert(status().state==GRAB_STOPPING && !strcmp(status().reason,reasons[scenario]));
    assert(status().stop_requested && !closures && !RecoveryUsed);
  }
}
static void configuration_and_wire_status(void)
{
  unsigned i;
  const char *invalid[]={"-1","301","1.5","nan"};
  char *get[]={"grab","get","loss_grace_ms"};
  reset(); assert(GrabTask_Command(3,get)); assert(strstr(output,"value=250"));
  setting("loss_grace_ms","0"); setting("loss_grace_ms","200"); setting("loss_grace_ms","300");
  for(i=0;i<4;i++) {
    char *set[]={"grab","set","loss_grace_ms",(char *)invalid[i]};
    assert(GrabTask_Command(4,set)); assert(strstr(output,"ERR"));
  }
  moving_blue(); camera_valid=false; step(true,1,-95,0);
  status_write(false);
  assert(strstr(output,"dx=na dy=na") && strstr(output,"loss_ms=50 loss_max_ms=50 grace_count=0\r\n"));
  status_write(true); assert(strstr(progress_log,"state=vision_grace") && strstr(progress_log,"grace_count=0\r\n"));
  /* Debug must remain a complete line even at uint32 counter maxima. */
  LossActive=false; LossLastMs=LossMaxMs=0xffffffffU;
  Status.reason="vision_recover_timeout"; status_write(true);
  assert(strlen(progress_log)<320 && strstr(progress_log,"loss_max_ms=4294967295") && strstr(progress_log,"grace_count=0\r\n"));
}
static void timestamp_and_sequence_wrap(void)
{
  unsigned i; uint32_t delta;
  moving_blue(); delta=0xffffff80U-clock_ms;
  clock_ms+=delta; Vision.Tick+=delta; camera.Data.Tick+=delta; RequestTick+=delta;
  LastPoll+=delta; LastControl+=delta; StartTick+=delta; StateTick+=delta;
  for(i=0;i<3;i++) { Axes[i].position_tick+=delta; Axes[i].state_tick+=delta; Axes[i].quiet_since+=delta; }
  for(i=0;i<4;i++) WheelLastPositionTick[i]+=delta;
  camera.Data.Sequence=LastSequence=0xfffffffeU;
  camera_valid=false; step(true,1,-95,0);
  camera_valid=true; step(true,1,-94,0); step(true,1,-93,0); step(true,1,-92,0);
  assert(status().state==GRAB_ALIGN && status().loss_ms==200 && !RecoveryUsed);
}
int main(void)
{
  one_missed_sample_does_not_stop();
  frozen_deadline_and_deceleration(); silence_duplicates_and_observation_count();
  short_episodes_preserve_quota(); near_target_reverse_and_pick_gap();
  extended_recovery_after_standstill(); faults_and_manual_stop_bypass_grace();
  configuration_and_wire_status(); timestamp_and_sequence_wrap();
  puts("grab_grace_test: PASS"); return 0;
}
