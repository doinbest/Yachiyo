#define main legacy_main
#include "grab_task_test.c"
#undef main

static void seed(unsigned i,float mm)
{
  Axes[i].mm=mm;Axes[i].position_valid=Axes[i].state_valid=true;
  Axes[i].raw=(int64_t)llroundf(mm*Config[i?C_Z_PPM:C_X_PPM]*65536/3200);
  Axes[i].position_tick=Axes[i].state_tick=clock_ms;Axes[i].flags=1;
}
static void configured_speed_and_acceleration(void)
{
  reset();configure(false);clock_ms=1000;Status.state=GRAB_LIFT;
  configured_x_acc=37;configured_z_acc=149;
  Config[C_X_TRANSFER_SPEED]=32;Config[C_Z_SPEED]=64;
  seed(1,-50);
  assert(axis_move(1,0));
  assert(z_position_acc==149 && z_position_rpm==120 && z_position_pulses==0);
  assert(!arrived(1,clock_ms)); /* Submission is not physical arrival. */
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  seed(0,0);
  assert(axis_move(0,.5f)); /* Short moves retain the same driver-owned ramp. */
  assert(x_position_acc==37 && x_position_rpm==60 && destination[0]==.5f);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  Status.state=GRAB_ALIGN;Config[C_X_SPEED]=16;
  assert(axis_move(0,1));
  assert(x_position_acc==37 && x_position_rpm==30);
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  assert(axis_move(1,-10)); /* Progressive visual Z remains a finite target. */
  assert(z_position_acc==149 && z_position_rpm==120 && z_position_pulses==-1000);
}
/* Integrate finite absolute drive commands, then deliver independently sampled
 * encoder/state replies. Unlike the legacy fixture, ACK does not move an axis. */
static void simultaneous_moves(void)
{
  reset();configure(false);clock_ms=1000;Status.state=GRAB_LIFT;
  physical[0]=0;physical[1]=-15;seed(0,0);seed(1,-15);
  assert(axis_move(1,10));
  pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
  assert(axis_move(0,15));
  float peak[2]={0,0};bool done=false;
  for(unsigned t=0;t<1000;t++) {
    clock_ms+=10;
    for(unsigned i=0;i<2;i++) {
      float v=(i?z_position_rpm:x_position_rpm)*3200.f/(100.f*60.f);
      if(v>peak[i])peak[i]=v;
      float step=fminf(fabsf(destination[i]-physical[i]),v*.01f);
      physical[i]+=copysignf(step,destination[i]-physical[i]);
      assert(physical[i]>=-15 && physical[i]<=15);
    }
    if(pending_valid) {
      MechanicalArm_EventTypeDef e=pending;
      pending_valid=false;
      unsigned i=e.Axis==MECHANICAL_ARM_AXIS_X?0:e.Axis==MECHANICAL_ARM_AXIS_Z?1:2;
      if(e.Action==MECHANICAL_ARM_ACTION_POSITION_READ) {
        e.CurrentPositionRaw=(int64_t)llroundf(physical[i]*100*65536/3200);
        if((i==0?MECHANICAL_ARM_X_POSITIVE_DIR:i==1?MECHANICAL_ARM_Z_POSITIVE_DIR:MECHANICAL_ARM_BASE_POSITIVE_DIR)!=0)e.CurrentPositionRaw=-e.CurrentPositionRaw;
      }
      e.StateFlags[e.Axis]=3;
      assert(GrabTask_MotorEventHandle(&e));
    }
    poll_axes(clock_ms);
    if(arrived(0,clock_ms) && arrived(1,clock_ms)) {done=true;break;}
  }
  assert(done && peak[0]>3 && peak[1]>5);
  assert(peak[0]<=Config[C_X_TRANSFER_SPEED] && peak[1]<=Config[C_Z_SPEED]);
  assert(fabsf(physical[0]-15)<.1f && fabsf(physical[1]-10)<.1f);
  assert(x_position_acc==20 && z_position_acc==255);
  assert(positions==2); /* Polling must not reissue position commands. */
  printf("simultaneous X/Z finite moves completed in %lu ms, peak %.2f/%.2f mm/s\n",(unsigned long)(clock_ms-1000),peak[0],peak[1]);
}
static void cancel_pending_motion(void)
{
  reset();configure(false);clock_ms=1000;Status.state=GRAB_LIFT;
  seed(1,-50);physical[1]=-50;move_arrives=false;
  assert(axis_move(1,10));
  assert(!arrived(1,clock_ms));
  GrabTask_Stop();unsigned before=positions;
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(positions==before && !Axes[1].commanded);
}
int main(void)
{
  configured_speed_and_acceleration();simultaneous_moves();cancel_pending_motion();
  puts("grab axis driver acceleration tests passed");
  return 0;
}
