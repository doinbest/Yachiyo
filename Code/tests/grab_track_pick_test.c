#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void command(unsigned n,char *a[])
{ output[0]=0;assert(GrabTask_Command(n,a));assert(!strstr(output,"ERR")); }

static void pickup_drift_and_cancel(void)
{
  char *pick[]={"grab","track","pick","3","-8","110"};
  char *begin[]={"grab","track","begin","3","20"};
  reset();configure(true);setting("x_min","0");setting("x_max","100");
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
  assert(!closures && status().state!=GRAB_DESCEND && !TrackPickParking);
  camera_valid=false;
  for(unsigned i=0;i<50;i++)step(true,0,25,0);
  assert(!closures && status().state==GRAB_TRACK_PAUSE);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  camera_valid=true;command(6,pick);run_to(GRAB_DESCEND,200,0);
  move_arrives=false;for(unsigned i=0;i<20;i++)step(true,0,0,0);
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
  assert(status().stop_confirmed && !closures);
  run_to(GRAB_CLOSE,100,0);
  assert(z_position_rpm==(uint16_t)floorf(110*100*60/3200));
  assert(closures==1 && fabsf(physical[1]+8)<.01f);
  run_to(GRAB_COMPLETE,150,0);
  assert(fabsf(physical[1]-10)<.01f && !tray_indexes && !base_moves);
  assert(!strcmp(status().result,"picked"));
  assert(Tracking.reference_left_mm==anchor);
  pickup_drift_and_cancel();
  puts("tracking one-click begin, retained budget, fast pickup then lift PASS");
}
