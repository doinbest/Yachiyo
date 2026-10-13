#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main

/* Real task parser/controller, with the existing virtual UART/mechanics fixture. */
static void stack_manual_height_and_release(void)
{
  char *align[]={"grab","align","color","3","0"};
  char *lower[]={"grab","stack","lower","-90"};
  char *adjust[]={"grab","stack","lower","-92.123"};
  char *release[]={"grab","stack","release"};
  char *lift[]={"grab","stack","lift"};
  reset();configure(true);setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_car_lift","0");setting("x_max","100");
  setting("body_speed","160");setting("dx_tol","1");setting("dy_tol","1");
  assert(GrabTask_Command(5,align));
  assert(status().state==GRAB_PREPARE);
  run_to(GRAB_HOLD,500,1);
  assert(requested_color==CAMERA_COLOR_BLUE && !tray_indexes && !closures);
  unsigned opened=openings,velocities=velocity_requests,turns=base_moves;
  Arm_GripperSet(ARM_GRIPPER_CATCH); /* The user loads and closes explicitly. */
  assert(GrabTask_Command(4,lower));
  move_arrives=false;
  for(unsigned i=0;i<15;i++)step(true,1,80,20);
  assert(openings==opened && physical[1]>-89 && status().state!=GRAB_HOLD);
  assert(GrabTask_Command(3,release)); /* Do not release while height is still moving. */
  assert(openings==opened);
  assert(GrabTask_Command(4,adjust)); /* New absolute target can supersede an unfinished height. */
  move_arrives=true;
  run_to(GRAB_HOLD,200,1);
  assert(fabsf(physical[1]+92.123f)<.011f && openings==opened);
  for(unsigned i=0;i<30;i++)step(true,1,100,30);
  assert(openings==opened && velocity_requests==velocities && base_moves==turns);
  assert(GrabTask_Command(3,release));
  assert(openings==opened+1 && fabsf(physical[1]+92.123f)<.011f);
  assert(GrabTask_Command(3,lift));run_to(GRAB_HOLD,200,1);
  assert(fabsf(physical[1])<.11f && openings==opened+1 && !tray_indexes);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  unsigned moves=positions;
  assert(GrabTask_Command(4,lower));assert(positions==moves && status().state==GRAB_IDLE);
}

static void common_static_entry_uses_packet_errors(void)
{
  char *color[]={"grab","align","color","1","0"};
  char *ring[]={"grab","align","ring","2","-60"};
  reset();configure(true);setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_car_lift","0");setting("x_max","100");
  setting("body_speed","160");setting("dx_tol","1");setting("dy_tol","1");
  pixel_center_override=true;pixel_center_x=334;pixel_center_y=338;
  assert(GrabTask_Command(5,color));run_to(GRAB_ALIGN,300,1);
  for(unsigned i=0;i<35;i++)step(true,1,40,0);
  assert(velocity_requests && status().state==GRAB_ALIGN && status().dx==40);
  assert(fabsf(align_parameter(C_BODY_SPEED)-160)<.01f);
  assert(align_parameter(C_DX_TOL)==1 && align_parameter(C_DY_TOL)==1);
  pixel_center_x=372;pixel_center_y=345;
  run_to(GRAB_HOLD,400,1);
  assert(requested_color==1 && status().dx==0 && status().dy==0);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  assert(GrabTask_Command(5,ring));run_to(GRAB_HOLD,500,1);
  assert(requested_color==11 && !closures);
  assert(fabsf(physical[1])<.11f);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
}

static void stacking_uses_already_compensated_packet_errors(void)
{
  char *stack[]={"grab","stack","align","3"};
  char *color[]={"grab","align","color","3","0"};
  reset();configure(true);setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_car_lift","0");setting("x_max","100");
  assert(GrabTask_Command(4,stack));
  assert(status().state==GRAB_PREPARE);
  /* The Pi owns compensation. Nonzero packet errors still need correction;
   * a fresh zero finishes directly, with no second STM32 alignment stage. */
  run_to(GRAB_ALIGN,300,1);
  for(unsigned i=0;i<20;i++)step(true,1,39,-1);
  assert(status().state==GRAB_ALIGN && !strcmp(status().operation,"stack_align"));
  for(unsigned i=0;i<500 && status().state!=GRAB_HOLD;i++) {
    step(true,1,0,0);
    assert(strcmp(status().reason,"stack_compensating"));
  }
  assert(status().state==GRAB_HOLD && status().stop_confirmed);
  assert(!strcmp(status().reason,"stack_aligned_ready"));
  assert(status().dx==0 && status().dy==0);
  assert(!closures && !tray_indexes);
  GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  { char *observe[]={"grab","track","observe","3"};
    assert(GrabTask_Command(4,observe));assert(status().state==GRAB_TRACK_OBSERVE);
    assert(!StackTest);
    GrabTask_Stop();run_to(GRAB_IDLE,150,1);
  }
  assert(GrabTask_Command(5,color));run_to(GRAB_HOLD,500,1);
  assert(status().dx==0 && status().dy==0 && !strcmp(status().reason,"color_aligned_ready"));
}

int main(void)
{
  char *stack[]={"grab","stack","align","3"};
  reset();configure(true);setting("z_min","-150");setting("z_max","0");
  setting("z_observe","0");setting("z_car_lift","0");setting("x_max","100");
  setting("loss_grace_ms","250");
  pixel_center_override=true;pixel_center_x=334;pixel_center_y=338;
  assert(GrabTask_Command(4,stack));run_to(GRAB_ALIGN,300,1);
  for(unsigned i=0;i<20;i++)step(true,1,-38,7);
  assert(status().dx==-38 && status().dy==7);
  /* Brief loss must recover using packet errors even with old-center CX/CY. */
  camera_valid=false;step(true,1,-38,7);
  assert(status().state==GRAB_VISION_GRACE);
  camera_valid=true;
  step(true,1,-38,7);step(true,1,-38,7);step(true,1,-38,7);
  assert(status().state==GRAB_ALIGN && status().dx==-38 && status().dy==7);
  assert(!strcmp(status().reason,"vision_gap_recovered"));
  /* Stop/reacquire keeps the same source, including a real zero. */
  camera_valid=false;
  for(unsigned i=0;i<100 && !RecoveryWaiting;i++)step(true,1,-38,7);
  assert(status().state==GRAB_VISION_PAUSE && RecoveryWaiting && status().stop_confirmed);
  camera_valid=true;
  /* New camera reference, independent of the old MCU static ref_u/ref_v. */
  pixel_center_x=372;pixel_center_y=345;
  run_to(GRAB_ALIGN,30,1);
  assert(status().dx==0 && status().dy==0 && RecoveryUsed==1);
  run_to(GRAB_HOLD,500,1);
  assert(status().state==GRAB_HOLD && status().dx==0 && status().dy==0);
  assert(status().stop_confirmed && !closures);
  stack_manual_height_and_release();
  common_static_entry_uses_packet_errors();
  stacking_uses_already_compensated_packet_errors();
  puts("Static alignment / manual stack height / separate release: PASS");
  return 0;
}
