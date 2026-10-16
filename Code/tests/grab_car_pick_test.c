#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void setup(void)
{
  reset();configure(true);setting("x_min","0");setting("x_max","100");
  setting("z_min","-150");setting("z_max","0");setting("z_car_lift","0");
  physical[0]=0;physical[1]=0;physical[2]=37+GRAB_BASE_WORK_OFFSET_DEG;
  RearBase=37;RearReferenceValid=true;
  for(unsigned i=0;i<3;i++)tray_inventory[i]=(Turntable_Inventory_t){TURNTABLE_OCCUPIED,(uint8_t)(i+1)};
}
static void start(unsigned slot,const char *distance)
{
  char number[2]={(char)('0'+slot),0};char *cmd[]={"grab","take",number,"work",(char *)distance};
  output[0]=0;assert(GrabTask_Command(distance?5:4,cmd));assert(GrabTask_IsBusy());
}
static void three_slots(void)
{
  for(unsigned slot=1;slot<=3;slot++)
  {
    setup();Config[C_Z_GRAB]=Config[C_Z_LIFT]=Config[C_Z_PLACE]=Config[C_BASE_CAR_OFFSET]=NAN;
    start(slot,NULL);tray_arrives=false;run_to(GRAB_INDEX,200,0);
    for(unsigned i=0;i<10;i++)step(true,0,0,0);
    assert(!base_moves && !closures && !openings && !grip_custom_calls && physical[1]==0);
    tray_arrives=true;run_to(GRAB_TURN,200,0);
    while(strcmp(status().reason,"car_pick_wait")){step(true,0,0,0);assert(physical[0]==0);}
    assert(grip_custom_calls==1 && fabsf(grip_custom_duty-10.6f)<.001f && !closures);
    uint32_t ready=CarPickWaitTick;unsigned moves=positions;base_arrives=false;
    for(unsigned i=0;i<5;i++)step(true,0,0,0);
    assert(clock_ms-ready>=250 && positions==moves && !closures && !openings);
    base_arrives=true;physical[2]=destination[2];
    run_to(GRAB_CLOSE,100,0);
    assert(CarPickWaitTick==ready && clock_ms-ready>=100 && fabsf(physical[1]+25)<.01f && closures==1 && !openings && physical[0]==0);
    assert(tray_inventory[slot-1].state==TURNTABLE_OCCUPIED);
    run_to(GRAB_COMPLETE,300,0);
    assert(!strcmp(status().operation,"car_pick") && !strcmp(status().result,"picked"));
    assert(tray_inventory[slot-1].state==TURNTABLE_EMPTY && target_color()==slot);
    assert(fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f && physical[1]==0 && physical[0]==(slot<=2?30:0));
    assert(!openings && closures==1 && grip_custom_calls==1 && !homes && !camera_requests && !GrabTask_IsBusy());
  }
}
static void stop_and_continue(void)
{
  setup();start(2,"60");run_to(GRAB_CLOSE,400,0);
  unsigned closed=closures;GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  assert(!openings && tray_inventory[1].state==TURNTABLE_UNKNOWN);
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(status().state==GRAB_IDLE && closures==closed);
  start(2,"25");run_to(GRAB_COMPLETE,400,0);
  assert(closures==closed && !openings && physical[1]==0 && fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f);
  assert(tray_inventory[1].state==TURNTABLE_EMPTY);
}
static void stop_before_close(void)
{
  const GrabTask_State_t stages[]={GRAB_INDEX,GRAB_TURN,GRAB_PLACE};
  for(unsigned i=0;i<3;i++)
  {
    setup();start(1,NULL);run_to(stages[i],400,0);
    if(i==1)while(strcmp(status().reason,"car_pick_wait"))step(true,0,0,0);
    if(i==2){move_arrives=false;step(true,0,0,0);step(true,0,0,0);}
    GrabTask_Stop();run_to(GRAB_IDLE,200,0);
    assert(!closures && !openings);unsigned before=positions;
    for(unsigned n=0;n<20;n++)step(true,0,0,0);
    assert(positions==before && status().state==GRAB_IDLE);
    move_arrives=true;start(1,"20");run_to(GRAB_CLOSE,400,0);
    assert(fabsf(physical[1]+25)<.01f); /* Preserve this pickup's original depth. */
    run_to(GRAB_COMPLETE,300,0);assert(closures==1 && !openings && tray_inventory[0].state==TURNTABLE_EMPTY);
  }
}
static void reject_before_motion(void)
{
  char *cmd[]={"grab","take","2","work","200"};
  setup();assert(GrabTask_Command(5,cmd));assert(!GrabTask_IsBusy() && !positions && !closures);
  cmd[4]="40";
  setup();RearReferenceValid=false;assert(GrabTask_Command(5,cmd));assert(!GrabTask_IsBusy() && !positions);
}
static void manual_selection_needs_no_stock_or_color(void)
{
  const Turntable_InventoryState_t states[]={TURNTABLE_UNKNOWN,TURNTABLE_EMPTY};
  for(unsigned i=0;i<2;i++)for(unsigned slot=1;slot<=3;slot++)
  {
    setup();tray_inventory[slot-1]=(Turntable_Inventory_t){states[i],0};
    start(slot,NULL);run_to(GRAB_COMPLETE,500,0);
    assert(!strcmp(status().result,"picked") && status().slot==slot);
    assert(target_color()==0); /* Unknown is not silently recorded as red. */
    assert(!openings && closures==1 && physical[1]==0 && fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f);
    assert(tray_inventory[slot-1].state==TURNTABLE_EMPTY);
  }
  char *take[]={"grab","take","2","rough"};
  setup();tray_inventory[1]=(Turntable_Inventory_t){TURNTABLE_UNKNOWN,0};
  start(2,NULL);run_to(GRAB_CLOSE,400,0);unsigned closed=closures;
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);start(2,NULL);run_to(GRAB_COMPLETE,400,0);
  assert(target_color()==0 && closures==closed && !openings);
  setup();setting("z_proc_place","-40");tray_inventory[1]=(Turntable_Inventory_t){TURNTABLE_UNKNOWN,0};
  assert(GrabTask_Command(4,take));assert(!GrabTask_IsBusy());
  assert(!strcmp(status().reason,"inventory_unconfirmed"));
}
static void pickup_clearance_precedes_base_turn(void)
{
  setup();start(1,NULL);run_to(GRAB_CLOSE,400,0);
  assert(physical[0]==0);run_to(GRAB_LIFT,100,0);
  move_arrives=false;unsigned turns=base_moves;
  for(unsigned i=0;i<10;i++)step(true,0,0,0);
  assert(base_moves==turns && status().state==GRAB_LIFT);
  assert(Axes[0].commanded && Axes[1].commanded);
  assert(Axes[0].target==30 && Axes[1].target==0);
  /* Releasing the held simulated drives completes their already accepted
   * targets; no second position command is needed by the real motors. */
  move_arrives=true;physical[1]=destination[1];step(true,0,0,0);
  assert(base_moves==turns && status().state==GRAB_LIFT);
  physical[0]=destination[0];run_to(GRAB_RETURN_TURN,300,0);
  assert(physical[0]==30 && physical[1]==0);
  run_to(GRAB_COMPLETE,300,0);assert(physical[0]==30 && !openings);
}
static void clearance_stop_preserves_absolute_target(void)
{
  setup();start(2,NULL);run_to(GRAB_LIFT,400,0);
  move_arrives=false;for(unsigned i=0;i<8;i++)step(true,0,0,0);
  physical[0]=15;physical[1]=-20;unsigned closed=closures;
  GrabTask_Stop();move_arrives=true;run_to(GRAB_IDLE,200,0);
  start(2,NULL);run_to(GRAB_COMPLETE,500,0);
  assert(physical[0]==30 && physical[1]==0 && closures==closed && !openings);
}
static void insufficient_clearance_rejects_before_motion(void)
{
  setup();setting("x_max","20");char *cmd[]={"grab","take","1","work"};
  assert(GrabTask_Command(4,cmd));assert(!GrabTask_IsBusy() && !positions && !closures);
  assert(!strcmp(status().reason,"car_pick_clearance_limits"));
}
static void cancel_ends_pickup_ownership(void)
{
  setup();start(1,NULL);run_to(GRAB_CLOSE,400,0);unsigned closed=closures;
  GrabTask_Cancel();run_to(GRAB_IDLE,200,0);
  assert(!CarPickActive && !CarPickClosed && !CarPickWaiting && TrackHeld);
  assert(!openings && closures==closed);
  /* A later explicit manual request is a new pickup, not the cancelled owner. */
  assert(GrabTask_CarPickStart(3));assert(CarPickSlot==3 && !CarPickClosed && CarPickClearanceX==0);
}
static void standalone_ring_lifts_before_turn(unsigned slot,bool pause)
{
  setup();assert(GrabTask_RingCarPickStart((uint8_t)slot,25));
  run_to(GRAB_CLOSE,600,0);run_to(GRAB_LIFT,100,0);
  unsigned turns=base_moves,closed=closures;z_arrives=false;
  for(unsigned i=0;i<30;i++)step(true,0,0,0);
  assert(Status.state==GRAB_LIFT && base_moves==turns && physical[1]==-25);
  assert(physical[0]==(slot<=2?30:0));
  if(pause){
    assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
    Config[C_CAR_PICK_DROP_MM]=60;assert(GrabTask_Continue());
    run_to(GRAB_LIFT,300,0);
    for(unsigned i=0;i<15;i++)step(true,0,0,0);
    assert(base_moves==turns && CarPickTarget==-25 && closures==closed);
  }
  z_arrives=true;physical[1]=destination[1];run_to(GRAB_COMPLETE,400,0);
  assert(physical[1]==0 && base_moves==turns+1 && closures==closed);
}
static void standalone_ring_prepare_lift_and_retract_overlap(void)
{
  setup();physical[0]=42;physical[1]=-10;
  assert(GrabTask_RingCarPickStart(1,NAN));run_to(GRAB_RETRACT,400,0);
  z_arrives=false;unsigned turns=base_moves;
  for(unsigned i=0;i<30;i++)step(true,0,0,0);
  assert(Axes[0].commanded && Axes[1].commanded && physical[0]==0 && physical[1]==-10);
  assert(Status.state==GRAB_RETRACT && base_moves==turns && !closures);
  z_arrives=true;physical[1]=destination[1];run_to(GRAB_COMPLETE,600,0);
  assert(physical[0]==30 && physical[1]==0 && closures==1);
}
int main(void)
{
  three_slots();stop_and_continue();stop_before_close();reject_before_motion();manual_selection_needs_no_stock_or_color();
  pickup_clearance_precedes_base_turn();clearance_stop_preserves_absolute_target();insufficient_clearance_rejects_before_motion();
  cancel_ends_pickup_ownership();
  standalone_ring_prepare_lift_and_retract_overlap();
  for(unsigned slot=1;slot<=3;slot++)standalone_ring_lifts_before_turn(slot,slot==1);
  puts("car pickup: ordinary X30/Base overlap; standalone ring parallel preparation, lifted Z before turn, three trays and pause/resume PASS");
}
