#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void cmd(unsigned count,char **tokens)
{ output[0]=0;assert(GrabTask_Command(count,tokens));if(strstr(output,"ERR"))fprintf(stderr,"%s",output);assert(!strstr(output,"ERR")); }
static void setup(void)
{
  reset();configure(true);
  setting("x_min","0");setting("x_max","100");
  setting("z_min","-150");setting("z_max","0");
  setting("z_grab","-50");setting("z_lift","0");
  setting("z_car_lift","0");
  physical[0]=30;physical[1]=0;physical[2]=37+GRAB_BASE_WORK_OFFSET_DEG;
  RearBase=37;RearReferenceValid=true;OutsidePose=true;
}
static void three_pieces(void)
{
  setup();
  for(unsigned slot=1;slot<=3;slot++)
  {
    char number[2]={(char)('0'+slot),0};
    char *pick[]={"grab","track","pickstore",number,number,"feedback"};
    unsigned old_closures=closures,old_indexes=tray_indexes,old_base=base_moves;
    if(slot==1)assert(GrabTask_TrackedStoreStart(1,1,GRAB_TRACK_FEEDBACK));else cmd(6,pick);run_to(GRAB_CLOSE,200,0);
    assert(closures==old_closures+1 && tray_inventory[slot-1].state==TURNTABLE_EMPTY);
    assert(tray_indexes==old_indexes+1); /* Index overlaps tracking, before lift. */
    assert(fabsf(physical[1]+50)<.01f);
    run_to(GRAB_INDEX,300,0);
    assert(base_moves==old_base && !strcmp(status().result,"pending"));
    run_to(GRAB_PLACE,300,0);
    assert(tray_indexes==old_indexes+1 && fabsf(physical[2]-37)<.15f);
    assert(fabsf(physical[0])<.01f);
    linear_state_flags[1]=1; /* Z is at target but driver reached never asserts. */
    run_to(GRAB_RELEASE,300,0);
    linear_state_flags[1]=3;
    assert(fabsf(physical[1]+40)<.01f);
    run_to(GRAB_COMPLETE,400,0);
    assert(!strcmp(status().result,"stored") && status().slot==slot);
    assert(tray_inventory[slot-1].state==TURNTABLE_OCCUPIED && tray_inventory[slot-1].color==slot);
    assert(fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f && physical[1]==0 && homes==0);
  }
}
static void cancel_and_resume(void)
{
  setup();char *pick[]={"grab","track","pickstore","3","2","visual"};
  cmd(6,pick);tray_arrives=false;run_to(GRAB_INDEX,300,0);
  for(unsigned i=0;i<10;i++)step(true,0,0,0);
  unsigned old=closures,opened=openings;GrabTask_Stop();run_to(GRAB_IDLE,200,0);
  assert(closures==old && openings==opened && tray_inventory[1].state==TURNTABLE_EMPTY);
  tray_arrives=true;char *resume[]={"grab","track","store","2"};cmd(4,resume);
  run_to(GRAB_COMPLETE,600,0);
  assert(closures==old && !strcmp(status().result,"stored"));
  assert(tray_inventory[1].state==TURNTABLE_OCCUPIED);
}
static void stopped_transfer_stages(void)
{
  const GrabTask_State_t stages[]={GRAB_PLACE,GRAB_RELEASE,GRAB_RETURN_TURN};
  for(unsigned i=0;i<3;i++)
  {
    setup();char *pick[]={"grab","track","pickstore","2","3","feedback"};cmd(6,pick);
    run_to(stages[i],500,0);
    unsigned closed=closures,opened=openings;GrabTask_Stop();run_to(GRAB_IDLE,200,0);
    /* The context, not a new pickup or edited drafts, selects the continuation. */
    setting("z_lift","-10");setting("track_store_drop_mm","20");
    char *wrong[]={"grab","track","store","1"};output[0]=0;assert(GrabTask_Command(4,wrong));assert(!strcmp(status().reason,"track_store_slot_mismatch") && !GrabTask_IsBusy());
    if(i<2){char *restart[]={"grab","track","pickstore","1","1","feedback"};output[0]=0;assert(GrabTask_Command(6,restart));assert(strstr(output,"ERR"));}
    char *resume[]={"grab","track","store","0"};cmd(4,resume);run_to(GRAB_COMPLETE,600,0);
    assert(closures==closed && openings==opened+(i==0));
    assert(!strcmp(status().result,"stored") && status().slot==3);
    assert(tray_inventory[2].state==TURNTABLE_OCCUPIED && tray_inventory[2].color==2);
    assert(fabsf(TrackStorePlaceTarget+40)<.01f && TrackStoreLiftTarget==0);
    assert(fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f && !homes);
  }
}
static void pick_only_then_store(void)
{
  setup();char *pick[]={"grab","track","pick","3","-50","110"};cmd(6,pick);
  run_to(GRAB_COMPLETE,500,0);assert(!strcmp(status().result,"picked") && TrackHeld && !tray_indexes);
  for(unsigned i=0;i<3;i++)step(true,0,0,0);
  tray_inventory[0].state=TURNTABLE_UNKNOWN;
  unsigned closed=closures;char *store[]={"grab","track","store","1"};cmd(4,store);
  run_to(GRAB_COMPLETE,600,0);assert(!strcmp(status().result,"stored") && closures==closed);
}
static void explicit_tray_does_not_require_inventory_confirmation(void)
{
  for(unsigned slot=1;slot<=3;slot++)
  {
    setup();
    for(unsigned i=0;i<3;i++)tray_inventory[i]=(Turntable_Inventory_t){TURNTABLE_UNKNOWN,0};
    Turntable_InventoryState_t previous=slot==2?TURNTABLE_OCCUPIED:TURNTABLE_UNKNOWN;
    tray_inventory[slot-1]=(Turntable_Inventory_t){previous,slot==2?4:0};
    char number[2]={(char)('0'+slot),0};char *pick[]={"grab","track","pickstore","2",number,"feedback"};
    if(slot==3)assert(GrabTask_TrackedStoreStart(2,3,GRAB_TRACK_FEEDBACK));else cmd(6,pick);
    assert(tray_inventory[slot-1].state==previous); /* No speculative ledger clearing. */
    run_to(GRAB_COMPLETE,600,0);
    assert(status().slot==slot && !strcmp(status().result,"stored"));
    assert(tray_inventory[slot-1].state==TURNTABLE_OCCUPIED && tray_inventory[slot-1].color==2);
    for(unsigned i=0;i<3;i++)if(i!=slot-1)assert(tray_inventory[i].state==TURNTABLE_UNKNOWN);
  }
  setup();for(unsigned i=0;i<3;i++)tray_inventory[i].state=TURNTABLE_UNKNOWN;
  char *automatic[]={"grab","track","pickstore","2","0","feedback"};
  output[0]=0;assert(GrabTask_Command(6,automatic));assert(strstr(output,"no_confirmed_empty_slot"));
  assert(!GrabTask_IsBusy() && !positions && !closures && !tray_indexes);
  tray_inventory[1].state=TURNTABLE_EMPTY;cmd(6,automatic);run_to(GRAB_COMPLETE,600,0);
  assert(status().slot==2 && tray_inventory[0].state==TURNTABLE_UNKNOWN && tray_inventory[2].state==TURNTABLE_UNKNOWN);
}
static void next_color_after_released_stop(void)
{
  const GrabTask_State_t stages[]={GRAB_RETRACT,GRAB_RETURN_TURN,GRAB_OBSERVE_Z,GRAB_OBSERVE_X};
  for(unsigned i=0;i<4;i++)
  {
    setup();char *yellow[]={"grab","track","pickstore","2","1","feedback"};cmd(6,yellow);
    run_to(stages[i],600,0);GrabTask_Stop();run_to(GRAB_IDLE,200,0);
    unsigned closed=closures,indexed=tray_indexes;
    assert(tray_inventory[0].state==TURNTABLE_OCCUPIED && tray_inventory[0].color==2);
    char *green[]={"grab","track","pickstore","4","2","feedback"};
    if(i==3)assert(GrabTask_TrackedStoreStart(4,2,GRAB_TRACK_FEEDBACK));else cmd(6,green);
    assert(status().state==GRAB_ROUTE_POSE && closures==closed && tray_indexes==indexed);
    if(i==0)
    {
      GrabTask_Stop();run_to(GRAB_IDLE,200,0);
      for(unsigned j=0;j<30;j++)step(true,0,0,0);
      assert(closures==closed && !GrabTask_IsBusy());cmd(6,green);
    }
    if(i==1)
    {
      fault("feedback_stale");run_to(GRAB_ERROR,200,0);
      assert(!NextTrack.pending && closures==closed);
      cmd(6,green);
    }
    run_to(GRAB_TRACK_OBSERVE,300,0);
    assert(closures==closed && tray_indexes==indexed && fabsf(physical[2]-(RearBase+GRAB_BASE_WORK_OFFSET_DEG))<.15f);
    run_to(GRAB_COMPLETE,600,0);
    assert(closures==closed+1 && tray_indexes==indexed+1);
    assert(tray_inventory[0].color==2 && tray_inventory[1].color==4 && !strcmp(status().result,"stored"));
  }
}
static void observation_boundary_return(void)
{
  setup();
  assert(observation_target(0,-1)==-1 && observation_target(1,1)==1);
  assert(observation_target(0,50)==50 && isnan(observation_target(0,NAN)));
  unsigned sent=positions;
  assert(!axis_move(0,-0.009f) && positions==sent && !strcmp(Status.reason,"axis_limit"));

  for(unsigned axis=0;axis<2;axis++)
  {
    setup();physical[axis]=axis?0.009f:-0.009f;
    assert(GrabTask_TrackedStoreStart(1,1,GRAB_TRACK_FEEDBACK));
    run_to(GRAB_CLOSE,300,0);
    assert(axis?ObserveZ==0:ObserveX==0);
    run_to(GRAB_COMPLETE,600,0);
    assert(!strcmp(status().result,"stored") && tray_inventory[0].state==TURNTABLE_OCCUPIED);
    assert(GrabTask_TrackedStoreStart(2,2,GRAB_TRACK_FEEDBACK));
    run_to(GRAB_COMPLETE,600,0);
    assert(!strcmp(status().result,"stored") && tray_inventory[1].color==2);
  }
}
static void early_index_pause_and_fresh_verification(void)
{
  setup();tray_arrives=false;
  assert(GrabTask_TrackedStoreStart(2,2,GRAB_TRACK_FEEDBACK));
  for(unsigned i=0;i<80 && !tray_indexes;i++)step(true,0,30,0);
  assert(tray_indexes==1 && !closures && !base_moves);
  assert(GrabTask_Pause());run_to(GRAB_IDLE,200,0);
  unsigned indexes=tray_indexes;
  for(unsigned i=0;i<20;i++)step(true,0,0,0);
  assert(!tray_moving && tray_indexes==indexes && tray_stops);
  assert(GrabTask_Continue());run_to(GRAB_INDEX,600,0);
  assert(tray_indexes==indexes+1 && closures==1 && !base_moves);
  for(unsigned i=0;i<10;i++)step(true,0,0,0);
  assert(!base_moves); /* A slow early index still gates Base's return. */
  tray_arrives=true;step(true,0,0,0);run_to(GRAB_COMPLETE,600,0);
  assert(closures==1 && tray_inventory[1].color==2);

  setup();assert(GrabTask_TrackedStoreStart(1,1,GRAB_TRACK_FEEDBACK));
  run_to(GRAB_INDEX,600,0);assert(tray_indexes==1);
  tray_status.feedback_valid=false;run_to(GRAB_PLACE,200,0);
  assert(tray_verifies==1 && tray_indexes==1); /* No second HOME/move. */
  run_to(GRAB_COMPLETE,400,0);
}
int main(void)
{
  early_index_pause_and_fresh_verification();
  observation_boundary_return();
  next_color_after_released_stop();
  explicit_tray_does_not_require_inventory_confirmation();
  three_pieces();cancel_and_resume();stopped_transfer_stages();pick_only_then_store();
  setup();setting("track_store_drop_mm","200");
  char *bad[]={"grab","track","pickstore","3","1","feedback"};
  output[0]=0;assert(GrabTask_Command(6,bad));assert(strstr(output,"ERR"));
  assert(!closures && !positions && !tray_indexes);
  puts("tracking store: three slots, index before Base, complete inventory/return, cancel/resume and limits PASS");
}
