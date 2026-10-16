#define main legacy_grab_test_main
#include "grab_task_test.c"
#undef main
int main(void)
{
  reset();configure(false);bus.ack_profile=MECANUM_ACK_NONE;
  assert(GrabTask_Start("fixed"));run_to(GRAB_DESCEND,500,0);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);assert(Status.stop_confirmed);

  reset();configure(true);bus.ack_profile=MECANUM_ACK_NONE;
  setting("x_min","0");setting("x_max","100");physical[0]=35;physical[1]=10;
  char *pick[]={"grab","track","pick","3","-8","110"};
  output[0]=0;assert(GrabTask_Command(6,pick));assert(!strstr(output,"ERR"));
  run_to(GRAB_DESCEND,200,0);
  GrabTask_Stop();run_to(GRAB_IDLE,200,0);assert(Status.stop_confirmed);

  reset();configure(false);bus.ack_profile=MECANUM_ACK_NONE;bus.locked=true;
  assert(!GrabTask_Start("fixed"));assert(!strcmp(Status.reason,"bus_unavailable"));
  bus.locked=false;bus.ack_profile=MECANUM_ACK_UNKNOWN;
  assert(!GrabTask_Start("fixed"));assert(!strcmp(Status.reason,"bus_unavailable"));
  puts("grab_none_profile_test: fixed task, tracking, actual stop and locked/unknown veto PASS");
}
