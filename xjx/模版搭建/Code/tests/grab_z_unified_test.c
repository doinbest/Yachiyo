#define main legacy_grab_main
#include "grab_task_test.c"
#undef main

static void all_speeds(float speed)
{
 assert(Config[C_Z_SPEED]==speed && Config[C_Z_PICK_SPEED]==speed && Config[C_QR_Z_SPEED]==speed);
}
int main(void)
{
 reset();all_speeds(250);
 setting("z_speed","80");all_speeds(80);
 setting("z_pick_speed","150");all_speeds(150);
 setting("qr_z_speed","250");all_speeds(250);
 configure(false);setting("z_ppm","480");setting("z_min","-150");setting("z_max","0");
 setting("z_speed","250");
 const GrabTask_State_t phases[]={GRAB_ROUTE_POSE,GRAB_DESCEND,GRAB_LIFT,GRAB_PLACE,GRAB_QR_SCAN};
 for(unsigned i=0;i<sizeof phases/sizeof *phases;i++){
   pending_valid=false;PendingAction=MECHANICAL_ARM_ACTION_NONE;
   Status.state=phases[i];Operation=OP_STORE;RingPlaceActive=false;
   assert(axis_move(1,-10));assert(z_position_rpm==2250 && z_position_acc==255);
 }
 reset();configure(true);setting("x_min","0");setting("x_max","100");physical[0]=35;
 char *pick[]={"grab","track","pick","3","-8","110"};
 assert(GrabTask_Command(6,pick));all_speeds(110); /* An explicit manual speed is shared too. */
 puts("grab_z_unified_test: default250, three aliases, scan/pick/lift/place, explicit pickup PASS");
}
