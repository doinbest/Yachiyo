#define main legacy_cancel_main
#include "contest_mission_test.c"
#undef main
static void setup_cancel(void)
{
 memset(&grab,0,sizeof grab);memset(&route,0,sizeof route);memset(&motion,0,sizeof motion);
 grab.reason="idle";grab.result="none";route.state=route.phase=route.reason="idle";
 start_fixture_custom=false;starts=0;QR_Init();ContestMission_Init();
 cancel_defer=false;
 assert(ContestMission_Start(NULL,&request));step();assert(starts==1);
}
int main(void)
{
 setup_cancel();inventory[0]=(Turntable_Inventory_t){TURNTABLE_OCCUPIED,4};
 ContestMission_Stop();step();ContestMission_Report();assert(strstr(report,"state=paused"));
 assert(ContestMission_HasPendingWork());assert(ContestMission_Continue());step();
 ContestMission_Cancel();ContestMission_Cancel();step();step();ContestMission_Report();
 assert(strstr(report,"state=cancelled") && strstr(report,"reason=operator_cancelled"));
 assert(!ContestMission_HasPendingWork() && !ContestMission_Continue());
 assert(inventory[0].state==TURNTABLE_OCCUPIED && inventory[0].color==4);
 route.state="idle";grab.state=GRAB_IDLE;starts=0;assert(ContestMission_Start(NULL,&request));
 ContestMission_Cancel();step();step();assert(!ContestMission_HasPendingWork() && !starts);
 setup_cancel();cancel_defer=true;ContestMission_Cancel();ContestMission_Process();ContestMission_Report();
 assert(strstr(report,"state=cancelling") && !strstr(report,"stop_confirmed=1"));
 assert(ContestMission_HasPendingWork() && !ContestMission_Continue());
 motion.state=CHASSIS_MOTION_ERROR;motion.stop_confirmed=false;cancel_stop_pending=false;cancel_stop.reason="timeout";
 ContestMission_Process();ContestMission_Report();
 assert(strstr(report,"state=cancel_wait") && !ContestMission_Continue());
 ContestMission_Stop();ContestMission_Report();assert(strstr(report,"state=cancel_wait"));
 cancel_defer=false;ContestMission_Cancel();step();step();ContestMission_Report();
 assert(strstr(report,"state=cancelled") && !ContestMission_HasPendingWork());
 setup_cancel();stop_request_tick_advance=true;ContestMission_Cancel();step();step();ContestMission_Report();
 assert(strstr(report,"state=cancelled") && !ContestMission_HasPendingWork());
 setup_cancel();grab.state=GRAB_PREPARE;grab.operation="store";grab.slot=1;
 assert(GrabTask_IsBusy());ContestMission_Stop();step();
 unsigned resumed=grab_resume_calls;route_resume_delayed=true;assert(ContestMission_Continue());
 for(unsigned i=0;i<10;i++)step();
 assert(grab_resume_calls==resumed && ContestMission_HasPendingWork());
 route_resume_delayed=false;step();assert(grab_resume_calls==resumed+1);
 puts("contest_mission_cancel_test: PASS (pause resumes, cancel ends, inventory preserved, preparation abort)");
}
