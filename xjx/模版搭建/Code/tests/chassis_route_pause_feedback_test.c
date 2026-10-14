#define main legacy_stop_main
#define ChassisRoute_IsBusy Fixture_RouteBusy
#define ChassisRoute_IsSubmitting Fixture_RouteSubmitting
#include "chassis_motion_test.c"
#undef ChassisRoute_IsBusy
#undef ChassisRoute_IsSubmitting
#undef main
#include "chassis_route.h"
#include "GrabTask.h"
bool ContestMission_HasPendingWork(void){return false;}
bool GrabTask_PoseClearanceReady(void){return false;}
bool GrabTask_QrPoseStart(void){return true;}
void GrabTask_StatusGet(GrabTask_Status_t *s){memset(s,0,sizeof *s);s->state=GRAB_COMPLETE;s->result="qr_pose_ready";}
bool Mecanum_Feedback_Select(uint8_t n){assert(!n);return true;}
void Mecanum_Feedback_Enable(bool enabled){(void)enabled;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){(void)d;(void)n;return true;}
static ChassisRoute_Status_t route_status(void){ChassisRoute_Status_t s;ChassisRoute_StatusGet(&s);return s;}
static void stop_tx(uint32_t now){tick=now;bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;bus.tx_complete=true;bus.sent_ms=now;ChassisMotion_Process();}
static void stop_sample(uint32_t now){tick=now;for(unsigned i=0;i<4;i++)location.wheels[i].position_ms=location.wheels[i].speed_ms=now;ChassisMotion_Process();}
int main(void)
{
 reset();ChassisRoute_Init();valid=true;reading(100,0);ChassisMotion_HeadingProcess();
 assert(ChassisMotion_AnchorSet(CHASSIS_MODEL_PI));
 location.origin_valid=location.feedback_valid=location.speed_valid=true;location.feedback_tick=tick;
 location.origin=(ChassisModel_Pose_t){2250,150,CHASSIS_MODEL_PI};location.observer.feedback=location.origin;
 location.units_per_rev=65536;
 for(unsigned i=0;i<4;i++){
  location.wheels[i].position_valid=location.wheels[i].speed_valid=location.wheels[i].state_valid=true;
  location.wheels[i].position_ms=location.wheels[i].speed_ms=location.wheels[i].state_ms=tick;location.wheels[i].state_flags=3;
 }
 assert(ChassisRoute_TaskStart(NULL,200));assert(ChassisRoute_TaskPause());
 stop_tx(110);assert(!status().stop_confirmed);ChassisRoute_Process();
 assert(strcmp(route_status().state,"paused"));
 stop_tx(120);stop_sample(130);ChassisRoute_Process();assert(!route_status().stop_confirmed);
 stop_sample(380);ChassisRoute_Process();assert(!route_status().stop_confirmed);
 stop_sample(630);ChassisRoute_Process();
 assert(!status().stop_confirmed && route_status().stop_confirmed && !strcmp(route_status().state,"paused"));
 puts("production route/motion pause: fresh stop monitor confirms pause without arrival bit PASS");
}
