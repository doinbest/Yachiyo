/* Real Mecanum + MotorBus + Motion. Prepared without compiling/running C.
 * The preceding fixture exercises the real committed-zero ownership latch. */
#define main legacy_mecanum_main
#define ChassisMotion_IsBusy FixtureMotionBusy
#define HWT101_Cal_ControlAngleGet FixtureAngleGet
#include "mecanum_bus_test.c"
#undef HWT101_Cal_ControlAngleGet
#undef ChassisMotion_IsBusy
#undef main
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "GrabTask.h"
static ChassisLocalization_Status_t position;
static bool producer_busy,pose_clearance,qr_departure,arm_occupied,imu_ready=true;
bool ChassisRoute_IsSubmitting(void){return false;}
bool GrabTask_IsBusy(void){return producer_busy;}
bool GrabTask_PoseClearanceReady(void){return pose_clearance;}
bool GrabTask_QrDepartureActive(void){return qr_departure;}
bool GrabTask_CalibrationActive(void){return false;}
uint8_t MechanicalArm_IsBusy(void){return arm_occupied;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *s){*s=position;}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *a)
{memset(a,0,sizeof *a);a->last_update_ms=tick;a->update_count=tick+1;return imu_ready;}
static ChassisMotion_Status_t motion_status(void){ChassisMotion_Status_t s;ChassisMotion_StatusGet(&s);return s;}
static void fresh_position(void)
{
 position.feedback_tick=tick;
 for(unsigned i=0;i<4;i++){
  Mecanum_Feedback_t *w=&position.wheels[i];memset(w,0,sizeof *w);
  w->speed_valid=w->position_valid=w->state_valid=true;w->state_flags=3;
  w->speed_ms=w->position_ms=w->state_ms=tick;
 }
}
static void setup(void)
{
 auto_ack=true;tx_result=HAL_OK;
 if(tick<100)tick=100;
 assert(MotorBus_Init(&uart)==HAL_OK);assert(Mecanum_Test_Stop());advance(80);
 assert(Mecanum_RecoveryAfterReset());count=0;
 producer_busy=pose_clearance=qr_departure=arm_occupied=false;imu_ready=true;
 MotorBus_Init(&uart);ChassisMotion_Init();
 memset(&position,0,sizeof position);
 assert(ChassisMotion_AnchorSet(0));
 position.origin_valid=position.feedback_valid=position.speed_valid=true;
 position.units_per_rev=65536;position.generation=1;
 position.origin=position.observer.feedback=(ChassisModel_Pose_t){1200,2100,0};
 assert(Mecanum_Velocity_Request(0,0,0,20));advance(80);fresh_position();
 assert(Mecanum_IsBusy() && Mecanum_VelocityHandoffReady());
}
static bool start_kind(unsigned kind)
{
 if(kind==0)return ChassisMotion_MoveTo(1200,2100,0,50,10000);
 if(kind==1)return ChassisMotion_TurnToSpeed(90,50,10000);
 ChassisPath path={0};path.count=2;path.length=100;path.turn_end=50;
 path.point[0]=(ChassisPath_Point){1200,2100,0,0,100};
 path.point[1]=(ChassisPath_Point){1200,2000,0,100,100};
 return ChassisMotion_PathStart(&path,50,50,10000,0);
}
int main(void)
{
 /* Retired raw task -> local return, turn or next path. No FE, no fake arrival,
  * and the explicit-stop latch is retained for IMU/manual-mode ownership. */
 for(unsigned kind=0;kind<3;kind++){
  setup();unsigned before=count;
  assert(start_kind(kind));assert(ChassisMotion_IsBusy());
  assert(!motion_status().stop_confirmed && Mecanum_IsBusy() && count==before);
  assert(motion_status().action_id==1 && motion_status().start_tick==tick);
 }
 /* Base-only posture travel is allowed after X/Z clearance, never active
  * tracking or a turn concurrent with the Base preparation. */
 setup();producer_busy=true;assert(!start_kind(0));assert(!ChassisMotion_IsBusy());
 pose_clearance=true;assert(start_kind(0));
 setup();producer_busy=pose_clearance=true;assert(!start_kind(1));
 setup();producer_busy=qr_departure=true;assert(start_kind(0));
 for(unsigned bad=0;bad<13;bad++){
  setup();
  switch(bad){
   case 0:assert(Mecanum_Velocity_Request(20,0,0,20));break;
   case 1:assert(Mecanum_Test_Stop());break;
   case 2:position.feedback_valid=false;break;
   case 3:position.wheels[2].speed_rpm=2;break;
   case 4:position.wheels[1].state_valid=false;break;
   case 5:position.wheels[3].state_flags=0x05;break;
   case 6:position.wheels[3].state_flags=0;break;
   case 7:position.wheels[0].state_ms=tick-601U;break;
   case 8:position.wheels[1].position_ms=tick-601U;break;
   case 9:position.wheels[2].speed_ms=tick-601U;break;
   case 10:imu_ready=false;break;
   case 11:arm_occupied=true;break;
   case 12:tx_result=HAL_ERROR;assert(Mecanum_Velocity_Request(20,0,0,20));advance(80);break;
  }
  unsigned before=count;
  assert(!start_kind(0));assert(!ChassisMotion_IsBusy());
  assert(motion_status().action_id==0 && count==before && Mecanum_IsBusy());
 }
 setup();assert(Mecanum_Velocity_Request(20,0,0,20));
 assert(Mecanum_Velocity_Request(0,0,0,20));
 assert(!start_kind(0) && !strcmp(motion_status().reason,"chassis_handoff_wait"));
 advance(80);fresh_position();assert(start_kind(0)); /* Pending zero must really drain. */
 setup();ChassisMotion_Target_t t;ChassisMotion_TargetDefaults(&t);
 assert(!ChassisMotion_Start(&t));assert(!ChassisMotion_VelocityStart(20,0,0));
 assert(!ChassisMotion_HoldStart(20,1000)); /* Fresh-map handoff is scoped. */
 setup();producer_busy=true;
 for(unsigned i=0;i<10;i++)assert(!start_kind(0));
 producer_busy=false;assert(start_kind(0)); /* Producer retirement releases wait. */
 puts("zero handoff: real bus latch, move/turn/path, pose overlap and unsafe rejection PASS");
 return 0;
}
