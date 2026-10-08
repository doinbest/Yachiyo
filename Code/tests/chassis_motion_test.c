#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "chassis_motion.h"
#include "chassis_config.h"
#include "mecanum_chassis.h"
#include "hwt101_calibration.h"
#include "chassis_localization.h"
static ChassisLocalization_Status_t location;
static bool route_busy,route_submitting,stop_polling;
void Mecanum_Feedback_StopMonitor(bool active){stop_polling=active;}
bool Mecanum_FeedbackGet(unsigned i,Mecanum_Feedback_t *out){*out=location.wheels[i];return true;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *out){*out=location;}
bool ChassisRoute_IsBusy(void){return route_busy;}
bool ChassisRoute_IsSubmitting(void){return route_submitting;}
bool Mecanum_CanStopCleanly(void){return true;}
static uint32_t tick;
static bool valid,cal_busy,arm_busy,material_busy,motor_busy,grab_busy,accept=true;
static unsigned sends,stops;
static uint8_t expected_acc,sent_acc;
static float sent_vx,sent_vy,sent_omega;
static HWT101_Angle_t angle;
static Mecanum_Status_t bus;
uint32_t HAL_GetTick(void){return tick;}
bool HWT101_Cal_IsBusy(void){return cal_busy;}
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *out){if(!valid||cal_busy)return false;*out=angle;return true;}
uint8_t MaterialVision_IsBusy(void){return material_busy;}
uint8_t MechanicalArm_IsBusy(void){return arm_busy;}
bool Mecanum_IsBusy(void){return motor_busy;}
bool GrabTask_IsBusy(void){return grab_busy;}
void Mecanum_StatusGet(Mecanum_Status_t *out){*out=bus;}
bool Mecanum_Velocity_Request(float vx,float vy,float w,uint8_t acc)
{assert(acc==expected_acc);sends++;sent_acc=acc;sent_vx=vx;sent_vy=vy;sent_omega=w;return accept;}
bool Mecanum_Test_Stop(void){stops++;bus.stop_sequence++;bus.tx_complete=false;bus.stage=MECANUM_STAGE_STOPPING;bus.stop_pending=true;return accept;}
bool Mecanum_RequestPlannedStop(void){return Mecanum_Test_Stop();}
static void reading(uint32_t now,float yaw){tick=now;angle.yaw=yaw;angle.last_update_ms=now;angle.update_count++;}
static ChassisMotion_Status_t status(void){ChassisMotion_Status_t s;ChassisMotion_StatusGet(&s);return s;}
static void reset(void){memset(&location,0,sizeof(location));route_busy=route_submitting=false;tick=0;sends=stops=0;expected_acc=sent_acc=0;valid=cal_busy=arm_busy=material_busy=motor_busy=grab_busy=false;accept=true;memset(&bus,0,sizeof(bus));bus.ack_profile=MECANUM_ACK_NONE;memset(&angle,0,sizeof(angle));ChassisMotion_Init();}
static void complete_stop(void){bus.stage=MECANUM_STAGE_STOPPED;bus.stop_pending=false;ChassisMotion_Process();}
int main(void)
{
  ChassisMotion_Target_t t;
  /* Indefinite ownership has no 60-second duration surrogate. Updates retain
   * action/elapsed progress, while finite run profiles keep drive acc=0. */
  reset();expected_acc=20;
  assert(ChassisMotion_VelocityStart(-100,0,0));
  assert(sent_vx==-100 && sent_acc==20 && status().indefinite_velocity);
  unsigned action=status().action_id;
  assert(!ChassisMotion_Start(&t));
  for(tick=20;tick<=61000;tick+=20) ChassisMotion_Process();
  assert(ChassisMotion_IsBusy() && stops==0 && status().elapsed_ms==61000);
  assert(ChassisMotion_VelocityUpdate(50,20,.2f));
  assert(sent_vx==50 && sent_vy==20 && fabsf(sent_omega-.2f)<.00001f);
  assert(status().action_id==action && status().start_tick==0);
  assert(!ChassisMotion_VelocityUpdate(NAN,0,0));assert(status().requested.vx_mm_s==50);
  assert(ChassisMotion_Stop(0));complete_stop();assert(!ChassisMotion_IsBusy());
  assert(!ChassisMotion_VelocityUpdate(0,0,0));
  reset();expected_acc=20;assert(ChassisMotion_VelocityStart(50,0,0));
  tick=200;ChassisMotion_Process();assert(ChassisMotion_IsBusy() && !stops && sent_vx==50);
  ChassisMotion_Stop(0);complete_stop();
  reset();expected_acc=20;assert(ChassisMotion_VelocityStart(50,0,0));
  bus.stop_sequence++;bus.stage=MECANUM_STAGE_STOPPED;
  assert(!ChassisMotion_VelocityUpdate(80,0,0));assert(sends==1);
  tick=20;ChassisMotion_Process();assert(stops==1 && sends==1);complete_stop();
  assert(!ChassisMotion_IsBusy());
  reset();expected_acc=20;assert(ChassisMotion_VelocityStart(0,0,2*CHASSIS_MODEL_PI));
  assert(sent_omega>1.745329f);ChassisMotion_Stop(0);complete_stop();
  reset();expected_acc=20;assert(!ChassisMotion_VelocityStart(0,0,2*CHASSIS_MODEL_PI+.01f));
  reset();expected_acc=20;valid=true;reading(0,179);
  assert(!ChassisMotion_HoldStart(101,1000));
  assert(!ChassisMotion_HoldStart(30,999));
  assert(ChassisMotion_HoldStart(-30,1000));assert(sent_vx==-30);
  assert(!status().map_anchor_valid && status().requested.heading_deg==179);
  ChassisMotion_HoldStatus_t hs;ChassisMotion_HoldStatusGet(&hs);assert(hs.active);
  reading(20,-179);ChassisMotion_Process();assert(fabsf(sent_omega+.04f)<.00001f);
  reading(160,-179);ChassisMotion_Process();assert(ChassisMotion_IsBusy() && !stops);
  for(unsigned ms=180;ms<=1000;ms+=20){reading(ms,-179);ChassisMotion_Process();}
  complete_stop();ChassisMotion_HoldStatusGet(&hs);assert(!hs.active && !strcmp(hs.reason,"complete"));
  /* UI 100 deg/s boundary remains radians on the wire. */
  reset();ChassisMotion_TargetDefaults(&t);t.transition_ms=0;t.omega_rad_s=1.745329f;
  assert(ChassisMotion_Start(&t));tick=20;ChassisMotion_Process();assert(fabsf(sent_omega-1.745329f)<.00001f);
  reset();t.omega_rad_s=-1.745329f;assert(ChassisMotion_Start(&t));
  /* Straight heading correction must not inherit the faster pure-turn ceiling. */
  reset();valid=true;reading(0,0);ChassisMotion_TargetDefaults(&t);
  t.mode=CHASSIS_MOTION_HOLD_CURRENT;t.vx_mm_s=100;t.transition_ms=0;
  assert(ChassisMotion_Start(&t));reading(20,-30);ChassisMotion_Process();assert(fabsf(sent_omega-.15f)<.00001f);
  /* A timed straight task captures module yaw; no artificial map origin. */
  reset();ChassisMotion_TargetDefaults(&t);t.mode=CHASSIS_MOTION_HOLD_CURRENT;
  t.vx_mm_s=50;t.transition_ms=0;
  assert(!ChassisMotion_Start(&t));assert(!strcmp(status().reason,"imu_invalid"));
  valid=true;reading(0,179);assert(ChassisMotion_Start(&t));
  assert(fabsf(status().requested.heading_deg-179)<.001f && !status().map_anchor_valid);
  reading(20,-179);ChassisMotion_Process();assert(sent_omega<0);
  assert(status().state==CHASSIS_MOTION_RUNNING && !status().map_anchor_valid);
  valid=false;ChassisMotion_Process();assert(stops==1);complete_stop();
  assert(status().state==CHASSIS_MOTION_ERROR);
  reset();valid=true;reading(0,-42);t.vx_mm_s=-50;
  assert(ChassisMotion_Start(&t));assert(fabsf(status().requested.heading_deg+42)<.001f);
  reading(20,-40);ChassisMotion_Process();assert(sent_vx<0 && sent_omega<0);
  tick+=301;ChassisMotion_Process();assert(stops==1);complete_stop();
  assert(status().state==CHASSIS_MOTION_ERROR);
  reset();valid=true;reading(0,12);t.vx_mm_s=0;t.vy_mm_s=30;t.hold_ms=100;
  assert(ChassisMotion_Start(&t));
  for(unsigned ms=20;ms<=600;ms+=20){reading(ms,12);ChassisMotion_Process();}
  complete_stop();assert(status().state==CHASSIS_MOTION_DONE && !strcmp(status().reason,"timed_complete"));
  assert(!status().map_anchor_valid && status().requested.heading_deg==12);
  t.omega_rad_s=.1f;assert(!ChassisMotion_Start(&t)); /* no two rotation controllers */
  reset();ChassisMotion_TargetDefaults(&t);assert(t.transition_ms==500&&t.stop_ms==500);t.vx_mm_s=100;t.hold_ms=100;
  assert(ChassisMotion_Start(&t));assert(ChassisMotion_IsBusy());
  for(tick=20;tick<=240;tick+=20)ChassisMotion_Process();
  tick=250;ChassisMotion_Process();assert(sent_vx<50); /* not due yet */
  tick=260;ChassisMotion_Process();assert(fabsf(sent_vx-53.1395f)<.001f);
  assert(status().control_dt_ms==20);
  for(tick=280;tick<=600;tick+=20)ChassisMotion_Process();
  assert(fabsf(sent_vx-100)<.001f);
  for(tick=620;tick<=1100;tick+=20)ChassisMotion_Process();
  assert(status().state==CHASSIS_MOTION_STOPPING&&stops==1);assert(ChassisMotion_IsBusy());
  complete_stop();assert(status().state==CHASSIS_MOTION_DONE);assert(!strcmp(status().reason,"timed_complete"));
  assert(!ChassisMotion_IsBusy());
  reset();ChassisMotion_TargetDefaults(&t);t.vx_mm_s=1000;t.transition_ms=0;
  assert(ChassisMotion_Start(&t));tick=20;ChassisMotion_Process();assert(fabsf(sent_vx-1000)<.01f);
  reset();t.vx_mm_s=4000;t.vy_mm_s=4000;assert(!ChassisMotion_Start(&t));t.vy_mm_s=0;t.vx_mm_s=NAN;assert(!ChassisMotion_Start(&t));
  t.vx_mm_s=50;t.omega_rad_s=1.746f;assert(!ChassisMotion_Start(&t));t.omega_rad_s=0;
  material_busy=true;assert(!ChassisMotion_Start(&t));material_busy=false;
  arm_busy=true;assert(!ChassisMotion_Start(&t));arm_busy=false;
  cal_busy=true;assert(!ChassisMotion_Start(&t));cal_busy=false;
  motor_busy=true;assert(!ChassisMotion_Start(&t));motor_busy=false;
  bus.ack_profile=MECANUM_ACK_UNKNOWN;assert(!ChassisMotion_Start(&t));assert(!strcmp(status().reason,"ack_profile_unknown"));bus.ack_profile=MECANUM_ACK_NONE;
  assert(ChassisMotion_Start(&t));tick=101;ChassisMotion_Process();assert(stops==1);complete_stop();assert(status().state==CHASSIS_MOTION_ERROR);assert(status().control_dt_ms==101);assert(!strcmp(status().reason,"control_gap"));
  reset();assert(ChassisMotion_Start(&t));tick=20;ChassisMotion_Process();assert(ChassisMotion_Stop(0));assert(stops==1);complete_stop();assert(!strcmp(status().reason,"cancelled"));
  reset();assert(ChassisMotion_Start(&t));tick=20;ChassisMotion_Process();bus.stage=MECANUM_STAGE_FAULT;ChassisMotion_Process();assert(stops==1&&status().state==CHASSIS_MOTION_STOPPING);complete_stop();assert(status().state==CHASSIS_MOTION_ERROR&&!strcmp(status().reason,"bus_fault"));
  reset();valid=true;reading(0,179);assert(ChassisMotion_AnchorSet(1));t.mode=CHASSIS_MOTION_HEADING;t.heading_deg=179;t.transition_ms=0;
  assert(ChassisMotion_Start(&t));reading(20,-179);ChassisMotion_Process();assert(fabsf(status().current_deg-181)<.001f);assert(fabsf(sent_omega+.04f)<.0001f);
  assert(fabsf(status().map_yaw_rad-(1+2*CHASSIS_MODEL_PI/180))<.0001f);
  tick=40;ChassisMotion_Process();assert(fabsf(sent_omega+.04f)<.0001f); /* same sample retains heading output */
  valid=false;ChassisMotion_Process();assert(!status().heading_valid&&!status().map_anchor_valid);assert(stops==1);complete_stop();assert(status().state==CHASSIS_MOTION_ERROR);
  reset();valid=true;reading(0,0);assert(ChassisMotion_AnchorSet(0));t.mode=CHASSIS_MOTION_ANGULAR_VELOCITY;assert(ChassisMotion_Start(&t));reading(20,NAN);ChassisMotion_Process();assert(!status().map_anchor_valid); /* direct omega doesn't require IMU */
  assert(status().state==CHASSIS_MOTION_RUNNING);ChassisMotion_Stop(0);complete_stop();
  reset();valid=true;reading(0,0);t.mode=CHASSIS_MOTION_HEADING;assert(!ChassisMotion_Start(&t));assert(!strcmp(status().reason,"anchor_invalid"));assert(ChassisMotion_AnchorSet(0));assert(ChassisMotion_Start(&t));reading(20,0);ChassisMotion_Process();ChassisMotion_AnchorInvalidate();ChassisMotion_Process();assert(stops==1);complete_stop();assert(!strcmp(status().reason,"anchor_invalid"));
  reset();ChassisMotion_TargetDefaults(&t);t.vx_mm_s=100;t.transition_ms=0;
  assert(ChassisMotion_Start(&t));tick=20;ChassisMotion_Process();assert(sent_vx==100);
  assert(ChassisMotion_Stop(100));tick=40;ChassisMotion_Process();assert(fabsf(sent_vx-90.45085f)<.001f);
  tick=60;ChassisMotion_Process();tick=80;ChassisMotion_Process();tick=100;ChassisMotion_Process();
  tick=120;ChassisMotion_Process();assert(stops==1);complete_stop();assert(!strcmp(status().reason,"cancelled"));
  reset();assert(ChassisMotion_Start(&t));accept=false;tick=20;ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_ERROR);assert(!strcmp(status().reason,"stop_rejected"));
  reset();assert(ChassisMotion_Start(&t));assert(ChassisMotion_Stop(0));bus.stage=MECANUM_STAGE_FAULT;ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_ERROR&&!strcmp(status().reason,"stop_failed"));
  reset();tick=0xFFFFFFF0U;assert(ChassisMotion_Start(&t));tick=4;ChassisMotion_Process();assert(status().control_dt_ms==20&&status().elapsed_ms==20); /* HAL tick wrap */
  reset();assert(ChassisMotion_Start(&t));bus.stage=MECANUM_STAGE_STOPPING;bus.locked=true;bus.error=MECANUM_ERROR_TX;bus.stop_pending=true;
  tick=20;ChassisMotion_Process();assert(status().state==CHASSIS_MOTION_STOPPING&&stops==1);
  complete_stop();assert(status().state==CHASSIS_MOTION_ERROR);
  reset();assert(ChassisMotion_Start(&t));assert(ChassisMotion_Stop(0));bus.locked=true;bus.error=MECANUM_ERROR_CANCELLED;
  complete_stop();assert(status().state==CHASSIS_MOTION_ERROR&&!strcmp(status().reason,"sync_cache_uncertain"));
  puts("chassis_motion_test: PASS (timed ramp, limits, guards, dt, stop, fault, heading, unwrap, invalidation)");
  return 0;
}
