#include "chassis_telemetry.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_route.h"
#include "mecanum_chassis.h"
#include "console_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static bool motion_busy;
static ChassisMotion_Status_t motion_fixture;
static ChassisMotion_Target_t submitted;
static bool route_busy;
bool ChassisRoute_IsBusy(void){return route_busy;}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *out){memset(out,0,sizeof(*out));out->state="waiting";out->reason="arrived";out->segment=2;out->action_id=9;out->target_x_mm=1200;out->stop_confirmed=true;}
static char reply[4096],frame[2048];
static bool tx_ready=true;
static unsigned telemetry_cancels;
bool ConsoleTx_TelemetryReady(void){return tx_ready;}
void ConsoleTx_TelemetryCancel(void){telemetry_cancels++;frame[0]=0;}
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *data,uint16_t size){assert(size<sizeof(reply));memcpy(reply,data,size);reply[size]=0;return true;}
bool ConsoleTx_Telemetry(const char *data,uint16_t size){assert(size<sizeof(frame));memcpy(frame,data,size);frame[size]=0;return true;}
uint32_t ConsoleTx_Dropped(void){return 7;}
bool Mecanum_IsBusy(void){return false;}
bool Mecanum_RecoveryAfterReset(void){return true;}
bool Mecanum_AckProfile_Set(Mecanum_AckProfile_t p){return p!=MECANUM_ACK_UNKNOWN;}
void Mecanum_Feedback_Enable(bool on){(void)on;}
bool Mecanum_Feedback_Select(uint8_t address){return address<=4;}
bool Mecanum_FeedbackGet(unsigned i,Mecanum_Feedback_t *out){memset(out,0,sizeof(*out));out->position_raw=i%2?-4294967295LL:4294967295LL;return true;}
void Mecanum_StatusGet(Mecanum_Status_t *out){memset(out,0,sizeof(*out));}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *out){*out=motion_fixture;if(!out->reason)out->reason="idle";}
void ChassisMotion_TargetDefaults(ChassisMotion_Target_t *t){memset(t,0,sizeof(*t));}
bool ChassisMotion_Start(const ChassisMotion_Target_t *t){submitted=*t;return t->hold_ms>0;}
bool ChassisMotion_Stop(uint32_t ms){(void)ms;return true;}
bool ChassisMotion_IsBusy(void){return motion_busy;}
bool ChassisMotion_AnchorSet(float yaw){(void)yaw;return true;}
int main(void)
{
  {char *run[]={"chassis","run","50","0","0","19500"};
   assert(ChassisTelemetry_Command(6,run));assert(submitted.mode==CHASSIS_MOTION_HOLD_CURRENT);
   run[4]="0.1";assert(ChassisTelemetry_Command(6,run));
   assert(submitted.mode==CHASSIS_MOTION_ANGULAR_VELOCITY && submitted.omega_rad_s>0);}
  char *bad[]={"chassis","stream","on","-1"};
  ChassisTelemetry_Init();tick=200;ChassisTelemetry_Process();assert(!frame[0]);
  assert(!ChassisTelemetry_Stream(true,0));
  assert(ChassisTelemetry_Command(4,bad));assert(strstr(reply,"ERR"));
  assert(ChassisTelemetry_Stream(true,12345));assert(strstr(reply,"\"kind\":\"config\""));puts(reply);
  assert(strstr(reply,"\"telemetry_period_ms\":2000"));
  tick=220;ChassisTelemetry_Process();assert(!frame[0]);
  tick=2200;ChassisLocalization_Process();ChassisTelemetry_Process();assert(strstr(frame,"\"v\":2"));
  assert(strlen(frame)<1000);assert(strstr(frame,"-4294967295"));assert(strstr(frame,"\"route\":[\"waiting\",\"arrived\",2,9"));assert(strstr(frame,"\"dropped\":7"));puts(frame);
  {char *snapshot[]={"chassis","snapshot"};assert(ChassisTelemetry_Command(2,snapshot));
   assert(strstr(reply,"\"v\":1") && strstr(reply,"\"position_valid\":[false,false,false,false]"));
   assert(strstr(reply,"\"stop_confirmed\":true"));
   printf("wire bytes compact=%u detailed=%u\n",(unsigned)strlen(frame),(unsigned)strlen(reply));
   assert(strlen(frame)*10 < strlen(reply)*7);}
  motion_fixture.distance_mode=true;motion_fixture.state=CHASSIS_MOTION_STOPPING;
  motion_fixture.reason="position_reached";motion_fixture.action_id=UINT32_MAX;
  motion_fixture.target_x_mm=motion_fixture.target_y_mm=10000;
  motion_fixture.error_x_mm=motion_fixture.error_y_mm=-20000;
  frame[0]=0;tick=UINT32_MAX;ChassisTelemetry_Process();
  assert(strstr(frame,"\"distance\":[") && strstr(frame,"4294967295") && strstr(frame,"\r\n"));
  assert(strlen(frame)<1200);puts(frame);motion_fixture.distance_mode=false;
  frame[0]=0;tick=100;ChassisTelemetry_Process();assert(!frame[0]);
  tick=2200;tx_ready=false;ChassisTelemetry_Process();assert(!frame[0]);
  tx_ready=true;ChassisTelemetry_Process();assert(frame[0]);
  {char *task[]={"chassis","task"};motion_fixture.distance_mode=true;assert(ChassisTelemetry_Command(2,task));
   assert(strstr(reply,"error_mm=") && strstr(reply,"stop_confirmed="));motion_fixture.distance_mode=false;}
  {char *task[]={"chassis","task"};motion_fixture.requested.mode=CHASSIS_MOTION_HOLD_CURRENT;
   motion_fixture.requested.heading_deg=12.5f;assert(ChassisTelemetry_Command(2,task));
   assert(strstr(reply,"mode=2 target_deg=12.50") && strstr(reply,"yaw_valid="));}
  assert(!ChassisTelemetry_ConfirmPositionUnits(16384));
  assert(ChassisTelemetry_ConfirmPositionUnits(65536));
  assert(ChassisTelemetry_Stream(false,0));frame[0]=0;tick=1000;ChassisTelemetry_Process();assert(!frame[0]);
  assert(telemetry_cancels>=2);
  route_busy=true;
  {char *profile[]={"chassis","profile","none"};char *feedback[]={"chassis","feedback","on"};
   assert(ChassisTelemetry_Command(3,profile));assert(strstr(reply,"ERR"));
   assert(ChassisTelemetry_Command(3,feedback));assert(strstr(reply,"ERR"));}
  assert(!ChassisTelemetry_Origin(0,0,0));
  assert(!ChassisTelemetry_ConfirmPositionUnits(0));
  route_busy=false;motion_busy=true;
  {char *feedback[]={"chassis","feedback","off"};assert(ChassisTelemetry_Command(3,feedback));assert(strstr(reply,"ERR"));}
  puts("telemetry: opt-in, session, bounds, raw int64, unavailable feedback PASS");return 0;
}
