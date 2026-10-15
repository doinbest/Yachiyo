#include "GrabTask.h"
#include "chassis_route.h"
#include "chassis_route_profile.h"
#include "chassis_config.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "console_tx.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
static ChassisLocalization_Status_t loc;
static ChassisMotion_Status_t motion;
static Mecanum_Status_t bus;
static unsigned starts,stops,feedback_selections;
static float last_speed;
static uint32_t last_timeout,tick;
static bool accept=true,last_turn,feedback_enabled;
static bool independent_stop, mission_pending;
static ChassisMotion_Profile_t selected_profile;
bool ContestMission_HasPendingWork(void){return mission_pending;}
ChassisMotion_Profile_t ChassisMotion_ProfileGet(void){return selected_profile;}
bool ChassisMotion_ProfileSet(ChassisMotion_Profile_t p){selected_profile=p;return true;}
bool Mecanum_Feedback_Select(uint8_t wheel){assert(wheel==0);feedback_selections++;return true;}
void Mecanum_Feedback_Enable(bool enabled){feedback_enabled=enabled;}
static bool grab_busy, clearance;
bool GrabTask_IsBusy(void){return grab_busy;}
bool GrabTask_PoseClearanceReady(void){return clearance;}
static unsigned qr_pose_calls;
static bool qr_pose_delayed;
bool GrabTask_QrPoseStart(void){qr_pose_calls++;if(qr_pose_delayed)grab_busy=true;return true;}
void GrabTask_StatusGet(GrabTask_Status_t *s){memset(s,0,sizeof *s);s->state=grab_busy?GRAB_ROUTE_POSE:GRAB_COMPLETE;s->result=grab_busy?"pending":"qr_pose_ready";}
static char reply[700];
uint32_t HAL_GetTick(void){return tick;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *s){*s=loc;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *s){*s=motion;}
void Mecanum_StatusGet(Mecanum_Status_t *s){*s=bus;}
bool ChassisMotion_IsBusy(void){return motion.state==CHASSIS_MOTION_RUNNING||motion.state==CHASSIS_MOTION_STOPPING;}
static bool action(float x,float y,float yaw,float speed,uint32_t timeout,bool turn)
{
  assert(timeout>0 && timeout<=400000);assert(turn || (speed>=10 && speed<=5000));
  assert(isfinite(yaw) && yaw>=-180 && yaw<=180);
  assert((!ChassisRoute_IsBusy() && !ChassisRoute_StationReserved()) || ChassisRoute_IsSubmitting());
  if(!accept){motion.reason="busy";return false;}
  starts++;last_speed=speed;last_timeout=timeout;last_turn=turn;
  motion.profile=selected_profile;motion.action_id++;motion.state=CHASSIS_MOTION_RUNNING;motion.reason="running";
  motion.target_x_mm=x;motion.target_y_mm=y;motion.target_map_yaw_deg=yaw;motion.stop_confirmed=false;
  return true;
}
bool ChassisMotion_MoveTo(float x,float y,float yaw,float speed,uint32_t timeout)
{ motion.linear_accel_mm_s2=300;return action(x,y,yaw,speed,timeout,false); }
bool ChassisMotion_MoveToAccel(float x,float y,float yaw,float speed,float accel,uint32_t timeout)
{ motion.linear_accel_mm_s2=accel;return action(x,y,yaw,speed,timeout,false); }
void ChassisMotion_TestParametersDefaults(ChassisMotion_TestParameters_t *p)
{p->curve_period_ms=1;p->linear_decel_mm_s2=300;p->linear_accel_mm_s2=300;p->cosine_k=200;p->heading_kp=CHASSIS_HEADING_TEST_KP;p->heading_kd=CHASSIS_HEADING_TEST_KD;}
bool ChassisMotion_MoveToTuned(float x,float y,float yaw,float speed,const ChassisMotion_TestParameters_t *p,uint32_t timeout)
{motion.linear_decel_mm_s2=p->linear_decel_mm_s2;motion.curve_period_ms=p->curve_period_ms;motion.linear_accel_mm_s2=p->linear_accel_mm_s2;motion.cosine_k=p->cosine_k;motion.heading_kp=p->heading_kp;motion.heading_kd=p->heading_kd;return action(x,y,yaw,speed,timeout,false);}
bool ChassisMotion_TurnTo(float yaw,uint32_t timeout)
{ return action(loc.observer.feedback.x_mm,loc.observer.feedback.y_mm,yaw,0,timeout,true); }
static float turn_limit;
static float curve_progress,curve_end;
float ChassisMotion_PathProgress(void){return motion.state==CHASSIS_MOTION_DONE?curve_end+10:curve_progress;}
bool ChassisMotion_PathStart(const ChassisPath *path,float speed,float turn,uint32_t timeout,float resume)
{
  turn_limit=turn;curve_progress=resume;curve_end=path->turn_end;
  ChassisPath_Point p=path->point[path->count-1];
  return action(p.x,p.y,p.yaw,speed,timeout,false);
}
bool ChassisMotion_TurnToSpeed(float yaw,float speed,uint32_t timeout)
{turn_limit=speed;return ChassisMotion_TurnTo(yaw,timeout);}
bool ChassisMotion_Stop(uint32_t ms){assert(ms==0);stops++;motion.state=CHASSIS_MOTION_STOPPING;return true;}
bool ChassisMotion_StopRequest(uint32_t token){assert(!token);return true;}
bool ChassisMotion_StopPending(void){return false;}
void ChassisMotion_StopStatusGet(ChassisStop_Status_t *s){memset(s,0,sizeof *s);s->tx_complete=s->wheels_stopped=motion.stop_confirmed||independent_stop;s->requested_ms=tick;}
bool ConsoleTx_Write(const uint8_t*d,uint16_t n){assert(n<sizeof(reply));memcpy(reply,d,n);reply[n]=0;return true;}
static ChassisRoute_Status_t status(void){ChassisRoute_Status_t s;ChassisRoute_StatusGet(&s);return s;}
static void advance_clock(uint32_t ms){tick+=ms;loc.feedback_tick=tick;}
static bool route_command(char *verb)
{
  char *tokens[]={"chassis","route",verb};
  assert(ChassisRoute_Command(3,tokens));
  return !strncmp(reply,"OK ",3);
}
static bool start_fixed_station_plan(void)
{
  RadarPlan_t plan={0};
  plan.valid=true;plan.count=CHASSIS_ROUTE_TARGET_COUNT+1U;
  for(unsigned i=0;i<plan.count;i++)
    assert(ChassisRouteProfile_FixedPoint(i,&plan.points[i]));
  return ChassisRoute_PlanStart(&plan,200,true);
}
static void setup(void)
{
  memset(&loc,0,sizeof(loc));memset(&motion,0,sizeof(motion));memset(&bus,0,sizeof(bus));
  motion.reason="idle";loc.origin_valid=loc.feedback_valid=loc.speed_valid=true;
  loc.origin.x_mm=loc.observer.feedback.x_mm=2250;loc.origin.y_mm=loc.observer.feedback.y_mm=150;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI;
  loc.generation=4;tick=100;loc.feedback_tick=tick;starts=stops=feedback_selections=0;accept=true;last_turn=false;
  independent_stop=grab_busy=clearance=qr_pose_delayed=mission_pending=false;selected_profile=CHASSIS_PROFILE_LINEAR;ChassisRoute_Init();
}
static void feedback_arrival(float dx,float dy,float dyaw)
{
  if(!last_turn){loc.observer.feedback.x_mm=motion.target_x_mm;loc.observer.feedback.y_mm=motion.target_y_mm;}
  loc.observer.feedback.x_mm+=dx;loc.observer.feedback.y_mm+=dy;
  loc.observer.feedback.yaw_rad=(motion.target_map_yaw_deg+dyaw)*CHASSIS_MODEL_PI/180;
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";
  ChassisRoute_Process();
}
static void arrive(void){feedback_arrival(0,0,0);}
static void finish_leg(void)
{
  unsigned guard=0;
  while(!strcmp(status().state,"running") || !strcmp(status().state,"stopping")){
    assert(++guard<=10);advance_clock(1000);arrive();
  }
  assert(!strcmp(status().state,"waiting") || !strcmp(status().state,"station") || !strcmp(status().state,"done"));
}
static void to_segment(unsigned segment)
{
  while(status().segment<segment){finish_leg();assert(route_command("next"));}
}
static void expect_cancel(void)
{
  unsigned before=starts;assert(ChassisRoute_Cancel());
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  advance_clock(1000);ChassisRoute_Process();assert(starts==before);
  assert(!strcmp(status().state,"cancelled"));assert(!route_command("next"));
}
int main(void)
{
  setup();loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;assert(!ChassisRoute_QrTestStart(50,1200));
  setup();loc.origin.y_mm=loc.observer.feedback.y_mm=2250;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=-CHASSIS_MODEL_PI/2;
  assert(ChassisRoute_QrTestStart(50,1200));
  assert(motion.target_x_mm==2100 && motion.target_y_mm==2100 && !last_turn);
  finish_leg();ChassisRoute_Process();
  assert(motion.target_y_mm==1200 && motion.target_map_yaw_deg==-90 && !last_turn);
  arrive();
  assert(last_turn && motion.target_x_mm==2100 && motion.target_y_mm==1200 &&
    motion.target_map_yaw_deg==90 && !strcmp(status().phase,"qr_scan_turn"));
  assert(!strcmp(status().state,"running") && !status().stop_confirmed);
  unsigned qr_turn_starts=starts;
  for(unsigned i=0;i<10;i++)ChassisRoute_Process();
  assert(starts==qr_turn_starts && !strcmp(status().state,"running"));
  arrive();ChassisRoute_Process();
  assert(!strcmp(status().state,"done") && status().total==2 && status().stop_confirmed);
  /* Lower QR route: keep left-facing yaw during the diagonal, turn clockwise,
   * advance up the right-hand lane, then turn 180 at the scan point. */
  setup();loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI;
  assert(ChassisRoute_QrTestStart(50,1200));
  assert(!last_turn && motion.target_x_mm==2100 && motion.target_y_mm==300 &&
    motion.target_map_yaw_deg==180);
  finish_leg();ChassisRoute_Process();
  assert(last_turn && motion.target_x_mm==2100 && motion.target_y_mm==300 &&
    motion.target_map_yaw_deg==90);
  arrive();
  assert(!last_turn && motion.target_x_mm==2100 && motion.target_y_mm==1200 &&
    motion.target_map_yaw_deg==90);
  arrive();
  assert(last_turn && motion.target_x_mm==2100 && motion.target_y_mm==1200 &&
    motion.target_map_yaw_deg==-90 && status().target_yaw_deg==-90 &&
    !strcmp(status().phase,"qr_scan_turn") && !status().stop_confirmed);
  qr_turn_starts=starts;
  arrive();ChassisRoute_Process();
  assert(!strcmp(status().state,"done") && status().stop_confirmed);
  for(unsigned i=0;i<10;i++)ChassisRoute_Process();
  assert(starts==qr_turn_starts); /* No second half-turn after completion. */
  for(unsigned scenario=0;scenario<3;scenario++){
    setup();loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI;
    assert(ChassisRoute_QrTestStart(50,1200));
    finish_leg();ChassisRoute_Process();arrive();arrive();
    assert(last_turn && !strcmp(status().phase,"qr_scan_turn"));
    if(scenario==0)expect_cancel();
    else {
      unsigned before=starts;
      motion.state=scenario==1?CHASSIS_MOTION_DONE:CHASSIS_MOTION_ERROR;
      motion.reason=scenario==1?"feedback_arrived":"bus_unavailable";
      motion.stop_confirmed=false;ChassisRoute_Process();
      assert(starts==before && !strcmp(status().state,"error") && !status().stop_confirmed);
      assert(!strcmp(status().reason,scenario==1?"arrival_unconfirmed":"bus_unavailable"));
    }
  }
  setup();loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI;
  assert(ChassisRoute_QrTestStart(50,1100));
  finish_leg();ChassisRoute_Process();assert(last_turn);
  expect_cancel(); /* Cancellation during the turn must not start the straight. */
  setup();assert(start_fixed_station_plan());
  assert(status().segment==1 && motion.target_x_mm==2100 && motion.target_y_mm==300);
  assert(status().target_y_mm==1200 && !strcmp(status().direction,"diagonal"));
  arrive();
  assert(!strcmp(status().state,"running") && status().segment==1);
  assert(starts==2 && last_turn && motion.target_x_mm==2100 && motion.target_y_mm==300 && motion.target_map_yaw_deg==90);
  arrive();assert(starts==3 && !last_turn && motion.target_x_mm==2100 && motion.target_y_mm==1200);
  assert(!strcmp(status().direction,"forward") && !ChassisRoute_StationResume());
  arrive();assert(!strcmp(status().state,"station") && status().station==5 && status().visit==1);
  setup();assert(!route_command("next"));assert(route_command("start"));
  assert(status().total==15 && last_speed==CHASSIS_ROUTE_DEFAULT_SPEED_MM_S && !last_turn);
  assert(!route_command("start"));
  for(unsigned i=1;i<=15;i++){
    RadarPlan_Point_t p;assert(ChassisRouteProfile_FixedPoint(i,&p));
    assert(status().segment==i && status().station==p.station && status().visit==p.visit);
    assert(status().target_x_mm==p.x_mm && status().target_y_mm==p.y_mm);
    assert(!route_command("next"));finish_leg();unsigned count=starts;
    for(unsigned j=0;j<10;j++){ChassisRoute_Process();}assert(starts==count);
    if(i<15){assert(ChassisRoute_IsBusy());assert(route_command("next"));}
  }
  assert(!ChassisRoute_IsBusy() && !strcmp(status().reason,"route_complete"));
  assert(loc.origin.x_mm==2250 && loc.origin.y_mm==150 && loc.generation==4);
  setup();assert(route_command("start"));to_segment(3);
  assert(last_turn && !strcmp(status().phase,"turn") && motion.target_map_yaw_deg==180);
  unsigned n=starts;arrive();assert(starts==n+1 && !last_turn && !strcmp(status().direction,"forward"));
  assert(status().target_yaw_deg==90 && status().phase_target_yaw_deg==180);
  arrive();assert(starts==n+2 && last_turn && !strcmp(status().phase,"arrival_turn"));
  uint32_t turn_timeout=last_timeout;advance_clock(1000);feedback_arrival(25,0,0);
  assert(starts==n+3 && !last_turn && last_speed<=50 && last_timeout==turn_timeout-1000);
  assert(status().segment==3 && status().target_x_mm==1200 && !strcmp(status().phase,"move"));
  arrive();assert(!strcmp(status().state,"waiting") && status().stop_confirmed);
  setup();assert(route_command("start"));to_segment(6);
  assert(last_turn && motion.target_map_yaw_deg==180);arrive();
  assert(!last_turn && motion.target_x_mm==425 && !strcmp(status().phase,"dock_approach"));
  arrive();assert(last_turn && motion.target_x_mm==425 && motion.target_map_yaw_deg==90);arrive();
  assert(!last_turn && motion.target_x_mm==350 && last_speed==50 && !strcmp(status().phase,"dock"));
  arrive();assert(!strcmp(status().state,"waiting"));assert(route_command("next"));
  assert(!last_turn && motion.target_x_mm==425 && motion.target_map_yaw_deg==90 && last_speed==50 && !strcmp(status().phase,"undock"));
  arrive();assert(last_turn && motion.target_x_mm==425 && motion.target_map_yaw_deg==180);
  arrive();assert(!last_turn && !strcmp(status().direction,"reverse"));finish_leg();
  const char *cancel_phases[]={"turn","dock_approach","arrival_turn","dock"};
  for(unsigned phase=0;phase<4;phase++){
    setup();assert(route_command("start"));to_segment(6);
    for(unsigned j=0;j<phase;j++){arrive();}assert(!strcmp(status().phase,cancel_phases[phase]));expect_cancel();
  }
  for(unsigned phase=0;phase<4;phase++){
    setup();assert(route_command("start"));to_segment(6);
    for(unsigned j=0;j<phase;j++){arrive();}
    unsigned count=starts;motion.state=CHASSIS_MOTION_DONE;motion.reason="feedback_arrived";
    motion.stop_confirmed=false;ChassisRoute_Process();
    assert(starts==count && !strcmp(status().reason,"arrival_unconfirmed"));
  }
  setup();assert(route_command("start"));to_segment(7);assert(!strcmp(status().phase,"undock"));expect_cancel();
  setup();assert(route_command("start"));assert(!strcmp(status().phase,"move"));expect_cancel();
  setup();loc.origin_valid=false;assert(!route_command("start") && !starts);
  setup();loc.origin.x_mm=2249;assert(!route_command("start"));
  setup();assert(route_command("start"));motion.action_id++;ChassisRoute_Process();assert(!strcmp(status().reason,"action_mismatch") && stops==1);
  setup();assert(route_command("start"));motion.state=CHASSIS_MOTION_DONE;motion.reason="timed_complete";ChassisRoute_Process();assert(!strcmp(status().reason,"arrival_unconfirmed") && starts==1);
  setup();assert(route_command("start"));motion.state=CHASSIS_MOTION_DONE;motion.reason="feedback_arrived";motion.stop_confirmed=false;ChassisRoute_Process();assert(!strcmp(status().state,"error") && starts==1);
  setup();assert(route_command("start"));finish_leg();loc.generation++;assert(!route_command("next") && stops==1 && !status().stop_confirmed);
  setup();assert(route_command("start"));finish_leg();loc.speed_valid=false;ChassisRoute_Process();assert(stops==1);
  setup();assert(route_command("start"));finish_leg();bus.locked=true;ChassisRoute_Process();assert(stops==1);
  setup();accept=false;assert(!route_command("start") && !ChassisRoute_IsBusy());
  setup();char *move[]={"chassis","move","-150","150"};assert(ChassisRoute_Command(4,move));assert(starts==1 && motion.target_x_mm==2100 && motion.target_y_mm==300 && last_speed==CHASSIS_DISTANCE_SPEED_MM_S);
  setup();char *bad[]={"chassis","move","250","250"};assert(ChassisRoute_Command(4,bad));assert(!starts && strstr(reply,"ERR"));
  /* Standalone tests use body-relative distances and the production MoveTo controller. */
  char *single[]={"chassis","test","1000","0","1000"};
  setup();assert(ChassisRoute_Command(5,single));
  assert(starts==1 && fabsf(motion.target_x_mm-1250)<0.01f && fabsf(motion.target_y_mm-150)<0.01f);
  assert(last_speed==1000 && strstr(reply,"profile=linear") && !last_turn);
  assert(motion.linear_accel_mm_s2==300 && strstr(reply,"accel_mm_s2=300.0"));
  char *custom[]={"chassis","test","1000","0","1000","linear","800"};
  setup();assert(ChassisRoute_Command(7,custom));assert(starts==1 && motion.linear_accel_mm_s2==800 && strstr(reply,"accel_mm_s2=800.0"));
  const char *invalid_accel[]={"0","9","5001","nan"};
  for(unsigned i=0;i<4;i++){setup();custom[6]=(char*)invalid_accel[i];assert(ChassisRoute_Command(7,custom));assert(!starts && strstr(reply,"accel_range"));}
  setup();custom[5]="cosine";custom[6]="800";assert(ChassisRoute_Command(7,custom));assert(starts==1 && motion.cosine_k==800);
  char *asym[]={"chassis","test","1000","0","1000","linear","500","0.02","0","1","300"};
  setup();assert(ChassisRoute_Command(11,asym));assert(starts==1 && motion.linear_accel_mm_s2==500 && motion.linear_decel_mm_s2==300);
  assert(strstr(reply,"decel_mm_s2=300.0"));
  asym[10]="0";setup();assert(ChassisRoute_Command(11,asym));assert(!starts && strstr(reply,"decel_range"));
  char *period[]={"chassis","test","1000","0","1000","linear","300","0.02","0","5"};
  setup();assert(ChassisRoute_Command(10,period));assert(starts==1 && motion.curve_period_ms==5);
  assert(strstr(reply,"curve_period_ms=5 control_period_ms=1"));
  period[9]="0";setup();assert(ChassisRoute_Command(10,period));assert(!starts && strstr(reply,"curve_period_range"));
  period[9]="1.5";setup();assert(ChassisRoute_Command(10,period));assert(!starts && strstr(reply,"curve_period_range"));
  char *tuned[]={"chassis","test","1000","0","1000","cosine","120","0.1","0.01"};
  setup();assert(ChassisRoute_Command(9,tuned));assert(starts==1 && motion.cosine_k==120 && motion.heading_kp==.1f && motion.heading_kd==.01f);
  assert(strstr(reply,"cosine_k=120 heading_kp=0.100000 heading_kd=0.010000"));
  setup();tuned[7]="nan";assert(ChassisRoute_Command(9,tuned));assert(!starts && strstr(reply,"heading_gain_range"));
  setup();tuned[7]="0.1";tuned[6]="120.5";assert(ChassisRoute_Command(9,tuned));assert(!starts && strstr(reply,"cosine_k_range"));
  setup();tuned[6]="120";assert(ChassisRoute_Command(8,tuned));assert(!starts);
  char *cosine[]={"chassis","test","1000","0","1000","cosine"};
  setup();assert(ChassisRoute_Command(6,cosine));assert(starts==1 && motion.profile==CHASSIS_PROFILE_COSINE && strstr(reply,"profile=cosine"));
  setup();mission_pending=true;assert(ChassisRoute_Command(6,cosine));assert(!starts && selected_profile==CHASSIS_PROFILE_LINEAR && strstr(reply,"mission_reserved"));
  setup();accept=false;assert(ChassisRoute_Command(6,cosine));assert(!starts && selected_profile==CHASSIS_PROFILE_LINEAR);
  setup();cosine[5]="invalid";assert(ChassisRoute_Command(6,cosine));assert(!starts && strstr(reply,"profile_required"));
  setup();loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;
  single[2]="300";single[3]="400";assert(ChassisRoute_Command(5,single));
  assert(fabsf(motion.target_x_mm-1850)<0.01f && fabsf(motion.target_y_mm-450)<0.01f);
  assert(fabsf(motion.target_map_yaw_deg-90)<0.01f);
  setup();single[2]="3000";single[3]="0";single[4]="10";
  assert(ChassisRoute_Command(5,single));assert(starts==1 && last_timeout>=300000 && last_timeout<=400000);
  setup();single[3]="1";assert(ChassisRoute_Command(5,single));assert(!starts && strstr(reply,"ERR"));
  single[2]="0";single[3]="0";assert(ChassisRoute_Command(5,single));assert(!starts);
  single[2]="1000";single[4]="nan";assert(ChassisRoute_Command(5,single));assert(!starts);
  single[4]="5001";assert(ChassisRoute_Command(5,single));assert(!starts);
  single[4]="1000";assert(ChassisRoute_Command(4,single));assert(!starts);
  setup();assert(route_command("start"));unsigned prior_starts=starts;
  assert(ChassisRoute_Command(5,single));assert(starts==prior_starts && strstr(reply,"route_reserved"));
  setup();accept=false;assert(ChassisRoute_Command(5,single));assert(!starts && strstr(reply,"ERR"));
  setup();assert(route_command("start"));char *query[]={"chassis","route","status"};assert(ChassisRoute_Command(3,query));assert(strstr(reply,"segment=1") && strstr(reply,"total=15") && strstr(reply,"phase=move"));
  char *stop[]={"chassis","stop"};assert(ChassisRoute_Command(2,stop));assert(stops==1);
  setup();char *slow[]={"chassis","route","start","10"};assert(ChassisRoute_Command(4,slow));
  assert(last_timeout>234000 && last_timeout<235000);finish_leg();assert(route_command("next"));finish_leg();assert(route_command("next"));
  uint32_t first_timeout=last_timeout;advance_clock(1500);arrive();assert(last_timeout==first_timeout-1500);
  advance_clock(1000);arrive();assert(last_timeout==first_timeout-2500);
  advance_clock(last_timeout);ChassisRoute_Process();assert(!strcmp(status().reason,"route_timeout") && stops==1);
  setup();char *fast[]={"chassis","route","start","5000"};assert(ChassisRoute_Command(4,fast));assert(last_timeout==60000);
  setup();fast[3]="5001";assert(ChassisRoute_Command(4,fast));assert(!starts && strstr(reply,"ERR"));
  assert(strstr(reply,"reason=speed_range") && !strcmp(status().state,"idle"));
  fast[3]="nan";assert(ChassisRoute_Command(4,fast));assert(!starts);
  assert(strstr(reply,"reason=format"));
  fast[3]="9";assert(ChassisRoute_Command(4,fast));assert(!starts);
  assert(strstr(reply,"reason=speed_range"));
  char *invalid[]={"chassis","route","oops"};assert(ChassisRoute_Command(3,invalid));
  assert(strstr(reply,"ERR") && strstr(reply,"reason=format") && !starts);
  assert(ChassisRoute_Command(2,invalid));assert(strstr(reply,"reason=format"));
  char *next[]={"chassis","route","next"};assert(ChassisRoute_Command(3,next));
  assert(strstr(reply,"reason=next_requires_waiting") && !starts);
  setup();assert(route_command("start"));unsigned running_starts=starts;
  fast[3]="200";assert(ChassisRoute_Command(4,fast));
  assert(strstr(reply,"ERR") && strstr(reply,"reason=route_reserved"));
  assert(starts==running_starts && status().segment==1 && !strcmp(status().state,"running"));
  assert(!strcmp(status().reason,"move"));
  setup();char *automatic[]={"chassis","route","auto","200"};assert(ChassisRoute_Command(4,automatic));
  for(unsigned i=1;i<=15;i++){
    assert(status().segment==i);finish_leg();assert(!strcmp(status().reason,"auto_continue"));unsigned count=starts;
    ChassisRoute_Process();assert(i==15?starts==count:starts>count);
  }
  assert(!ChassisRoute_IsBusy() && !strcmp(status().reason,"route_complete"));
  setup();assert(start_fixed_station_plan());assert(ChassisRoute_StationReserved());finish_leg();
  assert(!strcmp(status().state,"station") && status().station==5 && status().visit==1 && status().segment==1);
  assert(!ChassisRoute_IsBusy() && ChassisRoute_StationReserved());
  unsigned station_starts=starts;assert(ChassisRoute_Command(5,single));
  assert(starts==station_starts && strstr(reply,"route_reserved") && ChassisRoute_StationReserved());
  advance_clock(1200);ChassisRoute_Process();assert(starts==3 && !route_command("start"));
  assert(ChassisRoute_StationResume());finish_leg();advance_clock(100);ChassisRoute_Process();
  assert(status().segment==3 && ChassisRoute_StationReserved());
  while(!strcmp(status().state,"running")){arrive();assert(ChassisRoute_StationReserved());}
  assert(!strcmp(status().state,"station") && status().station==4 && status().visit==2 && status().target_yaw_deg==90);
  assert(!ChassisRoute_StationResume());assert(ChassisRoute_StationFinish());assert(!ChassisRoute_StationReserved());
  RadarPlan_t plan={0};plan.valid=true;plan.count=7;
  const RadarPlan_Point_t planned[]={
    {2250,150,1,0},{1800,150,0,0},{1800,800,0,0},{2100,1200,5,1},
    {2100,2050,0,0},{1200,2050,0,0},{1200,2100,4,2}};
  memcpy(plan.points,planned,sizeof(planned));setup();assert(ChassisRoute_PlanStart(&plan,200,true));
  plan.points[1].x_mm=999;assert(motion.target_x_mm==1800 && status().planned && status().total==6);
  for(unsigned i=0;i<3;i++){
    assert(!last_turn && motion.target_map_yaw_deg==(i==0?180:90));finish_leg();
    if(i<2){advance_clock(100);ChassisRoute_Process();}
  }
  assert(!strcmp(status().state,"station") && status().segment==3 && status().visit==1);
  assert(ChassisRoute_StationResume());
  for(unsigned i=0;i<3;i++){finish_leg();if(i<2){advance_clock(100);ChassisRoute_Process();}}
  assert(!strcmp(status().state,"station") && status().segment==6 && status().visit==2);assert(ChassisRoute_StationFinish());
  plan.count=4;plan.points[1]=(RadarPlan_Point_t){2100,1200,5,1};
  plan.points[2]=(RadarPlan_Point_t){1200,1200,0,0};plan.points[3]=(RadarPlan_Point_t){1200,1500,0,0};
  setup();assert(ChassisRoute_PlanStart(&plan,200,false));finish_leg();advance_clock(100);ChassisRoute_Process();
  assert(last_turn);arrive();feedback_arrival(0,-9,0);assert(!strcmp(status().state,"waiting"));
  advance_clock(100);ChassisRoute_Process();assert(status().segment==3 && !last_turn && motion.target_map_yaw_deg==180);
  assert(!strcmp(status().direction,"lateral"));finish_leg();
  plan.count=4;plan.points[1]=(RadarPlan_Point_t){2100,2100,0,0};
  plan.points[2]=(RadarPlan_Point_t){1200,2100,0,0};plan.points[3]=(RadarPlan_Point_t){800,2000,0,0};
  setup();assert(ChassisRoute_PlanStart(&plan,200,false));
  for(unsigned i=0;i<2;i++){finish_leg();advance_clock(100);ChassisRoute_Process();}
  assert(status().segment==3 && last_turn && fabsf(motion.target_map_yaw_deg+165.96376f)<0.01f);
  finish_leg();
  plan.count=2;plan.points[1]=(RadarPlan_Point_t){0,2400,0,0};
  setup();assert(ChassisRoute_PlanStart(&plan,10,false));assert(last_timeout==400000);
  puts("chassis_route_test: PASS (15 targets, phase sequence, stop gates, deadlines, station semantics, cancel)");
  return 0;
}

bool GrabTask_QrDepartureActive(void){return false;}
