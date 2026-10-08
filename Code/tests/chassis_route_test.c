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
static unsigned starts,stops;
static float last_speed;
static uint32_t last_timeout,tick;
static bool accept=true,last_turn;
bool GrabTask_IsBusy(void){return false;}
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
  if(!accept){motion.reason="rejected";return false;}
  starts++;last_speed=speed;last_timeout=timeout;last_turn=turn;
  motion.action_id++;motion.state=CHASSIS_MOTION_RUNNING;motion.reason="running";
  motion.target_x_mm=x;motion.target_y_mm=y;motion.target_map_yaw_deg=yaw;motion.stop_confirmed=false;
  return true;
}
bool ChassisMotion_MoveTo(float x,float y,float yaw,float speed,uint32_t timeout)
{ return action(x,y,yaw,speed,timeout,false); }
bool ChassisMotion_TurnTo(float yaw,uint32_t timeout)
{ return action(loc.observer.feedback.x_mm,loc.observer.feedback.y_mm,yaw,0,timeout,true); }
bool ChassisMotion_Stop(uint32_t ms){assert(ms==0);stops++;motion.state=CHASSIS_MOTION_STOPPING;return true;}
bool ConsoleTx_Write(const uint8_t*d,uint16_t n){assert(n<sizeof(reply));memcpy(reply,d,n);reply[n]=0;return true;}
static ChassisRoute_Status_t status(void){ChassisRoute_Status_t s;ChassisRoute_StatusGet(&s);return s;}
static void advance_clock(uint32_t ms){tick+=ms;loc.feedback_tick=tick;}
static void setup(void)
{
  memset(&loc,0,sizeof(loc));memset(&motion,0,sizeof(motion));memset(&bus,0,sizeof(bus));
  motion.reason="idle";loc.origin_valid=loc.feedback_valid=loc.speed_valid=true;
  loc.origin.x_mm=loc.observer.feedback.x_mm=2250;loc.origin.y_mm=loc.observer.feedback.y_mm=150;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;
  loc.generation=4;tick=100;loc.feedback_tick=tick;starts=stops=0;accept=true;last_turn=false;
  ChassisRoute_Init();
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
  while(status().segment<segment){finish_leg();assert(ChassisRoute_Next());}
}
static void expect_cancel(void)
{
  unsigned before=starts;assert(ChassisRoute_Cancel());
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;
  advance_clock(1000);ChassisRoute_Process();assert(starts==before);
  assert(!strcmp(status().state,"cancelled"));assert(!ChassisRoute_Next());
}
int main(void)
{
  setup();assert(ChassisRoute_StationStart(200));
  assert(status().segment==1 && motion.target_x_mm==2100 && motion.target_y_mm==300);
  assert(status().target_y_mm==1200 && !strcmp(status().direction,"diagonal"));
  arrive();
  assert(!strcmp(status().state,"running") && status().segment==1);
  assert(starts==2 && motion.target_x_mm==2100 && motion.target_y_mm==1200);
  assert(!strcmp(status().direction,"forward") && !ChassisRoute_StationResume());
  arrive();assert(!strcmp(status().state,"station") && status().station==5 && status().visit==1);
  setup();assert(!ChassisRoute_Next());assert(ChassisRoute_Start());
  assert(status().total==15 && last_speed==CHASSIS_ROUTE_DEFAULT_SPEED_MM_S && !last_turn);
  assert(!ChassisRoute_Start());
  for(unsigned i=1;i<=15;i++){
    RadarPlan_Point_t p;assert(ChassisRouteProfile_FixedPoint(i,&p));
    assert(status().segment==i && status().station==p.station && status().visit==p.visit);
    assert(status().target_x_mm==p.x_mm && status().target_y_mm==p.y_mm);
    assert(!ChassisRoute_Next());finish_leg();unsigned count=starts;
    for(unsigned j=0;j<10;j++){ChassisRoute_Process();}assert(starts==count);
    if(i<15){assert(ChassisRoute_IsBusy());assert(ChassisRoute_Next());}
  }
  assert(!ChassisRoute_IsBusy() && !strcmp(status().reason,"route_complete"));
  assert(loc.origin.x_mm==2250 && loc.origin.y_mm==150 && loc.generation==4);
  setup();assert(ChassisRoute_Start());to_segment(3);
  assert(last_turn && !strcmp(status().phase,"turn") && motion.target_map_yaw_deg==180);
  unsigned n=starts;arrive();assert(starts==n+1 && !last_turn && !strcmp(status().direction,"forward"));
  assert(status().target_yaw_deg==90 && status().phase_target_yaw_deg==180);
  arrive();assert(starts==n+2 && last_turn && !strcmp(status().phase,"arrival_turn"));
  uint32_t turn_timeout=last_timeout;advance_clock(1000);feedback_arrival(25,0,0);
  assert(starts==n+3 && !last_turn && last_speed<=50 && last_timeout==turn_timeout-1000);
  assert(status().segment==3 && status().target_x_mm==1200 && !strcmp(status().phase,"move"));
  arrive();assert(!strcmp(status().state,"waiting") && status().stop_confirmed);
  setup();assert(ChassisRoute_Start());to_segment(6);
  assert(last_turn && motion.target_map_yaw_deg==180);arrive();
  assert(!last_turn && motion.target_x_mm==425 && !strcmp(status().phase,"dock_approach"));
  arrive();assert(last_turn && motion.target_x_mm==425 && motion.target_map_yaw_deg==90);arrive();
  assert(!last_turn && motion.target_x_mm==350 && last_speed==50 && !strcmp(status().phase,"dock"));
  arrive();assert(!strcmp(status().state,"waiting"));assert(ChassisRoute_Next());
  assert(!last_turn && motion.target_x_mm==425 && motion.target_map_yaw_deg==90 && last_speed==50 && !strcmp(status().phase,"undock"));
  arrive();assert(last_turn && motion.target_x_mm==425 && motion.target_map_yaw_deg==180);
  arrive();assert(!last_turn && !strcmp(status().direction,"reverse"));finish_leg();
  const char *cancel_phases[]={"turn","dock_approach","arrival_turn","dock"};
  for(unsigned phase=0;phase<4;phase++){
    setup();assert(ChassisRoute_Start());to_segment(6);
    for(unsigned j=0;j<phase;j++){arrive();}assert(!strcmp(status().phase,cancel_phases[phase]));expect_cancel();
  }
  for(unsigned phase=0;phase<4;phase++){
    setup();assert(ChassisRoute_Start());to_segment(6);
    for(unsigned j=0;j<phase;j++){arrive();}
    unsigned count=starts;motion.state=CHASSIS_MOTION_DONE;motion.reason="feedback_arrived";
    motion.stop_confirmed=false;ChassisRoute_Process();
    assert(starts==count && !strcmp(status().reason,"arrival_unconfirmed"));
  }
  setup();assert(ChassisRoute_Start());to_segment(7);assert(!strcmp(status().phase,"undock"));expect_cancel();
  setup();assert(ChassisRoute_Start());assert(!strcmp(status().phase,"move"));expect_cancel();
  setup();loc.origin_valid=false;assert(!ChassisRoute_Start() && !starts);
  setup();loc.origin.x_mm=2249;assert(!ChassisRoute_Start());
  setup();assert(ChassisRoute_Start());motion.action_id++;ChassisRoute_Process();assert(!strcmp(status().reason,"action_mismatch") && stops==1);
  setup();assert(ChassisRoute_Start());motion.state=CHASSIS_MOTION_DONE;motion.reason="timed_complete";ChassisRoute_Process();assert(!strcmp(status().reason,"arrival_unconfirmed") && starts==1);
  setup();assert(ChassisRoute_Start());motion.state=CHASSIS_MOTION_DONE;motion.reason="feedback_arrived";motion.stop_confirmed=false;ChassisRoute_Process();assert(!strcmp(status().state,"error") && starts==1);
  setup();assert(ChassisRoute_Start());finish_leg();loc.generation++;assert(!ChassisRoute_Next() && stops==1 && !status().stop_confirmed);
  setup();assert(ChassisRoute_Start());finish_leg();loc.speed_valid=false;ChassisRoute_Process();assert(stops==1);
  setup();assert(ChassisRoute_Start());finish_leg();bus.locked=true;ChassisRoute_Process();assert(stops==1);
  setup();accept=false;assert(!ChassisRoute_Start() && !ChassisRoute_IsBusy());
  setup();char *move[]={"chassis","move","-150","150"};assert(ChassisRoute_Command(4,move));assert(starts==1 && motion.target_x_mm==2100 && motion.target_y_mm==300 && last_speed==CHASSIS_DISTANCE_SPEED_MM_S);
  setup();char *bad[]={"chassis","move","250","250"};assert(ChassisRoute_Command(4,bad));assert(!starts && strstr(reply,"ERR"));
  setup();assert(ChassisRoute_Start());char *query[]={"chassis","route","status"};assert(ChassisRoute_Command(3,query));assert(strstr(reply,"segment=1") && strstr(reply,"total=15") && strstr(reply,"phase=move"));
  char *stop[]={"chassis","stop"};assert(ChassisRoute_Command(2,stop));assert(stops==1);
  setup();char *slow[]={"chassis","route","start","10"};assert(ChassisRoute_Command(4,slow));
  assert(last_timeout>232000 && last_timeout<233000);finish_leg();assert(ChassisRoute_Next());finish_leg();assert(ChassisRoute_Next());
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
  setup();assert(ChassisRoute_Start());unsigned running_starts=starts;
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
  setup();assert(ChassisRoute_StationStart(200));assert(ChassisRoute_StationReserved());finish_leg();
  assert(!strcmp(status().state,"station") && status().station==5 && status().visit==1 && status().segment==1);
  assert(!ChassisRoute_IsBusy() && ChassisRoute_StationReserved());
  advance_clock(1200);ChassisRoute_Process();assert(starts==2 && !ChassisRoute_Start());
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
    assert(!last_turn && motion.target_map_yaw_deg==90);finish_leg();
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
