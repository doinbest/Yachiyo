#include "chassis_route.h"
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
static uint32_t last_timeout;
static uint32_t tick;
static bool accept=true;
static char reply[400];
uint32_t HAL_GetTick(void){return tick;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *s){*s=loc;}
void ChassisMotion_StatusGet(ChassisMotion_Status_t *s){*s=motion;}
void Mecanum_StatusGet(Mecanum_Status_t *s){*s=bus;}
bool ChassisMotion_IsBusy(void){return motion.state==CHASSIS_MOTION_RUNNING||motion.state==CHASSIS_MOTION_STOPPING;}
bool ChassisMotion_MoveTo(float x,float y,float yaw,float speed,uint32_t timeout)
{
  assert(speed>=10&&speed<=5000&&timeout>=60000&&timeout<=400000);last_speed=speed;last_timeout=timeout;assert(!ChassisRoute_IsBusy()||ChassisRoute_IsSubmitting());
  if(!accept){motion.reason="rejected";return false;}
  starts++;motion.action_id++;motion.state=CHASSIS_MOTION_RUNNING;motion.reason="running";
  motion.target_x_mm=x;motion.target_y_mm=y;motion.target_map_yaw_deg=yaw;motion.stop_confirmed=false;return true;
}
bool ChassisMotion_Stop(uint32_t ms){assert(ms==0);stops++;motion.state=CHASSIS_MOTION_STOPPING;return true;}
bool ConsoleTx_Write(const uint8_t*d,uint16_t n){assert(n<sizeof(reply));memcpy(reply,d,n);reply[n]=0;return true;}
static ChassisRoute_Status_t status(void){ChassisRoute_Status_t s;ChassisRoute_StatusGet(&s);return s;}
static void setup(void)
{
  memset(&loc,0,sizeof(loc));memset(&motion,0,sizeof(motion));memset(&bus,0,sizeof(bus));
  motion.reason="idle";loc.origin_valid=loc.feedback_valid=loc.speed_valid=true;
  loc.origin.x_mm=loc.observer.feedback.x_mm=2250;loc.origin.y_mm=loc.observer.feedback.y_mm=150;
  loc.origin.yaw_rad=loc.observer.feedback.yaw_rad=CHASSIS_MODEL_PI/2;
  loc.generation=4;tick=100;loc.feedback_tick=tick;starts=stops=0;accept=true;ChassisRoute_Init();
}
static void arrive(void)
{
  loc.observer.feedback.x_mm=motion.target_x_mm;loc.observer.feedback.y_mm=motion.target_y_mm;
  motion.state=CHASSIS_MOTION_DONE;motion.stop_confirmed=true;motion.reason="feedback_arrived";ChassisRoute_Process();
}
int main(void)
{
  const float points[16][2]={{2100,300},{2100,1200},{2100,2100},{1200,2100},{1200,350},{1200,1200},{350,1200},{1200,1200},{1200,2100},{1200,350},{1200,1200},{350,1200},{1200,1200},{2100,1200},{2100,300},{2250,150}};
  setup();assert(!ChassisRoute_Next());assert(ChassisRoute_Start());assert(last_speed==CHASSIS_ROUTE_DEFAULT_SPEED_MM_S);assert(!ChassisRoute_Start());
  for(unsigned i=0;i<16;i++)
  {
    assert(status().segment==i+1);assert(motion.target_x_mm==points[i][0]&&motion.target_y_mm==points[i][1]);
    assert(!ChassisRoute_Next());arrive();assert(starts==i+1);
    for(unsigned j=0;j<10;j++) { ChassisRoute_Process(); }
    assert(starts==i+1);
    if(i<15){assert(ChassisRoute_IsBusy());assert(!strcmp(status().state,"waiting"));assert(ChassisRoute_Next());}
  }
  assert(!ChassisRoute_IsBusy());assert(!strcmp(status().state,"done"));assert(!ChassisRoute_Next());
  assert(loc.origin.x_mm==2250&&loc.origin.y_mm==150); /* never snap/anchor each stop */
  setup();loc.origin_valid=false;assert(!ChassisRoute_Start());assert(!starts);
  setup();loc.origin.x_mm=2249;assert(!ChassisRoute_Start());
  setup();assert(ChassisRoute_Start());motion.action_id++;ChassisRoute_Process();assert(!strcmp(status().reason,"action_mismatch")&&stops==1);
  setup();assert(ChassisRoute_Start());motion.state=CHASSIS_MOTION_DONE;motion.reason="timed_complete";ChassisRoute_Process();assert(!strcmp(status().state,"error"));
  setup();assert(ChassisRoute_Start());arrive();assert(ChassisRoute_Cancel());assert(!ChassisRoute_Next());motion.state=CHASSIS_MOTION_DONE;ChassisRoute_Process();assert(!strcmp(status().state,"cancelled"));
  setup();assert(ChassisRoute_Start());arrive();loc.generation++;assert(!ChassisRoute_Next());assert(!status().stop_confirmed&&stops==1);
  motion.state=CHASSIS_MOTION_DONE;ChassisRoute_Process();assert(!strcmp(status().state,"error"));
  setup();assert(ChassisRoute_Start());arrive();loc.speed_valid=false;ChassisRoute_Process();assert(stops==1&&!status().stop_confirmed);
  setup();assert(ChassisRoute_Start());arrive();bus.locked=true;ChassisRoute_Process();assert(stops==1&&!status().stop_confirmed);
  setup();accept=false;assert(!ChassisRoute_Start());assert(!ChassisRoute_IsBusy());
  setup();char *move[]={"chassis","move","-150","150"};assert(ChassisRoute_Command(4,move));assert(starts==1&&last_speed==CHASSIS_DISTANCE_SPEED_MM_S);assert(motion.target_x_mm==2100&&motion.target_y_mm==300);
  setup();char *bad[]={"chassis","move","250","250"};assert(ChassisRoute_Command(4,bad));assert(starts==0&&strstr(reply,"ERR"));
  setup();assert(ChassisRoute_Start());char *query[]={"chassis","route","status"};assert(ChassisRoute_Command(3,query));assert(starts==1&&strstr(reply,"segment=1"));
  assert(ChassisRoute_Command(4,move));assert(starts==1);char *stop[]={"chassis","stop"};assert(ChassisRoute_Command(2,stop));assert(stops==1);
  setup();char *fast[]={"chassis","route","start","5000"};
  assert(ChassisRoute_Command(4,fast));assert(starts==1 && last_speed==5000 && strstr(reply,"speed_mm_s=5000"));
  fast[3]="20";assert(ChassisRoute_Command(4,fast));assert(starts==1 && strstr(reply,"ERR"));
  arrive();assert(ChassisRoute_Next());assert(last_speed==5000);
  setup();fast[3]="10";assert(ChassisRoute_Command(4,fast));arrive();assert(ChassisRoute_Next());assert(last_speed==10 && last_timeout>=180000);
  setup();fast[3]="5001";assert(ChassisRoute_Command(4,fast));assert(!starts&&strstr(reply,"ERR"));
  fast[3]="nan";assert(ChassisRoute_Command(4,fast));assert(!starts);
  fast[3]="9";assert(ChassisRoute_Command(4,fast));assert(!starts);
  setup();char *auto_run[]={"chassis","route","auto","5000"};
  assert(ChassisRoute_Command(4,auto_run));assert(strstr(reply,"auto=1") && strstr(reply,"dwell_ms=100") && starts==1);
  for(unsigned i=0;i<16;i++){
    assert(motion.target_x_mm==points[i][0]&&motion.target_y_mm==points[i][1]);
    arrive();assert(!strcmp(status().reason,"auto_dwell"));assert(!ChassisRoute_Next());
    tick+=CHASSIS_ROUTE_AUTO_DWELL_MS-1;loc.feedback_tick=tick;ChassisRoute_Process();assert(starts==i+1);
    tick++;loc.feedback_tick=tick;ChassisRoute_Process();
    assert(starts==(i<15?i+2:16));assert(last_speed==5000);
  }
  assert(!ChassisRoute_IsBusy()&&!strcmp(status().reason,"route_complete"));
  setup();assert(ChassisRoute_Command(4,auto_run));arrive();assert(ChassisRoute_Cancel());
  motion.state=CHASSIS_MOTION_DONE;tick+=1500;loc.feedback_tick=tick;ChassisRoute_Process();assert(starts==1);
  setup();assert(ChassisRoute_Command(4,auto_run));arrive();tick+=1000;loc.feedback_valid=false;ChassisRoute_Process();assert(starts==1&&stops==1);
  setup();assert(ChassisRoute_Command(4,auto_run));
  motion.state=CHASSIS_MOTION_RUNNING;motion.reason="position_correcting";ChassisRoute_Process();
  assert(starts==1 && !strcmp(status().reason,"position_correcting") && !status().stop_confirmed);
  motion.state=CHASSIS_MOTION_ERROR;motion.reason="position_not_reached";motion.stop_confirmed=true;
  ChassisRoute_Process();assert(starts==1 && status().stop_confirmed && !strcmp(status().state,"error"));
  setup(); assert(ChassisRoute_StationStart(50));
  arrive(); tick+=100;loc.feedback_tick=tick;ChassisRoute_Process();
  assert(starts==2); arrive(); assert(!strcmp(status().state,"station"));
  assert(!ChassisRoute_IsBusy() && ChassisRoute_StationReserved());
  tick+=1200;loc.feedback_tick=tick;ChassisRoute_Process();assert(starts==2);
  assert(!ChassisRoute_Start() && !ChassisRoute_Next());
  assert(ChassisRoute_StationResume()); assert(starts==3);
  arrive();tick+=100;loc.feedback_tick=tick;ChassisRoute_Process();assert(starts==4);
  arrive();assert(!strcmp(status().state,"station") && status().segment==4);
  assert(!ChassisRoute_StationResume());assert(ChassisRoute_StationFinish());
  tick+=1000;loc.feedback_tick=tick;ChassisRoute_Process();assert(starts==4);
  assert(!ChassisRoute_StationReserved());
  setup();assert(ChassisRoute_StationStart(50));arrive();tick+=100;loc.feedback_tick=tick;ChassisRoute_Process();arrive();
  assert(ChassisRoute_Cancel());assert(!ChassisRoute_StationResume());
  puts("chassis_route_test: PASS (manual, automatic, station handoff, cancellation)");return 0;
}
