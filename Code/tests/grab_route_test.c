#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "GrabRoute.h"
#include "GrabTask.h"
#include "chassis_route.h"
#include "contest_screen.h"
#include "QR.h"
#include "console_tx.h"
static uint32_t tick;
static ChassisRoute_Status_t route;
static unsigned starts,resumes,finishes,picks,cancels;
static int ready=1,screen_done,screen_error,task_busy;
static char reply[600];
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<sizeof(reply));memcpy(reply,d,n);reply[n]=0;return true;}
bool GrabTask_ConfigReady(const char *mode){assert(!strcmp(mode,"pick"));return ready;}
bool GrabTask_IsBusy(void){return task_busy;}
bool GrabTask_Start(const char *mode){assert(!strcmp(mode,"pick"));picks++;task_busy=1;return true;}
void GrabTask_Stop(void){task_busy=0;}
bool ChassisRoute_StationStart(float speed){assert(speed==50);starts++;route.segment=1;route.state="running";return true;}
bool ChassisRoute_StationResume(void){resumes++;route.segment=3;route.state="running";return true;}
bool ChassisRoute_StationFinish(void){finishes++;route.state="done";return true;}
bool ChassisRoute_Cancel(void){cancels++;route.state="cancelled";return true;}
bool ChassisRoute_IsBusy(void){return !strcmp(route.state,"running");}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *s){*s=route;}
uint8_t ContestScreen_TaskCodeRefresh(const char *s){assert(!strcmp(s,"156+123+516+231"));return 1;}
uint8_t ContestScreen_ProgressSet(uint8_t a,uint8_t b){assert(!a&&!b);return 1;}
uint8_t ContestScreen_TaskCodeSent(void){return screen_done;}
uint8_t ContestScreen_HasError(void){return screen_error;}
static void setup(void){memset(&route,0,sizeof(route));route.state="idle";route.reason="none";tick=0;starts=resumes=finishes=picks=cancels=0;ready=1;screen_done=screen_error=task_busy=0;QR_Init();GrabRoute_Init();}
int main(void)
{
  char *start[]={"grab","route","50"};QR_SnapshotTypeDef qr;
  setup();ready=0;assert(GrabRoute_Command(3,start));assert(!starts);
  setup();assert(GrabRoute_Command(3,start));assert(starts==1&&GrabRoute_IsBusy());
  route.state="stopping";route.reason="position_reached";GrabRoute_Process();
  assert(GrabRoute_IsBusy() && !cancels); /* normal arrival must await stop feedback */
  route.state="station";route.segment=2;route.stop_confirmed=true;GrabRoute_Process();
  tick=999;GrabRoute_Process();QR_SnapshotGet(&qr);assert(!qr.Valid&&!resumes);
  tick=1000;GrabRoute_Process();QR_SnapshotGet(&qr);assert(qr.Valid&&qr.Source==QR_SOURCE_SIMULATED&&!qr.Received&&!qr.Accepted);
  assert(QR_ColorGet(0,0)==1&&!resumes);tick+=1000;GrabRoute_Process();assert(!resumes);
  screen_done=1;GrabRoute_Process();assert(resumes==1);
  route.state="station";route.segment=4;GrabRoute_Process();assert(finishes==1&&picks==1&&!GrabRoute_IsBusy());
  GrabRoute_Process();assert(picks==1);
  setup();GrabRoute_Command(3,start);route.state="station";route.segment=2;route.stop_confirmed=true;GrabRoute_Process();tick=1000;GrabRoute_Process();screen_error=1;GrabRoute_Process();assert(!resumes&&!picks&&!GrabRoute_IsBusy()&&cancels);
  setup();GrabRoute_Command(3,start);GrabRoute_Stop();tick=9999;GrabRoute_Process();assert(!picks&&!resumes&&!GrabRoute_IsBusy());
  puts("grab_route_test: PASS (missing config, 1000ms, TX gate, handoff, error, cancel)");
}
