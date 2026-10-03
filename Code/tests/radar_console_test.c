/* Production wire formatter and command admission; actuator/scan endpoints are
 * deterministic fixtures, so a query cannot accidentally start real hardware. */
#include "radar_console.h"
#include "radar_scan.h"
#include "chassis_route.h"
#include "chassis_localization.h"
#include "chassis_motion.h"
#include "console_tx.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static RadarScan_Status_t scan;
static RadarMap_t map;
static RadarMap_Params_t params,captured;
static RadarSample_t samples[360];
static ChassisLocalization_Status_t location;
static ChassisRoute_Status_t route;
static char reply[4096],page[257];
static char foreground[4][257];
static unsigned foreground_count;
static unsigned scans,stops,moves;
static float submitted_speed;
static bool busy;
static RadarUart_Probe_t probe;
static RadarUart_Status_t uart_status;
void RadarUart_ProbeGet(RadarUart_Probe_t *s){*s=probe;}
void RadarUart_StatusGet(RadarUart_Status_t *s){*s=uart_status;}
bool RadarUart_ProbeStart(void){
  if(probe.state==RADAR_PROBE_TX||probe.state==RADAR_PROBE_WAITING)return false;
  probe.id++;probe.state=RADAR_PROBE_TX;probe.reason="query_sending";return true;
}
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<=256);if(foreground_count<4){memcpy(foreground[foreground_count],d,n);foreground[foreground_count++][n]=0;}memcpy(reply,d,n);reply[n]=0;return true;}
bool ConsoleTx_BulkReady(void){return true;}
bool ConsoleTx_Bulk(const char *d,uint16_t n){assert(n<=256);memcpy(page,d,n);page[n]=0;puts(page);return true;}
void RadarScan_StatusGet(RadarScan_Status_t *s){*s=scan;}
const RadarMap_t *RadarScan_Map(void){return &map;}
const RadarMap_Params_t *RadarScan_Params(void){return &params;}
const RadarMap_Params_t *RadarScan_MapParams(void){return &captured;}
const RadarSample_t *RadarScan_Points(uint16_t *n){*n=360;return scan.points_valid?samples:NULL;}
bool RadarScan_Start(const RadarMap_Params_t *p,uint32_t origin){(void)origin;captured=*p;scans++;scan.state=RADAR_SCAN_COLLECTING;return true;}
void RadarScan_Stop(void){stops++;scan.state=RADAR_SCAN_STOPPED;}
bool RadarScan_SetParam(const char *key,float v){if(strcmp(key,"threshold") || v<1) return false;params.threshold=(uint16_t)v;scan.map_applicable=false;return true;}
void ChassisLocalization_Get(ChassisLocalization_Status_t *s){*s=location;}
void ChassisRoute_StatusGet(ChassisRoute_Status_t *s){*s=route;}
bool ChassisRoute_IsBusy(void){return busy;}
bool ChassisRoute_StationReserved(void){return false;}
bool ChassisMotion_IsBusy(void){return busy;}
bool Mecanum_IsBusy(void){return false;}
bool GrabRoute_IsBusy(void){return false;}
bool GrabTask_IsBusy(void){return false;}
uint8_t MechanicalArm_IsBusy(void){return 0;}
uint8_t ArmVision_IsBusy(void){return 0;}
uint8_t MaterialVision_IsBusy(void){return 0;}
bool HWT101_Cal_IsBusy(void){return false;}
bool ChassisRoute_PlanStart(const RadarPlan_t *p,float speed,bool station){assert(p->valid && !station);submitted_speed=speed;moves++;return true;}
bool GrabRoute_StartPlanned(const RadarPlan_t *p,float speed){assert(p->valid);submitted_speed=speed;moves++;return true;}
void GrabRoute_Stop(void){}
bool ChassisRoute_Cancel(void){stops++;return true;}
static void command(const char *s)
{
  char text[64],*t[6],*p;unsigned n=0;foreground_count=0;assert(strlen(s)<sizeof(text));strcpy(text,s);
  for(p=strtok(text," ");p && n<6;p=strtok(NULL," "))t[n++]=p;
  assert(RadarConsole_Command(n,t));
}
static void pump(void){tick+=1000;RadarConsole_Process();}
int main(void)
{
  RadarMap_Defaults(&params);captured=params;RadarMap_Reset(&map);
  scan.state=RADAR_SCAN_READY;scan.reason="complete";scan.map_valid=scan.map_applicable=scan.points_valid=true;
  scan.map_id=scan.scan_id=4;scan.origin_generation=7;scan.coverage_tenths=3600;
  location.generation=7;location.origin_valid=location.feedback_valid=true;
  location.observer.feedback.x_mm=2250;location.observer.feedback.y_mm=150;
  location.observer.feedback.yaw_rad=3.14159265358979323846f/2;
  route.state=route.reason="idle";
  for(unsigned i=0;i<360;i++)samples[i]=(RadarSample_t){(uint16_t)(i*10),65535,255};
  RadarConsole_Init();command("radar status");assert(!moves && !scans);
  assert(foreground_count==4 && strstr(foreground[1],"\"page\":0,\"pages\":3"));
  assert(strstr(foreground[2],"\"last_alarm\":0") && strstr(foreground[2],"\"unit\":\"unknown\""));
  command("radar probe");assert(!moves&&!scans&&strstr(foreground[0],"probe_accepted"));
  assert(strstr(foreground[1],"\"state\":\"transmitting\""));
  command("radar probe");assert(strstr(reply,"probe_busy_or_uart"));
  command("radar scan");assert(!scans&&strstr(reply,"query_in_progress"));
  probe.state=RADAR_PROBE_CONFIRMED;probe.reason="sn_reply";probe.tx_complete=true;
  strcpy(probe.sn,"ABC123456");probe.raw_count=6;memset(probe.raw,0xce,6);
  pump();assert(strstr(foreground[foreground_count-1],"\"sn\":\"ABC123456\""));
  command("radar probe status");assert(foreground_count==3&&!moves&&!scans);
  pump();command("radar fetch map 4 0");pump();assert(strstr(page,"\"mask\":328000"));
  assert(strstr(page,"\"pose\":[2170.0,230.0,180.0]"));
  for(unsigned i=1;i<6;i++){char cmd[64];snprintf(cmd,sizeof(cmd),"radar fetch map 4 %u",i);command(cmd);pump();assert(strstr(page,"\"counts\":["));}
  command("radar fetch points 4 119");pump();assert(strstr(page,"\"offset\":357") && strstr(page,"65535,255"));
  for(unsigned i=0;i<120;i++){char cmd[64];snprintf(cmd,sizeof(cmd),"radar fetch points 4 %u",i);command(cmd);pump();assert(strstr(page,"\"k\":\"points\""));}
  command("radar fetch path 1 0");pump();unsigned path_pages=0;
  assert(sscanf(strstr(page,"\"pages\":")+8,"%u",&path_pages)==1 && path_pages>0);
  for(unsigned i=1;i<path_pages;i++){char cmd[64];snprintf(cmd,sizeof(cmd),"radar fetch path 1 %u",i);command(cmd);pump();assert(strstr(page,"\"k\":\"path\""));}
  command("radar get");pump();assert(strstr(page,"\"key\":\"lidar_x_mm\""));
  for(unsigned i=1;i<10;i++){char cmd[64];snprintf(cmd,sizeof(cmd),"radar fetch params 0 %u",i);command(cmd);pump();assert(strstr(page,"\"k\":\"params\""));}
  command("radar nav start 50");assert(moves==1 && submitted_speed==50);
  location.generation++;command("radar nav start 50");assert(moves==1 && strstr(reply,"map_origin"));location.generation--;
  command("radar set threshold 4");assert(map.mask==RADAR_MAP_FIXED_MASK);command("radar nav start 50");assert(moves==1);
  scan.map_applicable=true;command("radar scan");assert(scans==1 && captured.zero_deg==180 && captured.lidar_x_mm==2170);
  command("radar stop");assert(stops==1 && map.mask==RADAR_MAP_FIXED_MASK);
  scan.coverage_tenths=70;command("radar fetch map 4 0");pump();assert(strstr(page,"\"coverage\":3600"));
  location.observer.feedback.x_mm=1200;location.observer.feedback.y_mm=1200;
  location.observer.feedback.yaw_rad=3.14159265358979323846f;
  command("radar scan");assert(scans==2 && fabsf(captured.lidar_x_mm-1120)<0.01f && fabsf(captured.lidar_y_mm-1120)<0.01f && fabsf(captured.zero_deg-270)<0.01f);
  command("radar stop");location.observer.feedback.x_mm=2250;location.observer.feedback.y_mm=150;
  location.observer.feedback.yaw_rad=3.14159265358979323846f/2;
  busy=true;command("radar scan");assert(scans==2 && strstr(reply,"stationary"));busy=false;
  command("radar fetch map 3 0");pump();assert(strstr(reply,"snapshot_changed"));
  assert(moves==1);
  scan.origin_generation=0;scan.map_valid=false;scan.state=RADAR_SCAN_IDLE;
  command("radar scan");unsigned before=stops;pump();assert(stops==before);
  location.generation++;pump();assert(stops==before+1);
  /* Full uint32 identity values still need to fit the DL20 wire-page budget. */
  tick=UINT32_MAX-1000U;scan.state=RADAR_SCAN_READY;scan.map_valid=scan.map_applicable=scan.points_valid=true;
  scan.map_id=scan.scan_id=scan.origin_generation=location.generation=UINT32_MAX;
  RadarConsole_Init();reply[0]=page[0]=0;
  command("radar fetch points 4294967295 89");pump();
  assert(strstr(page,"\"k\":\"points\"") && !strstr(reply,"page_size"));
  command("radar nav start");assert(moves==2 && submitted_speed==200);
  command("radar task start");assert(moves==3 && submitted_speed==200);
  route.planned=true;route.state="running";route.reason="dock_approach";
  route.segment=193;route.total=193;route.station=5;route.visit=8;
  command("radar nav status");
  assert(strlen(reply)<=256 && strstr(reply,"\"k\":\"nav\"") && strstr(reply,"\"reason\":\"dock_approach\""));
  assert(strstr(reply,"\"index\":193,\"total\":193,\"station\":5,\"visit\":8,\"stopped\":false"));
  scan.bytes=scan.packets=scan.bad=scan.overflow=scan.alarm=UINT32_MAX;
  scan.last_alarm=UINT16_MAX;
  command("radar status");
  assert(foreground_count==4 && !strstr(reply,"page_size"));
  assert(strstr(foreground[1],"\"bytes\":4294967295") && strstr(foreground[1],"\"overflow\":4294967295"));
  assert(strstr(foreground[2],"\"alarm\":4294967295") && strstr(foreground[2],"\"last_alarm\":65535"));
  uart_status.commands_sent=uart_status.tx_errors=uart_status.uart_errors=UINT32_MAX;
  uart_status.rx_ready=true;strcpy(uart_status.last_tx,"LSTARH");
  command("radar status");assert(strstr(foreground[3],"\"last_tx\":\"LSTARH\""));
  assert(strstr(foreground[3],"\"tx_done\":4294967295")&&!strstr(reply,"page_size"));
  probe.id=probe.rx_bytes=probe.alarms=UINT32_MAX;probe.last_alarm=UINT16_MAX;
  memset(probe.sn,'Z',24);probe.sn[24]=0;probe.raw_count=24;memset(probe.raw,0xff,24);
  probe.state=RADAR_PROBE_UNCONFIRMED;probe.reason="reply_unknown";
  command("radar probe status");assert(foreground_count==3&&!strstr(reply,"page_size"));
  assert(strstr(foreground[2],"FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"));
  puts("radar_console_test: PASS (paging, pose, first scan, no implicit drive, origin and parameter validity)");
}
