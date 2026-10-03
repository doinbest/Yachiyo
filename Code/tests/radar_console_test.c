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
#include <stdlib.h>
#include <string.h>
static uint32_t tick;
static RadarScan_Status_t scan;
static RadarScan_StartStatus_t start_status;
static RadarMap_t map;
static RadarMap_Params_t params,captured;
static RadarSample_t samples[360];
static ChassisLocalization_Status_t location;
static ChassisRoute_Status_t route;
static char reply[4096],page[257];
static char foreground[6][257];
static unsigned foreground_count;
static unsigned scans,auto_scans,stops,moves;
static float submitted_speed;
static bool busy,bulk_ready=true;
#ifdef RADAR_CONSOLE_PLAN_FIXTURE
static bool edge_plan;
bool RadarPlan_FixtureBuildFromStart(const RadarMap_t *m,RadarStart_Id_t start,RadarPlan_t *p)
{
  if(!edge_plan) return RadarPlan_BuildFromStart(m,start,p);
  memset(p,0,sizeof(*p));p->valid=true;p->count=RADAR_PLAN_MAX_POINTS;p->start_id=(uint8_t)start;
  for(unsigned i=0;i<p->count;i++)p->points[i]=(RadarPlan_Point_t){INT16_MIN,INT16_MAX,UINT8_MAX,UINT8_MAX};
  return true;
}
#endif
static RadarUart_Probe_t probe;
static RadarUart_Status_t uart_status;
void RadarUart_ProbeGet(RadarUart_Probe_t *s){*s=probe;}
void RadarUart_StatusGet(RadarUart_Status_t *s){*s=uart_status;}
bool RadarUart_ProbeStart(void){
  if(probe.state==RADAR_PROBE_TX||probe.state==RADAR_PROBE_WAITING)return false;
  probe.id++;probe.state=RADAR_PROBE_TX;probe.reason="query_sending";return true;
}
uint32_t HAL_GetTick(void){return tick;}
bool ConsoleTx_Write(const uint8_t *d,uint16_t n){assert(n<=256);if(foreground_count<6){memcpy(foreground[foreground_count],d,n);foreground[foreground_count++][n]=0;}memcpy(reply,d,n);reply[n]=0;return true;}
bool ConsoleTx_BulkReady(void){return bulk_ready;}
bool ConsoleTx_Bulk(const char *d,uint16_t n){assert(n<=256);memcpy(page,d,n);page[n]=0;puts(page);return true;}
void RadarScan_StatusGet(RadarScan_Status_t *s){*s=scan;}
void RadarScan_StartStatusGet(RadarScan_StartStatus_t *s){*s=start_status;}
const RadarMap_t *RadarScan_Map(void){return &map;}
const RadarMap_Params_t *RadarScan_Params(void){return &params;}
const RadarMap_Params_t *RadarScan_MapParams(void){return &captured;}
const RadarSample_t *RadarScan_Points(uint16_t *n){*n=360;return scan.points_valid?samples:NULL;}
bool RadarScan_Start(const RadarMap_Params_t *p,uint32_t origin){(void)origin;captured=*p;scans++;scan.state=RADAR_SCAN_COLLECTING;return true;}
bool RadarScan_StartAuto(const RadarMap_Params_t *p,uint32_t origin){auto_scans++;return RadarScan_Start(p,origin);}
bool RadarScan_SelectStart(RadarStart_Id_t start){
  if(!start_status.scan_id || !start_status.automatic || RadarConsole_ScanBusy() || start==RADAR_START_UNKNOWN)return false;
  start_status.result.start_id=(uint8_t)start;start_status.result.reason="manual_selection";
  scan.map_id++;scan.points_map_id=scan.map_id;scan.map_applicable=true;scan.reason="complete";return true;
}
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
static void fast_pump(void){tick++;RadarConsole_Process();}
static unsigned page_field(const char *key)
{
  char name[32];unsigned value;
  snprintf(name,sizeof(name),"\"%s\":",key);
  const char *at=strstr(page,name);assert(at && sscanf(at+strlen(name),"%u",&value)==1);
  return value;
}
static void fetch(const char *kind,uint32_t id,unsigned number)
{
  char text[64];snprintf(text,sizeof(text),"radar fetch %s %lu %u",kind,(unsigned long)id,number);
  page[0]=reply[0]=0;command(text);fast_pump();
  assert(page[0] && strlen(page)<=256 && !strstr(reply,"page_size"));
  assert(page_field("page")==number);
}
/* These checks catch omitted/overlapping items when page capacity changes. */
static unsigned fetch_map(uint32_t id)
{
  unsigned seen=0;fetch("map",id,0);unsigned pages=page_field("pages");
  assert(strstr(page,"\"mask\":328000") && strstr(page,"\"pose\":[2170.0,230.0,180.0]"));
  for(unsigned number=1;number<pages;number++){
    fetch("map",id,number);assert(page_field("pages")==pages && page_field("offset")==seen);
    const char *at=strstr(page,"\"counts\":[");assert(at);at+=10;
    while(*at!=']'){
      char *end;unsigned long value=strtoul(at,&end,10);
      assert(end!=at && seen<RADAR_MAP_CELL_COUNT && value==map.counts[seen++]);at=end;
      if(*at==',')at++;else assert(*at==']');
    }
  }
  assert(seen==RADAR_MAP_CELL_COUNT);return pages;
}
static unsigned fetch_points(uint32_t id)
{
  unsigned seen=0,pages=0;
  for(unsigned number=0;!number || number<pages;number++){
    fetch("points",id,number);
    if(!number)pages=page_field("pages");else assert(page_field("pages")==pages);
    assert(page_field("offset")==seen && page_field("total")==360);
    const char *at=strstr(page,"\"points\":[");assert(at);at+=10;
    while(*at!=']'){
      unsigned angle,distance,energy;int used=0;
      assert(sscanf(at,"[%u,%u,%u]%n",&angle,&distance,&energy,&used)==3 && used>0 && seen<360);
      assert(angle==samples[seen].angle_tenths && distance==65535 && energy==255);seen++;at+=used;
      if(*at==',')at++;else assert(*at==']');
    }
  }
  assert(seen==360);return pages;
}
static unsigned fetch_path(uint32_t id,bool extremes)
{
  unsigned seen=0,pages=0;RadarPlan_t expected;assert(RadarPlan_Build(&map,&expected));
  for(unsigned number=0;!number || number<pages;number++){
    fetch("path",id,number);
    if(!number)pages=page_field("pages");else assert(page_field("pages")==pages);
    assert(page_field("offset")==seen);
    assert(page_field("start")==RADAR_START_LOWER);
    const char *at=strstr(page,"\"points\":[");assert(at);at+=10;
    while(*at!=']'){
      int x,y,used=0;unsigned station,visit;
      assert(sscanf(at,"[%d,%d,%u,%u]%n",&x,&y,&station,&visit,&used)==4 && used>0);
      if(extremes){assert(seen<RADAR_PLAN_MAX_POINTS && x==INT16_MIN && y==INT16_MAX && station==255 && visit==255);}
      else {assert(seen<expected.count && x==expected.points[seen].x_mm && y==expected.points[seen].y_mm && station==expected.points[seen].station && visit==expected.points[seen].visit);}
      seen++;at+=used;if(*at==',')at++;else assert(*at==']');
    }
  }
  assert(seen==(extremes?RADAR_PLAN_MAX_POINTS:expected.count));return pages;
}
int main(void)
{
  RadarMap_Defaults(&params);captured=params;RadarMap_Reset(&map);
  scan.state=RADAR_SCAN_READY;scan.reason="complete";scan.map_valid=scan.map_applicable=scan.points_valid=true;
  scan.map_id=scan.scan_id=4;scan.origin_generation=7;scan.coverage_tenths=3600;
  scan.points_map_id=4;start_status.scan_id=4;start_status.result.start_id=RADAR_START_LOWER;start_status.result.reason="manual_reference";
  location.generation=7;location.origin_valid=location.feedback_valid=true;
  location.observer.feedback.x_mm=2250;location.observer.feedback.y_mm=150;
  location.observer.feedback.yaw_rad=3.14159265358979323846f/2;
  route.state=route.reason="idle";
  for(unsigned i=0;i<360;i++)samples[i]=(RadarSample_t){(uint16_t)(i*10),65535,255};
  RadarConsole_Init();command("radar status");assert(!moves && !scans);
  assert(foreground_count==6 && strstr(foreground[1],"\"page\":0,\"pages\":3"));
  assert(strstr(foreground[2],"\"last_alarm\":0") && strstr(foreground[2],"\"unit\":\"unknown\""));
  assert(strstr(foreground[4],"\"k\":\"start\"") && strstr(foreground[4],"\"selected\":2") && strstr(foreground[5],"\"conflicts\":[0,0]"));
  command("radar start status");assert(foreground_count==2 && !moves && !scans);
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
  /* A ready next request must not wait for the unsolicited-report timer. */
  command("radar fetch map 4 1");page[0]=0;fast_pump();assert(strstr(page,"\"counts\":["));
  bulk_ready=false;command("radar fetch map 4 0");page[0]=0;fast_pump();assert(!page[0]);
  command("radar fetch map 4 1");assert(strstr(reply,"page_pending"));
  bulk_ready=true;fast_pump();assert(strstr(page,"\"page\":0,"));assert(!moves && !scans);
  for(unsigned i=0;i<RADAR_MAP_CELL_COUNT;i++)map.counts[i]=(uint16_t)(65000+i);
  unsigned map_pages=fetch_map(4),points_pages=fetch_points(4),path_pages=fetch_path(1,false);
  assert(map_pages<6 && points_pages<120);
  printf("normal identity pages: map=%u path=%u points=%u\n",map_pages,path_pages,points_pages);
  command("radar get");pump();assert(strstr(page,"\"key\":\"lidar_x_mm\""));
  for(unsigned i=1;i<10;i++){fetch("params",0,i);assert(strstr(page,"\"k\":\"params\""));}
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
  scan.points_map_id=UINT32_MAX;start_status.scan_id=UINT32_MAX;
  RadarConsole_Init();reply[0]=page[0]=0;
  map_pages=fetch_map(UINT32_MAX);points_pages=fetch_points(UINT32_MAX);path_pages=fetch_path(1,false);
  printf("uint32 identity pages: map=%u path=%u points=%u\n",map_pages,path_pages,points_pages);
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
  assert(foreground_count==6 && !strstr(reply,"page_size"));
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
  /* Auto capture must use the lower installation reference, independent of the
   * current lower-route observer pose. Queries and start selection never drive. */
  location.observer.feedback.x_mm=1200;location.observer.feedback.y_mm=1200;
  location.observer.feedback.yaw_rad=3.14159265358979323846f;
  command("radar scan auto");assert(auto_scans==1 && scans==4 && captured.lidar_x_mm==2170 && captured.lidar_y_mm==230 && captured.zero_deg==180);
  /* The first scan may finish before periodic bulk status can be sent. Its
   * accepted reply must establish an actual capture baseline in the foreground. */
  assert(foreground_count==2 && strstr(foreground[0],"scan_accepted") &&
         strstr(foreground[1],"\"k\":\"status\"") && strstr(foreground[1],"\"state\":\"collecting\""));
  scan.state=RADAR_SCAN_READY;scan.reason="start_ambiguous";scan.map_applicable=false;scan.points_map_id=0;
  start_status.automatic=true;start_status.result.start_id=RADAR_START_UNKNOWN;start_status.result.reason="ambiguous";
  start_status.scan_id--;scan.scan_id=start_status.scan_id;
  unsigned movement_count=moves;foreground_count=0;page[0]=0;fast_pump();
  assert(foreground_count==3 && strstr(foreground[0],"\"selected\":0") && strstr(foreground[1],"\"qr\":[0,0]") && strstr(foreground[2],"start_ambiguous"));
  foreground_count=0;fast_pump();assert(!foreground_count);
  command("radar plan");assert(strstr(reply,"start_unresolved"));
  command("radar nav start");assert(strstr(reply,"start_unresolved") && moves==movement_count);
  command("radar task start");assert(strstr(reply,"start_unresolved") && moves==movement_count);
  command("radar fetch points 4294967294 0");page[0]=0;fast_pump();assert(!page[0] && strstr(reply,"snapshot_changed"));
  command("radar fetch map 4294967295 0");fast_pump();assert(strstr(page,"\"mask\":328000"));
  command("radar fetch path 1 0");fast_pump();assert(strstr(page,"\"start\":2"));
  command("radar start upper");assert(foreground_count==4 && strstr(foreground[0],"start_selected") && strstr(foreground[1],"\"selected\":1") && strstr(foreground[3],"\"k\":\"status\"") && moves==movement_count);
  foreground_count=0;fast_pump();assert(!foreground_count);
  command("radar fetch path 2 0");fast_pump();assert(strstr(page,"\"start\":1") && strstr(page,"[2250,2250,1,0]"));
  command("radar nav start");assert(strstr(reply,"upper_start_plan_only") && moves==movement_count);
  command("radar task start");assert(strstr(reply,"upper_start_plan_only") && moves==movement_count);
  command("radar start lower");assert(foreground_count==4 && strstr(foreground[0],"start_selected") && strstr(foreground[1],"\"selected\":2") && moves==movement_count);
  foreground_count=0;fast_pump();assert(!foreground_count);
  start_status.scan_id=UINT32_MAX;start_status.result.qr_counts[0]=start_status.result.qr_counts[1]=UINT16_MAX;
  start_status.result.raw_counts[0]=start_status.result.raw_counts[1]=UINT16_MAX;
  start_status.result.conflict_mask[0]=start_status.result.conflict_mask[1]=UINT32_MAX;
  command("radar start status");assert(foreground_count==2 && strstr(foreground[0],"\"id\":4294967295") && strstr(foreground[1],"\"conflicts\":[4294967295,4294967295]"));
#ifdef RADAR_CONSOLE_PLAN_FIXTURE
  edge_plan=true;command("radar plan");assert(strstr(reply,"plan_ready"));
  path_pages=fetch_path(4,true);
  printf("int16/uint8 boundary path pages: %u (%u items)\n",path_pages,RADAR_PLAN_MAX_POINTS);
#endif
  puts("radar_console_test: PASS (paging, pose, first scan, no implicit drive, origin and parameter validity)");
}
