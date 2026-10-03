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
static unsigned scans,auto_scans,manual_scans,stops,moves;
static RadarStart_Id_t requested_start;
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
static RadarScan_Raw_t raw;
static uint8_t raw_bytes[RADAR_UART_RAW_BYTES];
static unsigned debug_scans;
static RadarScan_LiveStatus_t live_fixture;
static RadarLive_Point_t live_points[RADAR_LIVE_BINS];
static unsigned live_starts,live_stops,live_copies;
void RadarScan_LiveStatusGet(RadarScan_LiveStatus_t *s){*s=live_fixture;}
const char *RadarScan_LiveStateName(RadarScan_LiveState_t state){
  static const char *const names[]={"preparing","waiting","streaming","stopped","error"};return names[state];
}
bool RadarScan_LiveStart(const RadarMap_Params_t *p,uint32_t origin,RadarStart_Id_t start){
  if(live_fixture.active || scan.state==RADAR_SCAN_COLLECTING)return false;
  live_starts++;live_fixture.id++;live_fixture.active=true;live_fixture.state=RADAR_LIVE_PREPARING;
  live_fixture.reason="configuring";live_fixture.parameters=*p;live_fixture.origin_generation=origin;
  requested_start=start;return true;
}
void RadarScan_LiveStop(void){if(live_fixture.active){live_stops++;live_fixture.active=false;live_fixture.state=RADAR_LIVE_STOPPED;live_fixture.reason="operator_stop";}}
bool RadarScan_LiveCopy(RadarLive_Point_t points[RADAR_LIVE_BINS]){if(!live_fixture.sequence)return false;live_copies++;memcpy(points,live_points,sizeof live_points);return true;}
const char *RadarScan_StateName(RadarScan_State_t state){
  static const char *const names[]={"idle","preparing","wait_wrap","collecting","ready","stopped","error"};return names[state];
}
const char *RadarScan_UnitName(RadarScan_Unit_t unit){return unit==RADAR_UNIT_MM?"mm":unit==RADAR_UNIT_CM?"cm":"unknown_mm";}
void RadarScan_RawStatusGet(RadarScan_Raw_t *s){*s=raw;}
uint16_t RadarScan_RawRead(uint32_t offset,uint8_t *bytes,uint16_t capacity){
  if(raw.raw.active || offset>=raw.raw.total)return 0;
  uint16_t count=(uint16_t)(raw.raw.total-offset);if(count>capacity)count=capacity;
  memcpy(bytes,raw_bytes+offset,count);return count;
}
bool RadarScan_DebugStart(const RadarMap_Params_t *p,uint32_t origin,RadarStart_Id_t start){
  debug_scans++;captured=*p;requested_start=start;scan.state=RADAR_SCAN_COLLECTING;
  raw.id++;raw.origin_generation=origin;raw.parameters=*p;raw.start_id=(uint8_t)start;
  raw.state=RADAR_SCAN_COLLECTING;raw.raw.active=raw.raw.valid=true;return true;
}
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
bool RadarScan_StartAt(const RadarMap_Params_t *p,uint32_t origin,RadarStart_Id_t start){
  requested_start=start;manual_scans++;return RadarScan_Start(p,origin);
}
bool RadarScan_SelectStart(RadarStart_Id_t start){
  if(!start_status.scan_id || !start_status.automatic || RadarConsole_ScanBusy() || start==RADAR_START_UNKNOWN)return false;
  start_status.result.start_id=(uint8_t)start;start_status.result.reason="manual_selection";
  scan.map_id++;scan.points_map_id=scan.map_id;scan.map_applicable=true;scan.reason="complete";return true;
}
void RadarScan_Stop(void){if(live_fixture.active){RadarScan_LiveStop();return;}stops++;scan.state=RADAR_SCAN_STOPPED;}
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
static void assert_page_pose(float x,float y,float zero)
{
  float actual_x,actual_y,actual_zero;const char *at=strstr(page,"\"pose\":[");assert(at);
  assert(sscanf(at+8,"%f,%f,%f",&actual_x,&actual_y,&actual_zero)==3);
  assert(actual_x==x && actual_y==y && actual_zero==zero);
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
  assert(strstr(page,"\"mask\":328000") && strstr(page,"\"pose\":[2250.0,150.0,180.0]"));
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
static void test_raw_console(void)
{
  unsigned movement_count=moves,scan_count=scans;
  scan.state=RADAR_SCAN_IDLE;probe.state=RADAR_PROBE_IDLE;params.zero_deg=90;
  command("radar debug status");assert(strstr(reply,"\"k\":\"raw\"") && !debug_scans && moves==movement_count);
  busy=true;command("radar debug scan upper");assert(strstr(reply,"stationary_capture_required") && !debug_scans);busy=false;
  command("radar debug scan upper");
  assert(debug_scans==1 && scans==scan_count && requested_start==RADAR_START_UPPER && captured.zero_deg==90 && captured.lidar_x_mm==2250);
  assert(strstr(foreground[0],"debug_scan_accepted") && strstr(foreground[foreground_count-1],"\"k\":\"raw\""));
  command("radar fetch raw 1 0");page[0]=0;fast_pump();assert(!page[0] && strstr(reply,"snapshot_changed"));
  raw.raw.active=false;raw.raw.truncated=true;raw.complete=true;raw.state=RADAR_SCAN_READY;
  raw.id=raw.scan_id=raw.map_id=raw.origin_generation=UINT32_MAX;
  raw.raw.total=RADAR_UART_RAW_BYTES;raw.raw.received=raw.raw.window_start=UINT32_MAX;
  raw.raw.crc32=0xcbf43926U;raw.raw.losses=raw.bad=raw.restarts=UINT32_MAX;
  raw.circle_start=raw.circle_end=raw.point_total=UINT32_MAX;raw.points=4096;
  raw.cloud_truncated=true;raw.cloud_valid=false;raw.unit=RADAR_UNIT_MM;
  raw.parameters.lidar_x_mm=2330;raw.parameters.lidar_y_mm=2170;raw.parameters.zero_deg=270;
  for(unsigned i=0;i<RADAR_UART_RAW_BYTES;i++)raw_bytes[i]=(uint8_t)i;
  tick=UINT32_MAX-1000U;RadarConsole_Init();
  /* Frozen raw identity survives unrelated current-map/parameter revisions. */
  scan.scan_id=8;scan.map_id=9;scan.origin_generation=10;params.zero_deg=180;
  fetch("raw",UINT32_MAX,0);unsigned pages=page_field("pages");
  assert(strstr(page,"\"scan\":4294967295") && strstr(page,"\"map\":4294967295") && strstr(page,"\"plan\":0"));
  assert(strstr(page,"\"crc32\":\"CBF43926\"") && strstr(page,"\"complete\":true"));
  fetch("raw",UINT32_MAX,1);assert_page_pose(2330,2170,270);assert(strstr(page,"\"unit\":\"mm\""));
  raw.parameters.lidar_x_mm=2330.125f;raw.parameters.zero_deg=270.0625f;
  fetch("raw",UINT32_MAX,1);assert_page_pose(2330.125f,2170,270.0625f);
  fetch("raw",UINT32_MAX,2);assert(strstr(page,"\"filter\":[") && strstr(page,"\"point_total\":4294967295"));
  fetch("raw",UINT32_MAX,3);assert(strstr(page,"\"circle\":[4294967295,4294967295]") && strstr(page,"\"window\":4294967295"));
  fetch("raw",UINT32_MAX,4);assert(strstr(page,"\"losses\":4294967295") && strlen(page)<=256);
  unsigned seen=0;
  for(unsigned number=5;number<pages;number++){
    fetch("raw",UINT32_MAX,number);assert(page_field("pages")==pages && page_field("offset")==seen);
    const char *at=strstr(page,"\"hex\":\"");assert(at);at+=7;
    while(*at!='\"'){
      unsigned value;assert(sscanf(at,"%2x",&value)==1 && seen<RADAR_UART_RAW_BYTES && value==raw_bytes[seen++]);at+=2;
    }
  }
  assert(seen==RADAR_UART_RAW_BYTES && scans==scan_count && moves==movement_count);
  raw.cloud_valid=true;raw.scan_id=42;raw.map_id=0;raw.origin_generation=99;
  scan.scan_id=42;scan.points_map_id=0;scan.points_valid=true;
  fetch("cloud",42,0);assert(page_field("map")==0 && page_field("origin")==99);assert_page_pose(2330.125f,2170,270.0625f);
  raw.scan_id=raw.map_id=raw.origin_generation=UINT32_MAX;raw.cloud_valid=false;
  command("radar fetch raw 1 0");page[0]=0;fast_pump();assert(!page[0] && strstr(reply,"snapshot_changed"));
  raw.state=RADAR_SCAN_STOPPED;raw.complete=false;raw.scan_id=raw.map_id=0;raw.raw.total=9;
  fetch("raw",UINT32_MAX,0);assert(strstr(page,"\"scan\":0") && strstr(page,"\"map\":0") && strstr(page,"\"state\":\"stopped\""));
}
static unsigned base64_value(char c)
{
  const char *alphabet="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const char *at=strchr(alphabet,c);assert(at);return (unsigned)(at-alphabet);
}
static void assert_live_part(unsigned sequence,unsigned offset,const RadarLive_Point_t *expected)
{
  assert(strstr(page,"\"k\":\"live\"") && page_field("q")==sequence && page_field("o")==offset && page_field("n")==120);
  const char *at=strstr(page,"\"d\":\"");assert(at);at+=5;uint8_t decoded[96];unsigned used=0;
  while(*at!='\"'){
    unsigned value=(base64_value(at[0])<<18)|(base64_value(at[1])<<12)|(base64_value(at[2])<<6)|base64_value(at[3]);
    assert(used+3<=sizeof decoded);decoded[used++]=(uint8_t)(value>>16);decoded[used++]=(uint8_t)(value>>8);decoded[used++]=(uint8_t)value;at+=4;
  }
  assert(used==96 && strlen(page)<=256);
  for(unsigned i=0;i<24;i++)assert((unsigned)(decoded[i*4]|decoded[i*4+1]<<8)==expected[offset+i].angle_tenths &&
    (unsigned)(decoded[i*4+2]|decoded[i*4+3]<<8)==expected[offset+i].distance_mm);
}
static void test_live_console(void)
{
  unsigned movement_count=moves,static_scans=scans,debug_count=debug_scans;
  scan.state=RADAR_SCAN_READY;probe.state=RADAR_PROBE_IDLE;memset(&live_fixture,0,sizeof live_fixture);
  live_fixture.state=RADAR_LIVE_STOPPED;live_fixture.reason="not_started";
  command("radar live status");assert(strstr(reply,"\"k\":\"live_state\"") && !live_starts && moves==movement_count);
  busy=true;command("radar live start upper");assert(!live_starts && strstr(reply,"stationary_capture_required"));busy=false;
  command("radar live start upper");assert(live_starts==1 && requested_start==RADAR_START_UPPER && RadarConsole_ScanBusy());
  assert(strstr(foreground[0],"live_start_accepted") && strstr(foreground[1],"\"k\":\"live_state\""));
  command("radar scan lower");assert(scans==static_scans && strstr(reply,"live_in_progress"));
  command("radar debug scan lower");assert(debug_scans==debug_count && strstr(reply,"live_in_progress"));
  live_fixture.unit=RADAR_UNIT_MM;scan.unit=RADAR_UNIT_UNKNOWN;
  command("radar status");assert(strstr(foreground[2],"\"unit\":\"mm\"") && scan.unit==RADAR_UNIT_UNKNOWN);
  live_fixture.parameters.lidar_x_mm=2330.125f;live_fixture.parameters.lidar_y_mm=2170;live_fixture.parameters.zero_deg=270.0625f;
  live_fixture.id=UINT32_MAX;live_fixture.sequence=UINT32_MAX;live_fixture.age_ms=UINT32_MAX;
  live_fixture.points=120;live_fixture.state=RADAR_LIVE_STREAMING;live_fixture.reason="complete";
  location.generation=0;live_fixture.origin_generation=0;
  tick=UINT32_MAX-1000U;RadarConsole_Init();
  command("radar live status");assert(strstr(reply,"\"s\":4294966296") && strstr(reply,"\"id\":4294967295") && strlen(reply)<=256);
  for(unsigned i=0;i<120;i++)live_points[i]=(RadarLive_Point_t){(uint16_t)(i*30+5),i%7?UINT16_MAX:0};
  RadarLive_Point_t frozen[120];memcpy(frozen,live_points,sizeof frozen);
  page[0]=0;fast_pump();assert_live_part(UINT32_MAX,0,frozen);assert(live_copies==1);
  /* A requested historical page is served before the next live part. */
  command("radar fetch params 0 0");page[0]=0;fast_pump();assert(strstr(page,"\"k\":\"params\""));
  live_fixture.sequence=1;memset(live_points,0,sizeof live_points);
  for(unsigned part=1;part<5;part++){page[0]=0;fast_pump();assert_live_part(UINT32_MAX,part*24,frozen);}
  assert(live_copies==1);tick+=1000;page[0]=0;fast_pump();assert_live_part(1,0,live_points);assert(live_copies==2);
  command("radar live stop");assert(!RadarConsole_ScanBusy() && live_stops==1 && strstr(foreground[0],"live_stop_requested"));
  page[0]=0;fast_pump();assert(!strstr(page,"\"k\":\"live\""));
  command("radar live start lower");assert(live_starts==2);command("radar stop");assert(live_stops==2 && !live_fixture.active);
  assert(scans==static_scans && debug_scans==debug_count && moves==movement_count);
}
int main(void)
{
  RadarMap_Defaults(&params);captured=params;RadarMap_Reset(&map);
  assert(params.lidar_x_mm==2250 && params.lidar_y_mm==150 && params.zero_deg==180 &&
    params.angle_min_tenths==0 && params.angle_max_tenths==900 && params.threshold==2);
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
  assert(strstr(page,"\"pose\":[2250.0,150.0,180.0]"));
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
  scan.map_applicable=true;command("radar scan");assert(scans==1 && captured.zero_deg==180 && captured.lidar_x_mm==2250);
  command("radar stop");assert(stops==1 && map.mask==RADAR_MAP_FIXED_MASK);
  scan.coverage_tenths=70;command("radar fetch map 4 0");pump();assert(strstr(page,"\"coverage\":3600"));
  location.observer.feedback.x_mm=1200;location.observer.feedback.y_mm=1200;
  location.observer.feedback.yaw_rad=3.14159265358979323846f;
  command("radar scan");assert(scans==2 && fabsf(captured.lidar_x_mm-1200)<0.01f && fabsf(captured.lidar_y_mm-1200)<0.01f && fabsf(captured.zero_deg-270)<0.01f);
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
  command("radar scan auto");assert(auto_scans==1 && scans==4 && captured.lidar_x_mm==2250 && captured.lidar_y_mm==150 && captured.zero_deg==180);
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
  /* A manually chosen corner can capture its first circle without auto history.
   * The caller passes the lower installation reference, never observer feedback. */
  params.zero_deg=90;location.observer.feedback.x_mm=1200;location.observer.feedback.y_mm=1200;
  location.observer.feedback.yaw_rad=3.14159265358979323846f;
  scan.state=RADAR_SCAN_IDLE;scan.map_valid=false;scan.scan_id=scan.map_id=0;
  memset(&start_status,0,sizeof start_status);start_status.result.reason="not_scanned";
  RadarConsole_Init();command("radar scan upper");
  assert(manual_scans==1 && requested_start==RADAR_START_UPPER &&
    captured.lidar_x_mm==2250 && captured.lidar_y_mm==150 && captured.zero_deg==90);
  assert(foreground_count==2 && strstr(foreground[0],"scan_accepted") &&
    strstr(foreground[1],"\"state\":\"collecting\""));
  assert(moves==movement_count);
  /* A completed explicit upper scan is manual_reference with upper planning. */
  scan.state=RADAR_SCAN_READY;scan.reason="complete";scan.map_valid=scan.map_applicable=scan.points_valid=true;
  scan.scan_id=scan.map_id=scan.points_map_id=1;scan.origin_generation=location.generation;
  start_status.scan_id=1;start_status.automatic=false;
  start_status.result.start_id=RADAR_START_UPPER;start_status.result.reason="manual_reference";
  captured.lidar_x_mm=2250;captured.lidar_y_mm=2250;captured.zero_deg=180;
#ifdef RADAR_CONSOLE_PLAN_FIXTURE
  edge_plan=false;
#endif
  fast_pump();fetch("path",1,0);
  assert(page_field("start")==RADAR_START_UPPER && strstr(page,"[2250,2250,1,0]"));
  unsigned upper_pages=page_field("pages");
  for(unsigned number=1;number<upper_pages;number++){
    fetch("path",1,number);assert(page_field("start")==RADAR_START_UPPER);
  }
  assert(strstr(page,"[2250,2250,1,8]"));
  command("radar nav start");assert(strstr(reply,"upper_start_plan_only") && moves==movement_count);
  command("radar task start");assert(strstr(reply,"upper_start_plan_only") && moves==movement_count);
  command("radar scan lower");
  assert(manual_scans==2 && requested_start==RADAR_START_LOWER &&
    captured.lidar_x_mm==2250 && captured.lidar_y_mm==150 && captured.zero_deg==90);
  command("radar stop");busy=true;command("radar scan upper");
  assert(manual_scans==2 && strstr(reply,"stationary_capture_required"));busy=false;
  command("radar scan sideways");assert(manual_scans==2 && strstr(reply,"format"));
  test_raw_console();
  test_live_console();
  puts("radar_console_test: PASS (paging, pose, first scan, no implicit drive, origin and parameter validity)");
}
