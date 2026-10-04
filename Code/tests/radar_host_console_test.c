/* Wire-level host-console contract. The host service is a deterministic fake;
 * this test verifies command admission, priority, bounded pages and cloud bytes. */
#include "radar_host_console.h"
#include "radar_host.h"
#include "console_tx.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static RadarHost_Status_t HostStatus;
static RadarHostMap_Config_t Active,Draft;
static RadarHostMap_Result_t Result;
#define TEST_CLOUD_POINTS 4096U
static RadarSample_t Cloud[TEST_CLOUD_POINTS];
static RadarLive_Point_t Preview[RADAR_LIVE_BINS];
static uint16_t CloudCount,PreviewCount;
static unsigned HostInit,HostProcess,Starts,Stops,Scans,LiveStarts,LiveStops;
static unsigned DraftResets,Sets,Applies;
static char Reply[257],Urgent[257],Page[257];
static unsigned ReplyCount,UrgentCount,PageCount;
static bool BulkReady=true;

uint32_t HAL_GetTick(void){return 0U;}
void RadarHost_Init(void){HostInit++;}
void RadarHost_Process(void){HostProcess++;}
bool RadarHost_StartRotation(void){Starts++;return true;}
void RadarHost_Stop(void){Stops++;HostStatus.active=false;}
bool RadarHost_Scan(void){Scans++;return true;}
bool RadarHost_LiveStart(void){LiveStarts++;HostStatus.live_active=true;return true;}
void RadarHost_LiveStop(void){LiveStops++;HostStatus.live_active=false;}
void RadarHost_StatusGet(RadarHost_Status_t *s){*s=HostStatus;}
const RadarHostMap_Config_t *RadarHost_ActiveConfig(void){return &Active;}
const RadarHostMap_Config_t *RadarHost_DraftConfig(void){return HostStatus.draft_revision?&Draft:NULL;}
uint32_t RadarHost_DraftReset(void){DraftResets++;HostStatus.draft_revision++;return HostStatus.draft_revision;}
bool RadarHost_DraftSet(const char *key,const char *value){
  Sets++;
  if(!strcmp(key,"mission"))return !strcmp(value,"1-5-4-2-3-4-2-3-1");
  if(!strcmp(key,"start"))return !strcmp(value,"1,1,radar");
  return !strcmp(key,"threshold") && !strcmp(value,"1");
}
bool RadarHost_Apply(uint32_t revision){Applies++;return revision==HostStatus.draft_revision && revision!=0U;}
const RadarHostMap_Result_t *RadarHost_Map(uint32_t *map_id,uint32_t *scan_id){
  if(map_id)*map_id=HostStatus.map_id;
  if(scan_id)*scan_id=HostStatus.scan_id;
  return HostStatus.map_valid?&Result:NULL;
}
const RadarHostMap_Result_t *RadarHost_Path(uint32_t *plan_id,uint32_t *scan_id){
  if(plan_id)*plan_id=HostStatus.plan_id;
  if(scan_id)*scan_id=HostStatus.scan_id;
  return HostStatus.path_valid?&Result:NULL;
}
const RadarLive_Point_t *RadarHost_Preview(uint16_t *count,uint32_t *preview_id){
  if(count)*count=PreviewCount;
  if(preview_id)*preview_id=HostStatus.preview_id;
  return HostStatus.preview_id?Preview:NULL;
}
const RadarSample_t *RadarHost_Cloud(uint16_t *count,uint32_t *scan_id){
  if(count)*count=CloudCount;
  if(scan_id)*scan_id=HostStatus.scan_id;
  return HostStatus.cloud_valid?Cloud:NULL;
}

bool ConsoleTx_Write(const uint8_t *data,uint16_t size){
  assert(size<=256U);memcpy(Reply,data,size);Reply[size]='\0';ReplyCount++;return true;
}
bool ConsoleTx_Urgent(const char *data,uint16_t size){
  assert(size<=256U);memcpy(Urgent,data,size);Urgent[size]='\0';UrgentCount++;return true;
}
bool ConsoleTx_BulkReady(void){return BulkReady;}
bool ConsoleTx_Bulk(const char *data,uint16_t size){
  assert(BulkReady && size<=256U && size>=2U);
  assert(data[size-2U]=='\r' && data[size-1U]=='\n');
  memcpy(Page,data,size);Page[size]='\0';PageCount++;return true;
}

static void clear_output(void){Reply[0]=Urgent[0]=Page[0]='\0';}
static bool command_raw(const char *wire)
{
  char text[96],*tokens[8],*part;unsigned count=0U;
  assert(strlen(wire)<sizeof text);strcpy(text,wire);
  for(part=strtok(text," ");part!=NULL && count<8U;part=strtok(NULL," "))tokens[count++]=part;
  clear_output();return RadarHostConsole_Command(count,tokens);
}
static void command(const char *wire){assert(command_raw(wire));}
static uint32_t field(const char *name)
{
  char key[40];unsigned long value;const char *at;
  snprintf(key,sizeof key,"\"%s\":",name);at=strstr(Page,key);assert(at!=NULL);
  assert(sscanf(at+strlen(key),"%lu",&value)==1);return (uint32_t)value;
}
static void data_text(char *out,size_t capacity)
{
  const char *at=strstr(Page,"\"data\":\"");const char *end;size_t length;
  assert(at!=NULL);at+=8;end=strchr(at,'"');assert(end!=NULL);length=(size_t)(end-at);
  assert(length<capacity);memcpy(out,at,length);out[length]='\0';
}
static unsigned decode64(const char *text,uint8_t *out,unsigned capacity)
{
  static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  unsigned value=0U,bits=0U,count=0U;
  for(;*text!='\0' && *text!='=';text++){
    const char *at=strchr(alphabet,*text);assert(at!=NULL);value=(value<<6)|(unsigned)(at-alphabet);bits+=6U;
    if(bits>=8U){bits-=8U;assert(count<capacity);out[count++]=(uint8_t)(value>>bits);value&=(1U<<bits)-1U;}
  }
  return count;
}
static uint32_t crc32(const uint8_t *bytes,unsigned count)
{
  uint32_t crc=0xFFFFFFFFUL;
  for(unsigned i=0U;i<count;i++){
    crc^=bytes[i];for(unsigned bit=0U;bit<8U;bit++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);
  }
  return ~crc;
}
static uint32_t hex_field(const char *name)
{
  char key[40];unsigned value;const char *at;
  snprintf(key,sizeof key,"\"%s\":\"",name);at=strstr(Page,key);assert(at!=NULL);
  assert(sscanf(at+strlen(key),"%8x",&value)==1);return value;
}

static void fetch_cloud(unsigned count)
{
  uint8_t packed[TEST_CLOUD_POINTS*5U],part[128];unsigned used=0U,pages=0U;
  char wire[64],encoded[128];CloudCount=(uint16_t)count;HostStatus.cloud_valid=true;
  HostStatus.scan_id=count==4096U?UINT32_MAX:(100U+count);
  for(unsigned i=0U;i<count;i++)Cloud[i]=(RadarSample_t){(uint16_t)(i*7U),(uint16_t)(65535U-i),(uint8_t)i};
  for(unsigned page=0U;page==0U || page<pages;page++){
    snprintf(wire,sizeof wire,"radar host fetch cloud:%lu %u",(unsigned long)HostStatus.scan_id,page);
    command(wire);RadarHostConsole_Process();assert(Page[0]!='\0');
    assert(strstr(Page,"\"profile\":\"f1_host\"") && strstr(Page,"\"k\":\"host_cloud\""));
    assert(field("session")==HostStatus.session_id && field("config")==HostStatus.config_id);
    assert(field("scan")==HostStatus.scan_id && field("map")==HostStatus.map_id && field("plan")==HostStatus.plan_id);
    if(page==0U)pages=field("pages");else assert(field("pages")==pages);
    assert(field("page")==page && field("offset")==used/5U && field("total")==count);
    data_text(encoded,sizeof encoded);unsigned got=decode64(encoded,part,sizeof part);assert((got%5U)==0U);
    assert(used+got<=sizeof packed);memcpy(packed+used,part,got);used+=got;
  }
  assert(used==count*5U);
  for(unsigned i=0U;i<count;i++){
    assert(packed[i*5U]==(uint8_t)Cloud[i].angle_tenths);
    assert(packed[i*5U+1U]==(uint8_t)(Cloud[i].angle_tenths>>8));
    assert(packed[i*5U+2U]==(uint8_t)Cloud[i].distance_mm);
    assert(packed[i*5U+3U]==(uint8_t)(Cloud[i].distance_mm>>8));
    assert(packed[i*5U+4U]==Cloud[i].energy);
  }
  assert(hex_field("crc32")==crc32(packed,used));
}

static void test_grammar_and_priority(void)
{
  unsigned mutations;
  command("radar host status");assert(strstr(Reply,"\"k\":\"host_status\"") && !Starts && !Stops && !Scans);
  mutations=Starts+Stops+Scans+LiveStarts+LiveStops+DraftResets+Sets+Applies;
  command("radar host live status");assert(strstr(Reply,"host_status") && mutations==Starts+Stops+Scans+LiveStarts+LiveStops+DraftResets+Sets+Applies);
  command("radar host get active");RadarHostConsole_Process();assert(strstr(Page,"host_config"));
  assert(mutations==Starts+Stops+Scans+LiveStarts+LiveStops+DraftResets+Sets+Applies);
  command("radar host start");assert(Starts==1U && strstr(Reply,"start_accepted"));
  command("radar host scan");assert(Scans==1U && strstr(Reply,"scan_accepted"));
  command("radar host live start");assert(LiveStarts==1U);
  command("radar host live stop");assert(LiveStops==1U && strstr(Urgent,"live_stop_requested"));
  command("radar host draft reset");assert(DraftResets==1U);
  command("radar host set threshold 1");assert(Sets==1U && strstr(Reply,"draft_updated"));
  command("radar host set mission 1-5-4-2-3-4-2-3-1");assert(Sets==2U);
  command("radar host set start 1,1,radar");assert(Sets==3U);
  {
    char wire[64];snprintf(wire,sizeof wire,"radar host apply %lu",(unsigned long)HostStatus.draft_revision);
    command(wire);assert(Applies==1U && strstr(Reply,"config_applied"));
  }
  command("radar host set mission 1-1");assert(strstr(Reply,"draft_value"));
  command("radar host set start 0,1,radar");assert(strstr(Reply,"draft_value"));
  command("radar host apply nope");assert(strstr(Reply,"revision"));
  command("radar host fetch cloud:1 nope");assert(strstr(Reply,"page"));
  command("radar host fetch cloud 0");assert(strstr(Reply,"fetch_target"));
  command("radar host status extra");assert(strstr(Reply,"format"));
  assert(!command_raw("radar status"));

  /* A not-yet-enqueued page is cancelled. A copied complete page may drain,
   * then an urgent stop prevents every later host page. */
  HostStatus.scan_id=123U;CloudCount=37U;HostStatus.cloud_valid=true;
  command("radar host fetch cloud:123 0");unsigned before=PageCount;
  command("radar host stop");assert(Stops==1U && strstr(Urgent,"stop_requested"));
  RadarHostConsole_Process();assert(PageCount==before);
  HostStatus.active=true;command("radar host fetch cloud:123 0");RadarHostConsole_Process();before=PageCount;
  assert(before>0U);command("radar host stop");RadarHostConsole_Process();assert(PageCount==before);
}

static void test_automatic_order(void)
{
  bool saw_config=false,saw_map=false,saw_path=false,saw_cloud=false;
  HostStatus.map_id=11U;HostStatus.plan_id=12U;HostStatus.scan_id=13U;
  HostStatus.map_valid=HostStatus.path_valid=HostStatus.cloud_valid=true;CloudCount=1U;
  Result.waypoint_count=2U;
  for(unsigned i=0U;i<200U && !saw_cloud;i++){
    clear_output();RadarHostConsole_Process();if(!Page[0])continue;
    if(strstr(Page,"host_config"))saw_config=true;
    else if(strstr(Page,"host_map")){assert(saw_config);saw_map=true;}
    else if(strstr(Page,"host_path")){assert(saw_map);saw_path=true;}
    else if(strstr(Page,"host_cloud")){assert(saw_path);saw_cloud=true;}
  }
  assert(saw_config && saw_map && saw_path && saw_cloud);
}

int main(void)
{
  memset(&HostStatus,0,sizeof HostStatus);memset(&Active,0,sizeof Active);memset(&Result,0,sizeof Result);
  HostStatus.session_id=HostStatus.config_id=HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=UINT32_MAX;
  HostStatus.preview_id=UINT32_MAX;HostStatus.map_valid=HostStatus.path_valid=HostStatus.cloud_valid=true;
  HostStatus.reason="complete";Active.threshold=1U;Active.radar_x_mm=Active.radar_y_mm=230U;
  Active.fixed_mask=RADAR_HOST_FIXED_MASK;Active.protected_mask=RADAR_HOST_PROTECTED_MASK;Draft=Active;
  Result.path_valid=true;Result.waypoint_count=RADAR_HOST_MAX_WAYPOINTS;
  for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)Result.counts[i]=(uint16_t)(65535U-i);
  for(unsigned i=0U;i<Result.waypoint_count;i++)Result.waypoints[i]=(RadarHostMap_Waypoint_t){65535U,65535U,65535U,5U,5U,3U,4U};
  PreviewCount=RADAR_LIVE_BINS;for(unsigned i=0U;i<PreviewCount;i++)Preview[i]=(RadarLive_Point_t){(uint16_t)i,65535U};
  RadarHostConsole_Init();assert(HostInit==1U);
  test_grammar_and_priority();
  /* Reinitialize to discard explicit transfers before checking unsolicited order. */
  RadarHostConsole_Init();test_automatic_order();
  RadarHostConsole_Init();fetch_cloud(0U);fetch_cloud(1U);fetch_cloud(37U);fetch_cloud(4096U);
  printf("radar_host_console_test: PASS (%u bounded pages, full cloud CRC)\n",PageCount);
}
