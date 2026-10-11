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
static RadarStablePoint Stable[RADAR_STABLE_CAPACITY];
static uint16_t StableCount;
static uint16_t CloudCount,PreviewCount;
static unsigned HostInit,HostProcess,Starts,Stops,Scans,LiveStarts,LiveStops;
static unsigned DraftResets,Sets,Applies;
static char Reply[257],Urgent[257],Page[257];
static unsigned ReplyCount,UrgentCount,PageCount;
static bool BulkReady=true;

uint32_t HAL_GetTick(void){return 0U;}
void RadarHost_Init(void){HostInit++;}
#ifndef RADAR_HOST_WIRE_FIXTURE
void RadarHostMap_FreezeConfig(const RadarHostMap_Config_t *input,RadarHostMap_Config_t *output){*output=*input;}
#endif
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
const RadarStablePoint *RadarHost_Stable(uint16_t *count,uint32_t *scan_id){
  if(count)*count=StableCount;
  if(scan_id)*scan_id=HostStatus.scan_id;
  return HostStatus.map_valid?Stable:NULL;
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
  command("radar host scan 15");assert(Scans==2U && strstr(Reply,"scan_accepted"));
  command("radar host scan 15 fast");assert(Scans==3U && strstr(Reply,"scan_accepted"));
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
  RadarHostConsole_Process();assert(PageCount==before || strstr(Page,"host_status"));
  HostStatus.active=true;command("radar host fetch cloud:123 0");RadarHostConsole_Process();before=PageCount;
  assert(before>0U);command("radar host stop");RadarHostConsole_Process();assert(PageCount==before || strstr(Page,"host_status"));
}

static void test_completion_notifies_without_bulk_dump(void)
{
  bool saw_status=false;
  HostStatus.map_id=11U;HostStatus.plan_id=12U;HostStatus.scan_id=13U;
  HostStatus.map_valid=HostStatus.path_valid=HostStatus.cloud_valid=true;CloudCount=1U;
  HostStatus.scan_state=RADAR_SCAN_READY;Result.waypoint_count=2U;
  for(unsigned i=0U;i<200U;i++){
    clear_output();RadarHostConsole_Process();if(!Page[0])continue;
    assert(strstr(Page,"host_status"));saw_status=true;
  }
  assert(saw_status);
  command("radar host fetch map:11 0");RadarHostConsole_Process();assert(strstr(Page,"host_map"));
  command("radar host fetch path:12 0");RadarHostConsole_Process();assert(strstr(Page,"host_path"));
  command("radar host ack 12");assert(strstr(Reply,"result_acknowledged"));
}

static void test_result_read_survives_new_live_circles(void)
{
  char wire[64];unsigned pages=1U;
  HostStatus.live_active=true;HostStatus.map_valid=HostStatus.path_valid=true;
  HostStatus.scan_id=22U;HostStatus.map_id=20U;HostStatus.plan_id=21U;
  Result.raw_points=10U;Result.counts[1]=3U;Result.waypoint_count=2U;
  Result.waypoints[0].x_mm=230U;Result.waypoints[1].x_mm=775U;
  RadarHostConsole_Init();command("radar host status");
  for(unsigned page=0U;page<pages;page++){
    snprintf(wire,sizeof wire,"radar host fetch map:20 %u",page);
    command(wire);RadarHostConsole_Process();assert(strstr(Page,"host_map"));
    pages=field("pages");assert(field("scan")==22U && field("map")==20U && field("plan")==21U);
    HostStatus.scan_id++;HostStatus.map_id++;HostStatus.plan_id++;
    Result.raw_points=99U;Result.counts[1]=99U;Result.waypoints[0].x_mm=999U;
  }
  command("radar host fetch path:21 0");RadarHostConsole_Process();
  assert(strstr(Page,"w,230,") && field("scan")==22U);
  /* A lost last page is still readable after the producer has advanced. */
  command("radar host fetch path:21 0");RadarHostConsole_Process();
  assert(strstr(Page,"w,230,") && field("scan")==22U);
  command("radar host ack 21");assert(strstr(Reply,"result_acknowledged"));
  command("radar host ack 21");assert(strstr(Reply,"result_acknowledged"));
  assert(pages<28U);
  HostStatus.live_active=false;
}

static void test_preview_freezes_the_circle_until_every_page_is_sent(void)
{
  uint8_t packed[RADAR_LIVE_BINS*4U],part[128];char encoded[164];
  unsigned used=0U,pages=0U;uint32_t expected_crc=0U;
  HostStatus.preview_id=0U;HostStatus.live_active=true;HostStatus.active=false;
  RadarHostConsole_Init();PreviewCount=RADAR_LIVE_BINS;
  for(unsigned i=0U;i<PreviewCount;i++)Preview[i]=(RadarLive_Point_t){(uint16_t)(i*10U),(uint16_t)(1000U+i)};
  HostStatus.preview_id=101U;
  for(unsigned page=0U;page==0U || page<pages;page++){
    clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_preview"));
    if(page==0U){pages=field("pages");expected_crc=hex_field("crc32");assert(pages>1U);}
    assert(field("live_id")==101U);
    assert(field("page")==page && field("offset")==used/4U && field("total")==RADAR_LIVE_BINS);
    assert(hex_field("crc32")==expected_crc);
    data_text(encoded,sizeof encoded);unsigned got=decode64(encoded,part,sizeof part);
    assert(used+got<=sizeof packed);memcpy(packed+used,part,got);used+=got;
    /* The producer advances many times faster than this paced page sender. */
    HostStatus.preview_id=102U+page;PreviewCount=113U;
    for(unsigned i=0U;i<PreviewCount;i++)Preview[i]=(RadarLive_Point_t){3000U,(uint16_t)(5000U+page)};
    if(page==0U){
      uint8_t original[128];memcpy(original,part,got);
      BulkReady=false;clear_output();RadarHostConsole_Process();assert(Page[0]=='\0');BulkReady=true;
      command("radar host fetch preview:101 0");RadarHostConsole_Process();
      assert(field("live_id")==101U && field("page")==0U && hex_field("crc32")==expected_crc);
      data_text(encoded,sizeof encoded);assert(decode64(encoded,part,sizeof part)==got && !memcmp(part,original,got));
      command("radar host fetch preview:102 0");RadarHostConsole_Process();
      assert(strstr(Reply,"snapshot_or_page") && Page[0]=='\0');
    }
  }
  assert(used==sizeof packed && expected_crc==crc32(packed,used));
  for(unsigned i=0U;i<RADAR_LIVE_BINS;i++){
    assert((unsigned)(packed[i*4U]|packed[i*4U+1U]<<8)==i*10U);
    assert((unsigned)(packed[i*4U+2U]|packed[i*4U+3U]<<8)==1000U+i);
  }
  assert(field("live_id")==101U);
  clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_preview"));
  assert(field("page")==0U && field("live_id")==HostStatus.preview_id && field("total")==113U);
  command("radar host live stop");
  for(unsigned i=0U;i<10U;i++){clear_output();RadarHostConsole_Process();assert(!strstr(Page,"host_preview"));}
}

static void test_preview_uint32_identifiers_still_fit_the_wire_budget(void)
{
  unsigned pages=1U;
  HostStatus.session_id=HostStatus.config_id=HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=UINT32_MAX;
  HostStatus.preview_id=0U;HostStatus.live_active=true;RadarHostConsole_Init();
  HostStatus.preview_id=UINT32_MAX;PreviewCount=RADAR_LIVE_BINS;
  for(unsigned page=0U;page<pages;page++){
    clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_preview"));pages=field("pages");
    assert(field("page")==page && field("live_id")==UINT32_MAX && strlen(Page)<=256U);
  }
}

static void test_result_uint32_identifiers_still_fit_the_wire_budget(void)
{
  char wire[80];
  HostStatus.session_id=HostStatus.config_id=HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=UINT32_MAX;
  HostStatus.map_valid=HostStatus.path_valid=true;HostStatus.scan_state=RADAR_SCAN_READY;
  Result.raw_points=Result.filtered_points=Result.outside_points=UINT32_MAX;
  Result.waypoint_count=RADAR_HOST_MAX_WAYPOINTS;
  for(unsigned i=0U;i<25U;i++)Result.counts[i]=65535U;
  for(unsigned i=0U;i<Result.waypoint_count;i++)Result.waypoints[i]=(RadarHostMap_Waypoint_t){65535U,65535U,65535U,5U,5U,3U,4U};
  RadarHostConsole_Init();
  for(unsigned kind=0U;kind<2U;kind++){
    unsigned pages=1U;
    for(unsigned page=0U;page<pages;page++){
      snprintf(wire,sizeof wire,"radar host fetch %s:4294967295 %u",kind?"path":"map",page);
      command(wire);RadarHostConsole_Process();assert(strstr(Page,kind?"host_path":"host_map"));
      pages=field("pages");assert(field("page")==page && strlen(Page)<=256U);
    }
  }
  command("radar host ack 4294967295");assert(strstr(Reply,"result_acknowledged"));
  RadarHostConsole_Init();command("radar host result 4294967295");
  for(unsigned i=0U;i<300U;i++){
    clear_output();RadarHostConsole_Process();assert(Page[0] && strlen(Page)<=256U);
    assert(field("plan")==UINT32_MAX);
    if(strstr(Page,"host_result_end"))break;
  }
  assert(strstr(Page,"host_result_end") && field("mp")>0U && field("pp")>0U);
}

static void test_config_read_stays_coherent_during_continuous_mapping(void)
{
  HostStatus.config_id=55U;HostStatus.scan_id=30U;HostStatus.map_id=31U;HostStatus.plan_id=32U;
  HostStatus.live_active=true;Active.threshold=1U;RadarHostConsole_Init();
  command("radar host get active");
  for(unsigned page=0U;page<27U;page++) {
    RadarHostConsole_Process();assert(strstr(Page,"host_config"));
    assert(field("scan")==30U && field("map")==31U && field("plan")==32U);
    if(page==19U)assert(strstr(Page,"threshold,1"));
    HostStatus.scan_id++;HostStatus.map_id++;HostStatus.plan_id++;Active.threshold=99U;
  }
  HostStatus.live_active=false;
}

static void test_fast_scan_pushes_compact_pair_without_result_request(void)
{
  uint8_t packed[512],part[100];char encoded[140],wire[80];unsigned used=0U,pages=0U;
  HostStatus.session_id=41U;HostStatus.config_id=1U;HostStatus.scan_id=2U;
  HostStatus.map_id=3U;HostStatus.plan_id=4U;HostStatus.active=true;
  HostStatus.scan_state=RADAR_SCAN_COLLECTING;HostStatus.map_valid=true;
  Result.raw_points=1600U;Result.filtered_points=1400U;Result.outside_points=1398U;
  Result.scanned_mask=0x00200000U;Result.manual_mask=0U;Result.effective_mask=0x00250140U;
  Result.path_valid=true;Result.failed_from_task=Result.failed_to_task=0U;Result.waypoint_count=33U;
  memset(Result.counts,0,sizeof Result.counts);Result.counts[21]=2U;
  for(unsigned i=0U;i<33U;i++)Result.waypoints[i]=(RadarHostMap_Waypoint_t){230U,350U,425U,1U,2U,3U,4U};
  Result.waypoints[0]=(RadarHostMap_Waypoint_t){230U,230U,558U,1U,1U,1U,0U};
  RadarHostConsole_Init();command("radar host scan fast");assert(strstr(Reply,"scan_accepted"));
  HostStatus.active=false;HostStatus.scan_state=RADAR_SCAN_READY;
  for(unsigned i=0U;i<40U;i++){
    clear_output();RadarHostConsole_Process();if(!strstr(Page,"\"k\":\"r\""))continue;
    assert(strlen(Page)<=256U);unsigned page=field("p");if(!pages)pages=field("n");
    assert(page==used/72U && field("t")==488U);
    const char *at=strstr(Page,"\"d\":\"");assert(at);at+=5;const char *end=strchr(at,'"');assert(end);
    assert((unsigned)(end-at)<sizeof encoded);memcpy(encoded,at,(size_t)(end-at));encoded[end-at]='\0';
    unsigned got=decode64(encoded,part,sizeof part);assert(used+got<=sizeof packed);memcpy(packed+used,part,got);used+=got;
    if(page+1U==pages)break;
  }
  assert(pages==7U && used==488U && packed[0]==2U);
  assert((unsigned)(packed[1]|packed[2]<<8)==1600U);
  assert((unsigned)(packed[78]|packed[79]<<8)==0U); /* cell 24 */
  assert((unsigned)(packed[158]|packed[159]<<8)==230U);
  assert((unsigned)(packed[160]|packed[161]<<8)==230U);
  command("radar host bundle 4 0,4");
  for(unsigned i=0U;i<2U;i++){clear_output();RadarHostConsole_Process();assert(strstr(Page,"\"k\":\"r\""));assert(field("p")==i*4U);}
  command("radar host ack 4");assert(strstr(Reply,"result_acknowledged"));
  /* Worst-case identities cannot exceed the shared ASCII line budget. */
  HostStatus.session_id=HostStatus.config_id=HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=UINT32_MAX;
  RadarHostConsole_Init();command("radar host status fast");
  snprintf(wire,sizeof wire,"radar host bundle %lu 0",(unsigned long)UINT32_MAX);command(wire);
  RadarHostConsole_Process();assert(strstr(Page,"\"k\":\"r\"") && strlen(Page)<=256U);
  command("radar host stop");assert(strstr(Urgent,"stop_requested"));
}

static void test_stable_pages_freeze_xy_and_exceed_32_pages(void)
{
  char wire[80],encoded[164];uint8_t packed[2560],part[128];unsigned pages=1U,used=0U;
  HostStatus.session_id=HostStatus.config_id=HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=UINT32_MAX;
  HostStatus.map_valid=HostStatus.path_valid=true;HostStatus.scan_state=RADAR_SCAN_READY;
  StableCount=RADAR_STABLE_CAPACITY;
  for(unsigned i=0U;i<StableCount;i++)Stable[i]=(RadarStablePoint){(uint16_t)(i*10U),500U,(uint16_t)(200U+i),400U,3U,1U};
  RadarHostConsole_Init();
  for(unsigned page=0U;page<pages;page++){
    snprintf(wire,sizeof wire,"radar host fetch stable:%lu %u",(unsigned long)UINT32_MAX,page);
    command(wire);RadarHostConsole_Process();assert(strstr(Page,"host_stable"));pages=field("pages");
    assert(field("scan")==UINT32_MAX && field("offset")==used/10U && field("total")==256U);
    data_text(encoded,sizeof encoded);unsigned got=decode64(encoded,part,sizeof part);
    assert(got%10U==0U && used+got<=sizeof packed);memcpy(packed+used,part,got);used+=got;
    Stable[0].x_mm=999U;HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=20U;
  }
  assert(pages>32U && used==sizeof packed && packed[4]==200U && packed[5]==0U);
  assert(hex_field("crc32")==crc32(packed,used));StableCount=0U;
}

static void test_shared_stable_snapshot_mode_handover(void)
{
  RadarStablePoint point={0U,500U,300U,400U,3U,1U};
  RadarStableTx_Release(1U);RadarStableTx_Release(2U);
  assert(RadarStableTx_Freeze(1U,8U,&point,1U));
  command("radar host scan");assert(strstr(Reply,"scan_accepted"));
  assert(RadarStableTx_Freeze(2U,9U,&point,1U));
  assert(!RadarStableTx_Freeze(1U,10U,&point,1U));
  RadarStableTx_Release(2U);assert(RadarStableTx_Freeze(1U,10U,&point,1U));
  command("radar host live start");assert(strstr(Reply,"live_start_accepted"));
  assert(RadarStableTx_Freeze(2U,11U,&point,1U));RadarStableTx_Release(2U);
  HostStatus.live_active=false;
}

static void test_one_request_streams_frozen_results(void)
{
  unsigned maps=0U,paths=0U,mutations=Starts+Stops+Scans+LiveStarts+LiveStops+DraftResets+Sets+Applies;
  HostStatus.session_id=41U;HostStatus.config_id=1U;HostStatus.scan_id=2U;HostStatus.map_id=3U;HostStatus.plan_id=4U;
  HostStatus.scan_state=RADAR_SCAN_READY;HostStatus.map_valid=HostStatus.path_valid=true;HostStatus.preview_id=0U;
  Result.waypoint_count=RADAR_HOST_MAX_WAYPOINTS;Result.waypoints[0].x_mm=230U;
  RadarHostConsole_Init();command("radar host result 4");assert(!strstr(Reply,"ERR"));
  for(unsigned i=0U;i<300U;i++){
    clear_output();RadarHostConsole_Process();assert(Page[0]);assert(strlen(Page)<=256U);
    assert(field("scan")==2U && field("map")==3U && field("plan")==4U);
    if(strstr(Page,"host_result_end")){assert(field("mp")==maps && field("pp")==paths);break;}
    if(strstr(Page,"host_map")){assert(!paths && field("page")==maps++);}
    else {assert(strstr(Page,"host_path") && field("page")==paths++);if(paths==1U)assert(strstr(Page,"w,230,"));}
    HostStatus.scan_id=20U;HostStatus.map_id=30U;HostStatus.plan_id=40U;Result.waypoints[0].x_mm=999U;
  }
  assert(maps>0U && paths>0U && strstr(Page,"host_result_end"));
  assert(mutations==Starts+Stops+Scans+LiveStarts+LiveStops+DraftResets+Sets+Applies);
  command("radar host fetch path:4 0");RadarHostConsole_Process();assert(strstr(Page,"w,230,"));
  command("radar host ack 4");assert(strstr(Reply,"result_acknowledged"));
  command("radar host result 40");RadarHostConsole_Process();assert(strstr(Page,"host_map"));
  command("radar host stop");assert(strstr(Urgent,"stop_requested"));
  clear_output();RadarHostConsole_Process();assert(!strstr(Page,"host_map") && !strstr(Page,"host_path"));
  command("radar host result -1");assert(strstr(Reply,"plan"));
}

static void test_results_preempt_preview_and_reduce_preview_cadence(void)
{
  char command_text[64];
  HostStatus.live_active=true;HostStatus.scan_state=RADAR_SCAN_READY;HostStatus.preview_id=0U;
  HostStatus.scan_id=2U;HostStatus.map_id=3U;HostStatus.plan_id=4U;
  Result.waypoint_count=2U;PreviewCount=1U;RadarHostConsole_Init();
  for(unsigned cycle=0U;cycle<4U;cycle++){
    unsigned plan=4U+cycle;
    snprintf(command_text,sizeof command_text,"radar host result %u",plan);command(command_text);
    for(unsigned i=0U;i<300U;i++){
      clear_output();RadarHostConsole_Process();assert(Page[0]);
      assert(!strstr(Page,"host_preview"));
      if(strstr(Page,"host_result_end"))break;
    }
    assert(strstr(Page,"host_result_end"));
    snprintf(command_text,sizeof command_text,"radar host ack %u",plan);command(command_text);
    command(command_text);assert(strstr(Reply,"result_acknowledged")); /* repeats do not count */
    HostStatus.scan_id++;HostStatus.map_id++;HostStatus.plan_id++;HostStatus.preview_id++;
    clear_output();RadarHostConsole_Process();
    assert((strstr(Page,"host_preview")!=NULL)==(cycle==3U));
  }
  /* Preempt a multi-page preview with the next result, not page_pending. */
  RadarHostConsole_Init();PreviewCount=RADAR_LIVE_BINS;HostStatus.preview_id=100U;
  clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_preview") && field("pages")>1U);
  command("radar host result 8");assert(!strstr(Reply,"ERR"));
  RadarHostConsole_Process();assert(strstr(Page,"host_map"));
  HostStatus.live_active=false;
}

static void test_fast_result_bounds_and_periodic_preview(void)
{
  unsigned received=0U,pages=0U;char wire[64];
  HostStatus.session_id=HostStatus.config_id=HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=UINT32_MAX;
  HostStatus.active=HostStatus.live_active=false;HostStatus.scan_state=RADAR_SCAN_READY;
  Result.waypoint_count=RADAR_HOST_MAX_WAYPOINTS;
  RadarHostConsole_Init();command("radar host status fast");
  for(unsigned i=0U;i<40U;i++){
    clear_output();RadarHostConsole_Process();assert(strlen(Page)<=256U);
    if(strstr(Page,"\"k\":\"r\"")){assert(field("p")==received++);pages=field("n");assert(field("t")==158U+10U*RADAR_HOST_MAX_WAYPOINTS);}
    if(strstr(Page,"\"k\":\"e\""))break;
  }
  assert(received==29U && received==pages);
  HostStatus.session_id=HostStatus.config_id=1U;HostStatus.scan_id=HostStatus.map_id=HostStatus.plan_id=2U;
  HostStatus.live_active=true;HostStatus.live_state=RADAR_LIVE_WAITING;HostStatus.preview_id=10U;
  PreviewCount=RADAR_LIVE_BINS;Result.waypoint_count=2U;
  RadarHostConsole_Init();command("radar host live start fast");
  for(unsigned cycle=0U;cycle<4U;cycle++){
    for(unsigned i=0U;i<20U;i++){clear_output();RadarHostConsole_Process();if(strstr(Page,"\"k\":\"e\""))break;}
    assert(strstr(Page,"\"k\":\"e\""));
    snprintf(wire,sizeof wire,"radar host ack %lu",(unsigned long)HostStatus.plan_id);command(wire);
    HostStatus.scan_id++;HostStatus.map_id++;HostStatus.plan_id++;HostStatus.preview_id++;
  }
  /* Keep the established 4-result preview cadence even with automatic results. */
  clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_preview"));
  pages=field("pages");
  for(unsigned page=1U;page<pages;page++){
    clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_preview") && field("page")==page);
  }
  clear_output();RadarHostConsole_Process();assert(strstr(Page,"host_status"));
  clear_output();RadarHostConsole_Process();assert(strstr(Page,"\"k\":\"r\""));
  command("radar host stop");HostStatus.live_active=false;
}

#ifdef RADAR_HOST_WIRE_FIXTURE
int main(void)
{
  memset(&HostStatus,0,sizeof HostStatus);memset(&Result,0,sizeof Result);
  RadarHostMap_Defaults(&Active);Draft=Active;
  assert(RadarHostMap_Build(&Active,NULL,0U,&Result));assert(Result.waypoint_count==33U);
  HostStatus.session_id=41U;HostStatus.config_id=1U;HostStatus.reason="idle";
  RadarHostConsole_Init();
  CloudCount=3U;Cloud[0]=(RadarSample_t){0U,100U,7U};Cloud[1]=(RadarSample_t){901U,65535U,255U};Cloud[2]=(RadarSample_t){3599U,230U,1U};
  HostStatus.scan_id=2U;HostStatus.map_id=3U;HostStatus.plan_id=4U;
  HostStatus.map_valid=HostStatus.path_valid=HostStatus.cloud_valid=true;HostStatus.reason="complete";
  /* Config/cloud remain explicit diagnostics; normal completion only notifies. */
  command("radar host get active");
  for(unsigned i=0U;i<40U;i++){
    clear_output();RadarHostConsole_Process();if(Page[0])fwrite(Page,1U,strlen(Page),stdout);
  }
  command("radar host result 4");
  do {
    clear_output();RadarHostConsole_Process();assert(Page[0]);fwrite(Page,1U,strlen(Page),stdout);
  } while(!strstr(Page,"host_result_end"));
  for(unsigned kind=2U;kind<3U;kind++){
    unsigned pages=1U;char wire[64];
    const char *name=kind==0U?"map":kind==1U?"path":"cloud";
    unsigned id=kind==0U?3U:kind==1U?4U:2U;
    for(unsigned page=0U;page<pages;page++){
      snprintf(wire,sizeof wire,"radar host fetch %s:%u %u",name,id,page);
      command(wire);RadarHostConsole_Process();pages=field("pages");fwrite(Page,1U,strlen(Page),stdout);
    }
  }
  for(unsigned p=0U;p<36U;p++){
    char wire[80];snprintf(wire,sizeof wire,"radar host fetch config:1 %u",p);command(wire);RadarHostConsole_Process();
    assert(strstr(Page,"host_config") && field("scan")==2U);fwrite(Page,1U,strlen(Page),stdout);
  }
  command("radar host fetch stable:2 0");RadarHostConsole_Process();assert(strstr(Page,"host_stable"));fwrite(Page,1U,strlen(Page),stdout);
  command("radar host ack 4");
  /* Same real 33-point result, compact and pushed without a result request. */
  RadarHostConsole_Init();HostStatus.scan_state=RADAR_SCAN_READY;
  command("radar host status fast");
  do {
    clear_output();RadarHostConsole_Process();assert(Page[0]);fwrite(Page,1U,strlen(Page),stdout);
  } while(!strstr(Page,"\"k\":\"e\""));
  command("radar host ack 4");
  /* Independently exercise the preview-only fallback before a result ACK. */
  HostStatus.scan_state=RADAR_SCAN_IDLE;RadarHostConsole_Init();
  HostStatus.live_active=true;HostStatus.preview_id=10U;PreviewCount=RADAR_LIVE_BINS;
  for(unsigned i=0U;i<PreviewCount;i++)Preview[i]=(RadarLive_Point_t){(uint16_t)(i*10U),(uint16_t)(1000U+i)};
  /* Real C pages must still produce two whole JS snapshots when the next
   * circle arrives before the previous wireless transfer has finished. */
  for(unsigned circle=0U;circle<2U;circle++){
    unsigned sent=0U,pages=1U;
    while(sent<pages){
      clear_output();RadarHostConsole_Process();assert(Page[0]);fwrite(Page,1U,strlen(Page),stdout);
      if(!strstr(Page,"host_preview"))continue;
      assert(field("live_id")==10U+circle && field("page")==sent);pages=field("pages");sent++;
      HostStatus.preview_id=11U+circle;PreviewCount=113U;
      for(unsigned i=0U;i<PreviewCount;i++)Preview[i]=(RadarLive_Point_t){(uint16_t)(i*10U),(uint16_t)(5000U+1000U*circle+i)};
    }
  }
  return 0;
}
#else
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
  RadarHostConsole_Init();test_completion_notifies_without_bulk_dump();
  test_result_read_survives_new_live_circles();
  RadarHostConsole_Init();fetch_cloud(0U);fetch_cloud(1U);fetch_cloud(37U);fetch_cloud(4096U);
  test_preview_freezes_the_circle_until_every_page_is_sent();
  test_preview_uint32_identifiers_still_fit_the_wire_budget();
  test_result_uint32_identifiers_still_fit_the_wire_budget();
  test_config_read_stays_coherent_during_continuous_mapping();
  test_one_request_streams_frozen_results();
  test_results_preempt_preview_and_reduce_preview_cadence();
  test_fast_scan_pushes_compact_pair_without_result_request();
  test_stable_pages_freeze_xy_and_exceed_32_pages();
  test_shared_stable_snapshot_mode_handover();
  test_fast_result_bounds_and_periodic_preview();
  printf("radar_host_console_test: PASS (%u bounded pages, full cloud CRC)\n",PageCount);
}
#endif
