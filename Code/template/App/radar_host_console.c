/**
 * @file radar_host_console.c
 * @brief 将F407网页雷达结果序列化为不超过256字节的完整USART1行。
 */
#include "radar_host_console.h"

#include "console_tx.h"
#include "radar_host.h"

#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
  HOST_TRANSFER_NONE=0,
  HOST_TRANSFER_CONFIG,
  HOST_TRANSFER_MAP,
  HOST_TRANSFER_PATH,
  HOST_TRANSFER_PREVIEW,
  HOST_TRANSFER_CLOUD
} HostTransferKind_t;

typedef struct {
  HostTransferKind_t kind;
  uint32_t id;
  uint32_t page;
  bool draft;
  bool active;
} HostTransfer_t;

typedef struct {
  uint32_t session,config,scan,map,plan;
} HostIdentity_t;

#define HOST_CONFIG_RECORDS 27U
#define HOST_MAP_RECORDS    (3U+RADAR_MAP_CELL_COUNT)

static char Line[257];
static unsigned Length;
static bool FormatOk;
static HostTransfer_t Requested,Manual,Automatic;
static uint32_t SeenMap,SeenPreview;
static uint32_t CloudCrcScan;
static uint16_t CloudCrcCount;
static uint32_t CloudCrcValue;
static bool CloudCrcValid;

static const char *kind_name(HostTransferKind_t kind)
{
  static const char *const names[]={"none","host_config","host_map","host_path","host_preview","host_cloud"};
  return kind<=HOST_TRANSFER_CLOUD?names[kind]:names[0];
}

static void append(const char *format,...)
{
  va_list args;int count;
  if(!FormatOk)return;
  va_start(args,format);count=vsnprintf(Line+Length,sizeof Line-Length,format,args);va_end(args);
  if(count<0 || (unsigned)count>=sizeof Line-Length){FormatOk=false;return;}
  Length+=(unsigned)count;
}

static bool finish(bool foreground)
{
  append("}\r\n");
  if(!FormatOk || Length>256U)return false;
  return foreground?ConsoleTx_Write((const uint8_t *)Line,(uint16_t)Length):
                    ConsoleTx_Bulk(Line,(uint16_t)Length);
}

static void reply(bool ok,const char *reason,bool urgent)
{
  char text[112];int count=snprintf(text,sizeof text,"%s radar host %s\r\n",ok?"OK":"ERR",reason);
  if(count<=0 || count>256 || (unsigned)count>=sizeof text)return;
  if(urgent){if(!ConsoleTx_Urgent(text,(uint16_t)count))(void)ConsoleTx_Write((const uint8_t *)text,(uint16_t)count);}
  else (void)ConsoleTx_Write((const uint8_t *)text,(uint16_t)count);
}

static bool parse_u32(const char *text,uint32_t maximum,uint32_t *value)
{
  char *end;unsigned long parsed;
  if(text==NULL || *text=='\0' || *text=='-' || *text=='+')return false;
  errno=0;parsed=strtoul(text,&end,10);
  if(errno || *end!='\0' || parsed>maximum)return false;
  *value=(uint32_t)parsed;return true;
}

static HostIdentity_t identity(const RadarHost_Status_t *status)
{
  HostIdentity_t result={status->session_id,status->config_id,status->scan_id,status->map_id,status->plan_id};
  return result;
}

static void begin_transfer(HostTransferKind_t kind,const HostIdentity_t *id,
                           uint32_t page,uint32_t pages,uint32_t offset,uint32_t total)
{
  Length=0U;FormatOk=true;
  append("@RADAR {\"v\":1,\"profile\":\"f1_host\",\"k\":\"%s\","
         "\"session\":%lu,\"config\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":%lu,"
         "\"page\":%lu,\"pages\":%lu,\"offset\":%lu,\"total\":%lu,",
         kind_name(kind),(unsigned long)id->session,(unsigned long)id->config,
         (unsigned long)id->scan,(unsigned long)id->map,(unsigned long)id->plan,
         (unsigned long)page,(unsigned long)pages,(unsigned long)offset,(unsigned long)total);
}

static void status_page(void)
{
  RadarHost_Status_t status;char reason[25];size_t used=0U;
  RadarHost_StatusGet(&status);
  if(status.reason!=NULL) {
    while(status.reason[used]!='\0' && used+1U<sizeof reason) {
      char c=status.reason[used];reason[used]=(c=='"' || c=='\\')?'_':c;used++;
    }
  }
  reason[used]='\0';Length=0U;FormatOk=true;
  append("@RADAR {\"v\":1,\"profile\":\"f1_host\",\"k\":\"host_status\","
         "\"session\":%lu,\"config\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":%lu,"
         "\"a\":%u,\"l\":%u,\"ss\":%u,\"ls\":%u,\"cloud\":%u,\"preview\":%u,\"reason\":\"%s\"",
         (unsigned long)status.session_id,(unsigned long)status.config_id,
         (unsigned long)status.scan_id,(unsigned long)status.map_id,(unsigned long)status.plan_id,
         status.active?1U:0U,status.live_active?1U:0U,(unsigned)status.scan_state,
         (unsigned)status.live_state,status.cloud_points,status.preview_points,reason);
  if(!finish(true))reply(false,"page_size",false);
}

static unsigned base64_encode(const uint8_t *bytes,unsigned count,char *text,unsigned capacity)
{
  static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  unsigned source=0U,target=0U;
  while(source<count) {
    unsigned left=count-source;uint32_t value=(uint32_t)bytes[source++]<<16;
    if(left>1U)value|=(uint32_t)bytes[source++]<<8;
    if(left>2U)value|=bytes[source++];
    if(target+4U>=capacity)return 0U;
    text[target++]=alphabet[(value>>18)&63U];text[target++]=alphabet[(value>>12)&63U];
    text[target++]=left>1U?alphabet[(value>>6)&63U]:'=';
    text[target++]=left>2U?alphabet[value&63U]:'=';
  }
  text[target]='\0';return target;
}

static uint32_t crc32_update(uint32_t crc,const uint8_t *bytes,unsigned count)
{
  for(unsigned i=0U;i<count;i++) {
    crc^=bytes[i];
    for(unsigned bit=0U;bit<8U;bit++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);
  }
  return crc;
}

static uint32_t cloud_crc(const RadarSample_t *points,uint16_t count,uint32_t scan_id)
{
  uint32_t crc=0xFFFFFFFFUL;uint8_t bytes[5];
  if(CloudCrcValid && CloudCrcScan==scan_id && CloudCrcCount==count)return CloudCrcValue;
  for(uint16_t i=0U;i<count;i++) {
    bytes[0]=(uint8_t)points[i].angle_tenths;bytes[1]=(uint8_t)(points[i].angle_tenths>>8);
    bytes[2]=(uint8_t)points[i].distance_mm;bytes[3]=(uint8_t)(points[i].distance_mm>>8);
    bytes[4]=points[i].energy;crc=crc32_update(crc,bytes,sizeof bytes);
  }
  CloudCrcScan=scan_id;CloudCrcCount=count;CloudCrcValue=~crc;CloudCrcValid=true;return CloudCrcValue;
}

static uint32_t preview_crc(const RadarLive_Point_t *points,uint16_t count)
{
  uint32_t crc=0xFFFFFFFFUL;uint8_t bytes[4];
  for(uint16_t i=0U;i<count;i++) {
    bytes[0]=(uint8_t)points[i].angle_tenths;bytes[1]=(uint8_t)(points[i].angle_tenths>>8);
    bytes[2]=(uint8_t)points[i].distance_mm;bytes[3]=(uint8_t)(points[i].distance_mm>>8);
    crc=crc32_update(crc,bytes,sizeof bytes);
  }
  return ~crc;
}

static unsigned packed_capacity(HostTransferKind_t kind,const HostIdentity_t *id,
                                uint32_t total,unsigned bytes_per_point)
{
  unsigned best=1U;
  if(total==0U)return 1U;
  for(unsigned count=1U;count<=24U;count++) {
    uint32_t pages=(total+count-1U)/count;
    uint32_t page=pages-1U,offset=page*count;
    unsigned data=(bytes_per_point*count+2U)/3U*4U;
    begin_transfer(kind,id,page,pages,offset,total);
    if(FormatOk && Length+29U+data+4U<=256U)best=count;
    else break;
  }
  return best;
}

static bool config_record(const RadarHostMap_Config_t *config,unsigned record,char *text,unsigned capacity)
{
  int count=-1;
  switch(record) {
    case 0:count=snprintf(text,capacity,"source,active");break;
    case 1:count=snprintf(text,capacity,"width,2400");break;
    case 2:count=snprintf(text,capacity,"height,2400");break;
    case 3:case 4:case 5:case 6:count=snprintf(text,capacity,"x%u,%d",record-2U,config->grid.x_mm[record-2U]);break;
    case 7:case 8:case 9:case 10:count=snprintf(text,capacity,"y%u,%d",record-6U,config->grid.y_mm[record-6U]);break;
    case 11:count=snprintf(text,capacity,"radar_x,%u",config->radar_x_mm);break;
    case 12:count=snprintf(text,capacity,"radar_y,%u",config->radar_y_mm);break;
    case 13:count=snprintf(text,capacity,"angle_min,%u",config->angle_min_tenths);break;
    case 14:count=snprintf(text,capacity,"angle_max,%u",config->angle_max_tenths);break;
    case 15:count=snprintf(text,capacity,"distance_min,%u",config->distance_min_mm);break;
    case 16:count=snprintf(text,capacity,"distance_max,%u",config->distance_max_mm);break;
    case 17:count=snprintf(text,capacity,"energy_min,%u",config->energy_min);break;
    case 18:count=snprintf(text,capacity,"energy_max,%u",config->energy_max);break;
    case 19:count=snprintf(text,capacity,"threshold,%u",config->threshold);break;
    case 20:count=snprintf(text,capacity,"fixed,%08lX",(unsigned long)config->fixed_mask);break;
    case 21:count=snprintf(text,capacity,"manual,%08lX",(unsigned long)config->manual_mask);break;
    case 22:count=snprintf(text,capacity,"protected,%08lX",(unsigned long)config->protected_mask);break;
    case 23:count=snprintf(text,capacity,"planning,%s",config->planning_mode==RADAR_HOST_PLAN_POINT?"point":"mission");break;
    case 24:
      count=snprintf(text,capacity,"mission");
      for(unsigned i=0U;count>0 && i<config->mission_count;i++) {
        int added=snprintf(text+(unsigned)count,capacity-(unsigned)count,",%u",config->mission[i]);
        if(added<0 || (unsigned)(count+added)>=capacity){count=-1;break;}count+=added;
      }
      break;
    case 25:count=snprintf(text,capacity,"start,%u,%u,%u",config->start.row,config->start.column,config->start.anchor);break;
    case 26:count=snprintf(text,capacity,"goal,%u,%u,%u",config->goal.row,config->goal.column,config->goal.anchor);break;
    default:return false;
  }
  return count>=0 && (unsigned)count<capacity;
}

static bool render_config(const HostTransfer_t *transfer,const RadarHost_Status_t *status,uint32_t *pages)
{
  const RadarHostMap_Config_t *config=transfer->draft?RadarHost_DraftConfig():RadarHost_ActiveConfig();
  HostIdentity_t id=identity(status);char data[64];
  if(config==NULL || transfer->page>=HOST_CONFIG_RECORDS)return false;
  if(transfer->draft)id.config=transfer->id;else if(transfer->id!=status->config_id)return false;
  if(!config_record(config,(unsigned)transfer->page,data,sizeof data))return false;
  if(transfer->draft && transfer->page==0U)strcpy(data,"source,draft");
  *pages=HOST_CONFIG_RECORDS;begin_transfer(HOST_TRANSFER_CONFIG,&id,transfer->page,*pages,transfer->page,HOST_CONFIG_RECORDS);
  append("\"data\":\"%s\"",data);return finish(false);
}

static bool render_map(const HostTransfer_t *transfer,const RadarHost_Status_t *status,uint32_t *pages)
{
  uint32_t map_id,scan_id;const RadarHostMap_Result_t *map=RadarHost_Map(&map_id,&scan_id);
  HostIdentity_t id=identity(status);char data[64];int count;
  if(map==NULL || map_id!=transfer->id || transfer->page>=HOST_MAP_RECORDS)return false;
  id.map=map_id;id.scan=scan_id;*pages=HOST_MAP_RECORDS;
  if(transfer->page==0U)count=snprintf(data,sizeof data,"s,%u,%u,%u",map->raw_points,map->filtered_points,map->outside_points);
  else if(transfer->page==1U)count=snprintf(data,sizeof data,"m,%08lX,%08lX,%08lX",(unsigned long)map->scanned_mask,(unsigned long)map->manual_mask,(unsigned long)map->effective_mask);
  else if(transfer->page==2U)count=snprintf(data,sizeof data,"p,%u,%u,%u",map->path_valid?1U:0U,map->failed_from_task,map->failed_to_task);
  else count=snprintf(data,sizeof data,"c,%lu,%u",(unsigned long)(transfer->page-3U),map->counts[transfer->page-3U]);
  if(count<0 || (unsigned)count>=sizeof data)return false;
  begin_transfer(HOST_TRANSFER_MAP,&id,transfer->page,*pages,transfer->page,HOST_MAP_RECORDS);
  append("\"data\":\"%s\"",data);return finish(false);
}

static bool render_path(const HostTransfer_t *transfer,const RadarHost_Status_t *status,uint32_t *pages)
{
  uint32_t plan_id,scan_id;const RadarHostMap_Result_t *path=RadarHost_Path(&plan_id,&scan_id);
  HostIdentity_t id=identity(status);char data[64];int count;
  if(path==NULL || plan_id!=transfer->id)return false;
  *pages=path->waypoint_count?path->waypoint_count:1U;if(transfer->page>=*pages)return false;
  id.plan=plan_id;id.scan=scan_id;
  if(path->waypoint_count==0U)count=snprintf(data,sizeof data,"n,%u,%u,%u",path->path_valid?1U:0U,path->failed_from_task,path->failed_to_task);
  else {
    const RadarHostMap_Waypoint_t *point=&path->waypoints[transfer->page];
    count=snprintf(data,sizeof data,"w,%u,%u,%u,%u,%u,%u,%u",point->x_mm,point->y_mm,
      point->distance_to_next_mm,point->row,point->column,point->direction,point->turn);
  }
  if(count<0 || (unsigned)count>=sizeof data)return false;
  begin_transfer(HOST_TRANSFER_PATH,&id,transfer->page,*pages,transfer->page,path->waypoint_count);
  append("\"data\":\"%s\"",data);return finish(false);
}

static bool render_packed(const HostTransfer_t *transfer,const RadarHost_Status_t *status,uint32_t *pages)
{
  HostIdentity_t id=identity(status);const RadarSample_t *cloud=NULL;const RadarLive_Point_t *preview=NULL;
  uint16_t total=0U;uint32_t source_id=0U,crc;unsigned bytes_per_point,capacity,count;uint32_t offset;
  uint8_t raw[120];char data[164];
  if(transfer->kind==HOST_TRANSFER_CLOUD) {
    cloud=RadarHost_Cloud(&total,&source_id);bytes_per_point=5U;
    if(cloud==NULL || source_id!=transfer->id)return false;
    id.scan=source_id;
    crc=cloud_crc(cloud,total,source_id);
  } else {
    preview=RadarHost_Preview(&total,&source_id);bytes_per_point=4U;
    if(preview==NULL || source_id!=transfer->id)return false;
    crc=preview_crc(preview,total);
  }
  capacity=packed_capacity(transfer->kind,&id,total,bytes_per_point);
  *pages=total?(total+capacity-1U)/capacity:1U;if(transfer->page>=*pages)return false;
  offset=transfer->page*capacity;count=total-offset;if(count>capacity)count=capacity;
  for(unsigned i=0U;i<count;i++) {
    if(cloud!=NULL) {
      raw[i*5U]=(uint8_t)cloud[offset+i].angle_tenths;raw[i*5U+1U]=(uint8_t)(cloud[offset+i].angle_tenths>>8);
      raw[i*5U+2U]=(uint8_t)cloud[offset+i].distance_mm;raw[i*5U+3U]=(uint8_t)(cloud[offset+i].distance_mm>>8);
      raw[i*5U+4U]=cloud[offset+i].energy;
    } else {
      raw[i*4U]=(uint8_t)preview[offset+i].angle_tenths;raw[i*4U+1U]=(uint8_t)(preview[offset+i].angle_tenths>>8);
      raw[i*4U+2U]=(uint8_t)preview[offset+i].distance_mm;raw[i*4U+3U]=(uint8_t)(preview[offset+i].distance_mm>>8);
    }
  }
  if(base64_encode(raw,count*bytes_per_point,data,sizeof data)==0U && count!=0U)return false;
  begin_transfer(transfer->kind,&id,transfer->page,*pages,offset,total);
  append("\"crc32\":\"%08lX\",\"data\":\"%s\"",(unsigned long)crc,data);return finish(false);
}

static bool render(const HostTransfer_t *transfer,uint32_t *pages)
{
  RadarHost_Status_t status;RadarHost_StatusGet(&status);
  switch(transfer->kind) {
    case HOST_TRANSFER_CONFIG:return render_config(transfer,&status,pages);
    case HOST_TRANSFER_MAP:return render_map(transfer,&status,pages);
    case HOST_TRANSFER_PATH:return render_path(transfer,&status,pages);
    case HOST_TRANSFER_PREVIEW:case HOST_TRANSFER_CLOUD:return render_packed(transfer,&status,pages);
    default:return false;
  }
}

static void cancel_transfers(void)
{
  memset(&Requested,0,sizeof Requested);memset(&Manual,0,sizeof Manual);memset(&Automatic,0,sizeof Automatic);
}

static void automatic_begin(const RadarHost_Status_t *status)
{
  Automatic.kind=HOST_TRANSFER_CONFIG;Automatic.id=status->config_id;Automatic.page=0U;
  Automatic.draft=false;Automatic.active=true;
}

static void automatic_next(const RadarHost_Status_t *status)
{
  Automatic.page=0U;
  if(Automatic.kind==HOST_TRANSFER_CONFIG){Automatic.kind=HOST_TRANSFER_MAP;Automatic.id=status->map_id;}
  else if(Automatic.kind==HOST_TRANSFER_MAP){Automatic.kind=HOST_TRANSFER_PATH;Automatic.id=status->plan_id;}
  else if(Automatic.kind==HOST_TRANSFER_PATH){Automatic.kind=HOST_TRANSFER_CLOUD;Automatic.id=status->scan_id;}
  else memset(&Automatic,0,sizeof Automatic);
}

void RadarHostConsole_Init(void)
{
  RadarHost_Status_t status;
  RadarHost_Init();cancel_transfers();CloudCrcValid=false;
  RadarHost_StatusGet(&status);SeenMap=status.map_id;SeenPreview=status.preview_id;
}

void RadarHostConsole_Process(void)
{
  RadarHost_Status_t status;HostTransfer_t *transfer=NULL;uint32_t pages=0U;
  RadarHost_Process();RadarHost_StatusGet(&status);
  if(status.map_valid && status.map_id!=SeenMap){SeenMap=status.map_id;automatic_begin(&status);}
  if(status.preview_id!=0U && status.preview_id!=SeenPreview && !Manual.active) {
    SeenPreview=status.preview_id;Manual=(HostTransfer_t){HOST_TRANSFER_PREVIEW,status.preview_id,0U,false,true};
  }
  if(!ConsoleTx_BulkReady())return;
  if(Requested.active)transfer=&Requested;
  else if(Manual.active)transfer=&Manual;
  else if(Automatic.active)transfer=&Automatic;
  else return;
  if(!render(transfer,&pages)) {
    bool requested=transfer==&Requested;memset(transfer,0,sizeof *transfer);
    if(requested)reply(false,"snapshot_or_page",false);
    return;
  }
  if(transfer==&Requested){memset(&Requested,0,sizeof Requested);return;}
  transfer->page++;
  if(transfer->page>=pages) {
    if(transfer==&Automatic)automatic_next(&status);
    else memset(transfer,0,sizeof *transfer);
  }
}

static HostTransferKind_t parse_kind_id(const char *text,uint32_t *id)
{
  char kind[12];const char *colon=strchr(text,':');size_t length;
  if(colon==NULL || strchr(colon+1,':')!=NULL)return HOST_TRANSFER_NONE;
  length=(size_t)(colon-text);if(length==0U || length>=sizeof kind || !parse_u32(colon+1,UINT32_MAX,id))return HOST_TRANSFER_NONE;
  memcpy(kind,text,length);kind[length]='\0';
  if(!strcmp(kind,"config"))return HOST_TRANSFER_CONFIG;
  if(!strcmp(kind,"map"))return HOST_TRANSFER_MAP;
  if(!strcmp(kind,"path"))return HOST_TRANSFER_PATH;
  if(!strcmp(kind,"preview"))return HOST_TRANSFER_PREVIEW;
  if(!strcmp(kind,"cloud"))return HOST_TRANSFER_CLOUD;
  return HOST_TRANSFER_NONE;
}

bool RadarHostConsole_Command(unsigned count,char *tokens[])
{
  RadarHost_Status_t status;uint32_t value,page;HostTransferKind_t kind;bool ok;
  if(count<2U || strcmp(tokens[0],"radar") || strcmp(tokens[1],"host"))return false;
  if(count==3U && !strcmp(tokens[2],"status")){status_page();return true;}
  if(count==3U && !strcmp(tokens[2],"start")){
    ok=RadarHost_StartRotation();reply(ok,ok?"start_accepted":"start_busy_or_uart",false);return true;
  }
  if(count==3U && !strcmp(tokens[2],"scan")){
    ok=RadarHost_Scan();reply(ok,ok?"scan_accepted":"scan_busy_or_uart",false);return true;
  }
  if(count==3U && !strcmp(tokens[2],"stop")){
    RadarHost_Stop();cancel_transfers();reply(true,"stop_requested",true);return true;
  }
  if(count==4U && !strcmp(tokens[2],"live")) {
    if(!strcmp(tokens[3],"status")){status_page();return true;}
    if(!strcmp(tokens[3],"start")){ok=RadarHost_LiveStart();reply(ok,ok?"live_start_accepted":"live_busy_or_uart",false);return true;}
    if(!strcmp(tokens[3],"stop")){RadarHost_LiveStop();cancel_transfers();reply(true,"live_stop_requested",true);return true;}
  }
  if(count==4U && !strcmp(tokens[2],"draft") && !strcmp(tokens[3],"reset")) {
    (void)RadarHost_DraftReset();reply(true,"draft_reset",false);return true;
  }
  if(count==5U && !strcmp(tokens[2],"set")) {
    ok=RadarHost_DraftSet(tokens[3],tokens[4]);reply(ok,ok?"draft_updated":"draft_value",false);return true;
  }
  if(count==4U && !strcmp(tokens[2],"apply")) {
    if(!parse_u32(tokens[3],UINT32_MAX,&value)){reply(false,"revision",false);return true;}
    ok=RadarHost_Apply(value);reply(ok,ok?"config_applied":"revision_or_config",false);return true;
  }
  if(count==4U && !strcmp(tokens[2],"get") && (!strcmp(tokens[3],"active") || !strcmp(tokens[3],"draft"))) {
    RadarHost_StatusGet(&status);
    if(Manual.active || Requested.active){reply(false,"page_pending",false);return true;}
    Manual.kind=HOST_TRANSFER_CONFIG;Manual.page=0U;Manual.draft=!strcmp(tokens[3],"draft");
    Manual.id=Manual.draft?status.draft_revision:status.config_id;
    if(Manual.draft && RadarHost_DraftConfig()==NULL){reply(false,"no_draft",false);memset(&Manual,0,sizeof Manual);return true;}
    Manual.active=true;return true;
  }
  if(count==5U && !strcmp(tokens[2],"fetch")) {
    kind=parse_kind_id(tokens[3],&value);
    if(kind==HOST_TRANSFER_NONE){reply(false,"fetch_target",false);return true;}
    if(!parse_u32(tokens[4],65535U,&page)){reply(false,"page",false);return true;}
    if(Requested.active){reply(false,"page_pending",false);return true;}
    Requested=(HostTransfer_t){kind,value,page,false,true};return true;
  }
  reply(false,"format",false);return true;
}
