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

typedef struct {
  RadarLive_Point_t points[RADAR_LIVE_BINS];
  HostIdentity_t identity;
  uint32_t id,crc;
  uint16_t count;
  bool valid;
} HostPreview_t;

typedef struct {
  RadarHostMap_Result_t result;
  HostIdentity_t identity;
  bool valid;
  bool delivered;
  bool live;
} HostResult_t;

typedef struct {
  RadarHostMap_Config_t config;
  HostIdentity_t identity;
  bool valid,draft;
} HostConfig_t;

#define HOST_CONFIG_RECORDS 27U
#define HOST_MAP_RECORDS    (3U+RADAR_MAP_CELL_COUNT)

static char Line[257];
static unsigned Length;
static bool FormatOk;
static HostTransfer_t Requested,Manual;
static HostPreview_t FrozenPreview;
static HostResult_t FrozenResult;
static HostConfig_t FrozenConfig;
static bool PreviewDue;
static bool ResultStreaming,ResultEndPending;
static HostIdentity_t StreamIdentity;
static uint32_t StreamMapPages,StreamPathPages;
static unsigned ResultsSincePreview;
static uint32_t AcknowledgedPlan;
static uint32_t SeenPreview;
static RadarHost_Status_t ReportedStatus;
static uint32_t CloudCrcScan;
static uint16_t CloudCrcCount;
static uint32_t CloudCrcValue;
static bool CloudCrcValid;
/* Fast mode only changes the radar page's wire format, not its computation. */
#define HOST_BUNDLE_BYTES 84U
#define HOST_BUNDLE_HEADER 74U
static bool FastEnabled,FastEndPending;
static uint32_t FastPending,FastAutoPlan;

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

/* The advertised map/path pair remains readable until the browser confirms it.
 * Continuous sampling is much faster than the paced wireless download. */
static bool freeze_result(const RadarHost_Status_t *status)
{
  uint32_t map_id,scan_id;
  const RadarHostMap_Result_t *result;
  if(FrozenResult.valid)return true;
  result=RadarHost_Map(&map_id,&scan_id);
  if(!result || !status->map_valid || !status->plan_id)return false;
  FrozenResult.result=*result;FrozenResult.identity=identity(status);
  FrozenResult.identity.map=map_id;FrozenResult.identity.scan=scan_id;
  FrozenResult.valid=true;FrozenResult.delivered=false;FrozenResult.live=status->live_active;return true;
}

static void result_status(RadarHost_Status_t *status)
{
  if(!status->map_valid){status->map_id=status->plan_id=0U;return;}
  if(!FrozenResult.valid)return;
  status->scan_id=FrozenResult.identity.scan;status->map_id=FrozenResult.identity.map;
  status->plan_id=FrozenResult.identity.plan;
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

static bool status_page(bool foreground)
{
  RadarHost_Status_t status;char reason[25];size_t used=0U;
  RadarHost_StatusGet(&status);
  if(status.scan_state==RADAR_SCAN_READY)(void)freeze_result(&status);
  if(status.scan_state==RADAR_SCAN_READY || !status.map_valid)result_status(&status);
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
  if(FastEnabled)append(",\"fast\":1");
  if(!finish(foreground))return false;
  ReportedStatus=status;return true;
}

static bool status_changed(const RadarHost_Status_t *status)
{
  RadarHost_Status_t current=*status;
  if(current.scan_state==RADAR_SCAN_READY || !current.map_valid)result_status(&current);
  status=&current;
  return status->session_id!=ReportedStatus.session_id || status->config_id!=ReportedStatus.config_id ||
    status->scan_id!=ReportedStatus.scan_id || status->map_id!=ReportedStatus.map_id ||
    status->plan_id!=ReportedStatus.plan_id || status->active!=ReportedStatus.active ||
    status->live_active!=ReportedStatus.live_active || status->scan_state!=ReportedStatus.scan_state ||
    status->live_state!=ReportedStatus.live_state || status->reason!=ReportedStatus.reason;
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

static unsigned bundle_size(void)
{
  return HOST_BUNDLE_HEADER+10U*FrozenResult.result.waypoint_count;
}
static unsigned bundle_pages(void)
{
  return (bundle_size()+HOST_BUNDLE_BYTES-1U)/HOST_BUNDLE_BYTES;
}
static void put16(uint8_t *bytes,uint16_t value)
{
  bytes[0]=(uint8_t)value;bytes[1]=(uint8_t)(value>>8);
}
static void put32(uint8_t *bytes,uint32_t value)
{
  for(unsigned i=0U;i<4U;i++)bytes[i]=(uint8_t)(value>>(8U*i));
}
static bool finish_bundle(void)
{
  uint32_t crc;
  if(!FormatOk)return false;
  crc=crc32_update(0xFFFFFFFFUL,(const uint8_t *)Line+7U,Length-7U);
  crc=crc32_update(crc,(const uint8_t *)"}",1U);
  append(",\"c\":\"%08lX\"",(unsigned long)~crc);
  return finish(false);
}
static void begin_bundle(const char *kind)
{
  const HostIdentity_t *id=&FrozenResult.identity;
  Length=0U;FormatOk=true;
  append("@RADAR {\"v\":2,\"k\":\"%s\",\"i\":[%lu,%lu,%lu,%lu,%lu],\"l\":%u",kind,
    (unsigned long)id->session,(unsigned long)id->config,(unsigned long)id->scan,
    (unsigned long)id->map,(unsigned long)id->plan,FrozenResult.live?1U:0U);
}
/** @brief Serialize all map statistics and every waypoint without struct padding. */
static bool render_bundle(unsigned page)
{
  uint8_t header[HOST_BUNDLE_HEADER],raw[HOST_BUNDLE_BYTES];char encoded[116];
  const RadarHostMap_Result_t *r=&FrozenResult.result;
  unsigned offset=page*HOST_BUNDLE_BYTES,count,total=bundle_size();
  if(!FrozenResult.valid || page>=bundle_pages())return false;
  header[0]=1U;put16(header+1U,r->raw_points);put16(header+3U,r->filtered_points);put16(header+5U,r->outside_points);
  put32(header+7U,r->scanned_mask);put32(header+11U,r->manual_mask);put32(header+15U,r->effective_mask);
  header[19]=r->path_valid?1U:0U;header[20]=r->failed_from_task;header[21]=r->failed_to_task;
  put16(header+22U,r->waypoint_count);
  for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)put16(header+24U+2U*i,r->counts[i]);
  count=total-offset;if(count>HOST_BUNDLE_BYTES)count=HOST_BUNDLE_BYTES;
  for(unsigned i=0U;i<count;i++){
    unsigned index=offset+i;
    if(index<HOST_BUNDLE_HEADER)raw[i]=header[index];
    else {
      uint8_t point[10];const RadarHostMap_Waypoint_t *p=&r->waypoints[(index-HOST_BUNDLE_HEADER)/10U];
      put16(point,p->x_mm);put16(point+2U,p->y_mm);put16(point+4U,p->distance_to_next_mm);
      point[6]=p->row;point[7]=p->column;point[8]=p->direction;point[9]=p->turn;
      raw[i]=point[(index-HOST_BUNDLE_HEADER)%10U];
    }
  }
  if(!base64_encode(raw,count,encoded,sizeof encoded))return false;
  begin_bundle("r");append(",\"p\":%u,\"n\":%u,\"t\":%u,\"d\":\"%s\"",page,bundle_pages(),total,encoded);
  return finish_bundle();
}
static bool bundle_end(void)
{
  begin_bundle("e");append(",\"n\":%u",bundle_pages());return finish_bundle();
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

/**
 * @brief 冻结一圈预览，后续分页及补读不再取正在更新的采集缓存。
 * @param preview_id 要读取的完整圈编号；同次MCU运行内非零递增。
 * @param status 当前雷达服务状态，用于冻结配置及地图版本。
 * @retval true 已有该圈或成功复制当前圈；false 该圈不可用或另一圈正在发送。
 * @note 仅由主循环调用；保留已发送圈直到下一圈开始发送。
 */
static bool freeze_preview(uint32_t preview_id,const RadarHost_Status_t *status)
{
  uint16_t count;uint32_t source_id;
  const RadarLive_Point_t *points;
  if(FrozenPreview.valid && FrozenPreview.id==preview_id)return true;
  if(Manual.active && Manual.kind==HOST_TRANSFER_PREVIEW)return false;
  points=RadarHost_Preview(&count,&source_id);
  if(points==NULL || source_id!=preview_id || count>RADAR_LIVE_BINS)return false;
  memcpy(FrozenPreview.points,points,count*sizeof *points);
  FrozenPreview.identity=identity(status);FrozenPreview.id=source_id;FrozenPreview.count=count;
  FrozenPreview.crc=preview_crc(FrozenPreview.points,count);FrozenPreview.valid=true;
  return true;
}

static unsigned packed_capacity(HostTransferKind_t kind,const HostIdentity_t *id,
                                uint32_t total,unsigned bytes_per_point,uint32_t preview_id)
{
  unsigned best=1U;
  if(total==0U)return 1U;
  for(unsigned count=1U;count<=24U;count++) {
    uint32_t pages=(total+count-1U)/count;
    uint32_t page=pages-1U,offset=page*count;
    unsigned data=(bytes_per_point*count+2U)/3U*4U;
    begin_transfer(kind,id,page,pages,offset,total);
    if(kind==HOST_TRANSFER_PREVIEW)append("\"live_id\":%lu,",(unsigned long)preview_id);
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
  char data[64];
  if(config==NULL || transfer->page>=HOST_CONFIG_RECORDS)return false;
  if(!transfer->draft && transfer->id!=status->config_id)return false;
  if(!FrozenConfig.valid || FrozenConfig.draft!=transfer->draft || FrozenConfig.identity.config!=transfer->id) {
    /* New scan ids must not split a 27-page RAM read into different transfers. */
    FrozenConfig.config=*config;FrozenConfig.identity=identity(status);
    FrozenConfig.identity.config=transfer->id;FrozenConfig.draft=transfer->draft;FrozenConfig.valid=true;
  }
  if(!config_record(&FrozenConfig.config,(unsigned)transfer->page,data,sizeof data))return false;
  if(transfer->draft && transfer->page==0U)strcpy(data,"source,draft");
  *pages=HOST_CONFIG_RECORDS;begin_transfer(HOST_TRANSFER_CONFIG,&FrozenConfig.identity,transfer->page,*pages,transfer->page,HOST_CONFIG_RECORDS);
  append("\"data\":\"%s\"",data);return finish(false);
}

static bool result_record(HostTransferKind_t kind,const RadarHostMap_Result_t *result,
                          unsigned record,char *data,unsigned capacity)
{
  int count;
  if(kind==HOST_TRANSFER_MAP) {
    if(record==0U)count=snprintf(data,capacity,"s,%u,%u,%u",result->raw_points,result->filtered_points,result->outside_points);
    else if(record==1U)count=snprintf(data,capacity,"m,%08lX,%08lX,%08lX",(unsigned long)result->scanned_mask,(unsigned long)result->manual_mask,(unsigned long)result->effective_mask);
    else if(record==2U)count=snprintf(data,capacity,"p,%u,%u,%u",result->path_valid?1U:0U,result->failed_from_task,result->failed_to_task);
    else count=snprintf(data,capacity,"c,%u,%u",record-3U,result->counts[record-3U]);
  } else if(!result->waypoint_count)
    count=snprintf(data,capacity,"n,%u,%u,%u",result->path_valid?1U:0U,result->failed_from_task,result->failed_to_task);
  else {
    const RadarHostMap_Waypoint_t *point=&result->waypoints[record];
    count=snprintf(data,capacity,"w,%u,%u,%u,%u,%u,%u,%u",point->x_mm,point->y_mm,
      point->distance_to_next_mm,point->row,point->column,point->direction,point->turn);
  }
  return count>=0 && (unsigned)count<capacity;
}

static bool render_result(const HostTransfer_t *transfer,const RadarHost_Status_t *status,uint32_t *pages)
{
  char data[128],record[64];unsigned used=0U,page=0U,offset=0U,selected=0U,budget,total,records;
  const RadarHostMap_Result_t *result;
  const HostIdentity_t *id;
  if(!freeze_result(status))return false;
  result=&FrozenResult.result;id=&FrozenResult.identity;
  if(transfer->id!=(transfer->kind==HOST_TRANSFER_MAP?id->map:id->plan))return false;
  total=transfer->kind==HOST_TRANSFER_MAP?HOST_MAP_RECORDS:result->waypoint_count;
  records=total?total:1U;
  /* Reserve the largest possible page/offset header, then pack whole records. */
  begin_transfer(transfer->kind,id,records,records,records,total);
  /* Text CRC detects wireless byte loss even when the damaged JSON parses.
   * Reserve its 19 bytes without adding a delay to the USART1 scheduler. */
  if(!FormatOk || Length+31U>=256U)return false;
  budget=256U-Length-31U;if(budget>=sizeof data)budget=sizeof data-1U;
  for(unsigned i=0U;i<records;i++) {
    unsigned length;
    if(!result_record(transfer->kind,result,i,record,sizeof record))return false;
    length=(unsigned)strlen(record);if(length>budget)return false;
    if(used && used+1U+length>budget){page++;used=0U;}
    if(page==transfer->page) {
      if(!used)offset=i;
      if(used)data[selected++]=';';
      memcpy(data+selected,record,length);selected+=length;
    }
    used+=(used?1U:0U)+length;
  }
  *pages=page+1U;if(transfer->page>=*pages)return false;
  data[selected]='\0';begin_transfer(transfer->kind,id,transfer->page,*pages,offset,total);
  append("\"data\":\"%s\"",data);
  if(!FormatOk)return false;
  /* CRC covers the complete compact JSON without its CRC field, including
   * kind/identity/pagination. Close the protected JSON virtually first. */
  {
    uint32_t crc=crc32_update(0xFFFFFFFFUL,(const uint8_t *)Line+7U,Length-7U);
    crc=crc32_update(crc,(const uint8_t *)"}",1U);
    append(",\"crc32\":\"%08lX\"",(unsigned long)~crc);
  }
  if(!finish(false))return false;
  if(transfer->kind==HOST_TRANSFER_PATH && transfer->page+1U==*pages) {
    FrozenResult.delivered=true;
  }
  return true;
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
    if(!freeze_preview(transfer->id,status))return false;
    preview=FrozenPreview.points;total=FrozenPreview.count;bytes_per_point=4U;
    source_id=FrozenPreview.id;id=FrozenPreview.identity;crc=FrozenPreview.crc;
  }
  capacity=packed_capacity(transfer->kind,&id,total,bytes_per_point,source_id);
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
  if(transfer->kind==HOST_TRANSFER_PREVIEW)append("\"live_id\":%lu,",(unsigned long)source_id);
  append("\"crc32\":\"%08lX\",\"data\":\"%s\"",(unsigned long)crc,data);return finish(false);
}

static bool render(const HostTransfer_t *transfer,uint32_t *pages)
{
  RadarHost_Status_t status;RadarHost_StatusGet(&status);
  switch(transfer->kind) {
    case HOST_TRANSFER_CONFIG:return render_config(transfer,&status,pages);
    case HOST_TRANSFER_MAP:case HOST_TRANSFER_PATH:return render_result(transfer,&status,pages);
    case HOST_TRANSFER_PREVIEW:case HOST_TRANSFER_CLOUD:return render_packed(transfer,&status,pages);
    default:return false;
  }
}

static void cancel_transfers(void)
{
  memset(&Requested,0,sizeof Requested);memset(&Manual,0,sizeof Manual);
  ResultStreaming=ResultEndPending=false;
  FastPending=0U;FastEndPending=false;
}

/* Page counts let the receiver repair a lost first/last page without replaying
 * the full pair. Copy the identity: an ACK may arrive before this marker drains. */
static bool result_end(void)
{
  Length=0U;FormatOk=true;
  append("@RADAR {\"v\":1,\"profile\":\"f1_host\",\"k\":\"host_result_end\","
         "\"session\":%lu,\"config\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":%lu,\"mp\":%lu,\"pp\":%lu",
         (unsigned long)StreamIdentity.session,(unsigned long)StreamIdentity.config,
         (unsigned long)StreamIdentity.scan,(unsigned long)StreamIdentity.map,
         (unsigned long)StreamIdentity.plan,(unsigned long)StreamMapPages,(unsigned long)StreamPathPages);
  return finish(false);
}

void RadarHostConsole_Init(void)
{
  RadarHost_Status_t status;
  RadarHost_Init();cancel_transfers();CloudCrcValid=false;
  FrozenPreview.valid=FrozenResult.valid=PreviewDue=false;
  FrozenConfig.valid=false;
  AcknowledgedPlan=0U;
  ResultsSincePreview=0U;
  FastEnabled=false;FastAutoPlan=0U;
  RadarHost_StatusGet(&status);ReportedStatus=status;SeenPreview=status.preview_id;
}

void RadarHostConsole_Process(void)
{
  RadarHost_Status_t status;HostTransfer_t *transfer=NULL;uint32_t pages=0U;
  RadarHost_Process();RadarHost_StatusGet(&status);
  if(!ConsoleTx_BulkReady())return;
  if(FastPending){
    unsigned page=0U;while(!(FastPending&(1UL<<page)))page++;
    if(render_bundle(page)){
      FastPending&=~(1UL<<page);
      if(!FastPending)FrozenResult.delivered=true;
    }
    return;
  }
  if(FastEndPending){if(bundle_end())FastEndPending=false;return;}
  /* Keep the existing occasional complete preview; do not start and then
   * immediately discard it on every automatic result. Explicit scan/result
   * commands can still preempt this optional background transfer. */
  if(FastEnabled && PreviewDue && !Manual.active && status.live_active &&
     status.preview_id && freeze_preview(status.preview_id,&status)){
    PreviewDue=false;SeenPreview=status.preview_id;
    Manual=(HostTransfer_t){HOST_TRANSFER_PREVIEW,status.preview_id,0U,false,true};
  }
  if(FastEnabled && status.scan_state==RADAR_SCAN_READY && !status.active &&
     !(status.live_active && status.live_state==RADAR_LIVE_PREPARING) &&
     status.map_valid && status.plan_id && (FrozenResult.valid?FrozenResult.identity.plan:status.plan_id)!=FastAutoPlan && !Requested.active &&
     !Manual.active) {
    if(freeze_result(&status)){
      FastAutoPlan=FrozenResult.identity.plan;
      FastPending=(1UL<<bundle_pages())-1U;FastEndPending=true;
      if(Manual.kind==HOST_TRANSFER_PREVIEW)memset(&Manual,0,sizeof Manual);
      (void)status_page(false);
    }
    return;
  }
  if(Requested.active)transfer=&Requested;
  else if(Manual.active)transfer=&Manual;
  else if(ResultEndPending){if(result_end())ResultEndPending=false;return;}
  else if(PreviewDue && status.live_active && status.preview_id && freeze_preview(status.preview_id,&status)) {
    PreviewDue=false;SeenPreview=status.preview_id;
    Manual=(HostTransfer_t){HOST_TRANSFER_PREVIEW,status.preview_id,0U,false,true};transfer=&Manual;
  }
  /* Notify once per state/version change; a result request streams map/path
   * together, without a competing automatic config or complete-cloud dump. */
  else if(status_changed(&status)){(void)status_page(false);return;}
  else if(!AcknowledgedPlan && !FrozenResult.valid && status.live_active && status.preview_id!=0U && status.preview_id!=SeenPreview &&
          freeze_preview(status.preview_id,&status)) {
    SeenPreview=status.preview_id;Manual=(HostTransfer_t){HOST_TRANSFER_PREVIEW,status.preview_id,0U,false,true};
    transfer=&Manual;
  } else return;
  if(!render(transfer,&pages)) {
    bool requested=transfer==&Requested;memset(transfer,0,sizeof *transfer);
    if(!requested && ResultStreaming){ResultStreaming=false;reply(false,"snapshot_or_page",false);}
    if(requested)reply(false,"snapshot_or_page",false);
    return;
  }
  if(transfer==&Requested){memset(&Requested,0,sizeof Requested);return;}
  transfer->page++;
  if(transfer->page>=pages) {
    if(ResultStreaming && transfer->kind==HOST_TRANSFER_MAP) {
      StreamMapPages=pages;
      Manual=(HostTransfer_t){HOST_TRANSFER_PATH,StreamIdentity.plan,0U,false,true};
    } else {
      if(ResultStreaming){StreamPathPages=pages;ResultStreaming=false;ResultEndPending=true;}
      memset(transfer,0,sizeof *transfer);
    }
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
  if(count==4U && !strcmp(tokens[2],"status") && !strcmp(tokens[3],"fast")) {
    FastEnabled=true;FastAutoPlan=0U;
    if(!status_page(true))reply(false,"page_size",false);
    return true;
  }
  if(count==5U && !strcmp(tokens[2],"bundle")) {
    uint32_t selected=0U;const char *cursor=tokens[4];
    if(!parse_u32(tokens[3],UINT32_MAX,&value) || !value){reply(false,"plan",false);return true;}
    RadarHost_StatusGet(&status);
    if(!freeze_result(&status) || FrozenResult.identity.plan!=value){reply(false,"snapshot_or_page",false);return true;}
    if(!strcmp(cursor,"all"))selected=(1UL<<bundle_pages())-1U;
    else while(*cursor){
      char *end;unsigned long parsed;
      if(*cursor<'0' || *cursor>'9'){reply(false,"page",false);return true;}
      parsed=strtoul(cursor,&end,10);
      if(parsed>=bundle_pages() || (*end && (*end!=',' || !end[1]))){reply(false,"page",false);return true;}
      selected|=1UL<<parsed;cursor=*end?end+1:end;
    }
    if(!selected){reply(false,"page",false);return true;}
    FastPending|=selected;FastEndPending=true;FastAutoPlan=value;return true;
  }
  if(count==3U && !strcmp(tokens[2],"status")){if(!status_page(true))reply(false,"page_size",false);return true;}
  if(count==4U && !strcmp(tokens[2],"result")) {
    if(!parse_u32(tokens[3],UINT32_MAX,&value) || !value){reply(false,"plan",false);return true;}
    RadarHost_StatusGet(&status);
    if(!freeze_result(&status) || FrozenResult.identity.plan!=value){reply(false,"snapshot_or_page",false);return true;}
    if((ResultStreaming || ResultEndPending) && StreamIdentity.plan==value)return true;
    if(Requested.active || (Manual.active && Manual.kind!=HOST_TRANSFER_PREVIEW)) {
      reply(false,"page_pending",false);return true;
    }
    /* Important results preempt an automatic preview, but never change the map. */
    StreamIdentity=FrozenResult.identity;StreamMapPages=StreamPathPages=0U;
    Manual=(HostTransfer_t){HOST_TRANSFER_MAP,StreamIdentity.map,0U,false,true};
    ResultStreaming=true;ResultEndPending=false;return true;
  }
  if(count==4U && !strcmp(tokens[2],"ack")) {
    if(!parse_u32(tokens[3],UINT32_MAX,&value)){reply(false,"plan",false);return true;}
    ok=value!=0U && (value==AcknowledgedPlan || (FrozenResult.valid && FrozenResult.delivered && value==FrozenResult.identity.plan));
    if(ok && FrozenResult.valid && value==FrozenResult.identity.plan) {
      /* Keep the preview, but not on every result cycle. Map/path have already
       * arrived; every fourth distinct ACK permits one low-priority preview. */
      if(++ResultsSincePreview>=4U){ResultsSincePreview=0U;PreviewDue=true;}
      AcknowledgedPlan=value;FrozenResult.valid=false;FastPending=0U;FastEndPending=false;
    }
    reply(ok,ok?"result_acknowledged":"result_not_delivered",false);return true;
  }
  if(count==3U && !strcmp(tokens[2],"start")){
    ok=RadarHost_StartRotation();reply(ok,ok?"start_accepted":"start_busy_or_uart",false);return true;
  }
  if((count==3U || (count==4U && !strcmp(tokens[3],"fast"))) && !strcmp(tokens[2],"scan")){
    ok=RadarHost_Scan();if(ok){cancel_transfers();FrozenResult.valid=PreviewDue=false;FastEnabled=count==4U;FastAutoPlan=0U;}
    reply(ok,ok?"scan_accepted":"scan_busy_or_uart",false);return true;
  }
  if(count==3U && !strcmp(tokens[2],"stop")){
    RadarHost_Stop();cancel_transfers();reply(true,"stop_requested",true);return true;
  }
  if((count==4U || (count==5U && !strcmp(tokens[4],"fast"))) && !strcmp(tokens[2],"live")) {
    if(!strcmp(tokens[3],"status")){if(!status_page(true))reply(false,"page_size",false);return true;}
    if(!strcmp(tokens[3],"start")){ok=RadarHost_LiveStart();if(ok){cancel_transfers();FrozenResult.valid=PreviewDue=false;FastEnabled=count==5U;FastAutoPlan=0U;}reply(ok,ok?"live_start_accepted":"live_busy_or_uart",false);return true;}
    if(!strcmp(tokens[3],"stop")){RadarHost_LiveStop();cancel_transfers();reply(true,"live_stop_requested",true);return true;}
  }
  if(count==4U && !strcmp(tokens[2],"draft") && !strcmp(tokens[3],"reset")) {
    cancel_transfers();FrozenResult.valid=PreviewDue=false;
    (void)RadarHost_DraftReset();reply(true,"draft_reset",false);return true;
  }
  if(count==5U && !strcmp(tokens[2],"set")) {
    ok=RadarHost_DraftSet(tokens[3],tokens[4]);reply(ok,ok?"draft_updated":"draft_value",false);return true;
  }
  if(count==4U && !strcmp(tokens[2],"apply")) {
    if(!parse_u32(tokens[3],UINT32_MAX,&value)){reply(false,"revision",false);return true;}
    ok=RadarHost_Apply(value);if(ok){cancel_transfers();FrozenResult.valid=PreviewDue=false;}
    reply(ok,ok?"config_applied":"revision_or_config",false);return true;
  }
  if(count==4U && !strcmp(tokens[2],"get") && (!strcmp(tokens[3],"active") || !strcmp(tokens[3],"draft"))) {
    RadarHost_StatusGet(&status);
    if(Manual.active || Requested.active){reply(false,"page_pending",false);return true;}
    FrozenConfig.valid=false;
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
