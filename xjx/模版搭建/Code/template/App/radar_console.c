#include "radar_console.h"
#include "radar_host_console.h"
#include "radar_scan.h"
#include "chassis_route.h"
#include "chassis_localization.h"
#include "chassis_motion.h"
#include "chassis_config.h"
#include "GrabRoute.h"
#include "GrabTask.h"
#include "mechanical_arm.h"
#include "hwt101_calibration.h"
#include "console_tx.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

/* A requested page has its own <=256-byte low-priority slot. Replies and
 * chassis telemetry keep their priority; ConsoleTx paces the wireless bytes. */
static RadarPlan_t Plan;
static RadarScan_Status_t Scan;
static RadarScan_StartStatus_t Start;
static RadarScan_Raw_t Raw;
static RadarScan_LiveStatus_t Live;
static RadarStableStatus MapStable;
typedef struct {
  RadarMap_t map;
  RadarMap_Params_t parameters;
  RadarPlan_t plan;
  RadarScan_Status_t scan;
  RadarStableStatus stability;
  uint32_t plan_id;
  bool valid,delivered;
} RadarConsole_Result_t;
static RadarConsole_Result_t Frozen;
static uint32_t AcknowledgedPlan;
static RadarLive_Point_t LiveTx[RADAR_LIVE_BINS];
static uint32_t LiveTxId,LiveTxSequence,LiveFrameTick,LiveStateTick,LiveReportedId;
static RadarScan_LiveState_t LiveReportedState;
static unsigned LivePart;
static bool LiveSending;
static ChassisLocalization_Status_t Location;
static ChassisRoute_Status_t Route;
static uint32_t Session,PlanId,PlanMap,LastBulk,LastStatus,LastNav,ParamsRevision;
static uint32_t SeenMap;
static uint32_t StartReportedScan,StartReportedMap;
static bool Watching,Pending,FormatOk;
static uint32_t ProbeReportedId;
static RadarUart_ProbeState_t ProbeReportedState;
static uint32_t RawSession,RawReportedId;
static RadarScan_State_t RawReportedState;
static char Kind[9],Line[257];
static uint32_t RequestedId;
static unsigned RequestedPage,Length;
static bool ResultSending;
static uint32_t ResultOrigin,ResultScan,ResultMap,ResultPlan;
static unsigned ResultStage,ResultPage,ResultMapPages,ResultPathPages,ResultStablePages;
static uint32_t LastResultPage;
/* DL-20 bench: uninterrupted result bursts lose pages. This local, nonblocking
 * page cadence leaves air-link drain time; no USART1 chunk/command wait is added. */
#define RESULT_PAGE_MS 100U
static const char *const ParamNames[]={"lidar_x_mm","lidar_y_mm","zero_deg",
  "distance_min_mm","distance_max_mm","angle_min_tenths","angle_max_tenths",
  "threshold","energy_min","energy_max","situation","grid_version","zero_tenths",
  "m00_q16","m01_q16","m10_q16","m11_q16","tx_q16","ty_q16","upper_lidar_x_mm","upper_lidar_y_mm"};
static const char *const States[]={"idle","preparing","wait_wrap","collecting","ready","stopped","error"};
static void progress_page(bool foreground);

static void refresh(void)
{
  RadarScan_StatusGet(&Scan);RadarScan_StartStatusGet(&Start);
  RadarScan_RawStatusGet(&Raw);
  RadarScan_LiveStatusGet(&Live);
  ChassisLocalization_Get(&Location);ChassisRoute_StatusGet(&Route);
}
static bool applicable(void)
{
  /* Frozen radar projection is in field coordinates. Localization generation
   * tracks live odometry continuity, not changes to the captured field map. */
  return Scan.map_valid && Scan.map_applicable;
}
bool RadarConsole_ScanBusy(void)
{
  RadarScan_Status_t s;RadarScan_LiveStatus_t live;RadarScan_StatusGet(&s);RadarScan_LiveStatusGet(&live);
  return live.active || s.state==RADAR_SCAN_PREPARING || s.state==RADAR_SCAN_WAIT_WRAP || s.state==RADAR_SCAN_COLLECTING;
}
static bool actuators_busy(void)
{
  return ChassisMotion_IsBusy() || ChassisRoute_IsBusy() || ChassisRoute_StationReserved() ||
    GrabRoute_IsBusy() || GrabTask_IsBusy() || Mecanum_IsBusy() || MechanicalArm_IsBusy() ||
    HWT101_Cal_IsBusy();
}
static void reply(bool ok,const char *reason)
{
  char text[110];int n=snprintf(text,sizeof(text),"%s radar %s\r\n",ok?"OK":"ERR",reason);
  if(n>0 && n<(int)sizeof(text)) (void)ConsoleTx_Write((const uint8_t*)text,(uint16_t)n);
}
static void append(const char *format,...)
{
  int n;va_list ap;if(!FormatOk) return;
  va_start(ap,format);n=vsnprintf(Line+Length,sizeof(Line)-Length,format,ap);va_end(ap);
  if(n<0 || (unsigned)n>=sizeof(Line)-Length) {FormatOk=false;return;}Length+=(unsigned)n;
}
static void begin(const char *kind,unsigned page,unsigned pages)
{
  bool result=Frozen.valid && (!strcmp(kind,"map") || !strcmp(kind,"path") || !strcmp(kind,"stable") || !strcmp(kind,"result_end"));
  const RadarScan_Status_t *scan=result?&Frozen.scan:&Scan;
  uint32_t plan=result?Frozen.plan_id:PlanId;
  Length=0;FormatOk=true;
  if(result && Frozen.parameters.grid_version>=4U && (!strcmp(kind,"map") || !strcmp(kind,"path"))){
    append("@RADAR {\"v\":1,\"k\":\"%s\",\"i\":[%lu,%lu,%lu,%lu,%lu],\"page\":%u,\"pages\":%u,",
      kind,(unsigned long)Session,(unsigned long)scan->origin_generation,(unsigned long)scan->scan_id,
      (unsigned long)scan->map_id,(unsigned long)plan,page,pages);return;
  }
  append("@RADAR {\"v\":1,\"k\":\"%s\",\"session\":%lu,\"origin\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":%lu,\"page\":%u,\"pages\":%u,",
    kind,(unsigned long)Session,(unsigned long)scan->origin_generation,(unsigned long)scan->scan_id,
    (unsigned long)scan->map_id,(unsigned long)plan,page,pages);
}
static bool finish(bool foreground)
{
  if(strstr(Line,"\"k\":\"map\"") || strstr(Line,"\"k\":\"path\"")){
    uint32_t crc=UINT32_MAX;
    for(unsigned i=7U;i<=Length;i++){
      uint8_t value=i==Length?(uint8_t)'}':(uint8_t)Line[i];crc^=value;
      for(unsigned b=0U;b<8U;b++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);
    }
    append(",\"crc32\":\"%08lX\"",(unsigned long)~crc);
  }
  append("}\r\n");if(!FormatOk || Length>256U) {reply(false,"page_size");return false;}
  return foreground?ConsoleTx_Write((const uint8_t*)Line,(uint16_t)Length):ConsoleTx_Bulk(Line,(uint16_t)Length);
}
static void live_state_page(void)
{
  Length=0;FormatOk=true;
  append("@RADAR {\"v\":1,\"k\":\"live_state\",\"s\":%lu,\"id\":%lu,\"active\":%s,\"state\":\"%s\",\"reason\":\"%s\",\"q\":%lu,\"pose\":[%.9g,%.9g,%.9g],\"age_ms\":%lu,\"points\":%u",
    (unsigned long)Session,(unsigned long)Live.id,Live.active?"true":"false",RadarScan_LiveStateName(Live.state),
    Live.reason?Live.reason:"not_started",(unsigned long)Live.sequence,(double)Live.parameters.lidar_x_mm,
    (double)Live.parameters.lidar_y_mm,(double)Live.parameters.zero_deg,(unsigned long)Live.age_ms,Live.points);
  (void)finish(true);LiveReportedId=Live.id;LiveReportedState=Live.state;LiveStateTick=HAL_GetTick();
  if(Live.mapping)progress_page(true);
}
static bool live_part(void)
{
  static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint8_t bytes[96];unsigned offset=LivePart*24U;
  for(unsigned i=0;i<24;i++){
    const RadarLive_Point_t *p=&LiveTx[offset+i];bytes[i*4]=(uint8_t)p->angle_tenths;
    bytes[i*4+1]=(uint8_t)(p->angle_tenths>>8);bytes[i*4+2]=(uint8_t)p->distance_mm;
    bytes[i*4+3]=(uint8_t)(p->distance_mm>>8);
  }
  Length=0;FormatOk=true;
  append("@RADAR {\"v\":1,\"k\":\"live\",\"s\":%lu,\"id\":%lu,\"q\":%lu,\"o\":%u,\"n\":120,\"d\":\"",
    (unsigned long)Session,(unsigned long)LiveTxId,(unsigned long)LiveTxSequence,offset);
  for(unsigned i=0;i<sizeof bytes;i+=3){
    uint32_t value=((uint32_t)bytes[i]<<16)|((uint32_t)bytes[i+1]<<8)|bytes[i+2];
    append("%c%c%c%c",alphabet[value>>18],alphabet[(value>>12)&63U],alphabet[(value>>6)&63U],alphabet[value&63U]);
  }
  append("\"");return finish(false);
}
static float param_value(unsigned i)
{
  const RadarMap_Params_t *p=RadarScan_Params();
  switch(i){case 0:return p->lidar_x_mm;case 1:return p->lidar_y_mm;case 2:return p->zero_deg;
  case 3:return p->distance_min_mm;case 4:return p->distance_max_mm;case 5:return p->angle_min_tenths;
   case 6:return p->angle_max_tenths;case 7:return p->threshold;case 8:return p->energy_min;
   case 19:return p->upper_lidar_x_mm;case 20:return p->upper_lidar_y_mm;default:return p->energy_max;}
}
static bool build_plan(void)
{
  RadarStart_Id_t start=(RadarStart_Id_t)Start.result.start_id;
  if(!Scan.map_valid || start==RADAR_START_UNKNOWN) return false;
  PlanMap=Scan.map_id;PlanId++;if(!PlanId) PlanId=1U;
  return RadarPlan_BuildFromStart(RadarScan_Map(),start,&Plan);
}
static bool freeze_result(void)
{
  if(Frozen.valid)return true;
  const RadarMap_t *map=RadarScan_Map();const RadarMap_Params_t *parameters=RadarScan_MapParams();
  if(!Scan.map_valid || !map || !parameters || !PlanId || PlanMap!=Scan.map_id)return false;
  uint16_t count=0U;uint32_t scan_id=0U;const RadarStablePoint *points=RadarScan_VehicleStablePoints(&count,&scan_id);
  if(MapStable.window_circles==RADAR_STABLE_CIRCLES &&
     (!points || scan_id!=Scan.scan_id || !RadarStableTx_Freeze(1U,scan_id,points,count)))return false;
  Frozen.map=*map;Frozen.parameters=*parameters;Frozen.plan=Plan;Frozen.scan=Scan;
  Frozen.stability=MapStable;Frozen.plan_id=PlanId;Frozen.valid=true;Frozen.delivered=false;return true;
}
static void status_page(bool foreground)
{
  if(!foreground && (RadarScan_Params()->grid_version>=4U || Live.mapping)){progress_page(false);return;}
  begin("status",0,1);
  append("\"state\":\"%s\",\"reason\":\"%s\",\"elapsed\":%lu,\"applicable\":%s,\"points_valid\":%s",
    States[Scan.state],Scan.reason?Scan.reason:"none",(unsigned long)Scan.elapsed_ms,
    applicable()?"true":"false",Scan.points_valid?"true":"false");
  if(Scan.circles_target>1U && Length+14U<=253U)
    append(",\"c\":[%u,%u]",Scan.circles_done,Scan.circles_target);
  (void)finish(foreground);
  if(RadarScan_Params()->grid_version>=4U || Live.mapping)progress_page(foreground);
}
static void progress_page(bool foreground)
{
  const RadarStableStatus *s=Live.active&&Live.mapping?&Live.stability:&Scan.stability;
  Length=0U;FormatOk=true;
  append("@RADAR {\"v\":1,\"k\":\"progress\",\"i\":[%lu,%lu,%lu,%lu,%lu],\"live_active\":%s,\"mapping\":%s,\"window\":[%lu,%lu,%u],\"c\":[%u,%u]",
    (unsigned long)Session,(unsigned long)Scan.origin_generation,(unsigned long)Scan.scan_id,
    (unsigned long)Scan.map_id,(unsigned long)PlanId,Live.active?"true":"false",Live.mapping?"true":"false",
    (unsigned long)s->first_circle,(unsigned long)s->last_circle,s->window_circles,Scan.circles_done,Scan.circles_target);
  (void)finish(foreground);
}
static uint32_t stable_crc(const RadarStablePoint *points,uint16_t count)
{
  uint32_t crc=UINT32_MAX;
  for(uint16_t i=0U;i<count;i++){
    const RadarStablePoint *p=&points[i];
    uint8_t bytes[10]={(uint8_t)p->angle_tenths,(uint8_t)(p->angle_tenths>>8),
      (uint8_t)p->distance_mm,(uint8_t)(p->distance_mm>>8),(uint8_t)p->x_mm,(uint8_t)(p->x_mm>>8),
      (uint8_t)p->y_mm,(uint8_t)(p->y_mm>>8),p->votes,p->flags};
    for(unsigned k=0U;k<10U;k++){crc^=bytes[k];for(unsigned b=0U;b<8U;b++)crc=(crc>>1)^((crc&1U)?0xEDB88320UL:0U);}
  }
  return ~crc;
}
static bool stable_page(unsigned *page_count)
{
  static const char alphabet[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  uint16_t count=0U;uint32_t id=0U;unsigned capacity=1U,pages,offset,n;
  const RadarStablePoint *points=RadarStableTx_Points(1U,RequestedId,&count);
  if(!points){
    points=RadarScan_VehicleStablePoints(&count,&id);
    if(!points || id!=RequestedId || !RadarStableTx_Freeze(1U,id,points,count))return false;
    points=RadarStableTx_Points(1U,id,&count);
  }
  if(RequestedId!=(Frozen.valid?Frozen.scan.scan_id:Scan.scan_id))return false;
  for(unsigned take=1U;take<=8U;take++){
    unsigned draft_pages=count?(count+take-1U)/take:1U;
    begin("stable",draft_pages-1U,draft_pages);
    append("\"id\":%lu,\"offset\":%u,\"total\":%u,\"crc32\":\"FFFFFFFF\",\"data\":\"",
      (unsigned long)RequestedId,(draft_pages-1U)*take,count);
    if(FormatOk && Length+((take*10U+2U)/3U)*4U+5U<=256U)capacity=take;else break;
  }
  pages=count?(count+capacity-1U)/capacity:1U;if(RequestedPage>=pages)return false;
  offset=RequestedPage*capacity;n=count-offset;if(n>capacity)n=capacity;
  begin("stable",RequestedPage,pages);
  append("\"id\":%lu,\"offset\":%u,\"total\":%u,\"crc32\":\"%08lX\",\"data\":\"",
    (unsigned long)RequestedId,offset,count,(unsigned long)stable_crc(points,count));
  {
    uint8_t bytes[80];unsigned length=n*10U;
    for(unsigned i=0U;i<n;i++){
      const RadarStablePoint *p=&points[offset+i];
      bytes[i*10U]=(uint8_t)p->angle_tenths;bytes[i*10U+1U]=(uint8_t)(p->angle_tenths>>8);
      bytes[i*10U+2U]=(uint8_t)p->distance_mm;bytes[i*10U+3U]=(uint8_t)(p->distance_mm>>8);
      bytes[i*10U+4U]=(uint8_t)p->x_mm;bytes[i*10U+5U]=(uint8_t)(p->x_mm>>8);
      bytes[i*10U+6U]=(uint8_t)p->y_mm;bytes[i*10U+7U]=(uint8_t)(p->y_mm>>8);
      bytes[i*10U+8U]=p->votes;bytes[i*10U+9U]=p->flags;
    }
    for(unsigned i=0U;i<length;i+=3U){
      unsigned remain=length-i;uint32_t value=(uint32_t)bytes[i]<<16;
      if(remain>1U)value|=(uint32_t)bytes[i+1U]<<8;
      if(remain>2U)value|=bytes[i+2U];
      append("%c%c%c%c",alphabet[value>>18],alphabet[(value>>12)&63U],remain>1U?alphabet[(value>>6)&63U]:'=',remain>2U?alphabet[value&63U]:'=');
    }
  }
  append("\"");if(page_count)*page_count=pages;return finish(false);
}
static void start_pages(void)
{
  begin("start",0,2);
  append("\"id\":%lu,\"auto\":%s,\"selected\":%u,\"reason\":\"%s\"",
    (unsigned long)Start.scan_id,Start.automatic?"true":"false",Start.result.start_id,
    Start.result.reason?Start.result.reason:"not_scanned");
  (void)finish(true);
  begin("start",1,2);
  append("\"id\":%lu,\"qr\":[%u,%u],\"raw\":[%u,%u],\"conflicts\":[%lu,%lu]",
    (unsigned long)Start.scan_id,Start.result.qr_counts[0],Start.result.qr_counts[1],
    Start.result.raw_counts[0],Start.result.raw_counts[1],
    (unsigned long)Start.result.conflict_mask[0],(unsigned long)Start.result.conflict_mask[1]);
  (void)finish(true);StartReportedScan=Start.scan_id;StartReportedMap=Scan.map_id;
}
static void probe_pages(bool all)
{
  RadarUart_Probe_t p;unsigned i;
  static const char *const names[]={"idle","transmitting","waiting","confirmed","unconfirmed","error","cancelled"};
  RadarUart_ProbeGet(&p);
  begin("probe",0,3);
  append("\"id\":%lu,\"state\":\"%s\",\"reason\":\"%s\",\"tx\":%s",
    (unsigned long)p.id,names[p.state],p.reason?p.reason:"not_run",p.tx_complete?"true":"false");
  (void)finish(true);
  if(all){
    begin("probe",1,3);append("\"id\":%lu,\"bytes\":%lu,\"alarm\":%lu,\"last_alarm\":%u,\"elapsed\":%lu",
      (unsigned long)p.id,(unsigned long)p.rx_bytes,(unsigned long)p.alarms,(unsigned)p.last_alarm,(unsigned long)p.elapsed_ms);
    (void)finish(true);
    begin("probe",2,3);append("\"id\":%lu,\"sn\":\"%s\",\"hex\":\"",(unsigned long)p.id,p.sn);
    for(i=0;i<p.raw_count;i++)append("%02X",p.raw[i]);
    append("\"");(void)finish(true);
  }
  ProbeReportedId=p.id;ProbeReportedState=p.state;
}
static void nav_page(bool foreground)
{
  begin("nav",0,1);
  append("\"state\":\"%s\",\"reason\":\"%s\",\"index\":%lu,\"total\":%lu,\"station\":%lu,\"visit\":%lu,\"stopped\":%s",
    Route.planned?Route.state:"idle",Route.planned?Route.reason:"idle",
    (unsigned long)(Route.planned?Route.segment:0U),(unsigned long)(Route.planned?Route.total:0U),
    (unsigned long)Route.station,(unsigned long)Route.visit,Route.stop_confirmed?"true":"false");
  (void)finish(foreground);
}
/* Draft with the largest page/offset digits for this snapshot, then reserve
 * ]}\r\n and a comma with every worst-width item. Identity widths are actual;
 * item values never change the capacity or offsets between requested pages. */
static unsigned page_capacity(unsigned total,unsigned item_bytes)
{
  unsigned reserved=(!strcmp(Kind,"map") || !strcmp(Kind,"path"))?23U:4U;
  unsigned capacity=FormatOk && Length+reserved<256U?(256U-Length-reserved)/item_bytes:0U;
  if(!capacity) capacity=1U;
  if(total && capacity>total) capacity=total;
  return capacity;
}
static unsigned counts_capacity(const RadarMap_t *map)
{
  unsigned available=FormatOk && Length+23U<256U?256U-Length-23U:0U,capacity=1U;
  /* Frozen counts determine wire widths. Use the largest window, so uniform
   * offsets and all re-reads remain stable without budgeting six bytes for 0. */
  for(unsigned n=1;n<=RADAR_MAP_CELL_COUNT;n++){
    bool fits=true;
    for(unsigned from=0;from+n<=RADAR_MAP_CELL_COUNT;from++){
      unsigned bytes=0;
      for(unsigned i=from;i<from+n;i++){
        unsigned value=map->counts[i];bytes+=2U;while(value>=10U){bytes++;value/=10U;}
      }
      if(bytes>available){fits=false;break;}
    }
    if(!fits)break;
    capacity=n;
  }
  return capacity;
}
static void begin_raw(unsigned page,unsigned pages)
{
  Length=0;FormatOk=true;
  append("@RADAR {\"v\":1,\"k\":\"raw\",\"session\":%lu,\"origin\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":0,\"id\":%lu,\"page\":%u,\"pages\":%u,",
    (unsigned long)RawSession,(unsigned long)Raw.origin_generation,(unsigned long)Raw.scan_id,
    (unsigned long)Raw.map_id,(unsigned long)Raw.id,page,pages);
}
static bool raw_page(unsigned page,bool foreground)
{
  unsigned total=(unsigned)Raw.raw.total,capacity,pages,offset,i;uint8_t bytes[128];
  /* Worst page/offset widths are stable for the frozen window. */
  begin_raw(total+5U,total+5U);
  append("\"offset\":%u,\"total\":%u,\"hex\":\"",total,total);
  capacity=page_capacity(total,2U);if(capacity>sizeof bytes)capacity=sizeof bytes;
  pages=5U+(total?(total+capacity-1U)/capacity:0U);
  if(page>=pages)return false;
  begin_raw(page,pages);
  switch(page){
  case 0:
    append("\"state\":\"%s\",\"complete\":%s,\"truncated\":%s,\"total\":%u,\"crc32\":\"%08lX\"",
      RadarScan_StateName(Raw.state),Raw.complete?"true":"false",Raw.raw.truncated?"true":"false",total,
      (unsigned long)Raw.raw.crc32);break;
  case 1:
    append("\"pose\":[%.9g,%.9g,%.9g],\"unit\":\"%s\",\"start\":%u,\"auto\":%s,\"points\":%u",
      (double)Raw.parameters.lidar_x_mm,(double)Raw.parameters.lidar_y_mm,(double)Raw.parameters.zero_deg,
      RadarScan_UnitName(Raw.unit),Raw.start_id,Raw.automatic?"true":"false",Raw.points);break;
  case 2:
    append("\"filter\":[%u,%u,%u,%u,%u,%u,%u],\"point_total\":%lu",
      Raw.parameters.distance_min_mm,Raw.parameters.distance_max_mm,Raw.parameters.angle_min_tenths,
      Raw.parameters.angle_max_tenths,Raw.parameters.threshold,Raw.parameters.energy_min,Raw.parameters.energy_max,
      (unsigned long)Raw.point_total);break;
  case 3:
    append("\"window\":%lu,\"circle\":[%lu,%lu],\"cloud_truncated\":%s",
      (unsigned long)Raw.raw.window_start,(unsigned long)Raw.circle_start,(unsigned long)Raw.circle_end,
      Raw.cloud_truncated?"true":"false");break;
  case 4:
    append("\"received\":%lu,\"losses\":%lu,\"bad\":%lu,\"restarts\":%lu,\"cloud_valid\":%s",
      (unsigned long)Raw.raw.received,(unsigned long)Raw.raw.losses,(unsigned long)Raw.bad,
      (unsigned long)Raw.restarts,Raw.cloud_valid?"true":"false");break;
  default:
    offset=(page-5U)*capacity;
    i=RadarScan_RawRead(offset,bytes,(uint16_t)capacity);if(!i)return false;
    append("\"offset\":%u,\"total\":%u,\"hex\":\"",offset,total);
    for(unsigned at=0;at<i;at++)append("%02X",bytes[at]);
    append("\"");break;
  }
  return finish(foreground);
}
static void begin_cloud(const char *kind,unsigned page,unsigned pages)
{
  if(!Raw.cloud_valid || Raw.scan_id!=Scan.scan_id){begin(kind,page,pages);return;}
  Length=0;FormatOk=true;
  append("@RADAR {\"v\":1,\"k\":\"%s\",\"session\":%lu,\"origin\":%lu,\"scan\":%lu,\"map\":%lu,\"plan\":0,\"page\":%u,\"pages\":%u,",
    kind,(unsigned long)RawSession,(unsigned long)Raw.origin_generation,(unsigned long)Raw.scan_id,
    (unsigned long)Raw.map_id,page,pages);
}
static void cloud_pose(void)
{
  if(Raw.cloud_valid && Raw.scan_id==Scan.scan_id)
    append("\"pose\":[%.9g,%.9g,%.9g],",(double)Raw.parameters.lidar_x_mm,
      (double)Raw.parameters.lidar_y_mm,(double)Raw.parameters.zero_deg);
}
static bool fetch_page(unsigned *page_count)
{
  unsigned i,offset,total,pages,capacity;uint16_t count=0;const RadarSample_t *points;
  if((!strcmp(Kind,"map") || !strcmp(Kind,"path") || !strcmp(Kind,"stable")) && !freeze_result())return false;
  if(!strcmp(Kind,"stable"))return stable_page(page_count);
  if(!strcmp(Kind,"evidence")){
    const RadarScan_Evidence_t *e=RadarScan_Evidence();
    const RadarScan_EvidenceCircle_t *c;
    unsigned circle=RequestedPage/51U,part=RequestedPage%51U;
    if(!e || RequestedId!=e->scan_id || circle>=e->circles)return false;
    c=&e->circle[circle];Length=0;FormatOk=true;
    append("@RADAR {\"v\":1,\"k\":\"evidence\",\"scan\":%lu,\"map\":%lu,\"page\":%u,\"pages\":%u,\"circle\":%u,",
      (unsigned long)e->scan_id,(unsigned long)e->map_id,RequestedPage,e->circles*51U,circle);
    if(!part)append("\"mask\":%lu,\"pose\":[%.1f,%.1f,%.1f],\"rx\":[%u,%lu,%lu,%lu,%lu],\"raw\":%lu",
      (unsigned long)c->mask,(double)e->parameters.lidar_x_mm,(double)e->parameters.lidar_y_mm,
      (double)e->parameters.zero_deg,c->unit,(unsigned long)c->bad,(unsigned long)c->restarts,
      (unsigned long)c->unit_changes,(unsigned long)c->unit_overflows,(unsigned long)c->raw_points);
    else{
      unsigned cell=(part-1U)/2U,first=((part-1U)%2U)*2U;
      append("\"cell\":%u,\"count\":%u,\"n\":%u,\"offset\":%u,\"data\":\"",cell,c->counts[cell],c->hit_count[cell],first);
      for(i=first;i<first+2U && i<c->hit_count[cell];i++){
        const RadarScan_Hit_t *h=&c->hits[cell][i];
        append("%s%u,%u,%u,%u,%u,%u,%u,%u,%u",i==first?"":";",h->angle_tenths,h->distance_mm,
          h->energy,h->raw_distance,h->packet_start,h->packet_span,h->packet_count,h->packet_index,h->flags);
      }
      append("\"");
    }
    if(page_count)*page_count=e->circles*51U;
    return finish(false);
  }
  if(!strcmp(Kind,"raw")){
    if(!Raw.id || RequestedId!=Raw.id || Raw.raw.active)return false;
    return raw_page(RequestedPage,false);
  }
  if(!strcmp(Kind,"map"))
  {
    const RadarMap_t *m=&Frozen.map;const RadarMap_Params_t *p=&Frozen.parameters;
    const RadarScan_Status_t *scan=&Frozen.scan;const RadarStableStatus *stable=&Frozen.stability;
    if(RequestedId!=scan->map_id) return false;
    begin("map",RADAR_MAP_CELL_COUNT,RADAR_MAP_CELL_COUNT+1U);
    append("\"a\":3,\"c\":%u,\"f\":[%u,%u,%u,%u,%u,%u,%u],\"offset\":%u,\"counts\":[",
      scan->map_circles?scan->map_circles:1U,
      p->distance_min_mm,p->distance_max_mm,p->angle_min_tenths,p->angle_max_tenths,
      p->threshold,p->energy_min,p->energy_max,RADAR_MAP_CELL_COUNT);
    capacity=counts_capacity(m);
    unsigned metadata=p->grid_version>=4U?5U:1U;
    pages=metadata+(RADAR_MAP_CELL_COUNT+capacity-1U)/capacity;
    if(RequestedPage>=pages) return false;
    begin("map",RequestedPage,pages);
    if(!RequestedPage) append("\"g\":%u,\"mask\":%lu,\"pose\":[%.1f,%.1f,%.1f],\"coverage\":%u,\"age_ms\":%lu,\"applicable\":%s",
      p->grid_version?p->grid_version:3U,(unsigned long)m->mask,(double)p->lidar_x_mm,(double)p->lidar_y_mm,(double)p->zero_deg,
      3600U,(unsigned long)(HAL_GetTick()-scan->completed_tick),
      (scan->map_valid && scan->map_applicable)?"true":"false");
    else if(metadata>1U && RequestedPage==1U){
      append("\"geometry\":[%ld,%ld,%ld,%ld,%ld,%ld,%u],\"situation\":%u",
        (long)p->geometry.matrix_q16[0],(long)p->geometry.matrix_q16[1],(long)p->geometry.matrix_q16[2],(long)p->geometry.matrix_q16[3],
        (long)p->geometry.origin_q16[0],(long)p->geometry.origin_q16[1],p->geometry.zero_tenths,p->situation);
    }else if(metadata>1U && RequestedPage==2U){
      append("\"algorithm\":%u,\"start_id\":%u,\"window\":[%lu,%lu,%u],\"stable\":[%u,%u,%lu,%lu]",
        stable->window_circles==RADAR_STABLE_CIRCLES?1U:0U,p->start_id,
        (unsigned long)stable->first_circle,(unsigned long)stable->last_circle,stable->window_circles,
        stable->stable_count,stable->candidate_count,(unsigned long)stable->candidate_replacements,(unsigned long)stable->candidate_dropped);
    }else if(metadata>1U && RequestedPage==3U){
      append("\"stats\":[%lu,%lu,%lu]",(unsigned long)stable->raw_points,(unsigned long)stable->filtered_points,(unsigned long)stable->outside_points);
    }else if(metadata>1U && RequestedPage==4U){
      RadarGrid_t grid;RadarGrid_Default(&grid);append("\"grid\":[");
      for(i=0U;i<6U;i++)append("%s%d",i?",":"",grid.x_mm[i]);
      for(i=0U;i<6U;i++)append(",%d",grid.y_mm[i]);
      append("]");
    }else {offset=(RequestedPage-metadata)*capacity;
      if(RequestedPage==metadata)append("\"a\":%u,\"c\":%u,\"f\":[%u,%u,%u,%u,%u,%u,%u],",metadata>1U?4U:3U,
        scan->map_circles?scan->map_circles:1U,
        p->distance_min_mm,p->distance_max_mm,p->angle_min_tenths,p->angle_max_tenths,
        p->threshold,p->energy_min,p->energy_max);
      append("\"offset\":%u,\"counts\":[",offset);
      for(i=offset;i<RADAR_MAP_CELL_COUNT && i<offset+capacity;i++) append("%s%u",i==offset?"":",",m->counts[i]);
      append("]");}
  }
  else if(!strcmp(Kind,"path"))
  {
    const RadarPlan_t *plan=&Frozen.plan;
    unsigned first_capacity,later_capacity,item_bytes=1U;char item[32];
    if(!Frozen.plan_id || RequestedId!=Frozen.plan_id) return false;
    for(i=0;i<plan->count;i++){
      unsigned bytes=(unsigned)snprintf(item,sizeof(item),"[%d,%d,%u,%u],",plan->points[i].x_mm,
        plan->points[i].y_mm,plan->points[i].station,plan->points[i].visit);
      if(bytes>item_bytes)item_bytes=bytes;
    }
    begin("path",plan->count,plan->count);
    append("\"offset\":%u,\"start\":%u,\"valid\":%s,\"failed_leg\":%u,\"points\":[",plan->count,plan->start_id,plan->valid?"true":"false",plan->failed_leg);
    first_capacity=page_capacity(plan->count,item_bytes);
    begin("path",plan->count,plan->count);append("\"offset\":%u,\"points\":[",plan->count);
    later_capacity=page_capacity(plan->count,item_bytes);
    pages=1U+(plan->count>first_capacity?(plan->count-first_capacity+later_capacity-1U)/later_capacity:0U);
    if(RequestedPage>=pages)return false;
    offset=RequestedPage?first_capacity+(RequestedPage-1U)*later_capacity:0U;
    capacity=RequestedPage?later_capacity:first_capacity;
    begin("path",RequestedPage,pages);append("\"offset\":%u,",offset);
    if(!RequestedPage)append("\"start\":%u,\"valid\":%s,\"failed_leg\":%u,",plan->start_id,plan->valid?"true":"false",plan->failed_leg);
    append("\"points\":[");
    for(i=offset;i<plan->count && i<offset+capacity;i++) append("%s[%d,%d,%u,%u]",i==offset?"":",",
      plan->points[i].x_mm,plan->points[i].y_mm,plan->points[i].station,plan->points[i].visit);
    append("]");
  }
  else if(!strcmp(Kind,"points") || !strcmp(Kind,"cloud"))
  {
    points=RadarScan_Points(&count);
    if(!points || !Scan.points_valid || (!Scan.points_map_id && !Raw.cloud_valid) || RequestedId!=Scan.scan_id) return false;
    total=!strcmp(Kind,"points") && count>360U?360U:count;
    begin_cloud(Kind,total,total);
    cloud_pose(); /* Reserve the first page's frozen pose in every capacity draft. */
    append("\"offset\":%u,\"total\":%u,\"truncated\":%s,\"points\":[",total,total,Scan.truncated?"true":"false");
    capacity=page_capacity(total,18U);
    pages=total?(total+capacity-1U)/capacity:1U;if(RequestedPage>=pages) return false;
    begin_cloud(Kind,RequestedPage,pages);offset=RequestedPage*capacity;
    if(!RequestedPage)cloud_pose();
    append("\"offset\":%u,\"total\":%u,\"truncated\":%s,\"points\":[",offset,total,Scan.truncated?"true":"false");
    for(i=offset;i<total && i<offset+capacity;i++) {
      unsigned at=total?(uint32_t)i*count/total:0U;
      append("%s[%u,%u,%u]",i==offset?"":",",points[at].angle_tenths,points[at].distance_mm,points[at].energy);
    }append("]");
  }
  else if(!strcmp(Kind,"params"))
  {
    const RadarMap_Params_t *p=RadarScan_Params();pages=p->grid_version>=4U?21U:10U;
    if((RequestedId && RequestedId!=ParamsRevision) || RequestedPage>=pages) return false;
    begin("params",RequestedPage,pages);append("\"rev\":%lu,\"key\":\"%s\",\"value\":",
      (unsigned long)ParamsRevision,ParamNames[RequestedPage]);
    if(RequestedPage<10U || RequestedPage>=19U)append("%.1f",(double)param_value(RequestedPage));
    else{
      int32_t integer_value=RequestedPage==10U?p->situation:RequestedPage==11U?p->grid_version:
        RequestedPage==12U?p->geometry.zero_tenths:RequestedPage<17U?p->geometry.matrix_q16[RequestedPage-13U]:p->geometry.origin_q16[RequestedPage-17U];
      append("%ld",(long)integer_value);
    }
  }
  else return false;
  if(page_count)*page_count=pages;
  return finish(false);
}
/* Send the existing pages consecutively, without a radio round trip per page.
 * Static results are unchanged until replaced; identity changes end this stream
 * rather than joining pages from two captures. Missing pages remain fetchable. */
static void result_page(void)
{
  unsigned pages;
  if(!Frozen.valid || Frozen.scan.origin_generation!=ResultOrigin || Frozen.scan.scan_id!=ResultScan ||
     Frozen.scan.map_id!=ResultMap || Frozen.plan_id!=ResultPlan){
    ResultSending=false;reply(false,"snapshot_changed_or_page_range");return;
  }
  if(ResultStage<2U || (ResultStage==2U && Frozen.stability.window_circles==RADAR_STABLE_CIRCLES)){
    strcpy(Kind,ResultStage==0U?"map":ResultStage==1U?"path":"stable");
    RequestedId=ResultStage==0U?ResultMap:ResultStage==1U?ResultPlan:ResultScan;
    RequestedPage=ResultPage;
    if(!fetch_page(&pages)){ResultSending=false;reply(false,"snapshot_changed_or_page_range");return;}
    if(ResultStage==2U)ResultStablePages=pages;else if(ResultStage)ResultPathPages=pages;else ResultMapPages=pages;
    if(++ResultPage==pages){ResultStage++;ResultPage=0;}
  }else{
    begin("result_end",0,1);append("\"mp\":%u,\"pp\":%u,\"sp\":%u",ResultMapPages,ResultPathPages,ResultStablePages);
    if(finish(false)){ResultSending=false;Frozen.delivered=true;}
  }
}
void RadarConsole_Init(void)
{
  memset(&Plan,0,sizeof(Plan));Session=HAL_GetTick()+1U;if(!Session) Session=1;
  PlanId=PlanMap=SeenMap=LastBulk=LastStatus=LastNav=0;ParamsRevision=1;
  Watching=Pending=ResultSending=false;Kind[0]=0;
  Frozen.valid=false;AcknowledgedPlan=0U;
  StartReportedScan=StartReportedMap=0;
  ProbeReportedId=0;ProbeReportedState=RADAR_PROBE_IDLE;
  RawSession=Session;RawReportedId=0;RawReportedState=RADAR_SCAN_IDLE;
  LiveTxId=LiveTxSequence=LiveReportedId=0;LivePart=0;LiveSending=false;
  LiveFrameTick=HAL_GetTick()-1000U;LiveStateTick=0;LiveReportedState=RADAR_LIVE_STOPPED;
  RadarHostConsole_Init();
  RadarStableTx_Release(1U);memset(&MapStable,0,sizeof MapStable);
}
void RadarConsole_Process(void)
{
  RadarHostConsole_Process();
  uint32_t now=HAL_GetTick();refresh();
  if(RadarConsole_ScanBusy() && (ChassisMotion_IsBusy() || Mecanum_IsBusy())){
    RadarScan_Stop();refresh();
  }
  if(Scan.map_valid && Scan.map_id!=SeenMap){SeenMap=Scan.map_id;MapStable=Scan.stability;(void)build_plan();}
  {
    RadarUart_Probe_t p;RadarUart_ProbeGet(&p);
    if(p.id && (p.id!=ProbeReportedId || p.state!=ProbeReportedState))
      probe_pages(p.state!=RADAR_PROBE_TX && p.state!=RADAR_PROBE_WAITING);
  }
  if(Start.scan_id && (Start.scan_id!=StartReportedScan || Scan.map_id!=StartReportedMap)){
    start_pages();status_page(true);LastStatus=LastBulk=now;
  }
  if(Raw.id && (Raw.id!=RawReportedId || Raw.state!=RawReportedState)){
    (void)raw_page(0,true);RawReportedId=Raw.id;RawReportedState=Raw.state;
  }
  if(!Live.active)LiveSending=false;
  if(Live.id && (Live.id!=LiveReportedId || Live.state!=LiveReportedState ||
      (Live.active && (uint32_t)(now-LiveStateTick)>=2000U)))live_state_page();
  if(!Watching || !ConsoleTx_BulkReady()) return;
  if(Pending){if(!fetch_page(NULL)) reply(false,"snapshot_changed_or_page_range");Pending=false;LastBulk=now;return;}
  if(ResultSending){
    if((uint32_t)(now-LastResultPage)>=RESULT_PAGE_MS){result_page();LastResultPage=LastBulk=now;}
    return;
  }
  if(Live.active){
    if(!LiveSending && Live.sequence && (Live.id!=LiveTxId || Live.sequence!=LiveTxSequence) &&
       (uint32_t)(now-LiveFrameTick)>=1000U && RadarScan_LiveCopy(LiveTx)){
      LiveTxId=Live.id;LiveTxSequence=Live.sequence;LivePart=0;LiveSending=true;LiveFrameTick=now;
    }
    if(LiveSending){if(live_part()){LivePart++;if(LivePart==5U)LiveSending=false;}return;}
  }
  if((uint32_t)(now-LastBulk)<1000U) return;
  if(Route.planned && (uint32_t)(now-LastNav)>=1500U){nav_page(false);LastNav=now;}
  else if((uint32_t)(now-LastStatus)>=2000U){status_page(false);LastStatus=now;}
  else return;
  LastBulk=now;
}
static bool number(const char *s,float *value)
{
  char *end;errno=0;*value=strtof(s,&end);return end!=s && !*end && !errno && isfinite(*value);
}
static bool integer(const char *s,uint32_t *value)
{
  char *end;unsigned long n;errno=0;if(!*s || *s=='-') return false;
  n=strtoul(s,&end,10);if(end==s || *end || errno || n>0xFFFFFFFFUL) return false;*value=(uint32_t)n;return true;
}
bool RadarConsole_Command(unsigned n,char *t[])
{
  float value;uint32_t id,page;bool ok=false;const char *reason="format";
  if(n<2U || strcmp(t[0],"radar")) return false;
  if(!strcmp(t[1],"host"))return RadarHostConsole_Command(n,t);
  refresh();Watching=true;
  if(n==3U && !strcmp(t[1],"live") && !strcmp(t[2],"status")){live_state_page();return true;}
  if(n==3U && !strcmp(t[1],"live") && !strcmp(t[2],"stop")){
    RadarScan_LiveStop();LiveSending=false;reply(true,"live_stop_requested");refresh();live_state_page();return true;
  }
  if(((n==4U && (!strcmp(t[2],"start") || !strcmp(t[2],"preview"))) ||
      (n==5U && !strcmp(t[2],"map") && !strcmp(t[3],"start"))) && !strcmp(t[1],"live") &&
     (!strcmp(t[n-1U],"upper") || !strcmp(t[n-1U],"lower"))){
    RadarUart_Probe_t p;RadarUart_ProbeGet(&p);
    if(p.state==RADAR_PROBE_TX || p.state==RADAR_PROBE_WAITING)reason="query_in_progress";
    else if(actuators_busy())reason="stationary_capture_required";
    else {RadarStart_Id_t selected=!strcmp(t[n-1U],"upper")?RADAR_START_UPPER:RADAR_START_LOWER;
      ok=n==5U?RadarScan_LiveMapStart(RadarScan_Params(),Location.generation,selected):
        RadarScan_LiveStart(RadarScan_Params(),Location.generation,selected);
      reason=ok?"live_start_accepted":"scan_busy_or_parameters";}
    reply(ok,reason);
    if(ok){LiveSending=false;LiveTxSequence=0;LiveFrameTick=HAL_GetTick()-1000U;
      RadarStableTx_Release(2U);refresh();live_state_page();}
    return true;
  }
  if(n==2U && !strcmp(t[1],"status")){
    RadarUart_Status_t uart;RadarUart_StatusGet(&uart);
    RadarScan_Unit_t unit=Live.active?Live.unit:Scan.unit;
    /* Split diagnostics so full uint32 counters and identities fit 256 bytes. */
    status_page(true);begin("diag",0,3);
    append("\"bytes\":%lu,\"packets\":%lu,\"bad\":%lu,\"overflow\":%lu",
      (unsigned long)Scan.bytes,(unsigned long)Scan.packets,(unsigned long)Scan.bad,(unsigned long)Scan.overflow);
    (void)finish(true);begin("diag",1,3);
    append("\"alarm\":%lu,\"last_alarm\":%u,\"unit\":\"%s\"",
      (unsigned long)Scan.alarm,(unsigned)Scan.last_alarm,
      unit==RADAR_UNIT_CM?"cm":unit==RADAR_UNIT_MM?"mm":"unknown");
    (void)finish(true);begin("diag",2,3);
    append("\"tx_done\":%lu,\"tx_errors\":%lu,\"uart_errors\":%lu,\"rx_ready\":%s,\"last_tx\":\"%s\"",
      (unsigned long)uart.commands_sent,(unsigned long)uart.tx_errors,(unsigned long)uart.uart_errors,
      uart.rx_ready?"true":"false",uart.last_tx);
    (void)finish(true);if(Start.scan_id)start_pages();
    return true;
  }
  if(n==3U && !strcmp(t[1],"start")){
    if(!strcmp(t[2],"status")){start_pages();return true;}
    if(!strcmp(t[2],"upper") || !strcmp(t[2],"lower")){
      ok=RadarScan_SelectStart(!strcmp(t[2],"upper")?RADAR_START_UPPER:RADAR_START_LOWER);
      reply(ok,ok?"start_selected":"start_selection_unavailable");
      if(ok){refresh();(void)build_plan();SeenMap=Scan.map_id;start_pages();status_page(true);LastStatus=LastBulk=HAL_GetTick();}
      return true;
    }
  }
  if(n==3U && !strcmp(t[1],"probe") && !strcmp(t[2],"status")){probe_pages(true);return true;}
  if(n==3U && !strcmp(t[1],"debug") && !strcmp(t[2],"status")){
    (void)raw_page(0,true);return true;
  }
  if(n==4U && !strcmp(t[1],"debug") && !strcmp(t[2],"scan") &&
     (!strcmp(t[3],"upper") || !strcmp(t[3],"lower") || !strcmp(t[3],"auto"))){
    RadarStart_Id_t start=!strcmp(t[3],"auto")?RADAR_START_UNKNOWN:
      !strcmp(t[3],"upper")?RADAR_START_UPPER:RADAR_START_LOWER;
    RadarUart_Probe_t p;RadarUart_ProbeGet(&p);
    if(Live.active)reason="live_in_progress";
    else if(p.state==RADAR_PROBE_TX || p.state==RADAR_PROBE_WAITING)reason="query_in_progress";
    else if(actuators_busy())reason="stationary_capture_required";
    else {
      ok=RadarScan_DebugStart(RadarScan_Params(),Location.generation,start);
      reason=ok?"debug_scan_accepted":"scan_busy_or_parameters";
    }
    reply(ok,reason);
    if(ok){
      ResultSending=false;RawSession=Session;refresh();status_page(true);
      (void)raw_page(0,true);RawReportedId=Raw.id;RawReportedState=Raw.state;LastStatus=HAL_GetTick();
    }
    return true;
  }
  if(n==2U && !strcmp(t[1],"probe")){
    if(RadarConsole_ScanBusy()) reply(false,"scan_in_progress");
    else if(!RadarUart_ProbeStart()) reply(false,"probe_busy_or_uart");
    else {reply(true,"probe_accepted");probe_pages(false);}
    return true;
  }
  if(n==2U && !strcmp(t[1],"stop")){
    bool was_live=Live.active;RadarScan_Stop();reply(true,"scan_stop_requested");
    if(was_live){LiveSending=false;refresh();live_state_page();}return true;
  }
  if((n==2U || (n==3U && (!strcmp(t[2],"auto") || !strcmp(t[2],"upper") ||
                          !strcmp(t[2],"lower"))) ||
      (n==4U && (!strcmp(t[2],"upper") || !strcmp(t[2],"lower")) &&
       (!strcmp(t[3],"1") || !strcmp(t[3],"3") || !strcmp(t[3],"15")))) && !strcmp(t[1],"scan")){
    bool automatic=n==3U && !strcmp(t[2],"auto");
    RadarStart_Id_t start=n>=3U && !strcmp(t[2],"upper")?RADAR_START_UPPER:RADAR_START_LOWER;
    if(Live.active){reply(false,"live_in_progress");return true;}
    RadarUart_Probe_t p;RadarUart_ProbeGet(&p);
    if(p.state==RADAR_PROBE_TX || p.state==RADAR_PROBE_WAITING){reply(false,"query_in_progress");return true;}
    if(actuators_busy()) reason="stationary_capture_required";
    else{
      RadarMap_Params_t p=*RadarScan_Params();
      /* Registered reference pose is expressed at the validated chassis origin.
       * Only the sensor pose follows actual feedback; IMU/motor axes stay intact. */
      if(n==2U && Location.origin_valid && Location.feedback_valid){
        float yaw=Location.observer.feedback.yaw_rad-CHASSIS_MODEL_PI/2.0f;
        float dx=p.lidar_x_mm-2250.0f,dy=p.lidar_y_mm-150.0f;
        p.lidar_x_mm=Location.observer.feedback.x_mm+cosf(yaw)*dx-sinf(yaw)*dy;
        p.lidar_y_mm=Location.observer.feedback.y_mm+sinf(yaw)*dx+cosf(yaw)*dy;
        p.zero_deg+=yaw*180.0f/CHASSIS_MODEL_PI;
      }
      if(automatic)ok=RadarScan_StartAuto(&p,Location.generation);
      else if(n==4U)ok=RadarScan_StartAtCycles(&p,Location.generation,start,15U);
      else if(n==3U)ok=RadarScan_StartAt(&p,Location.generation,start);
      else ok=RadarScan_Start(&p,Location.generation);
      reason=ok?"scan_accepted":"scan_busy_or_parameters";
    }
    if(ok){
      /* Establish the capture baseline even if its circle completes before the
       * periodic bulk status slot becomes available. This also keeps queued
       * historical ready frames from masquerading as the requested scan. */
      ResultSending=false;RadarStableTx_Release(2U);reply(true,reason);refresh();status_page(true);LastStatus=HAL_GetTick();return true;
    }
  }
  else if(n==2U && !strcmp(t[1],"plan")){
    if(Start.automatic && Start.result.start_id==RADAR_START_UNKNOWN)reason="start_unresolved";
    else {ok=build_plan();reason=ok?"plan_ready":Scan.map_valid?"leg_unreachable":"no_complete_map";}
  }
  else if(n==2U && !strcmp(t[1],"get")){
    if(Pending) reason="page_pending";
    else {strcpy(Kind,"params");RequestedId=0;RequestedPage=0;Pending=true;ok=true;reason="fetch_params";}
  }
  else if(n==4U && !strcmp(t[1],"set") && number(t[3],&value)){
    if(strstr(t[2],"_q16") || !strcmp(t[2],"zero_tenths")){
      char *end;errno=0;long integer_value=strtol(t[3],&end,10);
      ok=end!=t[3] && !*end && !errno && integer_value>=INT32_MIN && integer_value<=INT32_MAX &&
        RadarScan_SetCalibrationParam(t[2],(int32_t)integer_value);
    }else ok=RadarScan_SetParam(t[2],value);
    if(ok) ParamsRevision++;
    reason=ok?"parameter_updated_ram":"parameter_range_or_key";
  }
  else if(n==3U && !strcmp(t[1],"result") && integer(t[2],&id)){
    if(Frozen.valid && id!=Frozen.plan_id && id==PlanId){Frozen.valid=false;RadarStableTx_Release(1U);}
    if(!id || (!Frozen.valid && id!=PlanId) || !freeze_result() || id!=Frozen.plan_id)reason="snapshot_changed_or_page_range";
    else{
      if(!ResultSending || ResultPlan!=id){
        ResultOrigin=Frozen.scan.origin_generation;ResultScan=Frozen.scan.scan_id;ResultMap=Frozen.scan.map_id;ResultPlan=id;
        ResultStage=ResultPage=ResultMapPages=ResultPathPages=ResultStablePages=0;LastResultPage=HAL_GetTick()-RESULT_PAGE_MS;ResultSending=true;
      }
      return true;
    }
  }
  else if(n==3U && !strcmp(t[1],"ack") && integer(t[2],&id)){
    ok=id && (id==AcknowledgedPlan || (Frozen.valid && id==Frozen.plan_id));
    if(ok && Frozen.valid && id==Frozen.plan_id){AcknowledgedPlan=id;Frozen.valid=ResultSending=false;RadarStableTx_Release(1U);}
    reason=ok?"result_acknowledged":"result_not_available";
  }
  else if(n==5U && !strcmp(t[1],"fetch") && integer(t[3],&id) && integer(t[4],&page)){
    if(strcmp(t[2],"map") && strcmp(t[2],"path") && strcmp(t[2],"points") && strcmp(t[2],"cloud") && strcmp(t[2],"params") && strcmp(t[2],"raw") && strcmp(t[2],"evidence") && strcmp(t[2],"stable")) reason="fetch_kind";
    else if(Pending) reason="page_pending";
    else if(page>65535U) reason="page_range";
    else {
      uint32_t current=!strcmp(t[2],"path")?PlanId:!strcmp(t[2],"map")?Scan.map_id:Scan.scan_id;
      uint32_t frozen=!strcmp(t[2],"path")?Frozen.plan_id:!strcmp(t[2],"map")?Frozen.scan.map_id:Frozen.scan.scan_id;
      if(Frozen.valid && (!strcmp(t[2],"map") || !strcmp(t[2],"path") || !strcmp(t[2],"stable")) && id!=frozen && id==current){
        Frozen.valid=ResultSending=false;RadarStableTx_Release(1U);
      }
      strcpy(Kind,t[2]);RequestedId=id;RequestedPage=(unsigned)page;Pending=true;return true;
    }
  }
  else if(n>=3U && (!strcmp(t[1],"nav") || !strcmp(t[1],"task"))){
    if(n==3U && !strcmp(t[1],"nav") && !strcmp(t[2],"status")){nav_page(true);return true;}
    if(n==3U && !strcmp(t[1],"nav") && !strcmp(t[2],"cancel")){
      if(Route.planned){GrabRoute_Stop();ok=ChassisRoute_Cancel();}else ok=true;
      reason="navigation_stop_requested";
    }
    else if((n==3U || n==4U) && !strcmp(t[2],"start") && (n==3U || number(t[3],&value))){
      if(n==3U) value=CHASSIS_ROUTE_DEFAULT_SPEED_MM_S;
      if(RadarConsole_ScanBusy()) reason="scan_in_progress";
      else if(Start.automatic && Start.result.start_id==RADAR_START_UNKNOWN)reason="start_unresolved";
      else if(Start.result.start_id==RADAR_START_UPPER)reason="upper_start_plan_only";
      else if(!applicable() || PlanMap!=Scan.map_id) reason="map_origin_or_parameters_changed";
      else if(!Plan.valid) reason="leg_unreachable";
      else if(actuators_busy()) reason="actuator_busy";
      else {
        ok=!strcmp(t[1],"task")?GrabRoute_StartPlanned(&Plan,value):ChassisRoute_PlanStart(&Plan,value,false);
        reason=ok?"navigation_accepted":"navigation_or_grab_not_ready";
      }
    }
  }
  reply(ok,reason);return true;
}
