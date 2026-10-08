/**
 * @file radar_host.c
 * @brief 网页雷达主机服务；配置只在RAM中，规划结果绝不进入车辆运动模块。
 */
#include "radar_host.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

static RadarHostMap_Config_t ActiveConfig,DraftConfig;
static RadarHostMap_Result_t Result;
static RadarLive_Point_t Preview[RADAR_LIVE_BINS];
static RadarHost_Status_t Status;
static uint32_t SessionSeed,ResultScanId,ResultConfigId,ObservedScanId,ObservedLiveSequence;
static bool DraftAvailable;

static uint32_t next_nonzero(uint32_t value)
{
  value++;return value?value:1U;
}

static bool parse_u32(const char *text,uint32_t maximum,uint32_t *value)
{
  char *end;
  unsigned long parsed;
  if(text==NULL || *text=='\0' || value==NULL || *text=='-' || *text=='+')return false;
  parsed=strtoul(text,&end,0);
  if(*end!='\0' || parsed>maximum)return false;
  *value=(uint32_t)parsed;return true;
}

static bool parse_mission(const char *text,uint8_t route[RADAR_HOST_MAX_MISSION_VISITS],uint8_t *count)
{
  const char *cursor=text;
  unsigned used=0U;
  if(text==NULL || *text=='\0' || count==NULL)return false;
  while(*cursor!='\0') {
    char *end;unsigned long task;
    if(used>=RADAR_HOST_MAX_MISSION_VISITS || *cursor<'0' || *cursor>'9')return false;
    task=strtoul(cursor,&end,10);
    if(task<1U || task>5U || (used>0U && route[used-1U]==(uint8_t)task))return false;
    route[used++]=(uint8_t)task;
    if(*end=='\0')break;
    if(*end!='-' || end[1]=='\0')return false;
    cursor=end+1;
  }
  if(used<2U || route[0]!=1U || route[used-1U]!=1U)return false;
  *count=(uint8_t)used;return true;
}

static bool parse_endpoint(const char *text,RadarHostMap_Endpoint_t *endpoint)
{
  char *end;
  unsigned long row,column;
  const char *cursor=text;
  if(text==NULL || endpoint==NULL)return false;
  row=strtoul(cursor,&end,10);if(end==cursor || *end!=',')return false;
  cursor=end+1;column=strtoul(cursor,&end,10);if(end==cursor || *end!=',')return false;
  if(row<1U || row>5U || column<1U || column>5U)return false;
  cursor=end+1;
  if(!strcmp(cursor,"cell"))endpoint->anchor=RADAR_HOST_ANCHOR_CELL;
  else if(!strcmp(cursor,"radar"))endpoint->anchor=RADAR_HOST_ANCHOR_RADAR;
  else if(!strcmp(cursor,"goal"))endpoint->anchor=RADAR_HOST_ANCHOR_GOAL_ZONE_ONE;
  else return false;
  endpoint->row=(uint8_t)row;endpoint->column=(uint8_t)column;return true;
}

static void scan_parameters(RadarMap_Params_t *parameters)
{
  memset(parameters,0,sizeof *parameters);
  parameters->lidar_x_mm=(float)ActiveConfig.radar_x_mm;
  parameters->lidar_y_mm=(float)ActiveConfig.radar_y_mm;
  parameters->zero_deg=90.0f;
  parameters->distance_min_mm=ActiveConfig.distance_min_mm;
  parameters->distance_max_mm=ActiveConfig.distance_max_mm;
  parameters->angle_min_tenths=ActiveConfig.angle_min_tenths;
  parameters->angle_max_tenths=ActiveConfig.angle_max_tenths;
  parameters->threshold=ActiveConfig.threshold;
  parameters->energy_min=ActiveConfig.energy_min;
  parameters->energy_max=ActiveConfig.energy_max;
}

static bool rebuild(uint32_t scan_id,const RadarSample_t *cloud,uint16_t count)
{
  RadarHostMap_Result_t next;
  if(scan_id==0U || cloud==NULL || !RadarHostMap_Build(&ActiveConfig,cloud,count,&next))return false;
  Result=next;ResultScanId=scan_id;ResultConfigId=Status.config_id;
  Status.map_id=next_nonzero(Status.map_id);Status.plan_id=next_nonzero(Status.plan_id);
  Status.filtered_points=Result.filtered_points;return true;
}

void RadarHost_Init(void)
{
  memset(&Status,0,sizeof Status);memset(&Result,0,sizeof Result);memset(Preview,0,sizeof Preview);
  RadarHostMap_Defaults(&ActiveConfig);DraftConfig=ActiveConfig;
  SessionSeed=next_nonzero(SessionSeed);Status.session_id=SessionSeed;Status.config_id=1U;
  Status.scan_state=RADAR_SCAN_IDLE;Status.live_state=RADAR_LIVE_STOPPED;Status.reason="idle";
  ResultScanId=ResultConfigId=ObservedScanId=ObservedLiveSequence=0U;DraftAvailable=false;
}

void RadarHost_Process(void)
{
  RadarScan_HostStatus_t scan;
  RadarScan_HostStatusGet(&scan);
  if(scan.scan_id!=0U && scan.scan_id!=ObservedScanId) {
    uint16_t count=0U;const RadarSample_t *cloud=RadarScan_HostPoints(&count);
    uint32_t live_scan=0U;
    const RadarHostMap_Result_t *live_map=RadarScan_HostLiveMap(&live_scan);
    ObservedScanId=scan.scan_id;
    if(live_map && live_scan==scan.scan_id) {
      Result=*live_map;ResultScanId=live_scan;ResultConfigId=Status.config_id;
      Status.map_id=next_nonzero(Status.map_id);Status.plan_id=next_nonzero(Status.plan_id);
      Status.filtered_points=Result.filtered_points;
    } else if(scan.points_valid && !scan.truncated && cloud!=NULL) (void)rebuild(scan.scan_id,cloud,count);
  }
  if(scan.live_sequence!=0U && scan.live_sequence!=ObservedLiveSequence) {
    RadarLive_Point_t next[RADAR_LIVE_BINS];
    if(RadarScan_HostLiveCopy(next)) {
      uint16_t count=0U;
      for(unsigned i=0U;i<RADAR_LIVE_BINS;i++)
        if(next[i].distance_mm)Preview[count++]=next[i];
      ObservedLiveSequence=scan.live_sequence;
      /* Backend live_sequence restarts on live start; the wire id does not. */
      Status.preview_id=next_nonzero(Status.preview_id);Status.preview_points=count;
    }
  }
}

bool RadarHost_StartRotation(void) {return RadarUart_StartScan();}
void RadarHost_Stop(void) {RadarScan_HostStop();}
bool RadarHost_Scan(void)
{
  RadarScan_HostStatus_t scan;
  RadarMap_Params_t parameters;scan_parameters(&parameters);
  RadarScan_HostStatusGet(&scan);
  /* An explicit single scan replaces host live viewing, never vehicle work. */
  if(scan.live_active)RadarScan_HostLiveStop();
  return RadarScan_HostStart(&parameters);
}
bool RadarHost_LiveStart(void)
{
  RadarMap_Params_t parameters;scan_parameters(&parameters);
  if(!RadarScan_HostLiveStart(&parameters,&ActiveConfig))return false;
  ObservedLiveSequence=0U;return true;
}
void RadarHost_LiveStop(void) {RadarScan_HostLiveStop();}

void RadarHost_StatusGet(RadarHost_Status_t *out)
{
  RadarScan_HostStatus_t scan;
  uint16_t cloud_count=0U;
  if(out==NULL)return;
  RadarScan_HostStatusGet(&scan);
  Status.draft_revision=DraftAvailable?Status.draft_revision:0U;
  Status.scan_id=scan.scan_id;Status.scan_state=scan.state;Status.live_state=scan.live_state;
  Status.reason=scan.live_active?scan.live_reason:scan.reason;
  Status.raw_points=scan.raw_points;Status.circle_restarts=scan.circle_restarts;
  Status.unit_changes=scan.unit_changes;Status.unit_overflows=scan.unit_overflows;
  Status.active=scan.active;Status.live_active=scan.live_active;
  Status.cloud_valid=RadarScan_HostPoints(&cloud_count)!=NULL;
  Status.cloud_points=cloud_count;Status.map_valid=Status.map_id!=0U && ResultConfigId==Status.config_id;
  Status.path_valid=Status.map_valid && Result.path_valid;
  Status.historical=Status.map_id!=0U && (ResultScanId!=scan.scan_id || ResultConfigId!=Status.config_id);
  *out=Status;
}

const RadarHostMap_Config_t *RadarHost_ActiveConfig(void) {return &ActiveConfig;}
const RadarHostMap_Config_t *RadarHost_DraftConfig(void) {return DraftAvailable?&DraftConfig:0;}

uint32_t RadarHost_DraftReset(void)
{
  DraftConfig=ActiveConfig;DraftAvailable=true;
  Status.draft_revision=next_nonzero(Status.draft_revision);return Status.draft_revision;
}

bool RadarHost_DraftSet(const char *key,const char *value)
{
  uint32_t parsed;
  if(!DraftAvailable || key==NULL || value==NULL)return false;
  if(!strcmp(key,"planning_mode")) {
    if(!strcmp(value,"mission"))DraftConfig.planning_mode=RADAR_HOST_PLAN_MISSION;
    else if(!strcmp(value,"point"))DraftConfig.planning_mode=RADAR_HOST_PLAN_POINT;
    else return false;
    return true;
  }
  if(!strcmp(key,"mission"))return parse_mission(value,DraftConfig.mission,&DraftConfig.mission_count);
  if(!strcmp(key,"start"))return parse_endpoint(value,&DraftConfig.start);
  if(!strcmp(key,"goal"))return parse_endpoint(value,&DraftConfig.goal);
  if(!parse_u32(value,0xFFFFFFFFUL,&parsed))return false;
  if(!strcmp(key,"angle_min")){if(parsed>3600U)return false;DraftConfig.angle_min_tenths=(uint16_t)parsed;}
  else if(!strcmp(key,"angle_max")){if(parsed>3600U)return false;DraftConfig.angle_max_tenths=(uint16_t)parsed;}
  else if(!strcmp(key,"distance_min")){if(parsed>65535U)return false;DraftConfig.distance_min_mm=(uint16_t)parsed;}
  else if(!strcmp(key,"distance_max")){if(parsed>65535U)return false;DraftConfig.distance_max_mm=(uint16_t)parsed;}
  else if(!strcmp(key,"energy_min")){if(parsed>255U)return false;DraftConfig.energy_min=(uint8_t)parsed;}
  else if(!strcmp(key,"energy_max")){if(parsed>255U)return false;DraftConfig.energy_max=(uint8_t)parsed;}
  else if(!strcmp(key,"radar_x")){if(parsed>2400U)return false;DraftConfig.radar_x_mm=(uint16_t)parsed;}
  else if(!strcmp(key,"radar_y")){if(parsed>2400U)return false;DraftConfig.radar_y_mm=(uint16_t)parsed;}
  else if(!strcmp(key,"threshold")){if(parsed<1U || parsed>4096U)return false;DraftConfig.threshold=(uint16_t)parsed;}
  else if(!strcmp(key,"manual_mask")){
    if((parsed&~RADAR_MAP_ALL_CELLS)!=0U || (parsed&DraftConfig.protected_mask)!=0U)return false;
    DraftConfig.manual_mask=parsed;
  } else if(strlen(key)==2U && (key[0]=='x' || key[0]=='y') && key[1]>='1' && key[1]<='4') {
    int16_t *lines=key[0]=='x'?DraftConfig.grid.x_mm:DraftConfig.grid.y_mm;
    if(parsed>2250U)return false;
    lines[(unsigned)(key[1]-'0')]=(int16_t)parsed;
  } else return false;
  return true;
}

bool RadarHost_Apply(uint32_t revision)
{
  RadarHostMap_Config_t next;
  RadarScan_HostStatus_t scan;
  uint16_t count=0U;
  const RadarSample_t *cloud;
  if(!DraftAvailable || revision==0U || revision!=Status.draft_revision)return false;
  next=DraftConfig;next.manual_mask&=RADAR_MAP_ALL_CELLS&~next.protected_mask;
  if(!RadarHostMap_ConfigValid(&next))return false;
  ActiveConfig=next;DraftConfig=next;Status.config_id=next_nonzero(Status.config_id);
  DraftAvailable=false;
  RadarScan_HostStatusGet(&scan);cloud=RadarScan_HostPoints(&count);
  if(scan.live_active)RadarScan_HostLiveConfigure(&ActiveConfig);
  else if(scan.points_valid && !scan.truncated && cloud!=NULL)(void)rebuild(scan.scan_id,cloud,count);
  return true;
}

const RadarHostMap_Result_t *RadarHost_Map(uint32_t *map_id,uint32_t *scan_id)
{
  if(map_id)*map_id=Status.map_id;
  if(scan_id)*scan_id=ResultScanId;
  return Status.map_id?&Result:0;
}
const RadarHostMap_Result_t *RadarHost_Path(uint32_t *plan_id,uint32_t *scan_id)
{
  if(plan_id)*plan_id=Status.plan_id;
  if(scan_id)*scan_id=ResultScanId;
  return Status.plan_id?&Result:0;
}
const RadarLive_Point_t *RadarHost_Preview(uint16_t *count,uint32_t *preview_id)
{
  if(count)*count=Status.preview_id?Status.preview_points:0U;
  if(preview_id)*preview_id=Status.preview_id;
  return Status.preview_id?Preview:0;
}
const RadarSample_t *RadarHost_Cloud(uint16_t *count,uint32_t *scan_id)
{
  RadarScan_HostStatus_t scan;const RadarSample_t *cloud=RadarScan_HostPoints(count);
  RadarScan_HostStatusGet(&scan);if(scan_id)*scan_id=cloud?scan.scan_id:0U;return cloud;
}
