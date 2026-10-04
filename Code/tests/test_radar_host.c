#include "radar_host.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static RadarScan_HostStatus_t fake_status;
static RadarSample_t fake_cloud[8];
static RadarLive_Point_t fake_preview[RADAR_LIVE_BINS];
static uint16_t fake_cloud_count;
static unsigned rotate_calls,scan_calls,stop_calls,live_start_calls,live_stop_calls;
static RadarMap_Params_t last_scan_parameters,last_live_parameters;

bool RadarUart_StartScan(void){rotate_calls++;return true;}
bool RadarScan_HostStart(const RadarMap_Params_t *params)
{
  scan_calls++;last_scan_parameters=*params;fake_status.parameters=*params;
  fake_status.active=true;fake_status.state=RADAR_SCAN_PREPARING;fake_status.reason="configuring";
  fake_status.points_valid=false;return true;
}
void RadarScan_HostStop(void)
{stop_calls++;fake_status.active=false;fake_status.state=RADAR_SCAN_STOPPED;fake_status.reason="operator_stop";}
void RadarScan_HostStatusGet(RadarScan_HostStatus_t *status){*status=fake_status;}
const RadarSample_t *RadarScan_HostPoints(uint16_t *count)
{
  if(count)*count=fake_status.points_valid?fake_cloud_count:0U;
  return fake_status.points_valid?fake_cloud:0;
}
bool RadarScan_HostLiveStart(const RadarMap_Params_t *params)
{
  live_start_calls++;last_live_parameters=*params;fake_status.parameters=*params;
  fake_status.live_active=true;fake_status.live_state=RADAR_LIVE_PREPARING;
  fake_status.live_reason="configuring";return true;
}
void RadarScan_HostLiveStop(void)
{live_stop_calls++;fake_status.live_active=false;fake_status.live_state=RADAR_LIVE_STOPPED;fake_status.live_reason="operator_stop";}
bool RadarScan_HostLiveCopy(RadarLive_Point_t points[RADAR_LIVE_BINS])
{
  if(!fake_status.live_sequence)return false;
  memcpy(points,fake_preview,sizeof fake_preview);return true;
}

static void complete_scan(uint32_t id)
{
  memset(&fake_status,0,sizeof fake_status);
  fake_status.scan_id=id;fake_status.state=RADAR_SCAN_READY;fake_status.reason="complete";
  fake_status.points_valid=true;fake_status.point_count=fake_cloud_count;
  fake_status.raw_points=fake_cloud_count;fake_status.coverage_tenths=3600;
}

static void test_defaults_and_transactional_draft(void)
{
  RadarHost_Status_t status;
  const RadarHostMap_Config_t *active,*draft;
  uint32_t revision,config_id;

  RadarHost_Init();RadarHost_StatusGet(&status);active=RadarHost_ActiveConfig();
  assert(status.session_id && status.config_id==1 && !status.map_id && !status.plan_id);
  assert(active->radar_x_mm==230 && active->radar_y_mm==230 && active->threshold==1);
  assert(active->protected_mask==RADAR_HOST_PROTECTED_MASK && active->fixed_mask==RADAR_HOST_FIXED_MASK);

  revision=RadarHost_DraftReset();assert(revision);draft=RadarHost_DraftConfig();
  assert(!memcmp(active,draft,sizeof *active));
  assert(RadarHost_DraftSet("angle_min","3500"));
  assert(RadarHost_DraftSet("angle_max","100"));
  assert(RadarHost_DraftSet("distance_min","50"));
  assert(RadarHost_DraftSet("distance_max","5000"));
  assert(RadarHost_DraftSet("energy_min","5"));
  assert(RadarHost_DraftSet("energy_max","200"));
  assert(RadarHost_DraftSet("radar_x","240"));
  assert(RadarHost_DraftSet("radar_y","250"));
  assert(RadarHost_DraftSet("x1","600"));assert(RadarHost_DraftSet("x2","1050"));
  assert(RadarHost_DraftSet("x3","1450"));assert(RadarHost_DraftSet("x4","1900"));
  assert(RadarHost_DraftSet("y1","610"));assert(RadarHost_DraftSet("y2","1060"));
  assert(RadarHost_DraftSet("y3","1460"));assert(RadarHost_DraftSet("y4","1910"));
  assert(RadarHost_DraftSet("threshold","2"));
  assert(RadarHost_DraftSet("manual_mask","2"));
  assert(RadarHost_DraftSet("planning_mode","point"));
  assert(RadarHost_DraftSet("mission","1-5-1"));
  assert(RadarHost_DraftSet("start","1,1,radar"));
  assert(RadarHost_DraftSet("goal","1,5,goal"));
  assert(RadarHost_Apply(revision));
  RadarHost_StatusGet(&status);active=RadarHost_ActiveConfig();config_id=status.config_id;
  assert(config_id==2 && active->angle_min_tenths==3500 && active->angle_max_tenths==100);
  assert(active->grid.x_mm[1]==600 && active->grid.y_mm[4]==1910 && active->planning_mode==RADAR_HOST_PLAN_POINT);
  assert(active->mission_count==3 && active->mission[1]==5 && active->goal.anchor==RADAR_HOST_ANCHOR_GOAL_ZONE_ONE);
  assert(!status.map_id && !status.plan_id && !RadarHost_Map(0,0) && !RadarHost_Path(0,0));

  revision=RadarHost_DraftReset();
  assert(RadarHost_DraftSet("x1","1020"));
  assert(!RadarHost_Apply(revision));
  RadarHost_StatusGet(&status);assert(status.config_id==config_id);
  assert(!RadarHost_DraftSet("manual_mask","1"));
  assert(!RadarHost_DraftSet("mission","1-7-1"));
  assert(!RadarHost_Apply(revision-1U));
  assert(RadarHost_DraftReset());assert(RadarHost_DraftSet("threshold","7"));
  assert(RadarHost_ActiveConfig()->threshold==2);
}

static void test_rebuild_scan_and_history(void)
{
  RadarHost_Status_t status,before_failure;
  const RadarHostMap_Result_t *map,*path;
  uint32_t map_id,scan_id,path_id;
  unsigned scans_before;

  RadarHost_Init();fake_cloud_count=1;fake_cloud[0]=(RadarSample_t){0,100,30};
  complete_scan(7);RadarHost_Process();RadarHost_StatusGet(&status);
  assert(status.scan_id==7 && status.map_id==1 && status.plan_id==1 && status.raw_points==1 && status.filtered_points==1);
  map=RadarHost_Map(&map_id,&scan_id);path=RadarHost_Path(&path_id,&scan_id);
  assert(map && path==map && map_id==1 && path_id==1 && scan_id==7 && map->path_valid);

  scans_before=scan_calls;
  uint32_t revision=RadarHost_DraftReset();assert(RadarHost_DraftSet("threshold","2"));
  assert(RadarHost_Apply(revision));RadarHost_StatusGet(&status);
  assert(scan_calls==scans_before && status.scan_id==7 && status.map_id==2 && status.plan_id==2 && status.config_id==2);
  assert(RadarHost_Map(&map_id,&scan_id)->counts[0]==1 && scan_id==7);

  complete_scan(8);fake_cloud[0]=(RadarSample_t){900,545,30};RadarHost_Process();
  RadarHost_StatusGet(&status);assert(status.scan_id==8 && status.map_id==3 && status.plan_id==3);
  before_failure=status;fake_status.state=RADAR_SCAN_ERROR;fake_status.reason="scan_timeout";
  fake_status.points_valid=false;RadarHost_Process();RadarHost_StatusGet(&status);
  assert(status.map_id==before_failure.map_id && status.plan_id==before_failure.plan_id);
  assert(RadarHost_Map(&map_id,&scan_id) && scan_id==8);
}

static void test_commands_preview_and_cloud(void)
{
  RadarHost_Status_t status;
  uint16_t count;uint32_t identity;
  const RadarLive_Point_t *preview;

  RadarHost_Init();rotate_calls=scan_calls=stop_calls=live_start_calls=live_stop_calls=0;
  assert(RadarHost_StartRotation() && rotate_calls==1);
  assert(RadarHost_Scan() && scan_calls==1);
  assert(last_scan_parameters.lidar_x_mm==230 && last_scan_parameters.lidar_y_mm==230 &&
    last_scan_parameters.zero_deg==90 && last_scan_parameters.angle_max_tenths==3600);
  RadarHost_Stop();assert(stop_calls==1);
  assert(RadarHost_LiveStart() && live_start_calls==1);
  assert(last_live_parameters.zero_deg==90 && last_live_parameters.angle_max_tenths==3600);
  fake_preview[0]=(RadarLive_Point_t){5,123};fake_status.live_sequence=4;fake_status.live_points=1;
  fake_status.live_active=true;RadarHost_Process();
  preview=RadarHost_Preview(&count,&identity);assert(preview && count==1 && identity==4 && preview[0].distance_mm==123);
  RadarHost_LiveStop();assert(live_stop_calls==1);

  fake_cloud_count=1;fake_cloud[0]=(RadarSample_t){0,100,10};complete_scan(12);RadarHost_Process();
  assert(RadarHost_Cloud(&count,&identity)==fake_cloud && count==1 && identity==12);
  RadarHost_StatusGet(&status);assert(!status.active && status.cloud_valid && status.preview_id==4);
}

int main(void)
{
  test_defaults_and_transactional_draft();
  test_rebuild_scan_and_history();
  test_commands_preview_and_cloud();
  puts("radar_host: transactional config, same-circle rebuild, scan history and preview PASS");
  return 0;
}
