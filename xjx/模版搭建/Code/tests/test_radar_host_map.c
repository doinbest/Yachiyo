#include "radar_host_map.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned checks,failures;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; \
  printf("FAIL line %d: %s\n",__LINE__,#c); } } while(0)

static uint32_t bit(unsigned row,unsigned column)
{
  return 1UL<<((row-1U)*5U+column-1U);
}

static RadarSample_t sample_at(const RadarHostMap_Config_t *config,float x,float y,uint8_t energy)
{
  const float dx=x-(float)config->radar_x_mm,dy=y-(float)config->radar_y_mm;
  float degrees=atan2f(dx,dy)*57.29577951308232f;
  RadarSample_t sample;
  if(degrees<0.0f) degrees+=360.0f;
  sample.angle_tenths=(uint16_t)lroundf(degrees*10.0f);
  sample.distance_mm=(uint16_t)lroundf(hypotf(dx,dy));
  sample.energy=energy;
  return sample;
}

static void test_defaults_projection_and_filters(void)
{
  RadarHostMap_Config_t config;
  RadarHostMap_Result_t result;
  RadarSample_t cardinal[4]={{0,500,10},{900,500,10},{1800,500,10},{2700,500,10}};
  RadarSample_t cross[3]={{3500,500,10},{100,500,10},{900,500,10}};

  RadarHostMap_DefaultsSituation(&config,1U);
  CHECK(RadarHostMap_ConfigValid(&config));
  CHECK(config.grid.x_mm[0]==150 && config.grid.y_mm[0]==150);
  CHECK(config.grid.x_mm[5]==2250 && config.grid.y_mm[5]==2250);
  CHECK(config.radar_x_mm==230 && config.radar_y_mm==230);
  CHECK(config.angle_min_tenths==0 && config.angle_max_tenths==3600);
  CHECK(config.distance_min_mm==100 && config.distance_max_mm==40000);
  CHECK(config.energy_min==0 && config.energy_max==255 && config.threshold==1);
  CHECK(config.fixed_mask==0x00050140UL);
  CHECK(config.protected_mask==(bit(1,1)|bit(3,1)|bit(5,3)|bit(3,5)|bit(1,3)));

  config.radar_x_mm=1200;config.radar_y_mm=1200;
  CHECK(RadarHostMap_Build(&config,cardinal,4,&result));
  CHECK(result.filtered_points==4);
  CHECK(result.counts[17]==1 && result.counts[13]==1 && result.counts[7]==1 && result.counts[11]==1);

  config.angle_min_tenths=3500;config.angle_max_tenths=100;
  config.distance_min_mm=500;config.distance_max_mm=500;
  config.energy_min=10;config.energy_max=10;
  CHECK(RadarHostMap_Build(&config,cross,3,&result));
  CHECK(result.filtered_points==2 && result.counts[17]==2);
}

static void test_obstacles_and_protection(void)
{
  RadarHostMap_Config_t config;
  RadarHostMap_Result_t result;
  RadarSample_t samples[2];

  RadarHostMap_DefaultsSituation(&config,1U);
  samples[0]=sample_at(&config,350.0f,350.0f,30);
  samples[1]=sample_at(&config,775.0f,350.0f,30);
  config.manual_mask=bit(1,1)|bit(1,2)|(1UL<<30);
  CHECK(RadarHostMap_Build(&config,samples,2,&result));
  CHECK(result.counts[0]==1 && result.counts[1]==1);
  CHECK(result.manual_mask==bit(1,2));
  CHECK((result.scanned_mask&bit(1,1))==0 && (result.scanned_mask&bit(1,2))!=0);
  CHECK((result.effective_mask&config.protected_mask)==0);
  CHECK((result.effective_mask&config.fixed_mask)==config.fixed_mask);
}

static void test_default_mission_and_diagnostics(void)
{
  static const uint8_t rows[33]={1,1,1,2,3,3,3,3,3,3,3,4,5,5,5,5,5,4,3,3,3,3,3,4,5,5,5,4,3,2,1,1,1};
  static const uint8_t columns[33]={1,2,3,3,3,4,5,4,3,2,1,1,1,2,3,4,5,5,5,4,3,2,1,1,1,2,3,3,3,3,3,2,1};
  RadarHostMap_Config_t config;
  RadarHostMap_Result_t result,reroute;

  RadarHostMap_DefaultsSituation(&config,1U);
  CHECK(RadarHostMap_Build(&config,0,0,&result));
  CHECK(result.path_valid && result.waypoint_count==33);
  for(unsigned i=0;i<33;i++) {
    CHECK(result.waypoints[i].row==rows[i]);
    CHECK(result.waypoints[i].column==columns[i]);
  }
  CHECK(result.waypoints[0].x_mm==230 && result.waypoints[0].y_mm==230);
  CHECK(result.waypoints[32].x_mm==230 && result.waypoints[32].y_mm==230);
  CHECK(result.waypoints[0].distance_to_next_mm==558);
  CHECK(result.waypoints[1].distance_to_next_mm==425);
  CHECK(result.waypoints[0].direction==RADAR_HOST_DIR_EAST && result.waypoints[0].turn==RADAR_HOST_TURN_START);
  CHECK(result.waypoints[1].direction==RADAR_HOST_DIR_EAST && result.waypoints[1].turn==RADAR_HOST_TURN_STRAIGHT);

  config.manual_mask=bit(3,4);
  CHECK(RadarHostMap_Build(&config,0,0,&reroute));
  CHECK(reroute.path_valid && reroute.waypoint_count!=result.waypoint_count);
  for(unsigned i=0;i<reroute.waypoint_count;i++)
    CHECK(!(reroute.waypoints[i].row==3 && reroute.waypoints[i].column==4));

  config.manual_mask=bit(1,2)|bit(2,1);
  CHECK(RadarHostMap_Build(&config,0,0,&result));
  CHECK(!result.path_valid && result.waypoint_count==0);
  CHECK(result.failed_from_task==1 && result.failed_to_task==5);
}

static void test_point_to_point_and_dynamic_grid(void)
{
  RadarHostMap_Config_t config;
  RadarHostMap_Result_t result;

  RadarHostMap_DefaultsSituation(&config,1U);
  config.planning_mode=RADAR_HOST_PLAN_POINT;
  config.start.row=1;config.start.column=1;config.start.anchor=RADAR_HOST_ANCHOR_RADAR;
  config.goal.row=1;config.goal.column=5;config.goal.anchor=RADAR_HOST_ANCHOR_GOAL_ZONE_ONE;
  CHECK(RadarHostMap_Build(&config,0,0,&result));
  CHECK(result.path_valid && result.waypoint_count==5);
  CHECK(result.waypoints[0].x_mm==230 && result.waypoints[0].y_mm==230);
  CHECK(result.waypoints[4].x_mm==2250 && result.waypoints[4].y_mm==150);

  RadarHostMap_DefaultsSituation(&config,1U);
  config.grid.x_mm[1]=650;config.grid.y_mm[2]=1100;
  CHECK(RadarHostMap_ConfigValid(&config));
  CHECK(RadarHostMap_Build(&config,0,0,&result));
  CHECK(result.path_valid && result.waypoint_count==33);
  CHECK(result.waypoints[1].x_mm==825 && result.waypoints[1].y_mm==350);
  config.grid.x_mm[2]=699;
  CHECK(!RadarHostMap_ConfigValid(&config));
}

static void test_case_two_mission_and_frozen_geometry(void)
{
  RadarHostMap_Config_t config,frozen;
  RadarHostMap_Result_t result;
  unsigned last_task_three=0U;

  RadarHostMap_Defaults(&config);
  CHECK(config.situation==2U && config.grid_version==4U);
  CHECK(config.geometry.matrix_q16[0]==65809 && config.geometry.matrix_q16[1]==680);
  CHECK(config.geometry.matrix_q16[2]==-407 && config.geometry.matrix_q16[3]==66623);
  CHECK(config.geometry.origin_q16[0]==149407997 && config.geometry.origin_q16[1]==20569846);
  CHECK(config.geometry.zero_tenths==1800U);
  CHECK(config.protected_mask==0x00404414UL);
  CHECK(RadarHostMap_Build(&config,0,0,&result));
  CHECK(result.path_valid && result.waypoint_count==33U);
  CHECK(result.waypoints[0].row==1U && result.waypoints[0].column==5U);
  CHECK(result.waypoints[0].x_mm==2050U && result.waypoints[0].y_mm==350U);
  CHECK(result.waypoints[result.waypoint_count-1U].x_mm==2050U);
  CHECK(result.waypoints[result.waypoint_count-1U].y_mm==350U);
  for(unsigned i=0U;i<result.waypoint_count;i++) {
    const RadarHostMap_Waypoint_t *point=&result.waypoints[i];
    CHECK(!(point->row==4U && point->column==4U));
    CHECK((config.fixed_mask&bit(point->row,point->column))==0U);
    if(point->row==5U && point->column==3U)last_task_three=i;
  }
  CHECK(last_task_three+1U<result.waypoint_count);
  CHECK(result.waypoints[last_task_three+1U].row==5U);
  CHECK(result.waypoints[last_task_three+1U].column==4U);

  config.radar_x_mm+=10U;config.radar_y_mm-=5U;
  RadarHostMap_FreezeConfig(&config,&frozen);
  CHECK(frozen.geometry.origin_q16[0]==149407997+10L*65536L);
  CHECK(frozen.geometry.origin_q16[1]==20569846-5L*65536L);
  CHECK(frozen.radar_x_mm==2290U && frozen.radar_y_mm==309U);
  config=frozen;RadarHostMap_FreezeConfig(&config,&frozen);
  CHECK(frozen.geometry.origin_q16[0]==config.geometry.origin_q16[0]);
  CHECK(frozen.geometry.origin_q16[1]==config.geometry.origin_q16[1]);
}

static void test_stable_xy_counts_and_window_statistics(void)
{
  RadarHostMap_Config_t config;
  RadarHostMap_Result_t result;
  RadarStableStatus status={0};
  RadarStablePoint points[4]={{0,10,550,150,3,0},{0,10,550,150,15,0},
                            {0,10,2250,150,3,0},{0,10,550,2250,3,0}};
  RadarHostMap_Defaults(&config);
  status.window_circles=15U;status.first_circle=17U;status.last_circle=31U;
  status.raw_points=100000U;status.filtered_points=90000U;status.outside_points=80000U;
  status.candidate_count=256U;status.candidate_replacements=70000U;status.candidate_dropped=80000U;
  CHECK(RadarHostMap_BuildStable(&config,points,4U,&status,&result));
  CHECK(result.raw_points==100000U && result.filtered_points==90000U && result.outside_points==80000U);
  CHECK(result.stability.window_circles==15U && result.stability.first_circle==17U && result.stability.last_circle==31U);
  CHECK(result.stability.candidate_count==256U && result.stability.candidate_replacements==70000U);
  CHECK(result.stability.candidate_dropped==80000U && result.stability.stable_count==4U);
  CHECK(result.counts[1]==2U);
  for(unsigned i=0U;i<25U;i++)if(i!=1U)CHECK(result.counts[i]==0U);
}

int main(void)
{
  test_case_two_mission_and_frozen_geometry();
  test_stable_xy_counts_and_window_statistics();
  test_defaults_projection_and_filters();
  test_obstacles_and_protection();
  test_default_mission_and_diagnostics();
  test_point_to_point_and_dynamic_grid();
  printf("Radar host map: %u checks, %u failures\n",checks,failures);
  return failures?1:0;
}
