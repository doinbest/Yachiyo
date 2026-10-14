#include "radar_map.h"
#include "radar_start.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
  RadarMap_Params_t p,upper;RadarMap_t map;RadarPlan_t plan;RadarStablePoint points[4]={0};
  RadarMap_Defaults(&p);
  assert(p.situation==2 && p.grid_version==4 && p.lidar_x_mm==2280 && p.lidar_y_mm==314);
  assert(p.geometry.matrix_q16[0]==65809 && p.geometry.origin_q16[0]==149407997);
  assert(RadarStart_ParamsForStart(&p,RADAR_START_UPPER,&upper));
  assert(upper.lidar_x_mm==2050 && upper.lidar_y_mm==2250 && upper.zero_deg==270);
  assert(upper.geometry.matrix_q16[0]==65536 && upper.geometry.matrix_q16[1]==0);
  p.upper_lidar_x_mm=2040.5f;p.upper_lidar_y_mm=2240.0f;
  assert(RadarStart_ParamsForStart(&p,RADAR_START_UPPER,&upper));
  assert(upper.lidar_x_mm==2040.5f && upper.lidar_y_mm==2240 && upper.zero_deg==270);
  assert(upper.geometry.origin_q16[0]==133726208L && upper.geometry.origin_q16[1]==146800640L);
  RadarMap_FreezeParams(&upper,&upper);
  assert(upper.lidar_x_mm==2040.5f && upper.geometry.origin_q16[0]==133726208L);
  assert(p.lidar_x_mm==2280 && p.lidar_y_mm==314 && p.geometry.origin_q16[0]==149407997);
  assert(RadarStart_ParamsForStart(&p,RADAR_START_LOWER,&upper));
  assert(upper.zero_deg==180 && upper.geometry.zero_tenths==1800);
  points[0].x_mm=150;points[0].y_mm=150;points[0].votes=3;
  points[1].x_mm=550;points[1].y_mm=550;points[1].votes=3;
  points[2].x_mm=2250;points[2].y_mm=150;points[2].votes=3;
  points[3].x_mm=2200;points[3].y_mm=300;points[3].votes=3;
  RadarMap_BuildStable(&map,&p,points,4);
  assert(map.counts[0]==1 && map.counts[6]==1 && map.counts[4]==1 && map.outside_points==1);
  RadarMap_Finish(&map,&p);assert(!(map.mask&(1UL<<4)) && map.counts[4]==1);
  RadarMap_Reset(&map);assert(RadarPlan_Build(&map,&plan));
  assert(plan.points[0].x_mm==2250 && plan.points[0].y_mm==150);
  unsigned visits=0;for(unsigned i=0;i<plan.count;i++)if(plan.points[i].station)visits++;
  assert(visits==9);
  assert(RADAR_MAP_FIXED_MASK&(1UL<<18)); /* R4C4 remains a fixed exclusion. */
  puts("radar_g4: factory2, independent upper, half-open stable counts, vehicle route PASS");
  return 0;
}
