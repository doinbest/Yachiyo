/**
 * @file radar_host_map.c
 * @brief F1雷达上位机的筛选、5×5障碍地图和确定性四邻域规划。
 */
#include "radar_host_map.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RADAR_HOST_DEG_TO_RAD 0.01745329251994329577f

static const uint8_t MissionRows[5]={1,3,5,3,1};
static const uint8_t MissionColumns[5]={1,1,3,5,3};
static const uint8_t DefaultMission[9]={1,5,4,2,3,4,2,3,1};

static bool endpoint_valid(const RadarHostMap_Endpoint_t *endpoint)
{
  return endpoint!=NULL && endpoint->row>=1U && endpoint->row<=5U &&
    endpoint->column>=1U && endpoint->column<=5U &&
    endpoint->anchor<=RADAR_HOST_ANCHOR_GOAL_ZONE_ONE;
}

void RadarHostMap_Defaults(RadarHostMap_Config_t *config)
{
  if(config==NULL) return;
  memset(config,0,sizeof(*config));
  /* Keep the verified backup/F1 workspace independent of the vehicle grid. */
  {
    static const RadarGrid_t diagnostic_grid={
      {150,550,1000,1400,1850,2250},{150,550,1000,1400,1850,2250}
    };
    config->grid=diagnostic_grid;
  }
  config->radar_x_mm=230U;config->radar_y_mm=230U;
  config->angle_min_tenths=0U;config->angle_max_tenths=3600U;
  config->distance_min_mm=100U;config->distance_max_mm=40000U;
  config->threshold=1U;config->energy_min=0U;config->energy_max=255U;
  config->fixed_mask=RADAR_HOST_FIXED_MASK;
  config->protected_mask=RADAR_HOST_PROTECTED_MASK;
  config->planning_mode=RADAR_HOST_PLAN_MISSION;
  config->mission_count=(uint8_t)(sizeof(DefaultMission)/sizeof(DefaultMission[0]));
  memcpy(config->mission,DefaultMission,sizeof(DefaultMission));
  config->start.row=1U;config->start.column=1U;config->start.anchor=RADAR_HOST_ANCHOR_RADAR;
  config->goal.row=1U;config->goal.column=5U;config->goal.anchor=RADAR_HOST_ANCHOR_GOAL_ZONE_ONE;
}

bool RadarHostMap_ConfigValid(const RadarHostMap_Config_t *config)
{
  unsigned i;
  if(config==NULL || !RadarGrid_Valid(&config->grid) ||
     config->radar_x_mm>2400U || config->radar_y_mm>2400U ||
     config->angle_min_tenths>3600U || config->angle_max_tenths>3600U ||
     config->distance_min_mm>config->distance_max_mm ||
     config->energy_min>config->energy_max || config->threshold==0U || config->threshold>4096U ||
     config->fixed_mask!=RADAR_HOST_FIXED_MASK || config->protected_mask!=RADAR_HOST_PROTECTED_MASK ||
     config->planning_mode>RADAR_HOST_PLAN_POINT || !endpoint_valid(&config->start) ||
     !endpoint_valid(&config->goal) || config->mission_count<2U ||
     config->mission_count>RADAR_HOST_MAX_MISSION_VISITS) return false;
  if(config->mission[0]!=1U || config->mission[config->mission_count-1U]!=1U) return false;
  for(i=0U;i<config->mission_count;i++) {
    if(config->mission[i]<1U || config->mission[i]>5U) return false;
    if(i>0U && config->mission[i]==config->mission[i-1U]) return false;
  }
  return true;
}

static bool sample_allowed(const RadarHostMap_Config_t *config,const RadarSample_t *sample)
{
  const uint16_t angle=(uint16_t)(sample->angle_tenths%3600U);
  const uint16_t minimum=(uint16_t)(config->angle_min_tenths%3600U);
  const uint16_t maximum=(uint16_t)(config->angle_max_tenths%3600U);
  bool angle_ok;
  if(config->angle_min_tenths==0U && config->angle_max_tenths==3600U) angle_ok=true;
  else if(config->angle_min_tenths<=config->angle_max_tenths) angle_ok=angle>=minimum && angle<=maximum;
  else angle_ok=angle>=minimum || angle<=maximum;
  return angle_ok && sample->distance_mm>=config->distance_min_mm &&
    sample->distance_mm<=config->distance_max_mm && sample->energy>=config->energy_min &&
    sample->energy<=config->energy_max;
}

static void project(const RadarHostMap_Config_t *config,const RadarSample_t *sample,float *x,float *y)
{
  const float angle=(float)(sample->angle_tenths%3600U)*0.1f*RADAR_HOST_DEG_TO_RAD;
  *x=(float)config->radar_x_mm+(float)sample->distance_mm*sinf(angle);
  *y=(float)config->radar_y_mm+(float)sample->distance_mm*cosf(angle);
}

static uint8_t mission_cell(uint8_t task)
{
  return (uint8_t)((MissionRows[task-1U]-1U)*5U+MissionColumns[task-1U]-1U);
}

static uint8_t endpoint_cell(const RadarHostMap_Endpoint_t *endpoint)
{
  return (uint8_t)((endpoint->row-1U)*5U+endpoint->column-1U);
}

static uint8_t direction_between(uint8_t from,uint8_t to)
{
  const uint8_t from_row=(uint8_t)(from/5U),from_column=(uint8_t)(from%5U);
  const uint8_t to_row=(uint8_t)(to/5U),to_column=(uint8_t)(to%5U);
  if(to_row>from_row) return RADAR_HOST_DIR_NORTH;
  if(to_column>from_column) return RADAR_HOST_DIR_EAST;
  if(to_row<from_row) return RADAR_HOST_DIR_SOUTH;
  return RADAR_HOST_DIR_WEST;
}

static uint8_t turn_between(uint8_t previous,uint8_t current)
{
  const uint8_t difference=(uint8_t)((current+4U-previous)%4U);
  if(difference==0U) return RADAR_HOST_TURN_STRAIGHT;
  if(difference==1U) return RADAR_HOST_TURN_RIGHT;
  if(difference==2U) return RADAR_HOST_TURN_REVERSE;
  return RADAR_HOST_TURN_LEFT;
}

static void endpoint_position(const RadarHostMap_Config_t *config,
                              const RadarHostMap_Endpoint_t *endpoint,uint8_t cell,
                              int16_t *x,int16_t *y)
{
  (void)RadarGrid_CellCenter(&config->grid,cell,x,y);
  if(endpoint->anchor==RADAR_HOST_ANCHOR_RADAR) {
    *x=(int16_t)config->radar_x_mm;*y=(int16_t)config->radar_y_mm;
  } else if(endpoint->anchor==RADAR_HOST_ANCHOR_GOAL_ZONE_ONE) {
    *x=2250;*y=150;
  }
}

static bool fill_waypoints(const RadarHostMap_Config_t *config,const uint8_t *cells,
                           uint16_t count,const RadarHostMap_Endpoint_t *start,
                           const RadarHostMap_Endpoint_t *goal,bool mission,
                           RadarHostMap_Result_t *result)
{
  uint16_t i;
  if(count==0U || count>RADAR_HOST_MAX_WAYPOINTS) return false;
  for(i=0U;i<count;i++) {
    int16_t x,y;
    RadarHostMap_Waypoint_t *waypoint=&result->waypoints[i];
    if(!RadarGrid_CellCenter(&config->grid,cells[i],&x,&y)) return false;
    if(mission) {
      if(i==0U || i+1U==count) {x=(int16_t)config->radar_x_mm;y=(int16_t)config->radar_y_mm;}
    } else if(i==0U) endpoint_position(config,start,cells[i],&x,&y);
    else if(i+1U==count) endpoint_position(config,goal,cells[i],&x,&y);
    waypoint->x_mm=(uint16_t)x;waypoint->y_mm=(uint16_t)y;
    waypoint->row=(uint8_t)(cells[i]/5U+1U);waypoint->column=(uint8_t)(cells[i]%5U+1U);
  }
  for(i=0U;i+1U<count;i++) {
    RadarHostMap_Waypoint_t *waypoint=&result->waypoints[i];
    const RadarHostMap_Waypoint_t *next=&result->waypoints[i+1U];
    const float dx=(float)next->x_mm-(float)waypoint->x_mm;
    const float dy=(float)next->y_mm-(float)waypoint->y_mm;
    waypoint->distance_to_next_mm=(uint16_t)lroundf(hypotf(dx,dy));
    waypoint->direction=direction_between(cells[i],cells[i+1U]);
    waypoint->turn=i==0U?RADAR_HOST_TURN_START:
      turn_between(result->waypoints[i-1U].direction,waypoint->direction);
  }
  result->waypoints[count-1U].direction=RADAR_HOST_DIR_END;
  result->waypoints[count-1U].turn=RADAR_HOST_TURN_END;
  result->waypoint_count=count;result->path_valid=true;
  return true;
}

static void plan_mission(const RadarHostMap_Config_t *config,RadarHostMap_Result_t *result)
{
  uint8_t route[RADAR_HOST_MAX_WAYPOINTS];
  uint16_t route_count=0U;
  unsigned leg;
  for(leg=0U;leg+1U<config->mission_count;leg++) {
    uint8_t path[RADAR_MAP_CELL_COUNT],path_count=0U;
    const uint8_t from=config->mission[leg],to=config->mission[leg+1U];
    unsigned i,first=leg==0U?0U:1U;
    if(!RadarGrid_Path(&config->grid,result->effective_mask,mission_cell(from),mission_cell(to),path,&path_count)) {
      result->failed_from_task=from;result->failed_to_task=to;return;
    }
    for(i=first;i<path_count;i++) {
      if(route_count>=RADAR_HOST_MAX_WAYPOINTS) {
        result->failed_from_task=from;result->failed_to_task=to;return;
      }
      route[route_count++]=path[i];
    }
  }
  (void)fill_waypoints(config,route,route_count,&config->start,&config->goal,true,result);
}

static void plan_point(const RadarHostMap_Config_t *config,RadarHostMap_Result_t *result)
{
  uint8_t path[RADAR_MAP_CELL_COUNT],count=0U;
  if(RadarGrid_Path(&config->grid,result->effective_mask,endpoint_cell(&config->start),
                    endpoint_cell(&config->goal),path,&count))
    (void)fill_waypoints(config,path,count,&config->start,&config->goal,false,result);
}

void RadarHostMap_Add(const RadarHostMap_Config_t *config,const RadarSample_t *sample,
                      RadarHostMap_Result_t *result)
{
  if(result->raw_points<0xFFFFU)++result->raw_points;
  if(sample_allowed(config,sample)) {
    float x,y;
    int8_t cell;
    if(result->filtered_points<0xFFFFU) ++result->filtered_points;
    project(config,sample,&x,&y);
    cell=RadarGrid_FindCell(&config->grid,x,y,false);
    if(cell<0) {if(result->outside_points<0xFFFFU) ++result->outside_points;return;}
    if(result->counts[(uint8_t)cell]<0xFFFFU) ++result->counts[(uint8_t)cell];
  }
}

void RadarHostMap_Finish(const RadarHostMap_Config_t *config,RadarHostMap_Result_t *result)
{
  uint32_t scanned=0U;
  result->path_valid=false;result->waypoint_count=0U;
  result->failed_from_task=result->failed_to_task=0U;
  for(unsigned i=0U;i<RADAR_MAP_CELL_COUNT;i++)
    if(result->counts[i]>=config->threshold) scanned|=1UL<<i;
  result->manual_mask=config->manual_mask&RADAR_MAP_ALL_CELLS&~config->protected_mask&~config->fixed_mask;
  result->scanned_mask=scanned&~config->protected_mask;
  result->effective_mask=(config->fixed_mask|result->manual_mask|result->scanned_mask)&RADAR_MAP_ALL_CELLS;
  if(config->planning_mode==RADAR_HOST_PLAN_MISSION) plan_mission(config,result);
  else plan_point(config,result);
}

bool RadarHostMap_Build(const RadarHostMap_Config_t *config,
                        const RadarSample_t *samples,uint16_t count,
                        RadarHostMap_Result_t *result)
{
  if(result==NULL) return false;
  memset(result,0,sizeof(*result));
  if(!RadarHostMap_ConfigValid(config) || (count>0U && samples==NULL)) return false;
  for(uint16_t i=0U;i<count;i++)RadarHostMap_Add(config,&samples[i],result);
  RadarHostMap_Finish(config,result);
  return true;
}
