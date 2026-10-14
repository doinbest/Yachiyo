/**
 * @file radar_host_map.h
 * @brief 网页雷达建图使用的F1兼容纯计算接口；不依赖HAL和全局状态。
 */
#ifndef RADAR_HOST_MAP_H
#define RADAR_HOST_MAP_H

#include "radar_map.h"
#include "radar_geometry.h"
#include "radar_stability.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RADAR_HOST_MAX_MISSION_VISITS 16U
#define RADAR_HOST_MAX_WAYPOINTS      RADAR_PLAN_MAX_POINTS
#define RADAR_HOST_FIXED_MASK         0x00050140UL
#define RADAR_HOST_PROTECTED_MASK     0x00404405UL

typedef enum {
  RADAR_HOST_PLAN_MISSION=0,
  RADAR_HOST_PLAN_POINT=1
} RadarHostMap_PlanningMode_t;

typedef enum {
  RADAR_HOST_ANCHOR_CELL=0,
  RADAR_HOST_ANCHOR_RADAR=1,
  RADAR_HOST_ANCHOR_GOAL_ZONE_ONE=2
} RadarHostMap_Anchor_t;

typedef enum {
  RADAR_HOST_DIR_NORTH=0,
  RADAR_HOST_DIR_EAST=1,
  RADAR_HOST_DIR_SOUTH=2,
  RADAR_HOST_DIR_WEST=3,
  RADAR_HOST_DIR_END=0xFF
} RadarHostMap_Direction_t;

typedef enum {
  RADAR_HOST_TURN_START=0,
  RADAR_HOST_TURN_STRAIGHT=1,
  RADAR_HOST_TURN_LEFT=2,
  RADAR_HOST_TURN_RIGHT=3,
  RADAR_HOST_TURN_REVERSE=4,
  RADAR_HOST_TURN_END=0xFF
} RadarHostMap_Turn_t;

typedef struct {
  uint8_t row,column; /**< 1..5。 */
  uint8_t anchor;     /**< RadarHostMap_Anchor_t。 */
} RadarHostMap_Endpoint_t;

typedef struct {
  RadarGrid_t grid;
  uint16_t radar_x_mm,radar_y_mm;
  uint16_t angle_min_tenths,angle_max_tenths;
  uint16_t distance_min_mm,distance_max_mm;
  uint16_t threshold;
  uint8_t energy_min,energy_max;
  uint32_t fixed_mask,manual_mask,protected_mask;
  uint8_t planning_mode;
  uint8_t mission_count;
  uint8_t mission[RADAR_HOST_MAX_MISSION_VISITS];
  RadarHostMap_Endpoint_t start,goal;
  RadarGeometry geometry;
  uint8_t situation,grid_version;
} RadarHostMap_Config_t;

typedef struct {
  uint16_t x_mm,y_mm,distance_to_next_mm;
  uint8_t row,column,direction,turn;
} RadarHostMap_Waypoint_t;

typedef struct {
  uint32_t raw_points,filtered_points,outside_points;
  uint16_t counts[RADAR_MAP_CELL_COUNT];
  uint32_t scanned_mask,manual_mask,effective_mask;
  bool path_valid;
  uint8_t failed_from_task,failed_to_task;
  uint16_t waypoint_count;
  RadarHostMap_Waypoint_t waypoints[RADAR_HOST_MAX_WAYPOINTS];
  RadarStableStatus stability;
} RadarHostMap_Result_t;

/** @brief 载入F1页面默认筛选、网格、保护格和1-5-4-2-3-4-2-3-1路线。 */
void RadarHostMap_Defaults(RadarHostMap_Config_t *config);
void RadarHostMap_DefaultsSituation(RadarHostMap_Config_t *config,uint8_t situation);
void RadarHostMap_FreezeConfig(const RadarHostMap_Config_t *input,RadarHostMap_Config_t *frozen);
bool RadarHostMap_BuildStable(const RadarHostMap_Config_t *config,
                             const RadarStablePoint *points,uint16_t count,
                             const RadarStableStatus *status,RadarHostMap_Result_t *result);
/** @return 配置范围、动态网格、任务顺序和端点完整合法时为true。 */
bool RadarHostMap_ConfigValid(const RadarHostMap_Config_t *config);
/** @brief 将一个毫米原始点加入当前圈的统计；result须先清零，config须合法。
 * @param config 同圈冻结的筛选、位姿及网格。
 * @param sample 原始角度、毫米距离及能量。
 * @param result 当前圈累计结果；尚未完成时不发布。
 */
void RadarHostMap_Add(const RadarHostMap_Config_t *config,const RadarSample_t *sample,
                      RadarHostMap_Result_t *result);
/** @brief 完整圈结束后按F1阈值/保护格生成障碍和路线；也用于同圈重规划。
 * @param config 合法的筛选、网格及路线配置。
 * @param result 已累计完整圈点数的结果。
 */
void RadarHostMap_Finish(const RadarHostMap_Config_t *config,RadarHostMap_Result_t *result);
/** @brief 从同一完整圈纯计算点数、障碍与展开路径；路径失败不丢失地图结果。
 * @return 配置及输入指针合法并完成建图时为true。
 */
bool RadarHostMap_Build(const RadarHostMap_Config_t *config,
                        const RadarSample_t *samples,uint16_t count,
                        RadarHostMap_Result_t *result);

#ifdef __cplusplus
}
#endif
#endif
