#ifndef NAVIGATION_CONFIG_H
#define NAVIGATION_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define NAV_GRID_ROWS                 5U
#define NAV_GRID_COLUMNS              5U
#define NAV_GRID_CELL_COUNT           (NAV_GRID_ROWS * NAV_GRID_COLUMNS)
#define NAV_GRID_BOUNDARY_COUNT       6U
#define NAV_MAX_WAYPOINTS             193U

#define NAV_RADAR_X_MM                230U
#define NAV_RADAR_Y_MM                230U

#define NAV_FILTER_MIN_ANGLE_TENTHS   0U
#define NAV_FILTER_MAX_ANGLE_TENTHS   3600U
#define NAV_FILTER_MIN_DISTANCE_MM    100U
#define NAV_FILTER_MAX_DISTANCE_MM    40000U
#define NAV_FILTER_MIN_INTENSITY      0U
#define NAV_FILTER_MAX_INTENSITY      255U

#define NAV_OBSTACLE_POINT_THRESHOLD  1U
#define NAV_SCAN_TIMEOUT_MS           10000UL

#define NAV_MISSION_POINT_COUNT       5U
#define NAV_MISSION_ORDER_LENGTH      9U

#define NAV_CELL_INDEX(row, column) \
  ((((uint32_t)(row) - 1UL) * NAV_GRID_COLUMNS) + ((uint32_t)(column) - 1UL))
#define NAV_CELL_MASK(row, column)    (1UL << NAV_CELL_INDEX((row), (column)))

#define NAV_FIXED_OBSTACLE_MASK \
  (NAV_CELL_MASK(2U, 2U) | NAV_CELL_MASK(2U, 4U) | \
   NAV_CELL_MASK(4U, 2U) | NAV_CELL_MASK(4U, 4U))

#define NAV_PROTECTED_CELL_MASK \
  (NAV_CELL_MASK(1U, 1U) | NAV_CELL_MASK(3U, 1U) | \
   NAV_CELL_MASK(5U, 3U) | \
   NAV_CELL_MASK(3U, 5U) | NAV_CELL_MASK(1U, 3U))

extern const uint16_t g_nav_grid_x_mm[NAV_GRID_BOUNDARY_COUNT];
extern const uint16_t g_nav_grid_y_mm[NAV_GRID_BOUNDARY_COUNT];
extern const uint8_t g_nav_mission_rows[NAV_MISSION_POINT_COUNT];
extern const uint8_t g_nav_mission_columns[NAV_MISSION_POINT_COUNT];
extern const uint8_t g_nav_mission_order[NAV_MISSION_ORDER_LENGTH];

#ifdef __cplusplus
}
#endif

#endif
