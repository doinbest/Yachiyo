#ifndef NAVIGATION_TYPES_H
#define NAVIGATION_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lds_parser.h"
#include "navigation_config.h"

typedef enum
{
  NAV_IDLE = 0,
  NAV_SCANNING,
  NAV_PLANNING,
  NAV_PATH_READY,
  NAV_ERROR
} NavState;

typedef enum
{
  NAV_ERROR_NONE = 0,
  NAV_ERROR_RADAR,
  NAV_ERROR_SCAN_TIMEOUT,
  NAV_ERROR_NOT_ENOUGH_POINTS,
  NAV_ERROR_NO_PATH,
  NAV_ERROR_PATH_OVERFLOW
} NavError;

typedef enum
{
  NAV_DIR_NORTH = 0,
  NAV_DIR_EAST,
  NAV_DIR_SOUTH,
  NAV_DIR_WEST,
  NAV_DIR_END = 0xFF
} NavDirection;

typedef enum
{
  NAV_TURN_START = 0,
  NAV_TURN_STRAIGHT,
  NAV_TURN_LEFT,
  NAV_TURN_RIGHT,
  NAV_TURN_REVERSE,
  NAV_TURN_END = 0xFF
} NavTurn;

typedef struct
{
  uint16_t x_mm;
  uint16_t y_mm;
  uint16_t distance_to_next_mm;
  uint8_t row;
  uint8_t column;
  uint8_t direction;
  uint8_t turn;
} NavWaypoint;

typedef struct
{
  NavState state;
  NavError error;
  uint8_t failed_from_task;
  uint8_t failed_to_task;
  uint16_t cell_point_counts[NAV_GRID_CELL_COUNT];
  uint32_t scanned_obstacle_mask;
  uint32_t effective_obstacle_mask;
  uint16_t valid_scan_points;
  uint32_t rx_bytes_received;
  uint32_t rx_measurement_packets;
  uint32_t rx_status_packets;
  uint32_t rx_bad_length_packets;
  uint32_t rx_bad_checksum_packets;
  uint32_t rx_discarded_bytes;
  uint16_t rx_max_declared_point_count;
  uint8_t rx_last_status_flags;
  uint8_t rx_distance_unit_mm;
  uint8_t rx_raw_sample_count;
  uint8_t rx_raw_sample[LDS_RAW_SAMPLE_CAPACITY];
  uint32_t scan_measurement_frames;
  uint32_t scan_raw_points;
  uint16_t scan_first_angle_tenths;
  uint16_t scan_last_angle_tenths;
  uint16_t scan_min_angle_tenths;
  uint16_t scan_max_angle_tenths;
  uint16_t scan_wrap_count;
  uint32_t map_processed_points;
  uint32_t map_angle_rejected_points;
  uint32_t map_distance_rejected_points;
  uint32_t map_intensity_rejected_points;
  uint32_t map_outside_field_points;
  uint16_t map_min_distance_mm;
  uint16_t map_max_distance_mm;
  uint16_t path_count;
  NavWaypoint path[NAV_MAX_WAYPOINTS];
} NavigationResult;

#ifdef __cplusplus
}
#endif

#endif
