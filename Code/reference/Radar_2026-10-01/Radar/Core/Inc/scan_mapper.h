#ifndef SCAN_MAPPER_H
#define SCAN_MAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "lds_parser.h"
#include "navigation_config.h"

typedef enum
{
  SCAN_WAIT_FIRST_WRAP = 0,
  SCAN_COLLECTING,
  SCAN_COMPLETE
} ScanMapperState;

typedef struct
{
  ScanMapperState state;
  uint16_t previous_start_angle_tenths;
  uint8_t have_previous_start;
  uint8_t pending_valid;
  LdsParserResult pending_variable_frame;
  uint16_t cell_point_counts[NAV_GRID_CELL_COUNT];
  uint16_t valid_scan_points;
  uint32_t measurement_frame_count;
  uint32_t raw_point_count;
  uint16_t first_start_angle_tenths;
  uint16_t last_start_angle_tenths;
  uint16_t min_start_angle_tenths;
  uint16_t max_start_angle_tenths;
  uint16_t wrap_count;
  uint8_t diagnostic_angle_seen;
  uint32_t processed_point_count;
  uint32_t angle_rejected_point_count;
  uint32_t distance_rejected_point_count;
  uint32_t intensity_rejected_point_count;
  uint32_t outside_field_point_count;
  uint16_t min_distance_mm;
  uint16_t max_distance_mm;
  uint8_t diagnostic_distance_seen;
} ScanMapper;

void ScanMapper_Init(ScanMapper *mapper);
void ScanMapper_PushFrame(ScanMapper *mapper, const LdsParserResult *frame);
uint8_t ScanMapper_IsComplete(const ScanMapper *mapper);

#ifdef __cplusplus
}
#endif

#endif
