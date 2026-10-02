/** @file radar_scan.h
 * @brief One stationary LDS50C rotation; published maps survive cancellation.
 */
#ifndef RADAR_SCAN_H
#define RADAR_SCAN_H
#include "radar_uart.h"
#include "radar_map.h"
#define RADAR_SCAN_POINT_CAPACITY 4096U
#define RADAR_SCAN_TIMEOUT_MS 10000U
typedef enum {
  RADAR_SCAN_IDLE, RADAR_SCAN_PREPARING, RADAR_SCAN_WAIT_WRAP, RADAR_SCAN_COLLECTING,
  RADAR_SCAN_READY, RADAR_SCAN_STOPPED, RADAR_SCAN_ERROR
} RadarScan_State_t;
typedef enum { RADAR_UNIT_UNKNOWN, RADAR_UNIT_MM, RADAR_UNIT_CM } RadarScan_Unit_t;
typedef struct {
  RadarScan_State_t state;
  const char *reason;
  uint32_t bytes, packets, bad, overflow, alarm, elapsed_ms;
  uint32_t scan_id, map_id, points_map_id, origin_generation, completed_tick;
  uint32_t raw_points, circle_restarts, unit_changes, unit_overflows;
  bool map_valid, points_valid, truncated, map_applicable;
  uint16_t point_count, coverage_tenths;
  RadarScan_Unit_t unit;
} RadarScan_Status_t;
HAL_StatusTypeDef RadarScan_Init(UART_HandleTypeDef *uart);
/** @brief Accept a stationary capture; caller checks chassis/arm and origin readiness.
 * @param params Frozen parameters for this capture, mm/degrees/tenths as named.
 * @param origin_generation Logical chassis-origin generation used by this map.
 * @return Accepted only; completed map is published by Process.
 */
bool RadarScan_Start(const RadarMap_Params_t *params, uint32_t origin_generation);
void RadarScan_Stop(void);
void RadarScan_Process(void);
void RadarScan_StatusGet(RadarScan_Status_t *status);
/** @brief Last complete map, including during a failed or cancelled recapture. */
const RadarMap_t *RadarScan_Map(void);
const RadarMap_Params_t *RadarScan_MapParams(void);
const char *RadarScan_StateName(RadarScan_State_t state);
const char *RadarScan_UnitName(RadarScan_Unit_t unit);
/** @brief Complete cloud only. Recapture reuses its single buffer, so returns NULL
 * until completion; points_map_id identifies the associated published map. */
const RadarSample_t *RadarScan_Points(uint16_t *count);
/** @brief Update the next capture's RAM parameters; never starts or clears a map. */
bool RadarScan_SetParam(const char *key, float value);
const RadarMap_Params_t *RadarScan_Params(void);
#endif
