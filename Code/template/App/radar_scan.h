/** @file radar_scan.h
 * @brief Static LDS50C maps and continuous clouds with independent history.
 */
#ifndef RADAR_SCAN_H
#define RADAR_SCAN_H
#include "radar_uart.h"
#include "radar_start.h"
#define RADAR_SCAN_POINT_CAPACITY 4096U
#define RADAR_SCAN_TIMEOUT_MS 10000U
#define RADAR_LIVE_BINS 120U
typedef enum { RADAR_UNIT_UNKNOWN, RADAR_UNIT_MM, RADAR_UNIT_CM } RadarScan_Unit_t;

/** @brief One retained return per 3deg bin; zero distance means empty.
 * Angles are the actual raw tenths of a degree, distances normalized to mm. */
typedef struct { uint16_t angle_tenths, distance_mm; } RadarLive_Point_t;
typedef enum {
  RADAR_LIVE_PREPARING, RADAR_LIVE_WAITING, RADAR_LIVE_STREAMING,
  RADAR_LIVE_STOPPED, RADAR_LIVE_ERROR
} RadarScan_LiveState_t;
typedef struct {
  RadarMap_Params_t parameters;
  RadarScan_LiveState_t state;
  RadarScan_Unit_t unit;
  const char *reason;
  uint32_t id, sequence, origin_generation, completed_tick, age_ms;
  uint32_t circle_restarts, unit_changes, unit_overflows;
  uint16_t points;
  bool active;
} RadarScan_LiveStatus_t;
/** @brief Start continuous nominal-corner point clouds without building maps.
 * Covers all raw angles; applies frozen distance/energy filters, nearest per bin.
 * @return false during a static/live capture or for an invalid corner/reference. */
bool RadarScan_LiveStart(const RadarMap_Params_t *lower_reference,
                         uint32_t origin_generation, RadarStart_Id_t start_id);
/** @brief Stop live only; preserves static map/cloud/raw and last live circle. */
void RadarScan_LiveStop(void);
void RadarScan_LiveStatusGet(RadarScan_LiveStatus_t *status);
/** @brief Copy the latest complete 120-bin cloud into a TX-owned frozen buffer.
 * Returns false until one complete revolution; callable from the main loop. */
bool RadarScan_LiveCopy(RadarLive_Point_t points[RADAR_LIVE_BINS]);
const char *RadarScan_LiveStateName(RadarScan_LiveState_t state);
typedef enum {
  RADAR_SCAN_IDLE, RADAR_SCAN_PREPARING, RADAR_SCAN_WAIT_WRAP, RADAR_SCAN_COLLECTING,
  RADAR_SCAN_READY, RADAR_SCAN_STOPPED, RADAR_SCAN_ERROR
} RadarScan_State_t;
typedef struct {
  RadarScan_State_t state;
  const char *reason;
  uint32_t bytes, packets, bad, overflow, alarm, elapsed_ms;
  uint32_t scan_id, map_id, points_map_id, origin_generation;
  uint32_t completed_tick; /**< Published map's circle completion time, not selection time. */
  uint32_t raw_points, circle_restarts, unit_changes, unit_overflows;
  bool map_valid, points_valid, truncated, map_applicable;
  uint16_t point_count, coverage_tenths;
  uint16_t last_alarm; /**< Latest LDS alarm code, little-endian on the sensor wire. */
  RadarScan_Unit_t unit;
} RadarScan_Status_t;
typedef struct {
  uint32_t scan_id; /**< Last completed circle; survives stop/error/new capture. */
  bool automatic;
  RadarStart_Result_t result;
} RadarScan_StartStatus_t;
/** @brief Explicit debug scan and its frozen raw/cloud binding.
 * circle_start/end delimit the first included measurement header and the final
 * boundary frame end, relative to raw.first_position. complete requires that
 * whole interval to remain in the contiguous raw window. */
typedef struct {
  RadarUart_Raw_t raw;
  RadarMap_Params_t parameters;
  RadarScan_State_t state;
  RadarScan_Unit_t unit;
  const char *reason;
  uint32_t id, scan_id, map_id, origin_generation;
  uint32_t circle_start, circle_end, point_total, bad, restarts;
  uint16_t points;
  uint8_t start_id;
  bool automatic, complete, cloud_truncated, cloud_valid;
} RadarScan_Raw_t;
/** @brief Explicit raw diagnostic capture with the nominal installation.
 * UNKNOWN evaluates both corners; UPPER/LOWER captures the named corner.
 * Ordinary Start/StartAt/StartAuto never arm raw recording. */
bool RadarScan_DebugStart(const RadarMap_Params_t *lower_reference,
                         uint32_t origin_generation, RadarStart_Id_t start_id);
void RadarScan_RawStatusGet(RadarScan_Raw_t *status);
uint16_t RadarScan_RawRead(uint32_t offset, uint8_t *bytes, uint16_t capacity);
HAL_StatusTypeDef RadarScan_Init(UART_HandleTypeDef *uart);
/** @brief Accept a stationary capture; caller checks chassis/arm and origin readiness.
 * @param params Frozen parameters for this capture, mm/degrees/tenths as named.
 * @param origin_generation Logical chassis-origin generation used by this map.
 * @return Accepted only; completed map is published by Process.
 */
bool RadarScan_Start(const RadarMap_Params_t *params, uint32_t origin_generation);
/** @brief Capture both nominal start hypotheses with the lower installation reference.
 * @return Accepted capture; only a unique result publishes its map automatically. */
bool RadarScan_StartAuto(const RadarMap_Params_t *lower_reference, uint32_t origin_generation);
/** @brief Capture one nominal corner using the lower installation reference.
 * @param lower_reference Frozen lower-start radar world reference and filters.
 * @param start_id Explicit upper/lower corner; no completed auto scan is required.
 * @return Accepted capture; publishes that corner after one complete circle.
 * Upper uses the stationary corner convention (+90deg, CW quadrant inward);
 * neither follows feedback.
 */
bool RadarScan_StartAt(const RadarMap_Params_t *lower_reference, uint32_t origin_generation,
                      RadarStart_Id_t start_id);
/** @brief Publish one retained candidate from the last completed automatic circle.
 * @return False during/new capture or before a complete automatic circle. */
bool RadarScan_SelectStart(RadarStart_Id_t start_id);
/** @brief Last completed start diagnostics; initial reason is not_scanned. */
void RadarScan_StartStatusGet(RadarScan_StartStatus_t *status);
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
