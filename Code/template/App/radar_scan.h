/** @file radar_scan.h
 * @brief Static LDS50C maps and continuous clouds with independent history.
 */
#ifndef RADAR_SCAN_H
#define RADAR_SCAN_H
#include "radar_uart.h"
#include "radar_start.h"
#include "radar_host_map.h"
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
  RadarMap_Params_t parameters;
  RadarScan_State_t state;
  const char *reason;
  RadarScan_Unit_t unit;
  uint32_t scan_id,completed_tick,elapsed_ms,raw_points;
  uint32_t circle_restarts,unit_changes,unit_overflows;
  uint16_t point_count,coverage_tenths;
  bool active,points_valid,truncated;
  uint32_t live_id,live_sequence,live_completed_tick;
  uint32_t live_circle_restarts,live_unit_changes,live_unit_overflows;
  uint16_t live_points;
  bool live_active;
  RadarScan_LiveState_t live_state;
  RadarScan_Unit_t live_unit;
  const char *live_reason;
} RadarScan_HostStatus_t;
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
  uint8_t circles_done, circles_target; /**< Complete circles in the active capture. */
  uint8_t map_circles; /**< Number of circles merged into the retained complete map. */
} RadarScan_Status_t;
typedef struct {
  uint32_t scan_id; /**< Last completed circle; survives stop/error/new capture. */
  bool automatic;
  RadarStart_Result_t result;
} RadarScan_StartStatus_t;
/** @brief Bounded per-cell provenance: first two and latest two returns. */
#define RADAR_EVIDENCE_HITS 4U
typedef struct {
  uint16_t angle_tenths,distance_mm,raw_distance;
  uint16_t packet_start,packet_span,packet_count,packet_index;
  uint8_t energy,flags; /**< bit0: CF; bit1: sensor distance unit was cm. */
} RadarScan_Hit_t;
typedef struct {
  uint16_t counts[RADAR_MAP_CELL_COUNT];
  uint8_t hit_count[RADAR_MAP_CELL_COUNT];
  RadarScan_Hit_t hits[RADAR_MAP_CELL_COUNT][RADAR_EVIDENCE_HITS];
  uint32_t mask,raw_points,bad,restarts,unit_changes,unit_overflows;
  RadarScan_Unit_t unit;
} RadarScan_EvidenceCircle_t;
typedef struct {
  RadarMap_Params_t parameters;
  RadarScan_EvidenceCircle_t circle[3];
  uint32_t scan_id,map_id,origin_generation;
  uint8_t circles;
  bool complete;
} RadarScan_Evidence_t;
/** @brief Last completed manual capture's per-circle evidence, read-only.
 * New manual/auto capture invalidates this bounded buffer (not the old map).
 * Ordinary scans record in RAM only; no extra automatic wireless upload. */
const RadarScan_Evidence_t *RadarScan_Evidence(void);
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
/** @brief One stationary run of 1 or 3 complete circles, using a frozen corner reference.
 * Classify each circle independently, then OR its effective obstacles; publish
 * and plan once after all circles. Diagnostics sum counts; cloud is last circle.
 * @param circles 1 retains ordinary scan behavior; 3 improves sparse returns.
 * @return Accepted only; no chassis/arm movement or per-circle motor restart. */
bool RadarScan_StartAtCycles(const RadarMap_Params_t *lower_reference, uint32_t origin_generation,
                            RadarStart_Id_t start_id, uint8_t circles);
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
/** @brief 使用页面提供的冻结参数采集完整一圈；不发布实车地图或起点结果。 */
bool RadarScan_HostStart(const RadarMap_Params_t *params);
/** @brief 停止页面单圈或持续预览；保留最后完整主机结果。 */
void RadarScan_HostStop(void);
void RadarScan_HostStatusGet(RadarScan_HostStatus_t *status);
/** @return 仅当唯一点云缓存当前属于完整、未截断的主机圈时返回。 */
const RadarSample_t *RadarScan_HostPoints(uint16_t *count);
/** @brief 页面持续采集：所有原始点逐圈建图，120分区只作预览；不改实车状态。
 * @param params 冻结的雷达采集参数，距离mm，角度0.1度。
 * @param config 合法F1筛选/网格/规划配置，内部复制到RAM。
 * @return 已受理启动；完整结果由Process发布，不代表雷达已物理转动。
 */
bool RadarScan_HostLiveStart(const RadarMap_Params_t *params,const RadarHostMap_Config_t *config);
/** @brief 更新持续建图的RAM配置并重采当前不完整圈，不启动/停止雷达。
 * @param config 合法新配置；仅在主机持续采集活动时生效。
 */
void RadarScan_HostLiveConfigure(const RadarHostMap_Config_t *config);
/** @brief 返回最近完整持续圈的F1地图及路线，不提供半圈结果。
 * @param scan_id 可选输出，该完整结果绑定的圈号。
 * @return 内部只读结果，下一完整圈可覆盖；无匹配完整圈时NULL。
 */
const RadarHostMap_Result_t *RadarScan_HostLiveMap(uint32_t *scan_id);
void RadarScan_HostLiveStop(void);
bool RadarScan_HostLiveCopy(RadarLive_Point_t points[RADAR_LIVE_BINS]);
/** @brief Update the next capture's RAM parameters; never starts or clears a map. */
bool RadarScan_SetParam(const char *key, float value);
const RadarMap_Params_t *RadarScan_Params(void);
#endif
