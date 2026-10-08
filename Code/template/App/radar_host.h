/**
 * @file radar_host.h
 * @brief F407网页雷达配置事务、扫描编排及版本化地图/路径快照。
 */
#ifndef RADAR_HOST_H
#define RADAR_HOST_H

#include "radar_host_map.h"
#include "radar_scan.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t session_id,config_id,draft_revision;
  uint32_t scan_id,map_id,plan_id;
  uint32_t preview_id; /* 独立完整预览圈编号；live stop/start不复用。 */
  RadarScan_State_t scan_state;
  RadarScan_LiveState_t live_state;
  const char *reason;
  uint32_t raw_points,filtered_points,circle_restarts,unit_changes,unit_overflows;
  uint16_t cloud_points,preview_points;
  bool active,live_active,map_valid,path_valid,cloud_valid,historical;
} RadarHost_Status_t;

void RadarHost_Init(void);
void RadarHost_Process(void);
bool RadarHost_StartRotation(void);
void RadarHost_Stop(void);
/** @brief Capture one circle; replace host live mode without changing vehicle work.
 *  @return true when accepted; false while another capture owns the radar or UART is unavailable.
 */
bool RadarHost_Scan(void);
bool RadarHost_LiveStart(void);
void RadarHost_LiveStop(void);
void RadarHost_StatusGet(RadarHost_Status_t *status);
const RadarHostMap_Config_t *RadarHost_ActiveConfig(void);
const RadarHostMap_Config_t *RadarHost_DraftConfig(void);
uint32_t RadarHost_DraftReset(void);
bool RadarHost_DraftSet(const char *key,const char *value);
bool RadarHost_Apply(uint32_t draft_revision);
const RadarHostMap_Result_t *RadarHost_Map(uint32_t *map_id,uint32_t *scan_id);
const RadarHostMap_Result_t *RadarHost_Path(uint32_t *plan_id,uint32_t *scan_id);
/**
 * @brief 读取最新完整120分区预览；发送多页前由控制台复制为冻结快照。
 * @param count 返回有效点数；允许NULL。
 * @param preview_id 返回独立非零圈编号，同次MCU运行内递增；允许NULL。
 * @retval NULL 尚无完整圈；其他 返回最新圈缓存，下一次Process可能更新。
 * @note 停止/重新开始不清除历史圈或重置圈编号；不代表静态建图scan_id。
 */
const RadarLive_Point_t *RadarHost_Preview(uint16_t *count,uint32_t *preview_id);
const RadarSample_t *RadarHost_Cloud(uint16_t *count,uint32_t *scan_id);

#ifdef __cplusplus
}
#endif
#endif
