/**
 * @file radar_start.h
 * @brief 双起点假设的静止雷达分类；仅使用赛事保证无障碍的两个前方格。
 */
#ifndef RADAR_START_H
#define RADAR_START_H

#include "radar_map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  RadarMap_Params_t params[2]; /**< 0=右上，1=右下；同一筛选条件。 */
  RadarMap_t maps[2];         /**< 流式累计全部扫描样本，无点云缓存截断。 */
} RadarStart_Context_t;

typedef struct {
  uint8_t start_id;           /**< 0=待判定，1=右上，2=右下。 */
  const char *reason;        /**< clear_regions、ambiguous或both_conflict。 */
  uint32_t conflict_mask[2]; /**< 原始阈值障碍与前方禁障两格的交集。 */
  uint16_t qr_counts[2], raw_counts[2]; /**< 保留每个假设的原始点数。 */
} RadarStart_Result_t;

/** @brief 从下方引用参数推导静态角点扫描参数，不改输入或车辆参考。
 * @param lower_reference 下方雷达XY(mm)、世界零角(度)和筛选条件。
 * @param start_id 上方或下方；g4上方独立可调XY、270°和单位矩阵，下方沿设备参数。
 * 历史g3仍按原镜像约定解释。
 * @param out 输出参数，可与输入同地址；筛选条件原样保留。
 * @return 非空输入/输出且起点有效时为true；未知起点返回false。 */
bool RadarStart_ParamsForStart(const RadarMap_Params_t *lower_reference,
                               RadarStart_Id_t start_id, RadarMap_Params_t *out);
/** @brief 清累计图；推导两个静态角点参考，NULL输入使用默认参数。 */
void RadarStart_Begin(RadarStart_Context_t *context,
                      const RadarMap_Params_t *lower_reference);
/** @brief 向两个起点假设计入一个已换算为mm、0.1°的原始样本。 */
void RadarStart_Add(RadarStart_Context_t *context, const RadarSample_t *sample);
/** @brief 仅当两个假设中恰好一个无冲突时选定起点；不自动选默认值。 */
void RadarStart_Finish(RadarStart_Context_t *context, RadarStart_Result_t *result);
/** @return 所选假设图；未知起点/空上下文返回NULL。 */
const RadarMap_t *RadarStart_Map(const RadarStart_Context_t *context, RadarStart_Id_t start_id);
/** @return 对应假设的雷达参考与筛选参数；未知起点/空上下文返回NULL。 */
const RadarMap_Params_t *RadarStart_Params(const RadarStart_Context_t *context,
                                         RadarStart_Id_t start_id);
/** @return 起点名义车辆位姿；未知起点返回NULL，不代表实测定位。 */
const RadarStart_Pose_t *RadarStart_Pose(RadarStart_Id_t start_id);

#ifdef __cplusplus
}
#endif
#endif
