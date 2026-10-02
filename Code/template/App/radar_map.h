/**
 * @file radar_map.h
 * @brief LDS50C 静止扫描建图与当前场地坐标下的工位路径规划；不依赖 HAL。
 */
#ifndef RADAR_MAP_H
#define RADAR_MAP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RADAR_MAP_CELL_COUNT       25U
#define RADAR_MAP_FIXED_MASK       0x00050140UL
#define RADAR_MAP_ALL_CELLS        0x01FFFFFFUL
#define RADAR_PLAN_MAX_POINTS      193U
#define RADAR_PLAN_MISSION_VISITS  9U

typedef struct {
  uint16_t angle_tenths; /**< 雷达局部顺时针角，0.1°；3600等价于0。 */
  uint16_t distance_mm;  /**< 已按雷达单位换算的距离，mm。 */
  uint8_t energy;        /**< 原始反射强度，0..255。 */
} RadarSample_t;

typedef struct {
  float lidar_x_mm, lidar_y_mm; /**< 雷达中心的当前地图坐标，mm。 */
  float zero_deg;              /**< 雷达局部角0从地图+X逆时针计的角度，deg。 */
  uint16_t distance_min_mm, distance_max_mm;
  uint16_t angle_min_tenths, angle_max_tenths; /**< 包含端点；min>max时跨0°。 */
  uint16_t threshold;          /**< 同格计数达到此值后阻塞；至少1。 */
  uint8_t energy_min, energy_max;
} RadarMap_Params_t;

typedef struct {
  uint32_t mask; /**< 低25位，bit=r*5+c；r从下向上、c从左向右，均0..4。 */
  uint16_t counts[RADAR_MAP_CELL_COUNT]; /**< 同顺序点数；达到65535后饱和。 */
} RadarMap_t;

typedef struct {
  int16_t x_mm, y_mm;
  uint8_t station; /**< 0=中转点，1=起点，2=粗工，3=暂存，4=原料，5=扫码。 */
  uint8_t visit;   /**< station非0时为任务序号0..8；不依赖中转点数量。 */
} RadarPlan_Point_t;

typedef struct {
  uint16_t count;
  uint8_t failed_leg; /* 失败工位段1..8；0表示没有失败或输入坐标无效。 */
  bool valid;
  RadarPlan_Point_t points[RADAR_PLAN_MAX_POINTS];
} RadarPlan_t;

/** @brief 载入参考扫描预设(2170,230,180°)，不表示安装位置已实测。 */
void RadarMap_Defaults(RadarMap_Params_t *params);
/** @return 数值有限、筛选范围合法且阈值至少1时为true。 */
bool RadarMap_ParamsValid(const RadarMap_Params_t *params);
/** @brief 清点数，保留固定四障碍；开始新圈时使用。 */
void RadarMap_Reset(RadarMap_t *map);
/** @brief 将局部顺时针角投影到地图，角零线从地图+X逆时针计deg。
 * @return 投影成功为true；不在此函数应用筛选范围。
 */
bool RadarMap_Project(const RadarMap_Params_t *params, const RadarSample_t *sample,
                      float *x_mm, float *y_mm);
/** @return 通过筛选且位于[150,2250)活动区、已计入格点数时为true。 */
bool RadarMap_Add(RadarMap_t *map, const RadarMap_Params_t *params,
                  const RadarSample_t *sample);
/** @brief 由点数阈值生成扫描障碍并OR固定四障碍；不豁免整个任务格。 */
void RadarMap_Finish(RadarMap_t *map, const RadarMap_Params_t *params);
/** @brief 在当前图规划1→5→4→2→3→4→2→3→1；同向直线点合并。
 * 中转点station=0；业务站点station为1..5、visit为0..8；起点1/0。
 * 格中心采用毫米曼哈顿代价；格内连接和输出路段均沿地图X/Y轴。
 * @return 完整可达为true；失败时valid=false、count=0并报告failed_leg。
 */
bool RadarPlan_Build(const RadarMap_t *map, RadarPlan_t *plan);
/** @brief 从任意当前图位置规划一段；四邻居格路与格内轴向连接，单位mm。
 * @return 可达为true；结果station/visit均为0，不触发业务工位。
 */
bool RadarPlan_BuildLeg(const RadarMap_t *map, float start_x_mm, float start_y_mm,
                        float end_x_mm, float end_y_mm, RadarPlan_t *plan);
/** @brief 导入历史图点：绕场地中心逆时针90°，T(x,y)=(2400-y,x)。 */
void RadarMap_LegacyPoint(float old_x_mm, float old_y_mm, float *x_mm, float *y_mm);
/** @brief 历史25位格掩码旋转：r'=c,c'=6-r，保留低25位。 */
uint32_t RadarMap_LegacyMask(uint32_t old_mask);

#ifdef __cplusplus
}
#endif
#endif
