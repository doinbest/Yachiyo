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
#define RADAR_MAP_REQUIRED_CLEAR_MASK ((1UL<<14)|(1UL<<22))
#define RADAR_MAP_TASK_CLEAR_MASK ((1UL<<2)|(1UL<<10)|(1UL<<14)|(1UL<<22))
#define RADAR_MAP_ALL_CELLS        0x01FFFFFFUL
#define RADAR_PLAN_MAX_POINTS      193U
#define RADAR_PLAN_MISSION_VISITS  9U
#define RADAR_GRID_BOUNDARY_COUNT  6U

typedef struct {
  int16_t x_mm[RADAR_GRID_BOUNDARY_COUNT];
  int16_t y_mm[RADAR_GRID_BOUNDARY_COUNT];
} RadarGrid_t;

typedef enum {
  RADAR_START_UNKNOWN=0,
  RADAR_START_UPPER=1,
  RADAR_START_LOWER=2
} RadarStart_Id_t;

typedef struct {
  float x_mm, y_mm, yaw_deg; /**< 车辆名义地图位姿；mm、从+X逆时针deg。 */
} RadarStart_Pose_t;

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
  uint16_t threshold;          /**< 筛选后每格判障最少点数；完整云与流式Add一致，至少1。 */
  uint8_t energy_min, energy_max;
} RadarMap_Params_t;

/** @brief One contiguous return on a known physical field edge. */
typedef struct {
  uint16_t counts[RADAR_MAP_CELL_COUNT], hits, outside_hits;
  float tangent_min_mm, tangent_max_mm;
  float first_x_mm, first_y_mm, last_x_mm, last_y_mm;
  uint8_t edge;
} RadarMap_BoundaryRun_t;

typedef struct {
  uint32_t mask; /**< 低25位，bit=r*5+c；r从下向上、c从左向右，均0..4。 */
  uint16_t counts[RADAR_MAP_CELL_COUNT]; /**< 同顺序点数；达到65535后饱和。 */
  uint32_t evidence_mask; /**< 格内点数达到threshold的原始扫描证据，未应用工位豁免。 */
  uint16_t outside_points; /**< 筛选后落在正式网格之外的点数。 */
  uint16_t background_points; /**< 有场外证据的场边回波数；counts/evidence仍保留原始点数。 */
  uint16_t accepted_points; /**< 正式150..2250网格真实点数；半开边界按F1参考方向旋转。 */
  bool cloud_built; /**< 已由完整云重建；不改变逐格点数判障规则。 */
  RadarMap_BoundaryRun_t boundary_first, boundary_pending;
  uint16_t boundary_counts[RADAR_MAP_CELL_COUNT]; /**< 已确认连续场边命中，不改原始counts。 */
  bool boundary_pending_is_first; /**< 首尾连续回波在完整圈结束时合并。 */
} RadarMap_t;

typedef struct {
  int16_t x_mm, y_mm;
  uint8_t station; /**< 0=中转点，1=起点，2=粗工，3=暂存，4=原料，5=扫码。 */
  uint8_t visit;   /**< station非0时为任务序号0..8；不依赖中转点数量。 */
} RadarPlan_Point_t;

typedef struct {
  uint16_t count;
  uint8_t failed_leg; /* 失败工位段1..8；0表示没有失败或输入坐标无效。 */
  uint8_t start_id; /**< 任务起点：0=无任务起点，1=右上，2=右下。 */
  bool valid;
  RadarPlan_Point_t points[RADAR_PLAN_MAX_POINTS];
} RadarPlan_t;

/** @brief 载入正式5×5判障边界：150,550,1000,1400,1850,2250 mm；物理场地仍0..2400。 */
void RadarGrid_Default(RadarGrid_t *grid);
/** @return X/Y均为150/2250或历史0/2400外边界，且格宽、高至少50 mm时为true。 */
bool RadarGrid_Valid(const RadarGrid_t *grid);
/** @brief 查找0基25格索引；endpoint允许坐标等于传入网格的最末边界。 */
int8_t RadarGrid_FindCell(const RadarGrid_t *grid,float x_mm,float y_mm,bool endpoint);
/** @brief 读取0基格中心坐标。 */
bool RadarGrid_CellCenter(const RadarGrid_t *grid,uint8_t cell,int16_t *x_mm,int16_t *y_mm);
/** @brief 在给定低25位障碍掩码上执行确定性的北、东、南、西四邻域A*。 */
bool RadarGrid_Path(const RadarGrid_t *grid,uint32_t blocked_mask,uint8_t start,uint8_t goal,
                    uint8_t path[RADAR_MAP_CELL_COUNT],uint8_t *count);

/** @brief 载入下起点区域中心(2250,150,180°)、0..360°和每格1点判障阈值。 */
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
/** @return 通过筛选并计入正式150..2250格点数时为true。
 * @note 参考半开边界经T旋转：150<X<=2250、150<=Y<2250；内部X边界归左格。
 */
bool RadarMap_Add(RadarMap_t *map, const RadarMap_Params_t *params,
                  const RadarSample_t *sample);
/** @brief 用同圈按扫描顺序的完整云重新建图；缓存首点可在圈内任意位置。
 * 按角度、距离、能量筛选后逐点投影并累计所在格；不做通用轮廓背景删除。
 * 每格独立达到threshold后判障，不向相邻格借用点数。
 * 场外点计入outside_points，并为贴近已知场边的长回波提供边界证据。
 * 结束调用FinishRaw保留前方禁障格证据；调用者决定是否再Finish。
 * @param count 完整云点数；截断/未完整的云不能替代原流式扫描结果。
 */
void RadarMap_BuildCloud(RadarMap_t *map, const RadarMap_Params_t *params,
                         const RadarSample_t *samples, uint16_t count);
/** @brief 按每格点数阈值生成证据；OR固定四障碍，保留禁障格。 */
void RadarMap_FinishRaw(RadarMap_t *map, const RadarMap_Params_t *params);
/** @brief 生成扫描障碍，保留点数与固定障碍；保护粗加工/暂存/二维码/原料任务格。
 * 同圈场边回波须同时满足离已知边界<=40 mm、相邻回波间距<=150 mm、
 * 连续至少8点、跨度>=400 mm、有法向场外点才不计入有效障碍；
 * 150 mm外围带不参与判障；网格内独立点仍按threshold判障。
 * 雷达安装在右下/右上启停格时，另外保留该起点格通行；对侧角与其他工位不豁免。
 * 原始FinishRaw证据、人工输入mask及规划函数不应用这项扫描豁免。
 */
void RadarMap_Finish(RadarMap_t *map, const RadarMap_Params_t *params);
/** @brief 在当前图规划1→5→4→2→3→4→2→3→1；同向直线点合并。
 * 中转点station=0；业务站点station为1..5、visit为0..8；起点1/0。
 * 在F1参考方向共用四邻居A*后旋转格链，按原350/775/1200/1625/2050通道行走；
 * 工位仅在端点格内轴向接驳，不另选跨格捷径；所有实际线段判障。
 * 格路采用毫米曼哈顿代价。首段接驳可通行时
 * 经(2100,300)短斜移进入右侧通道，再沿X=2100到扫码；返程经同接驳点
 * 斜向进入原起点。首段两条实际线段均检查障碍，受阻则保留格路绕行。
 * valid表示格路可达，车体旋转扫掠净空仍由离线诊断及实车验收核对。
 * @return 完整可达为true；失败时valid=false、count=0并报告failed_leg。
 */
bool RadarPlan_Build(const RadarMap_t *map, RadarPlan_t *plan);
/** @brief 同一业务路线按右上/右下起点规划并返回该起点；未知起点不规划。
 * 右上首段/返程经(2100,2100)，右下经(2100,300)；手动map.mask仍为权威障碍。
 */
bool RadarPlan_BuildFromStart(const RadarMap_t *map, RadarStart_Id_t start_id,
                              RadarPlan_t *plan);
/** @brief 从任意当前图位置规划一段；沿旋转后的F1四邻居格链及局部轴向接驳，单位mm。
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
