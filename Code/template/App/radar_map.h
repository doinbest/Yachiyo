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
  uint16_t threshold;          /**< 完整云按同物体点数判障；低级Add按同格点数；至少1。 */
  uint8_t energy_min, energy_max;
} RadarMap_Params_t;

typedef struct {
  uint32_t mask; /**< 低25位，bit=r*5+c；r从下向上、c从左向右，均0..4。 */
  uint16_t counts[RADAR_MAP_CELL_COUNT]; /**< 同顺序点数；达到65535后饱和。 */
  uint32_t evidence_mask; /**< 完整云中达到物体阈值的全部触点格，点数不跨格伪增。 */
  uint16_t outside_points; /**< 筛选后落在2400mm方场外的点数；可与background重叠。 */
  uint16_t background_points; /**< 被场外连通长轮廓判据移除的点数，含该轮廓场外点。 */
  uint16_t accepted_points; /**< 保留在[150,2250)活动网格内的真实点数。 */
  bool cloud_built; /**< 完整云按物体证据判障；低级Add仍按同格计数。 */
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

/** @brief 载入当前赛事5×5网格边界：150,550,1000,1400,1850,2250 mm。 */
void RadarGrid_Default(RadarGrid_t *grid);
/** @return 外边界为150/2250且每个格宽、高至少50 mm时为true。 */
bool RadarGrid_Valid(const RadarGrid_t *grid);
/** @brief 查找0基25格索引；endpoint允许坐标等于最末边界2250。 */
int8_t RadarGrid_FindCell(const RadarGrid_t *grid,float x_mm,float y_mm,bool endpoint);
/** @brief 读取0基格中心坐标。 */
bool RadarGrid_CellCenter(const RadarGrid_t *grid,uint8_t cell,int16_t *x_mm,int16_t *y_mm);
/** @brief 在给定低25位障碍掩码上执行确定性的北、东、南、西四邻域A*。 */
bool RadarGrid_Path(const RadarGrid_t *grid,uint32_t blocked_mask,uint8_t start,uint8_t goal,
                    uint8_t path[RADAR_MAP_CELL_COUNT],uint8_t *count);

/** @brief 载入下起点区域中心(2250,150,180°)、顺时针0..90°和1点物体阈值。 */
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
/** @brief 用同圈完整、角度顺序的云重新建图；缓存首点可为任意角。
 * 按选域起点虚拟循环读取原云；不复制云，不改变距离/投影。
 * 连续物体整体达到threshold后标记所有触点格。仅去掉确有场外延伸的
 * 连续长轮廓；近短物遮挡仅在背景端部空间连续和局部共线时连接。
 * counts为保留点的真实逐格数量，单点噪声不能跨独立物体凑阈值。
 * 结束调用FinishRaw保留前方禁障格证据；调用者决定是否再Finish。
 * @param count 完整云点数；截断/未完整的云不能替代原流式扫描结果。
 */
void RadarMap_BuildCloud(RadarMap_t *map, const RadarMap_Params_t *params,
                         const RadarSample_t *samples, uint16_t count);
/** @brief 完整云用物体证据，低级Add用同格阈值；OR固定四障碍，保留禁障格。 */
void RadarMap_FinishRaw(RadarMap_t *map, const RadarMap_Params_t *params);
/** @brief 生成扫描障碍；只豁免二维码/原料台前方两格，点数与固定障碍保留。 */
void RadarMap_Finish(RadarMap_t *map, const RadarMap_Params_t *params);
/** @brief 在当前图规划1→5→4→2→3→4→2→3→1；同向直线点合并。
 * 中转点station=0；业务站点station为1..5、visit为0..8；起点1/0。
 * 格中心采用毫米曼哈顿代价；通常采用地图X/Y轴连接。首腿直连可通行时
 * 使用起点到扫码的斜线，返程经(2100,300)接驳后斜向进入原起点。
 * valid表示格路可达，车体旋转扫掠净空仍由离线诊断及实车验收核对。
 * @return 完整可达为true；失败时valid=false、count=0并报告failed_leg。
 */
bool RadarPlan_Build(const RadarMap_t *map, RadarPlan_t *plan);
/** @brief 同一业务路线按右上/右下起点规划并返回该起点；未知起点不规划。
 * 右上返程经(2100,2100)，右下经(2100,300)；手动map.mask仍为权威障碍。
 */
bool RadarPlan_BuildFromStart(const RadarMap_t *map, RadarStart_Id_t start_id,
                              RadarPlan_t *plan);
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
