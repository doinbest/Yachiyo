#ifndef CHASSIS_MODEL_H
#define CHASSIS_MODEL_H
#include <stdbool.h>
#include <stdint.h>
#define CHASSIS_MODEL_PI 3.14159265358979323846f
/* Units: mm, s, rad, signed logical motor RPM; array order FL, RL, RR, FR. */
typedef struct
{
  float wheelbase_mm, track_mm, wheel_diameter_mm, gear_ratio;
} ChassisModel_Geometry_t;
typedef struct
{
  float vx_mm_s, vy_mm_s, omega_rad_s;
} ChassisModel_Velocity_t;
typedef struct
{
  float rpm[4];
  int16_t command[4];
  float scale;
} ChassisModel_Wheels_t;
typedef struct
{
  float x_mm, y_mm, yaw_rad;
} ChassisModel_Pose_t;
/** @brief X型麦轮逆解；组合后按最大RPM整组缩放，再四舍五入整数RPM。
 * @param max_rpm 1..32767，正有限值。
 * @param out rpm为缩放后未量化值；command为整数值；scale为共同缩放比例。
 * @return 输入和几何有效返回true；失败不修改输出。
 */
bool ChassisModel_Inverse(const ChassisModel_Geometry_t *g, const ChassisModel_Velocity_t *v,
                          float max_rpm, ChassisModel_Wheels_t *out);
/** @brief 带逻辑前向正负号的电机RPM正解；输入目标值时输出仅为模型推算。 */
bool ChassisModel_Forward(const ChassisModel_Geometry_t *g, const float rpm[4],
                          ChassisModel_Velocity_t *out);
/** @brief 车体速度/点向量旋转至地图；yaw从地图+X逆时针计rad，不含平移。 */
bool ChassisModel_BodyToMap(float vx, float vy, float yaw, float *map_x, float *map_y);
/** @brief 地图速度/点向量旋转至车体；yaw单位rad，不含平移。 */
bool ChassisModel_MapToBody(float map_x, float map_y, float yaw, float *vx, float *vy);
/** @brief 用中点航向作一次理想积分；dt单位秒且>=0，长间隔由调用者拒绝。
 * @return 失败不修改pose；成功也不等于实物定位。
 */
bool ChassisModel_IntegrateMidpoint(ChassisModel_Pose_t *pose, const ChassisModel_Velocity_t *v,
                                    float dt_s);
#endif
