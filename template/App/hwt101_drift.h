#ifndef HWT101_DRIFT_H
#define HWT101_DRIFT_H

#include "hwt101_i2c.h"

typedef enum
{
  HWT101_DRIFT_IDLE,
  HWT101_DRIFT_CALIBRATING,
  HWT101_DRIFT_VERIFYING,
  HWT101_DRIFT_DONE,
  HWT101_DRIFT_FAILED
} HWT101_DriftState_t;

typedef struct
{
  HWT101_DriftState_t state;
  const char *state_name;
  const char *reason;
  bool compensation_valid;
  bool verification_passed;
  bool restored_from_flash; /* DONE/PASS来自历史验证；本次启动没有重新验证。 */
  uint32_t elapsed_ms;
  uint32_t samples;
  uint32_t gap_ms;     /* 最近新数据距上个标定样本的间隔；失败后保留。 */
  uint32_t max_gap_ms; /* 本次标定开始以来的最大间隔，包含超限间隔。 */
  float raw_yaw_deg;
  float relative_raw_deg; /* 标定完成后，展开后的原始角度变化。 */
  float corrected_deg;    /* 仅 compensation_valid 为 true 时有效，范围 [-180,180)。 */
  float bias_dps;
  float half_bias_dps[2];
  float calibration_rms_deg;
  float raw_drift_dps;    /* 独立验证窗口的拟合斜率。 */
  float residual_dps;
  float validation_rms_deg;
} HWT101_DriftSnapshot_t;

/** @brief 开始 60 秒静止标定，随后自动验证 120 秒。
 * @return 已开始返回 true；忙、无新鲜数据或设备未就绪返回 false。
 * @pre 模块已预热，用户保证整个 180 秒内整车静止。只在主循环调用。
 * @note 不驱动电机，不写模块寄存器或 Flash。新标定撤销旧补偿。
 */
bool HWT101_Drift_Start(void);

/** @brief 取消标定并清除 RAM 中的补偿参数；不改变原始角度。 */
void HWT101_Drift_Clear(void);

/** @brief 在 HWT101_Process() 后调用；只读取缓存，非阻塞处理标定和验证。
 * @note 通信错误或采样中断令补偿失效；不在运动期间更新偏置。
 */
void HWT101_Drift_Process(void);

/** @brief 获取诊断快照，不访问 I2C。
 * @param snapshot 输出地址。
 * @return 非空地址返回 true，否则 false。
 * @note VERIFYING 时补偿仅供观察，DONE 才表示独立静止验证通过。
 */
bool HWT101_Drift_Get(HWT101_DriftSnapshot_t *snapshot);

/** @brief 获取供航向控制使用的最新补偿角度，只读取缓存。
 * @param angle 输出yaw（deg，[-180,180)）及对应原始样本时间和计数。
 * @return 仅DONE/PASS且通信、数据时效有效时返回true；失败不修改输出。
 * @note 不使用VERIFYING暂定值，不回退原始角度。主循环先采集并处理漂移。
 */
bool HWT101_Drift_ControlAngleGet(HWT101_Angle_t *angle);
/** @brief 从已校验记录恢复偏置，以当前新鲜角度重新建立零点。
 * @return 仅IDLE且样本新鲜、偏置有限且绝对值不超过5deg/s时成功。
 * @note DONE表示允许补偿控制，restored_from_flash区分历史验证与本次验证。 */
bool HWT101_Drift_Restore(float bias_dps);

#endif
