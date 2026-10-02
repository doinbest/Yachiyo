#ifndef HWT101_CALIBRATION_H
#define HWT101_CALIBRATION_H

#include "hwt101_i2c.h"

typedef enum
{
  HWT101_CAL_IDLE, HWT101_CAL_PREPARING, HWT101_CAL_CALIBRATING,
  HWT101_CAL_RESTORING, HWT101_CAL_VERIFYING, HWT101_CAL_SAVING,
  HWT101_CAL_DONE, HWT101_CAL_FAILED, HWT101_CAL_CANCELLED
} HWT101_CalState_t;

typedef struct
{
  HWT101_CalState_t state;
  const char *state_name, *reason, *result, *save_state;
  bool busy, verified, control_ready, normal_mode_confirmed;
  bool registers_valid, bias_after_valid, save_requested, save_readback_ok;
  uint16_t version, mode, bias_before, bias_after;
  HAL_StatusTypeDef hal;
  uint32_t run_id, elapsed_ms, total_ms, verify_ms, verify_elapsed_ms;
  uint32_t samples, gap_ms, max_gap_ms, i2c_errors;
  float relative_deg, drift_dps, rms_deg;
  bool zero_requested, zero_sample_received;
  float zero_before_deg, zero_after_deg;
} HWT101_CalStatus_t;

/** @brief 初始化本次启动的状态，不写模块或外部Flash。主循环初始化时调用。 */
void HWT101_Cal_Init(void);
/** @brief 初始化结束后安排一次上电5秒静止验证；等待新鲜数据期间也阻止运动。 */
void HWT101_Cal_BootVerifyArm(void);
/** @brief HWT101_Process之后调用；motion_idle须覆盖所有运动任务，等待数据最多5秒。 */
void HWT101_Cal_BootVerifyProcess(bool motion_idle);
/** @brief 触发20秒原生标定、30秒验证，通过后请求模块内部保存。
 * @pre 调用者须确认所有运动任务空闲，用户保证整车静止；只在主循环调用。
 * @return 已开始为true；忙、无新鲜样本或寄存器读取异常为false。 */
bool HWT101_Cal_Start(void);
/** @brief 独立Z轴归零：解锁、写CALIYAW、等待新样本；不标定、不保存。
 * @pre 所有运动空闲且整车静止；撤销本次航向验证及地图锚点资格。
 * @return true仅表示流程开始；完成后检查zero_sample_received/zero_after_deg。 */
bool HWT101_Cal_ZeroStart(void);
/** @brief 先解锁并写CALIYAW归零，再验证；不修改零偏或发送SAVE。
 * duration_ms仅支持5000或30000（内部测试）；控制台固定5秒，为归零后的完整采样时间。
 * @pre 同Start；通过后仅授予本次启动的航向测试资格。 */
bool HWT101_Cal_VerifyStart(uint32_t duration_ms);
/** @brief 取消并清除RAM验证资格；必要时异步恢复正常模式，不擦除零偏。 */
void HWT101_Cal_Cancel(void);
/** @brief HWT101_Process之后在主循环调用；无长时间阻塞等待。 */
void HWT101_Cal_Process(void);
/** @brief 获取缓存状态，失败不写输出；不访问I2C。 */
bool HWT101_Cal_GetStatus(HWT101_CalStatus_t *status);
/** @brief 当前是否正在标定、验证、保存或恢复模式。 */
bool HWT101_Cal_IsBusy(void);
/** @brief 本次验证通过且数据新鲜时返回模块角度，不进行外部漂移扣除。
 * @return 失败不修改angle；运行中的通信故障会撤销资格。 */
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *angle);
/** @brief 空闲时读取版本/模式/零偏供status诊断；不写寄存器。
 * @return HAL_BUSY表示流程运行中；读取失败保留数值并清除有效标志。 */
HAL_StatusTypeDef HWT101_Cal_RefreshRegisters(void);

#endif
