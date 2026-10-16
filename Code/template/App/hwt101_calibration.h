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
  bool read_recovering;
  uint32_t recovery_count, recovery_elapsed_ms;
  /* Last recovery's evidence survives normal-mode restoration and status reads. */
  const char *recovery_step;
  uint16_t recovery_expected_mode, recovery_mode_mask, recovery_mode_first, recovery_mode_last;
  bool recovery_mode_seen, recovery_mode_valid;
  HAL_StatusTypeDef recovery_mode_hal;
  uint32_t recovery_mode_reads, recovery_mode_tick;
  bool timing_valid;
  uint16_t peak_time_s, bias_time_s;
  uint8_t native_modes_seen;
  uint32_t native_wait_ms, native_normal_ms;
  float relative_deg, drift_dps, rms_deg;
  bool zero_requested, zero_sample_received;
  float zero_before_deg, zero_after_deg;
  const char *first_reason;
  HAL_StatusTypeDef first_hal;
  uint32_t first_fault_ms, first_hal_error, first_hal_state;
  uint8_t first_operation, first_register;
} HWT101_CalStatus_t;

/** @brief 初始化本次启动的状态，不写模块或外部Flash。主循环初始化时调用。 */
void HWT101_Cal_Init(void);
/** @brief 初始化结束后安排一次上电5秒静止验证；等待新鲜数据期间也阻止运动。 */
void HWT101_Cal_BootVerifyArm(void);
/** @brief HWT101_Process之后调用；motion_idle须覆盖所有运动任务，等待数据最多5秒。 */
void HWT101_Cal_BootVerifyProcess(bool motion_idle);
/** @brief 原生标定等待至少20秒及读回的两段时长之和，再验证30秒，通过后保存。
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
/** @brief HWT101_Process之后在主循环调用；读取中断最多等待5秒恢复。
 * 恢复须有两个新鲜更新及模式读回；验证窗口重采，不重放原生标定或归零。
 * 启动首次成功但异常的模式读回也计入复核证据，至少100 ms后只读确认；
 * 恢复为1/2直接续接原生等待，不重启计时或同轮重复启动确认。
 * SAVE发送后的模式/零偏读回共用首次读取异常起5秒窗口，只续接读取，
 * 保留已完成验证，不重发SAVE；确认失败总体FAIL，verified仍描述测量结果。 */
void HWT101_Cal_Process(void);
/** @brief 获取缓存状态，失败不写输出；不访问I2C。 */
bool HWT101_Cal_GetStatus(HWT101_CalStatus_t *status);
/** @brief 当前是否正在标定、验证、保存或恢复模式。 */
bool HWT101_Cal_IsBusy(void);
/** @brief 本次验证通过且数据新鲜时返回模块角度，不进行外部漂移扣除。
 * @return 失败不修改angle；运行中通信异常暂停使用，通过后连续两帧新鲜数据可恢复。
 * @note 不访问I2C；诊断读取中断由Process只读复核正常模式后重取两帧。
 * 恢复不归零、不保存、不重启运动；验证失败、取消或已确认异常模式仍不可用。 */
bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *angle);
/** @brief 空闲时读取版本/模式/零偏及两段时长供status诊断；不写寄存器。
 * 两段时长为可选诊断，失败时timing_valid=false并使用20秒等待下限。
 * VERSION/MODE/BIAS读取失败暂挂控制资格；未知模式由Process自动只读复核，
 * 已读到的异常MODE不会因后续诊断读取失败降级为未知或自动放行。
 * @return HAL_BUSY表示流程运行中；失败保留未读回值并清除诊断有效标志。 */
HAL_StatusTypeDef HWT101_Cal_RefreshRegisters(void);

#endif
