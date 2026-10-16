#ifndef GRAB_TRACKING_H
#define GRAB_TRACKING_H
#include <stdbool.h>
#include <stdint.h>

#define GRAB_TRACKING_SAMPLES 32U
#define GRAB_TRACKING_TRACE 128U

typedef enum { GRAB_TRACK_PREDICT, GRAB_TRACK_VISUAL, GRAB_TRACK_FEEDBACK } GrabTracking_Mode_t;

typedef struct
{
  float body_speed_mm_s, x_speed_mm_s, accel_mm_s2, lambda_s;
  float x_weight, lead_mm, tolerance_px;
  uint32_t control_ms, batch_ms, latency_ms, fit_ms, horizon_ms, grace_ms;
  float body_accel_mm_s2, x_accel_mm_s2; /* feedback-mode software response, not Emm grades */
  GrabTracking_Mode_t mode;
} GrabTracking_Config_t;

typedef struct
{
  uint32_t now_ms, sample_ms, sequence;
  uint32_t received_ms;
  bool receive_time_valid;
  bool valid, actuating;
  float error_px[2], j[2][3];
  float motion_mm[3]; /* Control motion: measured in visual/feedback, estimated only in predict mode. */
  float body_left_mm; /* Measured local left displacement only; owns the 50 mm budget in every mode. */
  float actual_mm_s[3], x_mm, x_min_mm, x_max_mm;
  uint32_t feedback_age_ms, batch_interval_ms;
  uint32_t request_ms, sent_ms;
} GrabTracking_Input_t;

typedef struct
{
  float target_px_s[2], target_px_s2[2], predicted_px[2];
  float feedforward_mm_s[3], correction_mm_s[3], velocity_mm_s[3];
  float x_target_mm, body_room_mm, x_room_mm;
  bool predicting, visual_pause, window_end, stable, model_valid;
} GrabTracking_Output_t;

typedef struct
{
  uint32_t time_ms, sample_ms, sequence;
  uint16_t feedback_age_ms, batch_interval_ms, request_age_ms, sent_age_ms;
  uint8_t flags; /* 1=fresh observation, 2=motion mode, 4=stable, 8=predicting */
  uint8_t mode; /* retained with this row, including frozen exports */
  float error_px[2], target_px_s[2], actual_mm_s[3], command_mm_s[3];
  float x_mm, forward_mm, left_mm, body_room_mm;
} GrabTracking_Record_t;

typedef struct
{
  uint32_t time_ms;
  float value[3];
} GrabTracking_Sample_t;

typedef struct
{
  uint32_t count, maximum_ms, bins[201]; /* bin 200 includes >=200ms */
} GrabTracking_Timing_t;

typedef struct
{
  GrabTracking_Sample_t samples_data[GRAB_TRACKING_SAMPLES], motion[GRAB_TRACKING_SAMPLES];
  unsigned samples, motions;
  uint32_t last_sequence, last_sample_ms, last_update_ms, stable_since, visual_interval_ms;
  bool have_sample, have_update, stable_pending, reference_valid, frozen;
  float reference_left_mm, target_px_s[2], target_px_s2[2], previous_mm_s[3];
  float raw_velocity_px_s[2], raw_accel_px_s2[2];
  float velocity_confidence[2];
  float last_error_px[2], last_motion_mm[3];
  float last_compensated_px[2];
  double error_square[2];
  uint32_t error_count, error_bins[2][101], latency_count;
  double latency_square[3]; /* receive prediction residuals at 0 / 50 / 100 ms */
  float prior_target[2], prior_velocity[2], prior_accel[2];
  uint32_t prior_ms;
  GrabTracking_Record_t records[GRAB_TRACKING_TRACE];
  unsigned record_head, record_count;
  uint32_t visual_updates, timer_updates, control_sequence, last_rx_ms, last_control_ms;
  GrabTracking_Timing_t control_timing, prediction_timing, visual_delay, computation_timing;
} GrabTracking_t;

/** @brief Read percentile from all-event MCU-ms histogram; final bin is saturated. */
unsigned GrabTracking_Percentile(const GrabTracking_Timing_t *timing,unsigned percent);

/** @brief 取得用户确认的追踪初值；不覆盖静态取放配置。
 * @param config 输出配置，速度mm/s，加速度mm/s²，延时ms。
 * @retval 无
 */
void GrabTracking_Defaults(GrabTracking_Config_t *config);
/** @brief 取得指定模式的初值；visual/feedback采用各自联调初值。 */
void GrabTracking_DefaultsForMode(GrabTracking_Config_t *config,GrabTracking_Mode_t mode);
/** @brief 清空追踪实例，未建立料台参考。
 * @param track 独立追踪状态。
 * @retval 无
 */
void GrabTracking_Init(GrabTracking_t *track);
/** @brief 人工停止时建立料台局部参考；唯有此操作更新50mm预算锚点。
 * @param track 独立状态。
 * @param left_mm 当前局部车体向左累计位移，mm（观察姿态朝料台）。
 * @retval 无
 */
void GrabTracking_Reference(GrabTracking_t *track,float left_mm);
/** @brief 开始一段记录，不清除料台参考、剩余预算或观察估计。
 * @param track 独立状态。
 * @retval 无
 */
void GrabTracking_Begin(GrabTracking_t *track);
/** Clear estimation history on a stopped mode change; retain reference and records. */
void GrabTracking_ResetModel(GrabTracking_t *track);
/** @brief 原模式拟合目标运动；验证模式按新视觉纠偏，可选反馈减速。
 * @param track 独立状态，调用期间持续保留。
 * @param config 独立追踪参数。
 * @param input 本次反馈/视觉快照；新测量按sequence及sample_ms去重。
 * @param output 输出预测、速度和暂停原因；本函数不发送硬件命令。
 * @retval 无
 * @note 误差直接使用B2 DX/DY（右/上为正），共用J；预测延迟/反馈加减速为联调初值。
 */
void GrabTracking_Update(GrabTracking_t *track,const GrabTracking_Config_t *config,
                         const GrabTracking_Input_t *input,GrabTracking_Output_t *output);
/** @brief 按时间顺序取得停止后冻结记录。
 * @param track 状态。
 * @param index 0为保留的最早记录。
 * @retval 记录指针；越界为NULL。
 */
const GrabTracking_Record_t *GrabTracking_Record(const GrabTracking_t *track,unsigned index);
#endif
