#include "hwt101_drift.h"
#include <math.h>
#include <string.h>

#define DRIFT_SAMPLE_MS 100U
#define DRIFT_CALIBRATION_MS 60000U
#define DRIFT_HALF_MS 30000U
#define DRIFT_VALIDATION_MS 120000U
/* 第一版实验阈值，需结合原始日志评估，不能作为模块规格。 */
#define DRIFT_MAX_HALF_DIFFERENCE_DPS 0.02f
#define DRIFT_MAX_CALIBRATION_RMS_DEG 0.1f
#define DRIFT_MAX_RESIDUAL_DPS 0.01f
#define DRIFT_MAX_STATIC_STEP_DEG 5.0f

/* 累积最小二乘统计量，不保存全部样本；double 减小残差相减误差。 */
typedef struct
{
  uint32_t n;
  double t, y, tt, ty, yy;
} DriftFit_t;

static HWT101_DriftSnapshot_t drift;
static DriftFit_t total_fit, half_fit[2], validation_fit;
static uint32_t start_tick, validation_tick, sample_tick, sample_count, error_count;
static float last_yaw;
static double unwrapped, reference;

static float Drift_Wrap(float angle)
{
  while (angle >= 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static void Drift_FitAdd(DriftFit_t *fit, double t, double y)
{
  fit->n++;
  fit->t += t; fit->y += y;
  fit->tt += t * t; fit->ty += t * y; fit->yy += y * y;
}

static bool Drift_FitResult(const DriftFit_t *fit, float *slope, float *rms)
{
  double centered_tt, centered_ty, centered_yy, residual;
  if (fit->n < 2U) return false;
  centered_tt = fit->tt - fit->t * fit->t / fit->n;
  if (centered_tt <= 0.0) return false;
  centered_ty = fit->ty - fit->t * fit->y / fit->n;
  centered_yy = fit->yy - fit->y * fit->y / fit->n;
  *slope = (float)(centered_ty / centered_tt);
  residual = centered_yy - centered_ty * centered_ty / centered_tt;
  if (residual < 0.0) residual = 0.0;
  *rms = (float)sqrt(residual / fit->n);
  return true;
}

static void Drift_Fail(const char *reason)
{
  drift.state = HWT101_DRIFT_FAILED;
  drift.reason = reason;
  drift.compensation_valid = false;
  drift.verification_passed = false;
}

void HWT101_Drift_Clear(void)
{
  memset(&drift, 0, sizeof(drift));
  memset(&total_fit, 0, sizeof(total_fit));
  memset(half_fit, 0, sizeof(half_fit));
  memset(&validation_fit, 0, sizeof(validation_fit));
  drift.reason = "none";
  unwrapped = reference = 0.0;
}

bool HWT101_Drift_Start(void)
{
  HWT101_Angle_t angle;
  HWT101_Status_t comm;
  if ((drift.state == HWT101_DRIFT_CALIBRATING) ||
      (drift.state == HWT101_DRIFT_VERIFYING)) return false;

  HWT101_Drift_Clear();
  if (!HWT101_Is_Ready() || !HWT101_Angle_Get(&angle) ||
      !HWT101_Angle_Is_Fresh(&angle, HWT101_DATA_FRESH_MS) ||
      !HWT101_Status_Get(&comm))
  {
    Drift_Fail("not_ready");
    return false;
  }
  drift.state = HWT101_DRIFT_CALIBRATING;
  drift.raw_yaw_deg = angle.yaw;
  drift.samples = 1U;
  start_tick = sample_tick = angle.last_update_ms;
  sample_count = angle.update_count;
  error_count = comm.i2c_error_count;
  last_yaw = angle.yaw;
  Drift_FitAdd(&total_fit, 0.0, 0.0);
  Drift_FitAdd(&half_fit[0], 0.0, 0.0);
  return true;
}

bool HWT101_Drift_Restore(float bias_dps)
{
  HWT101_Angle_t angle;
  HWT101_Status_t comm;
  if (drift.state != HWT101_DRIFT_IDLE || !isfinite(bias_dps) || fabsf(bias_dps) > 5.0f ||
      !HWT101_Is_Ready() || !HWT101_Angle_Get(&angle) ||
      !HWT101_Angle_Is_Fresh(&angle, HWT101_DATA_FRESH_MS) || !HWT101_Status_Get(&comm)) return false;
  HWT101_Drift_Clear();
  start_tick = validation_tick = sample_tick = angle.last_update_ms;
  sample_count = angle.update_count;
  error_count = comm.i2c_error_count;
  last_yaw = angle.yaw;
  drift.raw_yaw_deg = angle.yaw;
  drift.bias_dps = bias_dps;
  drift.state = HWT101_DRIFT_DONE;
  drift.compensation_valid = drift.verification_passed = drift.restored_from_flash = true;
  drift.samples = 1U;
  return true;
}

static void Drift_CalibrationFinish(uint32_t tick)
{
  float rms[2], total_rms;
  if (!Drift_FitResult(&total_fit, &drift.bias_dps, &total_rms) ||
      !Drift_FitResult(&half_fit[0], &drift.half_bias_dps[0], &rms[0]) ||
      !Drift_FitResult(&half_fit[1], &drift.half_bias_dps[1], &rms[1]))
  {
    Drift_Fail("samples");
    return;
  }
  drift.calibration_rms_deg = rms[0] > rms[1] ? rms[0] : rms[1];
  if (fabsf(drift.half_bias_dps[0] - drift.half_bias_dps[1]) > DRIFT_MAX_HALF_DIFFERENCE_DPS)
  {
    Drift_Fail("unstable_bias");
    return;
  }
  if (drift.calibration_rms_deg > DRIFT_MAX_CALIBRATION_RMS_DEG)
  {
    Drift_Fail("noise");
    return;
  }
  reference = unwrapped;
  validation_tick = tick;
  drift.relative_raw_deg = drift.corrected_deg = 0.0f;
  drift.compensation_valid = true;
  drift.state = HWT101_DRIFT_VERIFYING;
  Drift_FitAdd(&validation_fit, 0.0, 0.0);
}

void HWT101_Drift_Process(void)
{
  HWT101_Angle_t angle;
  HWT101_Status_t comm;
  uint32_t elapsed, interval;
  float delta;
  double seconds, raw_change, corrected;
  if ((drift.state == HWT101_DRIFT_IDLE) ||
      (drift.state == HWT101_DRIFT_FAILED)) return;

  drift.elapsed_ms = (uint32_t)(HAL_GetTick() - start_tick);
  if (!HWT101_Status_Get(&comm) || (comm.i2c_error_count != error_count) ||
      !HWT101_Is_Ready())
  {
    Drift_Fail("i2c");
    return;
  }
  if (!HWT101_Angle_Get(&angle) ||
      !HWT101_Angle_Is_Fresh(&angle, HWT101_DATA_FRESH_MS))
  {
    Drift_Fail("stale");
    return;
  }
  if (angle.update_count == sample_count) return;
  interval = (uint32_t)(angle.last_update_ms - sample_tick);
  drift.gap_ms = interval;
  if (interval > drift.max_gap_ms) drift.max_gap_ms = interval;
  if (interval > HWT101_DATA_FRESH_MS)
  {
    Drift_Fail("sample_gap");
    return;
  }
  if (interval < DRIFT_SAMPLE_MS) return;

  delta = Drift_Wrap(angle.yaw - last_yaw);
  if ((drift.state != HWT101_DRIFT_DONE) && (fabsf(delta) > DRIFT_MAX_STATIC_STEP_DEG))
  {
    Drift_Fail("motion");
    return;
  }
  sample_tick = angle.last_update_ms;
  sample_count = angle.update_count;
  last_yaw = angle.yaw;
  unwrapped += delta;
  drift.raw_yaw_deg = angle.yaw;
  drift.samples++;
  if (drift.state == HWT101_DRIFT_CALIBRATING)
  {
    elapsed = (uint32_t)(sample_tick - start_tick);
    seconds = elapsed / 1000.0;
    Drift_FitAdd(&total_fit, seconds, unwrapped);
    if (elapsed <= DRIFT_HALF_MS)
      Drift_FitAdd(&half_fit[0], seconds, unwrapped);
    else
      Drift_FitAdd(&half_fit[1], seconds - 30.0, unwrapped);
    if (elapsed >= DRIFT_CALIBRATION_MS) Drift_CalibrationFinish(sample_tick);
    return;
  }

  elapsed = (uint32_t)(sample_tick - validation_tick);
  seconds = elapsed / 1000.0;
  raw_change = unwrapped - reference;
  corrected = raw_change - drift.bias_dps * seconds;
  drift.relative_raw_deg = (float)raw_change;
  drift.corrected_deg = Drift_Wrap((float)corrected);
  if (drift.state == HWT101_DRIFT_VERIFYING)
  {
    Drift_FitAdd(&validation_fit, seconds, raw_change);
    if (elapsed >= DRIFT_VALIDATION_MS)
    {
      if (!Drift_FitResult(&validation_fit, &drift.raw_drift_dps, &drift.validation_rms_deg))
      {
        Drift_Fail("samples");
        return;
      }
      drift.residual_dps = drift.raw_drift_dps - drift.bias_dps;
      if (fabsf(drift.residual_dps) > DRIFT_MAX_RESIDUAL_DPS)
      {
        Drift_Fail("residual");
        return;
      }
      drift.verification_passed = true;
      drift.state = HWT101_DRIFT_DONE;
    }
  }
}

bool HWT101_Drift_Get(HWT101_DriftSnapshot_t *snapshot)
{
  static const char *const names[] = {"IDLE", "CALIBRATING", "VERIFYING", "DONE", "FAILED"};
  if (snapshot == NULL) return false;
  *snapshot = drift;
  snapshot->state_name = names[drift.state];
  if (snapshot->reason == NULL) snapshot->reason = "none";
  return true;
}

bool HWT101_Drift_ControlAngleGet(HWT101_Angle_t *angle)
{
  HWT101_Angle_t latest;
  HWT101_Status_t comm;
  double change, seconds;
  if ((angle == NULL) || (drift.state != HWT101_DRIFT_DONE) ||
      !drift.compensation_valid || !drift.verification_passed || !HWT101_Is_Ready() ||
      !HWT101_Status_Get(&comm) || (comm.i2c_error_count != error_count) ||
      !HWT101_Angle_Get(&latest) || !HWT101_Angle_Is_Fresh(&latest, HWT101_DATA_FRESH_MS) ||
      ((uint32_t)(latest.last_update_ms - sample_tick) > HWT101_DATA_FRESH_MS))
    return false;

  /* 用最新读取补上距上个拟合样本的变化，避免控制角度只以10Hz更新。 */
  change = unwrapped + Drift_Wrap(latest.yaw - last_yaw) - reference;
  seconds = (uint32_t)(latest.last_update_ms - validation_tick) / 1000.0;
  latest.yaw = Drift_Wrap((float)(change - drift.bias_dps * seconds));
  *angle = latest;
  return true;
}
