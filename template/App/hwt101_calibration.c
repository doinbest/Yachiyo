#include "hwt101_calibration.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include <math.h>
#include <string.h>

#define REG_VERSION 0x2EU
#define REG_MODE 0x48U
#define REG_BIAS 0x4CU
#define REG_KEY 0x69U
#define REG_SAVE 0x00U
#define REG_CALIYAW 0x76U
#define COMMAND_WAIT_MS 100U
#define CALIBRATION_MS 20000U
#define SAMPLE_MS 100U

typedef enum
{
  STEP_IDLE, STEP_START_UNLOCK, STEP_START_WRITE, STEP_START_CHECK, STEP_CAL_WAIT,
  STEP_EXIT_UNLOCK, STEP_EXIT_WRITE, STEP_EXIT_CHECK,
  STEP_ZERO_UNLOCK, STEP_ZERO_WRITE, STEP_ZERO_WAIT,
  STEP_VERIFY, STEP_SAVE_UNLOCK, STEP_SAVE_WRITE, STEP_SAVE_CHECK
} CalStep_t;

static HWT101_CalStatus_t cal;
static CalStep_t step;
static uint32_t run_tick, step_tick, cal_tick, verify_tick, verify_wait_tick;
static uint32_t error_baseline, last_sample_tick, last_sample_count;
static uint32_t control_tick, control_count;
static bool native_run, start_attempted, aborting, cancelled, control_valid;
static unsigned exit_attempts;
static float previous_yaw, unwrapped;
static double sum_t, sum_y, sum_tt, sum_ty, sum_yy;

static const char *const state_names[] = {
  "IDLE", "PREPARING", "CALIBRATING", "RESTORING", "VERIFYING", "SAVING",
  "DONE", "FAILED", "CANCELLED"
};

static void SetState(HWT101_CalState_t state)
{
  cal.state = state;
  cal.state_name = state_names[state];
  cal.busy = state >= HWT101_CAL_PREPARING && state <= HWT101_CAL_SAVING;
}

static void Finish(HWT101_CalState_t state)
{
  cal.elapsed_ms = (uint32_t)(HAL_GetTick() - run_tick);
  SetState(state);
  step = STEP_IDLE;
  if (state == HWT101_CAL_CANCELLED) cal.result = "CANCELLED";
  else cal.result = (state == HWT101_CAL_DONE && cal.verified) ? "PASS" : "FAIL";
}

static bool FreshAngle(HWT101_Angle_t *angle)
{
  return HWT101_Is_Ready() && HWT101_Angle_Get(angle) &&
         HWT101_Angle_Is_Fresh(angle, HWT101_DATA_FRESH_MS) && isfinite(angle->yaw);
}

static void BeginRestore(void)
{
  cal.normal_mode_confirmed = false;
  SetState(HWT101_CAL_RESTORING);
  exit_attempts = 0;
  step = STEP_EXIT_UNLOCK;
  step_tick = HAL_GetTick() - COMMAND_WAIT_MS;
}

static void Fail(const char *reason, HAL_StatusTypeDef hal)
{
  cal.reason = reason;
  cal.hal = hal;
  control_valid = false;
  aborting = true;
  if (start_attempted) BeginRestore();
  else Finish(HWT101_CAL_FAILED);
}

void HWT101_Cal_Init(void)
{
  memset(&cal, 0, sizeof(cal));
  SetState(HWT101_CAL_IDLE);
  cal.reason = "none";
  cal.result = "NONE";
  cal.save_state = "NOT_REQUESTED";
  step = STEP_IDLE;
  start_attempted = control_valid = native_run = aborting = cancelled = false;
}

HAL_StatusTypeDef HWT101_Cal_RefreshRegisters(void)
{
  uint16_t version, mode, bias;
  HAL_StatusTypeDef status;
  if (cal.busy) return HAL_BUSY;
  status = HWT101_ReadRegister(REG_VERSION, &version);
  if (status == HAL_OK) status = HWT101_ReadRegister(REG_MODE, &mode);
  if (status == HAL_OK) status = HWT101_ReadRegister(REG_BIAS, &bias);
  cal.registers_valid = status == HAL_OK;
  if (status == HAL_OK)
  {
    cal.version = version;
    cal.mode = mode;
    cal.normal_mode_confirmed = mode == 0U;
    if (cal.run_id == 0U) cal.bias_before = bias;
    cal.bias_after = bias;
    cal.bias_after_valid = true;
    if (mode != 0U) control_valid = false;
  }
  else
  {
    cal.normal_mode_confirmed = false;
    cal.bias_after_valid = false;
    control_valid = false;
  }
  return status;
}

static void BeginVerify(void)
{
  HWT101_Angle_t angle;
  SetState(HWT101_CAL_VERIFYING);
  step = STEP_VERIFY;
  verify_wait_tick = HAL_GetTick();
  last_sample_count = HWT101_Angle_Get(&angle) ? angle.update_count : 0U;
  cal.samples = 0;
  sum_t = sum_y = sum_tt = sum_ty = sum_yy = 0;
  unwrapped = 0;
}

static void BeginZero(void)
{
  SetState(HWT101_CAL_PREPARING);
  step = STEP_ZERO_UNLOCK;
  step_tick = HAL_GetTick() - COMMAND_WAIT_MS;
}

static bool StartRun(bool native, uint32_t duration_ms)
{
  HWT101_Angle_t angle;
  HWT101_Status_t comm;
  HAL_StatusTypeDef hal;
  uint32_t id;
  if (cal.busy || ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) return false;
  id = cal.run_id + 1U;
  memset(&cal, 0, sizeof(cal));
  SetState(HWT101_CAL_IDLE);
  cal.run_id = id;
  cal.reason = "none";
  cal.result = "PENDING";
  cal.save_state = "NOT_REQUESTED";
  cal.verify_ms = duration_ms;
  cal.total_ms = duration_ms + (native ? CALIBRATION_MS : 0U);
  control_valid = false;
  run_tick = HAL_GetTick();
  if (!FreshAngle(&angle))
  {
    cal.reason = "imu_not_fresh";
    Finish(HWT101_CAL_FAILED);
    return false;
  }
  hal = HWT101_Cal_RefreshRegisters();
  if (hal != HAL_OK || !cal.normal_mode_confirmed)
  {
    cal.reason = hal == HAL_OK ? "mode_not_normal" : "register_read";
    cal.hal = hal;
    Finish(HWT101_CAL_FAILED);
    return false;
  }
  cal.bias_before = cal.bias_after;
  native_run = native;
  start_attempted = aborting = cancelled = control_valid = false;
  (void)HWT101_Status_Get(&comm);
  error_baseline = comm.i2c_error_count;
  run_tick = HAL_GetTick();
  step_tick = run_tick - COMMAND_WAIT_MS;
  if (native)
  {
    SetState(HWT101_CAL_PREPARING);
    step = STEP_START_UNLOCK;
  }
  else BeginZero();
  return true;
}

bool HWT101_Cal_Start(void) { return StartRun(true, 30000U); }

bool HWT101_Cal_VerifyStart(uint32_t duration_ms)
{
  if (duration_ms != 10000U && duration_ms != 30000U) return false;
  return StartRun(false, duration_ms);
}

void HWT101_Cal_Cancel(void)
{
  if (cal.state == HWT101_CAL_RESTORING && cancelled) return;
  control_valid = cal.verified = false;
  cal.reason = "cancelled";
  cal.result = "CANCELLED";
  aborting = cancelled = true;
  if (start_attempted || !cal.normal_mode_confirmed) BeginRestore();
  else Finish(HWT101_CAL_CANCELLED);
}

static void ExitFailed(HAL_StatusTypeDef hal)
{
  cal.hal = hal;
  if (exit_attempts >= 3U)
  {
    cal.normal_mode_confirmed = control_valid = false;
    cal.reason = "normal_mode_unconfirmed";
    Finish(HWT101_CAL_FAILED);
  }
  else
  {
    step = STEP_EXIT_UNLOCK;
    step_tick = HAL_GetTick();
  }
}

static void VerifySample(void)
{
  HWT101_Angle_t angle;
  uint32_t gap;
  float delta;
  double t, y, denominator, slope, intercept, error;
  if (!FreshAngle(&angle))
  {
    Fail("stale", HAL_OK);
    return;
  }
  if (angle.update_count == last_sample_count)
  {
    if (cal.samples == 0U && (uint32_t)(HAL_GetTick() - verify_wait_tick) > HWT101_DATA_FRESH_MS)
      Fail("stale", HAL_OK);
    return;
  }
  if (cal.samples == 0U)
  {
    cal.zero_after_deg = angle.yaw;
    cal.zero_sample_received = true;
    previous_yaw = angle.yaw;
    verify_tick = last_sample_tick = angle.last_update_ms;
  }
  else
  {
    gap = (uint32_t)(angle.last_update_ms - last_sample_tick);
    if (gap < SAMPLE_MS) return;
    cal.gap_ms = gap;
    if (gap > cal.max_gap_ms) cal.max_gap_ms = gap;
    if (gap > HWT101_DATA_FRESH_MS) { Fail("sample_gap", HAL_OK); return; }
    delta = angle.yaw - previous_yaw;
    if (delta >= 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    if (fabsf(delta) > 5.0f) { Fail("motion", HAL_OK); return; }
    unwrapped += delta;
    previous_yaw = angle.yaw;
  }
  last_sample_count = angle.update_count;
  last_sample_tick = angle.last_update_ms;
  cal.verify_elapsed_ms = (uint32_t)(angle.last_update_ms - verify_tick);
  cal.relative_deg = unwrapped;
  t = (double)cal.verify_elapsed_ms / 1000.0;
  y = (double)unwrapped;
  ++cal.samples;
  sum_t += t; sum_y += y; sum_tt += t*t; sum_ty += t*y; sum_yy += y*y;
  if (cal.verify_elapsed_ms < cal.verify_ms) return;
  denominator = cal.samples * sum_tt - sum_t * sum_t;
  if (denominator <= 0.0) { Fail("samples", HAL_OK); return; }
  slope = (cal.samples * sum_ty - sum_t * sum_y) / denominator;
  intercept = (sum_y - slope * sum_t) / cal.samples;
  error = sum_yy - intercept * sum_y - slope * sum_ty;
  if (error < 0.0) error = 0.0;
  cal.drift_dps = (float)slope;
  cal.rms_deg = (float)sqrt(error / cal.samples);
  if (fabsf(cal.drift_dps) > 0.01f || cal.rms_deg > 0.1f)
  { Fail("drift_or_noise", HAL_OK); return; }
  cal.verified = true;
  cal.result = "PASS";
  control_tick = angle.last_update_ms;
  control_count = angle.update_count;
  if (native_run)
  {
    SetState(HWT101_CAL_SAVING);
    cal.save_state = "PENDING";
    step = STEP_SAVE_UNLOCK;
    step_tick = HAL_GetTick() - COMMAND_WAIT_MS;
  }
  else { control_valid = true; Finish(HWT101_CAL_DONE); }
}

static void SaveFailed(HAL_StatusTypeDef hal)
{
  cal.save_state = "ERROR";
  Fail("save_failed", hal); /* 保留verified，保存失败不改写验证测量结果。 */
}

void HWT101_Cal_Process(void)
{
  HAL_StatusTypeDef hal;
  HWT101_Status_t comm;
  HWT101_Angle_t angle;
  uint16_t value;
  uint32_t now = HAL_GetTick();
  (void)HWT101_Status_Get(&comm);
  if (cal.busy) cal.elapsed_ms = (uint32_t)(now - run_tick);
  if (cal.busy) cal.i2c_errors = (uint32_t)(comm.i2c_error_count - error_baseline);
  if (!cal.busy)
  {
    if (control_valid)
    {
      if (comm.i2c_error_count != error_baseline || !FreshAngle(&angle) ||
          (uint32_t)(angle.last_update_ms - control_tick) > HWT101_DATA_FRESH_MS)
      {
        control_valid = false;
        cal.reason = "control_data_invalid";
      }
      else if (angle.update_count != control_count)
      { control_tick = angle.last_update_ms; control_count = angle.update_count; }
    }
    return;
  }
  if (step != STEP_EXIT_UNLOCK && step != STEP_EXIT_WRITE && step != STEP_EXIT_CHECK &&
      step != STEP_SAVE_UNLOCK && step != STEP_SAVE_WRITE && step != STEP_SAVE_CHECK)
  {
    if (comm.i2c_error_count != error_baseline || !HWT101_Is_Ready())
    { Fail("i2c", HAL_ERROR); return; }
    if (step != STEP_VERIFY && !FreshAngle(&angle))
    { Fail("stale", HAL_OK); return; }
  }
  if (step != STEP_VERIFY && step != STEP_CAL_WAIT &&
      (uint32_t)(now - step_tick) < COMMAND_WAIT_MS) return;
  switch (step)
  {
    case STEP_START_UNLOCK:
      hal = HWT101_WriteRegister(REG_KEY, 0xB588U);
      if (hal != HAL_OK) { Fail("unlock", hal); break; }
      step = STEP_START_WRITE; step_tick = HAL_GetTick();
      break;
    case STEP_START_WRITE:
      start_attempted = true;
      cal.normal_mode_confirmed = false;
      hal = HWT101_WriteRegister(REG_MODE, 1U);
      if (hal != HAL_OK) { Fail("start_write", hal); break; }
      cal_tick = step_tick = HAL_GetTick();
      step = STEP_START_CHECK;
      break;
    case STEP_START_CHECK:
      hal = HWT101_ReadRegister(REG_MODE, &value);
      if (hal == HAL_OK) cal.mode = value;
      if (hal != HAL_OK || value != 1U) { Fail("start_readback", hal); break; }
      SetState(HWT101_CAL_CALIBRATING); step = STEP_CAL_WAIT;
      break;
    case STEP_CAL_WAIT:
      if ((uint32_t)(now - cal_tick) >= CALIBRATION_MS) BeginRestore();
      break;
    case STEP_EXIT_UNLOCK:
      ++exit_attempts;
      hal = HWT101_WriteRegister(REG_KEY, 0xB588U);
      if (hal != HAL_OK) { ExitFailed(hal); break; }
      step = STEP_EXIT_WRITE; step_tick = HAL_GetTick();
      break;
    case STEP_EXIT_WRITE:
      hal = HWT101_WriteRegister(REG_MODE, 0U);
      if (hal != HAL_OK) { ExitFailed(hal); break; }
      step = STEP_EXIT_CHECK; step_tick = HAL_GetTick();
      break;
    case STEP_EXIT_CHECK:
      hal = HWT101_ReadRegister(REG_MODE, &value);
      if (hal == HAL_OK) cal.mode = value;
      if (hal != HAL_OK || value != 0U) { ExitFailed(hal); break; }
      cal.normal_mode_confirmed = true;
      if (aborting)
      { Finish(cancelled ? HWT101_CAL_CANCELLED : (cal.verified ? HWT101_CAL_DONE : HWT101_CAL_FAILED)); break; }
      /* 恢复阶段出现过通信错误，本轮数据不能作为保存依据。 */
      if (comm.i2c_error_count != error_baseline)
      { cal.reason = "i2c"; cal.hal = HAL_ERROR; Finish(HWT101_CAL_FAILED); break; }
      hal = HWT101_ReadRegister(REG_BIAS, &value);
      cal.bias_after_valid = hal == HAL_OK;
      if (hal != HAL_OK) { Fail("bias_read", hal); break; }
      cal.bias_after = value;
      BeginZero();
      break;
    case STEP_ZERO_UNLOCK:
      hal = HWT101_WriteRegister(REG_KEY, 0xB588U);
      if (hal != HAL_OK) { Fail("yaw_zero_unlock", hal); break; }
      step = STEP_ZERO_WRITE; step_tick = HAL_GetTick();
      break;
    case STEP_ZERO_WRITE:
      if (!FreshAngle(&angle)) { Fail("stale", HAL_OK); break; }
      cal.zero_before_deg = angle.yaw;
      /* CALIYAW is an action, not a stored bias. Verification never sends SAVE. */
      hal = HWT101_WriteRegister(REG_CALIYAW, 0U);
      if (hal != HAL_OK) { Fail("yaw_zero_write", hal); break; }
      cal.zero_requested = true;
      step = STEP_ZERO_WAIT; step_tick = HAL_GetTick();
      break;
    case STEP_ZERO_WAIT:
      /* Ignore the reset jump and cached samples; start with a new sample. */
      BeginVerify();
      break;
    case STEP_VERIFY:
      VerifySample();
      break;
    case STEP_SAVE_UNLOCK:
      if (comm.i2c_error_count != error_baseline || !FreshAngle(&angle))
      { SaveFailed(HAL_ERROR); break; }
      hal = HWT101_WriteRegister(REG_KEY, 0xB588U);
      if (hal != HAL_OK) { SaveFailed(hal); break; }
      step = STEP_SAVE_WRITE; step_tick = HAL_GetTick();
      break;
    case STEP_SAVE_WRITE:
      if (comm.i2c_error_count != error_baseline || !FreshAngle(&angle))
      { SaveFailed(HAL_ERROR); break; }
      hal = HWT101_WriteRegister(REG_SAVE, 0U);
      if (hal != HAL_OK) { SaveFailed(hal); break; }
      cal.save_requested = true;
      cal.save_state = "REQUEST_SENT";
      step = STEP_SAVE_CHECK; step_tick = HAL_GetTick();
      break;
    case STEP_SAVE_CHECK:
      hal = HWT101_ReadRegister(REG_MODE, &value);
      if (hal == HAL_OK) cal.mode = value;
      if (hal != HAL_OK || value != 0U) { SaveFailed(hal); break; }
      hal = HWT101_ReadRegister(REG_BIAS, &value);
      cal.bias_after_valid = hal == HAL_OK;
      if (hal != HAL_OK) { SaveFailed(hal); break; }
      cal.bias_after = value;
      cal.save_readback_ok = true;
      cal.save_state = "READBACK_OK";
      /* 回读只能证实当前寄存器可读，断电保持需实物验证。 */
      if (FreshAngle(&angle) && comm.i2c_error_count == error_baseline)
      { control_valid = true; control_tick = angle.last_update_ms; control_count = angle.update_count; }
      Finish(HWT101_CAL_DONE);
      break;
    default: break;
  }
}

bool HWT101_Cal_GetStatus(HWT101_CalStatus_t *status)
{
  HWT101_Angle_t angle;
  if (status == NULL) return false;
  *status = cal;
  status->control_ready = HWT101_Cal_ControlAngleGet(&angle);
  return true;
}

bool HWT101_Cal_IsBusy(void) { return cal.busy; }

bool HWT101_Cal_ControlAngleGet(HWT101_Angle_t *angle)
{
  HWT101_Angle_t latest;
  HWT101_Status_t comm;
  if (angle == NULL || cal.busy || !control_valid || !cal.verified || !cal.normal_mode_confirmed)
    return false;
  (void)HWT101_Status_Get(&comm);
  if (!FreshAngle(&latest) || comm.i2c_error_count != error_baseline ||
      (uint32_t)(latest.last_update_ms - control_tick) > HWT101_DATA_FRESH_MS)
  { control_valid = false; return false; }
  *angle = latest;
  return true;
}
