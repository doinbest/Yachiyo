#include "hwt101_calibration.h"
#include "chassis_route.h"
#include "chassis_motion.h"
#include <math.h>
#include <string.h>

#define REG_VERSION 0x2EU
#define REG_MODE 0x48U
#define REG_BIAS 0x4CU
#define REG_PEAK_TIME 0x4BU
#define REG_BIAS_TIME 0x4DU
#define REG_KEY 0x69U
#define REG_SAVE 0x00U
#define REG_CALIYAW 0x76U
#define COMMAND_WAIT_MS 100U
#define CALIBRATION_MS 20000U
#define SAMPLE_MS 100U
#define RESTORE_WAIT_MS 5000U /* Allows 1s reprobes, two BUSY probes and three exit attempts. */
#define READ_RECOVERY_MS 5000U

typedef enum
{
  STEP_IDLE, STEP_START_UNLOCK, STEP_START_WRITE, STEP_START_CHECK, STEP_CAL_WAIT,
  STEP_EXIT_UNLOCK, STEP_EXIT_WRITE, STEP_EXIT_CHECK, STEP_BIAS_READ,
  STEP_ZERO_UNLOCK, STEP_ZERO_WRITE, STEP_ZERO_WAIT,
  STEP_VERIFY, STEP_SAVE_UNLOCK, STEP_SAVE_WRITE, STEP_SAVE_CHECK
} CalStep_t;

static HWT101_CalStatus_t cal;
static CalStep_t step;
static uint32_t run_tick, step_tick, cal_tick, verify_tick, verify_wait_tick;
static uint32_t restore_tick, native_poll_tick;
static uint32_t error_baseline, last_sample_tick, last_sample_count;
static uint32_t accepted_errors, recovery_tick, recovery_errors, recovery_last_count;
static uint32_t save_recovery_tick;
static unsigned recovery_samples;
static bool recovery_mode_pending, save_recovery_started;
static uint32_t control_tick, control_count;
static uint32_t control_errors, control_mode_tick;
static unsigned control_recovery_samples;
static bool control_mode_recheck_pending, control_mode_abnormal;
static bool native_run, start_attempted, aborting, cancelled, control_valid;
static unsigned exit_attempts;
static bool boot_pending;
static float previous_yaw, unwrapped;
static double sum_t, sum_y, sum_tt, sum_ty, sum_yy;

static const char *const state_names[] = {
  "IDLE", "PREPARING", "CALIBRATING", "RESTORING", "VERIFYING", "SAVING",
  "DONE", "FAILED", "CANCELLED"
};
static const char *const step_names[] = {
  "IDLE", "START_UNLOCK", "START_WRITE", "START_CHECK", "CAL_WAIT",
  "EXIT_UNLOCK", "EXIT_WRITE", "EXIT_CHECK", "BIAS_READ",
  "ZERO_UNLOCK", "ZERO_WRITE", "ZERO_WAIT", "VERIFY", "SAVE_UNLOCK",
  "SAVE_WRITE", "SAVE_CHECK"
};

static void SetState(HWT101_CalState_t state)
{
  cal.state = state;
  cal.state_name = state_names[state];
  cal.busy = state >= HWT101_CAL_PREPARING && state <= HWT101_CAL_SAVING;
}

static void Finish(HWT101_CalState_t state)
{
  cal.read_recovering = false;
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
  cal.read_recovering = false;
  cal.normal_mode_confirmed = false;
  SetState(HWT101_CAL_RESTORING);
  exit_attempts = 0;
  restore_tick = HAL_GetTick();
  step = STEP_EXIT_UNLOCK;
  step_tick = HAL_GetTick() - COMMAND_WAIT_MS;
}

static void RecordFailure(const char *reason, HAL_StatusTypeDef hal)
{
  HWT101_Status_t comm;
  if (strcmp(cal.first_reason, "none")) return;
  cal.first_reason = reason;
  cal.first_hal = hal;
  if (HWT101_Status_Get(&comm))
  {
    cal.first_fault_ms = comm.fault_ms;
    cal.first_hal_error = comm.hal_error;
    cal.first_hal_state = comm.hal_state;
    cal.first_operation = comm.fault_operation;
    cal.first_register = comm.fault_register;
    if (!strcmp(reason, "i2c")) cal.first_hal = comm.last_hal;
  }
}

static void Fail(const char *reason, HAL_StatusTypeDef hal)
{
  RecordFailure(reason, hal);
  if (cal.state == HWT101_CAL_SAVING) cal.save_state = "ERROR";
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
  cal.first_reason = "none";
  cal.result = "NONE";
  cal.save_state = "NOT_REQUESTED";
  step = STEP_IDLE;
  start_attempted = control_valid = native_run = aborting = cancelled = boot_pending = false;
  save_recovery_started = false;
  control_mode_recheck_pending = control_mode_abnormal = false;
  control_recovery_samples = 0U;
}

static void SuspendControl(void)
{
  HWT101_Angle_t angle;
  HWT101_Status_t comm;
  control_valid = false;
  control_recovery_samples = 0U;
  if (HWT101_Status_Get(&comm)) control_errors = comm.i2c_error_count;
  if (HWT101_Angle_Get(&angle))
  { control_tick = angle.last_update_ms; control_count = angle.update_count; }
}

HAL_StatusTypeDef HWT101_Cal_RefreshRegisters(void)
{
  uint16_t version, mode, bias, peak_time, bias_time;
  HAL_StatusTypeDef status;
  bool was_normal = cal.normal_mode_confirmed;
  if (cal.busy) return HAL_BUSY;
  status = HWT101_ReadRegister(REG_VERSION, &version);
  if (status == HAL_OK)
  {
    status = HWT101_ReadRegister(REG_MODE, &mode);
    if (status == HAL_OK)
    {
      /* A later BIAS failure cannot erase an actually observed bad MODE. */
      cal.mode = mode;
      cal.normal_mode_confirmed = mode == 0U;
      control_mode_abnormal = mode != 0U;
      control_mode_recheck_pending = false;
    }
  }
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
    if (mode != 0U || !was_normal)
    {
      SuspendControl();
      if (cal.verified) cal.reason = mode == 0U ?
          "control_data_invalid" : "mode_not_normal";
    }
    /* Timing is optional diagnostic evidence; unsupported/failed timing reads
     * do not turn an otherwise valid normal-mode status into a mode failure. */
    cal.timing_valid = HWT101_ReadRegister(REG_PEAK_TIME, &peak_time) == HAL_OK;
    if (cal.timing_valid) cal.timing_valid = HWT101_ReadRegister(REG_BIAS_TIME, &bias_time) == HAL_OK;
    if (cal.timing_valid) { cal.peak_time_s = peak_time; cal.bias_time_s = bias_time; }
  }
  else
  {
    cal.normal_mode_confirmed = false;
    cal.bias_after_valid = false;
    SuspendControl();
    control_mode_recheck_pending = !control_mode_abnormal;
    control_mode_tick = HAL_GetTick() - COMMAND_WAIT_MS;
    if (cal.verified) cal.reason = control_mode_abnormal ?
        "mode_not_normal" : "control_mode_unconfirmed";
    cal.timing_valid = false;
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
  cal.verify_elapsed_ms = 0;
  cal.relative_deg = cal.drift_dps = cal.rms_deg = 0;
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
  cal.first_reason = "none";
  cal.result = "PENDING";
  cal.save_state = "NOT_REQUESTED";
  cal.verify_ms = duration_ms;
  cal.total_ms = duration_ms + (native ? CALIBRATION_MS : 0U);
  control_valid = false;
  control_recovery_samples = 0U;
  control_mode_recheck_pending = control_mode_abnormal = false;
  save_recovery_started = false;
  run_tick = HAL_GetTick();
  if (!FreshAngle(&angle))
  {
    cal.reason = "imu_not_fresh";
    RecordFailure(cal.reason, HAL_OK);
    Finish(HWT101_CAL_FAILED);
    return false;
  }
  hal = HWT101_Cal_RefreshRegisters();
  if (hal != HAL_OK || !cal.normal_mode_confirmed)
  {
    cal.reason = hal == HAL_OK ? "mode_not_normal" : "register_read";
    cal.hal = hal;
    RecordFailure(cal.reason, hal);
    Finish(HWT101_CAL_FAILED);
    return false;
  }
  cal.bias_before = cal.bias_after;
  cal.native_wait_ms = CALIBRATION_MS;
  if (native && cal.timing_valid) {
    uint32_t configured = ((uint32_t)cal.peak_time_s + cal.bias_time_s) * 1000U;
    if (configured > cal.native_wait_ms) cal.native_wait_ms = configured;
  }
  if (native) cal.total_ms = duration_ms + cal.native_wait_ms;
  native_run = native;
  start_attempted = aborting = cancelled = control_valid = false;
  (void)HWT101_Status_Get(&comm);
  error_baseline = comm.i2c_error_count;
  accepted_errors = error_baseline;
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
bool HWT101_Cal_QuickStart(void) { return StartRun(true, 10000U); }
bool HWT101_Cal_ZeroStart(void) { return StartRun(false, 0U); }

bool HWT101_Cal_VerifyStart(uint32_t duration_ms)
{
  if (duration_ms != 5000U && duration_ms != 30000U) return false;
  return StartRun(false, duration_ms);
}

void HWT101_Cal_BootVerifyArm(void)
{
  if (cal.busy || cal.run_id != 0U) return;
  boot_pending = true;
  run_tick = HAL_GetTick();
  cal.reason = "boot_wait_imu";
  cal.result = "PENDING";
  cal.verify_ms = cal.total_ms = 5000U;
  SetState(HWT101_CAL_PREPARING);
}

void HWT101_Cal_BootVerifyProcess(bool motion_idle)
{
  HWT101_Angle_t angle;
  if (!boot_pending) return;
  if (!motion_idle || (uint32_t)(HAL_GetTick() - run_tick) >= 5000U)
  {
    boot_pending = false;
    cal.reason = motion_idle ? "boot_imu_timeout" : "boot_motion_busy";
    Finish(HWT101_CAL_FAILED);
    return;
  }
  if (!FreshAngle(&angle)) return;
  boot_pending = false;
  SetState(HWT101_CAL_IDLE);
  (void)StartRun(false, 5000U);
}

void HWT101_Cal_Cancel(void)
{
  if (boot_pending)
  {
    boot_pending = false;
    cal.reason = "cancelled";
    Finish(HWT101_CAL_CANCELLED);
    return;
  }
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

static void BeginReadRecovery(const HWT101_Status_t *comm)
{
  HWT101_Angle_t angle;
  if (cal.read_recovering) return;
  if (comm->i2c_error_count != accepted_errors) RecordFailure("i2c", comm->last_hal);
  cal.read_recovering = true;
  cal.recovery_count++;
  cal.recovery_elapsed_ms = 0;
  cal.reason = "i2c_recovering";
  control_valid = false;
  /* MODE may recover while BIAS still fails. Every post-SAVE read attempt
   * shares the first outage's deadline; never replay the accepted SAVE. */
  if (step == STEP_SAVE_CHECK)
  {
    if (!save_recovery_started)
    { save_recovery_tick = HAL_GetTick(); save_recovery_started = true; }
    recovery_tick = save_recovery_tick;
  }
  else recovery_tick = HAL_GetTick();
  recovery_errors = comm->i2c_error_count;
  recovery_last_count = HWT101_Angle_Get(&angle) ? angle.update_count : 0U;
  recovery_samples = 0;
  recovery_mode_pending = false;
  cal.recovery_step = step_names[step];
  cal.recovery_expected_mode = (step == STEP_START_CHECK || step == STEP_CAL_WAIT ||
                               step == STEP_EXIT_UNLOCK || step == STEP_EXIT_WRITE) ? 1U : 0U;
  /* WORKMODE 1 collects peak-to-peak noise; 2 computes zero bias. Both are native phases. */
  cal.recovery_mode_mask = cal.recovery_expected_mode == 1U ?
      ((cal.native_modes_seen & 0x04U) ? 0x0007U : 0x0006U) : 0x0001U;
  cal.recovery_mode_first = cal.recovery_mode_last = 0U;
  cal.recovery_mode_seen = cal.recovery_mode_valid = false;
  cal.recovery_mode_hal = HAL_OK;
  cal.recovery_mode_reads = cal.recovery_mode_tick = 0U;
}

/* A module may return to normal after its internal zero-bias stage. This is
 * only a phase observation: hold the full acquisition budget, then verify. */
static void NativeModeObserve(uint16_t mode)
{
  if (mode == 1U || mode == 2U) cal.native_modes_seen |= (uint8_t)(1U << mode);
  else if (mode == 0U && (cal.native_modes_seen & 0x04U)) {
    if (!(cal.native_modes_seen & 0x01U))
      cal.native_normal_ms = (uint32_t)(HAL_GetTick() - cal_tick);
    cal.native_modes_seen |= 0x01U;
    cal.reason = "native_returned_normal_wait";
  }
}

/* Never replay MODE=1, CALIYAW or SAVE while reads recover. An interrupted
 * verification restarts its sample window; SAVE_CHECK retains completed data. */
static bool ReadRecoveryProcess(const HWT101_Status_t *comm)
{
  HWT101_Angle_t angle;
  uint16_t mode;
  cal.recovery_elapsed_ms = (uint32_t)(HAL_GetTick() - recovery_tick);
  if (cal.recovery_elapsed_ms >= READ_RECOVERY_MS)
  { Fail("i2c_recovery_timeout", HAL_ERROR); return false; }
  if (comm->i2c_error_count != recovery_errors) {
    recovery_errors = comm->i2c_error_count;
    recovery_samples = 0;
    recovery_mode_pending = false;
    recovery_last_count = HWT101_Angle_Get(&angle) ? angle.update_count : 0U;
  }
  if (!FreshAngle(&angle)) { recovery_samples = 0; return false; }
  if (angle.update_count == recovery_last_count) return false;
  recovery_last_count = angle.update_count;
  if (++recovery_samples < 2U) return false;
  if (recovery_mode_pending &&
      (uint32_t)(HAL_GetTick() - cal.recovery_mode_tick) < COMMAND_WAIT_MS) return false;
  cal.recovery_mode_hal = HWT101_ReadRegister(REG_MODE, &mode);
  cal.recovery_mode_tick = HAL_GetTick();
  ++cal.recovery_mode_reads;
  cal.recovery_mode_valid = cal.recovery_mode_hal == HAL_OK;
  if (!cal.recovery_mode_valid) { recovery_mode_pending = false; return false; }
  if (!cal.recovery_mode_seen) cal.recovery_mode_first = mode;
  cal.recovery_mode_seen = true;
  cal.recovery_mode_last = mode;
  cal.mode = mode;
  if (mode >= 16U || !(cal.recovery_mode_mask & (1U << mode))) {
    if (recovery_mode_pending) { Fail("recovery_mode_changed", HAL_ERROR); return false; }
    /* Confirm a mismatching read after 100 ms; never rewrite MODE to hide it. */
    recovery_mode_pending = true;
    cal.reason = "recovery_mode_recheck";
    return false;
  }
  if (cal.recovery_expected_mode == 1U) NativeModeObserve(mode);
  recovery_mode_pending = false;
  if (step == STEP_VERIFY && cal.samples) {
    float delta = angle.yaw - previous_yaw;
    if (delta >= 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    if (fabsf(delta) > 5.0f) { Fail("motion", HAL_OK); return false; }
  }
  accepted_errors = comm->i2c_error_count;
  cal.read_recovering = false;
  if (step == STEP_VERIFY) BeginVerify();
  cal.reason = mode == 0U && cal.recovery_expected_mode == 1U ?
      "native_returned_normal_wait" : "i2c_recovered";
  if (step == STEP_START_CHECK)
  {
    SetState(HWT101_CAL_CALIBRATING);
    step = STEP_CAL_WAIT;
    /* The mode just read is the start confirmation. Resume next pass, keeping
     * both native clocks, rather than reading START_CHECK again this pass. */
    return false;
  }
  return true;
}

static void VerifySample(void)
{
  HWT101_Angle_t angle;
  uint32_t gap;
  float delta;
  double t, y, denominator, slope, intercept, error;
  if (!FreshAngle(&angle))
  {
    HWT101_Status_t comm;
    (void)HWT101_Status_Get(&comm);
    BeginReadRecovery(&comm);
    return;
  }
  if (angle.update_count == last_sample_count)
  {
    if (cal.samples == 0U && (uint32_t)(HAL_GetTick() - verify_wait_tick) > HWT101_DATA_FRESH_MS)
    {
      HWT101_Status_t comm;
      (void)HWT101_Status_Get(&comm);
      BeginReadRecovery(&comm);
    }
    return;
  }
  if (cal.samples == 0U)
  {
    if (!cal.zero_sample_received) {
      cal.zero_after_deg = angle.yaw;
      cal.zero_sample_received = true;
    }
    if (cal.verify_ms == 0U)
    {
      /* A fresh post-write sample is observable evidence, not drift verification. */
      Finish(HWT101_CAL_DONE);
      cal.reason = "yaw_zero_sample";
      cal.result = "ZERO_SAMPLE";
      return;
    }
    previous_yaw = angle.yaw;
    verify_tick = last_sample_tick = angle.last_update_ms;
  }
  else
  {
    gap = (uint32_t)(angle.last_update_ms - last_sample_tick);
    if (gap < SAMPLE_MS) return;
    cal.gap_ms = gap;
    if (gap > cal.max_gap_ms) cal.max_gap_ms = gap;
    if (gap > HWT101_DATA_FRESH_MS) {
      HWT101_Status_t comm;
      (void)HWT101_Status_Get(&comm);
      BeginReadRecovery(&comm); return;
    }
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
  control_errors = accepted_errors;
  control_recovery_samples = 0U;
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

/* Only the main loop performs automatic MODE reads. Cache getters cannot turn
 * an unconfirmed mode or an old angle into control qualification. */
static void ControlModeRecheck(void)
{
  HWT101_Angle_t angle;
  uint16_t mode;
  if (!control_mode_recheck_pending || !cal.verified ||
      cal.state != HWT101_CAL_DONE || strcmp(cal.result, "PASS") ||
      (native_run && !cal.save_readback_ok) || !FreshAngle(&angle) ||
      (uint32_t)(HAL_GetTick() - control_mode_tick) < COMMAND_WAIT_MS) return;
  control_mode_tick = HAL_GetTick();
  if (HWT101_ReadRegister(REG_MODE, &mode) != HAL_OK) return;
  cal.mode = mode;
  cal.normal_mode_confirmed = mode == 0U;
  control_mode_abnormal = mode != 0U;
  control_mode_recheck_pending = false;
  SuspendControl();
  cal.reason = mode == 0U ? "control_data_invalid" : "mode_not_normal";
}

/* Communication availability can recover without re-zeroing a verified heading. */
static bool ControlAngleRefresh(HWT101_Angle_t *angle)
{
  HWT101_Angle_t latest;
  HWT101_Status_t comm;
  if (cal.busy || !cal.verified || !cal.normal_mode_confirmed ||
      cal.state != HWT101_CAL_DONE || strcmp(cal.result, "PASS") ||
      (native_run && !cal.save_readback_ok))
    return false;
  if (!HWT101_Status_Get(&comm) || !FreshAngle(&latest))
  {
    control_valid = false;
    control_recovery_samples = 0U;
    cal.reason = "control_data_invalid";
    return false;
  }
  if (comm.i2c_error_count != control_errors)
  {
    control_errors = comm.i2c_error_count;
    control_valid = false;
    control_recovery_samples = 0U;
    /* The frame visible when the fault is detected is not recovery evidence. */
    control_tick = latest.last_update_ms;
    control_count = latest.update_count;
    cal.reason = "control_data_invalid";
    return false;
  }
  if (latest.update_count != control_count)
  {
    if ((uint32_t)(latest.last_update_ms - control_tick) > HWT101_DATA_FRESH_MS)
    {
      control_valid = false;
      control_recovery_samples = 0U;
      cal.reason = "control_data_invalid";
    }
    control_tick = latest.last_update_ms;
    control_count = latest.update_count;
    if (!control_valid && ++control_recovery_samples >= 2U)
    {
      control_valid = true;
      control_recovery_samples = 0U;
      cal.reason = "control_data_recovered";
    }
  }
  if (!control_valid) return false;
  *angle = latest;
  return true;
}

void HWT101_Cal_Process(void)
{
  HAL_StatusTypeDef hal;
  HWT101_Status_t comm;
  HWT101_Angle_t angle;
  uint16_t value;
  uint32_t now = HAL_GetTick();
  if (boot_pending) { cal.elapsed_ms = (uint32_t)(now - run_tick); return; }
  (void)HWT101_Status_Get(&comm);
  if (cal.busy) cal.elapsed_ms = (uint32_t)(now - run_tick);
  if (cal.busy) cal.i2c_errors = (uint32_t)(comm.i2c_error_count - error_baseline);
  if (!cal.busy)
  {
    ControlModeRecheck();
    (void)ControlAngleRefresh(&angle);
    return;
  }
  if (cal.state == HWT101_CAL_RESTORING && aborting)
  {
    if ((uint32_t)(now - restore_tick) >= RESTORE_WAIT_MS)
    {
      cal.normal_mode_confirmed = control_valid = false;
      cal.reason = "normal_mode_unconfirmed";
      Finish(HWT101_CAL_FAILED);
      return;
    }
    /* HWT101_Process owns one-second reprobing and host BUSY recovery.
     * Keep the three exit attempts for an available link, not a disconnected bus. */
    if (!HWT101_Is_Ready()) return;
  }
  /* Only unsent SAVE writes fail immediately. Once accepted, readback shares
   * ordinary read recovery and cannot resend the write or restart verification. */
  if (!aborting && step != STEP_SAVE_UNLOCK && step != STEP_SAVE_WRITE)
  {
    if (comm.i2c_error_count != accepted_errors || !FreshAngle(&angle))
      BeginReadRecovery(&comm);
    if (cal.read_recovering && !ReadRecoveryProcess(&comm)) return;
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
      cal_tick = step_tick = native_poll_tick = HAL_GetTick();
      step = STEP_START_CHECK;
      break;
    case STEP_START_CHECK:
      hal = HWT101_ReadRegister(REG_MODE, &value);
      if (hal == HAL_OK) cal.mode = value;
      if (hal != HAL_OK) {
        (void)HWT101_Status_Get(&comm); BeginReadRecovery(&comm); break;
      }
      if (value != 1U && value != 2U)
      {
        (void)HWT101_Status_Get(&comm);
        BeginReadRecovery(&comm);
        /* This successful mismatch is already the first observation. Count it
         * and wait at least 100 ms before the read-only confirmation. */
        cal.recovery_mode_first = cal.recovery_mode_last = value;
        cal.recovery_mode_seen = cal.recovery_mode_valid = true;
        cal.recovery_mode_hal = HAL_OK;
        cal.recovery_mode_reads = 1U;
        cal.recovery_mode_tick = HAL_GetTick();
        recovery_mode_pending = true;
        cal.reason = "recovery_mode_recheck";
        break;
      }
      NativeModeObserve(value);
      SetState(HWT101_CAL_CALIBRATING); step = STEP_CAL_WAIT;
      break;
    case STEP_CAL_WAIT:
      if ((uint32_t)(now - cal_tick) >= cal.native_wait_ms) {
        cal.reason = "none";
        BeginRestore();
      }
      else if ((uint32_t)(now - native_poll_tick) >= 200U) {
        native_poll_tick = now;
        hal = HWT101_ReadRegister(REG_MODE, &value);
        if (hal == HAL_OK) {
          cal.mode = value;
          if (value == 1U || value == 2U ||
              (value == 0U && (cal.native_modes_seen & 0x04U))) {
            NativeModeObserve(value);
            break;
          }
        }
        (void)HWT101_Status_Get(&comm);
        BeginReadRecovery(&comm);
      }
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
      if (hal != HAL_OK) {
        if (aborting) ExitFailed(hal);
        else { (void)HWT101_Status_Get(&comm); BeginReadRecovery(&comm); }
        break;
      }
      if (value != 0U) { ExitFailed(hal); break; }
      cal.normal_mode_confirmed = true;
      if (aborting)
      { Finish(cancelled ? HWT101_CAL_CANCELLED : HWT101_CAL_FAILED); break; }
      /* Only faults already reconciled by read recovery may continue. */
      if (comm.i2c_error_count != accepted_errors)
      { cal.reason = "i2c"; cal.hal = HAL_ERROR; Finish(HWT101_CAL_FAILED); break; }
      step = STEP_BIAS_READ;
      break;
    case STEP_BIAS_READ:
      hal = HWT101_ReadRegister(REG_BIAS, &value);
      cal.bias_after_valid = hal == HAL_OK;
      if (hal != HAL_OK) {
        (void)HWT101_Status_Get(&comm); BeginReadRecovery(&comm); break;
      }
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
      if (comm.i2c_error_count != accepted_errors || !FreshAngle(&angle))
      { SaveFailed(HAL_ERROR); break; }
      hal = HWT101_WriteRegister(REG_KEY, 0xB588U);
      if (hal != HAL_OK) { SaveFailed(hal); break; }
      step = STEP_SAVE_WRITE; step_tick = HAL_GetTick();
      break;
    case STEP_SAVE_WRITE:
      if (comm.i2c_error_count != accepted_errors || !FreshAngle(&angle))
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
      if (hal != HAL_OK || value != 0U) {
        (void)HWT101_Status_Get(&comm); BeginReadRecovery(&comm); break;
      }
      hal = HWT101_ReadRegister(REG_BIAS, &value);
      cal.bias_after_valid = hal == HAL_OK;
      if (hal != HAL_OK) {
        (void)HWT101_Status_Get(&comm); BeginReadRecovery(&comm); break;
      }
      cal.bias_after = value;
      cal.save_readback_ok = true;
      cal.save_state = "READBACK_OK";
      cal.reason = "none";
      cal.hal = HAL_OK;
      /* 回读只能证实当前寄存器可读，断电保持需实物验证。 */
      if (FreshAngle(&angle) && comm.i2c_error_count == accepted_errors)
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
  return angle != NULL && ControlAngleRefresh(angle);
}
