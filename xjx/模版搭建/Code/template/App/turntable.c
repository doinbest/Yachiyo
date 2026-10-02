#include "turntable.h"
#include "motor_bus.h"
#include "motor_id_config.h"
#include "console_tx.h"
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Emm42 默认 1.8°/16细分：指令3200脉冲/圈，实时位置65536单位/圈。
 * 二者不可互换；车载转盘与电机1:1直连。此模块不修改驱动配置。 */
#define COMMAND_PULSES_PER_REV 3200.0
#define FEEDBACK_UNITS_PER_REV 65536.0
enum { SLOT1_DEG, SLOT2_DEG, SLOT3_DEG, SPEED_RPM, ACC, POS_TOL_DEG,
       STABLE_MS, FEEDBACK_MS, PARAM_COUNT };
static const char *const Names[PARAM_COUNT] = {
  "slot1_deg", "slot2_deg", "slot3_deg", "speed_rpm", "acc",
  "pos_tol_deg", "stable_ms", "feedback_ms"
};
static float Params[PARAM_COUNT];
static Turntable_Inventory_t Inventory[TURNTABLE_SLOT_COUNT];
static Turntable_Status_t Status;
typedef enum { OP_NONE, OP_ORIGIN, OP_INDEX, OP_JOG, OP_STOP } Operation;
typedef enum { TT_PHASE_SAMPLE, TT_PHASE_ENABLE, TT_PHASE_MOVE, TT_PHASE_STOP_FRAME } Phase;
static Operation Op;
static Phase Step;
static uint8_t SampleKind, Flags;
static int64_t PositionRaw, OriginRaw;
static double TargetRaw, RequestedJog, DeltaDeg;
static uint32_t Token, PendingToken, SampleTick[3], LastCycle, StableSince;
static uint32_t Started, MotionBudget, StopStarted;
static bool Pending, Stable, MotionStarted;
static const char *StopReason;

static float wrap(float degree)
{
  float value = fmodf(degree, 360.0f);
  return value < 0.0f ? value + 360.0f : value;
}
static double shortest(double degree)
{
  degree = fmod(degree, 360.0);
  if (degree > 180.0) degree -= 360.0;
  if (degree < -180.0) degree += 360.0;
  return degree;
}
static bool fresh(void)
{
  unsigned i;
  uint32_t now = HAL_GetTick();
  for (i = 0; i < 3; i++)
    if (!SampleTick[i] || (uint32_t)(now - SampleTick[i]) > (uint32_t)Params[FEEDBACK_MS])
      return false;
  return true;
}
static void reply(const char *format, ...)
{
  char line[256];
  int n;
  va_list ap;
  va_start(ap, format);
  n = vsnprintf(line, sizeof(line), format, ap);
  va_end(ap);
  if (n > 0 && n < (int)sizeof(line))
    (void)ConsoleTx_Write((const uint8_t *)line, (uint16_t)n);
}
static void finish(const char *reason, bool arrived)
{
  Op = OP_NONE;
  Pending = false;
  Status.state = arrived ? "arrived" : "idle";
  Status.reason = reason;
  Status.arrived = arrived;
  Status.feedback_valid = fresh() && !MotorBus_IsQuarantined();
  Status.moving = Status.feedback_valid && (fabsf(Status.speed_rpm) > 1.0f || !(Flags & 2U));
  MotorBus_Release(MOTOR_BUS_TURNTABLE);
}
static void stop_start(const char *reason)
{
  MotorBus_Cancel(MOTOR_BUS_TURNTABLE);
  MotorBus_Release(MOTOR_BUS_TURNTABLE);
  Op = OP_STOP;
  Step = TT_PHASE_STOP_FRAME;
  Pending = false;
  StopReason = reason;
  StopStarted = HAL_GetTick();
  Stable = false;
  Status.state = "stopping";
  Status.arrived = false;
  Status.feedback_valid = false;
  /* Pre-stop samples cannot confirm this stop, even if another priority stop
   * cancels its transmission. Keep reference/position, clear only evidence age. */
  memset(SampleTick, 0, sizeof(SampleTick));
}
static void fail(const char *reason)
{
  if (MotionStarted && Op != OP_STOP) stop_start(reason);
  else finish(reason, false);
}
void Turntable_Init(void)
{
  unsigned i;
  static const float Defaults[PARAM_COUNT] = {0, NAN, NAN, 30, 8, 1, 200, 2000};
  memcpy(Params, Defaults, sizeof(Params));
  memset(&Status, 0, sizeof(Status));
  memset(SampleTick, 0, sizeof(SampleTick));
  for (i = 0; i < TURNTABLE_SLOT_COUNT; i++) {
    Inventory[i].state = TURNTABLE_UNKNOWN;
    Inventory[i].color = 0;
  }
  Status.state = "idle";
  Status.reason = "not_referenced";
  Status.angle_deg = Status.target_deg = NAN;
  Op = OP_NONE;
  Pending = Stable = MotionStarted = false;
  Token = 0;
}
bool Turntable_IsBusy(void) { return Op != OP_NONE; }
bool Turntable_ReferenceValid(void) { return Status.reference_valid; }
bool Turntable_SlotConfigured(uint8_t slot)
{ return slot >= 1U && slot <= TURNTABLE_SLOT_COUNT && isfinite(Params[slot - 1U]); }
Turntable_Inventory_t Turntable_InventoryGet(uint8_t slot)
{
  Turntable_Inventory_t unknown = {TURNTABLE_UNKNOWN, 0};
  return slot >= 1U && slot <= TURNTABLE_SLOT_COUNT ? Inventory[slot - 1U] : unknown;
}
bool Turntable_InventorySet(uint8_t slot, Turntable_InventoryState_t state, uint8_t color)
{
  if (slot < 1U || slot > TURNTABLE_SLOT_COUNT || state > TURNTABLE_OCCUPIED ||
      (state == TURNTABLE_OCCUPIED && (color < 1U || color > 6U))) return false;
  Inventory[slot - 1U].state = state;
  Inventory[slot - 1U].color = state == TURNTABLE_OCCUPIED ? color : 0U;
  return true;
}
uint8_t Turntable_FindEmpty(void)
{
  uint8_t slot;
  for (slot = 1; slot <= TURNTABLE_SLOT_COUNT; slot++)
    if (Inventory[slot - 1U].state == TURNTABLE_EMPTY) return slot;
  return 0;
}
void Turntable_StatusGet(Turntable_Status_t *out)
{
  if (!out) return;
  *out = Status;
  out->feedback_valid = fresh() && !MotorBus_IsQuarantined();
}
static bool start(Operation op)
{
  if (Turntable_IsBusy()) { Status.reason = "busy"; return false; }
  if (MotorBus_IsQuarantined()) { Status.reason = "bus_locked"; return false; }
  if (!MotorBus_Reserve(MOTOR_BUS_TURNTABLE)) { Status.reason = "bus_busy"; return false; }
  Op = op;
  Step = TT_PHASE_SAMPLE;
  SampleKind = 0;
  memset(SampleTick, 0, sizeof(SampleTick));
  Pending = Stable = MotionStarted = false;
  Started = HAL_GetTick();
  LastCycle = Started - 50U;
  Status.arrived = Status.feedback_valid = false;
  Status.state = "reading";
  Status.reason = "accepted";
  return true;
}
bool Turntable_Origin(void)
{
  if (!start(OP_ORIGIN)) return false;
  Status.slot = 1;
  Status.target_deg = 0;
  return true;
}
bool Turntable_Index(uint8_t slot)
{
  if (!Status.reference_valid) { Status.reason = "reference_required"; return false; }
  if (!Turntable_SlotConfigured(slot)) { Status.reason = "slot_unset"; return false; }
  if (!start(OP_INDEX)) return false;
  Status.slot = slot;
  Status.target_deg = wrap(Params[slot - 1U]);
  return true;
}
bool Turntable_Jog(float degree)
{
  if (!isfinite(degree) || fabs((double)degree) * COMMAND_PULSES_PER_REV / 360.0 > 4294967295.0) {
    Status.reason = "invalid_degree"; return false;
  }
  if (!start(OP_JOG)) return false;
  RequestedJog = degree;
  Status.slot = 0;
  Status.target_deg = NAN;
  return true;
}
void Turntable_Stop(void)
{
  if (Op != OP_STOP) stop_start("stopped");
}
static bool submit(const uint8_t *data, uint8_t length, uint8_t reply_length, bool priority)
{
  PendingToken = ++Token;
  if (!MotorBus_Submit(MOTOR_BUS_TURNTABLE, data, length, reply_length, PendingToken, priority))
    return false;
  Pending = true;
  return true;
}
static void sampled(void)
{
  uint32_t now = HAL_GetTick();
  bool stationary = fabsf(Status.speed_rpm) <= 1.0f && !(Flags & 0x0cU);
  Status.feedback_valid = fresh();
  Status.moving = !stationary || !(Flags & 2U);
  if (Status.reference_valid)
    Status.angle_deg = wrap((float)((double)(PositionRaw - OriginRaw) * 360.0 / FEEDBACK_UNITS_PER_REV));
  LastCycle = now;
  /* Reserve only one complete position/speed/state sample, never the whole
   * rotation. Arm and wheel feedback must remain fresh during slow indexing. */
  if (MotionStarted || Op == OP_STOP) MotorBus_Release(MOTOR_BUS_TURNTABLE);
  if (Flags & 0x0cU) { fail("driver_fault"); return; }
  if (Op == OP_STOP) {
    /* Position/state/speed come from a complete post-stop cycle. */
    if (Status.feedback_valid && stationary && (Flags & 2U)) finish(StopReason, false);
    return;
  }
  if (Op == OP_ORIGIN) {
    if (!Status.feedback_valid) { finish("feedback_stale", false); return; }
    if (!stationary) { finish("motor_moving", false); return; }
    OriginRaw = PositionRaw;
    Status.reference_valid = true;
    Status.angle_deg = 0;
    finish("reference_established", false);
    return;
  }
  if (!MotionStarted) {
    double actual = (double)PositionRaw * 360.0 / FEEDBACK_UNITS_PER_REV;
    if (Op == OP_INDEX) {
      double relative = (double)(PositionRaw - OriginRaw) * 360.0 / FEEDBACK_UNITS_PER_REV;
      DeltaDeg = shortest((double)Status.target_deg - relative);
    } else DeltaDeg = RequestedJog;
    TargetRaw = actual + DeltaDeg;
    if (Op == OP_JOG && Status.reference_valid)
      Status.target_deg = wrap((float)((double)(PositionRaw - OriginRaw) * 360.0 / FEEDBACK_UNITS_PER_REV + DeltaDeg));
    MotionBudget = (uint32_t)(fabs(DeltaDeg) / ((double)Params[SPEED_RPM] * 6.0) * 1000.0) +
                   (uint32_t)Params[FEEDBACK_MS] + (uint32_t)Params[STABLE_MS];
    Step = TT_PHASE_ENABLE;
    return;
  }
  {
    double error = (double)PositionRaw * 360.0 / FEEDBACK_UNITS_PER_REV - TargetRaw;
    bool at_target = Status.feedback_valid && fabs(error) <= Params[POS_TOL_DEG] &&
                     stationary && (Flags & 3U) == 3U;
    if (!at_target) Stable = false;
    else if (!Stable) { Stable = true; StableSince = now; }
    else if ((uint32_t)(now - StableSince) >= (uint32_t)Params[STABLE_MS]) {
      finish("arrived", true);
      return;
    }
    if ((uint32_t)(now - Started) > MotionBudget) fail("position_timeout");
  }
}
void Turntable_Process(void)
{
  MotorBus_Event_t event;
  uint8_t cmd[13];
  uint32_t now = HAL_GetTick();
  if (MotorBus_EventGet(MOTOR_BUS_TURNTABLE, &event)) {
    /* Stop cancellation leaves an event from the old transaction; drain it only
     * here so neither mechanical arm nor chassis can consume this owner's event. */
    if (Pending && event.token == PendingToken) {
      Pending = false;
      if ((Step == TT_PHASE_STOP_FRAME && event.result != MOTOR_BUS_TX_DONE) ||
          (Step != TT_PHASE_STOP_FRAME && event.result != MOTOR_BUS_REPLY)) {
        Status.feedback_valid = false;
        fail(event.result == MOTOR_BUS_TIMEOUT ? "feedback_timeout" : "transport_error");
      } else if (Step == TT_PHASE_STOP_FRAME) {
        if (MotorBus_IsQuarantined()) { finish("stopped_unverified", false); return; }
        (void)MotorBus_Reserve(MOTOR_BUS_TURNTABLE);
        Step = TT_PHASE_SAMPLE;
        SampleKind = 0;
        memset(SampleTick, 0, sizeof(SampleTick));
        LastCycle = now - 50U;
      } else if (Step == TT_PHASE_ENABLE || Step == TT_PHASE_MOVE) {
        if (event.length != 4 || event.data[2] != 2U) { fail("driver_rejected"); return; }
        if (Step == TT_PHASE_ENABLE) Step = TT_PHASE_MOVE;
        else {
          Step = TT_PHASE_SAMPLE;
          SampleKind = 0;
          MotionStarted = true;
          Started = now;
          LastCycle = now;
          MotorBus_Release(MOTOR_BUS_TURNTABLE);
          Status.state = "moving";
          Status.moving = true;
          Status.reason = "monitoring";
        }
      } else {
        if (SampleKind == 0) {
          uint32_t magnitude = ((uint32_t)event.data[3] << 24) | ((uint32_t)event.data[4] << 16) |
                               ((uint32_t)event.data[5] << 8) | event.data[6];
          PositionRaw = event.data[2] ? -(int64_t)magnitude : (int64_t)magnitude;
        } else if (SampleKind == 1) {
          Status.speed_rpm = (float)(((unsigned)event.data[3] << 8) | event.data[4]);
          if (event.data[2]) Status.speed_rpm = -Status.speed_rpm;
        } else Flags = event.data[2];
        SampleTick[SampleKind] = event.tick_ms ? event.tick_ms : 1U;
        if (++SampleKind == 3) { SampleKind = 0; sampled(); }
      }
    }
  }
  if (Op == OP_NONE || Pending) return;
  if (Op == OP_STOP && (uint32_t)(now - StopStarted) > (uint32_t)Params[FEEDBACK_MS] + 500U) {
    finish("stop_unconfirmed", false); return;
  }
  if (MotorBus_IsQuarantined() && Step != TT_PHASE_STOP_FRAME) {
    fail("bus_locked"); return;
  }
  cmd[0] = MOTOR_ID_TURNTABLE;
  if (Step == TT_PHASE_STOP_FRAME) {
    cmd[1] = 0xfe; cmd[2] = 0x98; cmd[3] = 0; cmd[4] = 0x6b;
    (void)submit(cmd, 5, 0, true);
  } else if (Step == TT_PHASE_ENABLE) {
    cmd[1] = 0xf3; cmd[2] = 0xab; cmd[3] = 1; cmd[4] = 0; cmd[5] = 0x6b;
    (void)submit(cmd, 6, 4, false);
  } else if (Step == TT_PHASE_MOVE) {
    uint32_t pulses = (uint32_t)(fabs(DeltaDeg) * COMMAND_PULSES_PER_REV / 360.0 + 0.5);
    unsigned rpm = (unsigned)Params[SPEED_RPM];
    cmd[1] = 0xfd; cmd[2] = DeltaDeg < 0.0; cmd[3] = (uint8_t)(rpm >> 8); cmd[4] = (uint8_t)rpm;
    cmd[5] = (uint8_t)Params[ACC]; cmd[6] = (uint8_t)(pulses >> 24); cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8); cmd[9] = (uint8_t)pulses;
    cmd[10] = 2; cmd[11] = 0; cmd[12] = 0x6b; /* relative to current realtime position */
    /* Once copied for transmission, a lost/rejected ACK cannot prove the motor
     * stayed still. The failure path must issue Stop as well as report failure. */
    if (submit(cmd, 13, 4, false)) MotionStarted = true;
  } else if (SampleKind || (uint32_t)(now - LastCycle) >= 50U) {
    static const uint8_t functions[] = {0x36, 0x35, 0x3a};
    static const uint8_t lengths[] = {8, 6, 4};
    if (!SampleKind && !MotorBus_Reserve(MOTOR_BUS_TURNTABLE)) return;
    cmd[1] = functions[SampleKind]; cmd[2] = 0x6b;
    (void)submit(cmd, 3, lengths[SampleKind], false);
  }
}
static int key_index(const char *key)
{
  unsigned i;
  for (i = 0; i < PARAM_COUNT; i++) if (!strcmp(key, Names[i])) return (int)i;
  return -1;
}
static bool number(const char *text, float *value)
{
  char *end;
  errno = 0;
  *value = strtof(text, &end);
  return !errno && end != text && !*end && isfinite(*value);
}
static bool parameter_valid(int key, float value)
{
  if (key == SLOT2_DEG || key == SLOT3_DEG) return value >= -360.0f && value <= 360.0f;
  if (key == SPEED_RPM) return value >= 1 && value <= 5000 && value == floorf(value);
  if (key == ACC) return value >= 0 && value <= 255 && value == floorf(value);
  if (key == POS_TOL_DEG) return value >= 0.1125f && value <= 180;
  if (key == STABLE_MS) return value >= 0 && value <= 60000 && value == floorf(value);
  if (key == FEEDBACK_MS) return value >= 100 && value <= 60000 && value == floorf(value);
  return false;
}
static const char *inventory_name(Turntable_InventoryState_t state)
{ return state == TURNTABLE_EMPTY ? "empty" : state == TURNTABLE_OCCUPIED ? "occupied" : "unknown"; }
static void inventory_reply(uint8_t slot)
{
  Turntable_Inventory_t item = Turntable_InventoryGet(slot);
  reply("OK turntable inventory slot=%u state=%s color=%u\r\n", (unsigned)slot,
        inventory_name(item.state), (unsigned)item.color);
}
bool Turntable_Command(unsigned count, char *tokens[])
{
  float value;
  int key;
  bool accepted;
  if (!count || strcmp(tokens[0], "turntable")) return false;
  if (count == 3 && !strcmp(tokens[1], "get")) {
    key = key_index(tokens[2]);
    if (key < 0) reply("ERR turntable unknown_key key=%s\r\n", tokens[2]);
    else if (!isfinite(Params[key])) reply("OK turntable config key=%s value=unset\r\n", Names[key]);
    else reply("OK turntable config key=%s value=%.6g\r\n", Names[key], (double)Params[key]);
  } else if (count == 4 && !strcmp(tokens[1], "set")) {
    key = key_index(tokens[2]);
    if (key < 0) reply("ERR turntable unknown_key key=%s\r\n", tokens[2]);
    else if (key == SLOT1_DEG) reply("ERR turntable readonly key=%s\r\n", Names[key]);
    else if (Turntable_IsBusy()) reply("ERR turntable busy key=%s\r\n", Names[key]);
    else if (!number(tokens[3], &value) || !parameter_valid(key, value))
      reply("ERR turntable invalid_parameter key=%s\r\n", Names[key]);
    else { Params[key] = value; reply("OK turntable set key=%s\r\n", Names[key]); }
  } else if (count == 2 && !strcmp(tokens[1], "status")) {
    Turntable_Status_t s;
    Turntable_StatusGet(&s);
    reply("OK turntable status state=%s ref=%u slot=%u arrived=%u moving=%u fresh=%u angle_deg=%.6g target_deg=%.6g speed_rpm=%.6g reason=%s\r\n",
          s.state, s.reference_valid, (unsigned)s.slot, s.arrived, s.moving, s.feedback_valid,
          (double)s.angle_deg, (double)s.target_deg, (double)s.speed_rpm, s.reason);
    for (uint8_t slot = 1; slot <= TURNTABLE_SLOT_COUNT; slot++) inventory_reply(slot);
  } else if (count == 2 && !strcmp(tokens[1], "origin")) {
    accepted = Turntable_Origin();
    reply("%s turntable %s action=origin\r\n", accepted ? "OK" : "ERR", accepted ? "accepted" : Status.reason);
  } else if (count == 3 && !strcmp(tokens[1], "index")) {
    accepted = number(tokens[2], &value) && value >= 1 && value <= 3 && value == floorf(value);
    if (accepted) accepted = Turntable_Index((uint8_t)value);
    else Status.reason = "invalid_slot";
    reply("%s turntable %s action=index\r\n", accepted ? "OK" : "ERR", accepted ? "accepted" : Status.reason);
  } else if (count == 3 && !strcmp(tokens[1], "jog")) {
    accepted = number(tokens[2], &value);
    if (accepted) accepted = Turntable_Jog(value);
    else Status.reason = "invalid_degree";
    reply("%s turntable %s action=jog\r\n", accepted ? "OK" : "ERR", accepted ? "accepted" : Status.reason);
  } else if (count == 2 && !strcmp(tokens[1], "stop")) {
    Turntable_Stop(); reply("OK turntable accepted action=stop\r\n");
  } else if (count >= 2 && !strcmp(tokens[1], "inventory")) {
    if (count == 2 || (count == 3 && !strcmp(tokens[2], "empty"))) {
      if (count == 3 && Turntable_IsBusy()) reply("ERR turntable busy\r\n");
      else for (uint8_t slot = 1; slot <= TURNTABLE_SLOT_COUNT; slot++) {
        if (count == 3) (void)Turntable_InventorySet(slot, TURNTABLE_EMPTY, 0);
        inventory_reply(slot);
      }
    } else if (count == 4 && number(tokens[2], &value) && value >= 1 && value <= 3 && value == floorf(value)) {
      uint8_t slot = (uint8_t)value;
      Turntable_InventoryState_t state = TURNTABLE_UNKNOWN;
      uint8_t color = 0;
      accepted = !Turntable_IsBusy();
      if (!strcmp(tokens[3], "empty")) state = TURNTABLE_EMPTY;
      else if (strcmp(tokens[3], "unknown")) {
        state = TURNTABLE_OCCUPIED;
        accepted = accepted && number(tokens[3], &value) && value >= 1 && value <= 6 && value == floorf(value);
        if (accepted) color = (uint8_t)value;
      }
      if (accepted && Turntable_InventorySet(slot, state, color)) inventory_reply(slot);
      else reply("ERR turntable %s\r\n", Turntable_IsBusy() ? "busy" : "invalid_inventory");
    } else reply("ERR turntable usage\r\n");
  } else reply("ERR turntable usage\r\n");
  return true;
}
