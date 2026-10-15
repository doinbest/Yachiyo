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
 * 二者不可互换；车载转盘与电机1:1直连。固定分度只在RAM选择Both应答。 */
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
typedef enum { OP_NONE, OP_ORIGIN, OP_INDEX, OP_JOG, OP_STOP, OP_POWER, OP_SAVE_ZERO, OP_HOME, OP_VERIFY, OP_ANGLE } Operation;
typedef enum { TT_PHASE_SAMPLE, TT_PHASE_ENABLE, TT_PHASE_MOVE, TT_PHASE_STOP_FRAME, TT_PHASE_SAVE_ZERO, TT_PHASE_HOME, TT_PHASE_ABORT_HOME } Phase;
static Operation Op;
static Phase Step;
static uint8_t SampleKind, Flags, HomeFlags;
static bool HomeSampleValid, HomeDegraded;
static int64_t PositionRaw, OriginRaw, HomeLastRaw;
static double TargetRaw, RequestedJog, DeltaDeg;
static uint32_t Token, PendingToken, SampleTick[3], LastCycle, StableSince;
static uint32_t Started, MotionBudget, StopStarted;
static bool Pending, Stable, MotionStarted, PowerEnabled, BootHomePending;
static const char *StopReason;
static bool PositiveHalfTurn;
static bool ArrivalReply;
static uint32_t ArrivalReplyTick;
static uint32_t PositionAccepted;

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
  /* HOME confirms standstill using its own completion flag, not position bit1. */
  Status.moving = Status.feedback_valid && (fabsf(Status.speed_rpm) > 1.0f || (!arrived && !(Flags & 2U)));
  MotorBus_Release(MOTOR_BUS_TURNTABLE);
}
static void stop_start(const char *reason)
{
  MotorBus_Cancel(MOTOR_BUS_TURNTABLE);
  MotorBus_Release(MOTOR_BUS_TURNTABLE);
  Step = Op == OP_HOME ? TT_PHASE_ABORT_HOME : TT_PHASE_STOP_FRAME;
  Op = OP_STOP;
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
  MotorBus_Diagnostic_t bus;
  MotorBus_DiagnosticGet(&bus);
  /* Report the first failing action immediately; subsequent rejected commands
   * and stop verification must not be the user's only evidence. */
  reply("ERR turntable fault reason=%s bus_locked=%u bus_reason=%s addr=%u func=0x%02X uart_error=0x%08lX\r\n",
        reason,(unsigned)bus.locked,bus.locked?bus.reason:"none",
        (unsigned)(bus.locked?bus.address:MOTOR_ID_TURNTABLE),
        (unsigned)(bus.locked?bus.function:0U),
        (unsigned long)(bus.locked?bus.uart_error:0U));
  if (MotionStarted && Op != OP_STOP) stop_start(reason);
  else finish(reason, false);
}
void Turntable_Init(void)
{
  /* Fixed index/jog/angle actions prepare Both in start(); boot HOME does not. */
  unsigned i;
  static const float Defaults[PARAM_COUNT] = {0, 90, 180, 30, 8, 2.5f, 200, 2000};
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
  BootHomePending = true; /* The boot coordinator releases the one-shot near HOME. */
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
  out->feedback_valid = (fresh() || (ArrivalReply && HAL_GetTick()-ArrivalReplyTick<=Params[FEEDBACK_MS])) && !MotorBus_IsQuarantined();
}
static bool start(Operation op)
{
  if (Turntable_IsBusy()) { Status.reason = "busy"; return false; }
  ArrivalReply=false;
  if (MotorBus_IsQuarantined()) { Status.reason = "bus_locked"; return false; }
  if(op==OP_INDEX || op==OP_ANGLE || op==OP_JOG) {
    int ready=MotorBus_PositionReplyEnsure(MOTOR_ID_TURNTABLE);
    if(ready!=1){Status.reason=ready<0?"reply_mode_failed":"bus_busy";return false;}
  }
  if (!MotorBus_Reserve(MOTOR_BUS_TURNTABLE)) { Status.reason = "bus_busy"; return false; }
  BootHomePending = false; /* An accepted explicit operation owns the turntable. */
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
  /* Slot 1 is the verified saved mechanical zero. Use HOME completion rather
   * than a near-zero position command's reached flag; preserve software stock. */
  if (slot == 1U) {
    if (!Turntable_Home()) return false;
    Status.slot = 1;
    Status.target_deg = 0;
    return true;
  }
  if (!start(OP_INDEX)) return false;
  Status.slot = slot;
  Status.target_deg = wrap(Params[slot - 1U]);
  return true;
}
bool Turntable_VerifySlot(uint8_t slot)
{
  if(!Status.reference_valid){Status.reason="reference_required";return false;}
  if(!Turntable_SlotConfigured(slot)){Status.reason="slot_unset";return false;}
  if(!start(OP_VERIFY))return false;
  Status.slot=slot;Status.target_deg=wrap(Params[slot-1U]);return true;
}
bool Turntable_SlotAngleGet(uint8_t slot,float *out)
{
  if(!out || !Turntable_SlotConfigured(slot))return false;
  *out=wrap(Params[slot-1U]);return true;
}
bool Turntable_AngleStart(float angle,uint8_t slot,bool positive_half_turn)
{
  if(!Status.reference_valid){Status.reason="reference_required";return false;}
  if(!isfinite(angle) || slot>TURNTABLE_SLOT_COUNT){Status.reason="invalid_degree";return false;}
  if(!start(OP_ANGLE))return false;
  Status.slot=slot;Status.target_deg=wrap(angle);PositiveHalfTurn=positive_half_turn;return true;
}
bool Turntable_AngleVerify(float angle,uint8_t slot)
{
  if(!Status.reference_valid){Status.reason="reference_required";return false;}
  if(!isfinite(angle) || slot>TURNTABLE_SLOT_COUNT){Status.reason="invalid_degree";return false;}
  if(!start(OP_VERIFY))return false;
  Status.slot=slot;Status.target_deg=wrap(angle);return true;
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
bool Turntable_SaveZero(void)
{
  return start(OP_SAVE_ZERO);
}
bool Turntable_Home(void)
{
  if (!start(OP_HOME)) return false;
  Step = TT_PHASE_ENABLE;
  HomeLastRaw = PositionRaw;
  HomeSampleValid = HomeDegraded = false;
  return true;
}
bool Turntable_Enable(bool enabled)
{
  if (!start(OP_POWER)) return false;
  PowerEnabled = enabled;
  Step = TT_PHASE_ENABLE;
  Status.state = enabled ? "enabling" : "disabling";
  return true;
}
void Turntable_BootHomeProcess(bool startup_allowed)
{
  if (!BootHomePending || !startup_allowed) return;
  if (Turntable_Home()) reply("OK turntable boot_home accepted mode=near\r\n");
  else if (MotorBus_IsQuarantined()) {
    BootHomePending = false;
    reply("ERR turntable boot_home reason=bus_locked\r\n");
  } /* A draining shared-bus transaction is retried by the next main loop. */
}
void Turntable_Stop(void)
{
  ArrivalReply=false;
  BootHomePending = false;
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
    if (Status.feedback_valid && stationary) {
      finish(StopReason, false);
      /* Fresh post-stop zero speed is authoritative even without position bit1. */
      Status.moving = false;
    }
    return;
  }
  if (Op == OP_VERIFY) {
    bool ready=Status.feedback_valid && stationary && (Flags & 1U) &&
      fabs(shortest((double)Status.target_deg-Status.angle_deg))<=Params[POS_TOL_DEG];
    finish(ready?"arrived":"position_changed",ready);return;
  }
  if (Op == OP_SAVE_ZERO) {
    if (!stationary) { finish("motor_moving", false); return; }
    Step = TT_PHASE_SAVE_ZERO;
    return;
  }
  if (Op == OP_HOME) {
    if (HomeFlags & 0x30U) { fail("home_driver_fault"); return; }
    if (HomeFlags & 8U) HomeDegraded = true;
    bool settled = Status.feedback_valid && HomeSampleValid && !(HomeFlags & 4U) &&
                   stationary && (!HomeDegraded || Status.speed_rpm == 0.0f) && (Flags & 1U) &&
                   fabs((double)(PositionRaw - HomeLastRaw) * 360.0 / FEEDBACK_UNITS_PER_REV) <= Params[POS_TOL_DEG];
    HomeLastRaw = PositionRaw;
    HomeSampleValid = Status.feedback_valid;
    if (!settled) Stable = false;
    else if (!Stable) { Stable = true; StableSince = now; }
    else if ((uint32_t)(now - StableSince) >= (uint32_t)Params[STABLE_MS]) {
      OriginRaw = PositionRaw;
      Status.reference_valid = true;
      Status.slot = 1;
      Status.angle_deg = Status.target_deg = 0;
      Status.reference_degraded = HomeDegraded;
      finish(HomeDegraded ? "home_approximate" : "home_complete", true);
      reply("OK turntable home complete ref=1 ref_degraded=%u home_flags=0x%02X reference=%s\r\n",
            HomeDegraded ? 1U : 0U, (unsigned)HomeFlags, HomeDegraded ? "approximate" : "verified");
      return;
    }
    if ((uint32_t)(now - Started) > 60000U) fail("home_timeout");
    return;
  }
  if (Op == OP_POWER) {
    bool matched = ((Flags & 1U) != 0U) == PowerEnabled;
    finish(matched ? (PowerEnabled ? "enabled" : "disabled") : "enable_state_mismatch", false);
    reply("%s turntable %s action=%s en=%u\r\n", matched ? "OK" : "ERR",
          matched ? "confirmed" : "enable_state_mismatch", PowerEnabled ? "enable" : "disable",
          (unsigned)((Flags & 1U) != 0U));
    return;
  }
  if (Op == OP_ORIGIN) {
    if (!Status.feedback_valid) { finish("feedback_stale", false); return; }
    if (!stationary) { finish("motor_moving", false); return; }
    OriginRaw = PositionRaw;
    Status.reference_degraded = false;
    Status.reference_valid = true;
    Status.angle_deg = 0;
    finish("reference_established", false);
    return;
  }
  if (!MotionStarted) {
    double actual = (double)PositionRaw * 360.0 / FEEDBACK_UNITS_PER_REV;
    if (Op == OP_INDEX || Op == OP_ANGLE) {
      double relative = (double)(PositionRaw - OriginRaw) * 360.0 / FEEDBACK_UNITS_PER_REV;
      DeltaDeg = shortest((double)Status.target_deg - relative);
      /* At the opposite tray, measured arrival error must not flip the
       * requested positive half-turn. Resume still targets the same angle. */
      if(Op==OP_ANGLE && PositiveHalfTurn && DeltaDeg<0 &&
         fabs(fabs(DeltaDeg)-180.0)<=Params[POS_TOL_DEG])DeltaDeg+=360.0;
    } else DeltaDeg = RequestedJog;
    TargetRaw = actual + DeltaDeg;
    if (Op == OP_JOG && Status.reference_valid)
      Status.target_deg = wrap((float)((double)(PositionRaw - OriginRaw) * 360.0 / FEEDBACK_UNITS_PER_REV + DeltaDeg));
    /* Cruise time alone omits the drive's acceleration/deceleration and the
     * shared-bus arrival samples. Allow twice cruise time, with a 5 s floor.
     * This is a wait budget, not a change to the commanded angle or speed. */
    MotionBudget = (uint32_t)fmax(5000.0,
      fabs(DeltaDeg) / ((double)Params[SPEED_RPM] * 6.0) * 2000.0 +
      Params[FEEDBACK_MS] + Params[STABLE_MS]);
    Step = TT_PHASE_ENABLE;
    return;
  }
  {
    double error = (double)PositionRaw * 360.0 / FEEDBACK_UNITS_PER_REV - TargetRaw;
    /* The host tolerance is the acceptance window. The drive's narrower
     * reached bit may remain clear; keep its exact position command active. */
    bool at_target = Status.feedback_valid && fabs(error) <= Params[POS_TOL_DEG] &&
                     stationary && (Flags & 1U);
    if (!at_target) Stable = false;
    else if (!Stable) { Stable = true; StableSince = now; }
    else if ((uint32_t)(now - StableSince) >= (uint32_t)Params[STABLE_MS]) {
      finish("arrived", true);
      reply("OK turntable arrived angle_deg=%.6g target_deg=%.6g speed_rpm=%.6g ref=%u\r\n",
            (double)Status.angle_deg,(double)Status.target_deg,
            (double)Status.speed_rpm,(unsigned)Status.reference_valid);
      return;
    }
    if ((uint32_t)(now - Started) > MotionBudget) {
      reply("ERR turntable position_timeout angle_deg=%.6g target_deg=%.6g error_deg=%.6g tol_deg=%.6g speed_rpm=%.6g flags=0x%02X fresh=%u elapsed_ms=%lu budget_ms=%lu\r\n",
        (double)Status.angle_deg,(double)Status.target_deg,error,(double)Params[POS_TOL_DEG],
        (double)Status.speed_rpm,(unsigned)Flags,(unsigned)Status.feedback_valid,
        (unsigned long)(now-Started),(unsigned long)MotionBudget);
      fail("position_timeout");
    }
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
      if (((Step == TT_PHASE_STOP_FRAME || Step == TT_PHASE_ABORT_HOME) && event.result != MOTOR_BUS_TX_DONE) ||
          (Step != TT_PHASE_STOP_FRAME && Step != TT_PHASE_ABORT_HOME && event.result != MOTOR_BUS_REPLY)) {
        Status.feedback_valid = false;
        fail(event.result == MOTOR_BUS_TIMEOUT ? "feedback_timeout" : "transport_error");
      } else if (Step == TT_PHASE_ABORT_HOME) {
        Step = TT_PHASE_STOP_FRAME;
      } else if (Step == TT_PHASE_SAVE_ZERO) {
        if (event.length != 4 || event.data[2] != 2U) { fail("driver_rejected"); return; }
        OriginRaw = PositionRaw;
        Status.reference_degraded = false;
        Status.reference_valid = true;
        Status.slot = 1;
        Status.angle_deg = Status.target_deg = 0;
        finish("mechanical_zero_saved", false);
        reply("OK turntable zero_saved stored=1 ref=1\r\n");
      } else if (Step == TT_PHASE_STOP_FRAME) {
        if (MotorBus_IsQuarantined()) { finish("stopped_unverified", false); return; }
        (void)MotorBus_Reserve(MOTOR_BUS_TURNTABLE);
        Step = TT_PHASE_SAMPLE;
        SampleKind = 0;
        memset(SampleTick, 0, sizeof(SampleTick));
        LastCycle = now - 50U;
      } else if (Step == TT_PHASE_ENABLE || Step == TT_PHASE_MOVE || Step == TT_PHASE_HOME) {
        if (event.length != 4 || (event.data[2] != 2U && !(Step==TT_PHASE_MOVE && event.data[2]==0x9fU))) { fail("driver_rejected"); return; }
        if (Step == TT_PHASE_ENABLE && Op == OP_POWER) {
          Step = TT_PHASE_SAMPLE;
          SampleKind = 0;
          LastCycle = now - 50U;
        } else if (Step == TT_PHASE_ENABLE) Step = Op == OP_HOME ? TT_PHASE_HOME : TT_PHASE_MOVE;
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
        if(event.length==4 && (event.data[2]==0xe2 || event.data[2]==0xee)) { fail("driver_rejected");return; }
        if (SampleKind == 0) {
          uint32_t magnitude = ((uint32_t)event.data[3] << 24) | ((uint32_t)event.data[4] << 16) |
                               ((uint32_t)event.data[5] << 8) | event.data[6];
          PositionRaw = event.data[2] ? -(int64_t)magnitude : (int64_t)magnitude;
        } else if (SampleKind == 1) {
          Status.speed_rpm = (float)(((unsigned)event.data[3] << 8) | event.data[4]);
          if (event.data[2]) Status.speed_rpm = -Status.speed_rpm;
        } else if (SampleKind == 2) Flags = event.data[2];
        else HomeFlags = event.data[2];
        if (SampleKind < 3) SampleTick[SampleKind] = event.tick_ms ? event.tick_ms : 1U;
        if (++SampleKind == (Op == OP_HOME ? 4 : 3)) { SampleKind = 0; sampled(); }
      }
    }
  }
  if (Op == OP_NONE || Pending) return;
  if(MotionStarted && (Op==OP_INDEX || Op==OP_ANGLE || Op==OP_JOG) &&
     MotorBus_PositionReached(MOTOR_ID_TURNTABLE,PositionAccepted)) {
    ArrivalReply=true;ArrivalReplyTick=now;
    finish("arrived_reply",true);Status.feedback_valid=true;Status.moving=false;
    reply("OK turntable arrived source=reached_reply target_deg=%.6g ref=%u\r\n",
      (double)Status.target_deg,(unsigned)Status.reference_valid);return;
  }
  if (Op == OP_STOP && (uint32_t)(now - StopStarted) > (uint32_t)Params[FEEDBACK_MS] + 500U) {
    finish("stop_unconfirmed", false); return;
  }
  if (MotorBus_IsQuarantined() && Step != TT_PHASE_STOP_FRAME && Step != TT_PHASE_ABORT_HOME) {
    fail("bus_locked"); return;
  }
  cmd[0] = MOTOR_ID_TURNTABLE;
  if (Step == TT_PHASE_ABORT_HOME) {
    cmd[1] = 0x9c; cmd[2] = 0x48; cmd[3] = 0x6b;
    (void)submit(cmd, 4, 0, true);
  } else if (Step == TT_PHASE_SAVE_ZERO) {
    cmd[1] = 0x93; cmd[2] = 0x88; cmd[3] = 1; cmd[4] = 0x6b;
    (void)submit(cmd, 5, 4, false);
  } else if (Step == TT_PHASE_HOME) {
    cmd[1] = 0x9a; cmd[2] = 0; cmd[3] = 0; cmd[4] = 0x6b;
    if (submit(cmd, 5, 4, false)) MotionStarted = true;
  } else if (Step == TT_PHASE_STOP_FRAME) {
    cmd[1] = 0xfe; cmd[2] = 0x98; cmd[3] = 0; cmd[4] = 0x6b;
    (void)submit(cmd, 5, 0, true);
  } else if (Step == TT_PHASE_ENABLE) {
    cmd[1] = 0xf3; cmd[2] = 0xab; cmd[3] = Op == OP_POWER ? (uint8_t)PowerEnabled : 1; cmd[4] = 0; cmd[5] = 0x6b;
    (void)submit(cmd, 6, 4, false);
  } else if (Step == TT_PHASE_MOVE) {
    double target_pulses = floor(fabs(TargetRaw) * COMMAND_PULSES_PER_REV / 360.0 + 0.5);
    if (target_pulses > 4294967295.0) { fail("invalid_degree"); return; }
    uint32_t pulses = (uint32_t)target_pulses;
    unsigned rpm = (unsigned)Params[SPEED_RPM];
    cmd[1] = 0xfd; cmd[2] = TargetRaw < 0.0; cmd[3] = (uint8_t)(rpm >> 8); cmd[4] = (uint8_t)rpm;
    cmd[5] = (uint8_t)Params[ACC]; cmd[6] = (uint8_t)(pulses >> 24); cmd[7] = (uint8_t)(pulses >> 16);
    cmd[8] = (uint8_t)(pulses >> 8); cmd[9] = (uint8_t)pulses;
    /* Compute the target from a fresh sample, then use documented absolute
     * mode 1. Mode 2 has repeatedly behaved as absolute on this car. */
    cmd[10] = 1; cmd[11] = 0; cmd[12] = 0x6b;
    /* Once copied for transmission, a lost/rejected ACK cannot prove the motor
     * stayed still. The failure path must issue Stop as well as report failure. */
    if (submit(cmd, 13, 4, false)) {MotionStarted = true;PositionAccepted=now;}
  } else if (SampleKind || (uint32_t)(now - LastCycle) >= 50U) {
    static const uint8_t functions[] = {0x36, 0x35, 0x3a, 0x3b};
    static const uint8_t lengths[] = {8, 6, 4, 4};
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
    reply("OK turntable status state=%s ref=%u slot=%u arrived=%u moving=%u fresh=%u angle_deg=%.6g target_deg=%.6g speed_rpm=%.6g reason=%s ref_degraded=%u\r\n",
          s.state, s.reference_valid, (unsigned)s.slot, s.arrived, s.moving, s.feedback_valid,
          (double)s.angle_deg, (double)s.target_deg, (double)s.speed_rpm, s.reason, s.reference_degraded ? 1U : 0U);
    for (uint8_t slot = 1; slot <= TURNTABLE_SLOT_COUNT; slot++) inventory_reply(slot);
  } else if (count == 2 && (!strcmp(tokens[1], "save_zero") || !strcmp(tokens[1], "home"))) {
    accepted = !strcmp(tokens[1], "home") ? Turntable_Home() : Turntable_SaveZero();
    reply("%s turntable %s action=%s\r\n", accepted ? "OK" : "ERR",
          accepted ? "accepted" : Status.reason, tokens[1]);
  } else if (count == 2 && (!strcmp(tokens[1], "enable") || !strcmp(tokens[1], "disable"))) {
    accepted = Turntable_Enable(!strcmp(tokens[1], "enable"));
    reply("%s turntable %s action=%s\r\n", accepted ? "OK" : "ERR",
          accepted ? "accepted" : Status.reason, tokens[1]);
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
