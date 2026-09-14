#include "chassis_telemetry.h"
#include "chassis_config.h"
#include "chassis_motion.h"
#include "chassis_localization.h"
#include "chassis_route.h"
#include "mecanum_chassis.h"
#include "console_tx.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>
static const ChassisModel_Geometry_t geometry = {CHASSIS_WHEELBASE_MM, CHASSIS_TRACK_WIDTH_MM,
                                                 CHASSIS_WHEEL_DIAMETER_MM,
                                                 CHASSIS_MOTOR_TO_WHEEL_RATIO};
static const uint8_t dirs[4] = {
    CHASSIS_MOTOR_FRONT_LEFT_FORWARD_DIR, CHASSIS_MOTOR_REAR_LEFT_FORWARD_DIR,
    CHASSIS_MOTOR_REAR_RIGHT_FORWARD_DIR, CHASSIS_MOTOR_FRONT_RIGHT_FORWARD_DIR};
static bool streaming;
#define TELEMETRY_PERIOD_MS 2000U
static uint32_t session_id, last_emit, trace_sequence;
static char line[2048];
static size_t length;
static bool format_ok;
static const char *boolean(bool value)
{
  return value ? "true" : "false";
}
static void reply(const char *text)
{
  (void)ConsoleTx_Write((const uint8_t *)text, (uint16_t)strlen(text));
}
static void append(const char *format, ...)
{
  int n;
  va_list args;
  if (!format_ok)
    return;
  va_start(args, format);
  n = vsnprintf(line + length, sizeof(line) - length, format, args);
  va_end(args);
  if (n < 0 || (size_t)n >= sizeof(line) - length)
  {
    format_ok = false;
    return;
  }
  length += (size_t)n;
}
static void begin(void)
{
  length = 0;
  format_ok = true;
}
static bool config(void)
{
  ChassisLocalization_Status_t localization;
  ChassisLocalization_Get(&localization);
  begin();
  append("\r\n@CHASSIS {\"v\":1,\"kind\":\"config\",\"session\":%lu,\"rev\":\"%s\",",
         (unsigned long)session_id, CHASSIS_CONFIG_REVISION);
  append("\"telemetry_period_ms\":%lu,", (unsigned long)TELEMETRY_PERIOD_MS);
  append("\"geometry\":{\"wheelbase_mm\":%.2f,\"track_mm\":%.2f,\"wheel_diameter_mm\":%.2f,\"gear_"
         "ratio\":%.3f},",
         (double)geometry.wheelbase_mm, (double)geometry.track_mm,
         (double)geometry.wheel_diameter_mm, (double)geometry.gear_ratio);
  append("\"wheel_ids\":[%u,%u,%u,%u],\"forward_dir\":[%u,%u,%u,%u],", CHASSIS_MOTOR_ID_FRONT_LEFT,
         CHASSIS_MOTOR_ID_REAR_LEFT, CHASSIS_MOTOR_ID_REAR_RIGHT, CHASSIS_MOTOR_ID_FRONT_RIGHT,
         dirs[0], dirs[1], dirs[2], dirs[3]);
  append("\"geometry_status\":{\"wheelbase\":\"%s\",\"track\":\"%s\",\"diameter\":"
         "\"%s\",\"gear_ratio\":\"%s\"},\"position_units_per_rev\":%lu,\"feedback_sign\":"
         "\"motor_raw\"}\r\n",
         CHASSIS_WHEELBASE_APPROXIMATE ? "estimated" : "measured",
         CHASSIS_TRACK_APPROXIMATE ? "estimated" : "measured",
         CHASSIS_DIAMETER_ASSUMED ? "assumed" : "measured",
         CHASSIS_GEAR_RATIO_ASSUMED ? "assumed" : "confirmed",
         (unsigned long)localization.units_per_rev);
  return format_ok && ConsoleTx_Write((const uint8_t *)line, (uint16_t)length);
}
void ChassisTelemetry_Init(void)
{
  streaming = false;
  session_id = last_emit = trace_sequence = 0;
  ChassisLocalization_Init();
}
bool ChassisTelemetry_Stream(bool on, uint32_t session)
{
  if (!on)
  {
    streaming = false;
    ConsoleTx_TelemetryCancel();
    return true;
  }
  if (!session)
    return false;
  session_id = session;
  ConsoleTx_TelemetryCancel();
  /* Let configuration/command replies drain before the first status snapshot. */
  last_emit = HAL_GetTick();
  trace_sequence++;
  /* Re-subscribing starts a new drawn segment but preserves the current estimate. */
  streaming = config();
  return streaming;
}
bool ChassisTelemetry_Origin(float x, float y, float yaw)
{
  if (!ChassisLocalization_Origin(x, y, yaw))
    return false;
  trace_sequence++;
  return true;
}
bool ChassisTelemetry_ConfirmPositionUnits(uint32_t value)
{
  if (!ChassisLocalization_ConfirmPositionUnits(value))
    return false;
  if (streaming)
    return config();
  return true;
}
void ChassisTelemetry_Process(void)
{
  /* Main-loop only snapshots keep formatting off the small interrupt/main stack. */
  static ChassisMotion_Status_t motion;
  static Mecanum_Status_t tx;
  static ChassisLocalization_Status_t localization;
  static ChassisRoute_Status_t route;
  const Mecanum_Feedback_t *f;
  uint32_t now = HAL_GetTick();
  static const char *states[] = {"idle", "running", "stopping", "done", "error"};
  if (!streaming || (uint32_t)(now - last_emit) < TELEMETRY_PERIOD_MS ||
      !ConsoleTx_TelemetryReady())
    return;
  ChassisMotion_StatusGet(&motion);
  Mecanum_StatusGet(&tx);
  ChassisLocalization_Get(&localization);
  ChassisRoute_StatusGet(&route);
  f = localization.wheels;
  last_emit = now;
  begin();
  append("\r\n@CHASSIS {\"v\":1,\"kind\":\"state\",\"session\":%lu,\"t_ms\":%lu,\"trace_seq\":%lu,",
         (unsigned long)session_id, (unsigned long)now, (unsigned long)trace_sequence);
  append("\"task\":{\"state\":\"%s\",\"reason\":\"%s\",\"id\":%lu,\"control_dt_ms\":%lu},",
         states[motion.state], motion.reason ? motion.reason : "none",
         (unsigned long)motion.action_id, (unsigned long)motion.control_dt_ms);
  append("\"target\":{\"body\":[%.3f,%.3f,%.5f],\"rpm\":[%d,%d,%d,%d]},",
         (double)motion.body_target.vx_mm_s, (double)motion.body_target.vy_mm_s,
         (double)motion.body_target.omega_rad_s, motion.rpm_command[0], motion.rpm_command[1],
         motion.rpm_command[2], motion.rpm_command[3]);
  append("\"feedback\":{\"rpm\":[%ld,%ld,%ld,%ld],\"pos\":[%lld,%lld,%lld,%lld],",
         (long)f[0].speed_rpm, (long)f[1].speed_rpm, (long)f[2].speed_rpm, (long)f[3].speed_rpm,
         (long long)f[0].position_raw, (long long)f[1].position_raw, (long long)f[2].position_raw,
         (long long)f[3].position_raw);
  append("\"speed_valid\":[%s,%s,%s,%s],\"position_valid\":[%s,%s,%s,%s],",
         boolean(f[0].speed_valid && now - f[0].speed_ms <= 600U),
         boolean(f[1].speed_valid && now - f[1].speed_ms <= 600U),
         boolean(f[2].speed_valid && now - f[2].speed_ms <= 600U),
         boolean(f[3].speed_valid && now - f[3].speed_ms <= 600U),
         boolean(f[0].position_valid && now - f[0].position_ms <= 600U),
         boolean(f[1].position_valid && now - f[1].position_ms <= 600U),
         boolean(f[2].position_valid && now - f[2].position_ms <= 600U),
         boolean(f[3].position_valid && now - f[3].position_ms <= 600U));
  append("\"speed_ms\":[%lu,%lu,%lu,%lu],\"position_ms\":[%lu,%lu,%lu,%lu],\"seq\":%lu,\"span_ms\":"
         "%lu},",
         (unsigned long)f[0].speed_ms, (unsigned long)f[1].speed_ms, (unsigned long)f[2].speed_ms,
         (unsigned long)f[3].speed_ms, (unsigned long)f[0].position_ms,
         (unsigned long)f[1].position_ms, (unsigned long)f[2].position_ms,
         (unsigned long)f[3].position_ms, (unsigned long)localization.feedback_sequence,
         (unsigned long)localization.observer.feedback_span_ms);
  append("\"yaw\":{\"valid\":%s,\"rad\":%.6f},\"pose\":{", boolean(motion.heading_valid && motion.map_anchor_valid),
         (double)motion.map_yaw_rad);
  append("\"command\":{\"valid\":%s,\"x_mm\":%.3f,\"y_mm\":%.3f,\"yaw_rad\":%.6f},",
         boolean(localization.observer.command_valid), (double)localization.observer.command.x_mm,
         (double)localization.observer.command.y_mm, (double)localization.observer.command.yaw_rad);
  append("\"feedback\":{\"valid\":%s,\"x_mm\":%.3f,\"y_mm\":%.3f,\"yaw_rad\":%.6f}},",
         boolean(localization.feedback_valid), (double)localization.observer.feedback.x_mm,
         (double)localization.observer.feedback.y_mm, (double)localization.observer.feedback.yaw_rad);
  append("\"localization\":{\"generation\":%lu,\"feedback_tick\":%lu,\"origin_valid\":%s},",
         (unsigned long)localization.generation, (unsigned long)localization.feedback_tick,
         boolean(localization.origin_valid));
  append("\"route\":{\"state\":\"%s\",\"reason\":\"%s\",\"segment\":%lu,\"action_id\":%lu,"
         "\"target\":[%.3f,%.3f,%.3f],\"error\":[%.3f,%.3f,%.3f],"
         "\"feedback_valid\":%s,\"stop_confirmed\":%s},",
         route.state ? route.state : "idle", route.reason ? route.reason : "none",
         (unsigned long)route.segment, (unsigned long)route.action_id,
         (double)route.target_x_mm, (double)route.target_y_mm, (double)route.target_yaw_deg,
         (double)route.error_x_mm, (double)route.error_y_mm, (double)route.error_heading_deg,
         boolean(route.feedback_valid), boolean(route.stop_confirmed));
  if(motion.distance_mode)
  {
    append("\"distance\":{\"active\":%s,\"state\":\"%s\",\"reason\":\"%s\",\"action_id\":%lu,"
           "\"target\":[%.3f,%.3f,%.3f],\"error\":[%.3f,%.3f,%.3f],\"feedback_valid\":%s,\"stop_confirmed\":%s},",
           boolean(ChassisMotion_IsBusy()),states[motion.state],motion.reason?motion.reason:"none",
           (unsigned long)motion.action_id,(double)motion.target_x_mm,(double)motion.target_y_mm,
           (double)motion.target_map_yaw_deg,(double)motion.error_x_mm,(double)motion.error_y_mm,
           (double)motion.error_heading_deg,boolean(localization.feedback_valid),boolean(motion.stop_confirmed));
  }
  append("\"tx\":{\"seq\":%lu,\"stage\":%u,\"error\":%u,\"ack_profile\":%u,\"tx_complete\":%s,"
         "\"acknowledged\":%s,\"acknowledged_wheels\":%u},\"dropped\":%lu}\r\n",
         (unsigned long)tx.sequence, (unsigned)tx.stage, (unsigned)tx.error,
         (unsigned)tx.ack_profile, boolean(tx.tx_complete), boolean(tx.acknowledged),
         (unsigned)tx.acknowledged_wheels, (unsigned long)ConsoleTx_Dropped());
  if (format_ok)
    (void)ConsoleTx_Telemetry(line, (uint16_t)length);
}
static bool number(const char *s, float *value)
{
  char *end;
  errno = 0;
  *value = strtof(s, &end);
  return end != s && !*end && !errno && isfinite(*value);
}
static bool integer(const char *s, uint32_t *value)
{
  char *end;
  unsigned long v;
  if (*s == '-')
    return false;
  errno = 0;
  v = strtoul(s, &end, 10);
  if (end == s || *end || errno || v > 0xFFFFFFFFUL)
    return false;
  *value = (uint32_t)v;
  return true;
}
bool ChassisTelemetry_Command(unsigned n, char *t[])
{
  bool ok = false;
  uint32_t value;
  float x, y, z;
  ChassisMotion_Target_t target;
  ChassisMotion_Status_t status;
  Mecanum_Status_t tx;
  char text[320];
  ChassisLocalization_Status_t localization;
  if (n < 2 || strcmp(t[0], "chassis"))
    return false;
  if ((ChassisRoute_IsBusy() || ChassisMotion_IsBusy()) &&
      (!strcmp(t[1], "origin") || !strcmp(t[1], "units") ||
       !strcmp(t[1], "profile") || (!strcmp(t[1], "feedback") && n > 2) ||
       !strcmp(t[1], "run") || !strcmp(t[1], "heading") || !strcmp(t[1], "reset-confirmed")))
  {
    reply("ERR chassis task reserved; stop/cancel before changing control settings\r\n");
    return true;
  }
  if (!strcmp(t[1], "stream"))
  {
    if (n == 3 && !strcmp(t[2], "off"))
      ok = ChassisTelemetry_Stream(false, 0);
    else if (n == 4 && !strcmp(t[2], "on") && integer(t[3], &value))
      ok = ChassisTelemetry_Stream(true, value);
  }
  else if (!strcmp(t[1], "origin"))
  {
    if (n == 5 && number(t[2], &x) && number(t[3], &y) && number(t[4], &z))
      ok = ChassisTelemetry_Origin(x, y, z * CHASSIS_MODEL_PI / 180.0f);
  }
  else if (!strcmp(t[1], "units"))
  {
    if (n == 3 && integer(t[2], &value))
      ok = ChassisTelemetry_ConfirmPositionUnits(value);
  }
  else if (!strcmp(t[1], "profile"))
  {
    if (n == 3 && !strcmp(t[2], "none"))
      ok = Mecanum_AckProfile_Set(MECANUM_ACK_NONE);
    else if (n == 3 && !strcmp(t[2], "receive"))
      ok = Mecanum_AckProfile_Set(MECANUM_ACK_RECEIVE);
  }
  else if (!strcmp(t[1], "feedback"))
  {
    if (n == 2)
    {
      Mecanum_Feedback_t feedback;
      unsigned i;
      for (i = 0; i < 4; i++)
      {
        (void)Mecanum_FeedbackGet(i, &feedback);
        (void)snprintf(text, sizeof(text),
          "OK wheel=%u raw_rpm=%ld raw_units=%lld flags=0x%02X valid=%u,%u,%u rx_ms=%lu,%lu,%lu\r\n",
          i + 1U, (long)feedback.speed_rpm, (long long)feedback.position_raw,
          feedback.state_flags, (unsigned)feedback.speed_valid,
          (unsigned)feedback.position_valid, (unsigned)feedback.state_valid,
          (unsigned long)feedback.speed_ms, (unsigned long)feedback.position_ms,
          (unsigned long)feedback.state_ms);
        reply(text);
      }
      return true;
    }
    if (n == 3 && (!strcmp(t[2], "on") || !strcmp(t[2], "off")))
    {
      Mecanum_Feedback_Enable(!strcmp(t[2], "on"));
      ok = true;
    }
    else if (n == 3 && integer(t[2], &value) && value <= 4U)
      ok = Mecanum_Feedback_Select((uint8_t)value);
  }
  else if (!strcmp(t[1], "reset-confirmed"))
  {
    if (n == 2 && !ChassisMotion_IsBusy())
      ok = Mecanum_RecoveryAfterReset();
  }
  else if (!strcmp(t[1], "run") || !strcmp(t[1], "heading"))
  {
    ChassisMotion_TargetDefaults(&target);
    if (n == 6 && number(t[2], &x) && number(t[3], &y) && number(t[4], &z) && integer(t[5], &value))
    {
      target.vx_mm_s = x;
      target.vy_mm_s = y;
      target.hold_ms = value;
      if (!strcmp(t[1], "heading"))
      {
        target.mode = CHASSIS_MOTION_HEADING;
        target.heading_deg = z;
      }
      else
        target.omega_rad_s = z;
      ok = ChassisMotion_Start(&target);
    }
  }
  else if (!strcmp(t[1], "stop") && ChassisMotion_IsBusy())
  {
    if (n == 2)
      ok = ChassisMotion_Stop(0);
  }
  else if (!strcmp(t[1], "task"))
  {
    ChassisMotion_StatusGet(&status);
    Mecanum_StatusGet(&tx);
    ChassisLocalization_Get(&localization);
    (void)snprintf(text, sizeof(text),
                   "OK chassis task state=%u reason=%s dt_ms=%lu seq=%lu stage=%u error=%u ack=%u "
                   "locked=%u units=%lu\r\n",
                   (unsigned)status.state, status.reason ? status.reason : "none",
                   (unsigned long)status.control_dt_ms, (unsigned long)tx.sequence,
                   (unsigned)tx.stage, (unsigned)tx.error, (unsigned)tx.ack_profile,
                   (unsigned)tx.locked, (unsigned long)localization.units_per_rev);
    reply(text);
    if(status.distance_mode)
    {
      (void)snprintf(text,sizeof(text),
        "OK chassis distance target_mm=%.1f,%.1f map_heading_deg=%.2f error_mm=%.1f,%.1f heading_error_deg=%.2f feedback_valid=%u stop_confirmed=%u\r\n",
        (double)status.target_x_mm,(double)status.target_y_mm,(double)status.target_map_yaw_deg,
        (double)status.error_x_mm,(double)status.error_y_mm,(double)status.error_heading_deg,
        localization.feedback_valid?1:0,status.stop_confirmed?1:0);
      reply(text);
    }
    return true;
  }
  else
    return false;
  reply(ok ? "OK chassis request accepted (not motion/ACK confirmation)\r\n"
           : "ERR chassis request rejected; inspect chassis task and command units\r\n");
  return true;
}
