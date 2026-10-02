#include "radar_scan.h"
#include <math.h>
#include <string.h>

static RadarMap_Params_t parameters, capture_parameters, published_parameters;
static RadarMap_t published_map, working_map;
/* One cloud only: recapture invalidates its firmware view until publication.
 * The console/browser can retain the previous complete cloud as history. */
static RadarSample_t cloud[RADAR_SCAN_POINT_CAPACITY];
static RadarScan_Status_t status;
static LdsPacket_t pending_packet;
static LdsEvent_t incoming;
static bool have_pending, have_previous;
static uint16_t previous_start;
static uint32_t progress, pending_progress, start_tick, capture_origin;
static uint32_t seen_bad, seen_loss, parameter_revision, capture_revision;

static bool active(void)
{
  return status.state == RADAR_SCAN_PREPARING || status.state == RADAR_SCAN_WAIT_WRAP ||
         status.state == RADAR_SCAN_COLLECTING;
}

const char *RadarScan_StateName(RadarScan_State_t state)
{
  static const char *const names[] = {"idle", "preparing", "wait_wrap", "collecting", "ready", "stopped", "error"};
  return (unsigned)state < sizeof names / sizeof names[0] ? names[state] : "unknown";
}

const char *RadarScan_UnitName(RadarScan_Unit_t unit)
{
  return unit == RADAR_UNIT_MM ? "mm" : unit == RADAR_UNIT_CM ? "cm" : "unknown_mm";
}

HAL_StatusTypeDef RadarScan_Init(UART_HandleTypeDef *uart)
{
  memset(&status, 0, sizeof status); RadarMap_Defaults(&parameters);
  RadarMap_Reset(&published_map); RadarMap_Reset(&working_map);
  status.state = RADAR_SCAN_IDLE; status.reason = "idle";
  have_pending = have_previous = false;
  parameter_revision = capture_revision = 0;
  return RadarUart_Init(uart);
}

static void reset_circle(const char *reason)
{
  have_pending = false; progress = pending_progress = 0;
  RadarMap_Reset(&working_map);
  status.point_count = status.coverage_tenths = 0;
  status.raw_points = 0; status.truncated = false;
  status.state = RADAR_SCAN_WAIT_WRAP; status.reason = reason;
}

bool RadarScan_Start(const RadarMap_Params_t *params, uint32_t origin_generation)
{
  RadarUart_Status_t uart_status;
  if (active() || !RadarMap_ParamsValid(params)) return false;
  if (!RadarUart_StartScan()) return false;
  capture_parameters = *params; capture_revision = parameter_revision;
  capture_origin = origin_generation; start_tick = HAL_GetTick();
  RadarUart_StatusGet(&uart_status); seen_bad = uart_status.bad; seen_loss = uart_status.loss_generation;
  have_previous = have_pending = false; progress = pending_progress = 0;
  RadarMap_Reset(&working_map);
  status.state = RADAR_SCAN_PREPARING; status.reason = "configuring";
  status.points_valid = false; status.point_count = 0; status.truncated = false;
  status.unit = RADAR_UNIT_UNKNOWN; status.elapsed_ms = 0; status.raw_points = 0;
  status.circle_restarts = status.unit_changes = status.unit_overflows = 0;
  status.coverage_tenths = 0;
  return true;
}

void RadarScan_Stop(void)
{
  if (active()) status.elapsed_ms = HAL_GetTick() - start_tick;
  have_pending = have_previous = false;
  RadarUart_StopScan(); status.state = RADAR_SCAN_STOPPED; status.reason = "operator_stop";
}

static void add_packet(const LdsPacket_t *packet, uint16_t span, uint32_t packet_progress)
{
  uint16_t i, angle;
  uint32_t offset, distance;
  RadarSample_t sample;
  for (i = 0; i < packet->count; i++) {
    /* CF FA uses the user's proven F1 sector/(N-1) convention. Its exact
     * endpoint convention remains a radar/manual verification item. CE FA
     * spans the next packet start, so divides that span by N. */
    offset = packet->fixed ? (packet->count > 1U ? (uint32_t)span * i / (packet->count - 1U) : 0U)
                           : (uint32_t)span * i / packet->count;
    if (packet_progress + offset >= 3600U) break;
    angle = (uint16_t)((packet->start_angle_tenths + offset) % 3600U);
    distance = packet->points[i].distance;
    if (status.unit == RADAR_UNIT_CM) distance *= 10U;
    if (distance > 65535U) { status.unit_overflows++; distance = 0; }
    sample.angle_tenths = angle; sample.distance_mm = (uint16_t)distance;
    sample.energy = packet->points[i].energy;
    status.raw_points++;
    RadarMap_Add(&working_map, &capture_parameters, &sample);
    if (status.point_count < RADAR_SCAN_POINT_CAPACITY) cloud[status.point_count++] = sample;
    else status.truncated = true;
  }
}

static void publish(void)
{
  RadarMap_Finish(&working_map, &capture_parameters);
  published_map = working_map; published_parameters = capture_parameters;
  status.scan_id++; status.map_id++; status.points_map_id = status.map_id;
  status.map_valid = status.points_valid = true;
  status.map_applicable = capture_revision == parameter_revision;
  status.origin_generation = capture_origin; status.completed_tick = HAL_GetTick();
  status.elapsed_ms = status.completed_tick - start_tick;
  status.coverage_tenths = 3600U; status.state = RADAR_SCAN_READY; status.reason = "complete";
  have_pending = have_previous = false; RadarUart_StopScan();
}

static void measurement(const LdsPacket_t *packet)
{
  uint16_t start = packet->start_angle_tenths, delta = 0;
  /* A forward packet step across zero has a large numeric decrease. Using
   * the circle's half-turn boundary avoids tying acceptance to a 270deg
   * packet start while still rejecting small backwards/disordered starts. */
  bool wrapped = have_previous && start < previous_start && (uint16_t)(previous_start - start) > 1800U;
  if (!have_previous) { previous_start = start; have_previous = true; return; }
  if (start == previous_start) return; /* a duplicate packet cannot advance a circle */
  if (start < previous_start && !wrapped) {
    if (status.state == RADAR_SCAN_COLLECTING) { status.circle_restarts++; reset_circle("angle_resync"); }
    previous_start = start; have_pending = false; return;
  }
  delta = (uint16_t)((start + 3600U - previous_start) % 3600U);
  if (status.state == RADAR_SCAN_WAIT_WRAP) {
    previous_start = start;
    if (!wrapped) return;
    reset_circle("collecting"); status.state = RADAR_SCAN_COLLECTING;
  } else {
    if (have_pending) add_packet(&pending_packet, delta, pending_progress);
    progress += delta;
    status.coverage_tenths = (uint16_t)(progress < 3600U ? progress : 3600U);
    previous_start = start;
    if (progress >= 3600U) { publish(); return; }
  }
  if (packet->fixed) {
    add_packet(packet, packet->sector_angle_tenths, progress); have_pending = false;
  } else {
    pending_packet = *packet; pending_progress = progress; have_pending = true;
  }
}

static void accept_event(const LdsEvent_t *event)
{
  if (!active()) return;
  if (event->kind == LDS_EVENT_STATUS) {
    RadarScan_Unit_t next = (event->data.status_flags & 1U) ? RADAR_UNIT_MM : RADAR_UNIT_CM;
    bool factor_changed = (status.unit == RADAR_UNIT_CM) != (next == RADAR_UNIT_CM);
    status.unit = next;
    if (factor_changed) {
      status.unit_changes++; status.circle_restarts++;
      if (status.state != RADAR_SCAN_PREPARING) reset_circle("unit_changed");
      have_previous = have_pending = false;
    }
  } else if (event->kind == LDS_EVENT_MEASUREMENT && status.state != RADAR_SCAN_PREPARING)
    measurement(&event->data.packet);
}

static void check_stream(const RadarUart_Status_t *uart_status)
{
  if (active() && (seen_bad != uart_status->bad || seen_loss != uart_status->loss_generation)) {
    if (status.state != RADAR_SCAN_PREPARING) {
      status.circle_restarts++; reset_circle(uart_status->loss_generation != seen_loss ? "rx_resync" : "frame_resync");
      have_previous = false;
    }
  }
  seen_bad = uart_status->bad; seen_loss = uart_status->loss_generation;
}

void RadarScan_Process(void)
{
  RadarUart_Status_t uart_status;
  RadarUart_StatusGet(&uart_status); check_stream(&uart_status);
  while (RadarUart_ReadEvent(&incoming)) accept_event(&incoming);
  RadarUart_Process(); RadarUart_StatusGet(&uart_status); check_stream(&uart_status);
  if (status.state == RADAR_SCAN_PREPARING && uart_status.configured) {
    reset_circle("waiting_wrap"); have_previous = false;
  }
  while (RadarUart_ReadEvent(&incoming)) accept_event(&incoming);
  if (active()) {
    status.elapsed_ms = HAL_GetTick() - start_tick;
    if (uart_status.command_failed || status.elapsed_ms >= RADAR_SCAN_TIMEOUT_MS) {
      status.state = RADAR_SCAN_ERROR;
      status.reason = uart_status.command_failed ? "command_failed" : "scan_timeout";
      have_pending = have_previous = false; RadarUart_StopScan();
    }
  }
}

void RadarScan_StatusGet(RadarScan_Status_t *out)
{
  RadarUart_Status_t uart_status;
  if (!out) return;
  *out = status; RadarUart_StatusGet(&uart_status);
  out->bytes = uart_status.received_bytes; out->packets = uart_status.packets;
  out->bad = uart_status.bad; out->overflow = uart_status.overwritten_bytes;
  out->alarm = uart_status.alarms;
  if (active()) out->elapsed_ms = HAL_GetTick() - start_tick;
}

const RadarMap_t *RadarScan_Map(void) { return status.map_valid ? &published_map : 0; }
const RadarMap_Params_t *RadarScan_MapParams(void) { return status.map_valid ? &published_parameters : 0; }
const RadarSample_t *RadarScan_Points(uint16_t *count)
{
  if (count) *count = status.points_valid ? status.point_count : 0;
  return status.points_valid ? cloud : 0;
}
const RadarMap_Params_t *RadarScan_Params(void) { return &parameters; }

bool RadarScan_SetParam(const char *key, float value)
{
  RadarMap_Params_t next = parameters;
  if (!key || !isfinite(value)) return false;
  if (!strcmp(key, "lidar_x_mm")) next.lidar_x_mm = value;
  else if (!strcmp(key, "lidar_y_mm")) next.lidar_y_mm = value;
  else if (!strcmp(key, "zero_deg") || !strcmp(key, "zero_offset_deg")) next.zero_deg = value;
  else {
    if (value < 0 || value > 65535 || floorf(value) != value) return false;
    if (!strcmp(key,"distance_min_mm") || !strcmp(key,"min_distance_mm")) next.distance_min_mm=(uint16_t)value;
    else if (!strcmp(key,"distance_max_mm") || !strcmp(key,"max_distance_mm")) next.distance_max_mm=(uint16_t)value;
    else if (!strcmp(key,"angle_min_tenths") || !strcmp(key,"min_angle_tenths")) next.angle_min_tenths=(uint16_t)value;
    else if (!strcmp(key,"angle_max_tenths") || !strcmp(key,"max_angle_tenths")) next.angle_max_tenths=(uint16_t)value;
    else if (!strcmp(key,"threshold")) next.threshold=(uint16_t)value;
    else if (!strcmp(key,"energy_min") || !strcmp(key,"min_energy")) { if(value>255)return false;next.energy_min=(uint8_t)value; }
    else if (!strcmp(key,"energy_max") || !strcmp(key,"max_energy")) { if(value>255)return false;next.energy_max=(uint8_t)value; }
    else return false;
  }
  if (!RadarMap_ParamsValid(&next)) return false;
  /* Compare named fields rather than structure padding. Derived world pose in
   * Start is separate from this user-parameter revision. */
  if (next.lidar_x_mm != parameters.lidar_x_mm || next.lidar_y_mm != parameters.lidar_y_mm ||
      next.zero_deg != parameters.zero_deg || next.distance_min_mm != parameters.distance_min_mm ||
      next.distance_max_mm != parameters.distance_max_mm || next.angle_min_tenths != parameters.angle_min_tenths ||
      next.angle_max_tenths != parameters.angle_max_tenths || next.threshold != parameters.threshold ||
      next.energy_min != parameters.energy_min || next.energy_max != parameters.energy_max) {
    parameters = next; parameter_revision++;
    if (status.map_valid) status.map_applicable = false;
  }
  return true;
}
