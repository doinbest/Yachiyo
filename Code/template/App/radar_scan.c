#include "radar_scan.h"
#include <math.h>
#include <string.h>

static RadarMap_Params_t parameters, capture_parameters, published_parameters;
static RadarMap_t published_map, working_map;
/* One cloud only: recapture invalidates its firmware view until publication.
 * The console/browser can retain the previous complete cloud as history. */
static RadarSample_t cloud[RADAR_SCAN_POINT_CAPACITY];
static RadarScan_Status_t status;
static RadarScan_StartStatus_t start_status;
static RadarStart_Context_t start_candidates;
static LdsPacket_t pending_packet;
static LdsEvent_t incoming;
static bool have_pending, have_previous;
static bool capture_auto,selection_available;
static RadarStart_Id_t capture_start;
static uint16_t previous_start;
static uint32_t progress, pending_progress, start_tick, capture_origin;
static uint32_t capture_completed_tick;
static uint32_t seen_bad, seen_loss, parameter_revision, capture_revision;
static RadarScan_Raw_t debug_snapshot;
static bool capture_debug;
static uint32_t incoming_end, debug_bad_start;
static RadarScan_LiveStatus_t live;
static RadarLive_Point_t live_work[RADAR_LIVE_BINS],live_latest[RADAR_LIVE_BINS];
static bool live_collecting;

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
const char *RadarScan_LiveStateName(RadarScan_LiveState_t state)
{
  static const char *const names[]={"preparing","waiting","streaming","stopped","error"};
  return (unsigned)state<sizeof names/sizeof names[0]?names[state]:"unknown";
}

HAL_StatusTypeDef RadarScan_Init(UART_HandleTypeDef *uart)
{
  memset(&status, 0, sizeof status); RadarMap_Defaults(&parameters);
  memset(&start_status, 0, sizeof start_status);start_status.result.reason="not_scanned";
  RadarMap_Reset(&published_map); RadarMap_Reset(&working_map);
  status.state = RADAR_SCAN_IDLE; status.reason = "idle";
  have_pending = have_previous = false;
  capture_auto=selection_available=false;
  capture_debug=false;memset(&debug_snapshot,0,sizeof debug_snapshot);
  memset(&live,0,sizeof live);live.state=RADAR_LIVE_STOPPED;live.reason="not_started";
  live_collecting=false;
  capture_start=RADAR_START_LOWER;
  parameter_revision = capture_revision = 0;
  return RadarUart_Init(uart);
}

bool RadarScan_LiveStart(const RadarMap_Params_t *params,uint32_t origin_generation,RadarStart_Id_t start_id)
{
  RadarUart_Status_t uart_status;uint32_t id=live.id+1U;
  if(active() || live.active || !RadarMap_ParamsValid(params) ||
     (start_id!=RADAR_START_UPPER && start_id!=RADAR_START_LOWER))return false;
  if(!RadarUart_StartScan())return false;
  memset(&live,0,sizeof live);live.id=id?id:1U;
  (void)RadarStart_ParamsForStart(params,start_id,&live.parameters);
  live.origin_generation=origin_generation;live.active=true;
  live.state=RADAR_LIVE_PREPARING;live.reason="configuring";
  memset(live_work,0,sizeof live_work);memset(live_latest,0,sizeof live_latest);
  live_collecting=have_previous=have_pending=false;progress=pending_progress=0;
  RadarUart_StatusGet(&uart_status);seen_bad=uart_status.bad;seen_loss=uart_status.loss_generation;
  return true;
}
void RadarScan_LiveStop(void)
{
  if(!live.active)return;
  live.active=false;live.state=RADAR_LIVE_STOPPED;live.reason="operator_stop";
  live_collecting=have_previous=have_pending=false;RadarUart_StopScan();
}
void RadarScan_LiveStatusGet(RadarScan_LiveStatus_t *out)
{
  if(!out)return;
  *out=live;
  out->age_ms=live.sequence?HAL_GetTick()-live.completed_tick:0;
}
bool RadarScan_LiveCopy(RadarLive_Point_t points[RADAR_LIVE_BINS])
{
  if(!points || !live.sequence)return false;
  memcpy(points,live_latest,sizeof live_latest);return true;
}
static void reset_live_circle(const char *reason)
{
  memset(live_work,0,sizeof live_work);live_collecting=have_pending=false;
  progress=pending_progress=0;live.state=RADAR_LIVE_WAITING;live.reason=reason;
}

static void reset_circle(const char *reason)
{
  have_pending = false; progress = pending_progress = 0;
  RadarMap_Reset(&working_map);
  if(capture_auto)RadarStart_Begin(&start_candidates,&capture_parameters);
  status.point_count = status.coverage_tenths = 0;
  status.raw_points = 0; status.truncated = false;
  if(capture_debug)debug_snapshot.circle_start=debug_snapshot.circle_end=0;
  status.state = RADAR_SCAN_WAIT_WRAP; status.reason = reason;
}

static bool start_capture(const RadarMap_Params_t *params, uint32_t origin_generation,
                          bool automatic,RadarStart_Id_t start_id)
{
  RadarUart_Status_t uart_status;
  if (active() || live.active || !RadarMap_ParamsValid(params)) return false;
  if (!RadarUart_StartScan()) return false;
  capture_debug=false;
  capture_parameters = *params; capture_revision = parameter_revision;
  capture_auto=automatic;capture_start=start_id;
  selection_available=false;
  capture_origin = origin_generation; start_tick = HAL_GetTick();
  RadarUart_StatusGet(&uart_status); seen_bad = uart_status.bad; seen_loss = uart_status.loss_generation;
  have_previous = have_pending = false; progress = pending_progress = 0;
  RadarMap_Reset(&working_map);
  if(capture_auto || capture_start==RADAR_START_UPPER){
    RadarStart_Begin(&start_candidates,&capture_parameters);
    if(!capture_auto)capture_parameters=*RadarStart_Params(&start_candidates,capture_start);
  }
  status.state = RADAR_SCAN_PREPARING; status.reason = "configuring";
  status.points_valid = false; status.point_count = 0; status.truncated = false;
  status.unit = RADAR_UNIT_UNKNOWN; status.elapsed_ms = 0; status.raw_points = 0;
  status.circle_restarts = status.unit_changes = status.unit_overflows = 0;
  status.coverage_tenths = 0;
  return true;
}

bool RadarScan_Start(const RadarMap_Params_t *params,uint32_t origin_generation)
{ return start_capture(params,origin_generation,false,RADAR_START_LOWER); }
bool RadarScan_StartAuto(const RadarMap_Params_t *params,uint32_t origin_generation)
{ return start_capture(params,origin_generation,true,RADAR_START_UNKNOWN); }
bool RadarScan_StartAt(const RadarMap_Params_t *params,uint32_t origin_generation,RadarStart_Id_t start_id)
{
  if(start_id!=RADAR_START_UPPER && start_id!=RADAR_START_LOWER)return false;
  return start_capture(params,origin_generation,false,start_id);
}
bool RadarScan_DebugStart(const RadarMap_Params_t *params,uint32_t origin_generation,RadarStart_Id_t start_id)
{
  RadarUart_Status_t uart_status;
  uint32_t id=debug_snapshot.id+1U;
  if(start_id!=RADAR_START_UNKNOWN && start_id!=RADAR_START_UPPER && start_id!=RADAR_START_LOWER)return false;
  if(!start_capture(params,origin_generation,start_id==RADAR_START_UNKNOWN,start_id))return false;
  memset(&debug_snapshot,0,sizeof debug_snapshot);capture_debug=true;
  debug_snapshot.id=id?id:1U;debug_snapshot.parameters=capture_parameters;
  debug_snapshot.origin_generation=origin_generation;debug_snapshot.start_id=(uint8_t)start_id;
  debug_snapshot.automatic=capture_auto;debug_snapshot.state=RADAR_SCAN_PREPARING;
  debug_snapshot.reason="configuring";
  RadarUart_StatusGet(&uart_status);debug_bad_start=uart_status.bad;
  RadarUart_RawStart();RadarUart_RawGet(&debug_snapshot.raw);return true;
}

static void finish_debug(bool circle_completed)
{
  RadarUart_Status_t uart_status;
  if(!capture_debug)return;
  RadarUart_RawStop();RadarUart_RawGet(&debug_snapshot.raw);RadarUart_StatusGet(&uart_status);
  debug_snapshot.state=status.state;debug_snapshot.reason=status.reason;
  debug_snapshot.unit=status.unit;debug_snapshot.bad=uart_status.bad-debug_bad_start;
  debug_snapshot.restarts=status.circle_restarts;debug_snapshot.point_total=status.raw_points;
  if(circle_completed){
    debug_snapshot.scan_id=status.scan_id;debug_snapshot.map_id=status.points_map_id;
    debug_snapshot.parameters=status.points_map_id?published_parameters:capture_parameters;
    debug_snapshot.start_id=start_status.result.start_id;
    debug_snapshot.points=status.point_count;debug_snapshot.cloud_truncated=status.truncated;
    debug_snapshot.complete=debug_snapshot.circle_end>debug_snapshot.circle_start &&
      debug_snapshot.circle_start>=debug_snapshot.raw.window_start &&
      debug_snapshot.circle_end<=debug_snapshot.raw.window_start+debug_snapshot.raw.total;
  }
  capture_debug=false;
}

void RadarScan_RawStatusGet(RadarScan_Raw_t *out)
{
  if(!out)return;
  *out=debug_snapshot;RadarUart_RawGet(&out->raw);
  if(capture_debug){out->state=status.state;out->reason=status.reason;out->unit=status.unit;}
  out->cloud_valid=out->scan_id && status.points_valid && out->scan_id==status.scan_id;
}

uint16_t RadarScan_RawRead(uint32_t offset,uint8_t *bytes,uint16_t capacity)
{ return debug_snapshot.id?RadarUart_RawRead(offset,bytes,capacity):0; }
static void publish_map(const RadarMap_t *map,const RadarMap_Params_t *params)
{
  published_map=*map;published_parameters=*params;
  RadarMap_Finish(&published_map,&published_parameters);
  status.map_id++;status.points_map_id=status.map_id;status.map_valid=true;
  status.map_applicable=capture_revision==parameter_revision;
  status.origin_generation=capture_origin;status.completed_tick=capture_completed_tick;
}
bool RadarScan_SelectStart(RadarStart_Id_t start_id)
{
  const RadarMap_t *map;const RadarMap_Params_t *params;
  if(active() || live.active || !selection_available)return false;
  map=RadarStart_Map(&start_candidates,start_id);params=RadarStart_Params(&start_candidates,start_id);
  if(!map || !params)return false;
  publish_map(map,params);start_status.result.start_id=(uint8_t)start_id;
  start_status.result.reason="manual_selection";
  status.state=RADAR_SCAN_READY;status.reason="complete";
  return true;
}
void RadarScan_StartStatusGet(RadarScan_StartStatus_t *out)
{ if(out)*out=start_status; }

void RadarScan_Stop(void)
{
  if(live.active){RadarScan_LiveStop();return;}
  if (active()) status.elapsed_ms = HAL_GetTick() - start_tick;
  have_pending = have_previous = false;
  status.state = RADAR_SCAN_STOPPED; status.reason = "operator_stop";
  finish_debug(false);RadarUart_StopScan();
}

static bool packet_sample(const LdsPacket_t *packet,uint16_t span,uint32_t packet_progress,
                          uint16_t i,RadarScan_Unit_t unit,RadarSample_t *sample,uint32_t *overflows)
{
  uint32_t offset, distance;
  /* Preserve the proven CF sector/(N-1) and CE next-start span/N convention. */
  offset=packet->fixed?(packet->count>1U?(uint32_t)span*i/(packet->count-1U):0U):(uint32_t)span*i/packet->count;
  if(packet_progress+offset>=3600U)return false;
  distance=packet->points[i].distance;if(unit==RADAR_UNIT_CM)distance*=10U;
  if(distance>65535U){(*overflows)++;distance=0;}
  sample->angle_tenths=(uint16_t)((packet->start_angle_tenths+offset)%3600U);
  sample->distance_mm=(uint16_t)distance;sample->energy=packet->points[i].energy;return true;
}
static void add_packet(const LdsPacket_t *packet, uint16_t span, uint32_t packet_progress)
{
  uint16_t i;
  RadarSample_t sample;
  for (i = 0; i < packet->count; i++) {
    if(!packet_sample(packet,span,packet_progress,i,status.unit,&sample,&status.unit_overflows))break;
    status.raw_points++;
    if(capture_auto)RadarStart_Add(&start_candidates,&sample);
    else RadarMap_Add(&working_map, &capture_parameters, &sample);
    if (status.point_count < RADAR_SCAN_POINT_CAPACITY) cloud[status.point_count++] = sample;
    else status.truncated = true;
  }
}
static void add_live_packet(const LdsPacket_t *packet,uint16_t span,uint32_t packet_progress)
{
  RadarSample_t sample;uint16_t i,bin;const RadarMap_Params_t *p=&live.parameters;
  for(i=0;i<packet->count;i++){
    if(!packet_sample(packet,span,packet_progress,i,live.unit,&sample,&live.unit_overflows))break;
    /* A diagnostic cloud covers all angles and the entire physical scene,
     * including walls outside the map. Only range/energy acceptance applies. */
    if(!sample.distance_mm || sample.distance_mm<p->distance_min_mm || sample.distance_mm>p->distance_max_mm ||
       sample.energy<p->energy_min || sample.energy>p->energy_max)continue;
    bin=(uint16_t)(sample.angle_tenths/(3600U/RADAR_LIVE_BINS));
    if(!live_work[bin].distance_mm || sample.distance_mm<live_work[bin].distance_mm){
      live_work[bin].angle_tenths=sample.angle_tenths;live_work[bin].distance_mm=sample.distance_mm;
    }
  }
}
static void add_current_packet(const LdsPacket_t *packet,uint16_t span,uint32_t packet_progress)
{
  if(live.active)add_live_packet(packet,span,packet_progress);
  else add_packet(packet,span,packet_progress);
}
static void publish_live(void)
{
  memcpy(live_latest,live_work,sizeof live_latest);live.points=0;
  for(unsigned i=0;i<RADAR_LIVE_BINS;i++)if(live_latest[i].distance_mm)live.points++;
  live.sequence++;if(!live.sequence)live.sequence=1U;
  live.completed_tick=HAL_GetTick();live.state=RADAR_LIVE_STREAMING;live.reason="complete";
  memset(live_work,0,sizeof live_work);progress=pending_progress=0;have_pending=false;
}

static void publish(void)
{
  /* The complete angular sequence is required to distinguish a field-external
   * surface from the foreground objects that briefly occlude it. Live preview
   * bins are never used for this step. Keep the streaming result if storage was
   * truncated: a partial cloud cannot establish a complete contour. */
  if(!status.truncated){
    if(capture_auto){
      for(unsigned i=0;i<2U;i++)
        RadarMap_BuildCloud(&start_candidates.maps[i],&start_candidates.params[i],
                            cloud,status.point_count);
    }else RadarMap_BuildCloud(&working_map,&capture_parameters,cloud,status.point_count);
  }
  status.scan_id++;
  memset(&start_status.result,0,sizeof start_status.result);
  start_status.scan_id=status.scan_id;start_status.automatic=capture_auto;
  status.points_valid=true;capture_completed_tick=HAL_GetTick();
  status.elapsed_ms = capture_completed_tick - start_tick;
  status.coverage_tenths = 3600U; status.state = RADAR_SCAN_READY; status.reason = "complete";
  if(capture_auto){
    RadarStart_Finish(&start_candidates,&start_status.result);selection_available=true;
    if(start_status.result.start_id!=RADAR_START_UNKNOWN)
      publish_map(RadarStart_Map(&start_candidates,(RadarStart_Id_t)start_status.result.start_id),
                  RadarStart_Params(&start_candidates,(RadarStart_Id_t)start_status.result.start_id));
    else {
      status.map_applicable=false;status.points_map_id=0;
      status.reason=!strcmp(start_status.result.reason,"both_conflict")?"start_both_conflict":"start_ambiguous";
    }
  } else {
    unsigned index=capture_start==RADAR_START_UPPER?0U:1U;
    RadarMap_FinishRaw(&working_map,&capture_parameters);
    start_status.result.start_id=(uint8_t)capture_start;start_status.result.reason="manual_reference";
    start_status.result.qr_counts[index]=working_map.counts[14];start_status.result.raw_counts[index]=working_map.counts[22];
    start_status.result.conflict_mask[index]=working_map.mask&RADAR_MAP_REQUIRED_CLEAR_MASK;
    publish_map(&working_map,&capture_parameters);
  }
  if(capture_debug)debug_snapshot.circle_end=incoming_end-debug_snapshot.raw.first_position;
  finish_debug(true);
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
    if(live.active){if(live_collecting){live.circle_restarts++;reset_live_circle("angle_resync");}}
    else if (status.state == RADAR_SCAN_COLLECTING) { status.circle_restarts++; reset_circle("angle_resync"); }
    previous_start = start; have_pending = false; return;
  }
  delta = (uint16_t)((start + 3600U - previous_start) % 3600U);
  if (live.active?!live_collecting:status.state==RADAR_SCAN_WAIT_WRAP) {
    previous_start = start;
    if (!wrapped) return;
    if(live.active){reset_live_circle("collecting");live_collecting=true;}
    else {reset_circle("collecting");status.state=RADAR_SCAN_COLLECTING;}
    if(capture_debug){
      uint32_t frame_bytes=(packet->fixed?10U:8U)+3U*packet->count;
      debug_snapshot.circle_start=incoming_end-frame_bytes-debug_snapshot.raw.first_position;
    }
  } else {
    if (have_pending) add_current_packet(&pending_packet, delta, pending_progress);
    progress += delta;
    if(!live.active)status.coverage_tenths=(uint16_t)(progress<3600U?progress:3600U);
    previous_start = start;
    if(progress>=3600U){if(live.active)publish_live();else {publish();return;}}
  }
  if (packet->fixed) {
    add_current_packet(packet, packet->sector_angle_tenths, progress); have_pending = false;
  } else {
    pending_packet = *packet; pending_progress = progress; have_pending = true;
  }
}

static void accept_event(const LdsEvent_t *event)
{
  if(live.active){
    if(event->kind==LDS_EVENT_STATUS){
      RadarScan_Unit_t next=(event->data.status_flags&1U)?RADAR_UNIT_MM:RADAR_UNIT_CM;
      bool changed=(live.unit==RADAR_UNIT_CM)!=(next==RADAR_UNIT_CM);live.unit=next;
      if(changed){live.unit_changes++;live.circle_restarts++;
        if(live.state!=RADAR_LIVE_PREPARING)reset_live_circle("unit_changed");
        have_previous=have_pending=false;}
    }else if(event->kind==LDS_EVENT_MEASUREMENT && live.state!=RADAR_LIVE_PREPARING)
      measurement(&event->data.packet);
    return;
  }
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
  if(live.active && (seen_bad!=uart_status->bad || seen_loss!=uart_status->loss_generation)){
    if(live.state!=RADAR_LIVE_PREPARING){
      live.circle_restarts++;reset_live_circle(uart_status->loss_generation!=seen_loss?"rx_resync":"frame_resync");
      have_previous=false;
    }
  }
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
  while (RadarUart_ReadEventAt(&incoming,&incoming_end)) accept_event(&incoming);
  RadarUart_Process(); RadarUart_StatusGet(&uart_status); check_stream(&uart_status);
  if (status.state == RADAR_SCAN_PREPARING && uart_status.configured) {
    reset_circle("waiting_wrap"); have_previous = false;
  }
  if(live.active && live.state==RADAR_LIVE_PREPARING && uart_status.configured){
    reset_live_circle("waiting_wrap");have_previous=false;
  }
  while (RadarUart_ReadEventAt(&incoming,&incoming_end)) accept_event(&incoming);
  if(live.active && uart_status.command_failed){
    live.active=false;live.state=RADAR_LIVE_ERROR;live.reason="command_failed";
    live_collecting=have_pending=have_previous=false;RadarUart_StopScan();
  }
  if (active()) {
    status.elapsed_ms = HAL_GetTick() - start_tick;
    if (uart_status.command_failed || status.elapsed_ms >= RADAR_SCAN_TIMEOUT_MS) {
      status.state = RADAR_SCAN_ERROR;
      status.reason = uart_status.command_failed ? "command_failed" : "scan_timeout";
      have_pending = have_previous = false;finish_debug(false);RadarUart_StopScan();
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
  out->alarm = uart_status.alarms; out->last_alarm = uart_status.last_alarm;
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
