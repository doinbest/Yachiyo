#include "navigation_report.h"

#define REPORT_LINE_CAPACITY 160U

typedef struct
{
  char data[REPORT_LINE_CAPACITY];
  uint16_t length;
  uint8_t overflow;
} ReportLine;

static void line_reset(ReportLine *line)
{
  line->length = 0U;
  line->overflow = 0U;
}

static void line_append_character(ReportLine *line, char value)
{
  if (line->length >= REPORT_LINE_CAPACITY)
  {
    line->overflow = 1U;
    return;
  }
  line->data[line->length++] = value;
}

static void line_append_text(ReportLine *line, const char *text)
{
  while (*text != '\0')
  {
    line_append_character(line, *text++);
  }
}

static void line_append_u32(ReportLine *line, uint32_t value)
{
  char digits[10];
  uint8_t count = 0U;

  do
  {
    digits[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U && count < sizeof(digits));

  while (count != 0U)
  {
    line_append_character(line, digits[--count]);
  }
}

static void line_append_hex32(ReportLine *line, uint32_t value)
{
  static const char hexadecimal[] = "0123456789ABCDEF";
  int8_t shift;

  for (shift = 28; shift >= 0; shift -= 4)
  {
    line_append_character(line,
                          hexadecimal[(value >> (uint8_t)shift) & 0x0FU]);
  }
}

static void line_append_hex8(ReportLine *line, uint8_t value)
{
  static const char hexadecimal[] = "0123456789ABCDEF";

  line_append_character(line, hexadecimal[(value >> 4) & 0x0FU]);
  line_append_character(line, hexadecimal[value & 0x0FU]);
}

static void line_finish(ReportLine *line,
                        NavigationReportWriter writer,
                        void *context)
{
  line_append_text(line, "\r\n");
  if (line->overflow == 0U)
  {
    writer(line->data, line->length, context);
  }
}

static const char *state_name(NavState state)
{
  switch (state)
  {
    case NAV_IDLE:
      return "IDLE";
    case NAV_SCANNING:
      return "SCANNING";
    case NAV_PLANNING:
      return "PLANNING";
    case NAV_PATH_READY:
      return "PATH_READY";
    case NAV_ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

static const char *error_name(NavError error)
{
  switch (error)
  {
    case NAV_ERROR_NONE:
      return "NONE";
    case NAV_ERROR_RADAR:
      return "RADAR";
    case NAV_ERROR_SCAN_TIMEOUT:
      return "SCAN_TIMEOUT";
    case NAV_ERROR_NOT_ENOUGH_POINTS:
      return "NOT_ENOUGH_POINTS";
    case NAV_ERROR_NO_PATH:
      return "NO_PATH";
    case NAV_ERROR_PATH_OVERFLOW:
      return "PATH_OVERFLOW";
    default:
      return "UNKNOWN";
  }
}

static const char *direction_name(uint8_t direction)
{
  switch (direction)
  {
    case NAV_DIR_NORTH:
      return "NORTH";
    case NAV_DIR_EAST:
      return "EAST";
    case NAV_DIR_SOUTH:
      return "SOUTH";
    case NAV_DIR_WEST:
      return "WEST";
    case NAV_DIR_END:
      return "END";
    default:
      return "UNKNOWN";
  }
}

static const char *turn_name(uint8_t turn)
{
  switch (turn)
  {
    case NAV_TURN_START:
      return "START";
    case NAV_TURN_STRAIGHT:
      return "STRAIGHT";
    case NAV_TURN_LEFT:
      return "LEFT";
    case NAV_TURN_RIGHT:
      return "RIGHT";
    case NAV_TURN_REVERSE:
      return "REVERSE";
    case NAV_TURN_END:
      return "END";
    default:
      return "UNKNOWN";
  }
}

static void emit_header(const NavigationResult *result,
                        uint16_t path_count,
                        NavigationReportWriter writer,
                        void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "NAV,state=");
  line_append_text(&line, state_name(result->state));
  line_append_text(&line, ",error=");
  line_append_text(&line, error_name(result->error));
  line_append_text(&line, ",valid=");
  line_append_u32(&line, result->valid_scan_points);
  line_append_text(&line, ",scanned=0x");
  line_append_hex32(&line, result->scanned_obstacle_mask);
  line_append_text(&line, ",effective=0x");
  line_append_hex32(&line, result->effective_obstacle_mask);
  line_append_text(&line, ",path_count=");
  line_append_u32(&line, path_count);
  line_finish(&line, writer, context);
}

static void emit_receiver_diagnostics(const NavigationResult *result,
                                      NavigationReportWriter writer,
                                      void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "RX,bytes=");
  line_append_u32(&line, result->rx_bytes_received);
  line_append_text(&line, ",measurement_packets=");
  line_append_u32(&line, result->rx_measurement_packets);
  line_append_text(&line, ",status_packets=");
  line_append_u32(&line, result->rx_status_packets);
  line_append_text(&line, ",bad_length=");
  line_append_u32(&line, result->rx_bad_length_packets);
  line_append_text(&line, ",bad_checksum=");
  line_append_u32(&line, result->rx_bad_checksum_packets);
  line_append_text(&line, ",discarded=");
  line_append_u32(&line, result->rx_discarded_bytes);
  line_finish(&line, writer, context);
}

static void emit_scan_diagnostics(const NavigationResult *result,
                                  NavigationReportWriter writer,
                                  void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "SCAN,frames=");
  line_append_u32(&line, result->scan_measurement_frames);
  line_append_text(&line, ",raw_points=");
  line_append_u32(&line, result->scan_raw_points);
  line_append_text(&line, ",first_angle=");
  line_append_u32(&line, result->scan_first_angle_tenths);
  line_append_text(&line, ",last_angle=");
  line_append_u32(&line, result->scan_last_angle_tenths);
  line_append_text(&line, ",min_angle=");
  line_append_u32(&line, result->scan_min_angle_tenths);
  line_append_text(&line, ",max_angle=");
  line_append_u32(&line, result->scan_max_angle_tenths);
  line_append_text(&line, ",wraps=");
  line_append_u32(&line, result->scan_wrap_count);
  line_append_text(&line, ",max_declared_points=");
  line_append_u32(&line, result->rx_max_declared_point_count);
  line_finish(&line, writer, context);
}

static void emit_raw_sample(const NavigationResult *result,
                            NavigationReportWriter writer,
                            void *context)
{
  ReportLine line;
  uint8_t index;
  uint8_t count = result->rx_raw_sample_count;

  if (count > LDS_RAW_SAMPLE_CAPACITY)
  {
    count = LDS_RAW_SAMPLE_CAPACITY;
  }

  line_reset(&line);
  line_append_text(&line, "RAW,hex=");
  for (index = 0U; index < count; ++index)
  {
    line_append_hex8(&line, result->rx_raw_sample[index]);
  }
  line_finish(&line, writer, context);
}

static void emit_map_diagnostics(const NavigationResult *result,
                                 NavigationReportWriter writer,
                                 void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "MAP,processed=");
  line_append_u32(&line, result->map_processed_points);
  line_append_text(&line, ",accepted=");
  line_append_u32(&line, result->valid_scan_points);
  line_append_text(&line, ",angle_reject=");
  line_append_u32(&line, result->map_angle_rejected_points);
  line_append_text(&line, ",distance_reject=");
  line_append_u32(&line, result->map_distance_rejected_points);
  line_append_text(&line, ",intensity_reject=");
  line_append_u32(&line, result->map_intensity_rejected_points);
  line_append_text(&line, ",outside=");
  line_append_u32(&line, result->map_outside_field_points);
  line_finish(&line, writer, context);
}

static void emit_range_diagnostics(const NavigationResult *result,
                                   NavigationReportWriter writer,
                                   void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "RANGE,min=");
  line_append_u32(&line, result->map_min_distance_mm);
  line_append_text(&line, ",max=");
  line_append_u32(&line, result->map_max_distance_mm);
  line_append_text(&line, ",status_flags=0x");
  line_append_hex8(&line, result->rx_last_status_flags);
  line_append_text(&line, ",unit_mm=");
  line_append_u32(&line, result->rx_distance_unit_mm);
  line_finish(&line, writer, context);
}

static void emit_end(NavigationReportWriter writer, void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "END");
  line_finish(&line, writer, context);
}

static void emit_cell(const NavigationResult *result,
                      uint8_t index,
                      NavigationReportWriter writer,
                      void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "CELL,index=");
  line_append_u32(&line, (uint32_t)index + 1U);
  line_append_text(&line, ",row=");
  line_append_u32(&line, (uint32_t)(index / NAV_GRID_COLUMNS) + 1U);
  line_append_text(&line, ",column=");
  line_append_u32(&line, (uint32_t)(index % NAV_GRID_COLUMNS) + 1U);
  line_append_text(&line, ",count=");
  line_append_u32(&line, result->cell_point_counts[index]);
  line_finish(&line, writer, context);
}

static void emit_waypoint(const NavWaypoint *waypoint,
                          uint16_t index,
                          NavigationReportWriter writer,
                          void *context)
{
  ReportLine line;

  line_reset(&line);
  line_append_text(&line, "WP,index=");
  line_append_u32(&line, (uint32_t)index + 1U);
  line_append_text(&line, ",row=");
  line_append_u32(&line, waypoint->row);
  line_append_text(&line, ",column=");
  line_append_u32(&line, waypoint->column);
  line_append_text(&line, ",x=");
  line_append_u32(&line, waypoint->x_mm);
  line_append_text(&line, ",y=");
  line_append_u32(&line, waypoint->y_mm);
  line_append_text(&line, ",distance=");
  line_append_u32(&line, waypoint->distance_to_next_mm);
  line_append_text(&line, ",direction=");
  line_append_text(&line, direction_name(waypoint->direction));
  line_append_text(&line, ",turn=");
  line_append_text(&line, turn_name(waypoint->turn));
  line_finish(&line, writer, context);
}

void NavigationReport_Emit(const NavigationResult *result,
                           NavigationReportWriter writer,
                           void *context)
{
  uint16_t path_count;
  uint16_t index;

  if (result == 0 || writer == 0)
  {
    return;
  }

  path_count = result->path_count;
  if (path_count > NAV_MAX_WAYPOINTS)
  {
    path_count = NAV_MAX_WAYPOINTS;
  }

  emit_header(result, path_count, writer, context);
  emit_receiver_diagnostics(result, writer, context);
  emit_scan_diagnostics(result, writer, context);
  emit_raw_sample(result, writer, context);
  emit_map_diagnostics(result, writer, context);
  emit_range_diagnostics(result, writer, context);
  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    emit_cell(result, (uint8_t)index, writer, context);
  }
  for (index = 0U; index < path_count; ++index)
  {
    emit_waypoint(&result->path[index], index, writer, context);
  }

  emit_end(writer, context);
}

void NavigationReportSession_Init(NavigationReportSession *session)
{
  if (session != 0)
  {
    session->sent = 0U;
  }
}

void NavigationReportSession_Poll(NavigationReportSession *session,
                                  const NavigationResult *result,
                                  NavigationReportWriter writer,
                                  void *context)
{
  if (session == 0 || result == 0 || writer == 0 || session->sent != 0U ||
      (result->state != NAV_PATH_READY && result->state != NAV_ERROR))
  {
    return;
  }

  session->sent = 1U;
  NavigationReport_Emit(result, writer, context);
}
