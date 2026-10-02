#include "navigation.h"

#include "grid_map.h"
#include "path_planner.h"
#include "scan_mapper.h"

NavigationResult g_navigation_result;

static const NavigationPort *g_navigation_port;
static ScanMapper g_scan_mapper;
static LdsReceiverEvent g_navigation_event_scratch;
static LdsParserStats g_parser_stats_scratch;
static uint32_t g_scan_start_tick;
static uint8_t g_receiver_stopped;

static void clear_result(void)
{
  uint16_t index;

  g_navigation_result.state = NAV_IDLE;
  g_navigation_result.error = NAV_ERROR_NONE;
  g_navigation_result.failed_from_task = 0U;
  g_navigation_result.failed_to_task = 0U;
  g_navigation_result.scanned_obstacle_mask = 0U;
  g_navigation_result.effective_obstacle_mask = 0U;
  g_navigation_result.valid_scan_points = 0U;
  g_navigation_result.rx_bytes_received = 0U;
  g_navigation_result.rx_measurement_packets = 0U;
  g_navigation_result.rx_status_packets = 0U;
  g_navigation_result.rx_bad_length_packets = 0U;
  g_navigation_result.rx_bad_checksum_packets = 0U;
  g_navigation_result.rx_discarded_bytes = 0U;
  g_navigation_result.rx_max_declared_point_count = 0U;
  g_navigation_result.rx_last_status_flags = 0U;
  g_navigation_result.rx_distance_unit_mm = 0U;
  g_navigation_result.rx_raw_sample_count = 0U;
  for (index = 0U; index < LDS_RAW_SAMPLE_CAPACITY; ++index)
  {
    g_navigation_result.rx_raw_sample[index] = 0U;
  }
  g_navigation_result.scan_measurement_frames = 0U;
  g_navigation_result.scan_raw_points = 0U;
  g_navigation_result.scan_first_angle_tenths = 0U;
  g_navigation_result.scan_last_angle_tenths = 0U;
  g_navigation_result.scan_min_angle_tenths = 0U;
  g_navigation_result.scan_max_angle_tenths = 0U;
  g_navigation_result.scan_wrap_count = 0U;
  g_navigation_result.map_processed_points = 0U;
  g_navigation_result.map_angle_rejected_points = 0U;
  g_navigation_result.map_distance_rejected_points = 0U;
  g_navigation_result.map_intensity_rejected_points = 0U;
  g_navigation_result.map_outside_field_points = 0U;
  g_navigation_result.map_min_distance_mm = 0U;
  g_navigation_result.map_max_distance_mm = 0U;
  g_navigation_result.path_count = 0U;
  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    g_navigation_result.cell_point_counts[index] = 0U;
  }
  for (index = 0U; index < NAV_MAX_WAYPOINTS; ++index)
  {
    g_navigation_result.path[index].x_mm = 0U;
    g_navigation_result.path[index].y_mm = 0U;
    g_navigation_result.path[index].distance_to_next_mm = 0U;
    g_navigation_result.path[index].row = 0U;
    g_navigation_result.path[index].column = 0U;
    g_navigation_result.path[index].direction = NAV_DIR_END;
    g_navigation_result.path[index].turn = NAV_TURN_END;
  }
}

static void publish_scan_diagnostics(void)
{
  uint8_t index;

  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    g_navigation_result.cell_point_counts[index] =
        g_scan_mapper.cell_point_counts[index];
  }
  g_navigation_result.valid_scan_points = g_scan_mapper.valid_scan_points;
  g_navigation_result.scan_measurement_frames =
      g_scan_mapper.measurement_frame_count;
  g_navigation_result.scan_raw_points = g_scan_mapper.raw_point_count;
  g_navigation_result.scan_first_angle_tenths =
      g_scan_mapper.first_start_angle_tenths;
  g_navigation_result.scan_last_angle_tenths =
      g_scan_mapper.last_start_angle_tenths;
  g_navigation_result.scan_min_angle_tenths =
      g_scan_mapper.min_start_angle_tenths;
  g_navigation_result.scan_max_angle_tenths =
      g_scan_mapper.max_start_angle_tenths;
  g_navigation_result.scan_wrap_count = g_scan_mapper.wrap_count;
  g_navigation_result.map_processed_points =
      g_scan_mapper.processed_point_count;
  g_navigation_result.map_angle_rejected_points =
      g_scan_mapper.angle_rejected_point_count;
  g_navigation_result.map_distance_rejected_points =
      g_scan_mapper.distance_rejected_point_count;
  g_navigation_result.map_intensity_rejected_points =
      g_scan_mapper.intensity_rejected_point_count;
  g_navigation_result.map_outside_field_points =
      g_scan_mapper.outside_field_point_count;
  g_navigation_result.map_min_distance_mm = g_scan_mapper.min_distance_mm;
  g_navigation_result.map_max_distance_mm = g_scan_mapper.max_distance_mm;

  if (g_navigation_port != 0 && g_navigation_port->receiver_stats != 0)
  {
    g_navigation_port->receiver_stats(&g_parser_stats_scratch);
    g_navigation_result.rx_bytes_received =
        g_parser_stats_scratch.bytes_received;
    g_navigation_result.rx_measurement_packets =
        g_parser_stats_scratch.good_measurement_packets;
    g_navigation_result.rx_status_packets =
        g_parser_stats_scratch.good_status_packets;
    g_navigation_result.rx_bad_length_packets =
        g_parser_stats_scratch.bad_length_packets;
    g_navigation_result.rx_bad_checksum_packets =
        g_parser_stats_scratch.bad_checksum_packets;
    g_navigation_result.rx_discarded_bytes =
        g_parser_stats_scratch.discarded_bytes;
    g_navigation_result.rx_max_declared_point_count =
        g_parser_stats_scratch.max_declared_point_count;
    g_navigation_result.rx_last_status_flags =
        g_parser_stats_scratch.last_status_flags;
    g_navigation_result.rx_distance_unit_mm =
        g_parser_stats_scratch.last_status_flags & 0x01U;
    g_navigation_result.rx_raw_sample_count =
        g_parser_stats_scratch.raw_sample_count;
    if (g_navigation_result.rx_raw_sample_count > LDS_RAW_SAMPLE_CAPACITY)
    {
      g_navigation_result.rx_raw_sample_count = LDS_RAW_SAMPLE_CAPACITY;
    }
    for (index = 0U;
         index < g_navigation_result.rx_raw_sample_count;
         ++index)
    {
      g_navigation_result.rx_raw_sample[index] =
          g_parser_stats_scratch.raw_sample[index];
    }
  }
  GridMap_BuildMasks(g_navigation_result.cell_point_counts,
                     &g_navigation_result.scanned_obstacle_mask,
                     &g_navigation_result.effective_obstacle_mask);
}

static void stop_receiver_once(void)
{
  if (g_receiver_stopped == 0U && g_navigation_port != 0 &&
      g_navigation_port->receiver_stop != 0)
  {
    g_receiver_stopped = 1U;
    g_navigation_port->receiver_stop();
  }
}

static void enter_error(NavError error)
{
  if (g_navigation_result.state == NAV_ERROR ||
      g_navigation_result.state == NAV_PATH_READY)
  {
    return;
  }

  publish_scan_diagnostics();
  if (g_navigation_result.error == NAV_ERROR_NONE)
  {
    g_navigation_result.error = error;
  }
  stop_receiver_once();
  g_navigation_result.state = NAV_ERROR;
}

void Navigation_SetPort(const NavigationPort *port)
{
  g_navigation_port = port;
}

void Navigation_Init(void)
{
#ifndef NAVIGATION_UNIT_TEST
  if (g_navigation_port == 0)
  {
    g_navigation_port = NavigationPort_Default();
  }
#endif

  clear_result();
  ScanMapper_Init(&g_scan_mapper);
  g_receiver_stopped = 0U;
  g_scan_start_tick = 0U;
  if (g_navigation_port != 0 && g_navigation_port->receiver_init != 0)
  {
    g_navigation_port->receiver_init();
  }
}

void Navigation_Start(void)
{
  if (g_navigation_result.state != NAV_IDLE)
  {
    return;
  }
  if (g_navigation_port == 0 || g_navigation_port->receiver_start == 0 ||
      g_navigation_port->receiver_status == 0 ||
      g_navigation_port->get_tick == 0 ||
      g_navigation_port->receiver_status() != LDS_RECEIVER_OK)
  {
    enter_error(NAV_ERROR_RADAR);
    return;
  }

  clear_result();
  ScanMapper_Init(&g_scan_mapper);
  g_receiver_stopped = 0U;
  g_scan_start_tick = g_navigation_port->get_tick();
  g_navigation_result.state = NAV_SCANNING;
  g_navigation_port->receiver_start();
  if (g_navigation_port->receiver_status() != LDS_RECEIVER_OK)
  {
    enter_error(NAV_ERROR_RADAR);
  }
}

void Navigation_Poll(void)
{
  if (g_navigation_result.state != NAV_SCANNING ||
      g_navigation_port == 0)
  {
    return;
  }

  if (g_navigation_port->receiver_poll != 0)
  {
    g_navigation_port->receiver_poll();
  }
  if (g_navigation_port->receiver_status == 0 ||
      g_navigation_port->receiver_status() != LDS_RECEIVER_OK)
  {
    enter_error(NAV_ERROR_RADAR);
    return;
  }

  if (g_navigation_port->read_event != 0)
  {
    while (g_navigation_port->read_event(
               &g_navigation_event_scratch) != 0U)
    {
      if (g_navigation_event_scratch.type ==
          LDS_RECEIVER_EVENT_MEASUREMENT)
      {
        ScanMapper_PushFrame(
            &g_scan_mapper,
            &g_navigation_event_scratch.payload.measurement);
      }
      if (ScanMapper_IsComplete(&g_scan_mapper) != 0U)
      {
        break;
      }
    }
  }

  if (ScanMapper_IsComplete(&g_scan_mapper) != 0U)
  {
    stop_receiver_once();
    if (g_navigation_port->receiver_status() != LDS_RECEIVER_OK)
    {
      enter_error(NAV_ERROR_RADAR);
      return;
    }

    publish_scan_diagnostics();
    g_navigation_result.state = NAV_PLANNING;
    if (PathPlanner_BuildMissionPath(
            g_navigation_result.effective_obstacle_mask,
            &g_navigation_result) == 0U)
    {
      g_navigation_result.state = NAV_ERROR;
      return;
    }
    g_navigation_result.state = NAV_PATH_READY;
    return;
  }

  if ((uint32_t)(g_navigation_port->get_tick() - g_scan_start_tick) >=
      NAV_SCAN_TIMEOUT_MS)
  {
    enter_error(NAV_ERROR_SCAN_TIMEOUT);
  }
}
