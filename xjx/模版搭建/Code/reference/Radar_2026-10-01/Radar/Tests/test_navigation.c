#include <stdint.h>
#include <string.h>

#include "grid_map.h"
#include "lds_dma_cursor.h"
#include "lds_result_queue.h"
#include "nav_math.h"
#include "navigation.h"
#include "navigation_config.h"
#include "navigation_port.h"
#include "navigation_report.h"
#include "navigation_types.h"
#include "path_planner.h"
#include "scan_mapper.h"

volatile uint32_t g_test_checks;
volatile uint32_t g_test_failures;
volatile uint32_t g_test_failure_lines[16];

#define CHECK(condition)                \
  do                                    \
  {                                     \
    ++g_test_checks;                    \
    if (!(condition))                   \
    {                                   \
      if (g_test_failures < 16U)        \
      {                                 \
        g_test_failure_lines[g_test_failures] = __LINE__; \
      }                                 \
      ++g_test_failures;                \
    }                                   \
  } while (0)

static void test_configuration_describes_the_required_field(void)
{
  CHECK(NAV_GRID_ROWS == 5U);
  CHECK(NAV_GRID_COLUMNS == 5U);
  CHECK(NAV_MAX_WAYPOINTS == 193U);
  CHECK(g_nav_grid_x_mm[0] == 150U);
  CHECK(g_nav_grid_x_mm[5] == 2250U);
  CHECK(g_nav_grid_y_mm[0] == 150U);
  CHECK(g_nav_grid_y_mm[5] == 2250U);
  CHECK(g_nav_mission_order[0] == 1U);
  CHECK(g_nav_mission_order[8] == 1U);
  CHECK((NAV_FIXED_OBSTACLE_MASK & NAV_CELL_MASK(2U, 2U)) != 0U);
  CHECK((NAV_FIXED_OBSTACLE_MASK & NAV_CELL_MASK(4U, 4U)) != 0U);
  CHECK((NAV_PROTECTED_CELL_MASK & NAV_CELL_MASK(3U, 1U)) != 0U);
  CHECK((NAV_PROTECTED_CELL_MASK & NAV_CELL_MASK(1U, 3U)) != 0U);
  CHECK((NAV_PROTECTED_CELL_MASK & NAV_CELL_MASK(1U, 1U)) != 0U);
}

static void test_polar_coordinates_follow_the_lidar_axis_convention(void)
{
  int32_t x;
  int32_t y;

  NavMath_PolarToCartesian(0U, 1000U, &x, &y);
  CHECK(x == 230);
  CHECK(y == 1230);

  NavMath_PolarToCartesian(900U, 1000U, &x, &y);
  CHECK(x == 1230);
  CHECK(y == 230);

  NavMath_PolarToCartesian(1800U, 1000U, &x, &y);
  CHECK(x == 230);
  CHECK(y == -770);

  NavMath_PolarToCartesian(2700U, 1000U, &x, &y);
  CHECK(x == -770);
  CHECK(y == 230);

  NavMath_PolarToCartesian(900U, 40000U, &x, &y);
  CHECK(x == 40230);
  CHECK(y == 230);
}

static void test_grid_boundaries_use_half_open_intervals(void)
{
  uint8_t row;
  uint8_t column;

  CHECK(GridMap_FindCell(150, 150, &row, &column) != 0U);
  CHECK(row == 1U && column == 1U);
  CHECK(GridMap_FindCell(549, 549, &row, &column) != 0U);
  CHECK(row == 1U && column == 1U);
  CHECK(GridMap_FindCell(550, 550, &row, &column) != 0U);
  CHECK(row == 2U && column == 2U);
  CHECK(GridMap_FindCell(2249, 2249, &row, &column) != 0U);
  CHECK(row == 5U && column == 5U);
  CHECK(GridMap_FindCell(2250, 1000, &row, &column) == 0U);
  CHECK(GridMap_FindCell(-1, 1000, &row, &column) == 0U);
  CHECK(GridMap_FindCell(1000, -1, &row, &column) == 0U);
}

static void test_grid_centers_and_obstacle_priority_are_stable(void)
{
  uint16_t counts[NAV_GRID_CELL_COUNT] = {0U};
  uint16_t x;
  uint16_t y;
  uint32_t scanned;
  uint32_t effective;

  GridMap_CellCenter(1U, 1U, &x, &y);
  CHECK(x == 350U && y == 350U);
  GridMap_CellCenter(3U, 5U, &x, &y);
  CHECK(x == 2050U && y == 1200U);

  counts[GridMap_CellIndex(1U, 1U)] = 3U;
  counts[GridMap_CellIndex(1U, 2U)] = 3U;
  counts[GridMap_CellIndex(3U, 1U)] = 3U;
  GridMap_BuildMasks(counts, &scanned, &effective);

  CHECK((scanned & NAV_CELL_MASK(1U, 1U)) == 0U);
  CHECK((scanned & NAV_CELL_MASK(1U, 2U)) != 0U);
  CHECK((scanned & NAV_CELL_MASK(3U, 1U)) == 0U);
  CHECK((effective & NAV_FIXED_OBSTACLE_MASK) == NAV_FIXED_OBSTACLE_MASK);
  CHECK((effective & NAV_CELL_MASK(1U, 2U)) != 0U);
  CHECK(GridMap_IsBlocked(effective, 4U, 4U) != 0U);
}

static void test_one_point_marks_an_obstacle_but_an_empty_cell_does_not(void)
{
  uint16_t counts[NAV_GRID_CELL_COUNT] = {0U};
  uint32_t scanned;
  uint32_t effective;

  counts[GridMap_CellIndex(3U, 3U)] = 1U;
  GridMap_BuildMasks(counts, &scanned, &effective);

  CHECK((scanned & NAV_CELL_MASK(3U, 2U)) == 0U);
  CHECK((scanned & NAV_CELL_MASK(3U, 3U)) != 0U);
  CHECK((effective & NAV_CELL_MASK(3U, 2U)) == 0U);
  CHECK((effective & NAV_CELL_MASK(3U, 3U)) != 0U);
}

static LdsParserResult make_fixed_frame(uint16_t start_angle,
                                        uint16_t sector_angle,
                                        uint16_t point_count,
                                        uint16_t distance_mm)
{
  LdsParserResult frame;
  uint16_t index;

  memset(&frame, 0, sizeof(frame));
  frame.packet_type = LDS_PACKET_FIXED_RESOLUTION;
  frame.start_angle_tenths = start_angle;
  frame.sector_angle_tenths = sector_angle;
  frame.point_count = point_count;
  for (index = 0U; index < point_count; ++index)
  {
    frame.points[index].distance_mm = distance_mm;
    frame.points[index].intensity = 50U;
  }
  return frame;
}

static LdsParserResult make_variable_frame(uint16_t start_angle,
                                           uint16_t point_count,
                                           uint16_t distance_mm)
{
  LdsParserResult frame;
  uint16_t index;

  memset(&frame, 0, sizeof(frame));
  frame.packet_type = LDS_PACKET_VARIABLE_RESOLUTION;
  frame.start_angle_tenths = start_angle;
  frame.point_count = point_count;
  for (index = 0U; index < point_count; ++index)
  {
    frame.points[index].distance_mm = distance_mm;
    frame.points[index].intensity = 50U;
  }
  return frame;
}

static void test_fixed_frames_start_and_stop_on_distinct_wraps(void)
{
  ScanMapper mapper;
  LdsParserResult frame;
  uint16_t frozen_count;

  ScanMapper_Init(&mapper);
  frame = make_fixed_frame(3500U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.state == SCAN_WAIT_FIRST_WRAP);
  CHECK(mapper.valid_scan_points == 0U);

  frame = make_fixed_frame(100U, 800U, 2U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.state == SCAN_COLLECTING);
  CHECK(mapper.valid_scan_points == 2U);

  frame = make_fixed_frame(3500U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == 3U);

  frame = make_fixed_frame(50U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(ScanMapper_IsComplete(&mapper) != 0U);
  CHECK(mapper.valid_scan_points == 3U);

  frozen_count = mapper.valid_scan_points;
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == frozen_count);
}

static void test_any_start_angle_decrease_marks_a_revolution_wrap(void)
{
  ScanMapper mapper;
  LdsParserResult frame;

  ScanMapper_Init(&mapper);
  frame = make_fixed_frame(2900U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_fixed_frame(100U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.state == SCAN_COLLECTING);

  ScanMapper_Init(&mapper);
  frame = make_fixed_frame(1200U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_fixed_frame(1100U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.state == SCAN_COLLECTING);
}

static void test_scan_mapper_records_raw_frame_and_angle_diagnostics(void)
{
  ScanMapper mapper;
  LdsParserResult frame;

  ScanMapper_Init(&mapper);
  frame = make_fixed_frame(3500U, 0U, 3U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_fixed_frame(100U, 0U, 2U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_fixed_frame(900U, 0U, 4U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_fixed_frame(50U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);

  CHECK(mapper.measurement_frame_count == 4U);
  CHECK(mapper.raw_point_count == 10U);
  CHECK(mapper.first_start_angle_tenths == 3500U);
  CHECK(mapper.last_start_angle_tenths == 50U);
  CHECK(mapper.min_start_angle_tenths == 50U);
  CHECK(mapper.max_start_angle_tenths == 3500U);
  CHECK(mapper.wrap_count == 2U);
}

static void test_variable_frames_settle_the_last_pending_frame_before_completion(void)
{
  ScanMapper mapper;
  LdsParserResult frame;

  ScanMapper_Init(&mapper);
  frame = make_variable_frame(3500U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_variable_frame(100U, 2U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.state == SCAN_COLLECTING);
  CHECK(mapper.valid_scan_points == 0U);

  frame = make_variable_frame(500U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == 2U);

  frame = make_variable_frame(3500U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == 3U);

  frame = make_variable_frame(100U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(ScanMapper_IsComplete(&mapper) != 0U);
  CHECK(mapper.valid_scan_points == 4U);
}

static void test_scan_filter_rejects_short_and_out_of_field_points(void)
{
  ScanMapper mapper;
  LdsParserResult frame;

  ScanMapper_Init(&mapper);
  frame = make_fixed_frame(3500U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  frame = make_fixed_frame(0U, 0U, 1U, 50U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == 0U);

  frame = make_fixed_frame(900U, 0U, 1U, 40000U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == 0U);

  frame = make_fixed_frame(1000U, 0U, 1U, 200U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.valid_scan_points == 1U);

  frame = make_fixed_frame(1100U, 0U, 1U, 3000U);
  ScanMapper_PushFrame(&mapper, &frame);
  CHECK(mapper.processed_point_count == 4U);
  CHECK(mapper.distance_rejected_point_count == 1U);
  CHECK(mapper.outside_field_point_count == 2U);
  CHECK(mapper.valid_scan_points == 1U);
  CHECK(mapper.min_distance_mm == 50U);
  CHECK(mapper.max_distance_mm == 40000U);
}

static void test_default_mission_path_matches_the_33_cell_reference(void)
{
  static const uint8_t expected_rows[33] = {
      1U, 1U, 1U, 2U, 3U, 3U, 3U, 3U, 3U, 3U, 3U,
      4U, 5U, 5U, 5U, 5U, 5U, 4U, 3U, 3U, 3U, 3U,
      3U, 4U, 5U, 5U, 5U, 4U, 3U, 2U, 1U, 1U, 1U};
  static const uint8_t expected_columns[33] = {
      1U, 2U, 3U, 3U, 3U, 4U, 5U, 4U, 3U, 2U, 1U,
      1U, 1U, 2U, 3U, 4U, 5U, 5U, 5U, 4U, 3U, 2U,
      1U, 1U, 1U, 2U, 3U, 3U, 3U, 3U, 3U, 2U, 1U};
  NavigationResult result;
  uint16_t index;

  memset(&result, 0, sizeof(result));
  CHECK(PathPlanner_BuildMissionPath(NAV_FIXED_OBSTACLE_MASK, &result) != 0U);
  CHECK(result.path_count == 33U);
  for (index = 0U; index < 33U; ++index)
  {
    CHECK(result.path[index].row == expected_rows[index]);
    CHECK(result.path[index].column == expected_columns[index]);
  }
  CHECK(result.path[0].x_mm == NAV_RADAR_X_MM);
  CHECK(result.path[0].y_mm == NAV_RADAR_Y_MM);
  CHECK(result.path[32].x_mm == NAV_RADAR_X_MM);
  CHECK(result.path[32].y_mm == NAV_RADAR_Y_MM);
}

static void test_waypoints_describe_distance_direction_and_turn(void)
{
  NavigationResult result;

  memset(&result, 0, sizeof(result));
  CHECK(PathPlanner_BuildMissionPath(NAV_FIXED_OBSTACLE_MASK, &result) != 0U);
  CHECK(result.path[0].distance_to_next_mm == 558U);
  CHECK(result.path[0].direction == NAV_DIR_EAST);
  CHECK(result.path[0].turn == NAV_TURN_START);
  CHECK(result.path[1].distance_to_next_mm == 425U);
  CHECK(result.path[1].direction == NAV_DIR_EAST);
  CHECK(result.path[1].turn == NAV_TURN_STRAIGHT);
  CHECK(result.path[2].direction == NAV_DIR_NORTH);
  CHECK(result.path[2].turn == NAV_TURN_LEFT);
  CHECK(result.path[6].direction == NAV_DIR_WEST);
  CHECK(result.path[6].turn == NAV_TURN_REVERSE);
  CHECK(result.path[32].distance_to_next_mm == 0U);
  CHECK(result.path[32].direction == NAV_DIR_END);
  CHECK(result.path[32].turn == NAV_TURN_END);
}

static void test_planner_reroutes_and_reports_the_failed_mission_leg(void)
{
  NavigationResult result;
  uint32_t reroute_mask = NAV_FIXED_OBSTACLE_MASK | NAV_CELL_MASK(3U, 4U);
  uint32_t trapped_mask = NAV_FIXED_OBSTACLE_MASK |
                          NAV_CELL_MASK(1U, 2U) |
                          NAV_CELL_MASK(2U, 1U);
  uint16_t index;

  memset(&result, 0, sizeof(result));
  CHECK(PathPlanner_BuildMissionPath(reroute_mask, &result) != 0U);
  for (index = 0U; index < result.path_count; ++index)
  {
    CHECK(!(result.path[index].row == 3U && result.path[index].column == 4U));
  }

  CHECK(PathPlanner_BuildMissionPath(trapped_mask, &result) == 0U);
  CHECK(result.error == NAV_ERROR_NO_PATH);
  CHECK(result.failed_from_task == 1U);
  CHECK(result.failed_to_task == 5U);
}

static void test_path_capacity_is_checked_before_each_write(void)
{
  NavigationResult result;

  memset(&result, 0, sizeof(result));
  result.path[2].x_mm = 0xA5A5U;
  CHECK(PathPlanner_BuildMissionPathLimited(NAV_FIXED_OBSTACLE_MASK,
                                            &result,
                                            2U) == 0U);
  CHECK(result.error == NAV_ERROR_PATH_OVERFLOW);
  CHECK(result.path[2].x_mm == 0xA5A5U);
}

typedef struct
{
  uint32_t tick;
  LdsReceiverStatus status;
  uint8_t stop_fails;
  uint8_t init_calls;
  uint8_t start_calls;
  uint8_t stop_calls;
  uint8_t poll_calls;
  LdsReceiverEvent events[16];
  uint8_t event_count;
  uint8_t event_index;
  LdsParserStats parser_stats;
} FakeNavigationPortState;

static FakeNavigationPortState g_fake_navigation;

static void fake_receiver_init(void)
{
  ++g_fake_navigation.init_calls;
}

static void fake_receiver_start(void)
{
  ++g_fake_navigation.start_calls;
}

static void fake_receiver_stop(void)
{
  ++g_fake_navigation.stop_calls;
  if (g_fake_navigation.stop_fails != 0U)
  {
    g_fake_navigation.status = LDS_RECEIVER_COMMAND_FAILED;
  }
}

static void fake_receiver_poll(void)
{
  ++g_fake_navigation.poll_calls;
}

static uint8_t fake_read_event(LdsReceiverEvent *event)
{
  if (g_fake_navigation.event_index >= g_fake_navigation.event_count)
  {
    return 0U;
  }
  *event = g_fake_navigation.events[g_fake_navigation.event_index++];
  return 1U;
}

static LdsReceiverStatus fake_receiver_status(void)
{
  return g_fake_navigation.status;
}

static void fake_receiver_stats(LdsParserStats *stats)
{
  if (stats != 0)
  {
    *stats = g_fake_navigation.parser_stats;
  }
}

static uint32_t fake_get_tick(void)
{
  return g_fake_navigation.tick;
}

static const NavigationPort g_fake_navigation_port = {
    fake_receiver_init,
    fake_receiver_start,
    fake_receiver_stop,
    fake_receiver_poll,
    fake_read_event,
    fake_receiver_status,
    fake_receiver_stats,
    fake_get_tick};

static void fake_navigation_reset(uint32_t tick)
{
  memset(&g_fake_navigation, 0, sizeof(g_fake_navigation));
  g_fake_navigation.tick = tick;
  g_fake_navigation.status = LDS_RECEIVER_OK;
  Navigation_SetPort(&g_fake_navigation_port);
  Navigation_Init();
}

static void fake_queue_frame(LdsParserResult frame)
{
  LdsReceiverEvent *event =
      &g_fake_navigation.events[g_fake_navigation.event_count++];

  memset(event, 0, sizeof(*event));
  event->type = LDS_RECEIVER_EVENT_MEASUREMENT;
  event->payload.measurement = frame;
}

static void fake_queue_status_marker(void)
{
  LdsReceiverEvent *event =
      &g_fake_navigation.events[g_fake_navigation.event_count++];

  memset(event, 0, sizeof(*event));
  event->type = LDS_RECEIVER_EVENT_STATUS;
}

static void fake_queue_complete_scan(uint16_t accepted_point_count)
{
  fake_queue_status_marker();
  fake_queue_frame(make_fixed_frame(200U, 0U, 1U, 200U));
  fake_queue_frame(make_fixed_frame(100U, 0U, 1U, 200U));
  fake_queue_status_marker();
  fake_queue_frame(make_fixed_frame(830U, 0U, accepted_point_count, 977U));
  fake_queue_frame(make_fixed_frame(900U, 0U, 1U, 200U));
  fake_queue_frame(make_fixed_frame(800U, 0U, 1U, 200U));
}

static void test_navigation_runs_once_and_publishes_a_stable_path(void)
{
  uint16_t frozen_path_count;

  fake_navigation_reset(100U);
  CHECK(g_fake_navigation.init_calls == 1U);
  CHECK(g_navigation_result.state == NAV_IDLE);
  Navigation_Start();
  Navigation_Start();
  CHECK(g_fake_navigation.start_calls == 1U);
  CHECK(g_navigation_result.state == NAV_SCANNING);

  fake_queue_complete_scan(30U);
  Navigation_Poll();
  CHECK(g_navigation_result.state == NAV_PATH_READY);
  CHECK(g_navigation_result.error == NAV_ERROR_NONE);
  CHECK(g_fake_navigation.stop_calls == 1U);
  CHECK(g_navigation_result.valid_scan_points == 32U);
  CHECK(g_navigation_result.path_count == 33U);

  frozen_path_count = g_navigation_result.path_count;
  Navigation_Poll();
  CHECK(g_navigation_result.path_count == frozen_path_count);
  CHECK(g_fake_navigation.stop_calls == 1U);
  CHECK(g_fake_navigation.start_calls == 1U);
}

static void test_navigation_timeout_handles_tick_wrap_and_preserves_root_error(void)
{
  uint32_t start_tick = 0xFFFFFF00UL;

  fake_navigation_reset(start_tick);
  Navigation_Start();
  g_fake_navigation.parser_stats.bytes_received = 1234U;
  g_fake_navigation.parser_stats.good_measurement_packets = 7U;
  g_fake_navigation.parser_stats.good_status_packets = 1U;
  g_fake_navigation.parser_stats.bad_length_packets = 2U;
  g_fake_navigation.parser_stats.bad_checksum_packets = 3U;
  g_fake_navigation.parser_stats.discarded_bytes = 4U;
  g_fake_navigation.parser_stats.max_declared_point_count = 300U;
  g_fake_navigation.parser_stats.last_status_flags = 0x0FU;
  g_fake_navigation.parser_stats.raw_sample_count = 3U;
  g_fake_navigation.parser_stats.raw_sample[0] = 0xCEU;
  g_fake_navigation.parser_stats.raw_sample[1] = 0xFAU;
  g_fake_navigation.parser_stats.raw_sample[2] = 0x01U;
  fake_queue_frame(make_fixed_frame(1200U, 0U, 5U, 200U));
  Navigation_Poll();
  g_fake_navigation.stop_fails = 1U;
  g_fake_navigation.tick = start_tick + NAV_SCAN_TIMEOUT_MS - 1U;
  Navigation_Poll();
  CHECK(g_navigation_result.state == NAV_SCANNING);

  g_fake_navigation.tick = start_tick + NAV_SCAN_TIMEOUT_MS;
  Navigation_Poll();
  CHECK(g_navigation_result.state == NAV_ERROR);
  CHECK(g_navigation_result.error == NAV_ERROR_SCAN_TIMEOUT);
  CHECK(g_fake_navigation.stop_calls == 1U);
  CHECK(g_navigation_result.rx_bytes_received == 1234U);
  CHECK(g_navigation_result.rx_measurement_packets == 7U);
  CHECK(g_navigation_result.rx_status_packets == 1U);
  CHECK(g_navigation_result.rx_bad_length_packets == 2U);
  CHECK(g_navigation_result.rx_bad_checksum_packets == 3U);
  CHECK(g_navigation_result.rx_discarded_bytes == 4U);
  CHECK(g_navigation_result.rx_max_declared_point_count == 300U);
  CHECK(g_navigation_result.rx_last_status_flags == 0x0FU);
  CHECK(g_navigation_result.rx_distance_unit_mm == 1U);
  CHECK(g_navigation_result.rx_raw_sample_count == 3U);
  CHECK(g_navigation_result.rx_raw_sample[0] == 0xCEU);
  CHECK(g_navigation_result.rx_raw_sample[1] == 0xFAU);
  CHECK(g_navigation_result.rx_raw_sample[2] == 0x01U);
  CHECK(g_navigation_result.scan_measurement_frames == 1U);
  CHECK(g_navigation_result.scan_raw_points == 5U);
  CHECK(g_navigation_result.scan_first_angle_tenths == 1200U);
  CHECK(g_navigation_result.scan_last_angle_tenths == 1200U);
  CHECK(g_navigation_result.scan_min_angle_tenths == 1200U);
  CHECK(g_navigation_result.scan_max_angle_tenths == 1200U);
  CHECK(g_navigation_result.scan_wrap_count == 0U);
}

static void test_navigation_accepts_a_sparse_complete_scan_and_reports_receiver_failure(void)
{
  fake_navigation_reset(0U);
  Navigation_Start();
  fake_queue_complete_scan(1U);
  Navigation_Poll();
  CHECK(g_navigation_result.state == NAV_PATH_READY);
  CHECK(g_navigation_result.error == NAV_ERROR_NONE);
  CHECK(g_navigation_result.valid_scan_points > 0U);
  CHECK(g_navigation_result.path_count > 0U);

  fake_navigation_reset(0U);
  Navigation_Start();
  g_fake_navigation.status = LDS_RECEIVER_DMA_START_FAILED;
  Navigation_Poll();
  CHECK(g_navigation_result.state == NAV_ERROR);
  CHECK(g_navigation_result.error == NAV_ERROR_RADAR);
}

static void test_navigation_keeps_failed_task_numbers_when_map_is_unreachable(void)
{
  fake_navigation_reset(0U);
  Navigation_Start();
  fake_queue_status_marker();
  fake_queue_frame(make_fixed_frame(200U, 0U, 1U, 200U));
  fake_queue_frame(make_fixed_frame(100U, 0U, 1U, 200U));
  fake_queue_status_marker();
  fake_queue_frame(make_fixed_frame(124U, 0U, 3U, 558U));
  fake_queue_frame(make_fixed_frame(776U, 0U, 3U, 558U));
  fake_queue_frame(make_fixed_frame(830U, 0U, 30U, 977U));
  fake_queue_frame(make_fixed_frame(900U, 0U, 1U, 200U));
  fake_queue_frame(make_fixed_frame(800U, 0U, 1U, 200U));
  Navigation_Poll();

  CHECK(g_navigation_result.state == NAV_ERROR);
  CHECK(g_navigation_result.error == NAV_ERROR_NO_PATH);
  CHECK(g_navigation_result.failed_from_task == 1U);
  CHECK(g_navigation_result.failed_to_task == 5U);
  CHECK((g_navigation_result.scanned_obstacle_mask & NAV_CELL_MASK(1U, 2U)) != 0U);
  CHECK((g_navigation_result.scanned_obstacle_mask & NAV_CELL_MASK(2U, 1U)) != 0U);
}

static void test_receiver_event_queue_preserves_measurement_status_order(void)
{
  LdsResultQueue queue;
  LdsReceiverEvent first;
  LdsReceiverEvent second;
  LdsReceiverEvent third;
  LdsReceiverEvent output;

  memset(&first, 0, sizeof(first));
  memset(&second, 0, sizeof(second));
  memset(&third, 0, sizeof(third));
  first.type = LDS_RECEIVER_EVENT_MEASUREMENT;
  first.payload.measurement = make_fixed_frame(100U, 0U, 1U, 200U);
  second.type = LDS_RECEIVER_EVENT_STATUS;
  second.payload.status.raw_flags = 0x21U;
  third.type = LDS_RECEIVER_EVENT_MEASUREMENT;
  third.payload.measurement = make_fixed_frame(300U, 0U, 1U, 400U);

  LdsResultQueue_Init(&queue);
  CHECK(LdsResultQueue_Pop(&queue, &output) == 0U);
  CHECK(LdsResultQueue_Push(&queue, &first) != 0U);
  CHECK(LdsResultQueue_Push(&queue, &second) != 0U);
  CHECK(LdsResultQueue_Push(&queue, &third) == 0U);
  CHECK(LdsResultQueue_Pop(&queue, &output) != 0U);
  CHECK(output.type == LDS_RECEIVER_EVENT_MEASUREMENT);
  CHECK(output.payload.measurement.start_angle_tenths == 100U);
  CHECK(LdsResultQueue_Pop(&queue, &output) != 0U);
  CHECK(output.type == LDS_RECEIVER_EVENT_STATUS);
  CHECK(output.payload.status.raw_flags == 0x21U);
  CHECK(LdsResultQueue_Pop(&queue, &output) == 0U);
}

static void test_dma_cursor_normalizes_the_zero_counter_boundary(void)
{
  CHECK(LdsDmaCursor_WritePosition(LDS_DMA_RX_BUFFER_SIZE) == 0U);
  CHECK(LdsDmaCursor_WritePosition(1U) == 255U);
  CHECK(LdsDmaCursor_WritePosition(0U) == 0U);
}

#define CAPTURED_REPORT_LINES 36U
#define CAPTURED_REPORT_WIDTH 160U

typedef struct
{
  char lines[CAPTURED_REPORT_LINES][CAPTURED_REPORT_WIDTH];
  uint16_t count;
} CapturedReport;

static CapturedReport g_captured_report;

static void capture_report_line(const char *data,
                                uint16_t length,
                                void *context)
{
  CapturedReport *capture = (CapturedReport *)context;

  if (capture->count >= CAPTURED_REPORT_LINES ||
      length >= CAPTURED_REPORT_WIDTH)
  {
    return;
  }
  memcpy(capture->lines[capture->count], data, length);
  capture->lines[capture->count][length] = '\0';
  ++capture->count;
}

static void test_navigation_report_emits_cells_and_every_waypoint(void)
{
  NavigationResult result;

  memset(&result, 0, sizeof(result));
  memset(&g_captured_report, 0, sizeof(g_captured_report));
  result.state = NAV_PATH_READY;
  result.error = NAV_ERROR_NONE;
  result.valid_scan_points = 32U;
  result.scanned_obstacle_mask = 0x00000002UL;
  result.effective_obstacle_mask = 0x12345678UL;
  result.cell_point_counts[0] = 7U;
  result.cell_point_counts[24] = 9U;
  result.path_count = 2U;
  result.path[0].row = 1U;
  result.path[0].column = 1U;
  result.path[0].x_mm = 230U;
  result.path[0].y_mm = 230U;
  result.path[0].distance_to_next_mm = 558U;
  result.path[0].direction = NAV_DIR_EAST;
  result.path[0].turn = NAV_TURN_START;
  result.path[1].row = 1U;
  result.path[1].column = 2U;
  result.path[1].x_mm = 775U;
  result.path[1].y_mm = 350U;
  result.path[1].direction = NAV_DIR_END;
  result.path[1].turn = NAV_TURN_END;
  result.rx_bytes_received = 1234U;
  result.rx_measurement_packets = 7U;
  result.rx_status_packets = 1U;
  result.rx_bad_length_packets = 2U;
  result.rx_bad_checksum_packets = 3U;
  result.rx_discarded_bytes = 4U;
  result.rx_max_declared_point_count = 300U;
  result.rx_last_status_flags = 0x0FU;
  result.rx_distance_unit_mm = 1U;
  result.rx_raw_sample_count = 4U;
  result.rx_raw_sample[0] = 0xCEU;
  result.rx_raw_sample[1] = 0xFAU;
  result.rx_raw_sample[2] = 0x01U;
  result.rx_raw_sample[3] = 0x00U;
  result.scan_measurement_frames = 6U;
  result.scan_raw_points = 900U;
  result.scan_first_angle_tenths = 3500U;
  result.scan_last_angle_tenths = 50U;
  result.scan_min_angle_tenths = 50U;
  result.scan_max_angle_tenths = 3500U;
  result.scan_wrap_count = 2U;
  result.map_processed_points = 700U;
  result.map_angle_rejected_points = 1U;
  result.map_distance_rejected_points = 2U;
  result.map_intensity_rejected_points = 3U;
  result.map_outside_field_points = 662U;
  result.map_min_distance_mm = 50U;
  result.map_max_distance_mm = 40000U;

  NavigationReport_Emit(&result, capture_report_line, &g_captured_report);

  CHECK(g_captured_report.count == 34U);
  CHECK(strcmp(g_captured_report.lines[0],
               "NAV,state=PATH_READY,error=NONE,valid=32,scanned=0x00000002,effective=0x12345678,path_count=2\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[1],
               "RX,bytes=1234,measurement_packets=7,status_packets=1,bad_length=2,bad_checksum=3,discarded=4\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[2],
               "SCAN,frames=6,raw_points=900,first_angle=3500,last_angle=50,min_angle=50,max_angle=3500,wraps=2,max_declared_points=300\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[3],
               "RAW,hex=CEFA0100\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[4],
               "MAP,processed=700,accepted=32,angle_reject=1,distance_reject=2,intensity_reject=3,outside=662\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[5],
               "RANGE,min=50,max=40000,status_flags=0x0F,unit_mm=1\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[6],
               "CELL,index=1,row=1,column=1,count=7\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[30],
               "CELL,index=25,row=5,column=5,count=9\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[31],
               "WP,index=1,row=1,column=1,x=230,y=230,distance=558,direction=EAST,turn=START\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[32],
               "WP,index=2,row=1,column=2,x=775,y=350,distance=0,direction=END,turn=END\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[33], "END\r\n") == 0);
}

static void test_navigation_report_names_terminal_errors(void)
{
  NavigationResult result;

  memset(&result, 0, sizeof(result));
  memset(&g_captured_report, 0, sizeof(g_captured_report));
  result.state = NAV_ERROR;
  result.error = NAV_ERROR_SCAN_TIMEOUT;

  NavigationReport_Emit(&result, capture_report_line, &g_captured_report);

  CHECK(g_captured_report.count == 32U);
  CHECK(strcmp(g_captured_report.lines[0],
               "NAV,state=ERROR,error=SCAN_TIMEOUT,valid=0,scanned=0x00000000,effective=0x00000000,path_count=0\r\n") == 0);
  CHECK(strcmp(g_captured_report.lines[31], "END\r\n") == 0);
}

static void test_navigation_report_session_sends_one_terminal_report(void)
{
  NavigationResult result;
  NavigationReportSession session;

  memset(&result, 0, sizeof(result));
  memset(&g_captured_report, 0, sizeof(g_captured_report));
  NavigationReportSession_Init(&session);

  result.state = NAV_SCANNING;
  NavigationReportSession_Poll(&session,
                               &result,
                               capture_report_line,
                               &g_captured_report);
  CHECK(g_captured_report.count == 0U);
  CHECK(session.sent == 0U);

  result.state = NAV_PATH_READY;
  NavigationReportSession_Poll(&session,
                               &result,
                               capture_report_line,
                               &g_captured_report);
  CHECK(g_captured_report.count == 32U);
  CHECK(session.sent != 0U);

  NavigationReportSession_Poll(&session,
                               &result,
                               capture_report_line,
                               &g_captured_report);
  CHECK(g_captured_report.count == 32U);
}

void test_finished(void)
{
  for (;;)
  {
  }
}

int main(void)
{
  test_configuration_describes_the_required_field();
  test_polar_coordinates_follow_the_lidar_axis_convention();
  test_grid_boundaries_use_half_open_intervals();
  test_grid_centers_and_obstacle_priority_are_stable();
  test_one_point_marks_an_obstacle_but_an_empty_cell_does_not();
  test_fixed_frames_start_and_stop_on_distinct_wraps();
  test_any_start_angle_decrease_marks_a_revolution_wrap();
  test_scan_mapper_records_raw_frame_and_angle_diagnostics();
  test_variable_frames_settle_the_last_pending_frame_before_completion();
  test_scan_filter_rejects_short_and_out_of_field_points();
  test_default_mission_path_matches_the_33_cell_reference();
  test_waypoints_describe_distance_direction_and_turn();
  test_planner_reroutes_and_reports_the_failed_mission_leg();
  test_path_capacity_is_checked_before_each_write();
  test_navigation_runs_once_and_publishes_a_stable_path();
  test_navigation_timeout_handles_tick_wrap_and_preserves_root_error();
  test_navigation_accepts_a_sparse_complete_scan_and_reports_receiver_failure();
  test_navigation_keeps_failed_task_numbers_when_map_is_unreachable();
  test_receiver_event_queue_preserves_measurement_status_order();
  test_dma_cursor_normalizes_the_zero_counter_boundary();
  test_navigation_report_emits_cells_and_every_waypoint();
  test_navigation_report_names_terminal_errors();
  test_navigation_report_session_sends_one_terminal_report();
  test_finished();
  return 0;
}
