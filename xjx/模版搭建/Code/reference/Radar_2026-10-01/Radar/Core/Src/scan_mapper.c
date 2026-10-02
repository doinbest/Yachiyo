#include "scan_mapper.h"

#include "grid_map.h"
#include "nav_math.h"

static uint32_t saturating_add_u32(uint32_t left, uint32_t right)
{
  if (left > (0xFFFFFFFFUL - right))
  {
    return 0xFFFFFFFFUL;
  }
  return left + right;
}

static void record_frame_diagnostics(ScanMapper *mapper,
                                     const LdsParserResult *frame,
                                     uint16_t current_start)
{
  mapper->measurement_frame_count =
      saturating_add_u32(mapper->measurement_frame_count, 1U);
  mapper->raw_point_count =
      saturating_add_u32(mapper->raw_point_count, frame->point_count);
  mapper->last_start_angle_tenths = current_start;

  if (mapper->diagnostic_angle_seen == 0U)
  {
    mapper->diagnostic_angle_seen = 1U;
    mapper->first_start_angle_tenths = current_start;
    mapper->min_start_angle_tenths = current_start;
    mapper->max_start_angle_tenths = current_start;
    return;
  }

  if (current_start < mapper->min_start_angle_tenths)
  {
    mapper->min_start_angle_tenths = current_start;
  }
  if (current_start > mapper->max_start_angle_tenths)
  {
    mapper->max_start_angle_tenths = current_start;
  }
}

static void clear_counts(ScanMapper *mapper)
{
  uint8_t index;

  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    mapper->cell_point_counts[index] = 0U;
  }
  mapper->valid_scan_points = 0U;
  mapper->processed_point_count = 0U;
  mapper->angle_rejected_point_count = 0U;
  mapper->distance_rejected_point_count = 0U;
  mapper->intensity_rejected_point_count = 0U;
  mapper->outside_field_point_count = 0U;
  mapper->min_distance_mm = 0U;
  mapper->max_distance_mm = 0U;
  mapper->diagnostic_distance_seen = 0U;
}

static uint8_t is_wrap(uint16_t previous, uint16_t current)
{
  return previous > current ? 1U : 0U;
}

static uint8_t angle_is_allowed(uint16_t angle)
{
#if NAV_FILTER_MIN_ANGLE_TENTHS == 0U && NAV_FILTER_MAX_ANGLE_TENTHS >= 3599U
  (void)angle;
  return 1U;
#elif NAV_FILTER_MIN_ANGLE_TENTHS <= NAV_FILTER_MAX_ANGLE_TENTHS
  return angle >= NAV_FILTER_MIN_ANGLE_TENTHS &&
                 angle <= NAV_FILTER_MAX_ANGLE_TENTHS
             ? 1U
             : 0U;
#else
  return angle >= NAV_FILTER_MIN_ANGLE_TENTHS ||
                 angle <= NAV_FILTER_MAX_ANGLE_TENTHS
             ? 1U
             : 0U;
#endif
}

static void accept_point(ScanMapper *mapper,
                         uint16_t angle_tenths,
                         const LdsPoint *point)
{
  int32_t x_mm;
  int32_t y_mm;
  uint8_t row;
  uint8_t column;
  uint8_t index;

  mapper->processed_point_count =
      saturating_add_u32(mapper->processed_point_count, 1U);
  if (mapper->diagnostic_distance_seen == 0U)
  {
    mapper->diagnostic_distance_seen = 1U;
    mapper->min_distance_mm = point->distance_mm;
    mapper->max_distance_mm = point->distance_mm;
  }
  else
  {
    if (point->distance_mm < mapper->min_distance_mm)
    {
      mapper->min_distance_mm = point->distance_mm;
    }
    if (point->distance_mm > mapper->max_distance_mm)
    {
      mapper->max_distance_mm = point->distance_mm;
    }
  }

  angle_tenths = (uint16_t)(angle_tenths % 3600U);
  if (angle_is_allowed(angle_tenths) == 0U)
  {
    mapper->angle_rejected_point_count =
        saturating_add_u32(mapper->angle_rejected_point_count, 1U);
    return;
  }
  if (point->distance_mm < NAV_FILTER_MIN_DISTANCE_MM ||
      point->distance_mm > NAV_FILTER_MAX_DISTANCE_MM)
  {
    mapper->distance_rejected_point_count =
        saturating_add_u32(mapper->distance_rejected_point_count, 1U);
    return;
  }
#if NAV_FILTER_MIN_INTENSITY > 0U
  if (point->intensity < NAV_FILTER_MIN_INTENSITY)
  {
    mapper->intensity_rejected_point_count =
        saturating_add_u32(mapper->intensity_rejected_point_count, 1U);
    return;
  }
#endif
#if NAV_FILTER_MAX_INTENSITY < 255U
  if (point->intensity > NAV_FILTER_MAX_INTENSITY)
  {
    mapper->intensity_rejected_point_count =
        saturating_add_u32(mapper->intensity_rejected_point_count, 1U);
    return;
  }
#endif

  NavMath_PolarToCartesian(angle_tenths, point->distance_mm, &x_mm, &y_mm);
  if (GridMap_FindCell(x_mm, y_mm, &row, &column) == 0U)
  {
    mapper->outside_field_point_count =
        saturating_add_u32(mapper->outside_field_point_count, 1U);
    return;
  }

  index = GridMap_CellIndex(row, column);
  if (mapper->cell_point_counts[index] != 0xFFFFU)
  {
    ++mapper->cell_point_counts[index];
  }
  if (mapper->valid_scan_points != 0xFFFFU)
  {
    ++mapper->valid_scan_points;
  }
}

static void process_fixed_frame(ScanMapper *mapper,
                                const LdsParserResult *frame)
{
  uint16_t index;
  uint16_t count = frame->point_count;
  uint16_t denominator = count > 1U ? (uint16_t)(count - 1U) : 1U;

  if (count > LDS_MAX_POINTS_PER_PACKET)
  {
    count = LDS_MAX_POINTS_PER_PACKET;
  }

  for (index = 0U; index < count; ++index)
  {
    uint32_t offset = count > 1U
                          ? ((uint32_t)frame->sector_angle_tenths * index) /
                                denominator
                          : 0U;
    uint16_t angle = (uint16_t)(((uint32_t)frame->start_angle_tenths +
                                 offset) %
                                3600U);
    accept_point(mapper, angle, &frame->points[index]);
  }
}

static void settle_pending_variable(ScanMapper *mapper,
                                    uint16_t next_start_angle_tenths)
{
  LdsParserResult *frame;
  uint16_t count;
  uint16_t index;
  uint16_t span;

  if (mapper->pending_valid == 0U)
  {
    return;
  }

  frame = &mapper->pending_variable_frame;
  count = frame->point_count;
  if (count > LDS_MAX_POINTS_PER_PACKET)
  {
    count = LDS_MAX_POINTS_PER_PACKET;
  }
  span = (uint16_t)(((uint32_t)next_start_angle_tenths + 3600U -
                     frame->start_angle_tenths) %
                    3600U);

  if (mapper->state == SCAN_COLLECTING && count != 0U)
  {
    for (index = 0U; index < count; ++index)
    {
      uint32_t offset = ((uint32_t)span * index) / count;
      uint16_t angle = (uint16_t)(((uint32_t)frame->start_angle_tenths +
                                   offset) %
                                  3600U);
      accept_point(mapper, angle, &frame->points[index]);
    }
  }
  mapper->pending_valid = 0U;
}

void ScanMapper_Init(ScanMapper *mapper)
{
  mapper->state = SCAN_WAIT_FIRST_WRAP;
  mapper->previous_start_angle_tenths = 0U;
  mapper->have_previous_start = 0U;
  mapper->pending_valid = 0U;
  mapper->measurement_frame_count = 0U;
  mapper->raw_point_count = 0U;
  mapper->first_start_angle_tenths = 0U;
  mapper->last_start_angle_tenths = 0U;
  mapper->min_start_angle_tenths = 0U;
  mapper->max_start_angle_tenths = 0U;
  mapper->wrap_count = 0U;
  mapper->diagnostic_angle_seen = 0U;
  clear_counts(mapper);
}

void ScanMapper_PushFrame(ScanMapper *mapper, const LdsParserResult *frame)
{
  uint16_t current_start;
  uint8_t wrapped;

  if (mapper == 0 || frame == 0 || mapper->state == SCAN_COMPLETE ||
      (frame->packet_type != LDS_PACKET_FIXED_RESOLUTION &&
       frame->packet_type != LDS_PACKET_VARIABLE_RESOLUTION))
  {
    return;
  }

  current_start = (uint16_t)(frame->start_angle_tenths % 3600U);
  record_frame_diagnostics(mapper, frame, current_start);
  wrapped = mapper->have_previous_start != 0U
                ? is_wrap(mapper->previous_start_angle_tenths, current_start)
                : 0U;

  settle_pending_variable(mapper, current_start);

  if (wrapped != 0U)
  {
    if (mapper->wrap_count != 0xFFFFU)
    {
      ++mapper->wrap_count;
    }
    if (mapper->state == SCAN_WAIT_FIRST_WRAP)
    {
      clear_counts(mapper);
      mapper->state = SCAN_COLLECTING;
    }
    else
    {
      mapper->state = SCAN_COMPLETE;
      mapper->previous_start_angle_tenths = current_start;
      mapper->have_previous_start = 1U;
      return;
    }
  }

  if (mapper->state == SCAN_COLLECTING)
  {
    if (frame->packet_type == LDS_PACKET_VARIABLE_RESOLUTION)
    {
      mapper->pending_variable_frame = *frame;
      mapper->pending_variable_frame.start_angle_tenths = current_start;
      mapper->pending_valid = 1U;
    }
    else
    {
      process_fixed_frame(mapper, frame);
    }
  }

  mapper->previous_start_angle_tenths = current_start;
  mapper->have_previous_start = 1U;
}

uint8_t ScanMapper_IsComplete(const ScanMapper *mapper)
{
  return mapper != 0 && mapper->state == SCAN_COMPLETE ? 1U : 0U;
}
