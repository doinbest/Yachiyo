#include "lds_parser.h"

#include <string.h>

#define LDS_HEADER_VARIABLE_0 0xCEU
#define LDS_HEADER_FIXED_0    0xCFU
#define LDS_HEADER_DATA_1     0xFAU
#define LDS_HEADER_STATUS_0   0x53U
#define LDS_HEADER_STATUS_1   0x54U
#define LDS_STATUS_END_0      0x45U
#define LDS_STATUS_END_1      0x44U
#define LDS_STATUS_FRAME_SIZE 8U

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint8_t is_start_byte(uint8_t value)
{
  return (value == LDS_HEADER_VARIABLE_0 ||
          value == LDS_HEADER_FIXED_0 ||
          value == LDS_HEADER_STATUS_0) ? 1U : 0U;
}

static void reset_frame(LdsParser *parser)
{
  parser->frame_size = 0U;
  parser->expected_size = 0U;
}

static void begin_or_discard(LdsParser *parser, uint8_t value)
{
  if (is_start_byte(value) != 0U)
  {
    parser->frame[0] = value;
    parser->frame_size = 1U;
  }
  else
  {
    parser->stats.discarded_bytes++;
  }
}

static uint16_t calculate_measurement_checksum(const uint8_t *frame,
                                               uint16_t point_count,
                                               uint8_t fixed_resolution)
{
  uint32_t sum;
  uint16_t index;
  uint16_t point_index;

  sum = point_count;
  sum += read_u16_le(&frame[4]);
  index = 6U;

  if (fixed_resolution != 0U)
  {
    sum += read_u16_le(&frame[index]);
    index += 2U;
  }

  for (point_index = 0U; point_index < point_count; ++point_index)
  {
    sum += frame[index];
    sum += read_u16_le(&frame[index + 1U]);
    index += 3U;
  }

  return (uint16_t)sum;
}

static void publish_measurement(LdsParser *parser)
{
  uint8_t fixed_resolution;
  uint16_t checksum_offset;
  uint16_t received_checksum;
  uint16_t calculated_checksum;
  uint16_t point_count;
  uint16_t point_index;
  uint16_t index;

  fixed_resolution = (parser->frame[0] == LDS_HEADER_FIXED_0) ? 1U : 0U;
  point_count = read_u16_le(&parser->frame[2]);
  checksum_offset = (uint16_t)(parser->expected_size - 2U);
  received_checksum = read_u16_le(&parser->frame[checksum_offset]);
  calculated_checksum = calculate_measurement_checksum(parser->frame,
                                                       point_count,
                                                       fixed_resolution);

  if (received_checksum != calculated_checksum)
  {
    parser->stats.bad_checksum_packets++;
    return;
  }

  parser->latest_result.packet_type = (fixed_resolution != 0U)
                                          ? LDS_PACKET_FIXED_RESOLUTION
                                          : LDS_PACKET_VARIABLE_RESOLUTION;
  parser->latest_result.point_count = point_count;
  parser->latest_result.start_angle_tenths = read_u16_le(&parser->frame[4]);
  index = 6U;

  if (fixed_resolution != 0U)
  {
    parser->latest_result.sector_angle_tenths = read_u16_le(&parser->frame[index]);
    index += 2U;
  }
  else
  {
    parser->latest_result.sector_angle_tenths = 0U;
  }

  for (point_index = 0U; point_index < point_count; ++point_index)
  {
    parser->latest_result.points[point_index].intensity = parser->frame[index];
    parser->latest_result.points[point_index].distance_mm =
        read_u16_le(&parser->frame[index + 1U]);
    index += 3U;
  }

  parser->stats.good_measurement_packets++;
  parser->result_ready = 1U;
}

static void publish_status(LdsParser *parser)
{
  uint8_t flags;

  if (parser->frame[6] != LDS_STATUS_END_0 ||
      parser->frame[7] != LDS_STATUS_END_1)
  {
    parser->stats.malformed_status_packets++;
    return;
  }

  flags = parser->frame[2];
  parser->stats.last_status_flags = flags;
  parser->latest_status.raw_flags = flags;
  parser->latest_status.distance_unit_mm = flags & 0x01U;
  parser->latest_status.intensity_enabled = (flags >> 1) & 0x01U;
  parser->latest_status.deshadow_enabled = (flags >> 2) & 0x01U;
  parser->latest_status.filter_enabled = (flags >> 3) & 0x01U;
  parser->stats.good_status_packets++;
  parser->status_ready = 1U;
}

static void feed_byte(LdsParser *parser, uint8_t value)
{
  uint8_t first_byte;
  uint8_t second_byte_valid;
  uint16_t point_count;
  uint16_t overhead;

  if (parser->stats.raw_sample_count < LDS_RAW_SAMPLE_CAPACITY)
  {
    parser->stats.raw_sample[parser->stats.raw_sample_count++] = value;
  }
  parser->stats.bytes_received++;

  if (parser->frame_size == 0U)
  {
    begin_or_discard(parser, value);
    return;
  }

  first_byte = parser->frame[0];

  if (parser->frame_size == 1U)
  {
    second_byte_valid = 0U;
    if ((first_byte == LDS_HEADER_VARIABLE_0 || first_byte == LDS_HEADER_FIXED_0) &&
        value == LDS_HEADER_DATA_1)
    {
      second_byte_valid = 1U;
    }
    else if (first_byte == LDS_HEADER_STATUS_0 && value == LDS_HEADER_STATUS_1)
    {
      second_byte_valid = 1U;
      parser->expected_size = LDS_STATUS_FRAME_SIZE;
    }

    if (second_byte_valid == 0U)
    {
      parser->stats.discarded_bytes++;
      reset_frame(parser);
      begin_or_discard(parser, value);
      return;
    }
  }

  if (parser->frame_size >= (uint16_t)sizeof(parser->frame))
  {
    parser->stats.bad_length_packets++;
    reset_frame(parser);
    begin_or_discard(parser, value);
    return;
  }

  parser->frame[parser->frame_size++] = value;

  if ((first_byte == LDS_HEADER_VARIABLE_0 || first_byte == LDS_HEADER_FIXED_0) &&
      parser->frame_size == 4U)
  {
    point_count = read_u16_le(&parser->frame[2]);
    if (point_count > parser->stats.max_declared_point_count)
    {
      parser->stats.max_declared_point_count = point_count;
    }
    if (point_count == 0U || point_count > LDS_MAX_POINTS_PER_PACKET)
    {
      parser->stats.bad_length_packets++;
      reset_frame(parser);
      return;
    }

    overhead = (first_byte == LDS_HEADER_FIXED_0) ? 10U : 8U;
    parser->expected_size = (uint16_t)(overhead + (3U * point_count));
  }

  if (parser->expected_size != 0U && parser->frame_size == parser->expected_size)
  {
    if (first_byte == LDS_HEADER_STATUS_0)
    {
      publish_status(parser);
    }
    else
    {
      publish_measurement(parser);
    }
    reset_frame(parser);
  }
}

void LdsParser_Init(LdsParser *parser)
{
  if (parser != 0)
  {
    memset(parser, 0, sizeof(*parser));
  }
}

void LdsParser_Feed(LdsParser *parser, const uint8_t *data, uint16_t length)
{
  uint16_t index;

  if (parser == 0 || data == 0)
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    feed_byte(parser, data[index]);
  }
}

uint8_t LdsParser_ReadResult(LdsParser *parser, LdsParserResult *result)
{
  if (parser == 0 || result == 0 || parser->result_ready == 0U)
  {
    return 0U;
  }

  memcpy(result, &parser->latest_result, sizeof(*result));
  parser->result_ready = 0U;
  return 1U;
}

uint8_t LdsParser_ReadStatus(LdsParser *parser, LdsParserStatus *status)
{
  if (parser == 0 || status == 0 || parser->status_ready == 0U)
  {
    return 0U;
  }

  memcpy(status, &parser->latest_status, sizeof(*status));
  parser->status_ready = 0U;
  return 1U;
}
