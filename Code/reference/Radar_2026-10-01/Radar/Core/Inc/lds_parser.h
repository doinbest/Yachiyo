#ifndef LDS_PARSER_H
#define LDS_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LDS_MAX_POINTS_PER_PACKET 256U
#define LDS_RAW_SAMPLE_CAPACITY 64U
#define LDS_MAX_MEASUREMENT_FRAME_SIZE \
  (2U + 2U + 2U + 2U + (3U * LDS_MAX_POINTS_PER_PACKET) + 2U)

typedef enum
{
  LDS_PACKET_NONE = 0,
  LDS_PACKET_VARIABLE_RESOLUTION,
  LDS_PACKET_FIXED_RESOLUTION
} LdsPacketType;

typedef struct
{
  uint16_t distance_mm;
  uint8_t intensity;
} LdsPoint;

typedef struct
{
  LdsPacketType packet_type;
  uint16_t point_count;
  uint16_t start_angle_tenths;
  uint16_t sector_angle_tenths;
  LdsPoint points[LDS_MAX_POINTS_PER_PACKET];
} LdsParserResult;

typedef struct
{
  uint8_t raw_flags;
  uint8_t distance_unit_mm;
  uint8_t intensity_enabled;
  uint8_t deshadow_enabled;
  uint8_t filter_enabled;
} LdsParserStatus;

typedef struct
{
  uint32_t bytes_received;
  uint32_t good_measurement_packets;
  uint32_t good_status_packets;
  uint32_t bad_checksum_packets;
  uint32_t bad_length_packets;
  uint32_t malformed_status_packets;
  uint32_t discarded_bytes;
  uint16_t max_declared_point_count;
  uint8_t last_status_flags;
  uint8_t raw_sample_count;
  uint8_t raw_sample[LDS_RAW_SAMPLE_CAPACITY];
} LdsParserStats;

typedef struct
{
  LdsParserStats stats;
  LdsParserResult latest_result;
  LdsParserStatus latest_status;
  uint8_t frame[LDS_MAX_MEASUREMENT_FRAME_SIZE];
  uint16_t frame_size;
  uint16_t expected_size;
  uint8_t result_ready;
  uint8_t status_ready;
} LdsParser;

void LdsParser_Init(LdsParser *parser);
void LdsParser_Feed(LdsParser *parser, const uint8_t *data, uint16_t length);
uint8_t LdsParser_ReadResult(LdsParser *parser, LdsParserResult *result);
uint8_t LdsParser_ReadStatus(LdsParser *parser, LdsParserStatus *status);

#ifdef __cplusplus
}
#endif

#endif
