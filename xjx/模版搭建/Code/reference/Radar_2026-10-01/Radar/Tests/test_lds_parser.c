#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lds_parser.h"
#include "lds_startup_commands.h"

#ifdef ARM_SIM_TEST
volatile uint32_t g_test_checks;
volatile uint32_t g_test_failures;
volatile uint32_t g_test_failure_lines[16];

#define CHECK(condition)                                      \
  do                                                          \
  {                                                           \
    ++g_test_checks;                                          \
    if (!(condition))                                         \
    {                                                         \
      if (g_test_failures < 16U)                              \
      {                                                       \
        g_test_failure_lines[g_test_failures] = __LINE__;     \
      }                                                       \
      ++g_test_failures;                                      \
    }                                                         \
  } while (0)
#else
static int failures;

#define CHECK(condition)                                                     \
  do                                                                         \
  {                                                                          \
    if (!(condition))                                                        \
    {                                                                        \
      ++failures;                                                            \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);          \
    }                                                                        \
  } while (0)
#endif

static uint16_t packet_checksum(const uint8_t *frame, uint16_t length_without_checksum)
{
  uint32_t sum;
  uint16_t index;
  uint16_t point_count;
  uint8_t fixed_resolution;

  fixed_resolution = (frame[0] == 0xCFU) ? 1U : 0U;
  point_count = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8);
  sum = point_count;
  sum += (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
  index = 6U;

  if (fixed_resolution != 0U)
  {
    sum += (uint16_t)frame[index] | ((uint16_t)frame[index + 1U] << 8);
    index += 2U;
  }

  while ((uint16_t)(index + 2U) < length_without_checksum)
  {
    sum += frame[index];
    sum += (uint16_t)frame[index + 1U] | ((uint16_t)frame[index + 2U] << 8);
    index += 3U;
  }

  return (uint16_t)sum;
}

static void append_checksum(uint8_t *frame, uint16_t length)
{
  uint16_t checksum = packet_checksum(frame, (uint16_t)(length - 2U));
  frame[length - 2U] = (uint8_t)(checksum & 0xFFU);
  frame[length - 1U] = (uint8_t)(checksum >> 8);
}

static void test_parses_fixed_resolution_measurement(void)
{
  LdsParser parser;
  LdsParserResult result;
  uint8_t frame[] = {
      0xCF, 0xFA,
      0x02, 0x00,
      0x64, 0x00,
      0x20, 0x03,
      0x21, 0x9C, 0x00,
      0x18, 0xF4, 0x01,
      0x00, 0x00};

  append_checksum(frame, (uint16_t)sizeof(frame));
  LdsParser_Init(&parser);
  LdsParser_Feed(&parser, frame, 5U);
  LdsParser_Feed(&parser, &frame[5], (uint16_t)(sizeof(frame) - 5U));

  CHECK(LdsParser_ReadResult(&parser, &result));
  CHECK(result.packet_type == LDS_PACKET_FIXED_RESOLUTION);
  CHECK(result.point_count == 2U);
  CHECK(result.start_angle_tenths == 100U);
  CHECK(result.sector_angle_tenths == 800U);
  CHECK(result.points[0].intensity == 33U);
  CHECK(result.points[0].distance_mm == 156U);
  CHECK(result.points[1].intensity == 24U);
  CHECK(result.points[1].distance_mm == 500U);
  CHECK(parser.stats.good_measurement_packets == 1U);
  CHECK(parser.stats.bad_checksum_packets == 0U);
}

static void test_rejects_bad_checksum_and_resynchronizes(void)
{
  LdsParser parser;
  LdsParserResult result;
  uint8_t bad_frame[] = {
      0xCE, 0xFA,
      0x01, 0x00,
      0x00, 0x00,
      0x10, 0xE8, 0x03,
      0x00, 0x00};
  uint8_t good_frame[] = {
      0xCE, 0xFA,
      0x01, 0x00,
      0x2C, 0x01,
      0x2A, 0xD0, 0x07,
      0x00, 0x00};

  append_checksum(bad_frame, (uint16_t)sizeof(bad_frame));
  bad_frame[sizeof(bad_frame) - 1U] ^= 0x80U;
  append_checksum(good_frame, (uint16_t)sizeof(good_frame));

  LdsParser_Init(&parser);
  LdsParser_Feed(&parser, bad_frame, (uint16_t)sizeof(bad_frame));
  CHECK(!LdsParser_ReadResult(&parser, &result));
  CHECK(parser.stats.bad_checksum_packets == 1U);

  LdsParser_Feed(&parser, good_frame, (uint16_t)sizeof(good_frame));
  CHECK(LdsParser_ReadResult(&parser, &result));
  CHECK(result.packet_type == LDS_PACKET_VARIABLE_RESOLUTION);
  CHECK(result.start_angle_tenths == 300U);
  CHECK(result.points[0].distance_mm == 2000U);
  CHECK(result.points[0].intensity == 42U);
}

static void test_parses_status_packet_after_noise(void)
{
  LdsParser parser;
  LdsParserStatus status;
  const uint8_t stream[] = {
      0x00, 0xCE, 0x00, 0xFA,
      0x53, 0x54, 0x0F, 0x00, 0x00, 0x00, 0x45, 0x44};

  LdsParser_Init(&parser);
  LdsParser_Feed(&parser, stream, (uint16_t)sizeof(stream));

  CHECK(LdsParser_ReadStatus(&parser, &status));
  CHECK(status.distance_unit_mm == 1U);
  CHECK(status.intensity_enabled == 1U);
  CHECK(status.deshadow_enabled == 1U);
  CHECK(status.filter_enabled == 1U);
  CHECK(parser.stats.good_status_packets == 1U);
  CHECK(parser.stats.last_status_flags == 0x0FU);
  CHECK(parser.stats.discarded_bytes >= 4U);
}

static void test_rejects_point_count_beyond_capacity(void)
{
  LdsParser parser;
  const uint16_t declared_point_count = LDS_MAX_POINTS_PER_PACKET + 44U;
  const uint8_t malformed_header[] = {
      0xCE, 0xFA,
      (uint8_t)(declared_point_count & 0xFFU),
      (uint8_t)(declared_point_count >> 8)};

  LdsParser_Init(&parser);
  LdsParser_Feed(&parser, malformed_header, (uint16_t)sizeof(malformed_header));

  CHECK(parser.stats.bad_length_packets == 1U);
  CHECK(parser.stats.max_declared_point_count == declared_point_count);
}

static void test_startup_commands_match_the_working_upper_host(void)
{
  static const char *const expected[] = {
      "LMDMMH", "LOCONH", "LFFF1H", "LSSS1H", "LSTARH"};
  uint8_t index;

  CHECK(LdsStartupCommand_Count() == 5U);
  for (index = 0U; index < LdsStartupCommand_Count(); ++index)
  {
    CHECK(strcmp(LdsStartupCommand_Get(index), expected[index]) == 0);
  }
  CHECK(LdsStartupCommand_Get(LdsStartupCommand_Count()) == 0);
}

static void test_captures_the_first_raw_bytes_for_hardware_diagnosis(void)
{
  LdsParser parser;
  uint8_t stream[LDS_RAW_SAMPLE_CAPACITY + 2U];
  uint16_t index;

  for (index = 0U; index < (uint16_t)sizeof(stream); ++index)
  {
    stream[index] = (uint8_t)index;
  }

  LdsParser_Init(&parser);
  LdsParser_Feed(&parser, stream, (uint16_t)sizeof(stream));

  CHECK(parser.stats.raw_sample_count == LDS_RAW_SAMPLE_CAPACITY);
  CHECK(parser.stats.raw_sample[0] == 0U);
  CHECK(parser.stats.raw_sample[LDS_RAW_SAMPLE_CAPACITY - 1U] ==
        LDS_RAW_SAMPLE_CAPACITY - 1U);
}

#ifdef ARM_SIM_TEST
void test_finished(void);
#endif

int main(void)
{
  test_parses_fixed_resolution_measurement();
  test_rejects_bad_checksum_and_resynchronizes();
  test_parses_status_packet_after_noise();
  test_rejects_point_count_beyond_capacity();
  test_startup_commands_match_the_working_upper_host();
  test_captures_the_first_raw_bytes_for_hardware_diagnosis();

#ifdef ARM_SIM_TEST
  test_finished();
  return 0;
#else
  if (failures != 0)
  {
    printf("%d test(s) failed\n", failures);
    return 1;
  }

  printf("all lds parser tests passed\n");
  return 0;
#endif
}

#ifdef ARM_SIM_TEST
void test_finished(void)
{
  for (;;)
  {
  }
}
#endif
