/** @file lds_parser.h
 * @brief LDS50C byte-stream parser, independent of STM32 HAL.
 * Protocol layout follows the user's F1 / Lds50cHost reference projects.
 */
#ifndef LDS_PARSER_H
#define LDS_PARSER_H
#include <stdbool.h>
#include <stdint.h>

#define LDS_MAX_POINTS_PER_PACKET 256U
#define LDS_MAX_FRAME_BYTES (10U + 3U * LDS_MAX_POINTS_PER_PACKET)

typedef struct { uint16_t distance; uint8_t energy; } LdsPoint_t;
typedef struct {
  bool fixed;
  uint16_t count, start_angle_tenths, sector_angle_tenths;
  LdsPoint_t points[LDS_MAX_POINTS_PER_PACKET];
} LdsPacket_t;
typedef enum { LDS_EVENT_NONE, LDS_EVENT_MEASUREMENT, LDS_EVENT_STATUS, LDS_EVENT_ALARM } LdsEventKind_t;
typedef struct {
  LdsEventKind_t kind;
  union { LdsPacket_t packet; uint8_t status_flags; uint16_t alarm_code; } data;
} LdsEvent_t;
typedef struct {
  uint32_t bytes, packets, status_packets, bad_checksum, bad_length, discarded, alarms;
  uint16_t last_alarm;
} LdsParser_Stats_t;
typedef struct {
  uint8_t frame[LDS_MAX_FRAME_BYTES];
  uint16_t used;
  LdsEvent_t event;
  LdsParser_Stats_t stats;
} LdsParser_t;

void LdsParser_Init(LdsParser_t *parser);
/** @brief Discard an incomplete frame while retaining lifetime diagnostics. */
void LdsParser_Reset(LdsParser_t *parser);
/** @brief Feed one byte; true publishes an event until the next parser call. */
bool LdsParser_FeedByte(LdsParser_t *parser, uint8_t value);
/** @brief Drain a complete event retained after checksum resynchronization. */
bool LdsParser_Poll(LdsParser_t *parser);
const LdsEvent_t *LdsParser_Event(const LdsParser_t *parser);
#endif
