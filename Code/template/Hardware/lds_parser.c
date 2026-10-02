#include "lds_parser.h"
#include <string.h>

static uint16_t word(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static void consume(LdsParser_t *p, uint16_t count)
{
  p->used = (uint16_t)(p->used - count);
  if (p->used) memmove(p->frame, p->frame + count, p->used);
}

void LdsParser_Init(LdsParser_t *p)
{ if (p) memset(p, 0, sizeof(*p)); }

void LdsParser_Reset(LdsParser_t *p)
{ if (p) { p->used = 0; p->event.kind = LDS_EVENT_NONE; } }

bool LdsParser_Poll(LdsParser_t *p)
{
  uint16_t count, length, at, i, start, sector;
  uint32_t sum;
  bool fixed;
  if (!p) return false;
  while (p->used >= 2U) {
    if (p->frame[0] == 0x53U && p->frame[1] == 0x54U) {
      if (p->used < 8U) return false;
      if (p->frame[6] != 0x45U || p->frame[7] != 0x44U) {
        p->stats.bad_length++; consume(p, 1); continue;
      }
      p->event.kind = LDS_EVENT_STATUS;
      p->event.data.status_flags = p->frame[2];
      p->stats.status_packets++; consume(p, 8); return true;
    }
    if (p->frame[0] == 0xCEU && p->frame[1] == 0xCEU) {
      if (p->used < 4U) return false;
      if (p->frame[2] != 0xCEU || p->frame[3] != 0xCEU) {
        p->stats.discarded++; consume(p, 1); continue;
      }
      if (p->used < 6U) return false;
      p->event.kind = LDS_EVENT_ALARM;
      p->event.data.alarm_code = word(p->frame + 4);
      p->stats.alarms++; p->stats.last_alarm = p->event.data.alarm_code;
      consume(p, 6); return true;
    }
    if ((p->frame[0] != 0xCEU && p->frame[0] != 0xCFU) || p->frame[1] != 0xFAU) {
      p->stats.discarded++; consume(p, 1); continue;
    }
    if (p->used < 4U) return false;
    fixed = p->frame[0] == 0xCFU;
    count = word(p->frame + 2);
    if (!count || count > LDS_MAX_POINTS_PER_PACKET) {
      p->stats.bad_length++; consume(p, 1); continue;
    }
    length = (uint16_t)((fixed ? 10U : 8U) + 3U * count);
    if (p->used < length) return false;
    start = word(p->frame + 4);
    sector = fixed ? word(p->frame + 6) : 0U;
    if (start >= 3600U || sector > 3600U) {
      p->stats.bad_length++; consume(p, 1); continue;
    }
    at = fixed ? 8U : 6U;
    sum = count + start + sector;
    for (i = 0; i < count; i++, at += 3U)
      sum += p->frame[at] + word(p->frame + at + 1U);
    if ((uint16_t)sum != word(p->frame + length - 2U)) {
      p->stats.bad_checksum++; consume(p, 1); continue;
    }
    p->event.kind = LDS_EVENT_MEASUREMENT;
    p->event.data.packet.fixed = fixed;
    p->event.data.packet.count = count;
    p->event.data.packet.start_angle_tenths = start;
    p->event.data.packet.sector_angle_tenths = sector;
    at = fixed ? 8U : 6U;
    for (i = 0; i < count; i++, at += 3U) {
      p->event.data.packet.points[i].energy = p->frame[at];
      p->event.data.packet.points[i].distance = word(p->frame + at + 1U);
    }
    p->stats.packets++; consume(p, length); return true;
  }
  return false;
}

bool LdsParser_FeedByte(LdsParser_t *p, uint8_t value)
{
  if (!p) return false;
  if (p->used == LDS_MAX_FRAME_BYTES) {
    p->stats.bad_length++; consume(p, 1);
  }
  p->frame[p->used++] = value;
  p->stats.bytes++;
  return LdsParser_Poll(p);
}

const LdsEvent_t *LdsParser_Event(const LdsParser_t *p)
{ return p ? &p->event : 0; }
