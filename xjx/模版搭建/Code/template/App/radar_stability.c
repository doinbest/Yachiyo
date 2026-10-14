#include "radar_stability.h"
#include <string.h>

/* Candidate replacement, peak ordering and fixed-neighborhood suppression
   reproduce the supplied Caaaar/Radar Core/Src/scan_mapper.c. The per-circle
   transaction and rolling representatives are F407 additions. */
#define DRAFT_MASK 0x8000U
#define HISTORY_MASK 0x7FFFU
#define NO_SLOT RADAR_STABLE_CAPACITY
#define FINAL_SELECT 1U
#define FINAL_SUPPORT 2U
#define FINAL_CONSUME 3U
#define FINAL_DONE 4U

static uint32_t add_u32(uint32_t a, uint32_t b)
{
    return a > UINT32_MAX - b ? UINT32_MAX : a + b;
}
static uint8_t votes(uint16_t mask)
{
    /* Parallel 16-bit population count keeps 15-circle peak comparisons
       constant-cost on Cortex-M4 as well as the host compiler. */
    uint32_t n=mask;
    n=n-((n>>1U)&0x5555U);
    n=(n&0x3333U)+((n>>2U)&0x3333U);
    n=(n+(n>>4U))&0x0F0FU;
    return (uint8_t)((n+(n>>8U))&0x1FU);
}
static uint8_t bit_get(const uint8_t *bits, uint16_t i)
{
    return (uint8_t)((bits[i / 8U] >> (i % 8U)) & 1U);
}
static void bit_set(uint8_t *bits, uint16_t i)
{
    bits[i / 8U] |= (uint8_t)(1U << (i % 8U));
}
static uint32_t center_error(uint16_t x, uint16_t y)
{
    int32_t dx = (int32_t)(x % RADAR_STABLE_BIN_MM) - 5;
    int32_t dy = (int32_t)(y % RADAR_STABLE_BIN_MM) - 5;
    return (uint32_t)(dx * dx + dy * dy);
}
static RadarStableRepresentative representative(uint16_t a, uint16_t d,
                                                uint16_t x, uint16_t y)
{
    RadarStableRepresentative p;
    p.angle_tenths = (uint16_t)(a % 3600U); p.distance_mm = d;
    p.x_mm = x; p.y_mm = y;
    return p;
}
static void record_stats(RadarStableCircleStats *s, uint32_t raw,
                         uint32_t filtered, uint32_t outside)
{
    s->raw_points = add_u32(s->raw_points, raw);
    s->filtered_points = add_u32(s->filtered_points, filtered);
    s->outside_points = add_u32(s->outside_points, outside);
}
static uint16_t effective_mask(const RadarStableContext *c, uint16_t mask)
{
    if (c->status.window_circles == RADAR_STABLE_CIRCLES)
        mask &= (uint16_t)~(uint16_t)(1U << c->next_slot);
    return mask;
}
static void save_replacement(RadarStableContext *c, uint16_t i)
{
    RadarStableUndo *u = &c->undo[i];
    RadarStableCandidate *p = &c->candidates[i];
    uint8_t j, k = 0U;
    u->bin_x = p->bin_x; u->bin_y = p->bin_y; u->circle_mask = p->circle_mask;
    /* An evictable bin has at most two valid votes plus the outgoing vote.
       Draft writes never modify committed representatives. */
    for (j = 0U; j < RADAR_STABLE_CIRCLES; ++j)
        if ((p->circle_mask & (uint16_t)(1U << j)) && k < 3U)
            u->representatives[k++] = p->representatives[j];
    bit_set(c->replaced, i);
}
void RadarStable_Init(RadarStableContext *c, uint8_t rolling)
{
    if (!c) return;
    memset(c, 0, sizeof(*c)); c->status.rolling = rolling ? 1U : 0U;
}
uint8_t RadarStable_BeginCircle(RadarStableContext *c, uint32_t sequence)
{
    if (!c || c->status.collecting || c->status.finalizing ||
        (!c->status.rolling && c->status.window_circles >= RADAR_STABLE_CIRCLES)) return 0U;
    c->saved_count = c->count; c->draft_sequence = sequence;
    c->draft_replacements = 0U; c->draft_dropped = 0U;
    memset(c->replaced, 0, sizeof(c->replaced));
    memset(&c->draft_stats, 0, sizeof(c->draft_stats));
    c->status.collecting = 1U;
    return 1U;
}
void RadarStable_Add(RadarStableContext *c, uint16_t a, uint16_t d,
                     uint16_t x, uint16_t y)
{
    uint16_t i, slot, replacement = NO_SLOT;
    uint8_t weakest = RADAR_STABLE_MIN_CIRCLES;
    uint16_t recent;
    RadarStableRepresentative p;
    if (!c || !c->status.collecting || x / RADAR_STABLE_BIN_MM > 255U ||
        y / RADAR_STABLE_BIN_MM > 255U) return;
    p = representative(a, d, x, y);
    recent = DRAFT_MASK;
    if (c->status.window_circles)
        recent |= (uint16_t)(1U << ((c->next_slot + RADAR_STABLE_CIRCLES - 1U) % RADAR_STABLE_CIRCLES));
    for (i = 0U; i < c->count; ++i) {
        RadarStableCandidate *q = &c->candidates[i];
        uint16_t mask = effective_mask(c, q->circle_mask);
        /* The outgoing bin's historical key is still an exact-bin match:
           its incoming measurement reuses the slot with fresh evidence. */
        if (q->circle_mask && q->bin_x == x / RADAR_STABLE_BIN_MM &&
            q->bin_y == y / RADAR_STABLE_BIN_MM) {
            if (!(q->circle_mask & DRAFT_MASK) ||
                center_error(x, y) < center_error(c->draft[i].x_mm, c->draft[i].y_mm))
                c->draft[i] = p;
            q->circle_mask |= DRAFT_MASK;
            return;
        }
        if (c->count == RADAR_STABLE_CAPACITY && !(mask & recent)) {
            uint8_t n = votes(mask);
            if (n < weakest) { weakest = n; replacement = i; }
        }
    }
    slot = c->count;
    if (slot == RADAR_STABLE_CAPACITY) {
        if (replacement == NO_SLOT) { c->draft_dropped = add_u32(c->draft_dropped, 1U); return; }
        slot = replacement; save_replacement(c, slot);
        c->draft_replacements = add_u32(c->draft_replacements, 1U);
    } else { ++c->count; }
    c->candidates[slot].bin_x = (uint8_t)(x / RADAR_STABLE_BIN_MM);
    c->candidates[slot].bin_y = (uint8_t)(y / RADAR_STABLE_BIN_MM);
    c->candidates[slot].circle_mask = DRAFT_MASK;
    c->draft[slot] = p;
}
void RadarStable_RecordStats(RadarStableContext *c, uint32_t raw,
                             uint32_t filtered, uint32_t outside)
{
    if (c && c->status.collecting) record_stats(&c->draft_stats, raw, filtered, outside);
}
void RadarStable_AbortCircle(RadarStableContext *c)
{
    uint16_t i;
    if (!c || !c->status.collecting) return;
    for (i = 0U; i < c->saved_count; ++i) {
        RadarStableCandidate *p = &c->candidates[i];
        if (bit_get(c->replaced, i)) {
            const RadarStableUndo *u = &c->undo[i];
            uint8_t j, k = 0U;
            p->bin_x = u->bin_x; p->bin_y = u->bin_y; p->circle_mask = u->circle_mask;
            for (j = 0U; j < RADAR_STABLE_CIRCLES; ++j)
                if ((u->circle_mask & (uint16_t)(1U << j)) && k < 3U)
                    p->representatives[j] = u->representatives[k++];
        } else p->circle_mask &= HISTORY_MASK;
    }
    for (; i < c->count; ++i) memset(&c->candidates[i], 0, sizeof(c->candidates[i]));
    c->count = c->saved_count; c->status.collecting = 0U;
}
uint8_t RadarStable_CommitCircle(RadarStableContext *c)
{
    uint16_t i, bit;
    uint8_t j;
    if (!c || !c->status.collecting) return 0U;
    bit = (uint16_t)(1U << c->next_slot);
    c->status.candidate_count = 0U;
    for (i = 0U; i < c->count; ++i) {
        RadarStableCandidate *p = &c->candidates[i];
        uint16_t mask = (uint16_t)(p->circle_mask & (uint16_t)~bit & HISTORY_MASK);
        if (p->circle_mask & DRAFT_MASK) {
            mask |= bit; p->representatives[c->next_slot] = c->draft[i];
        }
        p->circle_mask = mask;
        if (mask) ++c->status.candidate_count;
    }
    c->circles[c->next_slot] = c->draft_stats;
    c->circle_sequences[c->next_slot] = c->draft_sequence;
    c->next_slot = (uint8_t)((c->next_slot + 1U) % RADAR_STABLE_CIRCLES);
    if (c->status.window_circles < RADAR_STABLE_CIRCLES) ++c->status.window_circles;
    c->status.first_circle = c->circle_sequences[c->status.window_circles == RADAR_STABLE_CIRCLES ? c->next_slot : 0U];
    c->status.last_circle = c->draft_sequence;
    c->status.raw_points = 0U; c->status.filtered_points = 0U; c->status.outside_points = 0U;
    /* Recompute the 15 small records so a previously saturated total can
       recover exactly when the corresponding saturated circle expires. */
    for (j = 0U; j < c->status.window_circles; ++j) {
        c->status.raw_points = add_u32(c->status.raw_points, c->circles[j].raw_points);
        c->status.filtered_points = add_u32(c->status.filtered_points, c->circles[j].filtered_points);
        c->status.outside_points = add_u32(c->status.outside_points, c->circles[j].outside_points);
    }
    c->status.candidate_replacements = add_u32(c->status.candidate_replacements, c->draft_replacements);
    c->status.candidate_dropped = add_u32(c->status.candidate_dropped, c->draft_dropped);
    c->status.collecting = 0U;
    return c->status.window_circles == RADAR_STABLE_CIRCLES;
}

void RadarStableBatch_Init(RadarStableBatchContext *c)
{
    if (c) memset(c, 0, sizeof(*c));
}
uint8_t RadarStableBatch_BeginCircle(RadarStableBatchContext *c, uint32_t sequence)
{
    if (!c || c->status.collecting || c->status.finalizing ||
        c->status.window_circles >= RADAR_STABLE_CIRCLES) return 0U;
    memcpy(c->checkpoint, c->candidates, sizeof(c->candidates));
    c->saved_count = c->count; c->draft_sequence = sequence;
    c->draft_replacements = 0U; c->draft_dropped = 0U;
    memset(&c->draft_stats, 0, sizeof(c->draft_stats));
    c->status.collecting = 1U;
    return 1U;
}
void RadarStableBatch_Add(RadarStableBatchContext *c, uint16_t a, uint16_t d,
                          uint16_t x, uint16_t y)
{
    uint16_t i, slot, replacement = NO_SLOT, recent = DRAFT_MASK;
    uint8_t weakest = RADAR_STABLE_MIN_CIRCLES;
    RadarStableBatchCandidate *q;
    if (!c || !c->status.collecting || x / RADAR_STABLE_BIN_MM > 255U ||
        y / RADAR_STABLE_BIN_MM > 255U) return;
    if (c->status.window_circles) recent |= (uint16_t)(1U << (c->status.window_circles - 1U));
    for (i = 0U; i < c->count; ++i) {
        q = &c->candidates[i];
        if (q->x_mm / RADAR_STABLE_BIN_MM == x / RADAR_STABLE_BIN_MM &&
            q->y_mm / RADAR_STABLE_BIN_MM == y / RADAR_STABLE_BIN_MM) {
            q->circle_mask |= DRAFT_MASK;
            if (center_error(x, y) < center_error(q->x_mm, q->y_mm)) {
                q->angle_tenths = (uint16_t)(a % 3600U); q->distance_mm = d; q->x_mm = x; q->y_mm = y;
            }
            return;
        }
        if (c->count == RADAR_STABLE_CAPACITY && !(q->circle_mask & recent)) {
            uint8_t n = votes(q->circle_mask);
            if (n < weakest) { weakest = n; replacement = i; }
        }
    }
    slot = c->count;
    if (slot == RADAR_STABLE_CAPACITY) {
        if (replacement == NO_SLOT) { c->draft_dropped = add_u32(c->draft_dropped, 1U); return; }
        slot = replacement; c->draft_replacements = add_u32(c->draft_replacements, 1U);
    } else ++c->count;
    q = &c->candidates[slot];
    q->angle_tenths = (uint16_t)(a % 3600U); q->distance_mm = d; q->x_mm = x; q->y_mm = y;
    q->circle_mask = DRAFT_MASK;
}
void RadarStableBatch_RecordStats(RadarStableBatchContext *c, uint32_t raw,
                                  uint32_t filtered, uint32_t outside)
{
    if (c && c->status.collecting) record_stats(&c->draft_stats, raw, filtered, outside);
}
void RadarStableBatch_AbortCircle(RadarStableBatchContext *c)
{
    if (!c || !c->status.collecting) return;
    memcpy(c->candidates, c->checkpoint, sizeof(c->candidates));
    c->count = c->saved_count; c->status.collecting = 0U;
}
uint8_t RadarStableBatch_CommitCircle(RadarStableBatchContext *c)
{
    uint16_t i, bit;
    if (!c || !c->status.collecting) return 0U;
    bit = (uint16_t)(1U << c->status.window_circles);
    for (i = 0U; i < c->count; ++i) {
        RadarStableBatchCandidate *p = &c->candidates[i];
        if (p->circle_mask & DRAFT_MASK) p->circle_mask = (uint16_t)((p->circle_mask & HISTORY_MASK) | bit);
    }
    if (!c->status.window_circles) c->status.first_circle = c->draft_sequence;
    c->status.last_circle = c->draft_sequence;
    ++c->status.window_circles; c->status.candidate_count = c->count;
    c->status.raw_points = add_u32(c->status.raw_points, c->draft_stats.raw_points);
    c->status.filtered_points = add_u32(c->status.filtered_points, c->draft_stats.filtered_points);
    c->status.outside_points = add_u32(c->status.outside_points, c->draft_stats.outside_points);
    c->status.candidate_replacements = add_u32(c->status.candidate_replacements, c->draft_replacements);
    c->status.candidate_dropped = add_u32(c->status.candidate_dropped, c->draft_dropped);
    c->status.collecting = 0U;
    return c->status.window_circles == RADAR_STABLE_CIRCLES;
}

static uint16_t candidate_mask(const void *context, uint8_t compact, uint16_t i)
{
    return compact ? ((const RadarStableBatchContext *)context)->candidates[i].circle_mask :
                     ((const RadarStableContext *)context)->candidates[i].circle_mask;
}
static RadarStableRepresentative window_representative(const RadarStableContext *c,uint16_t i)
{
    RadarStableRepresentative p = {0U, 0U, 0U, 0U};
    const RadarStableCandidate *q=&c->candidates[i];
    uint8_t j,start=c->status.window_circles==RADAR_STABLE_CIRCLES?c->next_slot:0U;
    uint32_t best_error=UINT32_MAX;
    for(j=0U;j<RADAR_STABLE_CIRCLES;++j){
        uint8_t slot=(uint8_t)((start+j)%RADAR_STABLE_CIRCLES);
        if(q->circle_mask & (uint16_t)(1U<<slot)){
            RadarStableRepresentative r=q->representatives[slot];
            uint32_t e=center_error(r.x_mm,r.y_mm);
            if(e<best_error){best_error=e;p=r;}
        }
    }
    return p;
}
static RadarStableRepresentative candidate_point(const void *context, uint8_t compact,
                                                 uint16_t i)
{
    RadarStableRepresentative p = {0U, 0U, 0U, 0U};
    if (compact) {
        const RadarStableBatchCandidate *q = &((const RadarStableBatchContext *)context)->candidates[i];
        p.angle_tenths = q->angle_tenths; p.distance_mm = q->distance_mm; p.x_mm = q->x_mm; p.y_mm = q->y_mm;
    } else {
        const RadarStableContext *c = (const RadarStableContext *)context;
        p=c->draft[i];
    }
    return p;
}
static uint8_t in_neighborhood(RadarStableRepresentative a, RadarStableRepresentative b)
{
    int32_t dx = (int32_t)a.x_mm - b.x_mm, dy = (int32_t)a.y_mm - b.y_mm;
    return (uint32_t)(dx * dx + dy * dy) <= RADAR_STABLE_RADIUS_MM * RADAR_STABLE_RADIUS_MM;
}
static void select_begin(RadarStableFinalize *w)
{
    w->stage = FINAL_SELECT; w->index = 0U; w->best = NO_SLOT; w->best_votes = 0U;
}
static uint8_t finalize_begin(RadarStableFinalize *w, RadarStableStatus *status)
{
    if (status->collecting || status->finalizing) return 0U;
    memset(w, 0, sizeof(*w)); select_begin(w);
    status->finalizing = 1U; status->stable_count = 0U;
    return 1U;
}
static uint8_t finalize_step(const void *c, uint8_t compact, uint16_t count,
                             RadarStableFinalize *w, RadarStableStatus *status,
                             RadarStablePoint *output, uint32_t budget)
{
    if (!status->finalizing) return w->stage == FINAL_DONE;
    while (budget) {
        if (w->stage == FINAL_SELECT) {
            if (w->index < count) {
                uint16_t i = w->index++;
                uint8_t n;
                --budget;
                if(bit_get(w->consumed,i) || bit_get(w->failed,i))continue;
                n=votes(candidate_mask(c,compact,i));
                if (n > w->best_votes) {
                    w->best_votes = n; w->best = i;
                }
            } else if (w->best == NO_SLOT) {
                w->stage = FINAL_DONE; status->stable_count = w->output_count;
                status->finalizing = 0U; return 1U;
            } else {
                w->peak = candidate_point(c, compact, w->best);
                w->neighborhood_mask = 0U; w->index = 0U; w->stage = FINAL_SUPPORT;
            }
        } else if (w->stage == FINAL_SUPPORT) {
            if (w->index < count) {
                uint16_t i = w->index++;
                --budget;
                if (!bit_get(w->consumed, i) && candidate_mask(c, compact, i) &&
                    in_neighborhood(candidate_point(c, compact, i), w->peak))
                    w->neighborhood_mask |= candidate_mask(c, compact, i);
            } else if (votes(w->neighborhood_mask) < RADAR_STABLE_MIN_CIRCLES) {
                bit_set(w->failed, w->best); select_begin(w);
            } else {
                RadarStablePoint *p = &output[w->output_count++];
                p->angle_tenths = w->peak.angle_tenths; p->distance_mm = w->peak.distance_mm;
                p->x_mm = w->peak.x_mm; p->y_mm = w->peak.y_mm;
                p->votes = votes(w->neighborhood_mask); p->flags = RADAR_STABLE_POINT_CONFIRMED;
                w->stage = FINAL_CONSUME; w->index = 0U;
            }
        } else if (w->stage == FINAL_CONSUME) {
            if (w->index < count) {
                uint16_t i = w->index++;
                --budget;
                if (!bit_get(w->consumed, i) && candidate_mask(c, compact, i) &&
                    in_neighborhood(candidate_point(c, compact, i), w->peak)) bit_set(w->consumed, i);
            } else select_begin(w);
        } else return 1U;
    }
    return 0U;
}
uint8_t RadarStable_FinalizeBegin(RadarStableContext *c)
{
    uint16_t i;
    if(!c || !finalize_begin(&c->finalize,&c->status))return 0U;
    /* No circle is open during finalization. Reuse its scratch representatives
       so each peak check does not scan all fifteen history slots again. */
    for(i=0U;i<c->count;i++)c->draft[i]=window_representative(c,i);
    return 1U;
}
uint8_t RadarStable_FinalizeStep(RadarStableContext *c, uint32_t budget)
{
    return c ? finalize_step(c, 0U, c->count, &c->finalize, &c->status, c->output, budget) : 0U;
}
const RadarStablePoint *RadarStable_Points(const RadarStableContext *c, uint16_t *count)
{
    if (count) *count = c && !c->status.finalizing ? c->status.stable_count : 0U;
    return c ? c->output : 0;
}
const RadarStableStatus *RadarStable_GetStatus(const RadarStableContext *c)
{
    return c ? &c->status : 0;
}
uint8_t RadarStableBatch_FinalizeBegin(RadarStableBatchContext *c)
{
    return c ? finalize_begin(&c->finalize, &c->status) : 0U;
}
uint8_t RadarStableBatch_FinalizeStep(RadarStableBatchContext *c, uint32_t budget)
{
    return c ? finalize_step(c, 1U, c->count, &c->finalize, &c->status, c->output, budget) : 0U;
}
const RadarStablePoint *RadarStableBatch_Points(const RadarStableBatchContext *c, uint16_t *count)
{
    if (count) *count = c && !c->status.finalizing ? c->status.stable_count : 0U;
    return c ? c->output : 0;
}
const RadarStableStatus *RadarStableBatch_GetStatus(const RadarStableBatchContext *c)
{
    return c ? &c->status : 0;
}
