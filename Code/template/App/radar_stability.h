#ifndef RADAR_STABILITY_H
#define RADAR_STABILITY_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define RADAR_STABLE_CAPACITY 256U
#define RADAR_STABLE_CIRCLES 15U
#define RADAR_STABLE_BIN_MM 10U
#define RADAR_STABLE_RADIUS_MM 30U
#define RADAR_STABLE_MIN_CIRCLES 3U
#define RADAR_STABLE_POINT_CONFIRMED 1U
typedef struct {
    uint16_t angle_tenths, distance_mm, x_mm, y_mm;
    uint8_t votes, flags;
} RadarStablePoint;
typedef struct {
    uint16_t angle_tenths, distance_mm, x_mm, y_mm;
} RadarStableRepresentative;
typedef struct {
    uint8_t bin_x, bin_y;
    uint16_t circle_mask; /* 0..14 committed ring slots, bit 15 current draft */
    RadarStableRepresentative representatives[RADAR_STABLE_CIRCLES];
} RadarStableCandidate;
typedef struct {
    uint8_t bin_x, bin_y;
    uint16_t circle_mask;
    RadarStableRepresentative representatives[3];
} RadarStableUndo;
typedef struct {
    uint32_t first_circle, last_circle;
    uint32_t raw_points, filtered_points, outside_points;
    uint32_t candidate_replacements, candidate_dropped;
    uint16_t candidate_count, stable_count;
    uint8_t window_circles, rolling, collecting, finalizing;
} RadarStableStatus;
typedef struct {
    uint32_t raw_points, filtered_points, outside_points;
} RadarStableCircleStats;
typedef struct {
    uint8_t consumed[32], failed[32];
    RadarStableRepresentative peak;
    uint16_t index, best, neighborhood_mask, output_count;
    uint8_t stage, best_votes;
} RadarStableFinalize;
typedef struct {
    RadarStableCandidate candidates[RADAR_STABLE_CAPACITY];
    RadarStableRepresentative draft[RADAR_STABLE_CAPACITY];
    RadarStableUndo undo[RADAR_STABLE_CAPACITY];
    uint8_t replaced[32];
    RadarStablePoint output[RADAR_STABLE_CAPACITY];
    RadarStableCircleStats circles[RADAR_STABLE_CIRCLES], draft_stats;
    uint32_t circle_sequences[RADAR_STABLE_CIRCLES], draft_sequence;
    uint32_t draft_replacements, draft_dropped;
    RadarStableStatus status;
    RadarStableFinalize finalize;
    uint16_t count, saved_count;
    uint8_t next_slot;
} RadarStableContext;
/* Two static scan positions fit together in <=16 KiB. Their storage can share
   a union with RadarStableContext: neither mode retains a raw point cloud. */
typedef struct {
    uint16_t angle_tenths, distance_mm, x_mm, y_mm, circle_mask;
} RadarStableBatchCandidate;
typedef struct {
    RadarStableBatchCandidate candidates[RADAR_STABLE_CAPACITY];
    RadarStableBatchCandidate checkpoint[RADAR_STABLE_CAPACITY];
    RadarStablePoint output[RADAR_STABLE_CAPACITY];
    RadarStableCircleStats draft_stats;
    uint32_t draft_sequence, draft_replacements, draft_dropped;
    RadarStableStatus status;
    RadarStableFinalize finalize;
    uint16_t count, saved_count;
} RadarStableBatchContext;

/* Begin/Commit/Abort are whole-circle transactions. Begin rejects an active
   transaction or finalization; nonrolling mode stops after 15 commits.
   Add accepts field coordinates 0..2559 mm (uint8 bins); caller filters field
   bounds before Add. Stats count raw/filter/outside once at the caller's gate.
   A capacity replacement increments replacements; rejected new bins increment
   dropped. Neither counter counts normal temporal/spatial suppression.
   Finalize budget counts candidate inspections, not wall time. Points exposes
   output only when finalization finishes. Candidate evidence stays unchanged. */
void RadarStable_Init(RadarStableContext *context, uint8_t rolling);
uint8_t RadarStable_BeginCircle(RadarStableContext *context, uint32_t sequence);
void RadarStable_Add(RadarStableContext *context, uint16_t angle, uint16_t distance,
                     uint16_t x, uint16_t y);
void RadarStable_RecordStats(RadarStableContext *context, uint32_t raw,
                             uint32_t filtered, uint32_t outside);
void RadarStable_AbortCircle(RadarStableContext *context);
uint8_t RadarStable_CommitCircle(RadarStableContext *context);
uint8_t RadarStable_FinalizeBegin(RadarStableContext *context);
uint8_t RadarStable_FinalizeStep(RadarStableContext *context, uint32_t budget);
const RadarStablePoint *RadarStable_Points(const RadarStableContext *context,
                                          uint16_t *count);
const RadarStableStatus *RadarStable_GetStatus(const RadarStableContext *context);
void RadarStableBatch_Init(RadarStableBatchContext *context);
uint8_t RadarStableBatch_BeginCircle(RadarStableBatchContext *context, uint32_t sequence);
void RadarStableBatch_Add(RadarStableBatchContext *context, uint16_t angle,
                          uint16_t distance, uint16_t x, uint16_t y);
void RadarStableBatch_RecordStats(RadarStableBatchContext *context, uint32_t raw,
                                  uint32_t filtered, uint32_t outside);
void RadarStableBatch_AbortCircle(RadarStableBatchContext *context);
uint8_t RadarStableBatch_CommitCircle(RadarStableBatchContext *context);
uint8_t RadarStableBatch_FinalizeBegin(RadarStableBatchContext *context);
uint8_t RadarStableBatch_FinalizeStep(RadarStableBatchContext *context, uint32_t budget);
const RadarStablePoint *RadarStableBatch_Points(const RadarStableBatchContext *context,
                                               uint16_t *count);
const RadarStableStatus *RadarStableBatch_GetStatus(const RadarStableBatchContext *context);
typedef char RadarStablePoint_size_is_10[(sizeof(RadarStablePoint) == 10U) ? 1 : -1];
typedef char RadarStableRepresentative_size_is_8[(sizeof(RadarStableRepresentative) == 8U) ? 1 : -1];
typedef char RadarStableBatchCandidate_size_is_10[(sizeof(RadarStableBatchCandidate) == 10U) ? 1 : -1];
typedef char RadarStableContext_fits_44KiB[(sizeof(RadarStableContext) <= 44U * 1024U) ? 1 : -1];
typedef char RadarStableBatch_pair_fits_16KiB[(2U * sizeof(RadarStableBatchContext) <= 16U * 1024U) ? 1 : -1];
#ifdef __cplusplus
}
#endif
#endif
