#include "radar_geometry.h"
#include "radar_stability.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#endif

static RadarStableContext rolling;
static RadarStableBatchContext batch;
static void geometry_factories(void)
{
    RadarGeometry g;
    int32_t x, y;
    RadarGeometry_Default(&g, 1U);
    RadarGeometry_Project(&g, 0U, 1000U, &x, &y);
    assert(x == 230 && y == 1230);
    RadarGeometry_Project(&g, 900U, 1000U, &x, &y);
    assert(x == 1230 && y == 230);
    RadarGeometry_Project(&g, 1800U, 1000U, &x, &y);
    assert(x == 230 && y == -770);
    RadarGeometry_Project(&g, 2700U, 1000U, &x, &y);
    assert(x == -770 && y == 230);
    RadarGeometry_Project(&g, 450U, 100U, &x, &y);
    assert(x == 301 && y == 301);
    RadarGeometry_Project(&g, 4050U, 100U, &x, &y);
    assert(x == 301 && y == 301);
    RadarGeometry_Default(&g, 2U);
    RadarGeometry_Project(&g, 1800U, 1000U, &x, &y);
    assert(x == 3284 && y == 308);
    RadarGeometry_Project(&g, 0U, 1000U, &x, &y);
    assert(x == 1276 && y == 320);
    g.matrix_q16[0] = INT32_MAX; g.matrix_q16[1] = INT32_MAX;
    g.origin_q16[0] = INT32_MAX;
    RadarGeometry_Project(&g, 1350U, 65535U, &x, 0);
    assert(x == INT32_MAX);
    g.matrix_q16[0] = INT32_MIN; g.matrix_q16[1] = INT32_MIN;
    RadarGeometry_Project(&g, 1350U, 65535U, &x, 0);
    assert(x == INT32_MIN);
}

#ifndef RADAR_GEOMETRY_RED_ONLY
static const RadarStablePoint *finish(uint16_t *count, uint32_t budget)
{
    unsigned steps = 0U;
    assert(RadarStable_FinalizeBegin(&rolling));
    assert(!RadarStable_FinalizeStep(&rolling, 0U));
    while (!RadarStable_FinalizeStep(&rolling, budget)) assert(++steps < 300000U);
    return RadarStable_Points(&rolling, count);
}
static const RadarStablePoint *batch_finish(uint16_t *count)
{
    assert(RadarStableBatch_FinalizeBegin(&batch));
    while (!RadarStableBatch_FinalizeStep(&batch, 17U)) { }
    return RadarStableBatch_Points(&batch, count);
}
static void circle(uint32_t sequence, uint16_t x, uint16_t y, unsigned repeats)
{
    unsigned i;
    assert(RadarStable_BeginCircle(&rolling, sequence));
    for (i = 0U; i < repeats; ++i) RadarStable_Add(&rolling, 90U, 700U, x, y);
    (void)RadarStable_CommitCircle(&rolling);
}
static void distinct_votes_and_representative(void)
{
    const RadarStablePoint *p;
    uint16_t n;
    RadarStable_Init(&rolling, 0U);
    circle(1U, 100U, 100U, 100U);
    p = finish(&n, 1U); (void)p; assert(n == 0U);
    assert(RadarStable_BeginCircle(&rolling, 2U));
    RadarStable_Add(&rolling, 12U, 712U, 105U, 105U);
    RadarStable_Add(&rolling, 13U, 713U, 105U, 105U);
    RadarStable_RecordStats(&rolling, 9U, 2U, 1U);
    assert(!RadarStable_CommitCircle(&rolling));
    circle(3U, 101U, 101U, 1U);
    p = finish(&n, 2U);
    assert(n == 1U && p[0].votes == 3U && p[0].flags == 1U);
    assert(p[0].x_mm == 105U && p[0].y_mm == 105U);
    assert(p[0].angle_tenths == 12U && p[0].distance_mm == 712U);
    assert(RadarStable_GetStatus(&rolling)->raw_points == 9U);
    assert(RadarStable_GetStatus(&rolling)->filtered_points == 2U);
    assert(RadarStable_GetStatus(&rolling)->outside_points == 1U);
    p = finish(&n, 500U);
    assert(n == 1U && p[0].votes == 3U); /* finalization did not consume votes */
}
static void fixed_neighborhood_and_failed_edge(void)
{
    uint16_t n;
    const RadarStablePoint *p;
    unsigned i;
    RadarStable_Init(&rolling, 0U);
    for (i = 1U; i <= 3U; ++i) {
        assert(RadarStable_BeginCircle(&rolling, i));
        RadarStable_Add(&rolling, 1U, 500U, 105U, 105U);
        RadarStable_Add(&rolling, 2U, 550U, 155U, 105U);
        if (i == 1U) RadarStable_Add(&rolling, 3U, 525U, 130U, 105U);
        (void)RadarStable_CommitCircle(&rolling);
    }
    p = finish(&n, 3U);
    assert(n == 2U && p[0].x_mm == 105U && p[1].x_mm == 155U);
    RadarStable_Init(&rolling, 0U);
    circle(1U, 100U, 105U, 1U);
    circle(2U, 125U, 105U, 1U);
    circle(3U, 150U, 105U, 1U);
    p = finish(&n, 1U);
    assert(n == 1U && p[0].x_mm == 125U && p[0].votes == 3U);
    RadarStable_Init(&rolling, 0U);
    assert(RadarStable_BeginCircle(&rolling, 1U));
    RadarStable_Add(&rolling, 1U, 500U, 100U, 100U);
    RadarStable_Add(&rolling, 2U, 500U, 110U, 100U);
    RadarStable_Add(&rolling, 3U, 500U, 120U, 100U);
    (void)RadarStable_CommitCircle(&rolling);
    (void)finish(&n, 1U); assert(n == 0U);
    circle(2U, 130U, 100U, 1U);
    circle(3U, 100U, 100U, 1U);
    p = finish(&n, 1U); assert(n == 1U && p[0].votes == 3U);
}
static void rolling_expiry_and_abort(void)
{
    uint16_t n;
    const RadarStablePoint *p;
    unsigned i;
    RadarStable_Init(&rolling, 1U);
    for (i = 1U; i <= 15U; ++i) {
        assert(RadarStable_BeginCircle(&rolling, i));
        if (i <= 4U) RadarStable_Add(&rolling, (uint16_t)i, 100U,
                                    (uint16_t)(i == 1U ? 105U : 99U + i), 105U);
        RadarStable_RecordStats(&rolling, i, 1U, 0U);
        assert(RadarStable_CommitCircle(&rolling) == (i == 15U));
    }
    p = finish(&n, 7U);
    assert(n == 1U && p[0].x_mm == 105U && p[0].votes == 4U);
    assert(RadarStable_BeginCircle(&rolling, 16U));
    RadarStable_Add(&rolling, 16U, 150U, 104U, 105U);
    RadarStable_RecordStats(&rolling, 999U, 999U, 999U);
    RadarStable_AbortCircle(&rolling);
    p = finish(&n, 7U);
    assert(n == 1U && p[0].x_mm == 105U && p[0].votes == 4U);
    assert(RadarStable_GetStatus(&rolling)->raw_points == 120U);
    assert(RadarStable_GetStatus(&rolling)->first_circle == 1U);
    assert(RadarStable_GetStatus(&rolling)->last_circle == 15U);
    circle(16U, 0U, 0U, 0U);
    p = finish(&n, 7U);
    assert(n == 1U && p[0].x_mm == 103U && p[0].angle_tenths == 4U && p[0].votes == 3U);
    assert(RadarStable_GetStatus(&rolling)->first_circle == 2U);
    assert(RadarStable_GetStatus(&rolling)->last_circle == 16U);
    assert(RadarStable_GetStatus(&rolling)->raw_points == 119U);
    circle(17U, 0U, 0U, 0U);
    (void)finish(&n, 7U); assert(n == 0U);
}
static void fill_circle(void)
{
    unsigned i;
    for (i = 0U; i < 256U; ++i) RadarStable_Add(&rolling, (uint16_t)i, 100U,
        (uint16_t)(5U + 100U * (i % 16U)), (uint16_t)(5U + 100U * (i / 16U)));
}
static void capacity_replacement_and_rollback(void)
{
    RadarStableCandidate before[256];
    unsigned i;
    RadarStable_Init(&rolling, 1U);
    assert(RadarStable_BeginCircle(&rolling, 1U)); fill_circle();
    (void)RadarStable_CommitCircle(&rolling);
    assert(RadarStable_BeginCircle(&rolling, 2U));
    RadarStable_Add(&rolling, 1U, 100U, 2005U, 2005U);
    (void)RadarStable_CommitCircle(&rolling);
    assert(RadarStable_GetStatus(&rolling)->candidate_dropped == 1U);
    memcpy(before, rolling.candidates, sizeof(before));
    assert(RadarStable_BeginCircle(&rolling, 3U));
    RadarStable_Add(&rolling, 2U, 100U, 2005U, 2005U);
    RadarStable_Add(&rolling, 3U, 120U, 2065U, 2005U);
    RadarStable_AbortCircle(&rolling);
    assert(memcmp(before, rolling.candidates, sizeof(before)) == 0);
    assert(RadarStable_GetStatus(&rolling)->candidate_replacements == 0U);
    assert(RadarStable_BeginCircle(&rolling, 3U));
    RadarStable_Add(&rolling, 2U, 100U, 2005U, 2005U);
    (void)RadarStable_CommitCircle(&rolling);
    assert(rolling.candidates[0].bin_x == 200U && rolling.candidates[1].bin_x == 10U);
    assert(RadarStable_GetStatus(&rolling)->candidate_replacements == 1U);
    for (i = 4U; i <= 15U; ++i) circle(i, 2005U, 2005U, 1U);
    /* Filling current bins protects them. Replacement can still recover the
       three saved history representatives, including the outgoing slot. */
    assert(RadarStable_BeginCircle(&rolling, 16U));
    RadarStable_Add(&rolling, 9U, 900U, 2105U, 2105U);
    RadarStable_AbortCircle(&rolling);
    assert(RadarStable_GetStatus(&rolling)->first_circle == 1U);
}
static void replacement_restores_two_votes_and_outgoing_representative(void)
{
    RadarStableCandidate before[256];
    unsigned c, i;
    RadarStable_Init(&rolling, 1U);
    for (c = 1U; c <= 15U; ++c) {
        assert(RadarStable_BeginCircle(&rolling, c));
        if (c <= 4U) {
            for (i = 1U; i < 256U; ++i) RadarStable_Add(&rolling, (uint16_t)i, 100U,
                (uint16_t)(5U + 100U * (i % 16U)), (uint16_t)(5U + 100U * (i / 16U)));
        }
        if (c == 1U || c == 3U || c == 5U)
            RadarStable_Add(&rolling, (uint16_t)c, 100U, (uint16_t)(c == 1U ? 5U : c), 5U);
        (void)RadarStable_CommitCircle(&rolling);
    }
    /* Slot 255 has three total votes, two remain valid in the next window.
       Every other slot has three valid votes and cannot be replaced. */
    memcpy(before, rolling.candidates, sizeof(before));
    assert(RadarStable_BeginCircle(&rolling, 16U));
    RadarStable_Add(&rolling, 100U, 900U, 2105U, 2105U);
    assert(rolling.candidates[255].bin_x == 210U);
    assert(rolling.undo[255].circle_mask == 0x0015U);
    RadarStable_Add(&rolling, 200U, 999U, 106U, 5U);
    RadarStable_AbortCircle(&rolling);
    assert(!memcmp(before, rolling.candidates, sizeof(before)));
    assert(RadarStable_GetStatus(&rolling)->candidate_replacements == 0U);
    assert(RadarStable_BeginCircle(&rolling, 16U));
    RadarStable_Add(&rolling, 100U, 900U, 2105U, 2105U);
    assert(RadarStable_CommitCircle(&rolling));
    assert(rolling.candidates[255].circle_mask == 1U);
    assert(rolling.candidates[255].representatives[0].angle_tenths == 100U);
}
static void batch_checkpoint_and_static_limit(void)
{
    uint16_t n;
    const RadarStablePoint *p;
    unsigned i;
    RadarStableBatchCandidate before[256];
    RadarStableBatch_Init(&batch);
    assert(RadarStableBatch_BeginCircle(&batch, 1U));
    RadarStableBatch_Add(&batch, 1U, 500U, 101U, 105U);
    (void)RadarStableBatch_CommitCircle(&batch);
    memcpy(before, batch.candidates, sizeof(before));
    assert(RadarStableBatch_BeginCircle(&batch, 2U));
    RadarStableBatch_Add(&batch, 2U, 600U, 105U, 105U);
    RadarStableBatch_RecordStats(&batch, 100U, 50U, 40U);
    RadarStableBatch_AbortCircle(&batch);
    assert(!memcmp(before, batch.candidates, sizeof(before)));
    assert(RadarStableBatch_GetStatus(&batch)->raw_points == 0U);
    for (i = 2U; i <= 15U; ++i) {
        assert(RadarStableBatch_BeginCircle(&batch, i));
        RadarStableBatch_Add(&batch, (uint16_t)i, 500U, 102U, 105U);
        assert(RadarStableBatch_CommitCircle(&batch) == (i == 15U));
    }
    assert(!RadarStableBatch_BeginCircle(&batch, 16U));
    p = batch_finish(&n);
    assert(n == 1U && p[0].votes == 15U && p[0].angle_tenths == 2U);
    p = batch_finish(&n); assert(n == 1U && p[0].votes == 15U);
    RadarStable_Init(&rolling, 0U);
    for (i = 1U; i <= 15U; ++i) circle(i, 105U, 105U, 1U);
    assert(!RadarStable_BeginCircle(&rolling, 16U));
}
static void stats_saturate_and_coordinates(void)
{
    uint16_t n;
    RadarStable_Init(&rolling, 1U);
    assert(!RadarStable_CommitCircle(&rolling));
    assert(RadarStable_BeginCircle(&rolling, 8U));
    assert(!RadarStable_BeginCircle(&rolling, 9U));
    assert(!RadarStable_FinalizeBegin(&rolling));
    RadarStable_RecordStats(&rolling, UINT32_MAX, UINT32_MAX, UINT32_MAX);
    RadarStable_RecordStats(&rolling, 1U, 1U, 1U);
    RadarStable_Add(&rolling, 3601U, 100U, 2560U, 0U);
    (void)RadarStable_CommitCircle(&rolling);
    assert(RadarStable_GetStatus(&rolling)->raw_points == UINT32_MAX);
    assert(RadarStable_GetStatus(&rolling)->candidate_count == 0U);
    circle(9U, 2559U, 2559U, 1U);
    circle(10U, 2559U, 2559U, 1U);
    circle(11U, 2559U, 2559U, 1U);
    (void)finish(&n, 1U); assert(n == 1U);
}
static uint32_t random_state = 0x15370A49U;
static uint32_t random_u32(void)
{
    random_state = random_state * 1664525U + 1013904223U;
    return random_state;
}
#ifdef RADAR_F103_REFERENCE_TEST
extern void RadarRef_Init(void);
extern void RadarRef_Add(uint16_t a, uint16_t d, uint16_t x, uint16_t y);
extern void RadarRef_Commit(void);
extern uint16_t RadarRef_Finish(void);
extern void RadarRef_Point(uint16_t i, uint16_t *a, uint16_t *d, uint16_t *x, uint16_t *y);
extern void RadarRef_Project(uint16_t a, uint16_t d, int32_t *x, int32_t *y);
static void reference_projection(void)
{
    static const uint16_t distances[] = {0U, 1U, 100U, 999U, 2400U, 65535U};
    RadarGeometry g;
    unsigned a, d;
    RadarGeometry_Default(&g, RADAR_F103_REFERENCE_SITUATION);
    for (a = 0U; a < 3600U; ++a) for (d = 0U; d < 6U; ++d) {
        int32_t x, y, rx, ry;
        RadarGeometry_Project(&g, (uint16_t)a, distances[d], &x, &y);
        RadarRef_Project((uint16_t)a, distances[d], &rx, &ry);
        assert(x == rx && y == ry);
    }
}
#endif
static void randomized_static_equivalence(void)
{
    unsigned trial, c, i;
    for (trial = 0U; trial < 40U; ++trial) {
        RadarStable_Init(&rolling, 0U); RadarStableBatch_Init(&batch);
#ifdef RADAR_F103_REFERENCE_TEST
        RadarRef_Init();
#endif
        for (c = 1U; c <= 15U; ++c) {
            assert(RadarStable_BeginCircle(&rolling, c));
            assert(RadarStableBatch_BeginCircle(&batch, c));
            for (i = 0U; i < 420U; ++i) {
                uint16_t a = (uint16_t)(random_u32() % 3600U);
                uint16_t d = (uint16_t)(100U + random_u32() % 2000U);
                uint16_t x, y;
                if (i < 120U) {
                    /* Persistent modes jitter across neighboring 10 mm bins. */
                    x = (uint16_t)(205U + 90U * (i % 20U) + random_u32() % 31U);
                    y = (uint16_t)(205U + 90U * (i % 17U) + random_u32() % 31U);
                } else {
                    x = (uint16_t)(200U + random_u32() % 1900U);
                    y = (uint16_t)(200U + random_u32() % 1900U);
                }
                RadarStable_Add(&rolling, a, d, x, y);
                RadarStableBatch_Add(&batch, a, d, x, y);
#ifdef RADAR_F103_REFERENCE_TEST
                RadarRef_Add(a, d, x, y);
#endif
            }
            (void)RadarStable_CommitCircle(&rolling);
            (void)RadarStableBatch_CommitCircle(&batch);
#ifdef RADAR_F103_REFERENCE_TEST
            RadarRef_Commit();
#endif
        }
        {
            uint16_t n, bn;
            const RadarStablePoint *p = finish(&n, (uint32_t)(1U + trial * 11U));
            const RadarStablePoint *bp = batch_finish(&bn);
            assert(n == bn && !memcmp(p, bp, n * sizeof(*p)));
#ifdef RADAR_F103_REFERENCE_TEST
            assert(RadarRef_Finish() == n);
            for (i = 0U; i < n; ++i) {
                uint16_t a, d, x, y;
                RadarRef_Point((uint16_t)i, &a, &d, &x, &y);
                assert(p[i].angle_tenths == a && p[i].distance_mm == d);
                assert(p[i].x_mm == x && p[i].y_mm == y);
            }
#endif
        }
    }
}
static void benchmark(void)
{
    unsigned c, i, steps = 0U;
    double start, end, step_start, max_step_ms = 0.0;
    uint16_t n;
#ifdef _WIN32
    LARGE_INTEGER frequency, stamp;
    assert(QueryPerformanceFrequency(&frequency));
#define BENCH_TIME_MS(target) do { QueryPerformanceCounter(&stamp); (target) = 1000.0 * (double)stamp.QuadPart / (double)frequency.QuadPart; } while (0)
#else
#define BENCH_TIME_MS(target) do { (target) = 1000.0 * (double)clock() / CLOCKS_PER_SEC; } while (0)
#endif
    RadarStable_Init(&rolling, 1U);
    for (c = 1U; c <= 15U; ++c) {
        assert(RadarStable_BeginCircle(&rolling, c)); fill_circle();
        (void)RadarStable_CommitCircle(&rolling);
    }
    BENCH_TIME_MS(start);
    for (i = 0U; i < 20U; ++i) {
        uint8_t done;
        assert(RadarStable_FinalizeBegin(&rolling));
        do {
            double step_end;
            BENCH_TIME_MS(step_start);
            done = RadarStable_FinalizeStep(&rolling, 2048U); ++steps;
            BENCH_TIME_MS(step_end);
            if (step_end - step_start > max_step_ms) max_step_ms = step_end - step_start;
        } while (!done);
    }
    BENCH_TIME_MS(end); (void)RadarStable_Points(&rolling, &n); assert(n == 256U);
    printf("native finalize worst-case 256 peaks: %.3f ms/run, %.3f us/2048-inspection call (max %.3f ms), %u calls/run; no MCU timing claim\n",
        (end - start) / 20.0, 1000.0 * (end - start) / steps, max_step_ms, steps / 20U);
#undef BENCH_TIME_MS
}
#endif
int main(void)
{
    geometry_factories();
#ifndef RADAR_GEOMETRY_RED_ONLY
    distinct_votes_and_representative();
    fixed_neighborhood_and_failed_edge();
    rolling_expiry_and_abort();
    capacity_replacement_and_rollback();
    replacement_restores_two_votes_and_outgoing_representative();
    batch_checkpoint_and_static_limit();
    stats_saturate_and_coordinates();
    randomized_static_equivalence();
#ifdef RADAR_F103_REFERENCE_TEST
    reference_projection();
#endif
    benchmark();
    printf("radar stability: PASS; rolling=%lu bytes batch=%lu bytes pair=%lu bytes\n",
        (unsigned long)sizeof(rolling), (unsigned long)sizeof(batch),
        (unsigned long)(2U * sizeof(batch)));
#endif
    return 0;
}
