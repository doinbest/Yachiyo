#ifndef RADAR_GEOMETRY_H
#define RADAR_GEOMETRY_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    int32_t matrix_q16[4]; /* row major, local X/Y to field X/Y */
    int32_t origin_q16[2];
    uint16_t zero_tenths; /* raw angle for local +X; increasing angles turn to -Y */
} RadarGeometry;
void RadarGeometry_Default(RadarGeometry *geometry, uint8_t situation);
void RadarGeometry_Project(const RadarGeometry *geometry, uint16_t angle_tenths,
                           uint16_t distance_mm, int32_t *x_mm, int32_t *y_mm);
#ifdef __cplusplus
}
#endif
#endif
