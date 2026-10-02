#ifndef NAV_MATH_H
#define NAV_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int32_t NavMath_SinQ15(uint16_t angle_tenths);
int32_t NavMath_MulQ15(uint16_t value, int32_t q15_factor);
void NavMath_PolarToCartesian(uint16_t angle_tenths,
                              uint16_t distance_mm,
                              int32_t *x_mm,
                              int32_t *y_mm);
uint16_t NavMath_DistanceMm(int32_t x0, int32_t y0, int32_t x1, int32_t y1);

#ifdef __cplusplus
}
#endif

#endif
