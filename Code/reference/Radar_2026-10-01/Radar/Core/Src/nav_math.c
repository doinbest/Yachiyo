#include "nav_math.h"

#include "nav_sine_q15.h"
#include "navigation_config.h"

static uint32_t integer_square_root(uint32_t value)
{
  uint32_t result = 0U;
  uint32_t bit = 1UL << 30;

  while (bit > value)
  {
    bit >>= 2;
  }

  while (bit != 0U)
  {
    if (value >= result + bit)
    {
      value -= result + bit;
      result = (result >> 1) + bit;
    }
    else
    {
      result >>= 1;
    }
    bit >>= 2;
  }

  return result;
}

int32_t NavMath_SinQ15(uint16_t angle_tenths)
{
  uint16_t angle = (uint16_t)(angle_tenths % 3600U);

  if (angle <= 900U)
  {
    return (int32_t)g_nav_sine_q15[angle];
  }
  if (angle <= 1800U)
  {
    return (int32_t)g_nav_sine_q15[1800U - angle];
  }
  if (angle <= 2700U)
  {
    return -(int32_t)g_nav_sine_q15[angle - 1800U];
  }
  return -(int32_t)g_nav_sine_q15[3600U - angle];
}

int32_t NavMath_MulQ15(uint16_t value, int32_t q15_factor)
{
  int32_t product = (int32_t)value * q15_factor;

  if (product >= 0)
  {
    product += 16384;
  }
  else
  {
    product -= 16384;
  }
  return product / 32768;
}

void NavMath_PolarToCartesian(uint16_t angle_tenths,
                              uint16_t distance_mm,
                              int32_t *x_mm,
                              int32_t *y_mm)
{
  int32_t sine = NavMath_SinQ15(angle_tenths);
  int32_t cosine = NavMath_SinQ15((uint16_t)(angle_tenths + 900U));

  if (x_mm != 0)
  {
    *x_mm = (int32_t)NAV_RADAR_X_MM + NavMath_MulQ15(distance_mm, sine);
  }
  if (y_mm != 0)
  {
    *y_mm = (int32_t)NAV_RADAR_Y_MM + NavMath_MulQ15(distance_mm, cosine);
  }
}

uint16_t NavMath_DistanceMm(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  int32_t dx = x1 - x0;
  int32_t dy = y1 - y0;
  uint32_t squared = (uint32_t)(dx * dx) + (uint32_t)(dy * dy);
  uint32_t root = integer_square_root(squared);
  uint32_t lower_error = squared - (root * root);
  uint32_t next = root + 1U;
  uint32_t upper_error = (next * next) - squared;

  if (upper_error < lower_error)
  {
    root = next;
  }
  if (root > 65535U)
  {
    root = 65535U;
  }
  return (uint16_t)root;
}
