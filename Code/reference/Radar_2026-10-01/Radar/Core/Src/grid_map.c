#include "grid_map.h"

static uint8_t find_interval(int32_t value, const uint16_t boundaries[6])
{
  uint8_t index;

  for (index = 0U; index < NAV_GRID_COLUMNS; ++index)
  {
    if (value >= (int32_t)boundaries[index] &&
        value < (int32_t)boundaries[index + 1U])
    {
      return (uint8_t)(index + 1U);
    }
  }
  return 0U;
}

uint8_t GridMap_FindCell(int32_t x_mm,
                         int32_t y_mm,
                         uint8_t *row,
                         uint8_t *column)
{
  uint8_t found_column = find_interval(x_mm, g_nav_grid_x_mm);
  uint8_t found_row = find_interval(y_mm, g_nav_grid_y_mm);

  if (found_row == 0U || found_column == 0U)
  {
    return 0U;
  }
  if (row != 0)
  {
    *row = found_row;
  }
  if (column != 0)
  {
    *column = found_column;
  }
  return 1U;
}

uint8_t GridMap_CellIndex(uint8_t row, uint8_t column)
{
  if (row < 1U || row > NAV_GRID_ROWS ||
      column < 1U || column > NAV_GRID_COLUMNS)
  {
    return 0xFFU;
  }
  return (uint8_t)(((row - 1U) * NAV_GRID_COLUMNS) + (column - 1U));
}

void GridMap_CellCenter(uint8_t row,
                        uint8_t column,
                        uint16_t *x_mm,
                        uint16_t *y_mm)
{
  uint8_t row_index = (row >= 1U && row <= NAV_GRID_ROWS) ? (uint8_t)(row - 1U) : 0U;
  uint8_t column_index =
      (column >= 1U && column <= NAV_GRID_COLUMNS) ? (uint8_t)(column - 1U) : 0U;

  if (x_mm != 0)
  {
    *x_mm = (uint16_t)(((uint32_t)g_nav_grid_x_mm[column_index] +
                       g_nav_grid_x_mm[column_index + 1U]) /
                      2U);
  }
  if (y_mm != 0)
  {
    *y_mm = (uint16_t)(((uint32_t)g_nav_grid_y_mm[row_index] +
                       g_nav_grid_y_mm[row_index + 1U]) /
                      2U);
  }
}

void GridMap_BuildMasks(const uint16_t counts[NAV_GRID_CELL_COUNT],
                        uint32_t *scanned_mask,
                        uint32_t *effective_mask)
{
  uint8_t index;
  uint32_t scanned = 0U;

  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    if (counts[index] >= NAV_OBSTACLE_POINT_THRESHOLD)
    {
      scanned |= 1UL << index;
    }
  }

  scanned &= ~NAV_PROTECTED_CELL_MASK;
  if (scanned_mask != 0)
  {
    *scanned_mask = scanned;
  }
  if (effective_mask != 0)
  {
    *effective_mask = NAV_FIXED_OBSTACLE_MASK | scanned;
  }
}

uint8_t GridMap_IsBlocked(uint32_t mask, uint8_t row, uint8_t column)
{
  uint8_t index = GridMap_CellIndex(row, column);

  if (index == 0xFFU)
  {
    return 1U;
  }
  return (mask & (1UL << index)) != 0U ? 1U : 0U;
}
