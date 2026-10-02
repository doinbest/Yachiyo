#ifndef GRID_MAP_H
#define GRID_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "navigation_config.h"

uint8_t GridMap_FindCell(int32_t x_mm,
                         int32_t y_mm,
                         uint8_t *row,
                         uint8_t *column);
uint8_t GridMap_CellIndex(uint8_t row, uint8_t column);
void GridMap_CellCenter(uint8_t row,
                        uint8_t column,
                        uint16_t *x_mm,
                        uint16_t *y_mm);
void GridMap_BuildMasks(const uint16_t counts[NAV_GRID_CELL_COUNT],
                        uint32_t *scanned_mask,
                        uint32_t *effective_mask);
uint8_t GridMap_IsBlocked(uint32_t mask, uint8_t row, uint8_t column);

#ifdef __cplusplus
}
#endif

#endif
