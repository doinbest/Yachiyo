#include "navigation_config.h"

const uint16_t g_nav_grid_x_mm[NAV_GRID_BOUNDARY_COUNT] = {
    150U, 550U, 1000U, 1400U, 1850U, 2250U};

const uint16_t g_nav_grid_y_mm[NAV_GRID_BOUNDARY_COUNT] = {
    150U, 550U, 1000U, 1400U, 1850U, 2250U};

/* Task numbers 1..5 map to array indexes 0..4. */
const uint8_t g_nav_mission_rows[NAV_MISSION_POINT_COUNT] = {
    1U, 3U, 5U, 3U, 1U};

const uint8_t g_nav_mission_columns[NAV_MISSION_POINT_COUNT] = {
    1U, 1U, 3U, 5U, 3U};

const uint8_t g_nav_mission_order[NAV_MISSION_ORDER_LENGTH] = {
    1U, 5U, 4U, 2U, 3U, 4U, 2U, 3U, 1U};
