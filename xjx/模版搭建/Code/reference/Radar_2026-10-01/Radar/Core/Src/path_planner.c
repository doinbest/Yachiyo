#include "path_planner.h"

#include "grid_map.h"
#include "nav_math.h"

#define PATH_CELL_NONE 0xFFU
#define PATH_COST_MAX  0xFFFFFFFFUL

typedef struct
{
  uint8_t cells[NAV_GRID_CELL_COUNT];
  uint8_t count;
} CellPath;

static void index_to_cell(uint8_t index, uint8_t *row, uint8_t *column)
{
  *row = (uint8_t)((index / NAV_GRID_COLUMNS) + 1U);
  *column = (uint8_t)((index % NAV_GRID_COLUMNS) + 1U);
}

static uint32_t center_manhattan(uint8_t first, uint8_t second)
{
  uint8_t first_row;
  uint8_t first_column;
  uint8_t second_row;
  uint8_t second_column;
  uint16_t first_x;
  uint16_t first_y;
  uint16_t second_x;
  uint16_t second_y;
  int32_t dx;
  int32_t dy;

  index_to_cell(first, &first_row, &first_column);
  index_to_cell(second, &second_row, &second_column);
  GridMap_CellCenter(first_row, first_column, &first_x, &first_y);
  GridMap_CellCenter(second_row, second_column, &second_x, &second_y);
  dx = (int32_t)first_x - second_x;
  dy = (int32_t)first_y - second_y;
  if (dx < 0)
  {
    dx = -dx;
  }
  if (dy < 0)
  {
    dy = -dy;
  }
  return (uint32_t)(dx + dy);
}

static uint8_t neighbor_at(uint8_t cell, uint8_t direction, uint8_t *neighbor)
{
  uint8_t row;
  uint8_t column;

  index_to_cell(cell, &row, &column);
  switch (direction)
  {
    case 0U: /* North */
      if (row >= NAV_GRID_ROWS)
      {
        return 0U;
      }
      ++row;
      break;
    case 1U: /* East */
      if (column >= NAV_GRID_COLUMNS)
      {
        return 0U;
      }
      ++column;
      break;
    case 2U: /* South */
      if (row <= 1U)
      {
        return 0U;
      }
      --row;
      break;
    default: /* West */
      if (column <= 1U)
      {
        return 0U;
      }
      --column;
      break;
  }

  *neighbor = GridMap_CellIndex(row, column);
  return 1U;
}

static uint8_t select_best_open(const uint8_t open_set[NAV_GRID_CELL_COUNT],
                                const uint32_t f_cost[NAV_GRID_CELL_COUNT],
                                const uint32_t h_cost[NAV_GRID_CELL_COUNT],
                                const uint8_t order[NAV_GRID_CELL_COUNT])
{
  uint8_t index;
  uint8_t best = PATH_CELL_NONE;

  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    if (open_set[index] == 0U)
    {
      continue;
    }
    if (best == PATH_CELL_NONE || f_cost[index] < f_cost[best] ||
        (f_cost[index] == f_cost[best] && h_cost[index] < h_cost[best]) ||
        (f_cost[index] == f_cost[best] && h_cost[index] == h_cost[best] &&
         order[index] < order[best]))
    {
      best = index;
    }
  }
  return best;
}

static uint8_t plan_leg(uint8_t start,
                        uint8_t goal,
                        uint32_t blocked_mask,
                        CellPath *path)
{
  uint32_t g_cost[NAV_GRID_CELL_COUNT];
  uint32_t h_cost[NAV_GRID_CELL_COUNT];
  uint32_t f_cost[NAV_GRID_CELL_COUNT];
  uint8_t parent[NAV_GRID_CELL_COUNT];
  uint8_t open_set[NAV_GRID_CELL_COUNT];
  uint8_t closed_set[NAV_GRID_CELL_COUNT];
  uint8_t order[NAV_GRID_CELL_COUNT];
  uint8_t reverse[NAV_GRID_CELL_COUNT];
  uint8_t next_order = 0U;
  uint8_t index;
  uint8_t current;

  path->count = 0U;
  if ((blocked_mask & (1UL << start)) != 0U ||
      (blocked_mask & (1UL << goal)) != 0U)
  {
    return 0U;
  }

  for (index = 0U; index < NAV_GRID_CELL_COUNT; ++index)
  {
    g_cost[index] = PATH_COST_MAX;
    h_cost[index] = center_manhattan(index, goal);
    f_cost[index] = PATH_COST_MAX;
    parent[index] = PATH_CELL_NONE;
    open_set[index] = 0U;
    closed_set[index] = 0U;
    order[index] = PATH_CELL_NONE;
  }

  g_cost[start] = 0U;
  f_cost[start] = h_cost[start];
  open_set[start] = 1U;
  order[start] = next_order++;

  for (;;)
  {
    uint8_t direction;

    current = select_best_open(open_set, f_cost, h_cost, order);
    if (current == PATH_CELL_NONE)
    {
      return 0U;
    }
    if (current == goal)
    {
      break;
    }

    open_set[current] = 0U;
    closed_set[current] = 1U;
    for (direction = 0U; direction < 4U; ++direction)
    {
      uint8_t neighbor;
      uint32_t tentative;

      if (neighbor_at(current, direction, &neighbor) == 0U ||
          closed_set[neighbor] != 0U ||
          (blocked_mask & (1UL << neighbor)) != 0U)
      {
        continue;
      }

      tentative = g_cost[current] + center_manhattan(current, neighbor);
      if (tentative >= g_cost[neighbor])
      {
        continue;
      }

      parent[neighbor] = current;
      g_cost[neighbor] = tentative;
      f_cost[neighbor] = tentative + h_cost[neighbor];
      if (open_set[neighbor] == 0U)
      {
        open_set[neighbor] = 1U;
        order[neighbor] = next_order++;
      }
    }
  }

  current = goal;
  while (current != PATH_CELL_NONE && path->count < NAV_GRID_CELL_COUNT)
  {
    reverse[path->count++] = current;
    if (current == start)
    {
      break;
    }
    current = parent[current];
  }
  if (path->count == 0U || reverse[path->count - 1U] != start)
  {
    path->count = 0U;
    return 0U;
  }

  for (index = 0U; index < path->count; ++index)
  {
    path->cells[index] = reverse[path->count - 1U - index];
  }
  return 1U;
}

static uint8_t direction_between(uint8_t from, uint8_t to)
{
  uint8_t from_row;
  uint8_t from_column;
  uint8_t to_row;
  uint8_t to_column;

  index_to_cell(from, &from_row, &from_column);
  index_to_cell(to, &to_row, &to_column);
  if (to_row > from_row)
  {
    return NAV_DIR_NORTH;
  }
  if (to_column > from_column)
  {
    return NAV_DIR_EAST;
  }
  if (to_row < from_row)
  {
    return NAV_DIR_SOUTH;
  }
  return NAV_DIR_WEST;
}

static uint8_t turn_between(uint8_t previous, uint8_t current)
{
  uint8_t difference = (uint8_t)((current + 4U - previous) % 4U);

  if (difference == 0U)
  {
    return NAV_TURN_STRAIGHT;
  }
  if (difference == 1U)
  {
    return NAV_TURN_RIGHT;
  }
  if (difference == 2U)
  {
    return NAV_TURN_REVERSE;
  }
  return NAV_TURN_LEFT;
}

uint8_t PathPlanner_BuildMissionPathLimited(uint32_t blocked_mask,
                                            NavigationResult *result,
                                            uint16_t capacity)
{
  uint8_t route[NAV_MAX_WAYPOINTS];
  uint16_t route_count = 0U;
  uint8_t leg;

  if (result == 0)
  {
    return 0U;
  }
  if (capacity > NAV_MAX_WAYPOINTS)
  {
    capacity = NAV_MAX_WAYPOINTS;
  }

  result->error = NAV_ERROR_NONE;
  result->failed_from_task = 0U;
  result->failed_to_task = 0U;
  result->path_count = 0U;

  for (leg = 0U; leg + 1U < NAV_MISSION_ORDER_LENGTH; ++leg)
  {
    uint8_t from_task = g_nav_mission_order[leg];
    uint8_t to_task = g_nav_mission_order[leg + 1U];
    uint8_t from_cell = GridMap_CellIndex(g_nav_mission_rows[from_task - 1U],
                                          g_nav_mission_columns[from_task - 1U]);
    uint8_t to_cell = GridMap_CellIndex(g_nav_mission_rows[to_task - 1U],
                                        g_nav_mission_columns[to_task - 1U]);
    CellPath leg_path;
    uint8_t index;
    uint8_t first = leg == 0U ? 0U : 1U;

    if (plan_leg(from_cell, to_cell, blocked_mask, &leg_path) == 0U)
    {
      result->error = NAV_ERROR_NO_PATH;
      result->failed_from_task = from_task;
      result->failed_to_task = to_task;
      return 0U;
    }

    for (index = first; index < leg_path.count; ++index)
    {
      if (route_count >= capacity)
      {
        result->error = NAV_ERROR_PATH_OVERFLOW;
        return 0U;
      }
      route[route_count++] = leg_path.cells[index];
    }
  }

  result->path_count = route_count;
  for (leg = 0U; leg < route_count; ++leg)
  {
    uint8_t row;
    uint8_t column;
    uint16_t x;
    uint16_t y;

    index_to_cell(route[leg], &row, &column);
    GridMap_CellCenter(row, column, &x, &y);
    if (leg == 0U || leg + 1U == route_count)
    {
      x = NAV_RADAR_X_MM;
      y = NAV_RADAR_Y_MM;
    }
    result->path[leg].x_mm = x;
    result->path[leg].y_mm = y;
    result->path[leg].row = row;
    result->path[leg].column = column;

    if (leg + 1U < route_count)
    {
      uint8_t next_row;
      uint8_t next_column;
      uint16_t next_x;
      uint16_t next_y;

      index_to_cell(route[leg + 1U], &next_row, &next_column);
      GridMap_CellCenter(next_row, next_column, &next_x, &next_y);
      if (leg + 2U == route_count)
      {
        next_x = NAV_RADAR_X_MM;
        next_y = NAV_RADAR_Y_MM;
      }
      result->path[leg].distance_to_next_mm =
          NavMath_DistanceMm(x, y, next_x, next_y);
      result->path[leg].direction =
          direction_between(route[leg], route[leg + 1U]);
      result->path[leg].turn = leg == 0U
                                   ? NAV_TURN_START
                                   : turn_between(result->path[leg - 1U].direction,
                                                  result->path[leg].direction);
    }
    else
    {
      result->path[leg].distance_to_next_mm = 0U;
      result->path[leg].direction = NAV_DIR_END;
      result->path[leg].turn = NAV_TURN_END;
    }
  }

  return 1U;
}

uint8_t PathPlanner_BuildMissionPath(uint32_t blocked_mask,
                                     NavigationResult *result)
{
  return PathPlanner_BuildMissionPathLimited(blocked_mask,
                                             result,
                                             NAV_MAX_WAYPOINTS);
}
