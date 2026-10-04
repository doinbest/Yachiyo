import {MISSION_PROTECTED_MASK} from './radar-direct-model.mjs';


const NEIGHBOURS = [
  [1, 0],
  [0, 1],
  [-1, 0],
  [0, -1],
];
const DIRECTIONS = ['north', 'east', 'south', 'west'];

function validateMissionRoute(route) {
  if (!Array.isArray(route) || route.length < 2 || route.length > 16) {
    return '任务路线必须包含 2～16 个编号。';
  }
  if (route.some(number => !Number.isInteger(number) || number < 1 || number > 5)) {
    return '任务点编号只能是 1～5。';
  }
  if (route[0] !== 1 || route.at(-1) !== 1) {
    return '任务路线必须从 1 出发并回到 1。';
  }
  if (route.some((number, index) => index > 0 && number === route[index - 1])) {
    return '相邻任务点不能使用相同编号。';
  }
  return '';
}

export function parseMissionRoute(text) {
  if (typeof text !== 'string' || !text.trim()) {
    return {ok: false, route: [], error: '请输入任务路线。'};
  }
  const tokens = text.split(/[-,，\s]+/u).filter(Boolean);
  const route = [];
  for (const token of tokens) {
    if (!/^\d+$/u.test(token)) {
      return {ok: false, route: [], error: `无法识别任务点编号“${token}”。`};
    }
    route.push(Number(token));
  }
  const error = validateMissionRoute(route);
  return error ? {ok: false, route: [], error} : {ok: true, route, error: ''};
}

export function formatMissionRoute(route) {
  const error = validateMissionRoute(route);
  if (error) throw new RangeError(error);
  return route.join('-');
}

function missionEndpoint(number) {
  if (number === 1) return {row: 1, column: 1, anchor: 'radar'};
  if (number === 2) return {row: 3, column: 1, anchor: 'cell'};
  if (number === 3) return {row: 5, column: 3, anchor: 'cell'};
  if (number === 4) return {row: 3, column: 5, anchor: 'cell'};
  if (number === 5) return {row: 1, column: 3, anchor: 'cell'};
  throw new RangeError('任务点编号只能是 1～5。');
}

export function resolveMissionPoints(grid, radar) {
  return [1, 2, 3, 4, 5].map(number => {
    const endpoint = missionEndpoint(number);
    if (number === 1) {
      return {number, endpoint, xmm: radar.x, ymm: radar.y};
    }
    const cell = grid.getCell(endpoint.row, endpoint.column);
    return {
      number,
      endpoint,
      xmm: cell.centerXmm,
      ymm: cell.centerYmm,
    };
  });
}

function failedPlan(reason) {
  return {
    success: false,
    failureReason: reason,
    cells: [],
    waypoints: [],
    segmentDistancesMm: [],
    directions: [],
    turns: [],
    totalDistanceMm: 0,
  };
}

function isCell(endpoint) {
  return endpoint &&
    Number.isInteger(endpoint.row) && endpoint.row >= 1 && endpoint.row <= 5 &&
    Number.isInteger(endpoint.column) && endpoint.column >= 1 && endpoint.column <= 5;
}

function bitFor(cell) {
  return (1 << cell.bitIndex) >>> 0;
}

function distanceBetweenCentres(left, right) {
  return Math.abs(left.centerXmm - right.centerXmm) + Math.abs(left.centerYmm - right.centerYmm);
}

function directionBetween(from, to) {
  if (to.row === from.row + 1 && to.column === from.column) return 'north';
  if (to.row === from.row && to.column === from.column + 1) return 'east';
  if (to.row === from.row - 1 && to.column === from.column) return 'south';
  if (to.row === from.row && to.column === from.column - 1) return 'west';
  throw new Error('Path cells must be four-neighbour adjacent');
}

function turnBetween(previous, current) {
  const delta = (DIRECTIONS.indexOf(current) - DIRECTIONS.indexOf(previous) + 4) % 4;
  return ['straight', 'right', 'reverse', 'left'][delta];
}

function buildPlan(cells, waypoints) {
  if (!cells.length || cells.length !== waypoints.length) {
    throw new Error('A successful route needs one waypoint for every cell');
  }
  const segmentDistancesMm = [];
  const directions = [];
  const turns = [];
  for (let index = 0; index + 1 < waypoints.length; index += 1) {
    const deltaX = waypoints[index + 1].xmm - waypoints[index].xmm;
    const deltaY = waypoints[index + 1].ymm - waypoints[index].ymm;
    segmentDistancesMm.push(Math.hypot(deltaX, deltaY));
    const direction = directionBetween(cells[index], cells[index + 1]);
    directions.push(direction);
    turns.push(index === 0 ? 'start' : turnBetween(directions[index - 1], direction));
  }
  return {
    success: true,
    failureReason: '',
    cells,
    waypoints,
    segmentDistancesMm,
    directions,
    turns,
    totalDistanceMm: segmentDistancesMm.reduce((sum, distance) => sum + distance, 0),
  };
}

function resolveEndpoint(endpoint, cell, radar) {
  if (endpoint.anchor === 'radar') {
    return {xmm: radar.x, ymm: radar.y, row: cell.row, column: cell.column};
  }
  if (endpoint.anchor === 'goal-zone-one') {
    return {xmm: 2250, ymm: 150, row: cell.row, column: cell.column};
  }
  return {
    xmm: cell.centerXmm,
    ymm: cell.centerYmm,
    row: cell.row,
    column: cell.column,
  };
}

function popLowest(open) {
  let selected = 0;
  for (let index = 1; index < open.length; index += 1) {
    if (open[index].priority < open[selected].priority || (
      open[index].priority === open[selected].priority && open[index].sequence < open[selected].sequence
    )) selected = index;
  }
  return open.splice(selected, 1)[0];
}

export function planPointToPoint({grid, blockedMask, start, goal, radar = grid?.radar}) {
  if (!grid || !Array.isArray(grid.cells) || grid.cells.length !== 25) return failedPlan('A 5 x 5 grid is required.');
  if (!isCell(start)) return failedPlan('Start cell is outside the 5 x 5 map.');
  if (!isCell(goal)) return failedPlan('Goal cell is outside the 5 x 5 map.');
  if (!radar || !Number.isFinite(radar.x) || !Number.isFinite(radar.y)) {
    return failedPlan('Radar coordinates must be finite.');
  }

  const startCell = grid.getCell(start.row, start.column);
  const goalCell = grid.getCell(goal.row, goal.column);
  const mask = blockedMask >>> 0;
  if ((mask & bitFor(startCell)) !== 0) return failedPlan('The start cell is blocked.');
  if ((mask & bitFor(goalCell)) !== 0) return failedPlan('The goal cell is blocked.');

  const startIndex = startCell.bitIndex;
  const goalIndex = goalCell.bitIndex;
  const cameFrom = Array(25).fill(-1);
  const costs = Array(25).fill(Infinity);
  const closed = Array(25).fill(false);
  const open = [];
  let sequence = 0;
  costs[startIndex] = 0;
  open.push({
    index: startIndex,
    priority: distanceBetweenCentres(startCell, goalCell),
    sequence: sequence++,
  });

  while (open.length) {
    const currentIndex = popLowest(open).index;
    if (closed[currentIndex]) continue;
    if (currentIndex === goalIndex) {
      const cells = [];
      let cursor = currentIndex;
      while (cursor >= 0) {
        cells.push(grid.cells[cursor]);
        cursor = cameFrom[cursor];
      }
      cells.reverse();
      const waypoints = cells.map(cell => resolveEndpoint({anchor: 'cell'}, cell, radar));
      waypoints[0] = resolveEndpoint(start, cells[0], radar);
      waypoints[waypoints.length - 1] = resolveEndpoint(goal, cells.at(-1), radar);
      return buildPlan(cells, waypoints);
    }

    closed[currentIndex] = true;
    const current = grid.cells[currentIndex];
    for (const [rowDelta, columnDelta] of NEIGHBOURS) {
      const row = current.row + rowDelta;
      const column = current.column + columnDelta;
      if (row < 1 || row > 5 || column < 1 || column > 5) continue;
      const neighbour = grid.getCell(row, column);
      if ((mask & bitFor(neighbour)) !== 0 || closed[neighbour.bitIndex]) continue;
      const cost = costs[currentIndex] + distanceBetweenCentres(current, neighbour);
      if (cost >= costs[neighbour.bitIndex]) continue;
      cameFrom[neighbour.bitIndex] = currentIndex;
      costs[neighbour.bitIndex] = cost;
      open.push({
        index: neighbour.bitIndex,
        priority: cost + distanceBetweenCentres(neighbour, goalCell),
        sequence: sequence++,
      });
    }
  }
  return failedPlan('No route exists between the selected start and goal cells.');
}

export function planMissionRoute({grid, blockedMask, route, radar}) {
  const error = validateMissionRoute(route);
  if (error) return failedPlan(error);
  const byNumber = new Map(resolveMissionPoints(grid, radar).map(point => [point.number, point]));
  const points = route.map(number => byNumber.get(number));
  const effectiveMask = ((blockedMask >>> 0) & ~MISSION_PROTECTED_MASK) >>> 0;
  const cells = [];
  const waypoints = [];

  for (let index = 0; index + 1 < points.length; index += 1) {
    const from = points[index];
    const to = points[index + 1];
    const leg = planPointToPoint({
      grid,
      blockedMask: effectiveMask,
      start: from.endpoint,
      goal: to.endpoint,
      radar,
    });
    if (!leg.success) return failedPlan(`任务段 ${from.number}→${to.number} 无可用路径。`);
    const skip = index === 0 ? 0 : 1;
    cells.push(...leg.cells.slice(skip));
    waypoints.push(...leg.waypoints.slice(skip));
  }
  return buildPlan(cells, waypoints);
}

export function buildWaypointRows(plan) {
  if (!plan?.success) return [];
  return plan.waypoints.map((waypoint, index) => ({
    sequence: index + 1,
    cell: `R${waypoint.row}C${waypoint.column}`,
    xmm: Math.round(waypoint.xmm),
    ymm: Math.round(waypoint.ymm),
    nextDistanceMm: index < plan.segmentDistancesMm.length
      ? Math.round(plan.segmentDistancesMm[index])
      : null,
    nextDirection: plan.directions[index] ?? null,
    nextTurn: plan.turns[index] ?? null,
  }));
}
