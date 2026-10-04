import assert from 'node:assert/strict';
import test from 'node:test';

import {
  classifyObstacleCells,
  createDefaultRadarConfig,
  createGrid,
} from '../radar-direct-model.mjs';
import {
  buildWaypointRows,
  formatMissionRoute,
  parseMissionRoute,
  planMissionRoute,
  planPointToPoint,
  resolveMissionPoints,
} from '../radar-direct-planner.mjs';


function defaultMap() {
  const config = createDefaultRadarConfig();
  const grid = createGrid(config.map);
  const obstacles = classifyObstacleCells(grid, [], config.map.obstaclePointThreshold, 0);
  return {config, grid, blockedMask: obstacles.blockedMask};
}

function endpoint(row, column, anchor = 'cell') {
  return {row, column, anchor};
}

test('mission route parser accepts every reference separator and normalizes output', () => {
  const parsed = parseMissionRoute('1 - 5，4, 2 3\t-\n1');

  assert.deepEqual(parsed, {ok: true, route: [1, 5, 4, 2, 3, 1], error: ''});
  assert.equal(formatMissionRoute(parsed.route), '1-5-4-2-3-1');
});

test('mission route parser rejects each invalid sequence class with a visible error', () => {
  const invalid = [
    '',
    '1-a-1',
    '1-0-1',
    '1-6-1',
    '1-5-5-1',
    '2-3-1',
    '1-3-2',
    '1',
    '1-2-3-4-5-2-3-4-5-2-3-4-5-2-3-4-1',
  ];
  for (const value of invalid) {
    const result = parseMissionRoute(value);
    assert.equal(result.ok, false, value);
    assert.ok(result.error.trim(), value);
  }
});

test('mission points use radar origin and approved protected grid centres', () => {
  const {grid} = defaultMap();

  assert.deepEqual(
    resolveMissionPoints(grid, {x: 230, y: 230}).map(point => [point.number, point.xmm, point.ymm]),
    [
      [1, 230, 230],
      [2, 350, 1200],
      [3, 1200, 2050],
      [4, 2050, 1200],
      [5, 1200, 350],
    ],
  );

  const config = createDefaultRadarConfig();
  config.map.xLines[1] = 650;
  config.map.yLines[2] = 1100;
  const moved = createGrid(config.map);
  assert.deepEqual(
    resolveMissionPoints(moved, {x: 310, y: 270}).map(point => [point.xmm, point.ymm]),
    [[310, 270], [400, 1250], [1200, 2050], [2050, 1250], [1200, 350]],
  );
});

test('point-to-point planning uses only four neighbours and reference endpoint anchors', () => {
  const {grid, blockedMask} = defaultMap();
  const plan = planPointToPoint({
    grid,
    blockedMask,
    start: endpoint(1, 1, 'radar'),
    goal: endpoint(1, 5, 'goal-zone-one'),
  });

  assert.equal(plan.success, true, plan.failureReason);
  assert.deepEqual(plan.cells.map(cell => cell.column), [1, 2, 3, 4, 5]);
  assert.ok(plan.cells.every(cell => cell.row === 1));
  assert.deepEqual(plan.waypoints[0], {xmm: 230, ymm: 230, row: 1, column: 1});
  assert.deepEqual(plan.waypoints.at(-1), {xmm: 2250, ymm: 150, row: 1, column: 5});
  assert.deepEqual(plan.directions, ['east', 'east', 'east', 'east']);
  assert.deepEqual(plan.turns, ['start', 'straight', 'straight', 'straight']);
  for (let index = 1; index < plan.cells.length; index += 1) {
    const previous = plan.cells[index - 1];
    const current = plan.cells[index];
    assert.equal(Math.abs(previous.row - current.row) + Math.abs(previous.column - current.column), 1);
  }
});

test('planner reports blocked endpoints and an unreachable barrier explicitly', () => {
  const {grid, blockedMask} = defaultMap();
  const bit = (row, column) => 1 << ((row - 1) * 5 + column - 1);
  const blockedStart = planPointToPoint({
    grid,
    blockedMask: blockedMask | bit(1, 1),
    start: endpoint(1, 1),
    goal: endpoint(1, 5),
  });
  const blockedGoal = planPointToPoint({
    grid,
    blockedMask: blockedMask | bit(1, 5),
    start: endpoint(1, 1),
    goal: endpoint(1, 5),
  });
  let barrier = blockedMask;
  for (let row = 1; row <= 5; row += 1) barrier |= bit(row, 3);
  const unreachable = planPointToPoint({
    grid,
    blockedMask: barrier,
    start: endpoint(1, 1),
    goal: endpoint(1, 5),
  });

  assert.match(blockedStart.failureReason, /start/i);
  assert.match(blockedGoal.failureReason, /goal/i);
  assert.match(unreachable.failureReason, /route/i);
  assert.deepEqual(unreachable.cells, []);
  assert.deepEqual(unreachable.waypoints, []);
});

test('turn classification covers start straight left right and reverse', () => {
  const {grid, blockedMask} = defaultMap();
  const bit = (row, column) => 1 << ((row - 1) * 5 + column - 1);
  const straight = planPointToPoint({
    grid,
    blockedMask,
    start: endpoint(1, 1),
    goal: endpoint(1, 3),
  });
  const right = planPointToPoint({
    grid,
    blockedMask,
    start: endpoint(1, 1),
    goal: endpoint(3, 2),
  });
  const left = planPointToPoint({
    grid,
    blockedMask: blockedMask | bit(2, 1),
    start: endpoint(1, 1),
    goal: endpoint(3, 3),
  });
  const reverse = planMissionRoute({grid, blockedMask, route: [1, 5, 1], radar: {x: 230, y: 230}});

  assert.deepEqual(straight.turns, ['start', 'straight']);
  assert.ok(right.turns.includes('right'));
  assert.ok(left.turns.includes('left'));
  assert.ok(reverse.turns.includes('reverse'));
});

test('default mission preserves ordered checkpoints, repeated cells, and total distance', () => {
  const {config, grid, blockedMask} = defaultMap();
  const plan = planMissionRoute({
    grid,
    blockedMask,
    route: config.missionRoute,
    radar: {x: config.map.radarXmm, y: config.map.radarYmm},
  });

  assert.equal(plan.success, true, plan.failureReason);
  assert.ok(plan.cells.length > 25);
  assert.deepEqual(plan.waypoints[0], {xmm: 230, ymm: 230, row: 1, column: 1});
  assert.deepEqual(plan.waypoints.at(-1), {xmm: 230, ymm: 230, row: 1, column: 1});
  assert.ok(plan.totalDistanceMm > 0);
  assert.equal(plan.segmentDistancesMm.reduce((sum, value) => sum + value, 0), plan.totalDistanceMm);
  assert.ok(plan.cells.some((cell, index) => (
    plan.cells.findIndex(other => other.bitIndex === cell.bitIndex) !== index
  )));
  for (let index = 1; index < plan.cells.length; index += 1) {
    assert.notEqual(plan.cells[index - 1].bitIndex, plan.cells[index].bitIndex);
  }

  const expected = [[230, 230], [1200, 350], [2050, 1200], [350, 1200], [1200, 2050],
    [2050, 1200], [350, 1200], [1200, 2050], [230, 230]];
  let searchStart = 0;
  for (const [xmm, ymm] of expected) {
    const found = plan.waypoints.findIndex((point, index) => (
      index >= searchStart && point.xmm === xmm && point.ymm === ymm
    ));
    assert.ok(found >= searchStart, `missing checkpoint ${xmm},${ymm}`);
    searchStart = found + 1;
  }
});

test('mission planner reports the first unreachable numbered leg', () => {
  const {config, grid, blockedMask} = defaultMap();
  const bit = (row, column) => 1 << ((row - 1) * 5 + column - 1);
  let barrier = blockedMask;
  for (let row = 1; row <= 5; row += 1) barrier |= bit(row, 2);

  const plan = planMissionRoute({
    grid,
    blockedMask: barrier,
    route: config.missionRoute,
    radar: {x: 230, y: 230},
  });

  assert.equal(plan.success, false);
  assert.match(plan.failureReason, /1→5/);
});

test('physical distance and waypoint table track boundary and endpoint positions', () => {
  const original = defaultMap();
  const before = planPointToPoint({
    grid: original.grid,
    blockedMask: original.blockedMask,
    start: endpoint(1, 1, 'radar'),
    goal: endpoint(1, 5, 'goal-zone-one'),
  });
  const config = createDefaultRadarConfig();
  config.map.xLines[1] = 650;
  const movedGrid = createGrid(config.map);
  const movedBlocked = classifyObstacleCells(movedGrid, [], 3, 0).blockedMask;
  const after = planPointToPoint({
    grid: movedGrid,
    blockedMask: movedBlocked,
    start: endpoint(1, 1, 'radar'),
    goal: endpoint(1, 5, 'goal-zone-one'),
  });
  const rows = buildWaypointRows(after);

  assert.notEqual(before.totalDistanceMm, after.totalDistanceMm);
  assert.equal(rows.length, after.waypoints.length);
  assert.deepEqual(rows[0], {
    sequence: 1,
    cell: 'R1C1',
    xmm: 230,
    ymm: 230,
    nextDistanceMm: Math.round(after.segmentDistancesMm[0]),
    nextDirection: 'east',
    nextTurn: 'start',
  });
  assert.equal(rows.at(-1).nextDistanceMm, null);
  assert.equal(rows.at(-1).nextDirection, null);
  assert.equal(rows.at(-1).nextTurn, null);
});
