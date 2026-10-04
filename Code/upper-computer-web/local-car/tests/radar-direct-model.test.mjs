import assert from 'node:assert/strict';
import test from 'node:test';

import {
  MISSION_PROTECTED_MASK,
  RADAR_DIRECT_DEFAULTS,
  classifyObstacleCells,
  createDefaultRadarConfig,
  createGrid,
  filterRadarPoints,
  loadRadarDirectPreferences,
  normalizeRadarConfig,
  saveRadarDirectPreferences,
  scanToCsv,
  toFieldPoint,
} from '../radar-direct-model.mjs';


test('approved direct-host defaults are reproduced exactly and cloned', () => {
  const first = createDefaultRadarConfig();
  const second = createDefaultRadarConfig();

  assert.deepEqual(first, {
    filter: {
      minAngleDeg: 0,
      maxAngleDeg: 360,
      minDistanceMm: 100,
      maxDistanceMm: 40000,
      minEnergy: 0,
      maxEnergy: 255,
    },
    map: {
      radarXmm: 230,
      radarYmm: 230,
      xLines: [150, 550, 1000, 1400, 1850, 2250],
      yLines: [150, 550, 1000, 1400, 1850, 2250],
      obstaclePointThreshold: 1,
      fixedBlockedMask: 328000,
      manualBlockedMask: 0,
    },
    planningMode: 'mission',
    missionRoute: [1, 5, 4, 2, 3, 4, 2, 3, 1],
    start: {row: 1, column: 1, anchor: 'radar'},
    goal: {row: 1, column: 5, anchor: 'goal-zone-one'},
    displayMode: 'filtered',
    theme: 'dark',
    radarPort: '',
    radarBaudrate: 500000,
  });
  assert.deepEqual(RADAR_DIRECT_DEFAULTS, first);
  first.map.xLines[1] = 600;
  assert.equal(second.map.xLines[1], 550);
});

test('filter includes boundaries, full circle, and ranges that cross zero', () => {
  const points = [
    {angle_deg: 0, distance_mm: 100, energy: 0},
    {angle_deg: 359.99, distance_mm: 40000, energy: 255},
    {angle_deg: 180, distance_mm: 1000, energy: 100},
  ];
  assert.deepEqual(filterRadarPoints(points, RADAR_DIRECT_DEFAULTS.filter), points);

  const crossed = filterRadarPoints(points, {
    ...RADAR_DIRECT_DEFAULTS.filter,
    minAngleDeg: 350,
    maxAngleDeg: 10,
  });
  assert.deepEqual(crossed, [points[0], points[1]]);

  const inclusive = filterRadarPoints([
    {angle_deg: 10, distance_mm: 100, energy: 5},
    {angle_deg: 20, distance_mm: 200, energy: 10},
    {angle_deg: 9, distance_mm: 150, energy: 7},
    {angle_deg: 15, distance_mm: 99, energy: 7},
    {angle_deg: 15, distance_mm: 150, energy: 4},
  ], {
    minAngleDeg: 10,
    maxAngleDeg: 20,
    minDistanceMm: 100,
    maxDistanceMm: 200,
    minEnergy: 5,
    maxEnergy: 10,
  });
  assert.deepEqual(inclusive, [
    {angle_deg: 10, distance_mm: 100, energy: 5},
    {angle_deg: 20, distance_mm: 200, energy: 10},
  ]);
});

test('clockwise radar coordinates match the supplied host', () => {
  assert.deepEqual(
    toFieldPoint({angle_deg: 0, distance_mm: 1000, energy: 1}, {x: 230, y: 230}),
    {x: 230, y: 1230, source: {angle_deg: 0, distance_mm: 1000, energy: 1}},
  );
  assert.deepEqual(
    toFieldPoint({angle_deg: 90, distance_mm: 1000, energy: 1}, {x: 230, y: 230}),
    {x: 1230, y: 230, source: {angle_deg: 90, distance_mm: 1000, energy: 1}},
  );
});

test('grid keeps independent axes and assigns boundaries toward positive axes', () => {
  const config = createDefaultRadarConfig();
  config.map.xLines = [150, 600, 1000, 1400, 1850, 2250];
  config.map.yLines = [150, 550, 1100, 1400, 1850, 2250];
  const grid = createGrid(config.map);

  assert.equal(grid.getCell(1, 1).maxXmm, 600);
  assert.equal(grid.getCell(1, 1).maxYmm, 550);
  assert.deepEqual(
    {row: grid.locate(600, 550).row, column: grid.locate(600, 550).column},
    {row: 2, column: 2},
  );
  assert.equal(grid.locate(150, 150).bitIndex, 0);
  assert.equal(grid.locate(2250, 2250).bitIndex, 24);
  assert.equal(grid.locate(149.999, 500), null);
  assert.equal(grid.locate(500, 2250.001), null);
});

test('configuration rejects narrow cells without mutating the input', () => {
  const value = createDefaultRadarConfig();
  value.map.xLines = [150, 960, 1000, 1400, 1850, 2250];
  const before = structuredClone(value);

  assert.throws(() => normalizeRadarConfig(value), /50/);
  assert.deepEqual(value, before);
});

test('obstacle classification preserves counts and applies fixed manual scanned and protected precedence', () => {
  const config = createDefaultRadarConfig();
  const grid = createGrid(config.map);
  const bit = (row, column) => 1 << ((row - 1) * 5 + column - 1);
  const manualMask = bit(1, 1) | bit(1, 2) | bit(1, 3) | (1 << 30);
  const points = [
    {x: 350, y: 350},
    ...[1200, 1210, 1220].map(x => ({x, y: 350})),
    ...[775, 785, 795].map(x => ({x, y: 350})),
    ...[780, 790, 800].map(x => ({x, y: 780})),
  ];

  const result = classifyObstacleCells(grid, points, 3, manualMask);
  const cell = (row, column) => result.cells[(row - 1) * 5 + column - 1];

  assert.ok((MISSION_PROTECTED_MASK & bit(1, 1)) !== 0);
  assert.equal(result.manualMask, bit(1, 2));
  assert.equal(cell(1, 1).scannedPointCount, 1);
  assert.equal(cell(1, 1).protected, true);
  assert.equal(cell(1, 1).blocked, false);
  assert.deepEqual(cell(1, 1).flags, {fixed: false, manual: false, scanned: false});
  assert.equal(cell(1, 3).scannedPointCount, 3);
  assert.equal(cell(1, 3).protected, true);
  assert.equal(cell(1, 3).blocked, false);
  assert.equal(cell(1, 3).kind, 'protected');
  assert.deepEqual(cell(1, 2).flags, {fixed: false, manual: true, scanned: true});
  assert.equal(cell(1, 2).kind, 'manual');
  assert.equal(cell(2, 2).flags.fixed, true);
  assert.equal(cell(2, 2).kind, 'fixed');
  assert.ok((result.blockedMask & bit(1, 2)) !== 0);
  assert.ok((result.blockedMask & MISSION_PROTECTED_MASK) === 0);
});

test('normalization masks manual bits and clears all protected task cells', () => {
  const value = createDefaultRadarConfig();
  value.map.manualBlockedMask = 0xffffffff;

  const normalized = normalizeRadarConfig(value);

  assert.equal(normalized.map.manualBlockedMask >>> 0, (0x01ffffff & ~MISSION_PROTECTED_MASK) >>> 0);
});

test('preference load falls back on missing corrupt or invalid saved data', () => {
  const missing = {getItem: () => null};
  assert.deepEqual(loadRadarDirectPreferences(missing), {
    config: createDefaultRadarConfig(),
    warning: null,
  });

  for (const saved of ['{not-json', JSON.stringify({map: {radarXmm: -1}})]) {
    const result = loadRadarDirectPreferences({getItem: () => saved});
    assert.deepEqual(result.config, createDefaultRadarConfig());
    assert.match(result.warning, /defaults/i);
  }
});

test('preference save reports storage errors and never mutates configuration', () => {
  const config = createDefaultRadarConfig();
  config.missionRoute = [1, 2, 1];
  const before = structuredClone(config);
  const storage = {setItem() { throw new Error('quota denied'); }};

  const result = saveRadarDirectPreferences(storage, config);

  assert.deepEqual(result, {ok: false, error: 'quota denied'});
  assert.deepEqual(config, before);
});

test('preference save and load round-trip normalized values', () => {
  const values = new Map();
  const storage = {
    getItem(key) { return values.get(key) ?? null; },
    setItem(key, value) { values.set(key, value); },
  };
  const config = createDefaultRadarConfig();
  config.theme = 'light';
  config.radarPort = 'COM12';
  config.radarBaudrate = 921600;

  assert.deepEqual(saveRadarDirectPreferences(storage, config), {ok: true, error: null});
  assert.deepEqual(loadRadarDirectPreferences(storage), {config, warning: null});
});

test('CSV uses BOM, reference column order, invariant rounded angles, and CRLF', () => {
  const csv = scanToCsv([
    {angle_deg: 0, distance_mm: 123, energy: 4},
    {angle_deg: 12.3456789, distance_mm: 456, energy: 7},
  ]);

  assert.equal(
    csv,
    '\ufeffindex,angle_deg,distance_mm,energy\r\n' +
      '0,0,123,4\r\n' +
      '1,12.345679,456,7\r\n',
  );
});
