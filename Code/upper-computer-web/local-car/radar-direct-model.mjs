const FIELD_SIZE_MM = 2400;
const ACTIVE_MINIMUM_MM = 150;
const ACTIVE_MAXIMUM_MM = 2250;
const MINIMUM_CELL_SIZE_MM = 50;
const GRID_MASK = 0x01ffffff;
const STORAGE_KEY = 'lds50c-direct-host-v1';
const SUPPORTED_BAUDRATES = new Set([384000, 500000, 768000, 921600]);

const bitFor = (row, column) => 1 << ((row - 1) * 5 + column - 1);

export const MISSION_PROTECTED_MASK = (
  bitFor(1, 1) |
  bitFor(3, 1) |
  bitFor(5, 3) |
  bitFor(3, 5) |
  bitFor(1, 3)
) >>> 0;

const DEFAULT_VALUE = {
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
  displayMode: 'stable',
  theme: 'dark',
  radarPort: '',
  radarBaudrate: 500000,
};

function deepFreeze(value) {
  Object.freeze(value);
  for (const child of Object.values(value)) {
    if (child && typeof child === 'object' && !Object.isFrozen(child)) deepFreeze(child);
  }
  return value;
}

export const RADAR_DIRECT_DEFAULTS = deepFreeze(DEFAULT_VALUE);

export function createDefaultRadarConfig() {
  return {
    filter: {...DEFAULT_VALUE.filter},
    map: {
      ...DEFAULT_VALUE.map,
      xLines: [...DEFAULT_VALUE.map.xLines],
      yLines: [...DEFAULT_VALUE.map.yLines],
    },
    planningMode: DEFAULT_VALUE.planningMode,
    missionRoute: [...DEFAULT_VALUE.missionRoute],
    start: {...DEFAULT_VALUE.start},
    goal: {...DEFAULT_VALUE.goal},
    displayMode: DEFAULT_VALUE.displayMode,
    theme: DEFAULT_VALUE.theme,
    radarPort: DEFAULT_VALUE.radarPort,
    radarBaudrate: DEFAULT_VALUE.radarBaudrate,
  };
}

function finiteNumber(value, name, minimum, maximum) {
  if (typeof value !== 'number' || !Number.isFinite(value) || value < minimum || value > maximum) {
    throw new RangeError(`${name} must be a finite number from ${minimum} to ${maximum}`);
  }
  return value;
}

function integer(value, name, minimum, maximum) {
  if (!Number.isInteger(value) || value < minimum || value > maximum) {
    throw new RangeError(`${name} must be an integer from ${minimum} to ${maximum}`);
  }
  return value;
}

function normalizeLines(value, name) {
  if (!Array.isArray(value) || value.length !== 6) {
    throw new RangeError(`${name} grid must contain exactly six boundary lines`);
  }
  const lines = value.map((item, index) => finiteNumber(item, `${name} line ${index}`, 0, FIELD_SIZE_MM));
  if (lines[0] !== ACTIVE_MINIMUM_MM || lines.at(-1) !== ACTIVE_MAXIMUM_MM) {
    throw new RangeError(`${name} outer boundaries must remain at 150 and 2250 mm`);
  }
  for (let index = 1; index < lines.length; index += 1) {
    if (lines[index] - lines[index - 1] < MINIMUM_CELL_SIZE_MM) {
      throw new RangeError(`${name} cell widths must be at least 50 mm`);
    }
  }
  return lines;
}

function normalizeEndpoint(value, fallback, name) {
  const source = value && typeof value === 'object' && !Array.isArray(value) ? value : fallback;
  const anchor = source.anchor ?? fallback.anchor;
  if (!['cell', 'radar', 'goal-zone-one'].includes(anchor)) {
    throw new RangeError(`${name} anchor is invalid`);
  }
  return {
    row: integer(source.row ?? fallback.row, `${name} row`, 1, 5),
    column: integer(source.column ?? fallback.column, `${name} column`, 1, 5),
    anchor,
  };
}

function normalizeRoute(route) {
  if (!Array.isArray(route) || route.length < 2 || route.length > 16) {
    throw new RangeError('Mission route must contain 2 to 16 points');
  }
  const result = route.map((number, index) => integer(number, `Mission point ${index + 1}`, 1, 5));
  if (result[0] !== 1 || result.at(-1) !== 1) {
    throw new RangeError('Mission route must start and end at point 1');
  }
  if (result.some((number, index) => index > 0 && number === result[index - 1])) {
    throw new RangeError('Adjacent mission points cannot be identical');
  }
  return result;
}

export function normalizeRadarConfig(value) {
  if (value === null || typeof value !== 'object' || Array.isArray(value)) {
    throw new TypeError('Radar configuration must be an object');
  }
  const defaults = createDefaultRadarConfig();
  const filterSource = value.filter && typeof value.filter === 'object' ? value.filter : defaults.filter;
  const mapSource = value.map && typeof value.map === 'object' ? value.map : defaults.map;
  const filter = {
    minAngleDeg: finiteNumber(filterSource.minAngleDeg ?? defaults.filter.minAngleDeg, 'Minimum angle', 0, 360),
    maxAngleDeg: finiteNumber(filterSource.maxAngleDeg ?? defaults.filter.maxAngleDeg, 'Maximum angle', 0, 360),
    minDistanceMm: integer(filterSource.minDistanceMm ?? defaults.filter.minDistanceMm, 'Minimum distance', 0, 65535),
    maxDistanceMm: integer(filterSource.maxDistanceMm ?? defaults.filter.maxDistanceMm, 'Maximum distance', 0, 65535),
    minEnergy: integer(filterSource.minEnergy ?? defaults.filter.minEnergy, 'Minimum energy', 0, 255),
    maxEnergy: integer(filterSource.maxEnergy ?? defaults.filter.maxEnergy, 'Maximum energy', 0, 255),
  };
  if (filter.minDistanceMm > filter.maxDistanceMm) throw new RangeError('Minimum distance cannot exceed maximum distance');
  if (filter.minEnergy > filter.maxEnergy) throw new RangeError('Minimum energy cannot exceed maximum energy');

  const rawManualMask = mapSource.manualBlockedMask ?? defaults.map.manualBlockedMask;
  if (!Number.isInteger(rawManualMask)) throw new RangeError('Manual obstacle mask must be an integer');
  const map = {
    radarXmm: finiteNumber(mapSource.radarXmm ?? defaults.map.radarXmm, 'Radar X', 0, FIELD_SIZE_MM),
    radarYmm: finiteNumber(mapSource.radarYmm ?? defaults.map.radarYmm, 'Radar Y', 0, FIELD_SIZE_MM),
    xLines: normalizeLines(mapSource.xLines ?? defaults.map.xLines, 'X'),
    yLines: normalizeLines(mapSource.yLines ?? defaults.map.yLines, 'Y'),
    obstaclePointThreshold: integer(
      mapSource.obstaclePointThreshold ?? defaults.map.obstaclePointThreshold,
      'Obstacle point threshold',
      1,
      4096,
    ),
    fixedBlockedMask: defaults.map.fixedBlockedMask,
    manualBlockedMask: ((rawManualMask >>> 0) & GRID_MASK & ~MISSION_PROTECTED_MASK) >>> 0,
  };

  const planningMode = value.planningMode ?? defaults.planningMode;
  if (!['mission', 'point-to-point'].includes(planningMode)) throw new RangeError('Unknown planning mode');
  const displayMode = value.displayMode ?? defaults.displayMode;
  if (!['raw', 'filtered', 'stable', 'preview'].includes(displayMode)) throw new RangeError('Unknown radar display mode');
  const theme = value.theme ?? defaults.theme;
  if (!['dark', 'light'].includes(theme)) throw new RangeError('Unknown radar theme');
  const radarPort = value.radarPort ?? defaults.radarPort;
  if (typeof radarPort !== 'string' || (radarPort && !/^COM[1-9]\d*$/i.test(radarPort))) {
    throw new RangeError('Saved radar port must be empty or a Windows COM port');
  }
  const radarBaudrate = value.radarBaudrate ?? defaults.radarBaudrate;
  if (!SUPPORTED_BAUDRATES.has(radarBaudrate)) throw new RangeError('Unsupported radar baudrate');

  return {
    filter,
    map,
    planningMode,
    missionRoute: normalizeRoute(value.missionRoute ?? defaults.missionRoute),
    start: normalizeEndpoint(value.start, defaults.start, 'Start'),
    goal: normalizeEndpoint(value.goal, defaults.goal, 'Goal'),
    displayMode,
    theme,
    radarPort: radarPort.toUpperCase(),
    radarBaudrate,
  };
}

function normalizeAngle(angle) {
  const normalized = angle % 360;
  return normalized < 0 ? normalized + 360 : normalized;
}

export function filterRadarPoints(points, filter) {
  if (!Array.isArray(points)) throw new TypeError('Radar points must be an array');
  const normalized = normalizeRadarConfig({filter}).filter;
  const fullCircle = normalized.minAngleDeg === 0 && normalized.maxAngleDeg === 360;
  const minimum = normalizeAngle(normalized.minAngleDeg);
  const maximum = normalizeAngle(normalized.maxAngleDeg);
  return points.filter(point => {
    const angle = normalizeAngle(point.angle_deg);
    const insideAngle = fullCircle || (
      normalized.minAngleDeg <= normalized.maxAngleDeg
        ? angle >= minimum && angle <= maximum
        : angle >= minimum || angle <= maximum
    );
    return insideAngle &&
      point.distance_mm >= normalized.minDistanceMm &&
      point.distance_mm <= normalized.maxDistanceMm &&
      point.energy >= normalized.minEnergy &&
      point.energy <= normalized.maxEnergy;
  });
}

function cleanTrig(value) {
  return Math.abs(value) < 1e-12 ? 0 : value;
}

export function toFieldPoint(point, radar) {
  const radians = point.angle_deg * Math.PI / 180;
  return {
    x: radar.x + point.distance_mm * cleanTrig(Math.sin(radians)),
    y: radar.y + point.distance_mm * cleanTrig(Math.cos(radians)),
    source: point,
  };
}

function locateAxis(lines, value) {
  if (!Number.isFinite(value) || value < lines[0] || value > lines.at(-1)) return -1;
  if (value === lines.at(-1)) return lines.length - 2;
  for (let index = 0; index < lines.length - 1; index += 1) {
    if (value < lines[index + 1]) return index;
  }
  return -1;
}

export function createGrid(mapValue) {
  const map = normalizeRadarConfig({map: mapValue}).map;
  const xLines = [...map.xLines];
  const yLines = [...map.yLines];
  const cells = [];
  for (let row = 1; row <= 5; row += 1) {
    for (let column = 1; column <= 5; column += 1) {
      const minXmm = xLines[column - 1];
      const maxXmm = xLines[column];
      const minYmm = yLines[row - 1];
      const maxYmm = yLines[row];
      cells.push({
        row,
        column,
        bitIndex: (row - 1) * 5 + column - 1,
        minXmm,
        maxXmm,
        minYmm,
        maxYmm,
        centerXmm: (minXmm + maxXmm) / 2,
        centerYmm: (minYmm + maxYmm) / 2,
      });
    }
  }
  return {
    fieldSize: FIELD_SIZE_MM,
    activeMinimum: ACTIVE_MINIMUM_MM,
    activeMaximum: ACTIVE_MAXIMUM_MM,
    radar: {x: map.radarXmm, y: map.radarYmm},
    xLines,
    yLines,
    fixedBlockedMask: map.fixedBlockedMask >>> 0,
    cells,
    getCell(row, column) {
      if (!Number.isInteger(row) || !Number.isInteger(column) || row < 1 || row > 5 || column < 1 || column > 5) {
        throw new RangeError('Grid row and column must be from 1 to 5');
      }
      return cells[(row - 1) * 5 + column - 1];
    },
    locate(xmm, ymm) {
      const column = locateAxis(xLines, xmm);
      const row = locateAxis(yLines, ymm);
      return row < 0 || column < 0 ? null : cells[row * 5 + column];
    },
  };
}

export function classifyObstacleCells(grid, fieldPoints, threshold, manualMask) {
  if (!grid || !Array.isArray(grid.cells) || grid.cells.length !== 25) throw new TypeError('A 5 x 5 grid is required');
  if (!Array.isArray(fieldPoints)) throw new TypeError('Field points must be an array');
  integer(threshold, 'Obstacle point threshold', 1, 4096);
  if (!Number.isInteger(manualMask)) throw new RangeError('Manual obstacle mask must be an integer');

  const counts = Array(25).fill(0);
  for (const point of fieldPoints) {
    const cell = grid.locate(point.x, point.y);
    if (cell) counts[cell.bitIndex] += 1;
  }
  const normalizedManual = ((manualMask >>> 0) & GRID_MASK & ~MISSION_PROTECTED_MASK) >>> 0;
  let blockedMask = 0;
  const cells = grid.cells.map(cell => {
    const bit = (1 << cell.bitIndex) >>> 0;
    const fixed = (grid.fixedBlockedMask & bit) !== 0;
    const protectedCell = !fixed && (MISSION_PROTECTED_MASK & bit) !== 0;
    const manual = !protectedCell && (normalizedManual & bit) !== 0;
    const scanned = !protectedCell && counts[cell.bitIndex] >= threshold;
    const blocked = fixed || manual || scanned;
    if (blocked) blockedMask = (blockedMask | bit) >>> 0;
    const kind = fixed ? 'fixed' : protectedCell ? 'protected' : manual ? 'manual' : scanned ? 'scanned' : 'clear';
    return {
      ...cell,
      flags: {fixed, manual, scanned},
      scannedPointCount: counts[cell.bitIndex],
      protected: protectedCell,
      blocked,
      kind,
    };
  });
  return {cells, manualMask: normalizedManual, blockedMask: blockedMask >>> 0};
}

function formatAngle(value) {
  const rounded = Math.round(value * 1_000_000) / 1_000_000;
  if (Object.is(rounded, -0)) return '0';
  return rounded.toFixed(6).replace(/\.?0+$/, '');
}

export function scanToCsv(points) {
  if (!Array.isArray(points)) throw new TypeError('Radar points must be an array');
  const rows = points.map((point, index) => (
    `${index},${formatAngle(point.angle_deg)},${point.distance_mm},${point.energy}\r\n`
  ));
  return `\ufeffindex,angle_deg,distance_mm,energy\r\n${rows.join('')}`;
}

export function loadRadarDirectPreferences(storage) {
  try {
    const saved = storage.getItem(STORAGE_KEY);
    if (saved === null) return {config: createDefaultRadarConfig(), warning: null};
    const config = normalizeRadarConfig(JSON.parse(saved));
    return {config, warning: null};
  } catch (error) {
    return {
      config: createDefaultRadarConfig(),
      warning: `Saved radar configuration could not be used; defaults were loaded. ${error.message}`,
    };
  }
}

export function saveRadarDirectPreferences(storage, config) {
  try {
    const normalized = normalizeRadarConfig(config);
    storage.setItem(STORAGE_KEY, JSON.stringify(normalized));
    return {ok: true, error: null};
  } catch (error) {
    return {ok: false, error: error.message};
  }
}
