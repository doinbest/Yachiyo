const MINIMUM_VIEWPORT_HALF_SPAN = 300;
const MAXIMUM_VIEWPORT_HALF_SPAN = 40000;

const PALETTE = {
  dark: {
    background: '#061019', field: '#0b1a27', grid: '#4d6575', outer: '#8ea3ae',
    text: '#b9cbd3', point: '#83f7cb', fixed: 'rgba(250, 204, 21, .50)',
    manual: 'rgba(249, 115, 22, .55)', scanned: 'rgba(239, 68, 68, .50)',
    protected: 'rgba(34, 197, 94, .15)', route: '#38bdf8', radar: '#f8fafc',
    start: '#22c55e', goal: '#e879f9', mission: '#f97316', missionText: '#ffffff',
    missionOutline: '#ffedd5', cursor: '#fde047', cursorPanel: 'rgba(0, 0, 0, .68)',
  },
  light: {
    background: '#edf4f5', field: '#ffffff', grid: '#8ca2aa', outer: '#334155',
    text: '#1f3440', point: '#087f74', fixed: 'rgba(250, 204, 21, .55)',
    manual: 'rgba(249, 115, 22, .50)', scanned: 'rgba(239, 68, 68, .42)',
    protected: 'rgba(34, 197, 94, .12)', route: '#0369a1', radar: '#0f172a',
    start: '#15803d', goal: '#a21caf', mission: '#c2410c', missionText: '#ffffff',
    missionOutline: '#7c2d12', cursor: '#854d0e', cursorPanel: 'rgba(255, 255, 255, .86)',
  },
};

export function createDefaultViewport(fieldSize) {
  return {centerX: 0, centerY: 0, halfSpan: fieldSize};
}

export function zoomViewport(viewport, anchor, deltaY) {
  const requestedHalfSpan = viewport.halfSpan * (deltaY < 0 ? 0.8 : 1.25);
  const halfSpan = Math.max(
    MINIMUM_VIEWPORT_HALF_SPAN,
    Math.min(MAXIMUM_VIEWPORT_HALF_SPAN, requestedHalfSpan),
  );
  const ratio = halfSpan / viewport.halfSpan;
  return {
    centerX: anchor.x + (viewport.centerX - anchor.x) * ratio,
    centerY: anchor.y + (viewport.centerY - anchor.y) * ratio,
    halfSpan,
  };
}

export function panViewport(viewport, startPointer, currentPointer, canvasSize) {
  const millimetresPerPixel = viewport.halfSpan * 2 / canvasSize;
  return {
    centerX: viewport.centerX - (currentPointer.x - startPointer.x) * millimetresPerPixel,
    centerY: viewport.centerY + (currentPointer.y - startPointer.y) * millimetresPerPixel,
    halfSpan: viewport.halfSpan,
  };
}

export function isPointerDrag(start, current, threshold = 8) {
  return Math.hypot(current.x - start.x, current.y - start.y) >= threshold;
}

export function createCursorReading(point, radar) {
  const deltaX = point.x - radar.x;
  const deltaY = point.y - radar.y;
  let angle = Math.atan2(deltaX, deltaY) * 180 / Math.PI;
  if (angle < 0) angle += 360;
  if (Math.abs(deltaX) < 1e-9 && Math.abs(deltaY) < 1e-9) angle = 0;
  return {x: point.x, y: point.y, angle};
}

export function createMissionMarkers(points) {
  return (points || []).map(point => ({
    number: Number(point.number),
    label: String(point.number),
    row: Number(point.row ?? point.endpoint?.row),
    column: Number(point.column ?? point.endpoint?.column),
    x: Number(point.x ?? point.xmm),
    y: Number(point.y ?? point.ymm),
  }));
}

export function createWorldTransform(fieldSize, left, top, size, viewport = null) {
  const view = viewport || createDefaultViewport(fieldSize);
  const minX = view.centerX - view.halfSpan;
  const maxX = view.centerX + view.halfSpan;
  const minY = view.centerY - view.halfSpan;
  const maxY = view.centerY + view.halfSpan;
  const viewSpan = view.halfSpan * 2;
  return {
    viewMinimum: minX,
    viewMaximum: maxX,
    minX,
    maxX,
    minY,
    maxY,
    toX: value => left + (value - minX) / viewSpan * size,
    toY: value => top + (maxY - value) / viewSpan * size,
    fromX: value => minX + (value - left) / size * viewSpan,
    fromY: value => maxY - (value - top) / size * viewSpan,
  };
}

export function isPointVisible(point, viewportOrFieldSize) {
  let bounds = viewportOrFieldSize;
  if (typeof viewportOrFieldSize === 'number') {
    bounds = createWorldTransform(viewportOrFieldSize, 0, 0, 1);
  } else if (viewportOrFieldSize && Number.isFinite(viewportOrFieldSize.halfSpan)) {
    bounds = {
      minX: viewportOrFieldSize.centerX - viewportOrFieldSize.halfSpan,
      maxX: viewportOrFieldSize.centerX + viewportOrFieldSize.halfSpan,
      minY: viewportOrFieldSize.centerY - viewportOrFieldSize.halfSpan,
      maxY: viewportOrFieldSize.centerY + viewportOrFieldSize.halfSpan,
    };
  }
  return point.x >= bounds.minX && point.x <= bounds.maxX &&
    point.y >= bounds.minY && point.y <= bounds.maxY;
}

export function isInsideField(point, fieldSize) {
  return point.x >= 0 && point.x <= fieldSize && point.y >= 0 && point.y <= fieldSize;
}

function resizeCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.max(1, globalThis.devicePixelRatio || 1);
  const width = Math.max(320, Math.round(rect.width * dpr));
  const height = Math.max(320, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return {width, height, dpr};
}

function chooseGridStep(viewSpan) {
  const rawStep = viewSpan / 8;
  const magnitude = 10 ** Math.floor(Math.log10(rawStep));
  const normalized = rawStep / magnitude;
  const nice = normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10;
  return nice * magnitude;
}

function firstGridLine(minimum, step) {
  return Math.ceil(minimum / step) * step;
}

function normalizedCell(cell) {
  const rawFlags = cell.flags ?? 0;
  const flags = typeof rawFlags === 'number'
    ? {fixed: Boolean(rawFlags & 1), manual: Boolean(rawFlags & 2), scanned: Boolean(rawFlags & 4)}
    : rawFlags;
  return {
    ...cell,
    minX: cell.minX ?? cell.minXmm,
    maxX: cell.maxX ?? cell.maxXmm,
    minY: cell.minY ?? cell.minYmm,
    maxY: cell.maxY ?? cell.maxYmm,
    flags,
    count: cell.count ?? cell.scannedPointCount ?? 0,
    protected: cell.protected ?? cell.isProtected ?? false,
  };
}

function drawMillimetreGrid(ctx, layout, colors) {
  const {dpr, left, top, size, transform} = layout;
  const {toX, toY} = transform;
  const step = chooseGridStep(transform.maxX - transform.minX);
  ctx.save();
  ctx.strokeStyle = colors.grid;
  ctx.fillStyle = colors.text;
  for (let mm = firstGridLine(transform.minX, step); mm <= transform.maxX; mm += step) {
    ctx.globalAlpha = mm === 0 ? 0.75 : 0.18;
    ctx.lineWidth = mm === 0 ? 2 * dpr : dpr;
    ctx.beginPath(); ctx.moveTo(toX(mm), top); ctx.lineTo(toX(mm), top + size); ctx.stroke();
  }
  for (let mm = firstGridLine(transform.minY, step); mm <= transform.maxY; mm += step) {
    ctx.globalAlpha = mm === 0 ? 0.75 : 0.18;
    ctx.lineWidth = mm === 0 ? 2 * dpr : dpr;
    ctx.beginPath(); ctx.moveTo(left, toY(mm)); ctx.lineTo(left + size, toY(mm)); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  ctx.font = `${11 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
  ctx.textAlign = 'center';
  for (let mm = firstGridLine(transform.minX, step); mm <= transform.maxX; mm += step) {
    ctx.fillText(String(mm), toX(mm), top + size + 22 * dpr);
  }
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let mm = firstGridLine(transform.minY, step); mm <= transform.maxY; mm += step) {
    ctx.fillText(String(mm), left - 10 * dpr, toY(mm));
  }
  ctx.restore();
}

function drawCells(ctx, model, colors, transform, dpr) {
  const {toX, toY} = transform;
  for (const rawCell of model.cells || []) {
    const cell = normalizedCell(rawCell);
    let fill = null;
    if (cell.flags.fixed) fill = colors.fixed;
    else if (cell.protected) fill = colors.protected;
    else if (cell.flags.manual) fill = colors.manual;
    else if (cell.flags.scanned) fill = colors.scanned;
    if (fill) {
      ctx.fillStyle = fill;
      ctx.fillRect(
        toX(cell.minX),
        toY(cell.maxY),
        toX(cell.maxX) - toX(cell.minX),
        toY(cell.minY) - toY(cell.maxY),
      );
    }
    const centerX = (toX(cell.minX) + toX(cell.maxX)) / 2;
    const centerY = (toY(cell.minY) + toY(cell.maxY)) / 2;
    ctx.fillStyle = colors.text;
    ctx.globalAlpha = 0.72;
    ctx.font = `${10 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
    ctx.textAlign = 'center';
    ctx.fillText(`R${cell.row}C${cell.column}${cell.count ? ` · ${cell.count}` : ''}`, centerX, centerY);
    ctx.globalAlpha = 1;
  }
}

function drawBoundaryLines(ctx, model, colors, transform, dpr) {
  const {toX, toY} = transform;
  ctx.save();
  ctx.strokeStyle = colors.outer;
  ctx.lineWidth = 2 * dpr;
  ctx.setLineDash([7 * dpr, 5 * dpr]);
  for (const x of model.xLines) {
    ctx.beginPath(); ctx.moveTo(toX(x), toY(model.activeMin)); ctx.lineTo(toX(x), toY(model.activeMax)); ctx.stroke();
  }
  for (const y of model.yLines) {
    ctx.beginPath(); ctx.moveTo(toX(model.activeMin), toY(y)); ctx.lineTo(toX(model.activeMax), toY(y)); ctx.stroke();
  }
  ctx.setLineDash([]);
  ctx.fillStyle = colors.text;
  ctx.font = `${11 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
  ctx.textAlign = 'center';
  for (let index = 0; index < 5; index += 1) {
    const width = Math.round(model.xLines[index + 1] - model.xLines[index]);
    ctx.fillText(String(width), (toX(model.xLines[index]) + toX(model.xLines[index + 1])) / 2, toY(model.activeMax) - 9 * dpr);
  }
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  for (let index = 0; index < 5; index += 1) {
    const height = Math.round(model.yLines[index + 1] - model.yLines[index]);
    ctx.fillText(String(height), toX(model.activeMax) + 8 * dpr, (toY(model.yLines[index]) + toY(model.yLines[index + 1])) / 2);
  }
  ctx.restore();
}

function drawPointCloud(ctx, model, colors, transform, dpr) {
  ctx.save();
  ctx.fillStyle = colors.point;
  for (const point of model.points || []) {
    if (!isPointVisible(point, transform)) continue;
    const energy = point.energy ?? point.source?.energy ?? 0;
    ctx.globalAlpha = 0.35 + Math.min(1, energy / 255) * 0.65;
    ctx.beginPath();
    ctx.arc(transform.toX(point.x), transform.toY(point.y), Math.max(1.25, 1.65 * dpr), 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();
}

function normalizedCoordinate(point) {
  return {x: point.x ?? point.xmm, y: point.y ?? point.ymm};
}

function drawRoute(ctx, path, colors, transform, dpr) {
  if (!path || path.length < 2) return;
  const points = path.map(normalizedCoordinate);
  ctx.save();
  ctx.strokeStyle = colors.route;
  ctx.lineWidth = 4 * dpr;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.shadowBlur = 9 * dpr;
  ctx.shadowColor = colors.route;
  ctx.beginPath();
  ctx.moveTo(transform.toX(points[0].x), transform.toY(points[0].y));
  for (const point of points.slice(1)) ctx.lineTo(transform.toX(point.x), transform.toY(point.y));
  ctx.stroke();
  ctx.shadowBlur = 0;
  for (const point of points) {
    ctx.fillStyle = colors.route;
    ctx.beginPath(); ctx.arc(transform.toX(point.x), transform.toY(point.y), 4 * dpr, 0, Math.PI * 2); ctx.fill();
  }
  ctx.restore();
}

function drawMissionMarkers(ctx, points, colors, transform, dpr) {
  ctx.save();
  for (const marker of createMissionMarkers(points)) {
    if (!isPointVisible(marker, transform)) continue;
    const sourceX = transform.toX(marker.x);
    const sourceY = transform.toY(marker.y);
    const x = sourceX + (marker.number === 1 ? 18 * dpr : 0);
    const y = sourceY + (marker.number === 1 ? -18 * dpr : 0);
    if (marker.number === 1) {
      ctx.strokeStyle = colors.missionOutline;
      ctx.lineWidth = 1.5 * dpr;
      ctx.beginPath(); ctx.moveTo(sourceX, sourceY); ctx.lineTo(x, y); ctx.stroke();
    }
    ctx.fillStyle = colors.mission;
    ctx.strokeStyle = colors.missionOutline;
    ctx.lineWidth = 2 * dpr;
    ctx.shadowBlur = 7 * dpr;
    ctx.shadowColor = colors.mission;
    ctx.beginPath(); ctx.arc(x, y, 11 * dpr, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.fillStyle = colors.missionText;
    ctx.font = `800 ${11 * dpr}px system-ui, sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(marker.label, x, y + 0.5 * dpr);
  }
  ctx.restore();
}

function drawEndpoint(ctx, endpoint, label, color, transform, dpr) {
  if (!endpoint) return;
  const point = normalizedCoordinate(endpoint);
  const x = transform.toX(point.x);
  const y = transform.toY(point.y);
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(x, y, 11 * dpr, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = '#ffffff';
  ctx.font = `700 ${11 * dpr}px system-ui, sans-serif`;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, x, y + 0.5 * dpr);
  ctx.restore();
}

function drawRadar(ctx, radar, colors, transform, dpr) {
  const x = transform.toX(radar.x);
  const y = transform.toY(radar.y);
  ctx.save();
  ctx.strokeStyle = '#22d3ee';
  ctx.fillStyle = colors.radar;
  ctx.lineWidth = 3 * dpr;
  ctx.shadowBlur = 12 * dpr;
  ctx.shadowColor = '#22d3ee';
  ctx.beginPath(); ctx.arc(x, y, 10 * dpr, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
  ctx.shadowBlur = 0;
  ctx.beginPath(); ctx.moveTo(x, y - 13 * dpr); ctx.lineTo(x - 5 * dpr, y - 4 * dpr); ctx.lineTo(x + 5 * dpr, y - 4 * dpr); ctx.closePath(); ctx.fillStyle = '#22d3ee'; ctx.fill();
  ctx.fillStyle = colors.text;
  ctx.font = `${11 * dpr}px system-ui, sans-serif`;
  ctx.textAlign = 'left';
  ctx.fillText(`雷达 ${Math.round(radar.x)}, ${Math.round(radar.y)}`, x + 14 * dpr, y + 18 * dpr);
  ctx.restore();
}

function drawCompass(ctx, colors, layout) {
  const {dpr, left, top, size} = layout;
  const x = left + size - 45 * dpr;
  const y = top + 40 * dpr;
  ctx.save();
  ctx.strokeStyle = colors.outer;
  ctx.fillStyle = colors.text;
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + 24 * dpr, y); ctx.moveTo(x, y); ctx.lineTo(x, y - 24 * dpr); ctx.stroke();
  ctx.font = `700 ${11 * dpr}px system-ui, sans-serif`;
  ctx.fillText('X+', x + 27 * dpr, y + 3 * dpr);
  ctx.fillText('Y+', x - 8 * dpr, y - 29 * dpr);
  ctx.restore();
}

function drawCursorReading(ctx, state, colors, layout) {
  if (!state.cursorWorld) return;
  const reading = createCursorReading(state.cursorWorld, state.model.radar);
  const {dpr, left, top} = layout;
  const lines = [
    `X: ${reading.x.toFixed(1)} mm`,
    `Y: ${reading.y.toFixed(1)} mm`,
    `angle: ${reading.angle.toFixed(2)}°`,
  ];
  const padding = 9 * dpr;
  const lineHeight = 20 * dpr;
  ctx.save();
  ctx.fillStyle = colors.cursorPanel;
  ctx.fillRect(left + 8 * dpr, top + 8 * dpr, 174 * dpr, padding * 2 + lineHeight * lines.length);
  ctx.fillStyle = colors.cursor;
  ctx.font = `700 ${14 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  lines.forEach((line, index) => ctx.fillText(line, left + (17 * dpr), top + (17 * dpr) + index * lineHeight));
  ctx.restore();
}

function draw(canvas, state) {
  if (!state.model) return;
  const {width, height, dpr} = resizeCanvas(canvas);
  const ctx = canvas.getContext('2d');
  const colors = PALETTE[state.theme] || PALETTE.dark;
  const pad = 46 * dpr;
  const size = Math.max(1, Math.min(width, height) - pad * 2);
  const left = (width - size) / 2;
  const top = (height - size) / 2;
  state.viewport ||= createDefaultViewport(state.model.fieldSize);
  const transform = createWorldTransform(state.model.fieldSize, left, top, size, state.viewport);
  const layout = {width, height, dpr, pad, size, left, top, transform};
  state.layout = layout;

  const fieldLeft = transform.toX(0);
  const fieldTop = transform.toY(state.model.fieldSize);
  const fieldSize = transform.toX(state.model.fieldSize) - fieldLeft;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = colors.background;
  ctx.fillRect(0, 0, width, height);
  ctx.fillStyle = colors.field;
  ctx.fillRect(fieldLeft, fieldTop, fieldSize, fieldSize);
  drawMillimetreGrid(ctx, layout, colors);
  drawCells(ctx, state.model, colors, transform, dpr);
  drawBoundaryLines(ctx, state.model, colors, transform, dpr);
  drawPointCloud(ctx, state.model, colors, transform, dpr);
  drawRoute(ctx, state.model.path, colors, transform, dpr);
  drawEndpoint(ctx, state.model.start, 'S', colors.start, transform, dpr);
  drawEndpoint(ctx, state.model.goal, 'G', colors.goal, transform, dpr);
  drawMissionMarkers(ctx, state.model.missionPoints, colors, transform, dpr);
  drawRadar(ctx, state.model.radar, colors, transform, dpr);
  ctx.strokeStyle = colors.outer;
  ctx.lineWidth = 2 * dpr;
  ctx.strokeRect(fieldLeft, fieldTop, fieldSize, fieldSize);
  drawCompass(ctx, colors, layout);
  drawCursorReading(ctx, state, colors, layout);
}

function eventPoint(canvas, event, layout) {
  const rect = canvas.getBoundingClientRect();
  return {
    x: (event.clientX - rect.left) * layout.width / rect.width,
    y: (event.clientY - rect.top) * layout.height / rect.height,
  };
}

function findBand(lines, value) {
  for (let index = 0; index < lines.length - 1; index += 1) {
    if (value >= lines[index] && (value < lines[index + 1] || (
      index === lines.length - 2 && value <= lines[index + 1]
    ))) return index;
  }
  return -1;
}

function hitBoundary(state, point) {
  const {model, layout} = state;
  const tolerance = 10 * layout.dpr;
  const yMin = layout.transform.toY(model.activeMax);
  const yMax = layout.transform.toY(model.activeMin);
  for (let index = 0; index < 4; index += 1) {
    const x = layout.transform.toX(model.xLines[index + 1]);
    if (Math.abs(point.x - x) <= tolerance && point.y >= yMin && point.y <= yMax) {
      return {axis: 'X', index};
    }
  }
  const xMin = layout.transform.toX(model.activeMin);
  const xMax = layout.transform.toX(model.activeMax);
  for (let index = 0; index < 4; index += 1) {
    const y = layout.transform.toY(model.yLines[index + 1]);
    if (Math.abs(point.y - y) <= tolerance && point.x >= xMin && point.x <= xMax) {
      return {axis: 'Y', index};
    }
  }
  return null;
}

export function mountRadarDirectMap({
  canvas,
  onBoundaryDragged = () => {},
  onCellClicked = () => {},
  ResizeObserverClass = globalThis.ResizeObserver,
}) {
  const state = {
    model: null,
    theme: 'dark',
    layout: null,
    viewport: null,
    cursorWorld: null,
    drag: null,
    pan: null,
    pointerDown: null,
    resizeObserver: null,
  };

  const onPointerDown = event => {
    if (!state.model || !state.layout || event.button !== 0) return;
    const point = eventPoint(canvas, event, state.layout);
    state.pointerDown = point;
    state.drag = hitBoundary(state, point);
    state.pan = state.drag ? null : {
      start: point,
      viewport: {...(state.viewport || createDefaultViewport(state.model.fieldSize))},
    };
    canvas.setPointerCapture?.(event.pointerId);
    event.preventDefault();
  };

  const onPointerMove = event => {
    if (!state.model || !state.layout) return;
    const point = eventPoint(canvas, event, state.layout);
    const panning = state.pan && state.pointerDown &&
      isPointerDrag(state.pointerDown, point, 8 * state.layout.dpr);
    if (panning) state.viewport = panViewport(state.pan.viewport, state.pan.start, point, state.layout.size);
    const transform = createWorldTransform(
      state.model.fieldSize,
      state.layout.left,
      state.layout.top,
      state.layout.size,
      state.viewport || createDefaultViewport(state.model.fieldSize),
    );
    state.cursorWorld = {x: transform.fromX(point.x), y: transform.fromY(point.y)};
    const hit = state.drag || (!state.pointerDown ? hitBoundary(state, point) : null);
    canvas.style.cursor = hit ? (hit.axis === 'X' ? 'ew-resize' : 'ns-resize') : (panning ? 'grabbing' : 'grab');
    draw(canvas, state);
  };

  const onWheel = event => {
    if (!state.model || !state.layout || event.deltaY === 0) return;
    event.preventDefault();
    const point = eventPoint(canvas, event, state.layout);
    const anchor = {
      x: state.layout.transform.fromX(point.x),
      y: state.layout.transform.fromY(point.y),
    };
    state.viewport = zoomViewport(
      state.viewport || createDefaultViewport(state.model.fieldSize),
      anchor,
      event.deltaY,
    );
    state.cursorWorld = anchor;
    draw(canvas, state);
  };

  const onPointerUp = async event => {
    if (!state.model || !state.layout || !state.pointerDown) return;
    const point = eventPoint(canvas, event, state.layout);
    const cancelled = event.type === 'pointercancel';
    const panned = state.pan && isPointerDrag(state.pointerDown, point, 8 * state.layout.dpr);
    try {
      if (state.drag && !cancelled) {
        const raw = state.drag.axis === 'X'
          ? state.layout.transform.fromX(point.x)
          : state.layout.transform.fromY(point.y);
        const millimetres = Math.round(Math.max(0, Math.min(state.model.fieldSize, raw)));
        await onBoundaryDragged(state.drag.axis, state.drag.index, millimetres);
      } else if (panned) {
        state.viewport = panViewport(state.pan.viewport, state.pan.start, point, state.layout.size);
      } else if (!cancelled) {
        const xmm = state.layout.transform.fromX(point.x);
        const ymm = state.layout.transform.fromY(point.y);
        if (isInsideField({x: xmm, y: ymm}, state.model.fieldSize)) {
          const column = findBand(state.model.xLines, xmm);
          const row = findBand(state.model.yLines, ymm);
          if (row >= 0 && column >= 0) await onCellClicked(row + 1, column + 1);
        }
      }
    } finally {
      if (canvas.hasPointerCapture?.(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
      state.drag = null;
      state.pan = null;
      state.pointerDown = null;
      canvas.style.cursor = 'grab';
      draw(canvas, state);
    }
  };

  const listeners = [
    ['pointerdown', onPointerDown],
    ['pointermove', onPointerMove],
    ['pointerup', onPointerUp],
    ['pointercancel', onPointerUp],
    ['wheel', onWheel],
  ];
  for (const [type, listener] of listeners) {
    canvas.addEventListener(type, listener, type === 'wheel' ? {passive: false} : undefined);
  }
  if (ResizeObserverClass) {
    state.resizeObserver = new ResizeObserverClass(() => draw(canvas, state));
    state.resizeObserver.observe(canvas);
  }
  canvas.style.cursor = 'grab';

  return {
    render(model) {
      state.model = model;
      state.theme = model.theme === 'light' ? 'light' : 'dark';
      draw(canvas, state);
    },
    setTheme(theme) {
      state.theme = theme === 'light' ? 'light' : 'dark';
      draw(canvas, state);
    },
    dispose() {
      state.resizeObserver?.disconnect();
      for (const [type, listener] of listeners) canvas.removeEventListener(type, listener);
      state.model = null;
    },
  };
}
