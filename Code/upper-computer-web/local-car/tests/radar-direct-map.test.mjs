import assert from 'node:assert/strict';
import test from 'node:test';

import {
  createCursorReading,
  createDefaultViewport,
  createMissionMarkers,
  createWorldTransform,
  isInsideField,
  isPointVisible,
  isPointerDrag,
  mountRadarDirectMap,
  panViewport,
  zoomViewport,
} from '../radar-direct-map.mjs';


test('world transform places zero at centre and shows four complete quadrants', () => {
  const viewport = createDefaultViewport(2400);
  assert.deepEqual(viewport, {centerX: 0, centerY: 0, halfSpan: 2400});
  const transform = createWorldTransform(2400, 10, 20, 100, viewport);

  assert.equal(transform.toX(-2400), 10);
  assert.equal(transform.toX(0), 60);
  assert.equal(transform.toX(2400), 110);
  assert.equal(transform.toY(2400), 20);
  assert.equal(transform.toY(0), 70);
  assert.equal(transform.toY(-2400), 120);
  assert.equal(transform.fromX(35), -1200);
  assert.equal(transform.fromY(95), -1200);
});

test('visibility and map interactions retain negative quadrants but restrict field edits', () => {
  assert.equal(isPointVisible({x: -400, y: 300}, 2400), true);
  assert.equal(isPointVisible({x: -400, y: 300}, createDefaultViewport(2400)), true);
  assert.equal(isPointVisible({x: 500, y: -900}, 2400), true);
  assert.equal(isPointVisible({x: -2401, y: 0}, 2400), false);
  assert.equal(isPointVisible({x: 0, y: 2401}, 2400), false);
  assert.equal(isInsideField({x: 0, y: 0}, 2400), true);
  assert.equal(isInsideField({x: 2400, y: 2400}, 2400), true);
  assert.equal(isInsideField({x: -1, y: 300}, 2400), false);
  assert.equal(isInsideField({x: 300, y: -1}, 2400), false);
});

test('wheel zoom keeps its world anchor fixed and clamps both limits', () => {
  const initial = {centerX: 0, centerY: 0, halfSpan: 2400};
  const anchor = {x: 1200, y: 600};
  const zoomed = zoomViewport(initial, anchor, -100);
  assert.deepEqual(zoomed, {centerX: 240, centerY: 120, halfSpan: 1920});

  const before = createWorldTransform(2400, 0, 0, 100, initial);
  const after = createWorldTransform(2400, 0, 0, 100, zoomed);
  assert.equal(after.toX(anchor.x), before.toX(anchor.x));
  assert.equal(after.toY(anchor.y), before.toY(anchor.y));

  assert.deepEqual(
    zoomViewport({centerX: 0, centerY: 0, halfSpan: 320}, {x: 100, y: -50}, -100),
    {centerX: 6.25, centerY: -3.125, halfSpan: 300},
  );
  assert.deepEqual(
    zoomViewport({centerX: 0, centerY: 0, halfSpan: 35000}, {x: 1000, y: 2000}, 100),
    {centerX: -142.8571428571429, centerY: -285.7142857142858, halfSpan: 40000},
  );
});

test('panning and drag threshold match the reference host', () => {
  assert.deepEqual(
    panViewport(
      {centerX: 100, centerY: -200, halfSpan: 2400},
      {x: 400, y: 300},
      {x: 500, y: 350},
      1000,
    ),
    {centerX: -380, centerY: 40, halfSpan: 2400},
  );
  assert.equal(isPointerDrag({x: 10, y: 10}, {x: 17.9, y: 10}), false);
  assert.equal(isPointerDrag({x: 10, y: 10}, {x: 18, y: 10}), true);
});

test('cursor angle uses radar zero forward and increases clockwise', () => {
  const radar = {x: 230, y: 230};
  assert.equal(createCursorReading({x: 230, y: 1230}, radar).angle, 0);
  assert.equal(createCursorReading({x: 1230, y: 230}, radar).angle, 90);
  assert.equal(createCursorReading({x: 230, y: -770}, radar).angle, 180);
  assert.equal(createCursorReading({x: -770, y: 230}, radar).angle, 270);
});

test('mission marker normalization preserves identifiers, coordinates, order, and source', () => {
  const source = [
    {number: 1, row: 1, column: 1, x: 230, y: 230},
    {number: 2, row: 3, column: 1, x: 350, y: 1200},
    {number: 3, row: 5, column: 3, x: 1200, y: 2050},
    {number: 4, row: 3, column: 5, x: 2050, y: 1200},
    {number: 5, row: 1, column: 3, x: 1200, y: 350},
  ];
  const snapshot = structuredClone(source);

  const markers = createMissionMarkers(source);

  assert.notEqual(markers, source);
  assert.deepEqual(markers.map(marker => marker.number), [1, 2, 3, 4, 5]);
  assert.deepEqual(markers.map(marker => marker.label), ['1', '2', '3', '4', '5']);
  assert.deepEqual(markers.map(marker => [marker.x, marker.y]), [
    [230, 230], [350, 1200], [1200, 2050], [2050, 1200], [1200, 350],
  ]);
  assert.deepEqual(source, snapshot);
});


class FakeContext {
  constructor() {
    this.calls = [];
  }
}

for (const name of [
  'arc', 'beginPath', 'clearRect', 'closePath', 'fill', 'fillRect', 'fillText',
  'lineTo', 'moveTo', 'restore', 'save', 'setLineDash', 'stroke', 'strokeRect',
]) {
  FakeContext.prototype[name] = function (...args) {
    this.calls.push([name, ...args]);
  };
}

class FakeCanvas {
  constructor() {
    this.width = 0;
    this.height = 0;
    this.style = {};
    this.context = new FakeContext();
    this.listeners = new Map();
    this.captures = new Set();
  }

  getContext() { return this.context; }
  getBoundingClientRect() { return {left: 0, top: 0, width: 400, height: 400}; }
  addEventListener(type, listener) { this.listeners.set(type, listener); }
  removeEventListener(type, listener) {
    if (this.listeners.get(type) === listener) this.listeners.delete(type);
  }
  setPointerCapture(id) { this.captures.add(id); }
  hasPointerCapture(id) { return this.captures.has(id); }
  releasePointerCapture(id) { this.captures.delete(id); }
  dispatch(type, values = {}) {
    const listener = this.listeners.get(type);
    assert.ok(listener, `missing ${type} listener`);
    const event = {
      type,
      button: 0,
      pointerId: 1,
      clientX: 0,
      clientY: 0,
      deltaY: 0,
      prevented: false,
      preventDefault() { this.prevented = true; },
      ...values,
    };
    return {event, result: listener(event)};
  }
}

function renderModel() {
  const lines = [150, 550, 1000, 1400, 1850, 2250];
  const cells = [];
  for (let row = 1; row <= 5; row += 1) {
    for (let column = 1; column <= 5; column += 1) {
      cells.push({
        row,
        column,
        minXmm: lines[column - 1],
        maxXmm: lines[column],
        minYmm: lines[row - 1],
        maxYmm: lines[row],
        flags: {fixed: row === 2 && column === 2, manual: false, scanned: false},
        scannedPointCount: row === 1 && column === 1 ? 3 : 0,
        protected: row === 1 && column === 3,
      });
    }
  }
  return {
    fieldSize: 2400,
    activeMin: 150,
    activeMax: 2250,
    xLines: lines,
    yLines: lines,
    radar: {x: 230, y: 230},
    points: [{x: -400, y: 300, energy: 200}, {x: 300, y: 400, energy: 100}],
    cells,
    path: [{x: 230, y: 230}, {x: 775, y: 350}],
    missionPoints: [{number: 1, row: 1, column: 1, x: 230, y: 230}],
    start: null,
    goal: null,
    theme: 'dark',
  };
}

function canvasPointForWorld(x, y) {
  const transform = createWorldTransform(2400, 46, 46, 308, createDefaultViewport(2400));
  return {clientX: transform.toX(x), clientY: transform.toY(y)};
}

test('mounted renderer draws every semantic layer and wheel zoom prevents page scrolling', () => {
  const canvas = new FakeCanvas();
  const observers = [];
  class FakeResizeObserver {
    constructor(callback) { this.callback = callback; this.disconnected = false; observers.push(this); }
    observe(target) { this.target = target; }
    disconnect() { this.disconnected = true; }
  }
  const map = mountRadarDirectMap({
    canvas,
    onBoundaryDragged() {},
    onCellClicked() {},
    ResizeObserverClass: FakeResizeObserver,
  });

  map.render(renderModel());
  const methods = new Set(canvas.context.calls.map(call => call[0]));
  for (const method of ['fillRect', 'strokeRect', 'fillText', 'arc', 'lineTo', 'setLineDash']) {
    assert.ok(methods.has(method), method);
  }
  const before = canvas.context.calls.length;
  const wheel = canvas.dispatch('wheel', {...canvasPointForWorld(1200, 600), deltaY: -100});
  assert.equal(wheel.event.prevented, true);
  assert.ok(canvas.context.calls.length > before);
  assert.equal(observers[0].target, canvas);

  map.dispose();
  assert.equal(observers[0].disconnected, true);
  assert.equal(canvas.listeners.size, 0);
});

test('short click toggles a normal cell, while an eight-pixel drag only pans', async () => {
  const canvas = new FakeCanvas();
  const clicked = [];
  const map = mountRadarDirectMap({
    canvas,
    onBoundaryDragged() {},
    onCellClicked(row, column) { clicked.push([row, column]); },
    ResizeObserverClass: class { observe() {} disconnect() {} },
  });
  map.render(renderModel());
  const point = canvasPointForWorld(350, 350);

  canvas.dispatch('pointerdown', point);
  await canvas.dispatch('pointerup', {...point, clientX: point.clientX + 2}).result;
  assert.deepEqual(clicked, [[1, 1]]);

  canvas.dispatch('pointerdown', point);
  canvas.dispatch('pointermove', {...point, clientX: point.clientX + 8});
  await canvas.dispatch('pointerup', {...point, clientX: point.clientX + 8}).result;
  assert.deepEqual(clicked, [[1, 1]]);
  map.dispose();
});

test('dragging an internal boundary reports its axis, index, and rounded millimetres', async () => {
  const canvas = new FakeCanvas();
  const dragged = [];
  const map = mountRadarDirectMap({
    canvas,
    onBoundaryDragged(axis, index, millimetres) { dragged.push([axis, index, millimetres]); },
    onCellClicked() {},
    ResizeObserverClass: class { observe() {} disconnect() {} },
  });
  map.render(renderModel());

  canvas.dispatch('pointerdown', canvasPointForWorld(550, 1000));
  await canvas.dispatch('pointerup', canvasPointForWorld(600.4, 1000)).result;

  assert.deepEqual(dragged, [['X', 0, 600]]);
  map.dispose();
});
