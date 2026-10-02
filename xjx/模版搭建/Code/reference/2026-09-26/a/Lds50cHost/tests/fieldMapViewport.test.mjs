import assert from "node:assert/strict";
import test from "node:test";
import { readFile } from "node:fs/promises";

const moduleUrl = new URL("../src/Lds50cHost/wwwroot/js/fieldMap.js", import.meta.url);
const source = await readFile(moduleUrl, "utf8");
const fieldMap = await import(`data:text/javascript;base64,${Buffer.from(source).toString("base64")}`);

test("world transform places zero at the centre and shows four complete quadrants", () => {
    const transform = fieldMap.createWorldTransform(2400, 10, 20, 100);

    assert.equal(transform.toX(-2400), 10);
    assert.equal(transform.toX(0), 60);
    assert.equal(transform.toX(2400), 110);
    assert.equal(transform.toY(2400), 20);
    assert.equal(transform.toY(0), 70);
    assert.equal(transform.toY(-2400), 120);
    assert.equal(transform.fromX(35), -1200);
    assert.equal(transform.fromY(95), -1200);
});

test("point visibility keeps negative-coordinate returns inside the four-quadrant view", () => {
    assert.equal(fieldMap.isPointVisible({ x: -400, y: 300 }, 2400), true);
    assert.equal(fieldMap.isPointVisible({ x: 500, y: -900 }, 2400), true);
    assert.equal(fieldMap.isPointVisible({ x: -2401, y: 0 }, 2400), false);
    assert.equal(fieldMap.isPointVisible({ x: 0, y: 2401 }, 2400), false);
});

test("map interaction remains limited to the first-quadrant field", () => {
    assert.equal(fieldMap.isInsideField({ x: 0, y: 0 }, 2400), true);
    assert.equal(fieldMap.isInsideField({ x: 2400, y: 2400 }, 2400), true);
    assert.equal(fieldMap.isInsideField({ x: -1, y: 300 }, 2400), false);
    assert.equal(fieldMap.isInsideField({ x: 300, y: -1 }, 2400), false);
});

test("wheel zoom keeps the world coordinate below the cursor fixed", () => {
    const initial = { centerX: 0, centerY: 0, halfSpan: 2400 };
    const anchor = { x: 1200, y: 600 };
    const zoomed = fieldMap.zoomViewport(initial, anchor, -100);

    assert.deepEqual(zoomed, { centerX: 240, centerY: 120, halfSpan: 1920 });

    const before = fieldMap.createWorldTransform(2400, 0, 0, 100, initial);
    const after = fieldMap.createWorldTransform(2400, 0, 0, 100, zoomed);
    assert.equal(after.toX(anchor.x), before.toX(anchor.x));
    assert.equal(after.toY(anchor.y), before.toY(anchor.y));
});

test("wheel zoom clamps the visible half span without moving its anchor", () => {
    const close = fieldMap.zoomViewport(
        { centerX: 0, centerY: 0, halfSpan: 320 },
        { x: 100, y: -50 },
        -100);
    assert.deepEqual(close, { centerX: 6.25, centerY: -3.125, halfSpan: 300 });

    const far = fieldMap.zoomViewport(
        { centerX: 0, centerY: 0, halfSpan: 35000 },
        { x: 1000, y: 2000 },
        100);
    assert.deepEqual(far, {
        centerX: -142.8571428571429,
        centerY: -285.7142857142858,
        halfSpan: 40000
    });
});

test("cursor angle uses radar zero forward and increases clockwise", () => {
    const radar = { x: 230, y: 230 };

    assert.equal(fieldMap.createCursorReading({ x: 230, y: 1230 }, radar).angle, 0);
    assert.equal(fieldMap.createCursorReading({ x: 1230, y: 230 }, radar).angle, 90);
    assert.equal(fieldMap.createCursorReading({ x: 230, y: -770 }, radar).angle, 180);
    assert.equal(fieldMap.createCursorReading({ x: -770, y: 230 }, radar).angle, 270);
});

test("dragging the canvas pans the viewport as if the map were grabbed", () => {
    const viewport = { centerX: 100, centerY: -200, halfSpan: 2400 };
    const panned = fieldMap.panViewport(
        viewport,
        { x: 400, y: 300 },
        { x: 500, y: 350 },
        1000);

    assert.deepEqual(panned, { centerX: -380, centerY: 40, halfSpan: 2400 });
});

test("pointer movement must reach eight pixels before it becomes a drag", () => {
    const start = { x: 10, y: 10 };

    assert.equal(fieldMap.isPointerDrag(start, { x: 17.9, y: 10 }), false);
    assert.equal(fieldMap.isPointerDrag(start, { x: 18, y: 10 }), true);
});

test("mission marker normalization preserves approved identifiers coordinates and order", () => {
    const source = [
        { number: 1, row: 1, column: 1, x: 230, y: 230 },
        { number: 2, row: 3, column: 1, x: 350, y: 1200 },
        { number: 3, row: 5, column: 3, x: 1200, y: 2050 },
        { number: 4, row: 3, column: 5, x: 2050, y: 1200 },
        { number: 5, row: 1, column: 3, x: 1200, y: 350 }
    ];
    const snapshot = structuredClone(source);

    const markers = fieldMap.createMissionMarkers(source);

    assert.notEqual(markers, source);
    assert.deepEqual(markers.map(marker => marker.number), [1, 2, 3, 4, 5]);
    assert.deepEqual(markers.map(marker => marker.label), ["1", "2", "3", "4", "5"]);
    assert.deepEqual(markers.map(marker => [marker.x, marker.y]), [
        [230, 230], [350, 1200], [1200, 2050], [2050, 1200], [1200, 350]
    ]);
    assert.deepEqual(source, snapshot);
});

test("mission markers use the ordinary viewport visibility boundary", () => {
    const markers = fieldMap.createMissionMarkers([
        { number: 1, row: 1, column: 1, x: 230, y: 230 },
        { number: 2, row: 3, column: 1, x: 2500, y: 1200 }
    ]);
    const viewport = fieldMap.createWorldTransform(2400, 0, 0, 100);

    assert.deepEqual(markers.filter(marker => fieldMap.isPointVisible(marker, viewport)).map(marker => marker.number), [1]);
});
