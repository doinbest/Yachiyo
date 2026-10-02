const maps = new WeakMap();
const minimumViewportHalfSpan = 300;
const maximumViewportHalfSpan = 40000;

const palette = {
    dark: {
        background: "#061019", field: "#0b1a27", grid: "#4d6575", outer: "#8ea3ae",
        text: "#b9cbd3", point: "#83f7cb", fixed: "rgba(250, 204, 21, .50)",
        manual: "rgba(249, 115, 22, .55)", scanned: "rgba(239, 68, 68, .50)",
        route: "#38bdf8", radar: "#f8fafc", start: "#22c55e", goal: "#e879f9",
        mission: "#f97316", missionText: "#ffffff", missionOutline: "#ffedd5",
        cursor: "#fde047", cursorPanel: "rgba(0, 0, 0, .68)"
    },
    light: {
        background: "#edf4f5", field: "#ffffff", grid: "#8ca2aa", outer: "#334155",
        text: "#1f3440", point: "#087f74", fixed: "rgba(250, 204, 21, .55)",
        manual: "rgba(249, 115, 22, .50)", scanned: "rgba(239, 68, 68, .42)",
        route: "#0369a1", radar: "#0f172a", start: "#15803d", goal: "#a21caf",
        mission: "#c2410c", missionText: "#ffffff", missionOutline: "#7c2d12",
        cursor: "#854d0e", cursorPanel: "rgba(255, 255, 255, .86)"
    }
};

export function createDefaultViewport(fieldSize) {
    return { centerX: 0, centerY: 0, halfSpan: fieldSize };
}

export function zoomViewport(viewport, anchor, deltaY) {
    const requestedHalfSpan = viewport.halfSpan * (deltaY < 0 ? .8 : 1.25);
    const halfSpan = Math.max(minimumViewportHalfSpan, Math.min(maximumViewportHalfSpan, requestedHalfSpan));
    const ratio = halfSpan / viewport.halfSpan;
    return {
        centerX: anchor.x + (viewport.centerX - anchor.x) * ratio,
        centerY: anchor.y + (viewport.centerY - anchor.y) * ratio,
        halfSpan
    };
}

export function panViewport(viewport, startCanvas, currentCanvas, canvasSize) {
    const millimetresPerPixel = viewport.halfSpan * 2 / canvasSize;
    return {
        centerX: viewport.centerX - (currentCanvas.x - startCanvas.x) * millimetresPerPixel,
        centerY: viewport.centerY + (currentCanvas.y - startCanvas.y) * millimetresPerPixel,
        halfSpan: viewport.halfSpan
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
    return { x: point.x, y: point.y, angle };
}

export function createMissionMarkers(points) {
    return (points || []).map(point => ({
        number: Number(point.number),
        label: String(point.number),
        row: Number(point.row),
        column: Number(point.column),
        x: Number(point.x),
        y: Number(point.y)
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
        fromY: value => maxY - (value - top) / size * viewSpan
    };
}

export function isPointVisible(point, viewportOrFieldSize) {
    const bounds = typeof viewportOrFieldSize === "number"
        ? createWorldTransform(viewportOrFieldSize, 0, 0, 1)
        : viewportOrFieldSize;
    return point.x >= bounds.minX && point.x <= bounds.maxX &&
        point.y >= bounds.minY && point.y <= bounds.maxY;
}

export function isInsideField(point, fieldSize) {
    return point.x >= 0 && point.x <= fieldSize &&
        point.y >= 0 && point.y <= fieldSize;
}

export function initializeMap(canvas, dotnetRef) {
    const state = {
        dotnetRef,
        model: null,
        theme: "dark",
        layout: null,
        viewport: null,
        cursorWorld: null,
        drag: null,
        pan: null,
        pointerDown: null,
        resizeObserver: null,
        onPointerDown: event => pointerDown(canvas, event),
        onPointerMove: event => pointerMove(canvas, event),
        onPointerUp: event => pointerUp(canvas, event),
        onWheel: event => wheel(canvas, event)
    };

    maps.set(canvas, state);
    canvas.addEventListener("pointerdown", state.onPointerDown);
    canvas.addEventListener("pointermove", state.onPointerMove);
    canvas.addEventListener("pointerup", state.onPointerUp);
    canvas.addEventListener("pointercancel", state.onPointerUp);
    canvas.addEventListener("wheel", state.onWheel, { passive: false });
    state.resizeObserver = new ResizeObserver(() => draw(canvas));
    state.resizeObserver.observe(canvas);
}

export function renderMap(canvas, model) {
    const state = maps.get(canvas);
    if (!state) return;
    state.model = model;
    state.theme = model.theme || state.theme;
    draw(canvas);
}

export function setTheme(canvas, theme) {
    const state = maps.get(canvas);
    if (!state) return;
    state.theme = theme === "light" ? "light" : "dark";
    draw(canvas);
}

export function disposeMap(canvas) {
    const state = maps.get(canvas);
    if (!state) return;
    state.resizeObserver?.disconnect();
    canvas.removeEventListener("pointerdown", state.onPointerDown);
    canvas.removeEventListener("pointermove", state.onPointerMove);
    canvas.removeEventListener("pointerup", state.onPointerUp);
    canvas.removeEventListener("pointercancel", state.onPointerUp);
    canvas.removeEventListener("wheel", state.onWheel);
    maps.delete(canvas);
}

function resizeCanvas(canvas) {
    const rect = canvas.getBoundingClientRect();
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const width = Math.max(320, Math.round(rect.width * dpr));
    const height = Math.max(320, Math.round(rect.height * dpr));
    if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
    }
    return { width, height, dpr };
}

function draw(canvas) {
    const state = maps.get(canvas);
    if (!state?.model) return;

    const { width, height, dpr } = resizeCanvas(canvas);
    const ctx = canvas.getContext("2d");
    const colors = palette[state.theme] || palette.dark;
    const pad = 46 * dpr;
    const size = Math.max(1, Math.min(width, height) - pad * 2);
    const left = (width - size) / 2;
    const top = (height - size) / 2;
    state.viewport ||= createDefaultViewport(state.model.fieldSize);
    const transform = createWorldTransform(state.model.fieldSize, left, top, size, state.viewport);
    const layout = { width, height, dpr, pad, size, left, top, transform };
    state.layout = layout;

    const { toX, toY } = transform;
    const fieldLeft = toX(0);
    const fieldTop = toY(state.model.fieldSize);
    const fieldSize = toX(state.model.fieldSize) - fieldLeft;

    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = colors.background;
    ctx.fillRect(0, 0, width, height);
    ctx.fillStyle = colors.field;
    ctx.fillRect(fieldLeft, fieldTop, fieldSize, fieldSize);

    drawMillimetreGrid(ctx, state.model, layout, colors, toX, toY);
    drawCells(ctx, state.model, colors, toX, toY, dpr);
    drawBoundaryLines(ctx, state.model, colors, toX, toY, dpr);
    drawPointCloud(ctx, state.model, colors, toX, toY, dpr, transform);
    drawRoute(ctx, state.model, colors, toX, toY, dpr);
    if (state.model.start) drawEndpoint(ctx, state.model.start, "S", colors.start, state.model, toX, toY, dpr);
    if (state.model.goal) drawEndpoint(ctx, state.model.goal, "G", colors.goal, state.model, toX, toY, dpr);
    drawMissionMarkers(ctx, state.model.missionPoints, colors, toX, toY, dpr, transform);
    drawRadar(ctx, state.model.radar, colors, toX, toY, dpr);

    ctx.strokeStyle = colors.outer;
    ctx.lineWidth = 2 * dpr;
    ctx.strokeRect(fieldLeft, fieldTop, fieldSize, fieldSize);
    drawCursorReading(ctx, state, colors, layout);
}

function drawMillimetreGrid(ctx, model, layout, colors, toX, toY) {
    const { dpr, left, top, size, transform } = layout;
    const step = chooseGridStep(transform.maxX - transform.minX);
    ctx.save();
    ctx.strokeStyle = colors.grid;
    ctx.fillStyle = colors.text;
    for (let mm = firstGridLine(transform.minX, step); mm <= transform.maxX; mm += step) {
        ctx.globalAlpha = mm === 0 ? .75 : .18;
        ctx.lineWidth = mm === 0 ? 2 * dpr : dpr;
        ctx.beginPath(); ctx.moveTo(toX(mm), top); ctx.lineTo(toX(mm), top + size); ctx.stroke();
    }
    for (let mm = firstGridLine(transform.minY, step); mm <= transform.maxY; mm += step) {
        ctx.globalAlpha = mm === 0 ? .75 : .18;
        ctx.lineWidth = mm === 0 ? 2 * dpr : dpr;
        ctx.beginPath(); ctx.moveTo(left, toY(mm)); ctx.lineTo(left + size, toY(mm)); ctx.stroke();
    }
    ctx.globalAlpha = 1;
    ctx.font = `${11 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
    ctx.textAlign = "center";
    for (let mm = firstGridLine(transform.minX, step); mm <= transform.maxX; mm += step)
        ctx.fillText(String(mm), toX(mm), top + size + 22 * dpr);
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let mm = firstGridLine(transform.minY, step); mm <= transform.maxY; mm += step)
        ctx.fillText(String(mm), left - 10 * dpr, toY(mm));
    ctx.restore();
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

function drawCells(ctx, model, colors, toX, toY, dpr) {
    for (const cell of model.cells) {
        let fill = null;
        if ((cell.flags & 1) !== 0) fill = colors.fixed;
        else if ((cell.flags & 2) !== 0) fill = colors.manual;
        else if ((cell.flags & 4) !== 0) fill = colors.scanned;
        if (fill) {
            ctx.fillStyle = fill;
            ctx.fillRect(toX(cell.minX), toY(cell.maxY), toX(cell.maxX) - toX(cell.minX), toY(cell.minY) - toY(cell.maxY));
        }

        const centerX = (toX(cell.minX) + toX(cell.maxX)) / 2;
        const centerY = (toY(cell.minY) + toY(cell.maxY)) / 2;
        ctx.fillStyle = colors.text;
        ctx.globalAlpha = .72;
        ctx.font = `${10 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
        ctx.textAlign = "center";
        ctx.fillText(`R${cell.row}C${cell.column}${cell.count ? ` · ${cell.count}` : ""}`, centerX, centerY);
        ctx.globalAlpha = 1;
    }
}

function drawBoundaryLines(ctx, model, colors, toX, toY, dpr) {
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
    ctx.textAlign = "center";
    for (let i = 0; i < 5; i++) {
        const width = Math.round(model.xLines[i + 1] - model.xLines[i]);
        ctx.fillText(`${width}`, (toX(model.xLines[i]) + toX(model.xLines[i + 1])) / 2, toY(model.activeMax) - 9 * dpr);
    }
    ctx.textAlign = "left";
    ctx.textBaseline = "middle";
    for (let i = 0; i < 5; i++) {
        const height = Math.round(model.yLines[i + 1] - model.yLines[i]);
        ctx.fillText(`${height}`, toX(model.activeMax) + 8 * dpr, (toY(model.yLines[i]) + toY(model.yLines[i + 1])) / 2);
    }
    ctx.restore();
}

function drawPointCloud(ctx, model, colors, toX, toY, dpr, transform) {
    ctx.save();
    ctx.fillStyle = colors.point;
    for (const point of model.points) {
        if (!isPointVisible(point, transform)) continue;
        ctx.globalAlpha = .35 + Math.min(1, point.energy / 255) * .65;
        ctx.beginPath();
        ctx.arc(toX(point.x), toY(point.y), Math.max(1.25, 1.65 * dpr), 0, Math.PI * 2);
        ctx.fill();
    }
    ctx.restore();
}

function drawRoute(ctx, model, colors, toX, toY, dpr) {
    if (!model.path || model.path.length < 2) return;
    ctx.save();
    ctx.strokeStyle = colors.route;
    ctx.lineWidth = 4 * dpr;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.shadowBlur = 9 * dpr;
    ctx.shadowColor = colors.route;
    ctx.beginPath();
    ctx.moveTo(toX(model.path[0].x), toY(model.path[0].y));
    for (const point of model.path.slice(1)) ctx.lineTo(toX(point.x), toY(point.y));
    ctx.stroke();
    ctx.shadowBlur = 0;
    for (const point of model.path) {
        ctx.fillStyle = colors.route;
        ctx.beginPath(); ctx.arc(toX(point.x), toY(point.y), 4 * dpr, 0, Math.PI * 2); ctx.fill();
    }
    ctx.restore();
}

function drawMissionMarkers(ctx, points, colors, toX, toY, dpr, transform) {
    const markers = createMissionMarkers(points);
    ctx.save();
    for (const marker of markers) {
        if (!isPointVisible(marker, transform)) continue;
        const offsetX = marker.number === 1 ? 18 * dpr : 0;
        const offsetY = marker.number === 1 ? -18 * dpr : 0;
        const sourceX = toX(marker.x), sourceY = toY(marker.y);
        const x = sourceX + offsetX, y = sourceY + offsetY;
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
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(marker.label, x, y + .5 * dpr);
    }
    ctx.restore();
}

function drawEndpoint(ctx, endpoint, label, color, model, toX, toY, dpr) {
    const x = toX(endpoint.x);
    const y = toY(endpoint.y);
    ctx.save();
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(x, y, 11 * dpr, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = "#ffffff";
    ctx.font = `700 ${11 * dpr}px system-ui, sans-serif`;
    ctx.textAlign = "center"; ctx.textBaseline = "middle"; ctx.fillText(label, x, y + .5 * dpr);
    ctx.restore();
}

function drawRadar(ctx, radar, colors, toX, toY, dpr) {
    const x = toX(radar.x), y = toY(radar.y);
    ctx.save();
    ctx.strokeStyle = "#22d3ee";
    ctx.fillStyle = colors.radar;
    ctx.lineWidth = 3 * dpr;
    ctx.shadowBlur = 12 * dpr;
    ctx.shadowColor = "#22d3ee";
    ctx.beginPath(); ctx.arc(x, y, 10 * dpr, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.shadowBlur = 0;
    ctx.beginPath(); ctx.moveTo(x, y - 13 * dpr); ctx.lineTo(x - 5 * dpr, y - 4 * dpr); ctx.lineTo(x + 5 * dpr, y - 4 * dpr); ctx.closePath(); ctx.fillStyle = "#22d3ee"; ctx.fill();
    ctx.fillStyle = colors.text;
    ctx.font = `${11 * dpr}px system-ui, sans-serif`;
    ctx.textAlign = "left";
    ctx.fillText(`雷达 ${Math.round(radar.x)}, ${Math.round(radar.y)}`, x + 14 * dpr, y + 18 * dpr);
    ctx.restore();
}

function drawCursorReading(ctx, state, colors, layout) {
    if (!state.cursorWorld) return;

    const reading = createCursorReading(state.cursorWorld, state.model.radar);
    const { dpr, left, top } = layout;
    const lines = [
        `X: ${reading.x.toFixed(1)} mm`,
        `Y: ${reading.y.toFixed(1)} mm`,
        `angle: ${reading.angle.toFixed(2)}°`
    ];
    const padding = 9 * dpr;
    const lineHeight = 20 * dpr;
    const panelWidth = 174 * dpr;
    const panelHeight = padding * 2 + lineHeight * lines.length;

    ctx.save();
    ctx.fillStyle = colors.cursorPanel;
    ctx.fillRect(left + 8 * dpr, top + 8 * dpr, panelWidth, panelHeight);
    ctx.fillStyle = colors.cursor;
    ctx.font = `700 ${14 * dpr}px ui-monospace, SFMono-Regular, Consolas, monospace`;
    ctx.textAlign = "left";
    ctx.textBaseline = "top";
    for (let index = 0; index < lines.length; index++)
        ctx.fillText(lines[index], left + 8 * dpr + padding, top + 8 * dpr + padding + index * lineHeight);
    ctx.restore();
}

function pointerDown(canvas, event) {
    const state = maps.get(canvas);
    if (!state?.model || !state.layout || event.button !== 0) return;
    const point = eventPoint(canvas, event, state.layout);
    state.pointerDown = point;
    state.drag = hitBoundary(state, point);
    state.pan = state.drag ? null : {
        start: point,
        viewport: { ...(state.viewport || createDefaultViewport(state.model.fieldSize)) }
    };
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
}

function pointerMove(canvas, event) {
    const state = maps.get(canvas);
    if (!state?.model || !state.layout) return;
    const point = eventPoint(canvas, event, state.layout);
    const panning = state.pan && state.pointerDown &&
        isPointerDrag(state.pointerDown, point, 8 * state.layout.dpr);

    if (panning) {
        state.viewport = panViewport(state.pan.viewport, state.pan.start, point, state.layout.size);
    }

    const transform = createWorldTransform(
        state.model.fieldSize,
        state.layout.left,
        state.layout.top,
        state.layout.size,
        state.viewport || createDefaultViewport(state.model.fieldSize));
    state.cursorWorld = { x: transform.fromX(point.x), y: transform.fromY(point.y) };

    const hit = state.drag || (!state.pointerDown ? hitBoundary(state, point) : null);
    canvas.style.cursor = hit
        ? (hit.axis === "X" ? "ew-resize" : "ns-resize")
        : (panning ? "grabbing" : "grab");
    draw(canvas);
}

function wheel(canvas, event) {
    const state = maps.get(canvas);
    if (!state?.model || !state.layout || event.deltaY === 0) return;

    event.preventDefault();
    const point = eventPoint(canvas, event, state.layout);
    const anchor = { x: fromX(point.x, state), y: fromY(point.y, state) };
    state.viewport = zoomViewport(
        state.viewport || createDefaultViewport(state.model.fieldSize),
        anchor,
        event.deltaY);
    state.cursorWorld = anchor;
    draw(canvas);
}

async function pointerUp(canvas, event) {
    const state = maps.get(canvas);
    if (!state?.model || !state.layout || !state.pointerDown) return;
    const point = eventPoint(canvas, event, state.layout);
    const cancelled = event.type === "pointercancel";
    const panned = state.pan && isPointerDrag(state.pointerDown, point, 8 * state.layout.dpr);

    try {
        if (state.drag && !cancelled) {
            const rawMillimetres = state.drag.axis === "X" ? fromX(point.x, state) : fromY(point.y, state);
            const mm = Math.max(0, Math.min(state.model.fieldSize, rawMillimetres));
            await state.dotnetRef.invokeMethodAsync("OnBoundaryDragged", state.drag.axis, state.drag.index, mm);
        } else if (panned) {
            state.viewport = panViewport(state.pan.viewport, state.pan.start, point, state.layout.size);
        } else if (!cancelled) {
            const xMm = fromX(point.x, state), yMm = fromY(point.y, state);
            if (isInsideField({ x: xMm, y: yMm }, state.model.fieldSize)) {
                const column = findBand(state.model.xLines, xMm);
                const row = findBand(state.model.yLines, yMm);
                if (row >= 0 && column >= 0)
                    await state.dotnetRef.invokeMethodAsync("OnCellClicked", row + 1, column + 1);
            }
        }
    } finally {
        if (canvas.hasPointerCapture?.(event.pointerId))
            canvas.releasePointerCapture(event.pointerId);
        state.drag = null;
        state.pan = null;
        state.pointerDown = null;
        canvas.style.cursor = "grab";
        draw(canvas);
    }
}

function eventPoint(canvas, event, layout) {
    const rect = canvas.getBoundingClientRect();
    return { x: (event.clientX - rect.left) * layout.width / rect.width, y: (event.clientY - rect.top) * layout.height / rect.height };
}

function hitBoundary(state, point) {
    const { model, layout } = state;
    const tolerance = 10 * layout.dpr;
    const yMin = toCanvasY(model.activeMax, state), yMax = toCanvasY(model.activeMin, state);
    for (let index = 0; index < 4; index++) {
        const x = toCanvasX(model.xLines[index + 1], state);
        if (Math.abs(point.x - x) <= tolerance && point.y >= yMin && point.y <= yMax) return { axis: "X", index };
    }
    const xMin = toCanvasX(model.activeMin, state), xMax = toCanvasX(model.activeMax, state);
    for (let index = 0; index < 4; index++) {
        const y = toCanvasY(model.yLines[index + 1], state);
        if (Math.abs(point.y - y) <= tolerance && point.x >= xMin && point.x <= xMax) return { axis: "Y", index };
    }
    return null;
}

function toCanvasX(mm, state) { return state.layout.transform.toX(mm); }
function toCanvasY(mm, state) { return state.layout.transform.toY(mm); }
function fromX(px, state) { return state.layout.transform.fromX(px); }
function fromY(py, state) { return state.layout.transform.fromY(py); }

function findBand(lines, value) {
    for (let index = 0; index < lines.length - 1; index++) {
        if (value >= lines[index] && (value < lines[index + 1] || (index === lines.length - 2 && value <= lines[index + 1]))) return index;
    }
    return -1;
}
