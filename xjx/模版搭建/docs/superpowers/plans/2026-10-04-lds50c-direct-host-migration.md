# LDS50C Direct Host Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace only the current web console's “雷达建图” tab with a functionally equivalent USB-UART direct LDS50C host while leaving the F407 connection and every other console feature unchanged.

**Architecture:** Add an independent Python `RadarDirectBridge` beside the existing F407 `SerialBridge`; it owns the high-speed binary radar port, protocol parser, scan assembly, and complete-scan snapshots. New browser modules consume a source-neutral complete-scan contract and reproduce the supplied host's filtering, mapping, A* planning, diagnostics, CSV export, persistence, canvas interactions, and UI; the existing monitor/offline tabs stay on their current code path.

**Tech Stack:** Python 3 standard library + existing pyserial, vanilla ES modules, Canvas 2D, Node `node:test`, Python `unittest`, existing loopback `ThreadingHTTPServer`.

**Spec:** `docs/superpowers/specs/2026-10-04-lds50c-direct-host-migration-design.md`

## Global Constraints

- Treat the supplied `D:/Document/xwechat_files/wxid_gh3hulr1y4vm22_1404/msg/file/2026-10/a/a/Lds50cHost/` source and tests as the functional reference for direct-radar mode.
- Do not implement or expose the reference application's STM32 configuration/download/Flash mode.
- Do not modify `Code/template/`, firmware protocols, Keil, EIDE, IOC, pin, clock, motor, IMU, or motion behavior.
- Keep the existing F407 `SerialBridge`, 115200/8N1 connection, stop latch, preparation ownership, event stream, and HTTP request/response contracts unchanged.
- Never auto-select, auto-open, or auto-start a radar COM port; only an explicit user action may connect or start it.
- Support exactly 384000, 500000, 768000, and 921600 baud for the direct radar, defaulting to 500000, with 8N1 and no flow control.
- Publish only complete revolutions to the browser; retain the last complete scan, map, path, and saved settings across scan failure or disconnect.
- Keep the new radar UI and styles scoped to the radar-direct workspace; monitor, offline comparison, arm, chassis, vision, QR, parameters, system, and the global theme must not change.
- Use no new npm package, .NET runtime, web framework, cloud service, or automatic hardware action.
- Preserve the user's existing unrelated changes in `docs/README.md` and `docs/项目总结与AI交接.md`.

## Review Focus

- A fragmented header, corrupt frame, or declared point count above 4096 must remain bounded, increment the right counter, and resynchronize without publishing a partial scan; Task 1 pins this.
- A millimetre/centimetre status change during a revolution must discard only that incomplete revolution and accept the next complete one; Task 2 pins this.
- Stop, disconnect, and service shutdown racing with continuous scanning must terminate the radar worker, attempt `LSTOPH`, close only the radar port, and retain the latest complete scan; Task 2 pins this.
- Selecting a COM port already owned by the F407 bridge must report the radar connection failure while leaving the F407 connection and event cursor intact; Task 3 pins this.
- Stale scan revisions, malformed saved browser configuration, or storage write failure must not replace newer state, send device commands, or break the rest of the page; Tasks 4 and 6 pin this.

---

### Task 1: LDS50C Protocol Parser and Complete-Revolution Assembler

**Files:**
- Create: `Code/upper-computer-web/local-car/radar_direct_protocol.py`
- Create: `Code/upper-computer-web/local-car/tests/test_radar_direct_protocol.py`

**Interfaces:**
- Consumes: raw byte chunks from a pyserial reader.
- Produces: immutable `RawMeasurement`, `MeasurementFrame`, `StatusFrame`, `AlarmFrame`, and `RadarPoint` dataclasses; `Lds50cParser.feed(data: bytes) -> list[MeasurementFrame | StatusFrame | AlarmFrame]`; `normalize_measurement(frame: MeasurementFrame, uses_millimetres: bool) -> MeasurementFrame`; `ScanAssembler.push(frame: MeasurementFrame) -> tuple[RadarPoint, ...] | None`; `ScanAssembler.reset() -> None`.
- Exposes parser counters as `valid_frames`, `checksum_errors`, `length_errors`, and `discarded_bytes` integer attributes.

- [ ] **Step 1: Write failing parser tests for every reference frame type and Review Focus malformed input**

Add tests with deterministic frame builders and these assertions:

```python
def test_parser_accepts_normal_fixed_status_and_alarm_frames_across_chunks():
    frames = feed_one_byte_at_a_time(normal_frame + fixed_frame + status_frame + alarm_frame)
    assert [type(frame) for frame in frames] == [MeasurementFrame, MeasurementFrame, StatusFrame, AlarmFrame]
    assert frames[0].fixed_resolution is False
    assert frames[1].fixed_resolution is True
    assert frames[2].uses_millimetres is True
    assert frames[2].energy_enabled is True
    assert frames[3].code == 0x1234

def test_parser_bounds_bad_length_and_resynchronizes_after_bad_checksum():
    frames = parser.feed(oversized_header + bad_checksum_frame + valid_status_frame)
    assert frames == [expected_status]
    assert parser.length_errors == 1
    assert parser.checksum_errors == 1
    assert parser.discarded_bytes > 0
```

- [ ] **Step 2: Run the parser tests and verify they fail because the module does not exist**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test_radar_direct_protocol.py" -v`

Expected: FAIL with `ModuleNotFoundError` for `radar_direct_protocol`.

- [ ] **Step 3: Implement the frame dataclasses, checksum, and bounded stream parser**

Implement the reference layouts and little-endian decoding in `radar_direct_protocol.py`. Limit measurement point counts to 4096; preserve potential partial header prefixes across `feed` calls; on bad checksum consume the declared frame and count its bytes as discarded, matching the supplied parser.

- [ ] **Step 4: Run the parser tests and verify they pass**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test_radar_direct_protocol.py" -v`

Expected: all parser tests PASS.

- [ ] **Step 5: Write failing normalization and revolution-assembly tests**

Cover ordinary-frame interpolation using the next frame's start angle, fixed-resolution interpolation using sector angle, first-wrap arming, minimum 30-point completion, reset, centimetre-to-millimetre conversion, and centimetre overflow to zero:

```python
def test_assembler_arms_on_first_wrap_and_publishes_only_a_later_complete_revolution():
    assert assembler.push(frame_at_350_degrees) is None
    assert assembler.push(frame_at_10_degrees) is None
    completed = finish_next_full_revolution(assembler)
    assert len(completed) >= 30
    assert all(0.0 <= point.angle_deg < 360.0 for point in completed)

def test_centimetres_normalize_to_mm_and_overflow_becomes_zero():
    normalized = normalize_measurement(frame_with_distances(100, 7000), False)
    assert [m.distance_mm for m in normalized.measurements] == [1000, 0]
```

- [ ] **Step 6: Implement `normalize_measurement` and `ScanAssembler`**

Port the supplied `MeasurementUnitNormalizer` and `ScanAssembler` behavior exactly, including pending ordinary-frame interpolation, fixed-resolution interpolation, wrap detection at 3600 tenths, first-wrap arming, and `reset()`.

- [ ] **Step 7: Run the complete protocol test file**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test_radar_direct_protocol.py" -v`

Expected: all protocol and assembly tests PASS.

- [ ] **Step 8: Commit Task 1**

```bash
git add Code/upper-computer-web/local-car/radar_direct_protocol.py Code/upper-computer-web/local-car/tests/test_radar_direct_protocol.py
git commit -m "feat: parse LDS50C direct radar scans"
```

### Task 2: Independent Radar Serial Owner and Scan Lifecycle

**Files:**
- Create: `Code/upper-computer-web/local-car/radar_direct_bridge.py`
- Create: `Code/upper-computer-web/local-car/tests/test_radar_direct_bridge.py`
- Use: `Code/upper-computer-web/local-car/radar_direct_protocol.py`

**Interfaces:**
- Consumes: `Lds50cParser`, `ScanAssembler`, and a testable `serial_factory(**options)` compatible with pyserial.
- Produces: `RadarDirectBridge(serial_factory=open_radar_serial, *, first_scan_timeout=8.0, event_limit=200)` with methods `connect(port: str, baudrate: int) -> dict`, `disconnect() -> dict`, `start_rotation() -> dict`, `stop_rotation() -> dict`, `capture_single() -> dict`, `start_continuous() -> dict`, `stop_continuous() -> dict`, `status() -> dict`, `latest_scan(after_revision: int = 0) -> dict`, `get_events(after: int = 0, timeout: float = 0.0) -> dict`, and `close() -> None`.
- `status()` returns JSON-safe keys `connected`, `port`, `baudrate`, `operation`, `continuous`, `continuous_error`, `scan_revision`, `scan_received_at`, `radar_status`, `parser`, and `last_event_id`.
- `latest_scan()` returns `{revision, received_at, points}` only when `revision > after_revision`; otherwise it returns the current revision with `points: null`.

- [ ] **Step 1: Write failing lifecycle tests with a deterministic fake serial port**

The fake exposes `in_waiting`, `read(count)`, `write(data)`, and `close()`, plus a test-only `feed(bytes)` method. Pin the exact preflight command order and single-scan stop behavior:

```python
def test_single_scan_sends_reference_preflight_freezes_one_circle_and_stops():
    bridge.connect('COM7', 921600)
    feed_two_wraps_for_one_complete_circle(fake_port)
    result = bridge.capture_single()
    assert fake_port.writes == [b'LMDMMH', b'LOCONH', b'LFFF1H', b'LSSS1H', b'LSTARH', b'LSTOPH']
    assert result['scan_revision'] == 1
    assert bridge.latest_scan(0)['points']
    assert bridge.status()['operation'] == 'idle'
```

- [ ] **Step 2: Add failing continuous, unit-change, timeout, and race tests**

Assert that continuous mode publishes only completed circles, increments revisions per circle, preserves the last scan on failure, and that a unit flag change calls `ScanAssembler.reset()` before the next completion. Exercise concurrent `stop_continuous()`, `disconnect()`, and `close()` and assert the worker terminates, `LSTOPH` is attempted, the radar fake closes, and the retained revision remains readable.

- [ ] **Step 3: Run the bridge tests and verify they fail**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test_radar_direct_bridge.py" -v`

Expected: FAIL with `ModuleNotFoundError` for `radar_direct_bridge`.

- [ ] **Step 4: Implement `RadarDirectBridge` connection and operation ownership**

Use one condition-protected state object and one radar worker. Validate COM names with the same explicit `COM[1-9]\d*` policy as the current console, allow only the four specified baud rates, open 8N1 with no flow control, and reject a second operation while another is active. Do not import or call the existing `SerialBridge`.

- [ ] **Step 5: Implement scan sessions, state frames, alarms, events, and complete-scan revisions**

Single and continuous sessions send the five preflight commands, process parser frames, reset only the incomplete assembly on unit change, update four radar status flags, append alarm/error events, and publish immutable JSON-safe complete-scan snapshots. A single session always attempts `LSTOPH` in its cleanup; continuous cleanup does the same.

- [ ] **Step 6: Implement idempotent stop/disconnect/close cleanup**

Stop and disconnect may race but must converge without deadlock. `disconnect()` stops continuous scanning before closing; `close()` is safe to call more than once. Never clear `_latest_scan` or its revision during these paths.

- [ ] **Step 7: Run Task 1 and Task 2 Python tests**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test_radar_direct_*.py" -v`

Expected: all tests PASS with no live serial device.

- [ ] **Step 8: Commit Task 2**

```bash
git add Code/upper-computer-web/local-car/radar_direct_bridge.py Code/upper-computer-web/local-car/tests/test_radar_direct_bridge.py
git commit -m "feat: add independent radar serial lifecycle"
```

### Task 3: Loopback HTTP Integration Without F407 Regression

**Files:**
- Modify: `Code/upper-computer-web/local-car/serve.py:87-206,278-340`
- Create: `Code/upper-computer-web/local-car/tests/test_radar_direct_service.py`
- Test: `Code/upper-computer-web/local-car/tests/test_bridge.py`

**Interfaces:**
- Consumes: `RadarDirectBridge` from Task 2 and the existing `SerialBridge`.
- Produces: `make_server(bridge, root, port=8765, radar_bridge=None)`; when `radar_bridge` is supplied, `server.radar_bridge` serves the new routes without changing existing routes.
- GET routes: `/api/radar-direct/ports`, `/api/radar-direct/status`, `/api/radar-direct/scan?after=N`, `/api/radar-direct/events?after=N`.
- POST routes: `/api/radar-direct/connect`, `/disconnect`, `/start`, `/stop`, `/scan`, `/continuous/start`, `/continuous/stop`.

- [ ] **Step 1: Write failing HTTP routing and authentication tests**

Use fake main and radar bridges. Assert every POST requires the existing same-origin token, GET routes remain read-only, invalid `after` returns 400, and existing `/api/status`, `/api/connect`, `/api/send`, and `/api/stop` responses are byte-for-byte unaffected for the same fake main bridge state.

- [ ] **Step 2: Write the dual-port isolation test from Review Focus**

```python
def test_busy_radar_port_does_not_disconnect_or_advance_main_bridge():
    main.connect('COM7', 115200)
    before = main.status()
    response = post('/api/radar-direct/connect', {'port': 'COM7', 'baudrate': 921600})
    assert response.status == 400
    after = main.status()
    assert after['connected'] is True
    assert after['port'] == 'COM7'
    assert after['last_event_id'] == before['last_event_id']
```

- [ ] **Step 3: Run the service test and confirm the new routes fail**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test_radar_direct_service.py" -v`

Expected: FAIL because `make_server` has no radar bridge and the routes return 404.

- [ ] **Step 4: Extend `ConsoleHandler` with the radar-direct GET and POST dispatch**

Reuse `_local_request`, `_json`, `available_ports`, and the existing auth token from `server.bridge`. Do not add a second token. Map only the listed routes to `server.radar_bridge`; return 503 when the radar service is intentionally unavailable.

- [ ] **Step 5: Wire `RadarDirectBridge` into `make_server()` and `main()` lifecycle**

Keep `radar_bridge=None` compatible with existing tests. In production `main()`, construct the radar bridge, attach it to the server, and close it independently in every server-start failure and final shutdown path before closing the main bridge. Do not add radar commands to the CMD console.

- [ ] **Step 6: Run HTTP, existing bridge, and existing radar service tests**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test*.py" -v`

Expected: all tests PASS; no test opens a physical serial port.

- [ ] **Step 7: Commit Task 3**

```bash
git add Code/upper-computer-web/local-car/serve.py Code/upper-computer-web/local-car/tests/test_radar_direct_service.py
git commit -m "feat: expose isolated direct radar API"
```

### Task 4: Browser Filtering, Grid, Obstacle, Planning, Persistence, and CSV Core

**Files:**
- Create: `Code/upper-computer-web/local-car/radar-direct-model.mjs`
- Create: `Code/upper-computer-web/local-car/radar-direct-planner.mjs`
- Create: `Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs`
- Create: `Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs`

**Interfaces:**
- Consumes: JSON points shaped `{angle_deg, distance_mm, energy}` and scan revisions from Task 3.
- Produces from `radar-direct-model.mjs`: `RADAR_DIRECT_DEFAULTS`, `createDefaultRadarConfig()`, `normalizeRadarConfig(value)`, `filterRadarPoints(points, filter)`, `toFieldPoint(point, radar)`, `createGrid(map)`, `classifyObstacleCells(grid, fieldPoints, threshold, manualMask)`, `scanToCsv(points)`, `loadRadarDirectPreferences(storage)`, and `saveRadarDirectPreferences(storage, config)`.
- Produces from `radar-direct-planner.mjs`: `parseMissionRoute(text)`, `formatMissionRoute(route)`, `resolveMissionPoints(grid, radar)`, `planPointToPoint({grid, blockedMask, start, goal})`, `planMissionRoute({grid, blockedMask, route, radar})`, and `buildWaypointRows(plan)`.
- Uses reference constants exactly: field 2400, active boundaries 150/2250, internal lines 550/1000/1400/1850, minimum cell 50, threshold 3, fixed mask 328000, mission-protected cells R3C1/R5C3/R3C5/R1C3, and route `1-5-4-2-3-4-2-3-1`.

- [ ] **Step 1: Write failing filter, coordinate, grid, and obstacle tests**

Include full 0–360°, 350°→10° cross-zero, inclusive min/max distance and energy, the exact clockwise transform, independent X/Y boundaries, fixed/manual/scanned precedence, counts, protected mission cells, and minimum 50 mm boundary validation.

```javascript
test('clockwise radar coordinates match the supplied host', () => {
  assert.deepEqual(toFieldPoint({angle_deg:0,distance_mm:1000,energy:1},{x:230,y:230}), {x:230,y:1230,source:{angle_deg:0,distance_mm:1000,energy:1}});
  assert.deepEqual(toFieldPoint({angle_deg:90,distance_mm:1000,energy:1},{x:230,y:230}), {x:1230,y:230,source:{angle_deg:90,distance_mm:1000,energy:1}});
});
```

- [ ] **Step 2: Write failing configuration, persistence, and CSV tests**

Assert exact defaults, invalid saved JSON falling back without throwing, manual masks normalized to 25 bits and with protected cells cleared, storage write failure returning a visible error result without mutation, and CSV header/order using only the supplied complete scan.

- [ ] **Step 3: Run model tests and verify they fail**

Run: `node --test Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs`

Expected: FAIL with module-not-found errors.

- [ ] **Step 4: Implement `radar-direct-model.mjs` pure functions**

Port the supplied filter, coordinate transformer, grid definition, obstacle classifier, defaults, CSV format, and local storage semantics. Keep all functions free of DOM and network access so F407 can become a later data source.

- [ ] **Step 5: Run model tests and verify they pass**

Run: `node --test Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs`

Expected: all model tests PASS.

- [ ] **Step 6: Write failing mission-route and point-to-point planner tests**

Pin route grammar (hyphen, English/Chinese comma, whitespace), 2–16 entries, numbers 1–5, start/end at 1, no adjacent duplicate, point 1 at radar center, points 2–5 at protected grid centers, four-neighbor A*, repeated cells/backtracking, first unreachable leg reporting, total distance, cardinal direction, and start/straight/left/right/reverse turns.

- [ ] **Step 7: Implement `radar-direct-planner.mjs`**

Use deterministic four-neighbor traversal and reproduce the supplied mission planner's waypoint and turn semantics. Do not call the existing firmware C planner because this page's source of truth is the supplied direct host.

- [ ] **Step 8: Run both new Node core suites**

Run: `node --test Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs`

Expected: all tests PASS.

- [ ] **Step 9: Commit Task 4**

```bash
git add Code/upper-computer-web/local-car/radar-direct-model.mjs Code/upper-computer-web/local-car/radar-direct-planner.mjs Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs
git commit -m "feat: port direct radar mapping and planning core"
```

### Task 5: Reference-Equivalent Four-Quadrant Canvas

**Files:**
- Create: `Code/upper-computer-web/local-car/radar-direct-map.mjs`
- Create: `Code/upper-computer-web/local-car/tests/radar-direct-map.test.mjs`

**Interfaces:**
- Consumes: a render model with `fieldSize`, `activeMin`, `activeMax`, `xLines`, `yLines`, `radar`, `points`, `cells`, `path`, `missionPoints`, `start`, `goal`, and `theme`.
- Produces: pure helpers `createDefaultViewport(fieldSize)`, `createWorldTransform(fieldSize, left, top, size, viewport)`, `zoomViewport(viewport, anchor, deltaY)`, `panViewport(viewport, startPointer, currentPointer, canvasSize)`, `createCursorReading(point, radar)`, and `isPointVisible(point, viewport)`.
- Produces: `mountRadarDirectMap({canvas, onBoundaryDragged, onCellClicked}) -> {render(model), setTheme(theme), dispose()}`.

- [ ] **Step 1: Port the supplied viewport tests as failing Node tests**

Assert the default four-quadrant span, transform round trips, cursor-centered zoom, pan displacement, minimum/maximum zoom bounds, point visibility, and clockwise cursor angle. Use exact expected values from the supplied `tests/fieldMapViewport.test.mjs`.

- [ ] **Step 2: Run the map tests and verify they fail**

Run: `node --test Code/upper-computer-web/local-car/tests/radar-direct-map.test.mjs`

Expected: FAIL with module-not-found errors.

- [ ] **Step 3: Implement pure viewport and coordinate helpers**

Port the supplied map helper behavior, retaining a default center of `(0, 0)` and half-span `2400` for the 2400 mm field.

- [ ] **Step 4: Add failing interaction tests using a fake canvas/context**

Assert wheel prevents default and zooms around the pointer; a drag beyond the click threshold pans; a short click inside a normal cell invokes `onCellClicked(row, column)` once; dragging an internal line invokes `onBoundaryDragged(axis, internalIndex, roundedMm)`; `dispose()` removes listeners and disconnects the resize observer.

- [ ] **Step 5: Implement the renderer and interactions**

Render the same layers and meanings as the reference: millimetre grid, 2400 mm field, cells and counts, draggable boundary lines and widths/heights, energy-alpha point cloud, route and waypoints, S/G or mission markers, radar marker, compass, and cursor panel. Keep black/white palettes scoped inside this module.

- [ ] **Step 6: Run the complete map suite**

Run: `node --test Code/upper-computer-web/local-car/tests/radar-direct-map.test.mjs`

Expected: all viewport, drawing, and interaction tests PASS.

- [ ] **Step 7: Commit Task 5**

```bash
git add Code/upper-computer-web/local-car/radar-direct-map.mjs Code/upper-computer-web/local-car/tests/radar-direct-map.test.mjs
git commit -m "feat: add interactive four-quadrant radar map"
```

### Task 6: Radar Direct Client, Reference UI, and Tab Isolation

**Files:**
- Create: `Code/upper-computer-web/local-car/radar-direct-client.mjs`
- Create: `Code/upper-computer-web/local-car/radar-direct-view.mjs`
- Modify: `Code/upper-computer-web/local-car/index.html:17-62`
- Modify: `Code/upper-computer-web/local-car/styles.css`
- Modify: `Code/upper-computer-web/local-car/app.mjs:1-13,298-301`
- Modify: `Code/upper-computer-web/local-car/radar-view.mjs:28,70-78,363`
- Create: `Code/upper-computer-web/local-car/tests/radar-direct-client.test.mjs`
- Create: `Code/upper-computer-web/local-car/tests/radar-direct-view.test.mjs`
- Modify: `Code/upper-computer-web/local-car/tests/radar-view.test.mjs`

**Interfaces:**
- Consumes: `api(path, body?, signal?)` compatible with `BridgeLink.api`, Task 4 core functions, and Task 5 map mount.
- Produces: `RadarDirectClient({api, onState, onScan, onEvents, onError, pollDelayMs=250})` with `start()`, `suspend()`, `refresh()`, and `command(action: string, body?: object)`.
- Produces: `mountRadarDirectHost({root, api, storage=localStorage}) -> {setVisible(visible), refresh(), dispose(), snapshot()}`.
- Extends `mountRadarWorkspace` with optional `onTabChange = () => {}`; `selectTab(value)` calls it after updating the legacy tab state.

- [ ] **Step 1: Write failing client revision, reconnect, and no-command-on-start tests**

Assert `start()` performs only GET requests; scan revision 4 cannot be replaced by delayed revision 3; unchanged revisions do not re-download points; a service error marks the client unavailable, retries read-only status, and never replays the last POST; `suspend()` aborts polling without disconnecting radar hardware.

- [ ] **Step 2: Implement `RadarDirectClient`**

Poll lightweight status/events and request a scan only when `scan_revision` advances. `command()` maps only the explicit UI actions to Task 3 POST routes. Keep the client independent of F407 connection state and never call `BridgeLink.send()`.

- [ ] **Step 3: Write the failing reference-surface UI tests**

Using the project's fake DOM pattern, assert presence and labels for port, baud, connect/disconnect, start, stop, single, continuous, save configuration, raw/filtered toggle, filter inputs, radar XY, eight boundary inputs, planning mode, route text/endpoints, threshold, metrics, status flags, route summary/table, 12-point preview, CSV, logs, and the map canvas. Assert that no STM32 mode or Flash/download button exists.

- [ ] **Step 4: Add UI behavior and isolation tests**

Pin these behaviors:

```javascript
test('opening or switching to radar direct sends no hardware command', async () => {
  host.setVisible(true);
  await flush();
  assert.equal(requests.some(request => request.method === 'POST'), false);
});

test('radar direct visibility hides only the legacy map layout', () => {
  workspace.selectTab('radar');
  assert.equal(legacyLayout.hidden, true);
  assert.equal(directRoot.hidden, false);
  workspace.selectTab('monitor');
  assert.equal(legacyLayout.hidden, false);
  assert.equal(directRoot.hidden, true);
});
```

Also assert button disabling during mutually exclusive operations, explicit save behavior, invalid saved config fallback, storage write error display, saved port/baud restoration without automatic connection, raw/filtered toggle leaving obstacle/path objects unchanged, and disconnect retaining the last rendered scan.

- [ ] **Step 5: Add the radar-direct host container without deleting legacy monitor/offline markup**

Give the existing `.map-layout` an `id="legacy-map-layout"`. Add `<section id="radar-direct-workspace" hidden></section>` as its sibling. Keep the old hidden `radar-build-panel` markup because existing monitor/offline controller bindings still reference it; the parent legacy layout is hidden whenever the radar tab is selected, so none of the old radar page is visible.

- [ ] **Step 6: Implement `mountRadarDirectHost` with the reference layout**

Build the supplied host's hero header, serial toolbar, left settings column, center canvas, and right diagnostics column. Recompute filter/map/path immediately from the latest complete scan when local settings change. Save preferences only on the explicit button. Generate CSV from the retained complete raw scan and disable export when no complete scan exists.

- [ ] **Step 7: Scope and port the reference visual style**

Add `.radar-direct-*` CSS only. Reproduce the supplied dark/light panels, status cluster, three-column workspace, controls, metrics, legends, route table, log list, and responsive stacking. Do not change unscoped selectors or existing global theme variables.

- [ ] **Step 8: Integrate tab visibility and app startup**

Import and mount the new host in `app.mjs`; pass `onTabChange` to `mountRadarWorkspace`. Starting the console begins read-only radar status polling but does not connect or send. Page navigation and hash changes only show/hide workspaces. Dispose/abort radar client polling on page unload without sending a hardware stop command.

- [ ] **Step 9: Run focused frontend tests**

Run: `node --test Code/upper-computer-web/local-car/tests/radar-direct-client.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-view.test.mjs Code/upper-computer-web/local-car/tests/radar-view.test.mjs`

Expected: all tests PASS; existing monitor/offline assertions remain unchanged except for the new visibility callback assertions.

- [ ] **Step 10: Run the full Node test suite**

Run: `node --test Code/upper-computer-web/local-car/tests/*.test.mjs`

Expected: all tests PASS.

- [ ] **Step 11: Commit Task 6**

```bash
git add Code/upper-computer-web/local-car/radar-direct-client.mjs Code/upper-computer-web/local-car/radar-direct-view.mjs Code/upper-computer-web/local-car/index.html Code/upper-computer-web/local-car/styles.css Code/upper-computer-web/local-car/app.mjs Code/upper-computer-web/local-car/radar-view.mjs Code/upper-computer-web/local-car/tests/radar-direct-client.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-view.test.mjs Code/upper-computer-web/local-car/tests/radar-view.test.mjs
git commit -m "feat: replace radar tab with direct LDS50C host"
```

### Task 7: Documentation, Full Regression, and Delivery Evidence

**Files:**
- Modify: `README.md`
- Modify: `Code/upper-computer-web/local-car/README.md`
- Create: `docs/开发记录/2026-10-04-LDS50C直连上位机第一阶段.md`
- Do not modify: `docs/README.md`

**Interfaces:**
- Consumes: final behavior and verified test output from Tasks 1–6.
- Produces: current entry-point documentation, dual-port operating instructions, explicit phase boundary, verification record, and the exact remaining USB-UART hardware checklist.

- [ ] **Step 1: Update user-facing instructions**

Document that `start.cmd` still starts one local console service; the main F407 port and direct radar port are selected independently; list supported radar baud rates and exact single/continuous workflows; explain that switching pages does not start/stop hardware; state that the last complete circle remains available after stop/disconnect; describe CSV and saved local settings.

- [ ] **Step 2: Record source provenance and explicit exclusions**

In the dated development record, cite the supplied `Lds50cHost` path and behavioral source files, note that no application-level license file was found, record that the user supplied the prior host for migration, and state that STM32 configuration/download/Flash and F407 USART2 transport are excluded from phase one.

- [ ] **Step 3: Run all Python web-console tests**

Run: `python -m unittest discover -s Code/upper-computer-web/local-car/tests -p "test*.py" -v`

Expected: all tests PASS.

- [ ] **Step 4: Run all Node web-console tests**

Run: `node --test Code/upper-computer-web/local-car/tests/*.test.mjs`

Expected: all tests PASS.

- [ ] **Step 5: Run firmware host regression to prove unrelated project code still passes**

Run: `python Code/tests/run_firmware_tests.py`

Expected: all firmware host tests PASS. This is a regression check only; no firmware source is changed.

- [ ] **Step 6: Start the local console without hardware and inspect the reference-equivalent UI**

Run from the repository root: `python Code/upper-computer-web/local-car/serve.py --no-console --no-browser`

Open `http://127.0.0.1:8765/#map/radar`, verify both radar themes and responsive desktop/narrow layouts, then stop the local process. Expected: the reference controls and three-column layout are present; the UI starts disconnected; other tabs retain their prior layouts; no serial port is opened.

- [ ] **Step 7: Check repository scope and formatting**

Run: `git diff --check`

Run: `git status --short`

Expected: no whitespace errors; only planned implementation/documentation files plus the user's pre-existing unrelated changes appear.

- [ ] **Step 8: Commit documentation and verification record**

```bash
git add README.md Code/upper-computer-web/local-car/README.md docs/开发记录/2026-10-04-LDS50C直连上位机第一阶段.md
git commit -m "docs: document LDS50C direct host workflow"
```

- [ ] **Step 9: Report the hardware boundary accurately**

Report software test totals and UI inspection separately. Do not claim USB-UART hardware validation until the user supplies the actual COM port, baud rate, powered LDS50C, and requests a live scan. Provide the remaining checks: 3.3 V UART/common ground, exact COM/baud, preflight sequence, first full circle within 8 seconds, four status flags, 0°/90° direction, known distances, continuous stop retention, and CSV content.
