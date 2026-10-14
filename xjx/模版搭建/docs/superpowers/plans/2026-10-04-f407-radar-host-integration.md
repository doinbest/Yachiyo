# F407 Radar Host Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make only `#map/radar` use LDS50C data acquired and processed by the F407 over USART2 and returned through the existing USART1/DL-20 connection, while preserving the current radar-page workflow and leaving `#map/monitor` unchanged.

**Architecture:** Add an F1-compatible host profile beside the existing vehicle radar profile. The host profile reuses the production LDS parser, one complete-cloud buffer, and a generalized grid/A* core, but owns separate configuration, result identities, map/path snapshots, commands, and `@RADAR` frames so it cannot replace vehicle-monitor snapshots or start motion. A new browser host model/client consumes only `profile:"f1_host"` frames and drives the existing direct-radar UI through the shared F407 `BridgeLink`; compact map/path results arrive first and a CRC-checked packed cloud follows at low priority.

**Tech Stack:** STM32 HAL C99, existing USART2 DMA and USART1 paced `ConsoleTx`, vanilla ES modules, Canvas 2D, Node `node:test`, Python `unittest`, GCC host tests, Keil MDK Target `template`.

**Spec:** `docs/superpowers/specs/2026-10-04-f407-radar-host-integration-design.md`

## Global Constraints

- Treat `Code/reference/Radar_2026-10-01/Radar` as the authoritative mapping and planning baseline: radar `(230,230)`, clockwise raw angle, `X=230+d*sin(a)`, `Y=230+d*cos(a)`, threshold `1`, fixed mask `0x00050140`, protected R1C1/R3C1/R5C3/R3C5/R1C3, mission `1-5-4-2-3-4-2-3-1`, and the exact 33-point empty-map path.
- Change only the `#map/radar` data source and matching F407 host-profile code. Do not change `#map/monitor` layout, telemetry source, trajectories, buttons, command meanings, or vehicle radar navigation behavior.
- Do not make host-profile paths executable by `radar nav start`, `radar task start`, fixed routes, or any chassis/arm task.
- Keep USART1 at 115200/8N1, `CONSOLE_TX_CHUNK_BYTES=32`, `CONSOLE_TX_GAP_MS=30`, existing urgent/reply priority, and complete line/page boundaries.
- Keep USART2 PA2/PA3 at 921600/8N1 with the existing circular DMA, pins, clocks, IRQs, and IOC; do not run CubeMX.
- Reuse one 4096-point firmware cloud buffer. Do not add a second complete-cloud buffer or publish incomplete circles.
- Keep first-stage `/api/radar-direct/`, `radar_direct_bridge.py`, and direct-parser source available but unused by the current radar page; do not delete them in this change.
- Store host configuration only in F407 RAM; browser persistence remains an explicit local draft/theme save and never proves device application.
- Do not modify EIDE configuration or build EIDE. Update only the Keil file list for new F407 modules.
- Preserve the user's unrelated dirty `docs/README.md` and `docs/项目总结与AI交接.md` changes and exclude them from task commits.

## Review Focus

- Exact F1 tie-breaking is easy to disturb: default projection, protected masks, all 33 row/column pairs, distances, directions, turns, reroute, and first unreachable leg must match the reference; Tasks 1 and 2 pin this.
- A host scan must share USART2 without replacing the legacy vehicle map/path or leaking host frames into `RadarModel`; bad frames, unit changes, DMA loss, stop, and timeout must only restart/end the active host circle; Tasks 3 and 7 pin this.
- A partially sent, invalid, stale, or interrupted configuration batch must leave the active configuration and completed map unchanged, while a valid apply must rebuild the same frozen scan without starting the radar; Task 4 pins this.
- Missing pages, mixed identities, Base64 corruption, CRC mismatch, cloud cancellation, and a newer scan during download must never publish a partial cloud or delay stop/ordinary replies behind an unbounded frame; Tasks 5 and 6 pin this.
- Page mount, tab switching, reconnect, local save, and shared-port refresh must send no radar command automatically; only `#map/radar` changes, and `#map/monitor` must retain its DOM, sources, commands, and snapshots; Task 7 pins this.

---

### Task 1: Correct the Browser F1 Baseline Before Changing Its Data Source

**Files:**
- Modify: `Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs`
- Modify: `Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs`
- Modify: `Code/upper-computer-web/local-car/radar-direct-model.mjs`

**Interfaces:**
- Consumes: the existing phase-one browser configuration and planner APIs.
- Produces: `RADAR_DIRECT_DEFAULTS.map.obstaclePointThreshold === 1`; `MISSION_PROTECTED_MASK` containing R1C1, R3C1, R5C3, R3C5, and R1C3; unchanged public functions `createDefaultRadarConfig`, `normalizeRadarConfig`, `classifyObstacleCells`, and `planMissionRoute`.

- [ ] **Step 1: Change the default/protection tests first**

Update the expected default threshold from `3` to `1`. Add assertions that the protected mask contains bit 0 for R1C1 and that points/manual bits in R1C1 retain their count but never make it blocked.

- [ ] **Step 2: Add the exact 33-cell F1 mission-path expectation**

Assert the default plan produces these row and column sequences and starts/ends at `(230,230)`:

```javascript
const rows = [1,1,1,2,3,3,3,3,3,3,3,4,5,5,5,5,5,4,3,3,3,3,3,4,5,5,5,4,3,2,1,1,1];
const columns = [1,2,3,3,3,4,5,4,3,2,1,1,1,2,3,4,5,5,5,4,3,2,1,1,1,2,3,3,3,3,3,2,1];
assert.deepEqual(plan.cells.map(cell => cell.row), rows);
assert.deepEqual(plan.cells.map(cell => cell.column), columns);
assert.deepEqual(plan.waypoints[0], {xmm:230, ymm:230, row:1, column:1});
assert.deepEqual(plan.waypoints.at(-1), {xmm:230, ymm:230, row:1, column:1});
```

- [ ] **Step 3: Run the focused tests and verify RED**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs
```

Expected: FAIL because the current threshold is `3` and R1C1 is not protected. If the exact route test already passes, retain it as a characterization test and verify the two default/protection failures are genuine.

- [ ] **Step 4: Implement only the approved default and protection changes**

Set `obstaclePointThreshold: 1` and add `bitFor(1,1)` to `MISSION_PROTECTED_MASK`. Do not change coordinates, grid lines, fixed mask, mission order, A* neighbor order, or page styling.

- [ ] **Step 5: Verify GREEN and the complete direct-radar model suite**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-map.test.mjs
```

Expected: PASS.

- [ ] **Step 6: Commit Task 1**

```text
git add Code/upper-computer-web/local-car/radar-direct-model.mjs Code/upper-computer-web/local-car/tests/radar-direct-model.test.mjs Code/upper-computer-web/local-car/tests/radar-direct-planner.test.mjs
git commit -m "fix: align radar host defaults with F1"
```

### Task 2: Add a Shared Grid/A* Core and Pure F1 Host Mapping Module

**Files:**
- Modify: `Code/template/App/radar_map.h`
- Modify: `Code/template/App/radar_map.c`
- Create: `Code/template/App/radar_host_map.h`
- Create: `Code/template/App/radar_host_map.c`
- Modify: `Code/tests/test_radar_map.c`
- Create: `Code/tests/test_radar_host_map.c`
- Modify: `Code/tests/run_firmware_tests.py`

**Interfaces:**
- Consumes: `RadarSample_t` complete clouds and the existing 25-cell map representation.
- Produces in `radar_map.h`: `RadarGrid_t`, `RadarGrid_Default`, `RadarGrid_Valid`, `RadarGrid_FindCell`, `RadarGrid_CellCenter`, and `RadarGrid_Path`; existing `RadarMap_*` and vehicle `RadarPlan_*` behavior remains byte-for-byte compatible at the API level.
- Produces in `radar_host_map.h`: `RadarHostMap_Config_t`, `RadarHostMap_Endpoint_t`, `RadarHostMap_Waypoint_t`, `RadarHostMap_Result_t`, `RadarHostMap_Defaults`, `RadarHostMap_ConfigValid`, and `RadarHostMap_Build(const RadarHostMap_Config_t *, const RadarSample_t *, uint16_t, RadarHostMap_Result_t *)`.

- [ ] **Step 1: Write generic-grid regression tests before exposing the helpers**

Extend `test_radar_map.c` to prove that the new default grid helper gives the same cells, centers, paths, legacy vehicle plan, masks, and current contour-map results as the pre-refactor implementation. Include dynamic X/Y boundaries with the 50 mm minimum and endpoint `2250` handling.

- [ ] **Step 2: Add failing F1 host-map tests**

Create `test_radar_host_map.c` and pin:

- default `(230,230)`, zero line equivalent to `X=230+d*sin(a), Y=230+d*cos(a)`;
- angle 0/90/180/270 projections;
- full 0..3600 tenths filter, distance 100..40000, energy 0..255;
- threshold 1 and fixed mask `0x00050140`;
- protected R1C1/R3C1/R5C3/R3C5/R1C3 with counts retained;
- manual-mask normalization excluding protected/fixed-out-of-range bits;
- the exact 33 rows/columns from Task 1;
- first distance `558`, second distance `425`, direction/turn checks matching F1;
- reroute around R3C4 and trapped first leg reported as `1 -> 5`;
- point-to-point anchors and dynamic boundaries used by the current page.

- [ ] **Step 3: Register the new host test and verify RED**

Add `build_test("test_radar_host_map", "template/App/radar_host_map.c", "template/App/radar_map.c")` after `test_radar_map` in `run_firmware_tests.py`.

Run: `python Code/tests/run_firmware_tests.py`

Expected: the existing suite reaches the new test and fails because the new headers/functions do not exist.

- [ ] **Step 4: Generalize only the grid/A* internals in `radar_map.c`**

Move the current six boundaries, find-cell, center, Manhattan cost, four-neighbor order `{north,east,south,west}`, deterministic open selection, and 25-node path reconstruction behind `RadarGrid_*`. Keep the existing `RadarMap_*` and `RadarPlan_*` functions as wrappers using the current vehicle grid and route behavior; do not change current vehicle anchors, diagonal shortcut, return staging, station/visit fields, or contour filtering.

- [ ] **Step 5: Implement the pure F1 host mapper/planner**

In `radar_host_map.c`:

- validate the complete configuration without HAL or global state;
- filter and count every accepted complete-cloud point using the F1 projection and dynamic grid;
- form `effective_mask = fixed | manual | scanned`, then clear protected non-fixed cells while retaining counts;
- use only `RadarGrid_Path` for both mission and point-to-point planning;
- emit every expanded cell without collinear compression;
- replace only the first and last mission coordinates with `(radar_x,radar_y)`;
- calculate Euclidean segment distance rounded to the nearest integer and F1 direction/turn enums;
- leave the output zeroed and report the first failed numbered leg when planning fails.

- [ ] **Step 6: Run focused and full firmware host tests**

Run:

```text
python Code/tests/run_firmware_tests.py
```

Expected: all existing vehicle radar tests and the new F1 host-map test PASS with `-Wall -Wextra -Werror`.

- [ ] **Step 7: Commit Task 2**

```text
git add Code/template/App/radar_map.c Code/template/App/radar_map.h Code/template/App/radar_host_map.c Code/template/App/radar_host_map.h Code/tests/test_radar_map.c Code/tests/test_radar_host_map.c Code/tests/run_firmware_tests.py
git commit -m "feat: add F1 radar host mapping core"
```

### Task 3: Give the Host Profile Separate Scan Publication Without a Second Cloud

**Files:**
- Modify: `Code/template/App/radar_scan.h`
- Modify: `Code/template/App/radar_scan.c`
- Modify: `Code/tests/test_radar_scan.c`

**Interfaces:**
- Consumes: the existing `RadarUart` event stream, one `cloud[4096]`, and existing single/live capture mutual exclusion.
- Produces: `RadarScan_HostStatus_t`; `RadarScan_HostStart(const RadarMap_Params_t *)`; `RadarScan_HostStop()`; `RadarScan_HostStatusGet`; `RadarScan_HostPoints(uint16_t *)`; `RadarScan_HostLiveStart(const RadarMap_Params_t *)`; `RadarScan_HostLiveStop`; and `RadarScan_HostLiveCopy` using the existing 120 bins.
- Existing `RadarScan_Start*`, `RadarScan_Map`, `RadarScan_MapParams`, `RadarScan_Points`, `RadarScan_Live*`, and vehicle map IDs keep their contracts.

- [ ] **Step 1: Add failing ownership/isolation tests to `test_radar_scan.c`**

Cover this sequence with the existing fake LDS stream:

1. publish a legacy vehicle map and remember its map ID, parameters, mask, and path inputs;
2. start and complete a host scan;
3. verify the host exposes the complete cloud and a new host scan ID;
4. verify the legacy published map, parameters, map ID, start result, and legacy live state are unchanged;
5. verify legacy `RadarScan_Points` never returns the host-owned cloud under an old vehicle identity.

Also pin host bad-frame/DMA-loss/unit-change recovery, 10-second timeout, explicit stop, command failure, truncation, and no fixed retry exhaustion.

- [ ] **Step 2: Verify RED with the focused firmware test**

Run the same compile line used by `run_firmware_tests.py` for `test_radar_scan`, or run the complete firmware suite.

Expected: FAIL because host scan APIs and owner isolation do not exist.

- [ ] **Step 3: Refactor the shared acquisition state around an explicit owner**

Add an internal `RadarScan_Owner_t` with none/vehicle/host and route publication as follows:

- all owners reuse the existing parser events, wrap detection, recovery counters, 10-second single-scan window, command sequence, and one cloud array;
- vehicle completion follows the current `publish()` path unchanged;
- host completion increments only host scan identity/status and exposes the complete cloud without calling `publish_map`, `RadarStart_*`, or changing vehicle IDs;
- beginning either single capture invalidates the firmware view of the previous cloud owner but leaves completed maps/paths in their owning modules;
- live capture retains one shared 120-bin workspace and tags its owner so only the matching console can publish it.

- [ ] **Step 4: Preserve stop/recovery behavior for both owners**

Make host stop and live stop request `LSTOPH`, clear only incomplete acquisition state, and retain the last completed host map/path in the later host service. Keep legacy `RadarScan_Stop` behavior unchanged for vehicle commands.

- [ ] **Step 5: Run firmware tests and inspect memory-sensitive declarations**

Run: `python Code/tests/run_firmware_tests.py`

Expected: PASS. Confirm by source inspection that there remains exactly one `RadarSample_t[4096]` and one pair of 120-bin live arrays.

- [ ] **Step 6: Commit Task 3**

```text
git add Code/template/App/radar_scan.c Code/template/App/radar_scan.h Code/tests/test_radar_scan.c
git commit -m "feat: isolate F407 radar host captures"
```

### Task 4: Add the F407 Host Service, Transactional RAM Configuration, and Same-Circle Rebuild

**Files:**
- Create: `Code/template/App/radar_host.h`
- Create: `Code/template/App/radar_host.c`
- Create: `Code/tests/test_radar_host.c`
- Modify: `Code/tests/run_firmware_tests.py`

**Interfaces:**
- Consumes: Task 2 `RadarHostMap_*` and Task 3 `RadarScan_Host*` APIs.
- Produces: `RadarHost_Init`, `RadarHost_Process`, `RadarHost_StartRotation`, `RadarHost_Stop`, `RadarHost_Scan`, `RadarHost_LiveStart`, `RadarHost_LiveStop`, `RadarHost_StatusGet`, `RadarHost_ActiveConfig`, `RadarHost_DraftConfig`, `RadarHost_DraftReset`, `RadarHost_DraftSet(const char *key, const char *value)`, `RadarHost_Apply(uint32_t draft_revision)`, `RadarHost_Map`, `RadarHost_Path`, `RadarHost_Preview`, and `RadarHost_Cloud`.
- `RadarHost_Status_t` carries session/config/draft/scan/map/plan IDs, state/reason, raw/filtered point totals, parser/recovery counters, and active/live flags.

- [ ] **Step 1: Write a fake-scan host-service test first**

Use test fakes for the `RadarScan_Host*` functions and assert:

- defaults exactly match F1 and Task 1;
- `DraftReset` copies active config and returns a new nonzero draft revision;
- scalar keys cover `angle_min`, `angle_max`, `distance_min`, `distance_max`, `energy_min`, `energy_max`, `radar_x`, `radar_y`, `x1..x4`, `y1..y4`, `threshold`, `manual_mask`, `planning_mode`, compact hyphenated `mission`, and compact `row,column,anchor` start/goal endpoints;
- bad ranges, cells below 50 mm, invalid route text, protected manual bits, stale revision, and interrupted drafts leave active config/map/path unchanged;
- valid apply increments config/map/plan versions and rebuilds the same host scan ID without calling `RadarScan_HostStart`;
- apply before any complete cloud updates config but leaves map/path absent;
- new complete scans rebuild automatically, while scan failure retains the last complete host map/path as historical service results;
- no host service function calls chassis, route, grab, localization, or legacy radar navigation APIs.

- [ ] **Step 2: Register the test and verify RED**

Add `test_radar_host` to `run_firmware_tests.py`, compiling `radar_host.c`, `radar_host_map.c`, and `radar_map.c` with a host-scan stub include path.

Run: `python Code/tests/run_firmware_tests.py`

Expected: FAIL because `radar_host` does not exist.

- [ ] **Step 3: Implement active/draft configuration and identities**

Use fixed-size C structures only. Parse one key/value per call so the eventual command stays within 63 bytes and six tokens. Normalize manual masks by the 25-bit field and protected mask. Keep active configuration unchanged until a complete `RadarHostMap_ConfigValid` succeeds for the requested draft revision.

- [ ] **Step 4: Implement scan/live orchestration and rebuild**

`RadarHost_Process` observes Task 3 status changes, builds a new `RadarHostMap_Result_t` only from a newly completed non-truncated cloud, and preserves the last result on stop/error/timeout. A same-circle configuration apply calls only `RadarHostMap_Build`; it must not call scan start, radar commands, or any motion function.

- [ ] **Step 5: Run the firmware suite**

Run: `python Code/tests/run_firmware_tests.py`

Expected: PASS, including active/draft isolation and no-rescan rebuild assertions.

- [ ] **Step 6: Commit Task 4**

```text
git add Code/template/App/radar_host.c Code/template/App/radar_host.h Code/tests/test_radar_host.c Code/tests/run_firmware_tests.py
git commit -m "feat: manage F407 radar host snapshots"
```

### Task 5: Expose the Host Service Through Bounded USART1 Pages

**Files:**
- Create: `Code/template/App/radar_host_console.h`
- Create: `Code/template/App/radar_host_console.c`
- Modify: `Code/template/App/radar_console.c`
- Modify: `Code/template/App/radar_console.h`
- Modify: `Code/template/MDK-ARM/template.uvprojx`
- Create: `Code/tests/radar_host_console_test.c`
- Modify: `Code/tests/radar_console_test.c`
- Modify: `Code/tests/run_firmware_tests.py`

**Interfaces:**
- Consumes: Task 4 host service and existing `ConsoleTx_Write`, `ConsoleTx_Bulk`, and `ConsoleTx_BulkReady`.
- Produces: `RadarHostConsole_Init`, `RadarHostConsole_Process`, and `RadarHostConsole_Command(unsigned count, char *tokens[])` delegated from `RadarConsole_Command` when tokens begin `radar host`.
- Commands stay within current limits and use these exact shapes:
  - `radar host status`
  - `radar host get <active|draft>`
  - `radar host start|stop|scan`
  - `radar host live <start|stop|status>`
  - `radar host draft reset`
  - `radar host set <key> <compact-value>`
  - `radar host apply <draft-revision>`
  - `radar host fetch <config|map|path|preview|cloud>:<id> <page>`
- Produces `@RADAR` JSON with `profile:"f1_host"` and `k` values `host_status`, `host_config`, `host_map`, `host_path`, `host_preview`, and `host_cloud`.

- [ ] **Step 1: Write failing console grammar and priority tests**

In `radar_host_console_test.c`, fake the host service and capture `ConsoleTx` pages. Assert:

- every command shape above is accepted only at exact token counts;
- all malformed IDs/pages/compact routes/endpoints/extra tokens are rejected with a specific `ERR radar host ...` reason;
- status/get/fetch/live-status do not mutate service state;
- stop and live-stop are accepted during active transfer; an already-started or queued complete host page may drain, no later host page is scheduled, and the urgent stop reply wins at the next page boundary without cancelling unrelated legacy bulk output;
- config/map/path appear before any automatic cloud page;
- every emitted CRLF-terminated page is at most 256 bytes even with `UINT32_MAX` identities and maximum counts;
- ordinary and urgent replies can be selected at the next completed host page boundary;
- legacy `radar status`, `radar fetch ...`, live, debug, probe, navigation, and task tests remain unchanged.

- [ ] **Step 2: Add failing packed-cloud tests**

Pin the wire format as little-endian five-byte points `[angle_tenths u16, distance_mm u16, energy u8]`, Base64 without whitespace, metadata total count, and IEEE CRC32 over the complete packed byte sequence. Verify zero, one, typical, and 4096-point pagination and that a page never contains a partial five-byte point.

- [ ] **Step 3: Register and run the focused tests for RED**

Add the new console test to `run_firmware_tests.py`. Run the suite and confirm failure because the console module is missing.

- [ ] **Step 4: Implement bounded metadata/data pagination**

Build every line in a 257-byte buffer and calculate page capacity from the worst-case JSON header before packing data. Keep all frames newline-terminated and feed only complete pages to the low-priority bulk slot. Include session, config, scan, map, plan, page, pages, offset, and total identity on every transfer page so the browser can reject mixed transfers. The final emitted form must satisfy the 256-byte test by reducing payload capacity rather than dropping identity fields or assuming a fixed item count.

- [ ] **Step 5: Delegate host commands without changing legacy output**

Initialize/process the new host console from `RadarConsole_Init/Process`, and delegate `radar host` before legacy subcommand handling. Do not set legacy `Watching`, `SeenMap`, `Plan`, `PlanMap`, navigation state, or legacy pending-page state from host commands.

- [ ] **Step 6: Add the six new C/H files to the Keil Radar group**

Add `radar_host_map.c/.h`, `radar_host.c/.h`, and `radar_host_console.c/.h` to `template.uvprojx`. Do not alter target, device, memory, compiler, include paths, IOC, or EIDE.

- [ ] **Step 7: Run firmware tests**

Run: `python Code/tests/run_firmware_tests.py`

Expected: PASS, with legacy and host console pages independently bounded.

- [ ] **Step 8: Commit Task 5**

```text
git add Code/template/App/radar_host_console.c Code/template/App/radar_host_console.h Code/template/App/radar_console.c Code/template/App/radar_console.h Code/template/MDK-ARM/template.uvprojx Code/tests/radar_host_console_test.c Code/tests/radar_console_test.c Code/tests/run_firmware_tests.py
git commit -m "feat: stream F407 radar host results"
```

### Task 6: Add the Browser Host Snapshot Model, Transfer Client, and Bridge Classification

**Files:**
- Create: `Code/upper-computer-web/local-car/radar-host-model.mjs`
- Create: `Code/upper-computer-web/local-car/radar-host-client.mjs`
- Create: `Code/upper-computer-web/local-car/tests/radar-host-model.test.mjs`
- Create: `Code/upper-computer-web/local-car/tests/radar-host-client.test.mjs`
- Create: `Code/tests/radar_host_wire_contract_test.py`
- Modify: `Code/tests/run_firmware_tests.py`
- Modify: `Code/upper-computer-web/local-car/bridge.mjs`
- Modify: `Code/upper-computer-web/local-car/serial_bridge.py`
- Modify: `Code/upper-computer-web/local-car/tests/test_bridge.py`
- Modify: `Code/upper-computer-web/local-car/tests/test_radar_service.py`

**Interfaces:**
- Consumes: Task 5 `@RADAR profile=f1_host` lines and the existing async `send(wire)` callback.
- Produces: `parseRadarHost`, `RadarHostModel.accept(line, now)`, `RadarHostModel.connection(value)`, and immutable completed `config/map/path/preview/cloud` snapshots; `RadarHostClient({send, model, onState, onSnapshot, onTransfer, onError})` with `receive`, `connection`, `command`, `applyConfig`, `fetch`, and `cancelTransfer`.
- Host cloud snapshots expose browser points as `{angle_deg, distance_mm, energy}` only after all pages, identity, point count, and CRC pass.

- [ ] **Step 1: Write failing host-model assembly tests**

Cover:

- accepting only `profile:"f1_host"` and known host `k` values;
- out-of-order config/map/path pages assembled by common identity;
- duplicate pages idempotent;
- mixed session/config/scan/map/plan IDs rejected without replacing the previous completed snapshot;
- packed cloud decoding, exact five-byte boundaries, Base64 failure, total mismatch, CRC mismatch, missing pages, and a newer scan superseding an unfinished transfer;
- disconnect/reconnect/session change preserving only completed snapshots and marking them historical;
- map/path completion independent from cloud failure.

- [ ] **Step 2: Write failing client sequencing tests**

Assert these exact behaviors:

- mount/connection/tab changes send nothing;
- explicit start/stop/scan/live actions map to the Task 5 commands;
- scan completion fetches map then path, exposes both, and only then starts background cloud pages;
- a newer scan cancels the old cloud transfer and fetches new compact results;
- `cancelTransfer` sends no stop command;
- `applyConfig` sends `draft reset`, waits for the matching draft identity, sends one `radar host set key compact-value` per field, sends `apply <revision>`, and resolves only after an active config readback with matching version;
- failed/stale apply retains page draft and completed device results;
- commands satisfy `frameCommand`'s 63-byte/six-token limit.

- [ ] **Step 3: Verify RED**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/radar-host-model.test.mjs Code/upper-computer-web/local-car/tests/radar-host-client.test.mjs
```

Expected: FAIL because the modules do not exist.

- [ ] **Step 4: Implement the host model and transfer client**

Use browser/Node built-ins only. Keep partial pages in private pending maps, implement CRC32 in the model, and never mutate a completed snapshot until a replacement is fully verified. Keep map/path fetch foreground and cloud fetch cancellable/background.

- [ ] **Step 5: Classify host read/stop commands in both bridge layers**

Update JavaScript and Python command classifiers so:

- `radar host status`, `get ...`, `live status`, and `fetch kind:id page` are read-only during stop latch;
- `radar host stop` and `radar host live stop` are scoped priority stops;
- start/scan/live-start/draft/set/apply remain explicit mutations and are rejected by the bridge stop latch;
- host `@RADAR` pages remain transport events and do not flood JSONL/terminal history.

- [ ] **Step 6: Add the real-C-to-real-JS wire contract**

Run `radar_host_console_test.exe`, feed its actual host pages into `RadarHostModel`, and assert the F1 mask, 25 counts, 33 path points, `(230,230)` endpoints, config threshold 1, protected mask, packed cloud points, CRC, and all line lengths `<=256`.

- [ ] **Step 7: Run focused and bridge tests**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/radar-host-model.test.mjs Code/upper-computer-web/local-car/tests/radar-host-client.test.mjs
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p test_bridge.py
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p test_radar_service.py
python Code/tests/radar_host_wire_contract_test.py
```

- [ ] **Step 8: Commit Task 6**

```text
git add Code/upper-computer-web/local-car/radar-host-model.mjs Code/upper-computer-web/local-car/radar-host-client.mjs Code/upper-computer-web/local-car/tests/radar-host-model.test.mjs Code/upper-computer-web/local-car/tests/radar-host-client.test.mjs Code/upper-computer-web/local-car/bridge.mjs Code/upper-computer-web/local-car/serial_bridge.py Code/upper-computer-web/local-car/tests/test_bridge.py Code/upper-computer-web/local-car/tests/test_radar_service.py Code/tests/radar_host_wire_contract_test.py Code/tests/run_firmware_tests.py
git commit -m "feat: receive F407 radar host snapshots"
```

### Task 7: Switch Only `#map/radar` to the Shared F407 Client

**Files:**
- Modify: `Code/upper-computer-web/local-car/radar-direct-view.mjs`
- Modify: `Code/upper-computer-web/local-car/app.mjs`
- Modify: `Code/upper-computer-web/local-car/index.html`
- Modify: `Code/upper-computer-web/local-car/tests/radar-direct-view.test.mjs`
- Modify: `Code/upper-computer-web/local-car/tests/radar-view.test.mjs`
- Modify: `Code/upper-computer-web/local-car/tests/page-navigation.test.mjs`
- Modify if needed for scoped states only: `Code/upper-computer-web/local-car/styles.css`

**Interfaces:**
- Consumes: Task 6 `RadarHostClient`, shared `BridgeLink`, the existing direct-radar canvas/model helpers, and the unchanged `mountRadarWorkspace` used by monitor/offline.
- Produces: `mountRadarDirectHost({root, storage, hostClient, connectionController, mapFactory})`; public methods `receive(line)`, `connection(snapshot)`, `setVisible`, `refresh`, `dispose`, and `snapshot`.
- `connectionController` exposes `listPorts()`, `connect(port, 115200)`, `disconnect()`, and current `{state, port, baudrate}` without creating another serial owner.

- [ ] **Step 1: Rewrite view tests first for the shared-port contract**

Change the harness so there is no `/api/radar-direct/` or `RadarDirectClient`. Assert:

- the radar page lists the same mocked global ports, fixes baud to 115200, and calls the shared connect/disconnect controller;
- no connection, radar, config, fetch, or stop command is sent on mount, visibility changes, tab changes, port refresh, reconnect, theme change, or local save;
- explicit scan controls call only host-client actions;
- applying filter/map settings calls `hostClient.applyConfig` and does not locally replace device mask/path before matching readback;
- device map/path render before cloud completion;
- raw/filtered display uses preview until a matching full cloud arrives, then CSV becomes available;
- R1C1 is rendered protected and cannot be manually toggled;
- disconnect retains the last complete device result as historical.

- [ ] **Step 2: Add monitor-isolation tests before integration**

In `radar-view.test.mjs`, snapshot monitor DOM/source state, feed representative `profile:"f1_host"` frames, and assert:

- legacy `RadarModel.snapshots`, green device path, vehicle source, navigation status, and execution controls do not change;
- selecting `radar` then `monitor` sends no command;
- existing legacy `@RADAR` frames still update monitor/offline exactly as before.

- [ ] **Step 3: Run the view tests and verify RED**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/radar-direct-view.test.mjs Code/upper-computer-web/local-car/tests/radar-view.test.mjs Code/upper-computer-web/local-car/tests/page-navigation.test.mjs
```

Expected: FAIL because the radar page still calls the independent direct API/client.

- [ ] **Step 4: Adapt the existing radar page without rebuilding its DOM**

Keep current IDs, panels, canvas, responsive layout, themes, filters, boundary dragging, manual cells, route table, diagnostics, logs, and CSV. Replace local authoritative obstacle/path computation with device snapshots; local point filtering remains display-only. During transfer show raw/filtered totals reported by F407 separately from the number of preview points currently drawn.

- [ ] **Step 5: Wire the page to the shared bridge in `app.mjs`**

Extract reusable global connect/disconnect/list-port functions from the existing sidebar handlers and pass them through `connectionController`. Forward every received line to the host client first; only lines not accepted as `profile:"f1_host"` go to the legacy `radarView.receive`. On global connection changes, update both views, but do not add any automatic radar query.

- [ ] **Step 6: Preserve tab ownership exactly**

Keep `legacy-map-layout` visible for monitor/offline and the direct-style workspace visible only for radar. Do not move or edit monitor controls. Bump only the radar-related query version in `index.html` so the browser loads the new modules/styles.

- [ ] **Step 7: Run the radar UI and complete web suite**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/*.test.mjs
```

Expected: PASS, including the unchanged monitor/offline tests.

- [ ] **Step 8: Commit Task 7**

```text
git add Code/upper-computer-web/local-car/radar-direct-view.mjs Code/upper-computer-web/local-car/app.mjs Code/upper-computer-web/local-car/index.html Code/upper-computer-web/local-car/styles.css Code/upper-computer-web/local-car/tests/radar-direct-view.test.mjs Code/upper-computer-web/local-car/tests/radar-view.test.mjs Code/upper-computer-web/local-car/tests/page-navigation.test.mjs
git commit -m "feat: route radar page through F407"
```

### Task 8: Document, Build, and Verify the Complete Second Stage

**Files:**
- Modify: `Code/upper-computer-web/local-car/README.md`
- Modify: `docs/上位机协议.md`
- Modify: `AGENTS.md`
- Create: `docs/开发记录/2026-10-04-F407雷达建图上位机第二阶段.md`
- Do not modify: `docs/README.md`

**Interfaces:**
- Consumes: the completed firmware/web behavior and all verification output.
- Produces: current operating instructions, exact host commands/frame fields, bandwidth behavior, wiring/validation steps, and an evidence-based development record.

- [ ] **Step 1: Update current documentation**

Document:

- `#map/radar` now follows the shared F407 USART1 connection at 115200;
- USART2 remains 921600 and F407 owns complete-circle processing;
- F1 defaults `(230,230)`, threshold 1, protected R1C1 and other task cells;
- exact `radar host ...` commands and `profile:"f1_host"` frames;
- map/path-first and background packed-cloud behavior;
- same-circle re-filter/repartition without rescanning;
- RAM-only device config versus explicit browser local save;
- no automatic motion and no change to monitor/offline;
- first-stage direct code remains present but is no longer the current radar-page data source.

- [ ] **Step 2: Run format and diff checks**

Run:

```text
git diff --check
git status --short
```

Expected: no whitespace errors; only task files plus the user's pre-existing unrelated docs changes appear.

- [ ] **Step 3: Run the complete firmware regression**

Run: `python Code/tests/run_firmware_tests.py`

Expected: PASS. Save logs under `.embeddedskills/` according to project convention; do not commit generated logs.

- [ ] **Step 4: Run all web and bridge regressions**

Run:

```text
node --test Code/upper-computer-web/local-car/tests/*.test.mjs
python -m unittest discover -s Code/upper-computer-web/local-car/tests -p test_bridge.py
```

Expected: PASS.

- [ ] **Step 5: Build the Keil target**

Use the `keil` skill to build `Code/template/MDK-ARM/template.uvprojx`, Target `template`.

Expected: 0 errors and 0 warnings. Do not build EIDE and do not flash hardware in this task.

- [ ] **Step 6: Run a local browser isolation check**

Start the existing local console, open `http://127.0.0.1:8765/#map/radar`, and verify without connecting hardware:

- radar page shows the shared main port and fixed 115200 baud;
- no network call targets `/api/radar-direct/`;
- opening and switching tabs sends no serial command;
- wide/narrow layouts and black/white radar themes remain usable;
- switching to `#map/monitor` restores the unchanged monitor layout, telemetry source labels, path layers, and controls.

Record that this is a UI/software check, not an actual USART2/DL-20/radar validation.

- [ ] **Step 7: Commit documentation and final verified state**

```text
git add Code/upper-computer-web/local-car/README.md docs/上位机协议.md AGENTS.md docs/开发记录/2026-10-04-F407雷达建图上位机第二阶段.md
git commit -m "docs: describe F407 radar host workflow"
```

- [ ] **Step 8: Review the complete branch before hardware work**

Review the diff from `acfb9bc` through `HEAD`, with special attention to the five Review Focus items. Report test counts, Keil ROM/RAM deltas, unverified hardware assumptions, and the exact first on-board steps. Do not claim real radar, Zigbee throughput, or vehicle acceptance until those are separately performed.
