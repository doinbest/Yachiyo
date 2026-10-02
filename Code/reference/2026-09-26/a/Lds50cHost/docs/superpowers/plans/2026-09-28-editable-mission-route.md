# Editable Mission Route Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an editable, persistent multi-stop mission route whose default order is `1-5-4-2-3-4-2-3-1`, whose task-point coordinates follow the radar and movable grid, and whose expanded route supports revisits, backtracking, display, diagnostics, and STM32 download.

**Architecture:** Keep single-leg four-neighbour A* as the primitive, add a mission-point catalog and a multi-leg orchestrator in `Lds50cHost.Core`, then let `ApplicationState` choose point-to-point or mission planning. Classify points 2–5 as protected task cells while retaining their raw point counts, and send STM32 the expanded path using the unchanged path-payload v1 wire layout with a larger validated entry limit.

**Tech Stack:** .NET 10 (`net10.0-windows`), C#, ASP.NET Core Blazor Interactive Server, HTML canvas JavaScript, repository-local reflection-based xUnit-compatible test runners, Node's built-in test runner, PowerShell.

**Spec:** [`docs/superpowers/specs/2026-09-28-editable-mission-route-design.md`](../specs/2026-09-28-editable-mission-route-design.md)

## Global Constraints

- Default planning mode is mission route with exact sequence `1-5-4-2-3-4-2-3-1`.
- Mission sequences contain 2–16 identifiers, use only 1–5, start and end at 1, and reject consecutive duplicate identifiers.
- Point 1 is the current radar coordinate in R1C1; points 2, 3, 4, and 5 are the current centres of R3C1, R5C3, R3C5, and R1C3 respectively.
- Grid movement and radar-coordinate edits must recompute mission coordinates and route immediately; coordinates are derived and are never persisted independently.
- Movement remains four-neighbour only. Concatenation removes only the duplicate joint between adjacent legs and never globally deduplicates cells.
- Cells for points 2–5 are protected from manual and scanned blocking. Their point-cloud returns and scanned-point counts remain visible.
- The four fixed yellow cells remain blocked and unchanged.
- Preserve ordinary point-to-point planning and its saved start/goal selections.
- Keep STM32 path payload version 1. Allow at most 409 expanded entries so its payload remains at or below 4096 bytes.
- Add no new external packages and do not change radar parsing, filtering, scan acquisition, or coordinate-transform formulas.
- Do not stage or commit the existing untracked `.dotnet-cli/` directory or generated `artifacts/` output.
- Make product-file edits with `apply_patch`, follow RED→GREEN TDD for each behavior task, and commit each task only after its focused suite passes.

## Review Focus

1. **Legacy local configuration:** A pre-feature JSON file has no planning-mode or mission-route properties; Task 3 must load it with mission defaults instead of discarding otherwise valid map/filter settings.
2. **Route text edge cases:** Mixed supported separators should normalize, while letters, identifiers outside 1–5, consecutive duplicates, non-1 endpoints, fewer than 2 values, and more than 16 values must preserve the last valid route; Task 1 and Task 5 pin these cases.
3. **Return-to-radar anchoring:** The final mission leg ends at exact radar X/Y, not the R1C1 centre; Task 2 tests both a non-default radar coordinate and a moved grid.
4. **Protected-cell stale state:** Existing manual-mask bits and enough scan points to exceed threshold must not block or color points 2–5, but counts must remain available; Task 2 and Task 3 test both inputs.
5. **Serial boundary:** Repeated routes over 25 entries must round-trip, 409 entries must fit, and 410 entries must fail before framing; Task 4 locks all three boundaries.

---

## Test Setup

Run from the repository root in PowerShell:

```powershell
$env:DOTNET_CLI_HOME = (Resolve-Path '.\.dotnet-cli').Path
$env:DOTNET_SKIP_FIRST_TIME_EXPERIENCE = '1'
$env:DOTNET_NOLOGO = '1'
```

Focused custom-runner commands accept a class or method substring after `--`:

```powershell
.\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MissionRoute
.\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ApplicationStateTests
node --test .\tests\fieldMapViewport.test.mjs
```

### Task 1: Define mission configuration, text parsing, and live point coordinates

**Files:**

- Create: `src/Lds50cHost.Core/Configuration/PlanningMode.cs`
- Create: `src/Lds50cHost.Core/Configuration/MissionRouteSettings.cs`
- Create: `src/Lds50cHost.Core/Planning/MissionPointCatalog.cs`
- Create: `src/Lds50cHost.Core/Planning/MissionRouteTextCodec.cs`
- Modify: `src/Lds50cHost.Core/Configuration/AppConfiguration.cs`
- Modify: `tests/Lds50cHost.Core.Tests/Configuration/DefaultConfigurationTests.cs`
- Create: `tests/Lds50cHost.Core.Tests/Planning/MissionRouteSettingsTests.cs`

**Interfaces:**

- Produces `PlanningMode { MissionRoute, PointToPoint }` with `MissionRoute` as numeric/default value zero.
- Produces `MissionRouteSettings.CreateDefault()`, `IReadOnlyList<int> Sequence`, and `IReadOnlyList<string> Validate()`.
- Produces `MissionRouteTextCodec.TryParse(string?, out MissionRouteSettings, out string)` and `Format(MissionRouteSettings)`.
- Produces `MissionPointCatalog.Resolve(int, GridDefinition, double radarXmm, double radarYmm)`, `ResolveAll(...)`, and `ProtectedCellMask`.
- `AppConfiguration` gains init properties `PlanningMode PlanningMode` and non-null `MissionRouteSettings MissionRoute` while retaining its existing four constructor arguments for wire/config compatibility.

- [ ] **Step 1: Write failing configuration and route-codec tests**

  Add tests with these assertions:

  - `CreateDefault_UsesTheApprovedMissionRoute` expects mode `MissionRoute` and exact sequence `[1,5,4,2,3,4,2,3,1]`.
  - `TryParse_AcceptsSupportedSeparatorsAndNormalizes` parses `1 - 5，4, 2 3 - 1` and formats `1-5-4-2-3-1`.
  - `TryParse_RejectsEveryInvalidSequenceClass` separately checks letters, `0`, `6`, consecutive duplicate IDs, start not 1, end not 1, one value, and 17 values; each returns `false` and a non-empty Chinese error.
  - `ResolveAll_UsesApprovedDefaultCoordinates` expects points 1–5 at `(230,230)`, `(350,1200)`, `(1200,2050)`, `(2050,1200)`, `(1200,350)`.
  - `ResolveAll_FollowsRadarAndMovedGrid` changes radar X/Y and X/Y internal lines, then asserts point 1 follows radar while points 2–5 equal the new cell centres.
  - `ProtectedCellMask_ContainsExactlyPointsTwoThroughFive` checks R3C1, R5C3, R3C5, R1C3 and no other bit.

- [ ] **Step 2: Run focused tests and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MissionRoute
  ```

  Expected: compile failures for the new configuration, catalog, and codec types.

- [ ] **Step 3: Implement immutable mission settings and validation**

  `MissionRouteSettings` stores a private copied `int[]` behind a read-only view. `Validate()` returns exact errors for count, range, first/last 1, and consecutive duplicates. `CreateDefault()` creates a fresh sequence so callers cannot mutate a global array.

- [ ] **Step 4: Implement route text parsing and formatting**

  Split short hyphens, English/Chinese commas, and whitespace; parse using invariant integer rules; delegate semantic rules to `MissionRouteSettings.Validate()`. Never return a partially parsed valid object on failure. `Format` joins the validated values with `-`.

- [ ] **Step 5: Implement mission-point resolution and configuration defaults**

  Add:

  ```csharp
  public sealed record MissionPoint(int Number, MapEndpoint Endpoint, double Xmm, double Ymm);
  ```

  Point 1 resolves to `MapEndpoint.StartZone2` with exact radar X/Y. Points 2–5 resolve to `MapEndpoint.Cell(...)` and current grid-cell centres. Reject unknown identifiers with `ArgumentOutOfRangeException`.

- [ ] **Step 6: Run focused and full core tests; confirm GREEN**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MissionRoute
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release
  ```

- [ ] **Step 7: Commit**

  ```powershell
  git add src/Lds50cHost.Core/Configuration/PlanningMode.cs src/Lds50cHost.Core/Configuration/MissionRouteSettings.cs src/Lds50cHost.Core/Planning/MissionPointCatalog.cs src/Lds50cHost.Core/Planning/MissionRouteTextCodec.cs src/Lds50cHost.Core/Configuration/AppConfiguration.cs tests/Lds50cHost.Core.Tests/Configuration/DefaultConfigurationTests.cs tests/Lds50cHost.Core.Tests/Planning/MissionRouteSettingsTests.cs
  git commit -m "feat: define editable mission route settings"
  ```

### Task 2: Protect mission cells and compose multi-leg A* routes

**Files:**

- Modify: `src/Lds50cHost.Core/Mapping/ObstacleMap.cs`
- Modify: `src/Lds50cHost.Core/Mapping/ObstacleClassifier.cs`
- Create: `src/Lds50cHost.Core/Planning/PathPlanBuilder.cs`
- Modify: `src/Lds50cHost.Core/Planning/PathPlanner.cs`
- Create: `src/Lds50cHost.Core/Planning/MissionPathPlanner.cs`
- Modify: `tests/Lds50cHost.Core.Tests/Mapping/ObstacleClassifierTests.cs`
- Modify: `tests/Lds50cHost.Core.Tests/Planning/PathPlannerTests.cs`
- Create: `tests/Lds50cHost.Core.Tests/Planning/MissionPathPlannerTests.cs`

**Interfaces:**

- `ObstacleClassifier.Classify(...)` gains optional `uint protectedMask = 0` without breaking existing callers.
- `CellObstacle` gains `bool IsProtected`; `ObstacleMap.WithProtectedCells(uint)` returns a normalized copy and `TryToggleManual` refuses fixed or protected cells.
- `PathPlanBuilder.Build(IReadOnlyList<GridCell>, IReadOnlyList<PathWaypoint>)` is the single distance/direction/turn builder used by both planners.
- `MissionPathPlanner.Plan(GridDefinition, ObstacleMap, MissionRouteSettings, double radarXmm, double radarYmm)` returns one `PathPlan`.

- [ ] **Step 1: Write failing protected-obstacle tests**

  Prove that a task cell with a stale manual bit and at least threshold scan points:

  - keeps its `ScannedPointCount`;
  - has neither `Manual` nor `Scanned` blocking flags after protection;
  - reports `IsProtected == true` and `IsBlocked == false`;
  - cannot be toggled manually and leaves the normalized manual mask unchanged.

  Also verify a non-task cell with identical inputs remains blocked and the four fixed cells remain blocked.

- [ ] **Step 2: Write failing endpoint and mission-planner tests**

  Add:

  - `Plan_RadarOriginAsGoalUsesExactRadarCoordinates` for a cell-centre start returning to point 1 at non-default radar X/Y.
  - `Plan_DefaultMissionVisitsCheckpointsInOrderAndKeepsRepeatedCells` asserting success, more than 25 expanded cells, first/last `(230,230)`, ordered occurrence of all nine task coordinates, and at least one non-consecutive repeated cell.
  - `Plan_OutAndBackMissionProducesReverseTurn` using `1-5-1` and asserting a `Reverse` instruction at the turn-back point.
  - `Plan_RemovesOnlyAdjacentLegJoints` asserting no duplicated joint at a leg boundary while later visits to the same task cells remain.
  - `Plan_RecomputesAfterRadarAndGridMove` asserting the first/final radar coordinates and all four derived task centres change correctly.
  - `Plan_IdentifiesTheUnreachableMissionLeg` using a complete barrier and asserting the Chinese failure contains the exact `from→to` identifiers.
  - `Plan_DefensivelyProtectsTaskCells` passing an obstacle map constructed without protection and proving task endpoints still remain usable.

- [ ] **Step 3: Run focused tests and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- ObstacleClassifierTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MissionPathPlannerTests
  ```

- [ ] **Step 4: Implement protected obstacle normalization**

  Preserve fixed flags. For protected non-fixed cells, remove manual/scanned flags, retain counts, mark `IsProtected`, and clear their bits from `ManualMask`. `WithProtectedCells` must be idempotent so both state classification and mission planning can call it safely.

- [ ] **Step 5: Extract route-metric construction and fix symmetric endpoint anchoring**

  Move distance, cardinal direction, and turn calculation from `PathPlanner.BuildSuccess` to `PathPlanBuilder.Build`. Resolve `RadarOrigin`, `GoalZoneOne`, and cell-centre anchors for either the first or final endpoint, so a mission can return to radar coordinates.

- [ ] **Step 6: Implement multi-leg mission planning**

  Validate settings, resolve points through `MissionPointCatalog`, protect task cells defensively, call `PathPlanner.Plan` for every adjacent identifier pair, append the first leg in full and each later leg with `Skip(1)`, then rebuild metrics once across the merged route. On failure return `PathPlan.Failed($"任务段 {from}→{to} 无可用路径。")`.

- [ ] **Step 7: Run planning, mapping, and full core tests; confirm GREEN**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- ObstacleClassifierTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- PathPlannerTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MissionPathPlannerTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release
  ```

- [ ] **Step 8: Commit**

  ```powershell
  git add src/Lds50cHost.Core/Mapping/ObstacleMap.cs src/Lds50cHost.Core/Mapping/ObstacleClassifier.cs src/Lds50cHost.Core/Planning/PathPlanBuilder.cs src/Lds50cHost.Core/Planning/PathPlanner.cs src/Lds50cHost.Core/Planning/MissionPathPlanner.cs tests/Lds50cHost.Core.Tests/Mapping/ObstacleClassifierTests.cs tests/Lds50cHost.Core.Tests/Planning/PathPlannerTests.cs tests/Lds50cHost.Core.Tests/Planning/MissionPathPlannerTests.cs
  git commit -m "feat: plan protected multi-stop mission routes"
  ```

### Task 3: Integrate mission planning with application state and persistence

**Files:**

- Modify: `src/Lds50cHost/Services/ApplicationState.cs`
- Modify: `src/Lds50cHost/Services/ConfigurationStore.cs`
- Modify: `tests/Lds50cHost.App.Tests/Services/ApplicationStateTests.cs`
- Modify: `tests/Lds50cHost.App.Tests/Services/ConfigurationStoreTests.cs`

**Interfaces:**

- `ApplicationState` gains `UpdatePlanningMode(PlanningMode)` and `UpdateMissionRoute(MissionRouteSettings)`.
- `ApplicationState.ReprocessUnsafe()` always builds the displayed obstacle map with `MissionPointCatalog.ProtectedCellMask`, then chooses `PathPlanner` or `MissionPathPlanner` from `Configuration.PlanningMode`.
- `ConfigurationStore` validates local mission settings in addition to the unchanged STM32-serializable subset.

- [ ] **Step 1: Write failing application-state tests**

  Cover:

  - constructor defaults to mission mode and produces a successful path with more than 25 cells;
  - `UpdatePlanningMode` preserves `Start`/`Goal` and switches between ordinary and mission paths;
  - `UpdateMissionRoute` replaces the valid route and replans immediately;
  - moving grid boundaries changes 2–5 waypoint coordinates; changing radar X/Y changes both first and final route coordinates;
  - raw scan points exceeding threshold in points 2–5 keep counts but do not create red/blocked cells or break the route;
  - `TryToggleCell` rejects points 2–5 with a task-point-specific error and accepts an ordinary cell;
  - an unreachable intermediate leg makes `State.Path.Success == false` with the exact failed identifier pair.

- [ ] **Step 2: Write failing persistence tests**

  Add:

  - `SaveAndLoad_RoundTripsPlanningModeAndMissionSequence` with a non-default valid sequence.
  - `Load_LegacyJsonKeepsExistingFieldsAndAddsMissionDefaults` using JSON that contains only the old Filter, Map, Start, and Goal properties; assert those values survive and new values default correctly.
  - `Load_InvalidMissionSettingsFallsBackWithWarning` for a saved sequence that violates start/end or count rules.

- [ ] **Step 3: Run focused app tests and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ApplicationStateTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ConfigurationStoreTests
  ```

- [ ] **Step 4: Route all state reprocessing through the selected planner**

  Add local-configuration validation that combines filter, map, endpoint, enum, and mission validation before mutation. Apply protected classification on every reprocess. Keep `Changed` emission and locking consistent with existing update methods.

- [ ] **Step 5: Normalize legacy JSON and validate saves**

  Preserve old map/filter/start/goal values when new JSON properties are absent. Ensure `MissionRoute` is never null after load, use defaults only for the missing new fields, validate before atomic save, and keep corrupt/invalid configuration fallback warnings.

- [ ] **Step 6: Run focused and full app tests; confirm GREEN**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ApplicationStateTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ConfigurationStoreTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release
  ```

- [ ] **Step 7: Commit**

  ```powershell
  git add src/Lds50cHost/Services/ApplicationState.cs src/Lds50cHost/Services/ConfigurationStore.cs tests/Lds50cHost.App.Tests/Services/ApplicationStateTests.cs tests/Lds50cHost.App.Tests/Services/ConfigurationStoreTests.cs
  git commit -m "feat: persist and apply mission planning mode"
  ```

### Task 4: Allow expanded repeated routes in STM32 path payload v1

**Files:**

- Modify: `src/Lds50cHost.Core/Stm32/PathPayloadCodec.cs`
- Modify: `tests/Lds50cHost.Core.Tests/Stm32/Stm32ProtocolTests.cs`
- Modify: `tests/Lds50cHost.App.Tests/Services/Stm32CoordinatorTests.cs`
- Modify: `docs/stm32-protocol.md`

**Interfaces:**

- `PathPayloadCodec.MaximumEntryCount` is `409`.
- Wire fields and payload version stay unchanged; repeated row/column entries are explicitly valid.

- [ ] **Step 1: Write failing payload boundary tests**

  Construct `PathPayload` values directly and assert:

  - a 26-entry repeated route encodes and round-trips in order;
  - 409 entries with 408 segments encode to exactly 4091 bytes and round-trip;
  - 410 entries fail in both encode validation and an equivalent malformed decode count before buffer access;
  - a real default `MissionPathPlanner` result exceeds 25 entries and round-trips every entry and segment.

- [ ] **Step 2: Update the STM32 coordinator integration test**

  Make its plan helper select `MissionPathPlanner` for the default configuration. Decode the `SetPath` request and assert more than 25 entries and exact first/final radar coordinates, while the command/ACK/readback sequence remains unchanged.

- [ ] **Step 3: Run focused tests and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- Stm32ProtocolTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- Stm32CoordinatorTests
  ```

- [ ] **Step 4: Replace the unique-cell assumption with the frame-size limit**

  Keep the `u16` counts and v1 layout. Validate `Entries.Count <= 409`, segment count equals `max(0, entryCount - 1)`, row/column remain 1–5, and the encoded payload stays within `Stm32FrameCodec.MaximumPayloadLength`. Do not reject repeated cells.

- [ ] **Step 5: Update protocol documentation**

  Replace “最多25个格” with the 409-entry/4091-byte rule, state that repeated cells and backtracking are valid, and retain four-neighbour movement plus existing direction/turn values.

- [ ] **Step 6: Run core/app protocol tests and confirm GREEN**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- Stm32ProtocolTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- Stm32CoordinatorTests
  ```

- [ ] **Step 7: Commit**

  ```powershell
  git add src/Lds50cHost.Core/Stm32/PathPayloadCodec.cs tests/Lds50cHost.Core.Tests/Stm32/Stm32ProtocolTests.cs tests/Lds50cHost.App.Tests/Services/Stm32CoordinatorTests.cs docs/stm32-protocol.md
  git commit -m "feat: encode repeated mission routes for stm32"
  ```

### Task 5: Add the route editor, task markers, and mission diagnostics

**Files:**

- Create: `src/Lds50cHost/Components/MissionRouteEditor.razor`
- Modify: `src/Lds50cHost/Components/MapSettingsPanel.razor`
- Modify: `src/Lds50cHost/Components/FieldMap.razor`
- Modify: `src/Lds50cHost/Components/DiagnosticsPanel.razor`
- Modify: `src/Lds50cHost/Components/Pages/Home.razor`
- Modify: `src/Lds50cHost/wwwroot/js/fieldMap.js`
- Modify: `src/Lds50cHost/wwwroot/app.css`
- Modify: `tests/fieldMapViewport.test.mjs`

**Interfaces:**

- `MissionRouteEditor` consumes `ApplicationState` and applies only fully parsed `MissionRouteSettings`.
- `FieldMap.BuildModel()` supplies `missionPoints` as `{ number, row, column, x, y }[]` and supplies ordinary `start`/`goal` only in point-to-point mode.
- `fieldMap.js` exports `createMissionMarkers(points)` and uses the same normalized result for drawing numbered badges.

- [ ] **Step 1: Write failing JavaScript marker tests**

  Add tests that `createMissionMarkers`:

  - preserves all five approved identifiers and coordinates in order;
  - converts identifier labels consistently to display strings;
  - returns a new array and does not mutate the Blazor-supplied model;
  - lets existing viewport visibility logic exclude a marker only when it is outside the current view.

- [ ] **Step 2: Run the JavaScript test and confirm RED**

  ```powershell
  node --test .\tests\fieldMapViewport.test.mjs
  ```

  Expected: import/assertion failure because `createMissionMarkers` does not exist.

- [ ] **Step 3: Implement the mission route editor**

  Add a planning-mode selector. In mission mode show the editable text, “应用路线”, “恢复默认”, and five live point rows. Use `MissionRouteTextCodec`; on failure show its Chinese error and do not mutate state. On success call `State.UpdateMissionRoute`, clear the error, and replace the text with normalized `Format(...)`. In point-to-point mode retain the existing start/goal selectors.

- [ ] **Step 4: Add mission markers and mode-aware canvas endpoints**

  In `FieldMap.razor`, resolve all five live points through `MissionPointCatalog`. In JavaScript, draw compact high-contrast numbered badges after the route and before cursor overlay; offset point 1 so it remains legible beside the radar icon. Do not draw the old goal marker over point 1 in mission mode. Preserve zoom, pan, drag-boundary, and cell-click behavior.

- [ ] **Step 5: Add mission diagnostics and protected-cell wording**

  Show normalized task order in mission mode, complete expanded cell/coordinate counts, total distance, all turn chips including `掉头`, and the exact failed-leg message. Update map legend/help to explain numbered task points and that points 2–5 cannot be set as obstacles.

- [ ] **Step 6: Add responsive styles**

  Style route input/actions, coordinate rows, mode selector, mission-order diagnostics, and canvas badges for dark/light themes. Preserve the current map minimum sizes and existing responsive breakpoints.

- [ ] **Step 7: Run JavaScript tests and build the application; confirm GREEN**

  ```powershell
  node --test .\tests\fieldMapViewport.test.mjs
  .\.tools\dotnet\dotnet.exe build .\src\Lds50cHost\Lds50cHost.csproj --configuration Release
  ```

- [ ] **Step 8: Manually inspect both modes and themes**

  Verify:

  1. default sequence and all five coordinates match the approved values;
  2. mixed separators normalize after applying;
  3. invalid text leaves the plotted path unchanged;
  4. dragging a grid boundary updates labels and route;
  5. repeated sections of the route remain plotted and diagnostics retain the full order;
  6. ordinary point-to-point mode still works;
  7. markers remain legible in dark/light themes and through zoom/pan;
  8. points 2–5 cannot be toggled as manual obstacles.

- [ ] **Step 9: Commit**

  ```powershell
  git add src/Lds50cHost/Components/MissionRouteEditor.razor src/Lds50cHost/Components/MapSettingsPanel.razor src/Lds50cHost/Components/FieldMap.razor src/Lds50cHost/Components/DiagnosticsPanel.razor src/Lds50cHost/Components/Pages/Home.razor src/Lds50cHost/wwwroot/js/fieldMap.js src/Lds50cHost/wwwroot/app.css tests/fieldMapViewport.test.mjs
  git commit -m "feat: edit and display numbered mission routes"
  ```

### Task 6: Document, verify, and publish the completed upper-computer application

**Files:**

- Modify: `README.md`
- Generated, not committed: `artifacts/publish/win-x64-mission-route/`

- [ ] **Step 1: Update operator documentation**

  Document default identifiers/coordinates, route-editor syntax and limits, protected points 2–5, automatic coordinate updates after grid/radar changes, mission versus ordinary planning mode, repeated/backtracking behavior, failed-leg messages, and expanded STM32 path limits.

- [ ] **Step 2: Run all automated verification**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release
  node --test .\tests\fieldMapViewport.test.mjs
  .\.tools\dotnet\dotnet.exe build .\Lds50cHost.slnx --configuration Release
  git diff --check
  ```

  Expected: every test reports pass, the solution builds with zero errors, and whitespace validation is clean.

- [ ] **Step 3: Review the implementation against every spec acceptance scenario**

  Check each of the nine scenarios in `docs/superpowers/specs/2026-09-28-editable-mission-route-design.md`. Record any scenario not covered by automated evidence and run it manually before claiming completion.

- [ ] **Step 4: Publish to an unlocked output directory**

  Use a new directory so an older running executable cannot lock replacement files:

  ```powershell
  .\.tools\dotnet\dotnet.exe publish .\src\Lds50cHost\Lds50cHost.csproj --configuration Release --runtime win-x64 --self-contained true --output .\artifacts\publish\win-x64-mission-route --packages .\.tools\nuget --force -p:NuGetAudit=false -p:PublishSingleFile=false -p:DebugType=None -p:DebugSymbols=false
  Test-Path .\artifacts\publish\win-x64-mission-route\Lds50cHost.exe
  Get-FileHash .\artifacts\publish\win-x64-mission-route\Lds50cHost.exe -Algorithm SHA256
  ```

- [ ] **Step 5: Commit documentation**

  ```powershell
  git add README.md
  git commit -m "docs: explain editable mission route workflow"
  ```

- [ ] **Step 6: Confirm handoff state**

  ```powershell
  git status --short
  git log --oneline -10
  ```

  Expected: only the pre-existing untracked `.dotnet-cli/` may remain; generated publish artifacts are not committed; all feature, test, protocol, and README changes are committed.
