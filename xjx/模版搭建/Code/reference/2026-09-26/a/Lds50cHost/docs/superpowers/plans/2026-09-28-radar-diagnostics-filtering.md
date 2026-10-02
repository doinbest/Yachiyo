# Radar Diagnostics and Filtering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make every scan configure the LDS-50C consistently, report the device's actual filter/unit state, let the user compare received and host-filtered point clouds without changing planning, and export the latest complete received revolution as CSV.

**Architecture:** Keep protocol normalization and CSV serialization in `Lds50cHost.Core`; keep serial-session policy, application snapshots, HTTP download behavior, and Razor presentation in `Lds50cHost`. Both scan modes share one preflight/receive pipeline. `ApplicationState` owns separate received and filtered projections, while obstacle classification and A* planning remain wired only to filtered points.

**Tech Stack:** .NET 10 (`net10.0-windows`), C#, ASP.NET Core minimal endpoints, Blazor Interactive Server, the repository's reflection-based xUnit-compatible test runner, PowerShell publish script.

**Spec:** [`docs/superpowers/specs/2026-09-28-radar-diagnostics-filtering-design.md`](../specs/2026-09-28-radar-diagnostics-filtering-design.md)

## Global Constraints

- Preserve the existing exclusive serial-session, continuous-scan lease, disconnect, cancellation, and `LSTOPH` cleanup behavior.
- Send exact ASCII commands in this order for both scan modes: `LMDMMH`, `LOCONH`, `LFFF1H`, `LSSS1H`, `LSTARH`.
- Treat the received-raw view as device-returned data after the device's own filtering; do not describe it as optical samples before firmware processing.
- Normalize every completed `RadarScan` to millimetres before storing it. Never allow `ushort` overflow to wrap into a false near point.
- If a known unit mode changes while a revolution is being assembled, reset the assembler and wait for a fresh complete revolution so one `RadarScan` cannot mix interpretations.
- Obstacle classification and path planning must always use host-filtered field points, regardless of display mode.
- CSV export reads an immutable local reference to the latest complete scan, never an in-progress revolution, never the serial port, and never writes a server-side file.
- Do not add the vendor C++ SDK, DLLs, third-party packages, firmware filter-off controls, or changes to the STM32 protocol/path planner.
- Do not stage or commit the existing untracked `.dotnet-cli/` directory.
- During implementation, make product-file edits with `apply_patch` and commit after each passing task.

## Test Setup

Run once per PowerShell session from the repository root:

```powershell
$env:DOTNET_CLI_HOME = (Resolve-Path '.\.dotnet-cli').Path
$env:DOTNET_SKIP_FIRST_TIME_EXPERIENCE = '1'
$env:DOTNET_NOLOGO = '1'
```

The repository test runner accepts a class or method-name substring after `--`:

```powershell
.\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MeasurementUnitNormalizerTests
.\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- RadarCoordinatorTests
```

## Task 1: Normalize measurement frames to millimetres

**Files:**

- Create: `src/Lds50cHost.Core/RadarProtocol/MeasurementUnitNormalizer.cs`
- Create: `tests/Lds50cHost.Core.Tests/RadarProtocol/MeasurementUnitNormalizerTests.cs`

- [ ] **Step 1: Write failing unit tests**

  Add these tests:

  - `ToMillimetres_MillimetreFramePreservesDistancesAndMetadata`
  - `ToMillimetres_CentimetreFrameMultipliesDistancesByTen`
  - `ToMillimetres_CentimetreOverflowBecomesInvalidZero`

  Use frames containing multiple energy values and assert preservation of `IsFixedResolution`, `PointCount`, `StartAngleTenths`, `SectorAngleTenths`, energy, and point order. Include `6554 cm` as the overflow case and verify it becomes `0 mm`, not a wrapped value.

- [ ] **Step 2: Run the focused test and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MeasurementUnitNormalizerTests
  ```

  Expected: compile failure because `MeasurementUnitNormalizer` does not exist.

- [ ] **Step 3: Implement the smallest core API**

  Add:

  ```csharp
  public static class MeasurementUnitNormalizer
  {
      public static MeasurementFrame ToMillimetres(MeasurementFrame frame, bool usesMillimetres);
  }
  ```

  Return the original frame for millimetre mode. For centimetre mode, create a frame with cloned measurements; multiply each distance by 10 using a wider integer and map values above `ushort.MaxValue` to zero. A source distance of zero stays zero.

- [ ] **Step 4: Run the focused test and confirm GREEN**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- MeasurementUnitNormalizerTests
  ```

- [ ] **Step 5: Commit**

  ```powershell
  git add src/Lds50cHost.Core/RadarProtocol/MeasurementUnitNormalizer.cs tests/Lds50cHost.Core.Tests/RadarProtocol/MeasurementUnitNormalizerTests.cs
  git commit -m "feat: normalize radar distances to millimetres"
  ```

## Task 2: Store device status and separate received/filtered projections

**Files:**

- Create: `src/Lds50cHost/Services/RadarStatusSnapshot.cs`
- Modify: `src/Lds50cHost/Services/ApplicationState.cs`
- Modify: `tests/Lds50cHost.App.Tests/Services/ApplicationStateTests.cs`

- [ ] **Step 1: Write failing application-state tests**

  Add tests that prove:

  - `SetRadarStatus_StoresAllFlagsAndTimestamp` creates a snapshot from a `StatusFrame`.
  - `ClearRadarStatus_RemovesAStaleReading` sets the nullable status back to `null`.
  - `SetPointCloudViewMode_ChangesDisplayedPointsButNotPlanningInputs` uses a scan containing one point accepted by the current energy filter and one rejected point; `DisplayedPoints` and `DisplayedFieldPoints` change between modes, but every obstacle cell flag/count and every path cell/waypoint remains identical.
  - `UpdateMapSettings_ReprojectsReceivedAndFilteredPoints` changes radar X/Y and verifies both field-point collections move by the same offset.
  - default view mode is `HostFiltered`.

- [ ] **Step 2: Run the focused test and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ApplicationStateTests
  ```

  Expected: compile failures for the new status/view APIs.

- [ ] **Step 3: Add the immutable status snapshot**

  Define `RadarStatusSnapshot` with:

  ```csharp
  bool UsesMillimetres
  bool EnergyEnabled
  bool TrailingPointRemovalEnabled
  bool FilterEnabled
  DateTimeOffset ReceivedAt
  ```

  Provide `From(StatusFrame frame, DateTimeOffset receivedAt)` so bit interpretation remains in the protocol type and UI/state never reads masks directly.

- [ ] **Step 4: Extend `ApplicationState` data flow**

  Add:

  ```csharp
  public enum PointCloudViewMode { ReceivedRaw, HostFiltered }
  public IReadOnlyList<FieldPoint> RawFieldPoints { get; }
  public IReadOnlyList<RadarPoint> DisplayedPoints { get; }
  public IReadOnlyList<FieldPoint> DisplayedFieldPoints { get; }
  public PointCloudViewMode ViewMode { get; private set; }
  public RadarStatusSnapshot? RadarStatus { get; private set; }
  public void SetPointCloudViewMode(PointCloudViewMode mode);
  public void SetRadarStatus(StatusFrame frame, DateTimeOffset? receivedAt = null);
  public void ClearRadarStatus();
  ```

  In `ReprocessUnsafe`, transform `RawScan.Points` into a private received-field array and transform filtered points independently. Continue passing only the filtered field array to `ObstacleClassifier.Classify` and `PathPlanner.Plan`. Raise `Changed` after status/mode mutations.

- [ ] **Step 5: Run focused app tests and confirm GREEN**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- ApplicationStateTests
  ```

- [ ] **Step 6: Commit**

  ```powershell
  git add src/Lds50cHost/Services/RadarStatusSnapshot.cs src/Lds50cHost/Services/ApplicationState.cs tests/Lds50cHost.App.Tests/Services/ApplicationStateTests.cs
  git commit -m "feat: track radar status and point cloud view mode"
  ```

## Task 3: Share preflight and status-aware acquisition across scan modes

**Files:**

- Modify: `src/Lds50cHost/Services/RadarCoordinator.cs`
- Modify: `tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs`

- [ ] **Step 1: Update and add failing coordinator tests**

  Cover all of the following:

  - single scan writes exactly `LMDMMH`, `LOCONH`, `LFFF1H`, `LSSS1H`, `LSTARH`, then eventually `LSTOPH`;
  - continuous scan has the same five-command prefix and final stop;
  - failure while writing `LFFF1H` attempts only `LMDMMH`, `LOCONH`, `LFFF1H`, then cleanup `LSTOPH`, never `LSSS1H` or `LSTARH`, and retains the previous map;
  - a new session clears a stale `RadarStatus` before preflight;
  - status bytes `[0x53, 0x54, flags, r0, r1, r2, 0x45, 0x44]` update all four state fields;
  - a centimetre status before measurement frames causes completed scan distances to be ten times the wire values;
  - no status frame preserves the millimetre assumption established by `LMDMMH`;
  - changing the reported unit mid-revolution resets assembly, so the emitted scan contains only points from a later fully consistent revolution;
  - ASCII command replies/noise before binary frames are discarded and do not prevent a complete scan.

  Update existing tests that assume `transport.Writes[0] == "LSTARH"` to assert the complete prefix instead.

- [ ] **Step 2: Run the focused test and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- RadarCoordinatorTests
  ```

  Expected: order/status/unit assertions fail against the current coordinator.

- [ ] **Step 3: Extract one preflight routine**

  Store command bytes once and add a helper used by both session methods. It must:

  1. call `state.ClearRadarStatus()`;
  2. write the four configuration commands in order;
  3. write `LSTARH` only after all four succeed;
  4. wrap a write failure with the failing command name while preserving the original exception/cancellation semantics.

  Keep `LSTOPH` in the existing `finally` paths so cleanup is attempted even when preflight fails.

- [ ] **Step 4: Extract one frame-processing path**

  Keep a session-local `usesMillimetres = true`. For each parsed frame:

  - log `AlarmFrame` exactly as before;
  - on `StatusFrame`, store it with the receipt time; whenever its unit differs from the session's current interpretation, call `assembler.Reset()` before updating the interpretation and accepting further measurements;
  - on `MeasurementFrame`, call `MeasurementUnitNormalizer.ToMillimetres(frame, usesMillimetres)` before `assembler.Push`;
  - publish only completed scans and parser counters.

  Use shared helpers so single and continuous modes cannot drift in command order or unit handling.

- [ ] **Step 5: Run coordinator and regression tests**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- RadarCoordinatorTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release
  ```

- [ ] **Step 6: Commit**

  ```powershell
  git add src/Lds50cHost/Services/RadarCoordinator.cs tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs
  git commit -m "feat: configure and diagnose radar acquisition"
  ```

## Task 4: Serialize a complete scan as invariant CSV

**Files:**

- Create: `src/Lds50cHost.Core/Export/RadarScanCsvExporter.cs`
- Create: `tests/Lds50cHost.Core.Tests/Export/RadarScanCsvExporterTests.cs`

- [ ] **Step 1: Write failing exporter tests**

  Add tests for:

  - exact UTF-8 BOM bytes `EF BB BF`;
  - exact header `index,angle_deg,distance_mm,energy`;
  - zero-based index and source point order;
  - angle formatting with a decimal point under `zh-CN` and a comma-decimal culture such as `de-DE`;
  - millimetre distance and energy values copied exactly;
  - a terminating newline with no localized headings.

  Restore the original `CurrentCulture` and `CurrentUICulture` in `finally` so the custom sequential runner cannot leak culture into later tests.

- [ ] **Step 2: Run the focused test and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- RadarScanCsvExporterTests
  ```

- [ ] **Step 3: Implement the pure exporter**

  Add:

  ```csharp
  public static class RadarScanCsvExporter
  {
      public static byte[] Export(RadarScan scan);
  }
  ```

  Use angle format `0.######` with `CultureInfo.InvariantCulture`, `\r\n`, and `new UTF8Encoding(encoderShouldEmitUTF8Identifier: true)`. Do not read `ApplicationState`, the clock, filesystem, or serial port.

- [ ] **Step 4: Run the focused and full core suites**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release -- RadarScanCsvExporterTests
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release
  ```

- [ ] **Step 5: Commit**

  ```powershell
  git add src/Lds50cHost.Core/Export/RadarScanCsvExporter.cs tests/Lds50cHost.Core.Tests/Export/RadarScanCsvExporterTests.cs
  git commit -m "feat: serialize raw radar scans as csv"
  ```

## Task 5: Add the latest-scan download endpoint

**Files:**

- Create: `src/Lds50cHost/Services/RadarScanDownloadEndpoint.cs`
- Create: `src/Lds50cHost/Properties/AssemblyInfo.cs`
- Modify: `src/Lds50cHost/Program.cs`
- Create: `tests/Lds50cHost.App.Tests/Services/RadarScanDownloadEndpointTests.cs`

- [ ] **Step 1: Write failing endpoint tests**

  Execute the returned `IResult` against a `DefaultHttpContext` whose response body is a `MemoryStream` and whose services contain logging. Verify:

  - no `RawScan` returns HTTP 404 and an empty body;
  - a scan returns HTTP 200, `text/csv; charset=utf-8`, a safe name matching `lds50c-scan-yyyyMMdd-HHmmss.csv`, and exactly the exporter bytes;
  - replacing `ApplicationState.RawScan` after the handler captures its local reference cannot change the response body, demonstrating immutable snapshot export;
  - an injected exporter failure returns HTTP 500, adds a readable log entry, and does not clear the scan.

- [ ] **Step 2: Run the focused endpoint test and confirm RED**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- RadarScanDownloadEndpointTests
  ```

- [ ] **Step 3: Implement and map the endpoint**

  Provide:

  ```csharp
  public static IResult DownloadLatest(ApplicationState state);
  internal static IResult CreateResult(
      ApplicationState state,
      Func<RadarScan, byte[]> exporter);
  ```

  `DownloadLatest` delegates to `CreateResult` with `RadarScanCsvExporter.Export`. Capture `var scan = state.RawScan` once. Return `Results.NotFound()` for null; otherwise export and return `Results.File(bytes, "text/csv; charset=utf-8", fileName)`. Catch serialization exceptions at this boundary, call `state.AddLog(...)`, and return `Results.Problem(...)` without affecting acquisition. Add `[assembly: InternalsVisibleTo("Lds50cHost.App.Tests")]` so tests can drive only the failure seam without exposing it as application API.

  Map before the Razor fallback:

  ```csharp
  app.MapGet("/api/radar/latest-scan.csv", RadarScanDownloadEndpoint.DownloadLatest);
  ```

- [ ] **Step 4: Run focused tests and build the web project**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release -- RadarScanDownloadEndpointTests
  .\.tools\dotnet\dotnet.exe build .\src\Lds50cHost\Lds50cHost.csproj --configuration Release
  ```

- [ ] **Step 5: Commit**

  ```powershell
  git add src/Lds50cHost/Services/RadarScanDownloadEndpoint.cs src/Lds50cHost/Properties/AssemblyInfo.cs src/Lds50cHost/Program.cs tests/Lds50cHost.App.Tests/Services/RadarScanDownloadEndpointTests.cs
  git commit -m "feat: download the latest raw radar scan"
  ```

## Task 6: Expose point-cloud modes, status, and export in the UI

**Files:**

- Modify: `src/Lds50cHost/Components/FilterPanel.razor`
- Modify: `src/Lds50cHost/Components/DiagnosticsPanel.razor`
- Modify: `src/Lds50cHost/Components/FieldMap.razor`
- Modify: `src/Lds50cHost/Components/Pages/Home.razor`
- Modify: `src/Lds50cHost/wwwroot/app.css`

- [ ] **Step 1: Bind the canvas and preview to the explicit display contract**

  Change `FieldMap.BuildModel()` to use `State.DisplayedFieldPoints` and change the diagnostics preview to use `State.DisplayedPoints`. Build immediately; it must pass using Task 2's tested display contract, proving both components bind to one explicit state-selected source rather than selecting collections independently.

  ```powershell
  .\.tools\dotnet\dotnet.exe build .\src\Lds50cHost\Lds50cHost.csproj --configuration Release
  ```

- [ ] **Step 2: Add the display-mode control**

  In `FilterPanel.razor`, add a two-button segmented control labeled `接收原始` and `上位机筛选`. The active button reflects `State.ViewMode`; each click calls `SetPointCloudViewMode`. Add the fixed explanatory copy `只切换显示，不改变障碍格和路径`.

- [ ] **Step 3: Add status diagnostics**

  In `DiagnosticsPanel.razor`, render four rows for unit, energy, trailing-point removal, and device filtering:

  - when `State.RadarStatus` is null, show `等待雷达状态`;
  - expected values are millimetres/on/on/on;
  - known mismatches get a warning class and text that identifies the actual value;
  - show the status receipt time;
  - rename the preview heading according to the active display mode and keep the first-12 limit.

- [ ] **Step 4: Add the export action**

  Add an anchor styled as a button linking to `/api/radar/latest-scan.csv`. When no complete scan exists, render a disabled non-navigating button; when a scan exists, include the HTML `download` attribute. Do not call the endpoint through Blazor or touch the scan session.

- [ ] **Step 5: Update summary copy and styles**

  In `Home.razor`, ensure the top count wording distinguishes displayed points from received total. Add responsive styles for status warnings, status rows, mode help text, and export action without shrinking the map below its current minimum.

- [ ] **Step 6: Build and manually inspect both themes**

  ```powershell
  .\.tools\dotnet\dotnet.exe build .\Lds50cHost.slnx --configuration Release
  ```

  Manual acceptance:

  1. open the app with an unused port or stop any previous `Lds50cHost.exe` first;
  2. confirm the two mode buttons change only the plotted/previewed points;
  3. confirm grid colors, obstacle counts, and route do not change when toggling mode;
  4. verify null status, all-expected status, and at least one warning state remain readable in dark and light themes;
  5. confirm the export control is disabled before a scan and downloads after a scan.

- [ ] **Step 7: Commit**

  ```powershell
  git add src/Lds50cHost/Components/FilterPanel.razor src/Lds50cHost/Components/DiagnosticsPanel.razor src/Lds50cHost/Components/FieldMap.razor src/Lds50cHost/Components/Pages/Home.razor src/Lds50cHost/wwwroot/app.css
  git commit -m "feat: expose radar diagnostics and point cloud modes"
  ```

## Task 7: Document, verify, and publish the Windows build

**Files:**

- Modify: `README.md`
- Modify if needed: `docs/hardware-verification.md`
- Generated, not committed: `artifacts/publish/win-x64/`

- [ ] **Step 1: Update operator documentation**

  Document:

  - the five-command scan preflight;
  - the meaning of received raw versus host filtered;
  - the four status indicators and why a mismatch matters;
  - CSV columns and the fact that export is the latest complete revolution;
  - the unchanged rule that path planning always uses host-filtered points;
  - a hardware check comparing custom-app CSV, custom plotted wall distance, and factory-app wall distance at the same stationary setup.

- [ ] **Step 2: Run all automated verification**

  ```powershell
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --configuration Release
  .\.tools\dotnet\dotnet.exe run --project .\tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --configuration Release
  node --test .\tests\js\*.test.js
  .\.tools\dotnet\dotnet.exe build .\Lds50cHost.slnx --configuration Release
  git diff --check
  ```

  Expected baseline plus new tests: all core, app, and eight existing JavaScript tests pass; build and whitespace check succeed.

- [ ] **Step 3: Publish a fresh self-contained x64 build**

  Ensure no old process is holding output files, then run:

  ```powershell
  powershell -ExecutionPolicy Bypass -File .\scripts\publish-windows.ps1
  Test-Path .\artifacts\publish\win-x64\Lds50cHost.exe
  ```

  Do not commit `artifacts/`.

- [ ] **Step 4: Perform hardware acceptance at 921600 baud**

  With the radar and field stationary:

  1. connect at `921600` and start continuous scan;
  2. confirm status reads millimetres, energy on, trailing-point removal on, device filter on;
  3. compare received/raw and host-filtered displays without moving anything;
  4. export CSV and verify a suspicious vertical return's angle/distance exists in the received revolution;
  5. compare the same physical wall's distance with the factory app;
  6. stop continuous scan and confirm `LSTOPH` behavior remains normal.

  Hardware comparison is the final evidence for whether the earlier mid-field line came from device returns, host filtering, or unit/configuration mismatch.

- [ ] **Step 5: Commit documentation**

  ```powershell
  git add README.md docs/hardware-verification.md
  git commit -m "docs: explain radar diagnostics and raw scan export"
  ```

- [ ] **Step 6: Confirm repository handoff state**

  ```powershell
  git status --short
  git log --oneline -8
  ```

  Expected: only the pre-existing untracked `.dotnet-cli/` may remain; every product/test/doc change is committed.

## Review Focus

Before execution, review these failure-sensitive decisions:

1. **Unit transition:** Status may appear after measurement bytes. The plan resets the assembler when the known unit interpretation changes so no published revolution mixes unscaled and scaled values.
2. **Text replies:** Preflight success does not depend on command-response text. Parser-noise coverage proves ASCII replies cannot block later binary frames.
3. **Cleanup after partial preflight:** A failed configuration command aborts later commands and start, yet still attempts `LSTOPH` and keeps the prior completed map.
4. **Snapshot export during continuous scan:** The endpoint captures one immutable `RadarScan` reference before serialization, so a later scan replacement cannot produce mixed CSV rows.
5. **Display/planning isolation:** Both map and preview honor display mode, while classifier/path assertions remain tied only to filtered field points.
6. **Culture and Excel compatibility:** Export tests lock down BOM, invariant decimal point, column order, timestamp-safe filename, and content type.

---

Plan complete and saved to `docs/superpowers/plans/2026-09-28-radar-diagnostics-filtering.md`.
