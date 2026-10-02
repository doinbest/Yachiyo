# LDS-50C Global Device Session Coordination Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make continuous scanning, single scanning, commands, connection, and disconnection safe across multiple Blazor browser circuits while preserving the confirmed UI behavior.

**Architecture:** `RadarCoordinator` becomes the sole owner of radar transport lifecycle and device-operation arbitration. A lifecycle semaphore prevents connection transitions from racing new operations, a device semaphore permits only one serial consumer, and opaque continuous-scan leases prevent stale components from stopping newer sessions.

**Tech Stack:** C# 14, .NET 10, Blazor Server interactive components, `System.IO.Ports`, custom executable test harness.

**Spec:** `docs/superpowers/specs/2026-09-28-device-session-coordination-design.md`

## Global Constraints

- Keep existing LDS-50C commands and binary frame parsing unchanged.
- A competing device operation must fail immediately instead of waiting silently.
- A normal continuous-scan cancellation is not an error.
- Preserve the last completed scan after stop, disconnect, or failure.
- If `LSTOPH` fails during disconnect, close the serial port and report the stop failure.
- Continue supporting the confirmed physical device on `COM8` at `921600` baud without removing other documented baud rates.
- Do not add external packages.

## Review Focus

- A stale page lease must not stop a newer continuous session; Task 1 adds `StaleLease_CannotStopANewerContinuousSession`.
- Disconnect during a single scan must be rejected without closing the port; Task 2 adds `Disconnect_DuringSingleScanIsRejectedAndLeavesPortOpen`.
- A continuous stop-write failure during disconnect must still close the port and remain visible; Task 2 adds `Disconnect_StopWriteFailureClosesPortAndReportsFailure`.
- A new scan must not enter between continuous-stop completion and port close; Task 2 adds `Disconnect_BlocksACompetingContinuousStartUntilPortIsClosed`.
- Component disposal after another page restarts scanning must be harmless; Task 3 relies on the stale-lease test and uses conditional lease stop only.

---

### Task 1: Continuous Session Lease and Device Arbitration

**Files:**
- Create: `src/Lds50cHost/Services/ContinuousScanLease.cs`
- Modify: `src/Lds50cHost/Services/RadarCoordinator.cs`
- Test: `tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs`

**Interfaces:**
- Produces: `public readonly record struct ContinuousScanLease(long Id)`.
- Produces: `Task<ContinuousScanLease> StartContinuousAsync(CancellationToken cancellationToken = default)`.
- Produces: `Task<bool> StopContinuousAsync(ContinuousScanLease? expectedLease = null)`; returns `false` only when a supplied lease is stale, and propagates the current session's stop failure.
- Produces: `bool IsContinuousScanning`, `string? ContinuousError`, and `event Action? ContinuousStateChanged`.
- Preserves: `Task<RadarScan> CaptureSingleScanAsync(...)` and `Task SendCommandAsync(...)`, both guarded by the shared device-operation gate.

- [ ] **Step 1: Write failing lease and exclusivity tests**

Add tests named `StaleLease_CannotStopANewerContinuousSession`, `StartContinuous_RejectsACompetingSingleScanUntilStopped`, and `StopContinuous_ReportsAStopCommandWriteFailure`. Assert that lease A cannot stop session B, the competing single scan throws `InvalidOperationException`, and stop-write `IOException` is returned while the last scan remains intact.

- [ ] **Step 2: Run the coordinator tests and verify RED**

Run: `.\.tools\dotnet\dotnet.exe run --project tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --no-restore -- RadarCoordinatorTests`

Expected: compile or assertion failure because lease-aware signatures and stale-lease behavior do not exist.

- [ ] **Step 3: Implement the lease-aware continuous session API**

Create `ContinuousScanLease` and update `RadarCoordinator` to allocate monotonically increasing IDs, store the active lease atomically with its cancellation source/task, and make conditional stop a no-op for a mismatched lease. Keep the device operation gate held for the full single or continuous scan and release it in `finally`.

- [ ] **Step 4: Run coordinator tests and verify GREEN**

Run the Task 1 command.

Expected: all `RadarCoordinatorTests` pass, including the three new lifecycle tests.

- [ ] **Step 5: Commit Task 1**

```powershell
git add src/Lds50cHost/Services/ContinuousScanLease.cs src/Lds50cHost/Services/RadarCoordinator.cs tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs
git commit -m "fix: add continuous scan session leases"
```

### Task 2: Serialize Connection Lifecycle with Radar Operations

**Files:**
- Modify: `src/Lds50cHost/Services/RadarCoordinator.cs`
- Modify: `tests/Lds50cHost.App.Tests/Services/FakeByteTransport.cs`
- Test: `tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs`

**Interfaces:**
- Consumes: lease-aware session API from Task 1.
- Produces: `bool IsOpen`.
- Produces: `Task ConnectAsync(string portName, int baudRate, CancellationToken cancellationToken = default)`.
- Produces: `Task DisconnectAsync(CancellationToken cancellationToken = default)`.
- Updates fake transport with deterministic open/close state and optional lifecycle hooks needed by concurrency tests.

- [ ] **Step 1: Write failing connection-race tests**

Add `Disconnect_DuringSingleScanIsRejectedAndLeavesPortOpen`, `Disconnect_StopWriteFailureClosesPortAndReportsFailure`, and `Disconnect_BlocksACompetingContinuousStartUntilPortIsClosed`. The tests must assert the exact final `IsOpen` and `IsContinuousScanning` states, not only exception types.

- [ ] **Step 2: Run coordinator tests and verify RED**

Run the Task 1 test command.

Expected: compile failure because `ConnectAsync`/`DisconnectAsync` do not exist, followed by assertion failures until lifecycle transitions are atomic.

- [ ] **Step 3: Implement lifecycle serialization**

Add a lifecycle `SemaphoreSlim` around operation admission and connection transitions. `DisconnectAsync` must hold this barrier while stopping continuous scanning and closing the port, reject an unrelated active single scan, close the port even after a stop-write failure, and then rethrow a message that includes both facts. `ConnectAsync` and every scan/command admission must use the same barrier.

- [ ] **Step 4: Run coordinator tests and verify GREEN**

Run the Task 1 test command.

Expected: all coordinator tests pass, including deterministic cross-circuit races.

- [ ] **Step 5: Commit Task 2**

```powershell
git add src/Lds50cHost/Services/RadarCoordinator.cs tests/Lds50cHost.App.Tests/Services/FakeByteTransport.cs tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs
git commit -m "fix: serialize radar connection lifecycle"
```

### Task 3: Bind the Blazor Control Panel to Global Sessions

**Files:**
- Modify: `src/Lds50cHost/Components/RadarControlPanel.razor`
- Modify: `README.md`
- Test: `tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs`

**Interfaces:**
- Consumes: `RadarCoordinator.ConnectAsync`, `DisconnectAsync`, `StartContinuousAsync`, both `StopContinuousAsync` forms, `SendCommandAsync`, `IsOpen`, `IsContinuousScanning`, `ContinuousError`, and `ContinuousStateChanged`.
- Produces: no new service API; the component stores `ContinuousScanLease? _ownedContinuousLease` only for conditional disposal.

- [ ] **Step 1: Add the component-disposal behavior to the stale-lease test**

Extend the service-level stale-lease scenario so a conditional stop with lease A occurs after session B starts and asserts session B remains active. This pins the exact behavior used by component disposal without introducing a UI testing dependency.

- [ ] **Step 2: Replace direct transport mutations in the control panel**

Remove direct `OpenAsync`, `CloseAsync`, and radar `WriteAsync` calls. Route them through `RadarCoordinator`; store the lease returned when this component starts continuous scanning; explicit stop targets the current global session; `DisposeAsync` conditionally stops only `_ownedContinuousLease`.

- [ ] **Step 3: Update shared UI state and documentation**

Render connection and continuous status from the coordinator so every browser circuit sees the same controls. Update `README.md` to distinguish frozen single-scan mode from per-revolution continuous mode and retain the limitation that this is not vehicle localization or closed-loop dynamic avoidance.

- [ ] **Step 4: Build the Blazor project**

Run: `.\.tools\dotnet\dotnet.exe build src\Lds50cHost\Lds50cHost.csproj -c Release --no-restore`

Expected: exit code 0 and 0 compiler errors.

- [ ] **Step 5: Commit Task 3**

```powershell
git add src/Lds50cHost/Components/RadarControlPanel.razor README.md tests/Lds50cHost.App.Tests/Services/RadarCoordinatorTests.cs
git commit -m "feat: bind continuous scan UI to global sessions"
```

### Task 4: Full Verification and Windows Publication

**Files:**
- Verify: `tests/Lds50cHost.Core.Tests/Lds50cHost.Core.Tests.csproj`
- Verify: `tests/Lds50cHost.App.Tests/Lds50cHost.App.Tests.csproj`
- Generate: `artifacts/publish/win-x64-continuous/`

**Interfaces:**
- Consumes: all completed tasks.
- Produces: self-contained Windows executable at `artifacts/publish/win-x64-continuous/Lds50cHost.exe`.

- [ ] **Step 1: Run all protocol/core tests**

Run: `.\.tools\dotnet\dotnet.exe run --project tests\Lds50cHost.Core.Tests\Lds50cHost.Core.Tests.csproj --no-restore`

Expected: `RESULT 48/48 passed`.

- [ ] **Step 2: Run all application tests**

Run: `.\.tools\dotnet\dotnet.exe run --project tests\Lds50cHost.App.Tests\Lds50cHost.App.Tests.csproj --no-restore`

Expected: every listed test passes with `0` failures.

- [ ] **Step 3: Run Release build and diff checks**

Run: `.\.tools\dotnet\dotnet.exe build src\Lds50cHost\Lds50cHost.csproj -c Release --no-restore` and `git diff --check`.

Expected: build exit code 0, 0 compiler errors, and no whitespace errors.

- [ ] **Step 4: Publish the self-contained Windows build**

Run: `.\.tools\dotnet\dotnet.exe publish src\Lds50cHost\Lds50cHost.csproj -c Release -r win-x64 --self-contained true --no-restore -o artifacts\publish\win-x64-continuous`

Expected: exit code 0 and a new `Lds50cHost.exe` in the target directory.

- [ ] **Step 5: Request final whole-range code review**

Review from the commit before Task 1 through Task 3's commit, with explicit focus on stale leases, disconnect serialization, stop failure reporting, and component disposal.

- [ ] **Step 6: Commit any review fixes and re-run Steps 1-4**

Expected: the reviewed final commit retains all passing tests and regenerates the published executable.
