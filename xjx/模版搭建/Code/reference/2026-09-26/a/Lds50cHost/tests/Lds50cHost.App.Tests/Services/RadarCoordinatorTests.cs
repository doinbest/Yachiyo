using System.Text;
using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.RadarProtocol;
using Lds50cHost.Core.Scanning;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class RadarCoordinatorTests
{
    [Fact]
    public async Task CaptureSingleScan_WritesStartReturnsACompleteRevolutionAndStops()
    {
        await using var transport = new FakeByteTransport();
        foreach (var frame in CompleteRevolutionFrames())
        {
            var split = frame.Length / 2;
            transport.Enqueue(frame[..split], frame[split..]);
        }
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var scan = await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None);

        Assert.Equal(40, scan.Points.Count);
        Assert.Equal(ExpectedSessionCommands, WrittenCommands(transport));
        Assert.True(ReferenceEquals(scan, state.RawScan));
    }

    [Fact]
    public async Task CaptureSingleScan_TimeoutKeepsThePreviousMapAndStillAttemptsStop()
    {
        await using var transport = new FakeByteTransport();
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var oldScan = new RadarScan([new RadarPoint(0, 100, 1)], DateTimeOffset.UtcNow);
        state.SetRawScan(oldScan);
        var coordinator = new RadarCoordinator(transport, state);

        await Assert.ThrowsAsync<TimeoutException>(async () =>
            await coordinator.CaptureSingleScanAsync(TimeSpan.FromMilliseconds(40), CancellationToken.None));

        Assert.True(ReferenceEquals(oldScan, state.RawScan));
        Assert.Equal("LSTOPH", Encoding.ASCII.GetString(transport.Writes[^1]));
    }

    [Fact]
    public async Task CaptureContinuous_UpdatesEveryCompletedRevolutionUntilCanceledAndStops()
    {
        await using var transport = new FakeByteTransport();
        foreach (var frame in TwoCompleteRevolutionsFrames()) transport.Enqueue(frame);
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);
        RadarScan? previousScan = null;
        var completedScans = 0;
        var twoScansReceived = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        state.Changed += () =>
        {
            if (state.RawScan is null || ReferenceEquals(state.RawScan, previousScan)) return;
            previousScan = state.RawScan;
            completedScans++;
            if (completedScans == 2) twoScansReceived.TrySetResult();
        };

        await coordinator.StartContinuousAsync();
        await twoScansReceived.Task.WaitAsync(TimeSpan.FromSeconds(1));
        await coordinator.StopContinuousAsync();

        Assert.Equal(2, completedScans);
        Assert.Equal(ExpectedSessionCommands, WrittenCommands(transport));
    }

    [Fact]
    public async Task CaptureContinuous_WithoutACompleteFirstRevolutionTimesOutAndStops()
    {
        await using var transport = new FakeByteTransport();
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var previousScan = new RadarScan([new RadarPoint(0, 250, 12)], DateTimeOffset.UtcNow);
        state.SetRawScan(previousScan);
        var coordinator = new RadarCoordinator(transport, state)
        {
            ContinuousFirstRevolutionTimeout = TimeSpan.FromMilliseconds(40)
        };
        var stopped = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        coordinator.ContinuousStateChanged += () =>
        {
            if (!coordinator.IsContinuousScanning && coordinator.ContinuousError is not null)
                stopped.TrySetResult();
        };

        await coordinator.StartContinuousAsync();
        await stopped.Task.WaitAsync(TimeSpan.FromSeconds(1));

        Assert.False(coordinator.IsContinuousScanning);
        Assert.Contains("波特率", coordinator.ContinuousError ?? string.Empty);
        Assert.True(ReferenceEquals(previousScan, state.RawScan));
        Assert.Equal(ExpectedSessionCommands, WrittenCommands(transport));
    }

    [Fact]
    public async Task CaptureContinuous_AfterFirstCompleteRevolutionDoesNotUseStartupTimeout()
    {
        await using var transport = new FakeByteTransport();
        foreach (var frame in CompleteRevolutionFrames()) transport.Enqueue(frame);
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state)
        {
            ContinuousFirstRevolutionTimeout = TimeSpan.FromMilliseconds(40)
        };
        var firstScanReceived = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        state.Changed += () =>
        {
            if (state.RawScan is not null) firstScanReceived.TrySetResult();
        };

        await coordinator.StartContinuousAsync();
        await firstScanReceived.Task.WaitAsync(TimeSpan.FromSeconds(1));
        await Task.Delay(80);

        Assert.True(coordinator.IsContinuousScanning);
        Assert.Null(coordinator.ContinuousError);
        await coordinator.StopContinuousAsync();
    }

    [Fact]
    public async Task CaptureSingleScan_PreflightFailureAbortsStartAttemptsStopAndKeepsPreviousMap()
    {
        await using var transport = new FakeByteTransport
        {
            OnWrite = bytes => Encoding.ASCII.GetString(bytes) == "LFFF1H"
                ? throw new IOException("selected write failed")
                : []
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var previousScan = new RadarScan([new RadarPoint(0, 250, 12)], DateTimeOffset.UtcNow);
        state.SetRawScan(previousScan);
        var coordinator = new RadarCoordinator(transport, state);

        var exception = await Assert.ThrowsAsync<IOException>(async () =>
            await coordinator.CaptureSingleScanAsync(TimeSpan.FromMilliseconds(100), CancellationToken.None));

        Assert.Contains("LFFF1H", exception.Message);
        Assert.Equal(new[] { "LMDMMH", "LOCONH", "LFFF1H", "LSTOPH" }, WrittenCommands(transport));
        Assert.True(ReferenceEquals(previousScan, state.RawScan));
    }

    [Fact]
    public async Task CaptureSingleScan_NewSessionClearsAStaleRadarStatus()
    {
        await using var transport = new FakeByteTransport();
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        state.SetRadarStatus(new StatusFrame(0b1111, [0, 0, 0]));
        var coordinator = new RadarCoordinator(transport, state);

        await Assert.ThrowsAsync<TimeoutException>(async () =>
            await coordinator.CaptureSingleScanAsync(TimeSpan.FromMilliseconds(40), CancellationToken.None));

        Assert.Null(state.RadarStatus);
    }

    [Fact]
    public async Task CaptureSingleScan_StatusFrameUpdatesAllReportedSettings()
    {
        await using var transport = new FakeByteTransport();
        transport.Enqueue(StatusFrameBytes(0b1011));
        foreach (var frame in CompleteRevolutionFrames()) transport.Enqueue(frame);
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None);

        var status = Assert.NotNull(state.RadarStatus);
        Assert.True(status.UsesMillimetres);
        Assert.True(status.EnergyEnabled);
        Assert.False(status.TrailingPointRemovalEnabled);
        Assert.True(status.FilterEnabled);
    }

    [Fact]
    public async Task CaptureSingleScan_CentimetreStatusNormalizesCompletedScanToMillimetres()
    {
        await using var transport = new FakeByteTransport();
        transport.Enqueue(StatusFrameBytes(0b1110));
        foreach (var frame in CompleteRevolutionFrames()) transport.Enqueue(frame);
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var scan = await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None);

        Assert.Equal((ushort)3000, scan.Points[0].DistanceMm);
        Assert.True(scan.Points.All(point => point.DistanceMm >= 3000));
    }

    [Fact]
    public async Task CaptureSingleScan_MissingStatusUsesMillimetresRequestedByPreflight()
    {
        await using var transport = new FakeByteTransport();
        foreach (var frame in CompleteRevolutionFrames()) transport.Enqueue(frame);
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var scan = await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None);

        Assert.Equal((ushort)300, scan.Points[0].DistanceMm);
        Assert.Null(state.RadarStatus);
    }

    [Fact]
    public async Task CaptureSingleScan_UnitChangeMidRevolutionDiscardsMixedAssembly()
    {
        await using var transport = new FakeByteTransport();
        transport.Enqueue(
            StatusFrameBytes(0b1111),
            FixedFrame(3500, 40, 10),
            FixedFrame(0, 40, 10),
            FixedFrame(1200, 40, 10),
            StatusFrameBytes(0b1110),
            FixedFrame(2400, 40, 10),
            FixedFrame(3500, 40, 10),
            FixedFrame(0, 40, 10),
            FixedFrame(1200, 40, 10),
            FixedFrame(2400, 40, 10),
            FixedFrame(3500, 40, 10),
            FixedFrame(0, 40, 10));
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var scan = await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None);

        Assert.Equal(40, scan.Points.Count);
        Assert.True(scan.Points.All(point => point.DistanceMm >= 3000));
    }

    [Fact]
    public async Task CaptureSingleScan_AsciiCommandReplyIsDiscardedBeforeBinaryFrames()
    {
        await using var transport = new FakeByteTransport
        {
            OnWrite = bytes => Encoding.ASCII.GetString(bytes) == "LSTARH"
                ? new[] { Encoding.ASCII.GetBytes("OK LSTARH\r\n") }
                    .Concat(CompleteRevolutionFrames())
                : []
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var scan = await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None);

        Assert.Equal(40, scan.Points.Count);
        Assert.True(state.RadarDiscardedBytes >= Encoding.ASCII.GetByteCount("OK LSTARH\r\n"));
    }

    [Fact]
    public async Task StartContinuous_RejectsACompetingSingleScanUntilStopped()
    {
        await using var transport = new FakeByteTransport();
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        await coordinator.StartContinuousAsync();
        await Assert.ThrowsAsync<InvalidOperationException>(async () =>
            await coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(1), CancellationToken.None));
        await coordinator.StopContinuousAsync();

        Assert.Equal("LSTOPH", Encoding.ASCII.GetString(transport.Writes[^1]));
    }

    [Fact]
    public async Task StaleLease_CannotStopANewerContinuousSession()
    {
        await using var transport = new FakeByteTransport();
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var firstLease = await coordinator.StartContinuousAsync();
        await coordinator.StopContinuousAsync(firstLease);
        var secondLease = await coordinator.StartContinuousAsync();

        var stopped = await coordinator.StopContinuousAsync(firstLease);

        Assert.False(stopped);
        Assert.True(coordinator.IsContinuousScanning);
        Assert.True(await coordinator.StopContinuousAsync(secondLease));
        Assert.False(coordinator.IsContinuousScanning);
    }

    [Fact]
    public async Task StopContinuous_ReportsAStopCommandWriteFailure()
    {
        await using var transport = new FakeByteTransport
        {
            OnWrite = bytes => Encoding.ASCII.GetString(bytes) == "LSTOPH"
                ? throw new IOException("stop write failed")
                : []
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var previousScan = new RadarScan([new RadarPoint(0, 250, 12)], DateTimeOffset.UtcNow);
        state.SetRawScan(previousScan);
        var coordinator = new RadarCoordinator(transport, state);

        await coordinator.StartContinuousAsync();
        var exception = await Assert.ThrowsAsync<IOException>(async () =>
            await coordinator.StopContinuousAsync());

        Assert.Contains("stop write failed", exception.Message);
        Assert.False(coordinator.IsContinuousScanning);
        Assert.True(ReferenceEquals(previousScan, state.RawScan));
    }

    [Fact]
    public async Task Disconnect_DuringSingleScanIsRejectedAndLeavesPortOpen()
    {
        var startWritten = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        await using var transport = new FakeByteTransport
        {
            OnWrite = bytes =>
            {
                if (Encoding.ASCII.GetString(bytes) == "LSTARH") startWritten.TrySetResult();
                return [];
            }
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);
        using var scanCancellation = new CancellationTokenSource();
        var scanTask = coordinator.CaptureSingleScanAsync(TimeSpan.FromSeconds(5), scanCancellation.Token);
        await startWritten.Task.WaitAsync(TimeSpan.FromSeconds(1));

        await Assert.ThrowsAsync<InvalidOperationException>(async () =>
            await coordinator.DisconnectAsync());

        Assert.True(coordinator.IsOpen);
        Assert.True(transport.IsOpen);
        scanCancellation.Cancel();
        await Assert.ThrowsAsync<OperationCanceledException>(async () => await scanTask);
    }

    [Fact]
    public async Task Disconnect_StopWriteFailureClosesPortAndReportsFailure()
    {
        await using var transport = new FakeByteTransport
        {
            OnWrite = bytes => Encoding.ASCII.GetString(bytes) == "LSTOPH"
                ? throw new IOException("stop write failed")
                : []
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);
        await coordinator.StartContinuousAsync();

        var exception = await Assert.ThrowsAsync<IOException>(async () =>
            await coordinator.DisconnectAsync());

        Assert.Contains("stop write failed", exception.Message);
        Assert.Contains("closed", exception.Message.ToLowerInvariant());
        Assert.False(coordinator.IsOpen);
        Assert.False(transport.IsOpen);
        Assert.False(coordinator.IsContinuousScanning);
    }

    [Fact]
    public async Task Disconnect_BlocksACompetingContinuousStartUntilPortIsClosed()
    {
        var closeEntered = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var allowClose = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        await using var transport = new FakeByteTransport
        {
            BeforeCloseAsync = async cancellationToken =>
            {
                closeEntered.TrySetResult();
                await allowClose.Task.WaitAsync(cancellationToken);
            }
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);
        await coordinator.StartContinuousAsync();

        var disconnectTask = coordinator.DisconnectAsync();
        await closeEntered.Task.WaitAsync(TimeSpan.FromSeconds(1));
        var competingStart = coordinator.StartContinuousAsync();

        Assert.False(competingStart.IsCompleted);
        allowClose.TrySetResult();
        await disconnectTask;
        await Assert.ThrowsAsync<InvalidOperationException>(async () => await competingStart);
        Assert.False(coordinator.IsOpen);
        Assert.False(transport.IsOpen);
        Assert.False(coordinator.IsContinuousScanning);
    }

    [Fact]
    public async Task Connect_WhenAnotherRequestOpenedTheDeviceRejectsInsteadOfReportingTheRequestedEndpoint()
    {
        var firstOpenEntered = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var allowFirstOpen = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        await using var transport = new FakeByteTransport(initiallyOpen: false)
        {
            BeforeOpenAsync = async cancellationToken =>
            {
                firstOpenEntered.TrySetResult();
                await allowFirstOpen.Task.WaitAsync(cancellationToken);
            }
        };
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var coordinator = new RadarCoordinator(transport, state);

        var firstConnect = coordinator.ConnectAsync("COM8", 921600);
        await firstOpenEntered.Task.WaitAsync(TimeSpan.FromSeconds(1));
        var competingConnect = coordinator.ConnectAsync("COM9", 500000);
        Assert.False(competingConnect.IsCompleted);

        allowFirstOpen.TrySetResult();
        await firstConnect;
        await Assert.ThrowsAsync<InvalidOperationException>(async () => await competingConnect);
        Assert.True(coordinator.IsOpen);
        Assert.True(transport.IsOpen);
    }

    private static IEnumerable<byte[]> CompleteRevolutionFrames()
    {
        yield return FixedFrame(3500, 40, 10);
        yield return FixedFrame(0, 40, 10);
        yield return FixedFrame(1200, 40, 10);
        yield return FixedFrame(2400, 40, 10);
        yield return FixedFrame(3500, 40, 10);
        yield return FixedFrame(0, 40, 10);
    }

    private static IEnumerable<byte[]> TwoCompleteRevolutionsFrames()
    {
        foreach (var frame in CompleteRevolutionFrames()) yield return frame;
        yield return FixedFrame(1200, 40, 10);
        yield return FixedFrame(2400, 40, 10);
        yield return FixedFrame(3500, 40, 10);
        yield return FixedFrame(0, 40, 10);
    }

    private static readonly string[] ExpectedSessionCommands =
        ["LMDMMH", "LOCONH", "LFFF1H", "LSSS1H", "LSTARH", "LSTOPH"];

    private static string[] WrittenCommands(FakeByteTransport transport) =>
        transport.Writes.Select(Encoding.ASCII.GetString).ToArray();

    private static byte[] StatusFrameBytes(byte flags) =>
        [0x53, 0x54, flags, 0x00, 0x00, 0x00, 0x45, 0x44];

    private static byte[] FixedFrame(ushort start, ushort sector, int count)
    {
        var measurements = Enumerable.Range(0, count)
            .Select(index => new RawMeasurement(checked((byte)(index + 1)), checked((ushort)(300 + index))))
            .ToArray();
        var bytes = new List<byte> { 0xCF, 0xFA };
        AddUInt16(bytes, checked((ushort)count));
        AddUInt16(bytes, start);
        AddUInt16(bytes, sector);
        foreach (var measurement in measurements)
        {
            bytes.Add(measurement.Energy);
            AddUInt16(bytes, measurement.DistanceMm);
        }

        AddUInt16(bytes, LdsChecksum.ComputeMeasurement(checked((ushort)count), start, sector, measurements));
        return bytes.ToArray();
    }

    private static void AddUInt16(List<byte> bytes, ushort value)
    {
        bytes.Add((byte)value);
        bytes.Add((byte)(value >> 8));
    }
}
