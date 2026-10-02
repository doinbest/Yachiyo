using System.Runtime.ExceptionServices;
using System.Text;
using Lds50cHost.Core.RadarProtocol;
using Lds50cHost.Core.Scanning;

namespace Lds50cHost.Services;

public sealed class RadarCoordinator(IByteTransport transport, ApplicationState state)
{
    private static readonly (string Name, byte[] Bytes)[] StartupCommands =
    [
        CreateCommand("LMDMMH"),
        CreateCommand("LOCONH"),
        CreateCommand("LFFF1H"),
        CreateCommand("LSSS1H"),
        CreateCommand("LSTARH")
    ];
    private static readonly byte[] StopCommand = Encoding.ASCII.GetBytes("LSTOPH");

    private readonly SemaphoreSlim _lifecycleGate = new(1, 1);
    private readonly SemaphoreSlim _sessionGate = new(1, 1);
    private readonly object _continuousGate = new();
    private CancellationTokenSource? _continuousCancellation;
    private Task? _continuousTask;
    private Exception? _continuousFailure;
    private ContinuousScanLease? _activeContinuousLease;
    private long _nextContinuousLeaseId;

    public event Action? ContinuousStateChanged;

    public bool IsOpen => transport.IsOpen;

    public bool IsContinuousScanning
    {
        get { lock (_continuousGate) return _continuousCancellation is not null; }
    }

    public string? ContinuousError
    {
        get { lock (_continuousGate) return _continuousFailure?.Message; }
    }

    internal TimeSpan ContinuousFirstRevolutionTimeout { get; init; } = TimeSpan.FromSeconds(8);

    public async Task<RadarScan> CaptureSingleScanAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        if (timeout <= TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(timeout));
        await EnterOperationAsync(cancellationToken);
        try
        {
            return await CaptureSingleSessionAsync(timeout, cancellationToken);
        }
        finally
        {
            _sessionGate.Release();
        }
    }

    public async Task<ContinuousScanLease> StartContinuousAsync(CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        CancellationTokenSource cancellation;
        TaskCompletionSource completion;
        ContinuousScanLease lease;

        await _lifecycleGate.WaitAsync(cancellationToken);
        try
        {
            EnsureOpen();
            lock (_continuousGate)
            {
                if (_continuousCancellation is not null)
                    throw new InvalidOperationException("Continuous radar scanning is already active.");

                EnterSessionOrThrow();
                try
                {
                    cancellation = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
                    completion = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
                    lease = new ContinuousScanLease(checked(++_nextContinuousLeaseId));
                    _continuousCancellation = cancellation;
                    _continuousTask = completion.Task;
                    _continuousFailure = null;
                    _activeContinuousLease = lease;
                }
                catch
                {
                    _sessionGate.Release();
                    throw;
                }
            }
        }
        finally
        {
            _lifecycleGate.Release();
        }

        _ = RunContinuousOwnedSessionAsync(lease, cancellation, completion);
        ContinuousStateChanged?.Invoke();
        return lease;
    }

    public async Task<bool> StopContinuousAsync(ContinuousScanLease? expectedLease = null)
    {
        await _lifecycleGate.WaitAsync();
        try
        {
            return await StopContinuousCoreAsync(expectedLease);
        }
        finally
        {
            _lifecycleGate.Release();
        }
    }

    public async Task ConnectAsync(
        string portName,
        int baudRate,
        CancellationToken cancellationToken = default)
    {
        if (string.IsNullOrWhiteSpace(portName))
            throw new ArgumentException("A serial port is required.", nameof(portName));

        await _lifecycleGate.WaitAsync(cancellationToken);
        try
        {
            if (transport.IsOpen)
                throw new InvalidOperationException("The radar serial port is already open.");
            EnterSessionOrThrow();
            try
            {
                await transport.OpenAsync(portName, baudRate, cancellationToken);
            }
            finally
            {
                _sessionGate.Release();
            }
        }
        finally
        {
            _lifecycleGate.Release();
        }
    }

    public async Task DisconnectAsync(CancellationToken cancellationToken = default)
    {
        await _lifecycleGate.WaitAsync(cancellationToken);
        try
        {
            Exception? stopFailure = null;
            if (IsContinuousScanning)
            {
                try
                {
                    await StopContinuousCoreAsync();
                }
                catch (Exception exception)
                {
                    stopFailure = exception;
                }
            }

            if (!_sessionGate.Wait(0))
                throw new InvalidOperationException("Another radar operation is already active; the serial port remains open.");

            Exception? closeFailure = null;
            try
            {
                if (transport.IsOpen) await transport.CloseAsync(cancellationToken);
            }
            catch (Exception exception)
            {
                closeFailure = exception;
            }
            finally
            {
                _sessionGate.Release();
            }

            if (stopFailure is not null)
            {
                if (closeFailure is not null)
                {
                    throw new IOException(
                        $"Radar stop failed ({stopFailure.Message}) and closing the serial port also failed ({closeFailure.Message}).",
                        new AggregateException(stopFailure, closeFailure));
                }

                throw new IOException(
                    $"Radar stop failed, but the serial port was closed: {stopFailure.Message}",
                    stopFailure);
            }

            if (closeFailure is not null) ExceptionDispatchInfo.Capture(closeFailure).Throw();
        }
        finally
        {
            _lifecycleGate.Release();
        }
    }

    private async Task<bool> StopContinuousCoreAsync(ContinuousScanLease? expectedLease = null)
    {
        CancellationTokenSource? cancellation;
        Task? task;
        Exception? failure;
        lock (_continuousGate)
        {
            if (expectedLease is not null && _activeContinuousLease != expectedLease)
                return false;

            cancellation = _continuousCancellation;
            task = _continuousTask;
            failure = _continuousFailure;
        }

        if (cancellation is not null && task is not null)
        {
            cancellation.Cancel();
            await task;
            lock (_continuousGate) failure = _continuousFailure;
        }

        if (failure is not null) ExceptionDispatchInfo.Capture(failure).Throw();
        return true;
    }

    public async Task SendCommandAsync(string command, CancellationToken cancellationToken = default)
    {
        if (string.IsNullOrWhiteSpace(command)) throw new ArgumentException("Radar command is required.", nameof(command));
        await EnterOperationAsync(cancellationToken);
        try
        {
            await transport.WriteAsync(Encoding.ASCII.GetBytes(command), cancellationToken);
        }
        finally
        {
            _sessionGate.Release();
        }
    }

    private async Task<RadarScan> CaptureSingleSessionAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        var parser = new Lds50cParser();
        var assembler = new ScanAssembler();
        using var timeoutCancellation = new CancellationTokenSource(timeout);
        using var linkedCancellation = CancellationTokenSource.CreateLinkedTokenSource(
            cancellationToken,
            timeoutCancellation.Token);
        var usesMillimetres = true;

        try
        {
            await ConfigureAndStartAsync(cancellationToken);
            await foreach (var chunk in transport.ReadAllAsync(linkedCancellation.Token))
            {
                foreach (var frame in parser.Feed(chunk.Span))
                {
                    var completedScan = ProcessFrame(frame, assembler, ref usesMillimetres);
                    if (completedScan is null) continue;
                    state.SetRawScan(completedScan);
                    return completedScan;
                }
            }

            throw new IOException("Radar input ended before a complete revolution was received.");
        }
        catch (OperationCanceledException) when (timeoutCancellation.IsCancellationRequested && !cancellationToken.IsCancellationRequested)
        {
            throw new TimeoutException("Timed out while waiting for one complete radar revolution.");
        }
        finally
        {
            state.SetParserCounters(parser.ValidFrames, parser.ChecksumErrors, parser.LengthErrors, parser.DiscardedBytes);
            try { await transport.WriteAsync(StopCommand, CancellationToken.None); }
            catch (Exception exception) when (exception is IOException or InvalidOperationException or UnauthorizedAccessException)
            {
                state.AddLog($"Radar stop command failed: {exception.Message}");
            }
        }
    }

    private async Task RunContinuousOwnedSessionAsync(
        ContinuousScanLease lease,
        CancellationTokenSource cancellation,
        TaskCompletionSource completion)
    {
        Exception? failure = null;
        try
        {
            await CaptureContinuousSessionAsync(cancellation.Token);
        }
        catch (Exception exception)
        {
            failure = exception;
            state.AddLog($"Continuous radar scan failed: {exception.Message}");
        }
        finally
        {
            _sessionGate.Release();
            lock (_continuousGate)
            {
                if (ReferenceEquals(_continuousCancellation, cancellation) && _activeContinuousLease == lease)
                {
                    _continuousCancellation = null;
                    _continuousTask = null;
                    _continuousFailure = failure;
                    _activeContinuousLease = null;
                }
            }

            cancellation.Dispose();
            completion.TrySetResult();
            ContinuousStateChanged?.Invoke();
        }
    }

    private async Task CaptureContinuousSessionAsync(CancellationToken cancellationToken)
    {
        var parser = new Lds50cParser();
        var assembler = new ScanAssembler();
        var usesMillimetres = true;

        try
        {
            await ConfigureAndStartAsync(cancellationToken);
            if (ContinuousFirstRevolutionTimeout <= TimeSpan.Zero)
                throw new InvalidOperationException("Continuous first-revolution timeout must be positive.");

            using var firstRevolutionCancellation = new CancellationTokenSource(
                ContinuousFirstRevolutionTimeout);
            using var readCancellation = CancellationTokenSource.CreateLinkedTokenSource(
                cancellationToken,
                firstRevolutionCancellation.Token);
            var receivedFirstRevolution = false;

            try
            {
                await foreach (var chunk in transport.ReadAllAsync(readCancellation.Token))
                {
                    foreach (var frame in parser.Feed(chunk.Span))
                    {
                        var completedScan = ProcessFrame(frame, assembler, ref usesMillimetres);
                        if (completedScan is null) continue;
                        if (!receivedFirstRevolution)
                        {
                            receivedFirstRevolution = true;
                            firstRevolutionCancellation.CancelAfter(Timeout.InfiniteTimeSpan);
                        }

                        state.SetRawScan(completedScan);
                        state.SetParserCounters(
                            parser.ValidFrames,
                            parser.ChecksumErrors,
                            parser.LengthErrors,
                            parser.DiscardedBytes);
                    }
                }
            }
            catch (OperationCanceledException) when (
                firstRevolutionCancellation.IsCancellationRequested &&
                !receivedFirstRevolution &&
                !cancellationToken.IsCancellationRequested)
            {
                throw new TimeoutException(
                    "8 秒内未收到完整一圈雷达数据，请检查波特率、串口连接和雷达供电。");
            }

            throw new IOException("Radar input ended while continuous scanning was active.");
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
        }
        finally
        {
            state.SetParserCounters(parser.ValidFrames, parser.ChecksumErrors, parser.LengthErrors, parser.DiscardedBytes);
            await transport.WriteAsync(StopCommand, CancellationToken.None);
        }
    }

    private async Task ConfigureAndStartAsync(CancellationToken cancellationToken)
    {
        state.ClearRadarStatus();
        foreach (var command in StartupCommands)
        {
            try
            {
                await transport.WriteAsync(command.Bytes, cancellationToken);
            }
            catch (OperationCanceledException)
            {
                throw;
            }
            catch (Exception exception)
            {
                throw new IOException(
                    $"Radar preflight command {command.Name} failed: {exception.Message}",
                    exception);
            }
        }
    }

    private RadarScan? ProcessFrame(
        LdsFrame frame,
        ScanAssembler assembler,
        ref bool usesMillimetres)
    {
        if (frame is AlarmFrame alarm)
        {
            state.AddLog($"Radar alarm 0x{alarm.Code:X4}.");
            return null;
        }

        if (frame is StatusFrame status)
        {
            if (status.UsesMillimetres != usesMillimetres) assembler.Reset();
            usesMillimetres = status.UsesMillimetres;
            state.SetRadarStatus(status);
            return null;
        }

        if (frame is not MeasurementFrame measurement) return null;
        return assembler
            .Push(MeasurementUnitNormalizer.ToMillimetres(measurement, usesMillimetres))
            .CompletedScan;
    }

    private static (string Name, byte[] Bytes) CreateCommand(string command) =>
        (command, Encoding.ASCII.GetBytes(command));

    private void EnterSessionOrThrow()
    {
        if (!_sessionGate.Wait(0))
            throw new InvalidOperationException("Another radar operation is already active.");
    }

    private async Task EnterOperationAsync(CancellationToken cancellationToken)
    {
        await _lifecycleGate.WaitAsync(cancellationToken);
        try
        {
            EnsureOpen();
            EnterSessionOrThrow();
        }
        finally
        {
            _lifecycleGate.Release();
        }
    }

    private void EnsureOpen()
    {
        if (!transport.IsOpen) throw new InvalidOperationException("The radar serial port is not open.");
    }
}
