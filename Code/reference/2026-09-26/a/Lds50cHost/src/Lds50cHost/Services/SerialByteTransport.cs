using System.IO.Ports;
using System.Runtime.CompilerServices;
using System.Threading.Channels;

namespace Lds50cHost.Services;

public sealed class SerialByteTransport : IByteTransport
{
    private readonly SemaphoreSlim _writeGate = new(1, 1);
    private readonly object _gate = new();
    private SerialPort? _port;
    private Channel<ReadOnlyMemory<byte>>? _incoming;
    private CancellationTokenSource? _readCancellation;
    private Task? _readTask;

    public bool IsOpen
    {
        get
        {
            lock (_gate) return _port?.IsOpen == true;
        }
    }

    public static IReadOnlyList<string> GetPortNames() => SerialPort.GetPortNames().OrderBy(name => name).ToArray();

    public Task OpenAsync(string portName, int baudRate, CancellationToken cancellationToken = default)
    {
        if (string.IsNullOrWhiteSpace(portName)) throw new ArgumentException("Serial port name is required.", nameof(portName));
        if (baudRate <= 0) throw new ArgumentOutOfRangeException(nameof(baudRate));
        cancellationToken.ThrowIfCancellationRequested();

        lock (_gate)
        {
            if (_port?.IsOpen == true) throw new InvalidOperationException("The serial port is already open.");

            var port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                Handshake = Handshake.None,
                ReadTimeout = 250,
                WriteTimeout = 1000
            };
            port.Open();
            _incoming = Channel.CreateUnbounded<ReadOnlyMemory<byte>>(new UnboundedChannelOptions
            {
                SingleWriter = true,
                SingleReader = false
            });
            _readCancellation = new CancellationTokenSource();
            _port = port;
            _readTask = Task.Factory.StartNew(
                () => SerialReadPump.Run(port.Read, _incoming.Writer, _readCancellation.Token),
                CancellationToken.None,
                TaskCreationOptions.LongRunning,
                TaskScheduler.Default);
        }

        return Task.CompletedTask;
    }

    public async Task CloseAsync(CancellationToken cancellationToken = default)
    {
        SerialPort? port;
        CancellationTokenSource? readCancellation;
        Task? readTask;
        Channel<ReadOnlyMemory<byte>>? incoming;
        lock (_gate)
        {
            port = _port;
            readCancellation = _readCancellation;
            readTask = _readTask;
            incoming = _incoming;
            _port = null;
            _readCancellation = null;
            _readTask = null;
            _incoming = null;
        }

        if (port is null) return;
        readCancellation?.Cancel();
        try { port.Close(); } catch (InvalidOperationException) { }
        if (readTask is not null)
        {
            try { await readTask.WaitAsync(cancellationToken); }
            catch (OperationCanceledException) when (readCancellation?.IsCancellationRequested == true) { }
            catch (IOException) when (readCancellation?.IsCancellationRequested == true) { }
        }

        incoming?.Writer.TryComplete();
        readCancellation?.Dispose();
        port.Dispose();
    }

    public async ValueTask WriteAsync(ReadOnlyMemory<byte> bytes, CancellationToken cancellationToken = default)
    {
        SerialPort port;
        lock (_gate)
        {
            port = _port is { IsOpen: true }
                ? _port
                : throw new InvalidOperationException("The serial port is not open.");
        }

        await _writeGate.WaitAsync(cancellationToken);
        try
        {
            cancellationToken.ThrowIfCancellationRequested();
            var payload = bytes.ToArray();
            port.Write(payload, 0, payload.Length);
        }
        finally
        {
            _writeGate.Release();
        }
    }

    public async IAsyncEnumerable<ReadOnlyMemory<byte>> ReadAllAsync(
        [EnumeratorCancellation] CancellationToken cancellationToken = default)
    {
        Channel<ReadOnlyMemory<byte>> incoming;
        lock (_gate)
        {
            incoming = _incoming ?? throw new InvalidOperationException("The serial port is not open.");
        }

        await foreach (var chunk in incoming.Reader.ReadAllAsync(cancellationToken)) yield return chunk;
    }

    public async ValueTask DisposeAsync()
    {
        await CloseAsync();
        _writeGate.Dispose();
    }

}
