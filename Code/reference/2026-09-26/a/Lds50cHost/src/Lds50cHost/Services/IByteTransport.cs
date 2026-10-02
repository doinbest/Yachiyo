namespace Lds50cHost.Services;

public interface IByteTransport : IAsyncDisposable
{
    bool IsOpen { get; }
    Task OpenAsync(string portName, int baudRate, CancellationToken cancellationToken = default);
    Task CloseAsync(CancellationToken cancellationToken = default);
    ValueTask WriteAsync(ReadOnlyMemory<byte> bytes, CancellationToken cancellationToken = default);
    IAsyncEnumerable<ReadOnlyMemory<byte>> ReadAllAsync(CancellationToken cancellationToken = default);
}
