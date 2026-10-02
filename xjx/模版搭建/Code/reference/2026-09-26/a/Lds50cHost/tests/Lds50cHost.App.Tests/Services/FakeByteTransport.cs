using System.Runtime.CompilerServices;
using System.Threading.Channels;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

internal sealed class FakeByteTransport(bool initiallyOpen = true) : IByteTransport
{
    private readonly Channel<ReadOnlyMemory<byte>> _incoming = Channel.CreateUnbounded<ReadOnlyMemory<byte>>();
    private readonly List<byte[]> _writes = [];

    public bool IsOpen { get; private set; } = initiallyOpen;
    public IReadOnlyList<byte[]> Writes => _writes;
    public Func<byte[], IEnumerable<byte[]>>? OnWrite { get; set; }
    public Func<CancellationToken, Task>? BeforeOpenAsync { get; set; }
    public Func<CancellationToken, Task>? BeforeCloseAsync { get; set; }

    public async Task OpenAsync(string portName, int baudRate, CancellationToken cancellationToken = default)
    {
        if (BeforeOpenAsync is not null) await BeforeOpenAsync(cancellationToken);
        IsOpen = true;
    }

    public async Task CloseAsync(CancellationToken cancellationToken = default)
    {
        if (BeforeCloseAsync is not null) await BeforeCloseAsync(cancellationToken);
        IsOpen = false;
        _incoming.Writer.TryComplete();
    }

    public ValueTask WriteAsync(ReadOnlyMemory<byte> bytes, CancellationToken cancellationToken = default)
    {
        var copy = bytes.ToArray();
        _writes.Add(copy);
        if (OnWrite is not null)
        {
            foreach (var response in OnWrite(copy)) _incoming.Writer.TryWrite(response);
        }

        return ValueTask.CompletedTask;
    }

    public async IAsyncEnumerable<ReadOnlyMemory<byte>> ReadAllAsync(
        [EnumeratorCancellation] CancellationToken cancellationToken = default)
    {
        await foreach (var chunk in _incoming.Reader.ReadAllAsync(cancellationToken)) yield return chunk;
    }

    public void Enqueue(params byte[][] chunks)
    {
        foreach (var chunk in chunks) _incoming.Writer.TryWrite(chunk);
    }

    public ValueTask DisposeAsync()
    {
        _incoming.Writer.TryComplete();
        return ValueTask.CompletedTask;
    }
}
