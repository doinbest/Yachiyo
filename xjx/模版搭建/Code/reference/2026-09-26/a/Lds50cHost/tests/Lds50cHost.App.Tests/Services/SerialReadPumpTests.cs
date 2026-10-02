using System.Threading.Channels;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class SerialReadPumpTests
{
    [Fact]
    public async Task Run_UsesBlockingReaderAndPublishesReceivedBytes()
    {
        using var cancellation = new CancellationTokenSource();
        var incoming = Channel.CreateUnbounded<ReadOnlyMemory<byte>>();
        var readCount = 0;

        int Read(byte[] buffer, int offset, int count)
        {
            if (Interlocked.Increment(ref readCount) == 1)
            {
                var bytes = new byte[] { 0xCE, 0xFA, 0x02, 0x00 };
                bytes.CopyTo(buffer, offset);
                return bytes.Length;
            }

            cancellation.Cancel();
            throw new TimeoutException();
        }

        var pump = Task.Run(() => SerialReadPump.Run(Read, incoming.Writer, cancellation.Token));
        var chunk = await incoming.Reader.ReadAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(1));
        await pump.WaitAsync(TimeSpan.FromSeconds(1));

        Assert.Equal(new byte[] { 0xCE, 0xFA, 0x02, 0x00 }, chunk.ToArray());
    }
}
