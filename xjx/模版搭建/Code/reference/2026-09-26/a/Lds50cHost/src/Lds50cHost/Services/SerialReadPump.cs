using System.Threading.Channels;

namespace Lds50cHost.Services;

public delegate int BlockingSerialRead(byte[] buffer, int offset, int count);

public static class SerialReadPump
{
    public static void Run(
        BlockingSerialRead read,
        ChannelWriter<ReadOnlyMemory<byte>> writer,
        CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(read);
        ArgumentNullException.ThrowIfNull(writer);

        var buffer = new byte[4096];
        Exception? failure = null;
        try
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                int count;
                try
                {
                    count = read(buffer, 0, buffer.Length);
                }
                catch (TimeoutException)
                {
                    continue;
                }

                if (count > 0) writer.TryWrite(buffer.AsMemory(0, count).ToArray());
            }
        }
        catch (Exception exception) when (exception is IOException or UnauthorizedAccessException or InvalidOperationException)
        {
            if (!cancellationToken.IsCancellationRequested) failure = exception;
        }
        finally
        {
            writer.TryComplete(failure);
        }
    }
}
