using System.Text;
using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Planning;
using Lds50cHost.Core.Stm32;

namespace Lds50cHost.Services;

public sealed record DownloadResult(bool Success, string Error)
{
    public static DownloadResult Ok() => new(true, string.Empty);
    public static DownloadResult Failed(string error) => new(false, error);
}

public sealed class Stm32Coordinator(IByteTransport transport, TimeSpan responseTimeout)
{
    public async Task<DownloadResult> SaveConfigurationToFlashAsync(CancellationToken cancellationToken)
    {
        if (responseTimeout <= TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(responseTimeout));

        var codec = new Stm32FrameCodec();
        var pending = new Queue<Stm32Frame>();
        using var readCancellation = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        await using var reader = transport.ReadAllAsync(readCancellation.Token).GetAsyncEnumerator(readCancellation.Token);
        try
        {
            const ushort sequence = 1;
            var response = await SendAndReceiveAsync(
                Stm32Command.SaveConfiguration,
                sequence,
                ReadOnlyMemory<byte>.Empty,
                codec,
                pending,
                reader,
                readCancellation,
                cancellationToken);
            var error = ValidateAck(response, sequence, "save configuration to Flash");
            return error is null ? DownloadResult.Ok() : DownloadResult.Failed(error);
        }
        catch (TimeoutException exception)
        {
            return DownloadResult.Failed($"STM32 response timeout: {exception.Message}");
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            return DownloadResult.Failed("STM32 response timeout.");
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception exception) when (exception is ArgumentException or InvalidOperationException or IOException or NotSupportedException)
        {
            return DownloadResult.Failed(exception.Message);
        }
    }

    public async Task<DownloadResult> DownloadApplyAndVerifyAsync(
        AppConfiguration configuration,
        PathPlan path,
        CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(configuration);
        ArgumentNullException.ThrowIfNull(path);
        if (responseTimeout <= TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(responseTimeout));

        var codec = new Stm32FrameCodec();
        var pending = new Queue<Stm32Frame>();
        using var readCancellation = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        await using var reader = transport.ReadAllAsync(readCancellation.Token).GetAsyncEnumerator(readCancellation.Token);
        ushort sequence = 1;
        try
        {
            var configurationPayload = ConfigurationPayloadCodec.Encode(configuration);
            var pathPayload = PathPayloadCodec.Encode(path);

            var stageSequence = sequence++;
            var stage = await SendAndReceiveAsync(
                Stm32Command.StageConfiguration,
                stageSequence,
                configurationPayload,
                codec,
                pending,
                reader,
                readCancellation,
                cancellationToken);
            var error = ValidateAck(stage, stageSequence, "stage configuration");
            if (error is not null) return DownloadResult.Failed(error);

            var applySequence = sequence++;
            var apply = await SendAndReceiveAsync(
                Stm32Command.ApplyConfiguration,
                applySequence,
                ReadOnlyMemory<byte>.Empty,
                codec,
                pending,
                reader,
                readCancellation,
                cancellationToken);
            error = ValidateAck(apply, applySequence, "apply configuration");
            if (error is not null) return DownloadResult.Failed(error);

            var setPathSequence = sequence++;
            var setPath = await SendAndReceiveAsync(
                Stm32Command.SetPath,
                setPathSequence,
                pathPayload,
                codec,
                pending,
                reader,
                readCancellation,
                cancellationToken);
            error = ValidateAck(setPath, setPathSequence, "set path");
            if (error is not null) return DownloadResult.Failed(error);

            var readSequence = sequence++;
            var readback = await SendAndReceiveAsync(
                Stm32Command.ReadConfiguration,
                readSequence,
                ReadOnlyMemory<byte>.Empty,
                codec,
                pending,
                reader,
                readCancellation,
                cancellationToken);
            if (readback.Sequence != readSequence)
                return DownloadResult.Failed($"Readback sequence mismatch: expected {readSequence}, received {readback.Sequence}.");
            if (readback.Command == Stm32Command.Nack) return DownloadResult.Failed(DescribeNack(readback, "read configuration"));
            if (readback.Command != Stm32Command.ReadConfiguration)
                return DownloadResult.Failed($"Expected ReadConfiguration response, received {readback.Command}.");

            var expected = ConfigurationPayloadCodec.Decode(configurationPayload);
            var actual = ConfigurationPayloadCodec.Decode(readback.Payload.ToArray());
            var mismatch = DescribeConfigurationMismatch(expected, actual);
            return mismatch is null ? DownloadResult.Ok() : DownloadResult.Failed(mismatch);
        }
        catch (TimeoutException exception)
        {
            return DownloadResult.Failed($"STM32 response timeout: {exception.Message}");
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            return DownloadResult.Failed("STM32 response timeout.");
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception exception) when (exception is ArgumentException or InvalidOperationException or IOException or NotSupportedException)
        {
            return DownloadResult.Failed(exception.Message);
        }
    }

    private async Task<Stm32Frame> SendAndReceiveAsync(
        Stm32Command command,
        ushort sequence,
        ReadOnlyMemory<byte> payload,
        Stm32FrameCodec codec,
        Queue<Stm32Frame> pending,
        IAsyncEnumerator<ReadOnlyMemory<byte>> reader,
        CancellationTokenSource readCancellation,
        CancellationToken cancellationToken)
    {
        var request = new Stm32Frame(1, command, sequence, payload.ToArray());
        await transport.WriteAsync(Stm32FrameCodec.Encode(request), cancellationToken);
        return await ReceiveAsync(codec, pending, reader, readCancellation);
    }

    private async Task<Stm32Frame> ReceiveAsync(
        Stm32FrameCodec codec,
        Queue<Stm32Frame> pending,
        IAsyncEnumerator<ReadOnlyMemory<byte>> reader,
        CancellationTokenSource readCancellation)
    {
        while (pending.Count == 0)
        {
            readCancellation.CancelAfter(responseTimeout);
            bool hasData;
            try
            {
                hasData = await reader.MoveNextAsync();
            }
            finally
            {
                if (!readCancellation.IsCancellationRequested)
                    readCancellation.CancelAfter(Timeout.InfiniteTimeSpan);
            }
            if (!hasData) throw new IOException("STM32 input ended before a response frame was received.");
            foreach (var frame in codec.Feed(reader.Current.Span)) pending.Enqueue(frame);
        }

        return pending.Dequeue();
    }

    private static string? ValidateAck(Stm32Frame response, ushort expectedSequence, string operation)
    {
        if (response.Sequence != expectedSequence)
            return $"Response sequence mismatch during {operation}: expected {expectedSequence}, received {response.Sequence}.";
        if (response.Command == Stm32Command.Nack) return DescribeNack(response, operation);
        return response.Command == Stm32Command.Ack
            ? null
            : $"Expected ACK during {operation}, received {response.Command}.";
    }

    private static string DescribeNack(Stm32Frame frame, string operation)
    {
        var reason = frame.Payload.Count == 0 ? "no reason supplied" : Encoding.UTF8.GetString(frame.Payload.ToArray());
        return $"STM32 NACK during {operation}: {reason}.";
    }

    private static string? DescribeConfigurationMismatch(AppConfiguration expected, AppConfiguration actual)
    {
        if (expected.Map.RadarXmm != actual.Map.RadarXmm) return $"Readback mismatch: RadarX expected {expected.Map.RadarXmm}, received {actual.Map.RadarXmm}.";
        if (expected.Map.RadarYmm != actual.Map.RadarYmm) return $"Readback mismatch: RadarY expected {expected.Map.RadarYmm}, received {actual.Map.RadarYmm}.";
        if (!expected.Map.XLines.SequenceEqual(actual.Map.XLines)) return "Readback mismatch: XLines differ.";
        if (!expected.Map.YLines.SequenceEqual(actual.Map.YLines)) return "Readback mismatch: YLines differ.";
        if (expected.Filter != actual.Filter) return "Readback mismatch: Filter settings differ.";
        if (expected.Map.ObstaclePointThreshold != actual.Map.ObstaclePointThreshold) return "Readback mismatch: ObstaclePointThreshold differs.";
        if (expected.Map.FixedBlockedMask != actual.Map.FixedBlockedMask) return "Readback mismatch: FixedBlockedMask differs.";
        if (expected.Map.ManualBlockedMask != actual.Map.ManualBlockedMask) return "Readback mismatch: ManualBlockedMask differs.";
        if (expected.Start != actual.Start) return "Readback mismatch: Start endpoint differs.";
        if (expected.Goal != actual.Goal) return "Readback mismatch: Goal endpoint differs.";
        return null;
    }
}
