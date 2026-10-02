using System.Text;
using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;
using Lds50cHost.Core.Stm32;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class Stm32CoordinatorTests
{
    [Fact]
    public async Task SaveConfigurationToFlash_SendsTheDedicatedCommandAndRequiresAck()
    {
        await using var transport = new FakeByteTransport();
        transport.OnWrite = write =>
        {
            var request = DecodeOne(write);
            return [Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, request.Sequence, []))];
        };
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.SaveConfigurationToFlashAsync(CancellationToken.None);

        Assert.True(result.Success, result.Error);
        var request = Assert.Single(DecodeWrites(transport));
        Assert.Equal(Stm32Command.SaveConfiguration, request.Command);
        Assert.Equal(0, request.Payload.Count);
    }

    [Fact]
    public async Task DownloadApplyAndVerify_PerformsTheApprovedSequenceAndMatchesReadback()
    {
        await using var transport = new FakeByteTransport();
        var configuration = AppConfiguration.CreateDefault();
        var plan = CreatePlan(configuration);
        ConfigureResponder(transport, configuration);
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.DownloadApplyAndVerifyAsync(configuration, plan, CancellationToken.None);

        Assert.True(result.Success, result.Error);
        var writes = DecodeWrites(transport);
        Assert.Equal(
            new[]
            {
                Stm32Command.StageConfiguration,
                Stm32Command.ApplyConfiguration,
                Stm32Command.SetPath,
                Stm32Command.ReadConfiguration
            },
            writes.Select(frame => frame.Command));
        var pathFrame = writes.Single(frame => frame.Command == Stm32Command.SetPath);
        var path = PathPayloadCodec.Decode(pathFrame.Payload.ToArray());
        Assert.True(path.Entries.Count > 25);
        Assert.Equal((ushort)230, path.Entries[0].Xmm);
        Assert.Equal((ushort)230, path.Entries[0].Ymm);
        Assert.Equal((ushort)230, path.Entries[^1].Xmm);
        Assert.Equal((ushort)230, path.Entries[^1].Ymm);
    }

    [Fact]
    public async Task DownloadApplyAndVerify_FailedMissionPathWritesNothing()
    {
        await using var transport = new FakeByteTransport();
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.DownloadApplyAndVerifyAsync(
            AppConfiguration.CreateDefault(),
            PathPlan.Failed("任务段 1→5 无可用路径。"),
            CancellationToken.None);

        Assert.False(result.Success);
        Assert.Contains("successful path", result.Error.ToLowerInvariant());
        Assert.Empty(transport.Writes);
    }

    [Fact]
    public async Task DownloadApplyAndVerify_FailsOnNack()
    {
        await using var transport = new FakeByteTransport();
        transport.OnWrite = write =>
        {
            var request = DecodeOne(write);
            return [Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Nack, request.Sequence, Encoding.UTF8.GetBytes("rejected")))];
        };
        var configuration = AppConfiguration.CreateDefault();
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.DownloadApplyAndVerifyAsync(configuration, CreatePlan(configuration), CancellationToken.None);

        Assert.False(result.Success);
        Assert.Contains("rejected", result.Error.ToLowerInvariant());
    }

    [Fact]
    public async Task DownloadApplyAndVerify_FailsOnTimeout()
    {
        await using var transport = new FakeByteTransport();
        var configuration = AppConfiguration.CreateDefault();
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(40));

        var result = await coordinator.DownloadApplyAndVerifyAsync(configuration, CreatePlan(configuration), CancellationToken.None);

        Assert.False(result.Success);
        Assert.Contains("timeout", result.Error.ToLowerInvariant());
    }

    [Fact]
    public async Task DownloadApplyAndVerify_RejectsOutOfOrderSequence()
    {
        await using var transport = new FakeByteTransport();
        transport.OnWrite = write =>
        {
            var request = DecodeOne(write);
            return [Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, (ushort)(request.Sequence + 1), []))];
        };
        var configuration = AppConfiguration.CreateDefault();
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.DownloadApplyAndVerifyAsync(configuration, CreatePlan(configuration), CancellationToken.None);

        Assert.False(result.Success);
        Assert.Contains("sequence", result.Error.ToLowerInvariant());
    }

    [Fact]
    public async Task DownloadApplyAndVerify_RejectsADuplicateSequence()
    {
        await using var transport = new FakeByteTransport();
        var configuration = AppConfiguration.CreateDefault();
        var decoder = new Stm32FrameCodec();
        transport.OnWrite = write =>
        {
            var request = Assert.Single(decoder.Feed(write));
            var response = Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, request.Sequence, []));
            return request.Command == Stm32Command.StageConfiguration ? [response, response] : [response];
        };
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.DownloadApplyAndVerifyAsync(configuration, CreatePlan(configuration), CancellationToken.None);

        Assert.False(result.Success);
        Assert.Contains("sequence", result.Error.ToLowerInvariant());
    }

    [Fact]
    public async Task DownloadApplyAndVerify_ReportsAFieldLevelReadbackMismatch()
    {
        await using var transport = new FakeByteTransport();
        var configuration = AppConfiguration.CreateDefault();
        var mismatched = configuration with
        {
            Map = configuration.Map with { RadarXmm = configuration.Map.RadarXmm + 1 }
        };
        ConfigureResponder(transport, mismatched);
        var coordinator = new Stm32Coordinator(transport, TimeSpan.FromMilliseconds(200));

        var result = await coordinator.DownloadApplyAndVerifyAsync(configuration, CreatePlan(configuration), CancellationToken.None);

        Assert.False(result.Success);
        Assert.Contains("radarx", result.Error.ToLowerInvariant());
    }

    private static void ConfigureResponder(FakeByteTransport transport, AppConfiguration readback)
    {
        var decoder = new Stm32FrameCodec();
        transport.OnWrite = write =>
        {
            var request = Assert.Single(decoder.Feed(write));
            var response = request.Command == Stm32Command.ReadConfiguration
                ? new Stm32Frame(1, Stm32Command.ReadConfiguration, request.Sequence, ConfigurationPayloadCodec.Encode(readback))
                : new Stm32Frame(1, Stm32Command.Ack, request.Sequence, []);
            var encoded = Stm32FrameCodec.Encode(response);
            var split = Math.Min(3, encoded.Length);
            return [encoded[..split], encoded[split..]];
        };
    }

    private static PathPlan CreatePlan(AppConfiguration configuration)
    {
        var grid = new GridDefinition(configuration.Map);
        var obstacles = ObstacleClassifier.Classify(
            grid,
            [],
            configuration.Map.ObstaclePointThreshold,
            configuration.Map.ManualBlockedMask,
            MissionPointCatalog.ProtectedCellMask);
        return configuration.PlanningMode == PlanningMode.MissionRoute
            ? MissionPathPlanner.Plan(
                grid,
                obstacles,
                configuration.MissionRoute,
                configuration.Map.RadarXmm,
                configuration.Map.RadarYmm)
            : PathPlanner.Plan(
            grid,
            obstacles,
            configuration.Start,
            configuration.Goal,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);
    }

    private static Stm32Frame DecodeOne(byte[] bytes) => Assert.Single(new Stm32FrameCodec().Feed(bytes));

    private static IReadOnlyList<Stm32Frame> DecodeWrites(FakeByteTransport transport)
    {
        var codec = new Stm32FrameCodec();
        return transport.Writes.SelectMany(write => codec.Feed(write)).ToArray();
    }
}
