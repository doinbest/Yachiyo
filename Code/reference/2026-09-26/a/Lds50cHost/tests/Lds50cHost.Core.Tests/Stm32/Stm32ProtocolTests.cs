using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;
using Lds50cHost.Core.Stm32;

namespace Lds50cHost.Core.Tests.Stm32;

public sealed class Stm32ProtocolTests
{
    [Fact]
    public void Crc16CcittFalse_MatchesTheStandardCheckValue()
    {
        var bytes = System.Text.Encoding.ASCII.GetBytes("123456789");

        Assert.Equal((ushort)0x29B1, Crc16Ccitt.Compute(bytes));
    }

    [Fact]
    public void Encode_UsesTheExactPrefixAndLittleEndianSequenceAndLength()
    {
        var frame = new Stm32Frame(1, Stm32Command.StageConfiguration, 0x1234, [0xAA, 0xBB]);

        var encoded = Stm32FrameCodec.Encode(frame);

        Assert.Equal((byte)0xA5, encoded[0]);
        Assert.Equal((byte)0x5A, encoded[1]);
        Assert.Equal((byte)1, encoded[2]);
        Assert.Equal((byte)0x10, encoded[3]);
        Assert.Equal((byte)0x34, encoded[4]);
        Assert.Equal((byte)0x12, encoded[5]);
        Assert.Equal((byte)0x02, encoded[6]);
        Assert.Equal((byte)0x00, encoded[7]);
        Assert.Equal((byte)0xAA, encoded[8]);
        Assert.Equal((byte)0xBB, encoded[9]);
    }

    [Fact]
    public void Feed_HandlesSplitInputAndNoiseBeforeThePrefix()
    {
        var codec = new Stm32FrameCodec();
        var encoded = Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, 7, [0x42]));
        byte[] first = [0x00, 0xA5, 0x00, .. encoded[..5]];

        var before = codec.Feed(first);
        var after = codec.Feed(encoded[5..]);

        Assert.Empty(before);
        var decoded = Assert.Single(after);
        Assert.Equal((byte)1, decoded.Version);
        Assert.Equal(Stm32Command.Ack, decoded.Command);
        Assert.Equal((ushort)7, decoded.Sequence);
        Assert.Equal(new byte[] { 0x42 }, decoded.Payload);
        Assert.Equal(3L, codec.DiscardedBytes);
    }

    [Fact]
    public void Feed_RejectsCorruptCrcAndRecoversForTheNextFrame()
    {
        var codec = new Stm32FrameCodec();
        var invalid = Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, 1, []));
        invalid[^1] ^= 0xFF;
        var valid = Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, 2, []));

        var frames = codec.Feed([.. invalid, .. valid]);

        var decoded = Assert.Single(frames);
        Assert.Equal((ushort)2, decoded.Sequence);
        Assert.Equal(1L, codec.CrcErrors);
    }

    [Fact]
    public void Feed_RejectsPayloadLengthOverFourThousandAndNinetySix()
    {
        var codec = new Stm32FrameCodec();
        byte[] excessive = [0xA5, 0x5A, 0x01, 0x10, 0x01, 0x00, 0x01, 0x10];
        var valid = Stm32FrameCodec.Encode(new Stm32Frame(1, Stm32Command.Ack, 3, []));

        var frames = codec.Feed([.. excessive, .. valid]);

        Assert.Equal((ushort)3, Assert.Single(frames).Sequence);
        Assert.Equal(1L, codec.LengthErrors);
    }

    [Fact]
    public void ConfigurationPayload_VersionOneRoundTripsEveryField()
    {
        var source = AppConfiguration.CreateDefault() with
        {
            Start = MapEndpoint.Cell(1, 2),
            Goal = MapEndpoint.Cell(5, 4)
        };

        var payload = ConfigurationPayloadCodec.Encode(source);
        var decoded = ConfigurationPayloadCodec.Decode(payload);

        AssertConfigurationEqual(source, decoded);
    }

    [Fact]
    public void ConfigurationPayload_RejectsAnUnknownVersion()
    {
        var payload = ConfigurationPayloadCodec.Encode(AppConfiguration.CreateDefault());
        payload[0] = 2;

        Assert.Throws<NotSupportedException>(() => ConfigurationPayloadCodec.Decode(payload));
    }

    [Fact]
    public void PathPayload_VersionOneRoundTripsRouteEntriesAndSegments()
    {
        var configuration = AppConfiguration.CreateDefault();
        var grid = new GridDefinition(configuration.Map);
        var obstacles = ObstacleClassifier.Classify(
            grid,
            [],
            configuration.Map.ObstaclePointThreshold,
            configuration.Map.ManualBlockedMask,
            MissionPointCatalog.ProtectedCellMask);
        var plan = MissionPathPlanner.Plan(
            grid,
            obstacles,
            configuration.MissionRoute,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        var encoded = PathPayloadCodec.Encode(plan);
        var decoded = PathPayloadCodec.Decode(encoded);

        Assert.Equal((byte)1, decoded.Version);
        Assert.True(plan.Cells.Count > 25);
        Assert.Equal(plan.Cells.Count, decoded.Entries.Count);
        Assert.Equal(plan.SegmentDistancesMm.Count, decoded.Segments.Count);
        for (var index = 0; index < plan.Cells.Count; index++)
        {
            Assert.Equal((byte)plan.Cells[index].Row, decoded.Entries[index].Row);
            Assert.Equal((byte)plan.Cells[index].Column, decoded.Entries[index].Column);
            Assert.Equal((ushort)Math.Round(plan.Waypoints[index].Xmm), decoded.Entries[index].Xmm);
            Assert.Equal((ushort)Math.Round(plan.Waypoints[index].Ymm), decoded.Entries[index].Ymm);
        }

        for (var index = 0; index < plan.SegmentDistancesMm.Count; index++)
        {
            Assert.Equal((ushort)Math.Round(plan.SegmentDistancesMm[index]), decoded.Segments[index].DistanceMm);
            Assert.Equal(plan.Directions[index], decoded.Segments[index].Direction);
            Assert.Equal(plan.Turns[index], decoded.Segments[index].Turn);
        }
    }

    [Fact]
    public void PathPayload_RepeatedTwentySixEntryRouteRoundTripsInOrder()
    {
        var payload = CreatePathPayload(26);

        var decoded = PathPayloadCodec.Decode(PathPayloadCodec.Encode(payload));

        Assert.Equal(payload.Entries, decoded.Entries);
        Assert.Equal(payload.Segments, decoded.Segments);
    }

    [Fact]
    public void PathPayload_MaximumEntryRouteFitsOneFrameAndRoundTrips()
    {
        var payload = CreatePathPayload(PathPayloadCodec.MaximumEntryCount);

        var encoded = PathPayloadCodec.Encode(payload);
        var decoded = PathPayloadCodec.Decode(encoded);

        Assert.Equal(4091, encoded.Length);
        Assert.True(encoded.Length <= Stm32FrameCodec.MaximumPayloadLength);
        Assert.Equal(PathPayloadCodec.MaximumEntryCount, decoded.Entries.Count);
        Assert.Equal(PathPayloadCodec.MaximumEntryCount - 1, decoded.Segments.Count);
        Assert.Equal(payload.Entries, decoded.Entries);
        Assert.Equal(payload.Segments, decoded.Segments);
    }

    [Fact]
    public void PathPayload_RejectsFourHundredTenEntriesBeforeReadingTheirBodies()
    {
        var oversized = CreatePathPayload(PathPayloadCodec.MaximumEntryCount + 1);
        var malformedHeader = new byte[] { PathPayloadCodec.PayloadVersion, 0x9A, 0x01, 0, 0 };

        Assert.Throws<ArgumentException>(() => PathPayloadCodec.Encode(oversized));
        Assert.Throws<ArgumentException>(() => PathPayloadCodec.Decode(malformedHeader));
    }

    private static PathPayload CreatePathPayload(int entryCount)
    {
        var entries = Enumerable.Range(0, entryCount)
            .Select(index => new PathEntry(
                (byte)(index % 2 + 1),
                (byte)(index % 2 + 1),
                checked((ushort)(200 + index)),
                checked((ushort)(300 + index))))
            .ToArray();
        var segments = Enumerable.Range(0, Math.Max(0, entryCount - 1))
            .Select(index => new PathSegment(
                checked((ushort)(100 + index)),
                index % 2 == 0 ? CardinalDirection.North : CardinalDirection.South,
                index == 0 ? TurnInstruction.Start : TurnInstruction.Reverse))
            .ToArray();
        return new PathPayload(PathPayloadCodec.PayloadVersion, entries, segments);
    }

    private static void AssertConfigurationEqual(AppConfiguration expected, AppConfiguration actual)
    {
        Assert.Equal(expected.Map.RadarXmm, actual.Map.RadarXmm);
        Assert.Equal(expected.Map.RadarYmm, actual.Map.RadarYmm);
        Assert.Equal(expected.Map.XLines, actual.Map.XLines);
        Assert.Equal(expected.Map.YLines, actual.Map.YLines);
        Assert.Equal(expected.Map.ObstaclePointThreshold, actual.Map.ObstaclePointThreshold);
        Assert.Equal(expected.Map.FixedBlockedMask, actual.Map.FixedBlockedMask);
        Assert.Equal(expected.Map.ManualBlockedMask, actual.Map.ManualBlockedMask);
        Assert.Equal(expected.Filter, actual.Filter);
        Assert.Equal(expected.Start, actual.Start);
        Assert.Equal(expected.Goal, actual.Goal);
    }
}
