using Lds50cHost.Core.RadarProtocol;

namespace Lds50cHost.Core.Tests.RadarProtocol;

public sealed class Lds50cParserTests
{
    [Fact]
    public void Feed_ParsesTheDocumentedOnePointNormalFrame()
    {
        var parser = new Lds50cParser();
        byte[] frame = [0xCE, 0xFA, 0x01, 0x00, 0x00, 0x00, 0x1F, 0x69, 0x01, 0x89, 0x01];

        var parsed = Assert.Single(parser.Feed(frame));

        Assert.True(parsed is MeasurementFrame);
        var measurement = (MeasurementFrame)parsed;
        Assert.False(measurement.IsFixedResolution);
        Assert.Equal((ushort)1, measurement.PointCount);
        Assert.Equal((ushort)0, measurement.StartAngleTenths);
        Assert.Null(measurement.SectorAngleTenths);
        var point = Assert.Single(measurement.Measurements);
        Assert.Equal((byte)31, point.Energy);
        Assert.Equal((ushort)361, point.DistanceMm);
        Assert.Equal(1L, parser.ValidFrames);
    }

    [Fact]
    public void Feed_ParsesAFixedResolutionFrameWithSectorAngle()
    {
        var parser = new Lds50cParser();
        var frame = BuildMeasurementFrame(
            fixedResolution: true,
            startAngleTenths: 100,
            sectorAngleTenths: 360,
            new RawMeasurement(5, 600),
            new RawMeasurement(7, 700));

        var parsed = (MeasurementFrame)Assert.Single(parser.Feed(frame));

        Assert.True(parsed.IsFixedResolution);
        Assert.Equal((ushort)2, parsed.PointCount);
        Assert.Equal((ushort)100, parsed.StartAngleTenths);
        Assert.Equal((ushort?)360, parsed.SectorAngleTenths);
        Assert.Equal(new[] { new RawMeasurement(5, 600), new RawMeasurement(7, 700) }, parsed.Measurements);
    }

    [Fact]
    public void Feed_ParsesStatusAndAlarmFrames()
    {
        var parser = new Lds50cParser();
        byte[] bytes =
        [
            0x53, 0x54, 0x0F, 0x10, 0x20, 0x30, 0x45, 0x44,
            0xCE, 0xCE, 0xCE, 0xCE, 0x34, 0x12
        ];

        var frames = parser.Feed(bytes);

        Assert.Equal(2, frames.Count);
        Assert.True(frames[0] is StatusFrame);
        var status = (StatusFrame)frames[0];
        Assert.Equal((byte)0x0F, status.Flags);
        Assert.True(status.UsesMillimetres);
        Assert.True(status.EnergyEnabled);
        Assert.True(status.TrailingPointRemovalEnabled);
        Assert.True(status.FilterEnabled);
        Assert.Equal(new byte[] { 0x10, 0x20, 0x30 }, status.ReservedBytes);
        Assert.True(frames[1] is AlarmFrame);
        Assert.Equal((ushort)0x1234, ((AlarmFrame)frames[1]).Code);
    }

    [Fact]
    public void Feed_AcceptsAFrameOneByteAtATime()
    {
        var parser = new Lds50cParser();
        var frame = BuildMeasurementFrame(false, 900, null, new RawMeasurement(44, 1234));
        var parsed = new List<LdsFrame>();

        foreach (var value in frame)
        {
            parsed.AddRange(parser.Feed([value]));
        }

        var measurement = (MeasurementFrame)Assert.Single(parsed);
        Assert.Equal((ushort)900, measurement.StartAngleTenths);
    }

    [Fact]
    public void Feed_DiscardsNoiseBeforeAHeader()
    {
        var parser = new Lds50cParser();
        var valid = BuildMeasurementFrame(false, 0, null, new RawMeasurement(1, 100));
        byte[] bytes = [0x99, 0xCE, 0x01, .. valid];

        var parsed = parser.Feed(bytes);

        Assert.Single(parsed);
        Assert.Equal(3L, parser.DiscardedBytes);
    }

    [Fact]
    public void Feed_RecoversFromABadChecksumBeforeAValidFrame()
    {
        var parser = new Lds50cParser();
        var invalid = BuildMeasurementFrame(false, 0, null, new RawMeasurement(1, 100));
        invalid[^1] ^= 0xFF;
        var valid = BuildMeasurementFrame(false, 100, null, new RawMeasurement(2, 200));

        var parsed = parser.Feed([.. invalid, .. valid]);

        var measurement = (MeasurementFrame)Assert.Single(parsed);
        Assert.Equal((ushort)100, measurement.StartAngleTenths);
        Assert.Equal(1L, parser.ChecksumErrors);
        Assert.Equal(1L, parser.ValidFrames);
    }

    [Fact]
    public void Feed_RejectsAnExcessiveDeclaredPointCountAndResynchronizes()
    {
        var parser = new Lds50cParser(maximumPointCount: 4096);
        var valid = BuildMeasurementFrame(false, 200, null, new RawMeasurement(3, 300));
        byte[] bytes = [0xCE, 0xFA, 0x01, 0x10, .. valid];

        var parsed = parser.Feed(bytes);

        var measurement = (MeasurementFrame)Assert.Single(parsed);
        Assert.Equal((ushort)200, measurement.StartAngleTenths);
        Assert.Equal(1L, parser.LengthErrors);
    }

    private static byte[] BuildMeasurementFrame(
        bool fixedResolution,
        ushort startAngleTenths,
        ushort? sectorAngleTenths,
        params RawMeasurement[] measurements)
    {
        var bytes = new List<byte>
        {
            fixedResolution ? (byte)0xCF : (byte)0xCE,
            0xFA
        };
        AddUInt16(bytes, checked((ushort)measurements.Length));
        AddUInt16(bytes, startAngleTenths);
        if (fixedResolution) AddUInt16(bytes, sectorAngleTenths ?? throw new ArgumentNullException(nameof(sectorAngleTenths)));

        uint checksum = (uint)measurements.Length + startAngleTenths + (sectorAngleTenths ?? 0);
        foreach (var measurement in measurements)
        {
            bytes.Add(measurement.Energy);
            AddUInt16(bytes, measurement.DistanceMm);
            checksum += measurement.Energy;
            checksum += measurement.DistanceMm;
        }

        AddUInt16(bytes, unchecked((ushort)checksum));
        return bytes.ToArray();
    }

    private static void AddUInt16(List<byte> bytes, ushort value)
    {
        bytes.Add((byte)value);
        bytes.Add((byte)(value >> 8));
    }
}
