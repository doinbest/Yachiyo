using Lds50cHost.Core.RadarProtocol;

namespace Lds50cHost.Core.Tests.RadarProtocol;

public sealed class MeasurementUnitNormalizerTests
{
    [Fact]
    public void ToMillimetres_MillimetreFramePreservesDistancesAndMetadata()
    {
        var frame = CreateFrame(
            new RawMeasurement(17, 0),
            new RawMeasurement(23, 1234));

        var normalized = MeasurementUnitNormalizer.ToMillimetres(frame, usesMillimetres: true);

        Assert.True(ReferenceEquals(frame, normalized));
        Assert.True(normalized.IsFixedResolution);
        Assert.Equal((ushort)2, normalized.PointCount);
        Assert.Equal((ushort)123, normalized.StartAngleTenths);
        Assert.Equal((ushort?)456, normalized.SectorAngleTenths);
        Assert.Equal(
            new[] { new RawMeasurement(17, 0), new RawMeasurement(23, 1234) },
            normalized.Measurements);
    }

    [Fact]
    public void ToMillimetres_CentimetreFrameMultipliesDistancesByTen()
    {
        var frame = CreateFrame(
            new RawMeasurement(31, 0),
            new RawMeasurement(47, 361));

        var normalized = MeasurementUnitNormalizer.ToMillimetres(frame, usesMillimetres: false);

        Assert.False(ReferenceEquals(frame, normalized));
        Assert.True(normalized.IsFixedResolution);
        Assert.Equal((ushort)2, normalized.PointCount);
        Assert.Equal((ushort)123, normalized.StartAngleTenths);
        Assert.Equal((ushort?)456, normalized.SectorAngleTenths);
        Assert.Equal(
            new[] { new RawMeasurement(31, 0), new RawMeasurement(47, 3610) },
            normalized.Measurements);
    }

    [Fact]
    public void ToMillimetres_CentimetreOverflowBecomesInvalidZero()
    {
        var frame = CreateFrame(
            new RawMeasurement(52, 6553),
            new RawMeasurement(61, 6554),
            new RawMeasurement(70, ushort.MaxValue));

        var normalized = MeasurementUnitNormalizer.ToMillimetres(frame, usesMillimetres: false);

        Assert.Equal(
            new[]
            {
                new RawMeasurement(52, 65530),
                new RawMeasurement(61, 0),
                new RawMeasurement(70, 0)
            },
            normalized.Measurements);
    }

    private static MeasurementFrame CreateFrame(params RawMeasurement[] measurements) =>
        new(
            IsFixedResolution: true,
            PointCount: checked((ushort)measurements.Length),
            StartAngleTenths: 123,
            SectorAngleTenths: 456,
            Measurements: measurements);
}
