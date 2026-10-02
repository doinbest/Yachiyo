namespace Lds50cHost.Core.RadarProtocol;

public static class LdsChecksum
{
    public static ushort ComputeMeasurement(
        ushort pointCount,
        ushort startAngleTenths,
        ushort? sectorAngleTenths,
        ReadOnlySpan<RawMeasurement> measurements)
    {
        uint sum = pointCount;
        sum += startAngleTenths;
        sum += sectorAngleTenths ?? 0;

        foreach (var measurement in measurements)
        {
            sum += measurement.Energy;
            sum += measurement.DistanceMm;
        }

        return unchecked((ushort)sum);
    }
}
