namespace Lds50cHost.Core.RadarProtocol;

public static class MeasurementUnitNormalizer
{
    public static MeasurementFrame ToMillimetres(MeasurementFrame frame, bool usesMillimetres)
    {
        ArgumentNullException.ThrowIfNull(frame);
        if (usesMillimetres) return frame;

        var measurements = frame.Measurements
            .Select(measurement => new RawMeasurement(
                measurement.Energy,
                ToMillimetres(measurement.DistanceMm)))
            .ToArray();

        return frame with { Measurements = measurements };
    }

    private static ushort ToMillimetres(ushort centimetres)
    {
        var millimetres = (uint)centimetres * 10u;
        return millimetres <= ushort.MaxValue ? (ushort)millimetres : (ushort)0;
    }
}
