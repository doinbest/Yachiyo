namespace Lds50cHost.Core.RadarProtocol;

public readonly record struct RawMeasurement(byte Energy, ushort DistanceMm);

public sealed record MeasurementFrame(
    bool IsFixedResolution,
    ushort PointCount,
    ushort StartAngleTenths,
    ushort? SectorAngleTenths,
    RawMeasurement[] Measurements) : LdsFrame;
