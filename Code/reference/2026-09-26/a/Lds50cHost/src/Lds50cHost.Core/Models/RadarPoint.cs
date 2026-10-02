namespace Lds50cHost.Core.Models;

public readonly record struct RadarPoint(double AngleDeg, ushort DistanceMm, byte Energy);
