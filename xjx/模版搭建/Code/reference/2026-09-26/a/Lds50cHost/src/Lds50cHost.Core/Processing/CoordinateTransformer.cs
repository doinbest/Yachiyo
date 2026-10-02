using Lds50cHost.Core.Models;

namespace Lds50cHost.Core.Processing;

public static class CoordinateTransformer
{
    public static FieldPoint ToField(RadarPoint point, double radarXmm, double radarYmm)
    {
        var angleRadians = point.AngleDeg * Math.PI / 180d;
        return new FieldPoint(
            radarXmm + point.DistanceMm * Math.Sin(angleRadians),
            radarYmm + point.DistanceMm * Math.Cos(angleRadians),
            point);
    }
}
