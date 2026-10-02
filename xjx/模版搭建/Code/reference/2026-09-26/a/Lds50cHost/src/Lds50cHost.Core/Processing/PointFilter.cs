using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Models;

namespace Lds50cHost.Core.Processing;

public static class PointFilter
{
    public static IReadOnlyList<RadarPoint> Apply(
        IEnumerable<RadarPoint> source,
        FilterSettings settings)
    {
        ArgumentNullException.ThrowIfNull(source);
        ArgumentNullException.ThrowIfNull(settings);

        var errors = settings.Validate();
        if (errors.Count > 0)
            throw new ArgumentException(string.Join(" ", errors), nameof(settings));

        return source
            .Where(point =>
                settings.ContainsAngle(point.AngleDeg) &&
                point.DistanceMm >= settings.MinDistanceMm &&
                point.DistanceMm <= settings.MaxDistanceMm &&
                point.Energy >= settings.MinEnergy &&
                point.Energy <= settings.MaxEnergy)
            .ToArray();
    }
}
