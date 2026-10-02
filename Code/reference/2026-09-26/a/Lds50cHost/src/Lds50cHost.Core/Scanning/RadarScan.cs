using Lds50cHost.Core.Models;

namespace Lds50cHost.Core.Scanning;

public sealed class RadarScan
{
    private readonly RadarPoint[] _points;

    public RadarScan(IEnumerable<RadarPoint> points, DateTimeOffset completedAt)
    {
        _points = points.ToArray();
        CompletedAt = completedAt;
    }

    public IReadOnlyList<RadarPoint> Points => Array.AsReadOnly(_points);
    public DateTimeOffset CompletedAt { get; }
}
