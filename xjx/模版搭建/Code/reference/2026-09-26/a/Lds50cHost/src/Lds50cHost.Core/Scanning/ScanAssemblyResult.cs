using Lds50cHost.Core.Models;

namespace Lds50cHost.Core.Scanning;

public sealed class ScanAssemblyResult
{
    private readonly RadarPoint[] _newPoints;

    public ScanAssemblyResult(IEnumerable<RadarPoint> newPoints, RadarScan? completedScan)
    {
        _newPoints = newPoints.ToArray();
        CompletedScan = completedScan;
    }

    public IReadOnlyList<RadarPoint> NewPoints => Array.AsReadOnly(_newPoints);
    public RadarScan? CompletedScan { get; }
}
