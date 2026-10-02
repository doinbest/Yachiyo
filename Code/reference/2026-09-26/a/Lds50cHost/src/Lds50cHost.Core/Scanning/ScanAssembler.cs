using Lds50cHost.Core.Models;
using Lds50cHost.Core.RadarProtocol;

namespace Lds50cHost.Core.Scanning;

public sealed class ScanAssembler
{
    private const ushort FullCircleTenths = 3600;

    private readonly int _minimumPointsPerScan;
    private readonly Func<DateTimeOffset> _clock;
    private readonly List<RadarPoint> _currentPoints = [];
    private MeasurementFrame? _pendingNormalFrame;
    private ushort? _lastStartAngleTenths;
    private bool _isArmed;

    public ScanAssembler(int minimumPointsPerScan = 30, Func<DateTimeOffset>? clock = null)
    {
        if (minimumPointsPerScan < 1)
            throw new ArgumentOutOfRangeException(nameof(minimumPointsPerScan));

        _minimumPointsPerScan = minimumPointsPerScan;
        _clock = clock ?? (() => DateTimeOffset.UtcNow);
    }

    public ScanAssemblyResult Push(MeasurementFrame frame)
    {
        ArgumentNullException.ThrowIfNull(frame);
        if (frame.PointCount != frame.Measurements.Length)
            throw new ArgumentException("The declared point count must match the measurement array.", nameof(frame));

        var startAngleTenths = NormalizeTenths(frame.StartAngleTenths);
        var wrapped = _lastStartAngleTenths.HasValue && startAngleTenths < _lastStartAngleTenths.Value;
        var newPoints = new List<RadarPoint>();

        if (_pendingNormalFrame is not null)
        {
            var resolved = InterpolateNormal(_pendingNormalFrame, startAngleTenths);
            if (_isArmed)
            {
                _currentPoints.AddRange(resolved);
                newPoints.AddRange(resolved);
            }
        }

        RadarScan? completedScan = null;
        if (wrapped)
        {
            if (_isArmed && _currentPoints.Count >= _minimumPointsPerScan)
            {
                completedScan = new RadarScan(_currentPoints, _clock());
            }

            _currentPoints.Clear();
            if (!_isArmed)
            {
                newPoints.Clear();
                _isArmed = true;
            }
        }

        if (frame.IsFixedResolution)
        {
            _pendingNormalFrame = null;
            if (_isArmed)
            {
                var resolved = InterpolateFixed(frame);
                _currentPoints.AddRange(resolved);
                newPoints.AddRange(resolved);
            }
        }
        else
        {
            _pendingNormalFrame = frame;
        }

        _lastStartAngleTenths = startAngleTenths;
        return new ScanAssemblyResult(newPoints, completedScan);
    }

    public void Reset()
    {
        _currentPoints.Clear();
        _pendingNormalFrame = null;
        _lastStartAngleTenths = null;
        _isArmed = false;
    }

    private static RadarPoint[] InterpolateFixed(MeasurementFrame frame)
    {
        if (frame.Measurements.Length == 0) return [];

        var sectorAngleTenths = frame.SectorAngleTenths ??
            throw new ArgumentException("A fixed-resolution frame must include a sector angle.", nameof(frame));
        var stepTenths = frame.Measurements.Length == 1
            ? 0d
            : sectorAngleTenths / (double)(frame.Measurements.Length - 1);
        return CreatePoints(frame, stepTenths);
    }

    private static RadarPoint[] InterpolateNormal(MeasurementFrame frame, ushort nextStartAngleTenths)
    {
        if (frame.Measurements.Length == 0) return [];

        var start = NormalizeTenths(frame.StartAngleTenths);
        var clockwiseDelta = (nextStartAngleTenths - start + FullCircleTenths) % FullCircleTenths;
        var stepTenths = clockwiseDelta / (double)frame.Measurements.Length;
        return CreatePoints(frame, stepTenths);
    }

    private static RadarPoint[] CreatePoints(MeasurementFrame frame, double stepTenths)
    {
        var start = NormalizeTenths(frame.StartAngleTenths);
        var points = new RadarPoint[frame.Measurements.Length];
        for (var index = 0; index < points.Length; index++)
        {
            var measurement = frame.Measurements[index];
            var angleDeg = NormalizeDegrees((start + stepTenths * index) / 10d);
            points[index] = new RadarPoint(angleDeg, measurement.DistanceMm, measurement.Energy);
        }

        return points;
    }

    private static ushort NormalizeTenths(ushort angleTenths) =>
        (ushort)(angleTenths % FullCircleTenths);

    private static double NormalizeDegrees(double angleDeg)
    {
        var normalized = angleDeg % 360d;
        return normalized < 0 ? normalized + 360d : normalized;
    }
}
