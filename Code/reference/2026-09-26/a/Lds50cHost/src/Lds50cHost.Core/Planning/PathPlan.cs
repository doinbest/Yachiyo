using Lds50cHost.Core.Mapping;

namespace Lds50cHost.Core.Planning;

public sealed class PathPlan
{
    private readonly GridCell[] _cells;
    private readonly PathWaypoint[] _waypoints;
    private readonly double[] _segmentDistancesMm;
    private readonly CardinalDirection[] _directions;
    private readonly TurnInstruction[] _turns;

    private PathPlan(
        bool success,
        string failureReason,
        IEnumerable<GridCell> cells,
        IEnumerable<PathWaypoint> waypoints,
        IEnumerable<double> segmentDistancesMm,
        IEnumerable<CardinalDirection> directions,
        IEnumerable<TurnInstruction> turns)
    {
        Success = success;
        FailureReason = failureReason;
        _cells = cells.ToArray();
        _waypoints = waypoints.ToArray();
        _segmentDistancesMm = segmentDistancesMm.ToArray();
        _directions = directions.ToArray();
        _turns = turns.ToArray();
    }

    public bool Success { get; }
    public string FailureReason { get; }
    public IReadOnlyList<GridCell> Cells => Array.AsReadOnly(_cells);
    public IReadOnlyList<PathWaypoint> Waypoints => Array.AsReadOnly(_waypoints);
    public IReadOnlyList<double> SegmentDistancesMm => Array.AsReadOnly(_segmentDistancesMm);
    public IReadOnlyList<CardinalDirection> Directions => Array.AsReadOnly(_directions);
    public IReadOnlyList<TurnInstruction> Turns => Array.AsReadOnly(_turns);
    public double TotalDistanceMm => _segmentDistancesMm.Sum();

    public static PathPlan Failed(string reason) =>
        new(false, reason, [], [], [], [], []);

    public static PathPlan Succeeded(
        IEnumerable<GridCell> cells,
        IEnumerable<PathWaypoint> waypoints,
        IEnumerable<double> segmentDistancesMm,
        IEnumerable<CardinalDirection> directions,
        IEnumerable<TurnInstruction> turns) =>
        new(true, string.Empty, cells, waypoints, segmentDistancesMm, directions, turns);
}
