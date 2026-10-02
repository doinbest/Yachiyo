using Lds50cHost.Core.Planning;
using Lds50cHost.Core.Stm32;

namespace Lds50cHost.Services;

public sealed record PathWaypointTableRow(
    int Sequence,
    string Cell,
    ushort Xmm,
    ushort Ymm,
    ushort? NextDistanceMm,
    CardinalDirection? NextDirection,
    TurnInstruction? NextTurn);

public static class PathWaypointTableBuilder
{
    public static IReadOnlyList<PathWaypointTableRow> Build(PathPlan plan)
    {
        var payload = PathPayloadCodec.Decode(PathPayloadCodec.Encode(plan));
        return payload.Entries.Select((entry, index) =>
        {
            PathSegment? nextSegment = index < payload.Segments.Count
                ? payload.Segments[index]
                : null;
            return new PathWaypointTableRow(
                index + 1,
                $"R{entry.Row}C{entry.Column}",
                entry.Xmm,
                entry.Ymm,
                nextSegment?.DistanceMm,
                nextSegment?.Direction,
                nextSegment?.Turn);
        }).ToArray();
    }
}
