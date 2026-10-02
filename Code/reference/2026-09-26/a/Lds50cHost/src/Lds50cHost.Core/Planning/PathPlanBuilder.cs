using Lds50cHost.Core.Mapping;

namespace Lds50cHost.Core.Planning;

public static class PathPlanBuilder
{
    public static PathPlan Build(
        IReadOnlyList<GridCell> cells,
        IReadOnlyList<PathWaypoint> waypoints)
    {
        ArgumentNullException.ThrowIfNull(cells);
        ArgumentNullException.ThrowIfNull(waypoints);
        if (cells.Count == 0) throw new ArgumentException("A successful route must contain at least one cell.", nameof(cells));
        if (cells.Count != waypoints.Count)
            throw new ArgumentException("Each route cell must have one waypoint.", nameof(waypoints));

        var distances = new double[waypoints.Count - 1];
        var directions = new CardinalDirection[cells.Count - 1];
        var turns = new TurnInstruction[directions.Length];
        for (var index = 0; index < directions.Length; index++)
        {
            var deltaX = waypoints[index + 1].Xmm - waypoints[index].Xmm;
            var deltaY = waypoints[index + 1].Ymm - waypoints[index].Ymm;
            distances[index] = Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
            directions[index] = DirectionBetween(cells[index], cells[index + 1]);
            turns[index] = index == 0 ? TurnInstruction.Start : TurnBetween(directions[index - 1], directions[index]);
        }

        return PathPlan.Succeeded(cells, waypoints, distances, directions, turns);
    }

    private static CardinalDirection DirectionBetween(GridCell from, GridCell to)
    {
        if (to.Row == from.Row + 1 && to.Column == from.Column) return CardinalDirection.North;
        if (to.Row == from.Row && to.Column == from.Column + 1) return CardinalDirection.East;
        if (to.Row == from.Row - 1 && to.Column == from.Column) return CardinalDirection.South;
        if (to.Row == from.Row && to.Column == from.Column - 1) return CardinalDirection.West;
        throw new InvalidOperationException("Path cells must be four-neighbour adjacent.");
    }

    private static TurnInstruction TurnBetween(CardinalDirection previous, CardinalDirection current)
    {
        var delta = ((int)current - (int)previous + 4) % 4;
        return delta switch
        {
            0 => TurnInstruction.Straight,
            1 => TurnInstruction.Right,
            2 => TurnInstruction.Reverse,
            3 => TurnInstruction.Left,
            _ => throw new InvalidOperationException()
        };
    }
}
