using Lds50cHost.Core.Mapping;

namespace Lds50cHost.Core.Planning;

public static class PathPlanner
{
    private static readonly (int RowDelta, int ColumnDelta)[] NeighbourOffsets =
    [
        (1, 0),
        (0, 1),
        (-1, 0),
        (0, -1)
    ];

    public static PathPlan Plan(
        GridDefinition grid,
        ObstacleMap obstacles,
        MapEndpoint start,
        MapEndpoint goal,
        double radarXmm,
        double radarYmm)
    {
        ArgumentNullException.ThrowIfNull(grid);
        ArgumentNullException.ThrowIfNull(obstacles);
        ArgumentNullException.ThrowIfNull(start);
        ArgumentNullException.ThrowIfNull(goal);

        if (!IsValidCell(start.Row, start.Column)) return PathPlan.Failed("Start cell is outside the 5 x 5 map.");
        if (!IsValidCell(goal.Row, goal.Column)) return PathPlan.Failed("Goal cell is outside the 5 x 5 map.");
        if (!double.IsFinite(radarXmm) || !double.IsFinite(radarYmm))
            return PathPlan.Failed("Radar coordinates must be finite.");
        if (obstacles.IsBlocked(start.Row, start.Column)) return PathPlan.Failed("The start cell is blocked.");
        if (obstacles.IsBlocked(goal.Row, goal.Column)) return PathPlan.Failed("The goal cell is blocked.");

        var startIndex = GridCell.ToBitIndex(start.Row, start.Column);
        var goalIndex = GridCell.ToBitIndex(goal.Row, goal.Column);
        var cameFrom = Enumerable.Repeat(-1, 25).ToArray();
        var costs = Enumerable.Repeat(double.PositiveInfinity, 25).ToArray();
        var closed = new bool[25];
        var open = new PriorityQueue<int, double>();

        costs[startIndex] = 0;
        open.Enqueue(startIndex, Heuristic(grid.GetCell(start.Row, start.Column), grid.GetCell(goal.Row, goal.Column)));

        while (open.TryDequeue(out var currentIndex, out _))
        {
            if (closed[currentIndex]) continue;
            if (currentIndex == goalIndex)
            {
                var cells = ReconstructCells(grid, cameFrom, currentIndex);
                return BuildSuccess(cells, start, goal, radarXmm, radarYmm);
            }

            closed[currentIndex] = true;
            var current = CellFromIndex(grid, currentIndex);
            foreach (var (rowDelta, columnDelta) in NeighbourOffsets)
            {
                var row = current.Row + rowDelta;
                var column = current.Column + columnDelta;
                if (!IsValidCell(row, column) || obstacles.IsBlocked(row, column)) continue;

                var neighbour = grid.GetCell(row, column);
                var neighbourIndex = neighbour.BitIndex;
                if (closed[neighbourIndex]) continue;

                var tentativeCost = costs[currentIndex] + DistanceBetweenCentres(current, neighbour);
                if (tentativeCost >= costs[neighbourIndex]) continue;

                cameFrom[neighbourIndex] = currentIndex;
                costs[neighbourIndex] = tentativeCost;
                var priority = tentativeCost + Heuristic(neighbour, grid.GetCell(goal.Row, goal.Column));
                open.Enqueue(neighbourIndex, priority);
            }
        }

        return PathPlan.Failed("No route exists between the selected start and goal cells.");
    }

    private static PathPlan BuildSuccess(
        IReadOnlyList<GridCell> cells,
        MapEndpoint start,
        MapEndpoint goal,
        double radarXmm,
        double radarYmm)
    {
        var waypoints = cells
            .Select(cell => new PathWaypoint(cell.CenterXmm, cell.CenterYmm, cell.Row, cell.Column))
            .ToArray();

        waypoints[0] = ResolveEndpoint(start, cells[0], radarXmm, radarYmm);
        waypoints[^1] = ResolveEndpoint(goal, cells[^1], radarXmm, radarYmm);
        return PathPlanBuilder.Build(cells, waypoints);
    }

    private static PathWaypoint ResolveEndpoint(
        MapEndpoint endpoint,
        GridCell cell,
        double radarXmm,
        double radarYmm) => endpoint.Anchor switch
    {
        EndpointAnchor.RadarOrigin => new PathWaypoint(radarXmm, radarYmm, cell.Row, cell.Column),
        EndpointAnchor.GoalZoneOne => new PathWaypoint(2250, 150, cell.Row, cell.Column),
        _ => new PathWaypoint(cell.CenterXmm, cell.CenterYmm, cell.Row, cell.Column)
    };

    private static IReadOnlyList<GridCell> ReconstructCells(GridDefinition grid, int[] cameFrom, int currentIndex)
    {
        var cells = new List<GridCell>();
        while (currentIndex >= 0)
        {
            cells.Add(CellFromIndex(grid, currentIndex));
            currentIndex = cameFrom[currentIndex];
        }

        cells.Reverse();
        return cells;
    }

    private static GridCell CellFromIndex(GridDefinition grid, int index) =>
        grid.GetCell(index / 5 + 1, index % 5 + 1);

    private static bool IsValidCell(int row, int column) => row is >= 1 and <= 5 && column is >= 1 and <= 5;

    private static double DistanceBetweenCentres(GridCell left, GridCell right) =>
        Math.Abs(left.CenterXmm - right.CenterXmm) + Math.Abs(left.CenterYmm - right.CenterYmm);

    private static double Heuristic(GridCell current, GridCell goal) => DistanceBetweenCentres(current, goal);

}
