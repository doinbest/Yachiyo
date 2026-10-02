using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;

namespace Lds50cHost.Core.Planning;

public static class MissionPathPlanner
{
    public static PathPlan Plan(
        GridDefinition grid,
        ObstacleMap obstacles,
        MissionRouteSettings settings,
        double radarXmm,
        double radarYmm)
    {
        ArgumentNullException.ThrowIfNull(grid);
        ArgumentNullException.ThrowIfNull(obstacles);
        ArgumentNullException.ThrowIfNull(settings);
        var errors = settings.Validate();
        if (errors.Count > 0) return PathPlan.Failed(string.Join(" ", errors));

        var effectiveObstacles = obstacles.WithProtectedCells(MissionPointCatalog.ProtectedCellMask);
        var points = settings.Sequence
            .Select(number => MissionPointCatalog.Resolve(number, grid, radarXmm, radarYmm))
            .ToArray();
        var cells = new List<GridCell>();
        var waypoints = new List<PathWaypoint>();

        for (var index = 0; index + 1 < points.Length; index++)
        {
            var from = points[index];
            var to = points[index + 1];
            var leg = PathPlanner.Plan(
                grid,
                effectiveObstacles,
                from.Endpoint,
                to.Endpoint,
                radarXmm,
                radarYmm);
            if (!leg.Success) return PathPlan.Failed($"任务段 {from.Number}→{to.Number} 无可用路径。");

            var skip = index == 0 ? 0 : 1;
            cells.AddRange(leg.Cells.Skip(skip));
            waypoints.AddRange(leg.Waypoints.Skip(skip));
        }

        return PathPlanBuilder.Build(cells, waypoints);
    }
}
