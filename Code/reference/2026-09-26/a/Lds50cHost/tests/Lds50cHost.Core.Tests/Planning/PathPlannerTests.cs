using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Tests.Planning;

public sealed class PathPlannerTests
{
    [Fact]
    public void Plan_DefaultStartZoneTwoToGoalZoneOneUsesTheBottomRow()
    {
        var (grid, obstacles) = CreateMap();

        var plan = PathPlanner.Plan(
            grid,
            obstacles,
            MapEndpoint.StartZone2,
            MapEndpoint.GoalZone1,
            230,
            230);

        Assert.True(plan.Success, plan.FailureReason);
        Assert.Equal(new[] { 1, 2, 3, 4, 5 }, plan.Cells.Select(cell => cell.Column));
        Assert.True(plan.Cells.All(cell => cell.Row == 1));
        Assert.Equal((230d, 230d), (plan.Waypoints[0].Xmm, plan.Waypoints[0].Ymm));
        Assert.Equal((2250d, 150d), (plan.Waypoints[^1].Xmm, plan.Waypoints[^1].Ymm));
        Assert.Equal(4, plan.SegmentDistancesMm.Count);
        Assert.True(plan.Directions.All(direction => direction == CardinalDirection.East));
        Assert.Equal(TurnInstruction.Start, plan.Turns[0]);
        Assert.True(plan.Turns.Skip(1).All(turn => turn == TurnInstruction.Straight));
    }

    [Fact]
    public void Plan_UsesOnlyFourNeighboursAndAvoidsEveryFixedCell()
    {
        var (grid, obstacles) = CreateMap();

        var plan = PathPlanner.Plan(
            grid,
            obstacles,
            MapEndpoint.Cell(1, 1),
            MapEndpoint.Cell(5, 5),
            230,
            230);

        Assert.True(plan.Success, plan.FailureReason);
        for (var index = 1; index < plan.Cells.Count; index++)
        {
            var previous = plan.Cells[index - 1];
            var current = plan.Cells[index];
            Assert.Equal(1, Math.Abs(previous.Row - current.Row) + Math.Abs(previous.Column - current.Column));
        }

        Assert.True(plan.Cells.All(cell => !obstacles.IsBlocked(cell.Row, cell.Column)));
        Assert.True(plan.Cells.All(cell => !new[] { (2, 2), (2, 4), (4, 2), (4, 4) }
            .Contains((cell.Row, cell.Column))));
    }

    [Fact]
    public void Plan_PhysicalDistanceChangesAfterABoundaryDrag()
    {
        var (grid, obstacles) = CreateMap();
        var before = PathPlanner.Plan(grid, obstacles, MapEndpoint.StartZone2, MapEndpoint.GoalZone1, 230, 230);

        Assert.True(grid.TryMoveXLine(0, 650, out var error), error);
        obstacles = ObstacleClassifier.Classify(grid, [], 3, 0);
        var after = PathPlanner.Plan(grid, obstacles, MapEndpoint.StartZone2, MapEndpoint.GoalZone1, 230, 230);

        Assert.True(before.Success);
        Assert.True(after.Success);
        Assert.True(Math.Abs(before.TotalDistanceMm - after.TotalDistanceMm) > 1e-6);
    }

    [Fact]
    public void Plan_RejectsABlockedStartOrGoal()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);
        var startBit = 1u << GridCell.ToBitIndex(1, 1);
        var goalBit = 1u << GridCell.ToBitIndex(1, 5);

        var blockedStart = PathPlanner.Plan(
            grid,
            ObstacleClassifier.Classify(grid, [], 3, startBit),
            MapEndpoint.StartZone2,
            MapEndpoint.GoalZone1,
            230,
            230);
        var blockedGoal = PathPlanner.Plan(
            grid,
            ObstacleClassifier.Classify(grid, [], 3, goalBit),
            MapEndpoint.StartZone2,
            MapEndpoint.GoalZone1,
            230,
            230);

        Assert.False(blockedStart.Success);
        Assert.Contains("start", blockedStart.FailureReason.ToLowerInvariant());
        Assert.False(blockedGoal.Success);
        Assert.Contains("goal", blockedGoal.FailureReason.ToLowerInvariant());
    }

    [Fact]
    public void Plan_ReturnsAnExplicitFailureWhenNoRouteExists()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);
        uint barrierMask = 0;
        for (var row = 1; row <= 5; row++) barrierMask |= 1u << GridCell.ToBitIndex(row, 3);
        var obstacles = ObstacleClassifier.Classify(grid, [], 3, barrierMask);

        var plan = PathPlanner.Plan(
            grid,
            obstacles,
            MapEndpoint.Cell(1, 1),
            MapEndpoint.Cell(1, 5),
            230,
            230);

        Assert.False(plan.Success);
        Assert.Contains("route", plan.FailureReason.ToLowerInvariant());
        Assert.Empty(plan.Cells);
        Assert.Empty(plan.Waypoints);
    }

    [Fact]
    public void Plan_RadarOriginAsGoalUsesExactRadarCoordinates()
    {
        var (grid, obstacles) = CreateMap();

        var plan = PathPlanner.Plan(
            grid,
            obstacles,
            MapEndpoint.Cell(1, 3),
            MapEndpoint.StartZone2,
            310,
            270);

        Assert.True(plan.Success, plan.FailureReason);
        Assert.Equal((310d, 270d), (plan.Waypoints[^1].Xmm, plan.Waypoints[^1].Ymm));
    }

    private static (GridDefinition Grid, ObstacleMap Obstacles) CreateMap()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);
        return (grid, ObstacleClassifier.Classify(grid, [], 3, 0));
    }
}
