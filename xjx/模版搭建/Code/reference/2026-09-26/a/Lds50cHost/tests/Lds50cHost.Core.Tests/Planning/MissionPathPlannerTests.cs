using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Tests.Planning;

public sealed class MissionPathPlannerTests
{
    [Fact]
    public void Plan_DefaultMissionVisitsCheckpointsInOrderAndKeepsRepeatedCells()
    {
        var (configuration, grid, obstacles) = CreateMap();

        var plan = MissionPathPlanner.Plan(
            grid,
            obstacles,
            configuration.MissionRoute,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        Assert.True(plan.Success, plan.FailureReason);
        Assert.True(plan.Cells.Count > 25, $"Expected more than 25 cells, got {plan.Cells.Count}.");
        Assert.Equal((230d, 230d), (plan.Waypoints[0].Xmm, plan.Waypoints[0].Ymm));
        Assert.Equal((230d, 230d), (plan.Waypoints[^1].Xmm, plan.Waypoints[^1].Ymm));
        AssertCheckpointsInOrder(
            plan,
            new (double X, double Y)[]
            {
                (230, 230), (1200, 350), (2050, 1200), (350, 1200), (1200, 2050),
                (2050, 1200), (350, 1200), (1200, 2050), (230, 230)
            });
        Assert.True(
            plan.Cells.GroupBy(cell => cell.BitIndex).Any(group => group.Count() > 1),
            "Expected the mission route to keep later visits to an earlier cell.");
    }

    [Fact]
    public void Plan_OutAndBackMissionProducesReverseTurn()
    {
        var (configuration, grid, obstacles) = CreateMap();
        var route = new MissionRouteSettings { Sequence = new[] { 1, 5, 1 } };

        var plan = MissionPathPlanner.Plan(
            grid,
            obstacles,
            route,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        Assert.True(plan.Success, plan.FailureReason);
        Assert.Contains(plan.Turns, turn => turn == TurnInstruction.Reverse);
    }

    [Fact]
    public void Plan_RemovesOnlyAdjacentLegJoints()
    {
        var (configuration, grid, obstacles) = CreateMap();

        var plan = MissionPathPlanner.Plan(
            grid,
            obstacles,
            configuration.MissionRoute,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        Assert.True(plan.Success, plan.FailureReason);
        for (var index = 1; index < plan.Cells.Count; index++)
            Assert.False(plan.Cells[index - 1].BitIndex == plan.Cells[index].BitIndex);
        Assert.True(plan.Cells.Count(cell => cell.Row == 3 && cell.Column == 5) >= 2);
        Assert.True(plan.Cells.Count(cell => cell.Row == 3 && cell.Column == 1) >= 2);
    }

    [Fact]
    public void Plan_RecomputesAfterRadarAndGridMove()
    {
        var (configuration, grid, _) = CreateMap();
        Assert.True(grid.TryMoveXLine(0, 650, out var xError), xError);
        Assert.True(grid.TryMoveYLine(1, 1100, out var yError), yError);
        var obstacles = ObstacleClassifier.Classify(grid, [], 3, 0);

        var plan = MissionPathPlanner.Plan(grid, obstacles, configuration.MissionRoute, 310, 270);

        Assert.True(plan.Success, plan.FailureReason);
        AssertCheckpointsInOrder(
            plan,
            new (double X, double Y)[]
            {
                (310, 270), (1200, 350), (2050, 1250), (400, 1250), (1200, 2050),
                (2050, 1250), (400, 1250), (1200, 2050), (310, 270)
            });
    }

    [Fact]
    public void Plan_IdentifiesTheUnreachableMissionLeg()
    {
        var configuration = AppConfiguration.CreateDefault();
        var grid = new GridDefinition(configuration.Map);
        uint barrierMask = 0;
        for (var row = 1; row <= 5; row++) barrierMask |= 1u << GridCell.ToBitIndex(row, 2);
        var obstacles = ObstacleClassifier.Classify(grid, [], 3, barrierMask);

        var plan = MissionPathPlanner.Plan(
            grid,
            obstacles,
            configuration.MissionRoute,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        Assert.False(plan.Success);
        Assert.Contains("1→5", plan.FailureReason);
    }

    [Fact]
    public void Plan_DefensivelyProtectsTaskCells()
    {
        var configuration = AppConfiguration.CreateDefault();
        var grid = new GridDefinition(configuration.Map);
        var taskFiveBit = 1u << GridCell.ToBitIndex(1, 3);
        var obstacles = ObstacleClassifier.Classify(grid, [], 3, taskFiveBit);
        var route = new MissionRouteSettings { Sequence = new[] { 1, 5, 1 } };

        var plan = MissionPathPlanner.Plan(
            grid,
            obstacles,
            route,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        Assert.True(plan.Success, plan.FailureReason);
    }

    private static void AssertCheckpointsInOrder(PathPlan plan, params (double X, double Y)[] expected)
    {
        var searchStart = 0;
        foreach (var checkpoint in expected)
        {
            var found = -1;
            for (var index = searchStart; index < plan.Waypoints.Count; index++)
            {
                if (plan.Waypoints[index].Xmm == checkpoint.X && plan.Waypoints[index].Ymm == checkpoint.Y)
                {
                    found = index;
                    break;
                }
            }

            Assert.True(found >= 0, $"Checkpoint ({checkpoint.X},{checkpoint.Y}) was not found after index {searchStart}.");
            searchStart = found + 1;
        }
    }

    private static (AppConfiguration Configuration, GridDefinition Grid, ObstacleMap Obstacles) CreateMap()
    {
        var configuration = AppConfiguration.CreateDefault();
        var grid = new GridDefinition(configuration.Map);
        var obstacles = ObstacleClassifier.Classify(
            grid,
            [],
            configuration.Map.ObstaclePointThreshold,
            configuration.Map.ManualBlockedMask);
        return (configuration, grid, obstacles);
    }
}
