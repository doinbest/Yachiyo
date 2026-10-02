using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class PathWaypointTableBuilderTests
{
    [Fact]
    public void Build_PreservesEveryOrderedPointAndAlignsItsNextSegment()
    {
        var firstCell = new GridCell(1, 1, 0, 100, 0, 100);
        var secondCell = new GridCell(1, 2, 100, 200, 0, 100);
        var plan = PathPlan.Succeeded(
            [firstCell, secondCell, firstCell],
            [
                new PathWaypoint(10, 20, 1, 1),
                new PathWaypoint(150, 50, 1, 2),
                new PathWaypoint(10, 20, 1, 1)
            ],
            [140, 140],
            [CardinalDirection.East, CardinalDirection.West],
            [TurnInstruction.Start, TurnInstruction.Reverse]);

        var rows = PathWaypointTableBuilder.Build(plan);

        Assert.Equal(3, rows.Count);
        Assert.Equal(
            new PathWaypointTableRow(
                1, "R1C1", 10, 20, 140, CardinalDirection.East, TurnInstruction.Start),
            rows[0]);
        Assert.Equal(
            new PathWaypointTableRow(
                2, "R1C2", 150, 50, 140, CardinalDirection.West, TurnInstruction.Reverse),
            rows[1]);
        Assert.Equal(
            new PathWaypointTableRow(3, "R1C1", 10, 20, null, null, null),
            rows[2]);
    }
}
