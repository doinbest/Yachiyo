using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Tests.Mapping;

public sealed class ObstacleClassifierTests
{
    [Fact]
    public void Classify_AlwaysMarksTheFourApprovedFixedCells()
    {
        var grid = CreateGrid();

        var map = ObstacleClassifier.Classify(grid, [], 3, 0);

        Assert.True(map.Get(2, 2).Flags.HasFlag(ObstacleFlags.Fixed));
        Assert.True(map.Get(2, 4).Flags.HasFlag(ObstacleFlags.Fixed));
        Assert.True(map.Get(4, 2).Flags.HasFlag(ObstacleFlags.Fixed));
        Assert.True(map.Get(4, 4).Flags.HasFlag(ObstacleFlags.Fixed));
        Assert.False(map.Get(1, 1).Flags.HasFlag(ObstacleFlags.Fixed));
    }

    [Fact]
    public void TryToggleManual_TogglesANormalCellThroughTheManualMask()
    {
        var grid = CreateGrid();
        var initial = ObstacleClassifier.Classify(grid, [], 3, 0);

        var toggled = initial.TryToggleManual(1, 1, out var manualMask);
        var updated = ObstacleClassifier.Classify(grid, [], 3, manualMask);

        Assert.True(toggled);
        Assert.True(updated.Get(1, 1).Flags.HasFlag(ObstacleFlags.Manual));
    }

    [Fact]
    public void TryToggleManual_CannotClearOrAlterAFixedCell()
    {
        var grid = CreateGrid();
        var initialMask = 1u << GridCell.ToBitIndex(2, 2);
        var map = ObstacleClassifier.Classify(grid, [], 3, initialMask);

        var toggled = map.TryToggleManual(2, 2, out var resultingMask);

        Assert.False(toggled);
        Assert.Equal(initialMask, resultingMask);
        Assert.True(map.IsBlocked(2, 2));
    }

    [Fact]
    public void Classify_UsesTheConfiguredPointThreshold()
    {
        var grid = CreateGrid();
        FieldPoint[] points = [Point(200, 200), Point(210, 210), Point(220, 220)];

        var twoPoints = ObstacleClassifier.Classify(grid, points.Take(2), 3, 0);
        var threePoints = ObstacleClassifier.Classify(grid, points, 3, 0);

        Assert.False(twoPoints.Get(1, 1).Flags.HasFlag(ObstacleFlags.Scanned));
        Assert.Equal(2, twoPoints.Get(1, 1).ScannedPointCount);
        Assert.True(threePoints.Get(1, 1).Flags.HasFlag(ObstacleFlags.Scanned));
        Assert.Equal(3, threePoints.Get(1, 1).ScannedPointCount);
    }

    [Fact]
    public void Classify_RecountsIndependentlyAfterABoundaryMoves()
    {
        var grid = CreateGrid();
        FieldPoint[] points = [Point(540, 200), Point(560, 200)];
        var before = ObstacleClassifier.Classify(grid, points, 3, 0);

        Assert.True(grid.TryMoveXLine(0, 600, out var error), error);
        var after = ObstacleClassifier.Classify(grid, points, 3, 0);

        Assert.Equal(1, before.Get(1, 1).ScannedPointCount);
        Assert.Equal(1, before.Get(1, 2).ScannedPointCount);
        Assert.Equal(2, after.Get(1, 1).ScannedPointCount);
        Assert.Equal(0, after.Get(1, 2).ScannedPointCount);
    }

    [Fact]
    public void Classify_ProtectedTaskCellKeepsCountButCannotBecomeBlockedOrToggled()
    {
        var grid = CreateGrid();
        var protectedBit = 1u << GridCell.ToBitIndex(1, 3);
        var ordinaryBit = 1u << GridCell.ToBitIndex(1, 2);
        FieldPoint[] points =
        [
            Point(1200, 350), Point(1210, 350), Point(1220, 350),
            Point(775, 350), Point(785, 350), Point(795, 350)
        ];

        var map = ObstacleClassifier.Classify(
            grid,
            points,
            3,
            protectedBit | ordinaryBit,
            MissionPointCatalog.ProtectedCellMask);

        var protectedCell = map.Get(1, 3);
        Assert.Equal(3, protectedCell.ScannedPointCount);
        Assert.True(protectedCell.IsProtected);
        Assert.False(protectedCell.IsBlocked);
        Assert.False(protectedCell.Flags.HasFlag(ObstacleFlags.Manual));
        Assert.False(protectedCell.Flags.HasFlag(ObstacleFlags.Scanned));
        Assert.False(map.TryToggleManual(1, 3, out var resultingMask));
        Assert.Equal(ordinaryBit, resultingMask);

        var ordinaryCell = map.Get(1, 2);
        Assert.False(ordinaryCell.IsProtected);
        Assert.True(ordinaryCell.IsBlocked);
        Assert.True(ordinaryCell.Flags.HasFlag(ObstacleFlags.Manual));
        Assert.True(ordinaryCell.Flags.HasFlag(ObstacleFlags.Scanned));
        Assert.True(map.Get(2, 2).IsBlocked);
        Assert.True(map.Get(2, 4).IsBlocked);
        Assert.True(map.Get(4, 2).IsBlocked);
        Assert.True(map.Get(4, 4).IsBlocked);
    }

    private static GridDefinition CreateGrid() => new(AppConfiguration.CreateDefault().Map);

    private static FieldPoint Point(double x, double y) =>
        new(x, y, new RadarPoint(0, 100, 1));
}
