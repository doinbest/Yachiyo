using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;

namespace Lds50cHost.Core.Tests.Mapping;

public sealed class GridDefinitionTests
{
    [Fact]
    public void Constructor_UsesTheApprovedInitialWidthsAndHeights()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);

        Assert.Equal(new[] { 400d, 450d, 400d, 450d, 400d }, CellSizes(grid.XLines));
        Assert.Equal(new[] { 400d, 450d, 400d, 450d, 400d }, CellSizes(grid.YLines));
    }

    [Fact]
    public void Locate_NumbersRowsBottomUpAndInternalBoundariesTowardPositiveAxes()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);

        var bottomLeft = Assert.NotNull(grid.Locate(200, 200));
        var onFirstInternalLines = Assert.NotNull(grid.Locate(550, 550));

        Assert.Equal(1, bottomLeft.Row);
        Assert.Equal(1, bottomLeft.Column);
        Assert.Equal(2, onFirstInternalLines.Row);
        Assert.Equal(2, onFirstInternalLines.Column);
    }

    [Fact]
    public void Locate_IncludesBothOuterBoundaries()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);

        var minimum = Assert.NotNull(grid.Locate(150, 150));
        var maximum = Assert.NotNull(grid.Locate(2250, 2250));

        Assert.Equal((1, 1), (minimum.Row, minimum.Column));
        Assert.Equal((5, 5), (maximum.Row, maximum.Column));
    }

    [Fact]
    public void Locate_RejectsPointsOutsideTheActiveArea()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);

        Assert.Null(grid.Locate(149.999, 500));
        Assert.Null(grid.Locate(500, 2250.001));
    }

    [Fact]
    public void TryMoveXLine_MovesOnlyTheSelectedAxisAndBoundary()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);
        var oldY = grid.YLines.ToArray();

        var moved = grid.TryMoveXLine(0, 600, out var error);

        Assert.True(moved, error);
        Assert.Equal(new[] { 150d, 600d, 1000d, 1400d, 1850d, 2250d }, grid.XLines);
        Assert.Equal(oldY, grid.YLines);
    }

    [Fact]
    public void TryMoveLine_RejectsCrossingOrGapsBelowFiftyAndPreservesOldLines()
    {
        var grid = new GridDefinition(AppConfiguration.CreateDefault().Map);
        var oldX = grid.XLines.ToArray();
        var oldY = grid.YLines.ToArray();

        var narrow = grid.TryMoveXLine(0, 960, out var narrowError);
        var crossing = grid.TryMoveYLine(1, 500, out var crossingError);

        Assert.False(narrow);
        Assert.False(crossing);
        Assert.True(!string.IsNullOrWhiteSpace(narrowError));
        Assert.True(!string.IsNullOrWhiteSpace(crossingError));
        Assert.Equal(oldX, grid.XLines);
        Assert.Equal(oldY, grid.YLines);
    }

    private static double[] CellSizes(IReadOnlyList<double> lines) =>
        Enumerable.Range(0, lines.Count - 1).Select(index => lines[index + 1] - lines[index]).ToArray();
}
