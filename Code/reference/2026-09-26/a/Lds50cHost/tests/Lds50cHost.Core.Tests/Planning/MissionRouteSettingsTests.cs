using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Tests.Planning;

public sealed class MissionRouteSettingsTests
{
    [Fact]
    public void TryParse_AcceptsSupportedSeparatorsAndNormalizes()
    {
        var parsed = MissionRouteTextCodec.TryParse("1 - 5，4, 2 3 - 1", out var settings, out var error);

        Assert.True(parsed, error);
        Assert.Equal(new[] { 1, 5, 4, 2, 3, 1 }, settings.Sequence);
        Assert.Equal("1-5-4-2-3-1", MissionRouteTextCodec.Format(settings));
    }

    [Fact]
    public void TryParse_RejectsEveryInvalidSequenceClass()
    {
        var invalidValues = new[]
        {
            "1-a-1",
            "1-0-1",
            "1-6-1",
            "1-5-5-1",
            "2-3-1",
            "1-3-2",
            "1",
            "1-2-3-4-5-2-3-4-5-2-3-4-5-2-3-4-1"
        };

        foreach (var value in invalidValues)
        {
            var parsed = MissionRouteTextCodec.TryParse(value, out _, out var error);

            Assert.False(parsed, $"Expected '{value}' to be rejected.");
            Assert.True(!string.IsNullOrWhiteSpace(error), $"Expected '{value}' to report a Chinese validation error.");
        }
    }

    [Fact]
    public void ResolveAll_UsesApprovedDefaultCoordinates()
    {
        var configuration = AppConfiguration.CreateDefault();
        var grid = new GridDefinition(configuration.Map);

        var points = MissionPointCatalog.ResolveAll(
            grid,
            configuration.Map.RadarXmm,
            configuration.Map.RadarYmm);

        Assert.Equal(new[] { 1, 2, 3, 4, 5 }, points.Select(point => point.Number));
        Assert.Equal(
            new[] { (230d, 230d), (350d, 1200d), (1200d, 2050d), (2050d, 1200d), (1200d, 350d) },
            points.Select(point => (point.Xmm, point.Ymm)));
    }

    [Fact]
    public void ResolveAll_FollowsRadarAndMovedGrid()
    {
        var configuration = AppConfiguration.CreateDefault();
        var grid = new GridDefinition(configuration.Map);
        Assert.True(grid.TryMoveXLine(0, 650, out var xError), xError);
        Assert.True(grid.TryMoveYLine(1, 1100, out var yError), yError);

        var points = MissionPointCatalog.ResolveAll(grid, 310, 270);

        Assert.Equal(
            new[] { (310d, 270d), (400d, 1250d), (1200d, 2050d), (2050d, 1250d), (1200d, 350d) },
            points.Select(point => (point.Xmm, point.Ymm)));
    }

    [Fact]
    public void ProtectedCellMask_ContainsExactlyPointsTwoThroughFive()
    {
        var expected =
            1u << GridCell.ToBitIndex(3, 1) |
            1u << GridCell.ToBitIndex(5, 3) |
            1u << GridCell.ToBitIndex(3, 5) |
            1u << GridCell.ToBitIndex(1, 3);

        Assert.Equal(expected, MissionPointCatalog.ProtectedCellMask);
    }
}
