using Lds50cHost.Core.Planning;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class MapEndpointSelectionTests
{
    [Fact]
    public void SelectionValues_PreserveBothPhysicalStopZones()
    {
        Assert.Equal("start-zone-2", MapEndpointSelection.ToValue(MapEndpoint.StartZone2));
        Assert.Equal("goal-zone-1", MapEndpointSelection.ToValue(MapEndpoint.GoalZone1));
        Assert.True(MapEndpointSelection.TryParse("start-zone-2", true, out var start));
        Assert.True(MapEndpointSelection.TryParse("goal-zone-1", false, out var goal));
        Assert.Equal(MapEndpoint.StartZone2, start);
        Assert.Equal(MapEndpoint.GoalZone1, goal);
    }

    [Fact]
    public void SelectionValues_RoundTripAnOrdinaryCell()
    {
        Assert.True(MapEndpointSelection.TryParse("cell-12", true, out var endpoint));
        Assert.Equal(MapEndpoint.Cell(3, 3), endpoint);
        Assert.Equal("cell-12", MapEndpointSelection.ToValue(endpoint));
    }

    [Fact]
    public void SelectionValues_RejectAStopZoneInTheWrongSelector()
    {
        Assert.False(MapEndpointSelection.TryParse("goal-zone-1", true, out _));
        Assert.False(MapEndpointSelection.TryParse("start-zone-2", false, out _));
    }
}
