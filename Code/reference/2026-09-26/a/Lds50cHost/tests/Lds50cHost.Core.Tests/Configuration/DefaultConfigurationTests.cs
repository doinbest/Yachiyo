using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Tests.Configuration;

public sealed class DefaultConfigurationTests
{
    [Fact]
    public void CreateDefault_UsesTheApprovedPhysicalFieldValues()
    {
        var configuration = AppConfiguration.CreateDefault();

        Assert.Equal(230d, configuration.Map.RadarXmm);
        Assert.Equal(230d, configuration.Map.RadarYmm);
        Assert.Equal(new double[] { 150, 550, 1000, 1400, 1850, 2250 }, configuration.Map.XLines);
        Assert.Equal(new double[] { 150, 550, 1000, 1400, 1850, 2250 }, configuration.Map.YLines);
        Assert.Equal(3, configuration.Map.ObstaclePointThreshold);
        Assert.Equal(328000u, configuration.Map.FixedBlockedMask);
        Assert.Equal(0u, configuration.Map.ManualBlockedMask);

        Assert.Equal(0d, configuration.Filter.MinAngleDeg);
        Assert.Equal(360d, configuration.Filter.MaxAngleDeg);
        Assert.Equal((ushort)100, configuration.Filter.MinDistanceMm);
        Assert.Equal((ushort)40000, configuration.Filter.MaxDistanceMm);
        Assert.Equal((byte)0, configuration.Filter.MinEnergy);
        Assert.Equal((byte)255, configuration.Filter.MaxEnergy);
        Assert.Equal(MapEndpoint.StartZone2, configuration.Start);
        Assert.Equal(MapEndpoint.GoalZone1, configuration.Goal);
    }

    [Fact]
    public void CreateDefault_UsesTheApprovedMissionRoute()
    {
        var configuration = AppConfiguration.CreateDefault();

        Assert.Equal(PlanningMode.MissionRoute, configuration.PlanningMode);
        Assert.Equal(new[] { 1, 5, 4, 2, 3, 4, 2, 3, 1 }, configuration.MissionRoute.Sequence);
    }

    [Fact]
    public void FilterValidation_RejectsReversedDistanceRange()
    {
        var settings = new FilterSettings
        {
            MinAngleDeg = 0,
            MaxAngleDeg = 360,
            MinDistanceMm = 2000,
            MaxDistanceMm = 1000,
            MinEnergy = 0,
            MaxEnergy = 255
        };

        Assert.NotEmpty(settings.Validate());
    }

    [Fact]
    public void ContainsAngle_TreatsZeroToThreeSixtyAsTheFullCircle()
    {
        var settings = AppConfiguration.CreateDefault().Filter;

        Assert.True(settings.ContainsAngle(0));
        Assert.True(settings.ContainsAngle(219.5));
        Assert.True(settings.ContainsAngle(359.999));
    }

    [Fact]
    public void ContainsAngle_SupportsARangeThatCrossesZero()
    {
        var settings = AppConfiguration.CreateDefault().Filter with
        {
            MinAngleDeg = 350,
            MaxAngleDeg = 10
        };

        Assert.True(settings.ContainsAngle(355));
        Assert.True(settings.ContainsAngle(5));
        Assert.False(settings.ContainsAngle(180));
    }

    [Fact]
    public void MapValidation_RejectsAnInternalGapBelowFiftyMillimetres()
    {
        var settings = AppConfiguration.CreateDefault().Map with
        {
            XLines = new double[] { 150, 190, 1000, 1400, 1850, 2250 }
        };

        Assert.NotEmpty(settings.Validate());
    }
}
