using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.Processing;

namespace Lds50cHost.Core.Tests.Processing;

public sealed class PointFilterTests
{
    [Fact]
    public void Apply_IncludesAllAngleDistanceAndEnergyBoundaries()
    {
        var settings = new FilterSettings
        {
            MinAngleDeg = 10,
            MaxAngleDeg = 20,
            MinDistanceMm = 100,
            MaxDistanceMm = 200,
            MinEnergy = 5,
            MaxEnergy = 10
        };
        RadarPoint[] points =
        [
            new(10, 100, 5),
            new(20, 200, 10)
        ];

        var filtered = PointFilter.Apply(points, settings);

        Assert.Equal(points, filtered);
    }

    [Fact]
    public void Apply_TreatsZeroToThreeSixtyAsTheFullCircle()
    {
        var settings = AppConfiguration.CreateDefault().Filter;
        RadarPoint[] points = [new(0, 100, 0), new(180, 1000, 100), new(359.99, 40000, 255)];

        var filtered = PointFilter.Apply(points, settings);

        Assert.Equal(points, filtered);
    }

    [Fact]
    public void Apply_SupportsAnAngleRangeCrossingZero()
    {
        var settings = AppConfiguration.CreateDefault().Filter with
        {
            MinAngleDeg = 350,
            MaxAngleDeg = 10
        };
        RadarPoint[] points = [new(355, 1000, 50), new(5, 1000, 50), new(180, 1000, 50)];

        var filtered = PointFilter.Apply(points, settings);

        Assert.Equal(new[] { points[0], points[1] }, filtered);
    }

    [Fact]
    public void Apply_ExcludesAPointOutsideAnySingleFilter()
    {
        var settings = new FilterSettings
        {
            MinAngleDeg = 10,
            MaxAngleDeg = 20,
            MinDistanceMm = 100,
            MaxDistanceMm = 200,
            MinEnergy = 5,
            MaxEnergy = 10
        };
        RadarPoint[] points =
        [
            new(9, 150, 7),
            new(15, 99, 7),
            new(15, 150, 4),
            new(15, 150, 7)
        ];

        var filtered = PointFilter.Apply(points, settings);

        Assert.Equal(new[] { points[3] }, filtered);
        Assert.Equal(4, points.Length);
    }
}
