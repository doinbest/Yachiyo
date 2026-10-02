using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Configuration;

public sealed record AppConfiguration(
    FilterSettings Filter,
    MapSettings Map,
    MapEndpoint Start,
    MapEndpoint Goal)
{
    public PlanningMode PlanningMode { get; init; } = PlanningMode.MissionRoute;
    public MissionRouteSettings MissionRoute { get; init; } = MissionRouteSettings.CreateDefault();

    public static AppConfiguration CreateDefault()
    {
        var lines = new double[] { 150, 550, 1000, 1400, 1850, 2250 };
        return new AppConfiguration(
            new FilterSettings
            {
                MinAngleDeg = 0,
                MaxAngleDeg = 360,
                MinDistanceMm = 100,
                MaxDistanceMm = 40000,
                MinEnergy = 0,
                MaxEnergy = 255
            },
            new MapSettings
            {
                RadarXmm = 230,
                RadarYmm = 230,
                XLines = lines,
                YLines = lines,
                ObstaclePointThreshold = 3,
                FixedBlockedMask = 328000u,
                ManualBlockedMask = 0
            },
            MapEndpoint.StartZone2,
            MapEndpoint.GoalZone1);
    }
}
