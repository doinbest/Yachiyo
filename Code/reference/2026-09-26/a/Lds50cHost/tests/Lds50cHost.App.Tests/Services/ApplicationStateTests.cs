using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.Planning;
using Lds50cHost.Core.RadarProtocol;
using Lds50cHost.Core.Scanning;
using Lds50cHost.Core.Stm32;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class ApplicationStateTests
{
    [Fact]
    public void Constructor_DefaultsToTheEditableMissionRoute()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());

        Assert.Equal(PlanningMode.MissionRoute, state.Configuration.PlanningMode);
        Assert.Equal(new[] { 1, 5, 4, 2, 3, 4, 2, 3, 1 }, state.Configuration.MissionRoute.Sequence);
        Assert.True(state.Path.Success, state.Path.FailureReason);
        Assert.True(state.Path.Waypoints.Count > 25);
        Assert.Equal(new PathWaypoint(230, 230, 1, 1), state.Path.Waypoints[0]);
        Assert.Equal(new PathWaypoint(230, 230, 1, 1), state.Path.Waypoints[^1]);
    }

    [Fact]
    public void UpdatePlanningMode_PreservesEndpointsAndSwitchesTheActivePlanner()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var missionCount = state.Path.Waypoints.Count;
        var start = state.Configuration.Start;
        var goal = state.Configuration.Goal;

        state.UpdatePlanningMode(PlanningMode.PointToPoint);

        Assert.Equal(start, state.Configuration.Start);
        Assert.Equal(goal, state.Configuration.Goal);
        Assert.True(state.Path.Success, state.Path.FailureReason);
        Assert.True(state.Path.Waypoints.Count < missionCount);

        state.UpdatePlanningMode(PlanningMode.MissionRoute);

        Assert.Equal(missionCount, state.Path.Waypoints.Count);
    }

    [Fact]
    public void UpdateMissionRoute_AcceptsAValidBacktrackingRouteAndReplans()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());

        state.UpdateMissionRoute(new MissionRouteSettings { Sequence = new[] { 1, 5, 1 } });

        Assert.Equal(new[] { 1, 5, 1 }, state.Configuration.MissionRoute.Sequence);
        Assert.True(state.Path.Success, state.Path.FailureReason);
        Assert.Equal(new PathWaypoint(230, 230, 1, 1), state.Path.Waypoints[0]);
        Assert.Equal(new PathWaypoint(230, 230, 1, 1), state.Path.Waypoints[^1]);
        Assert.True(state.Path.Turns.Contains(TurnInstruction.Reverse));
    }

    [Fact]
    public void GridAndRadarChanges_MoveMissionPointCoordinates()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        Assert.True(state.TryMoveBoundary("X", 0, 650, out var xError), xError);
        Assert.True(state.TryMoveBoundary("Y", 1, 1100, out var yError), yError);

        state.UpdateMapSettings(state.Configuration.Map with { RadarXmm = 310, RadarYmm = 270 });

        var points = MissionPointCatalog.ResolveAll(
            state.Grid,
            state.Configuration.Map.RadarXmm,
            state.Configuration.Map.RadarYmm);
        Assert.Equal((310d, 270d), (points[0].Xmm, points[0].Ymm));
        Assert.Equal((400d, 1250d), (points[1].Xmm, points[1].Ymm));
        Assert.Equal(new PathWaypoint(310, 270, 1, 1), state.Path.Waypoints[0]);
        Assert.Equal(new PathWaypoint(310, 270, 1, 1), state.Path.Waypoints[^1]);
    }

    [Fact]
    public void ScanPointsInsideMissionCells_AreCountedButNeverBlockTheRoute()
    {
        var defaults = AppConfiguration.CreateDefault();
        var state = new ApplicationState(defaults with
        {
            Map = defaults.Map with { ObstaclePointThreshold = 3 }
        });
        var point = PointAt(350, 1200, defaults.Map.RadarXmm, defaults.Map.RadarYmm);

        state.SetRawScan(new RadarScan([point, point, point], DateTimeOffset.UtcNow));

        var taskCell = state.ObstacleMap.Get(3, 1);
        Assert.Equal(3, taskCell.ScannedPointCount);
        Assert.True(taskCell.IsProtected);
        Assert.False(taskCell.Flags.HasFlag(ObstacleFlags.Scanned));
        Assert.False(taskCell.IsBlocked);
        Assert.True(state.Path.Success, state.Path.FailureReason);
    }

    [Fact]
    public void TryToggleCell_RejectsMissionCellsButAllowsOrdinaryCells()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());

        var taskCellToggled = state.TryToggleCell(3, 1, out var taskError);
        var ordinaryCellToggled = state.TryToggleCell(1, 2, out var ordinaryError);

        Assert.False(taskCellToggled);
        Assert.Contains("任务点", taskError);
        Assert.True(ordinaryCellToggled, ordinaryError);
    }

    [Fact]
    public void ManualBarrier_ReportsTheMissionLegThatCannotBePlanned()
    {
        var defaults = AppConfiguration.CreateDefault();
        var barrier = Enumerable.Range(1, 5)
            .Aggregate(0u, (mask, row) => mask | (1u << GridCell.ToBitIndex(row, 2)));
        var state = new ApplicationState(defaults with
        {
            Map = defaults.Map with { ManualBlockedMask = barrier }
        });

        Assert.False(state.Path.Success);
        Assert.Contains("1→5", state.Path.FailureReason);
    }

    [Fact]
    public void Constructor_DefaultsToHostFilteredPointCloudView()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());

        Assert.Equal(PointCloudViewMode.HostFiltered, state.ViewMode);
    }

    [Fact]
    public void SetRadarStatus_StoresAllFlagsAndTimestamp()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var receivedAt = new DateTimeOffset(2026, 9, 28, 8, 30, 0, TimeSpan.Zero);

        state.SetRadarStatus(new StatusFrame(0b1011, [0, 0, 0]), receivedAt);

        var status = Assert.NotNull(state.RadarStatus);
        Assert.True(status.UsesMillimetres);
        Assert.True(status.EnergyEnabled);
        Assert.False(status.TrailingPointRemovalEnabled);
        Assert.True(status.FilterEnabled);
        Assert.Equal(receivedAt, status.ReceivedAt);
    }

    [Fact]
    public void ClearRadarStatus_RemovesAStaleReading()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        state.SetRadarStatus(new StatusFrame(0b1111, [0, 0, 0]));

        state.ClearRadarStatus();

        Assert.Null(state.RadarStatus);
    }

    [Fact]
    public void SetPointCloudViewMode_ChangesDisplayedPointsButNotPlanningInputs()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        state.UpdateFilter(state.Configuration.Filter with { MinEnergy = 10 });
        state.SetRawScan(new RadarScan(
            [new RadarPoint(0, 300, 5), new RadarPoint(0, 400, 50)],
            DateTimeOffset.UtcNow));
        var obstacleBefore = state.ObstacleMap.Cells
            .Select(cell => (cell.Cell.BitIndex, cell.Flags, cell.ScannedPointCount))
            .ToArray();
        var pathCellsBefore = state.Path.Cells.ToArray();
        var pathWaypointsBefore = state.Path.Waypoints.ToArray();

        Assert.Equal(1, state.DisplayedPoints.Count);
        Assert.Equal(1, state.DisplayedFieldPoints.Count);
        state.SetPointCloudViewMode(PointCloudViewMode.ReceivedRaw);

        Assert.Equal(2, state.DisplayedPoints.Count);
        Assert.Equal(2, state.DisplayedFieldPoints.Count);
        Assert.Equal(
            obstacleBefore,
            state.ObstacleMap.Cells
                .Select(cell => (cell.Cell.BitIndex, cell.Flags, cell.ScannedPointCount))
                .ToArray());
        Assert.Equal(pathCellsBefore, state.Path.Cells);
        Assert.Equal(pathWaypointsBefore, state.Path.Waypoints);
    }

    [Fact]
    public void UpdateMapSettings_ReprojectsReceivedAndFilteredPoints()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        state.UpdateFilter(state.Configuration.Filter with { MinEnergy = 10 });
        state.SetRawScan(new RadarScan(
            [new RadarPoint(0, 100, 5), new RadarPoint(0, 200, 50)],
            DateTimeOffset.UtcNow));

        state.UpdateMapSettings(state.Configuration.Map with
        {
            RadarXmm = 330,
            RadarYmm = 430
        });

        Assert.Equal(2, state.RawFieldPoints.Count);
        Assert.Equal(new FieldPoint(330, 530, new RadarPoint(0, 100, 5)), state.RawFieldPoints[0]);
        Assert.Equal(new FieldPoint(330, 630, new RadarPoint(0, 200, 50)), state.RawFieldPoints[1]);
        Assert.Equal(1, state.FieldPoints.Count);
        Assert.Equal(new FieldPoint(330, 630, new RadarPoint(0, 200, 50)), state.FieldPoints[0]);
    }

    [Fact]
    public void UpdateFilter_ReprocessesTheFrozenScanWithoutReplacingIt()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var scan = new RadarScan(
            [new RadarPoint(0, 500, 5), new RadarPoint(0, 500, 50)],
            DateTimeOffset.UtcNow);
        state.SetRawScan(scan);

        state.UpdateFilter(state.Configuration.Filter with { MinEnergy = 10 });

        Assert.True(ReferenceEquals(scan, state.RawScan));
        Assert.Equal(1, state.FilteredPoints.Count);
        Assert.Equal((byte)50, state.FilteredPoints[0].Energy);
    }

    [Fact]
    public void TryMoveBoundary_ValidMoveReclassifiesPointsAndReplans()
    {
        var defaults = AppConfiguration.CreateDefault();
        var state = new ApplicationState(defaults with
        {
            Map = defaults.Map with { ObstaclePointThreshold = 2 }
        });
        state.SetRawScan(new RadarScan(
            [new RadarPoint(90, 310, 10), new RadarPoint(90, 330, 10)],
            DateTimeOffset.UtcNow));
        Assert.True(state.Path.Success);

        var moved = state.TryMoveBoundary("X", 0, 600, out var error);

        Assert.True(moved, error);
        Assert.Equal(2, state.ObstacleMap.Get(1, 1).ScannedPointCount);
        Assert.True(state.ObstacleMap.Get(1, 1).Flags.HasFlag(ObstacleFlags.Scanned));
        Assert.False(state.Path.Success);
        Assert.Equal(600d, state.Configuration.Map.XLines[1]);
    }

    [Fact]
    public void TryMoveBoundary_InvalidMoveLeavesStateUnchanged()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var before = ConfigurationPayloadCodec.Encode(state.Configuration);

        var moved = state.TryMoveBoundary("X", 0, 980, out var error);

        Assert.False(moved);
        Assert.True(!string.IsNullOrWhiteSpace(error));
        Assert.Equal(before, ConfigurationPayloadCodec.Encode(state.Configuration));
    }

    [Fact]
    public void TryToggleCell_FixedCellDoesNothing()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var before = state.Configuration.Map.ManualBlockedMask;

        var toggled = state.TryToggleCell(2, 2, out var error);

        Assert.False(toggled);
        Assert.True(!string.IsNullOrWhiteSpace(error));
        Assert.Equal(before, state.Configuration.Map.ManualBlockedMask);
        Assert.True(state.ObstacleMap.Get(2, 2).Flags.HasFlag(ObstacleFlags.Fixed));
    }

    [Fact]
    public void TryToggleCell_NormalCellTogglesManualBlocking()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());

        var toggled = state.TryToggleCell(1, 2, out var error);

        Assert.True(toggled, error);
        Assert.True(state.ObstacleMap.Get(1, 2).Flags.HasFlag(ObstacleFlags.Manual));
        Assert.True((state.Configuration.Map.ManualBlockedMask & (1u << GridCell.ToBitIndex(1, 2))) != 0);
    }

    [Fact]
    public void SetTheme_DoesNotChangeSerializedStm32Configuration()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var before = ConfigurationPayloadCodec.Encode(state.Configuration);

        state.SetTheme(MapTheme.Light);

        Assert.Equal(MapTheme.Light, state.Theme);
        Assert.Equal(before, ConfigurationPayloadCodec.Encode(state.Configuration));
    }

    private static RadarPoint PointAt(double x, double y, double radarX, double radarY)
    {
        var dx = x - radarX;
        var dy = y - radarY;
        var angle = Math.Atan2(dx, dy) * 180d / Math.PI;
        if (angle < 0) angle += 360d;
        return new RadarPoint(angle, checked((ushort)Math.Round(Math.Sqrt(dx * dx + dy * dy))), 50);
    }
}
