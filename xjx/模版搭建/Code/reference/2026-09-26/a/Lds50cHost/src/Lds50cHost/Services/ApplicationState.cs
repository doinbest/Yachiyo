using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.Planning;
using Lds50cHost.Core.Processing;
using Lds50cHost.Core.RadarProtocol;
using Lds50cHost.Core.Scanning;
using Lds50cHost.Core.Stm32;

namespace Lds50cHost.Services;

public enum ApplicationMode
{
    DirectRadar,
    Stm32Configuration
}

public enum MapTheme
{
    Dark,
    Light
}

public enum PointCloudViewMode
{
    ReceivedRaw,
    HostFiltered
}

public sealed class ApplicationState
{
    private readonly object _gate = new();
    private readonly List<string> _logs = [];
    private FieldPoint[] _rawFieldPoints = [];
    private RadarPoint[] _filteredPoints = [];
    private FieldPoint[] _fieldPoints = [];

    public ApplicationState(AppConfiguration configuration)
    {
        ValidateConfiguration(configuration);
        Configuration = configuration;
        Grid = new GridDefinition(configuration.Map);
        ObstacleMap = ObstacleClassifier.Classify(
            Grid,
            [],
            configuration.Map.ObstaclePointThreshold,
            configuration.Map.ManualBlockedMask,
            MissionPointCatalog.ProtectedCellMask);
        NormalizeManualMaskUnsafe();
        Path = PlanConfiguredPathUnsafe();
    }

    public event Action? Changed;

    public AppConfiguration Configuration { get; private set; }
    public GridDefinition Grid { get; private set; }
    public RadarScan? RawScan { get; private set; }
    public IReadOnlyList<FieldPoint> RawFieldPoints => Array.AsReadOnly(_rawFieldPoints);
    public IReadOnlyList<RadarPoint> FilteredPoints => Array.AsReadOnly(_filteredPoints);
    public IReadOnlyList<FieldPoint> FieldPoints => Array.AsReadOnly(_fieldPoints);
    public IReadOnlyList<RadarPoint> DisplayedPoints => ViewMode == PointCloudViewMode.ReceivedRaw
        ? RawScan?.Points ?? Array.Empty<RadarPoint>()
        : FilteredPoints;
    public IReadOnlyList<FieldPoint> DisplayedFieldPoints => ViewMode == PointCloudViewMode.ReceivedRaw
        ? RawFieldPoints
        : FieldPoints;
    public ObstacleMap ObstacleMap { get; private set; }
    public PathPlan Path { get; private set; }
    public ApplicationMode Mode { get; private set; }
    public MapTheme Theme { get; private set; } = MapTheme.Dark;
    public PointCloudViewMode ViewMode { get; private set; } = PointCloudViewMode.HostFiltered;
    public RadarStatusSnapshot? RadarStatus { get; private set; }
    public IReadOnlyList<string> Logs
    {
        get { lock (_gate) return _logs.ToArray(); }
    }
    public long RadarValidFrames { get; private set; }
    public long RadarChecksumErrors { get; private set; }
    public long RadarLengthErrors { get; private set; }
    public long RadarDiscardedBytes { get; private set; }

    public void SetRawScan(RadarScan scan)
    {
        ArgumentNullException.ThrowIfNull(scan);
        lock (_gate)
        {
            RawScan = scan;
            ReprocessUnsafe();
        }
        Changed?.Invoke();
    }

    public void ApplyConfiguration(AppConfiguration configuration)
    {
        ValidateConfiguration(configuration);
        lock (_gate)
        {
            Configuration = configuration;
            Grid = new GridDefinition(configuration.Map);
            ReprocessUnsafe();
        }
        Changed?.Invoke();
    }

    public void UpdateFilter(FilterSettings filter)
    {
        ArgumentNullException.ThrowIfNull(filter);
        var errors = filter.Validate();
        if (errors.Count > 0) throw new ArgumentException(string.Join(" ", errors), nameof(filter));

        lock (_gate)
        {
            Configuration = Configuration with { Filter = filter };
            ReprocessUnsafe();
        }
        Changed?.Invoke();
    }

    public void UpdateMapSettings(MapSettings map)
    {
        ArgumentNullException.ThrowIfNull(map);
        ApplyConfiguration(Configuration with { Map = map });
    }

    public void UpdateEndpoints(MapEndpoint start, MapEndpoint goal) =>
        ApplyConfiguration(Configuration with { Start = start, Goal = goal });

    public void UpdatePlanningMode(PlanningMode mode)
    {
        if (!Enum.IsDefined(mode))
            throw new ArgumentOutOfRangeException(nameof(mode));
        ApplyConfiguration(Configuration with { PlanningMode = mode });
    }

    public void UpdateMissionRoute(MissionRouteSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        ApplyConfiguration(Configuration with { MissionRoute = settings });
    }

    public bool TryMoveBoundary(string axis, int internalIndex, double millimetres, out string error)
    {
        bool moved;
        lock (_gate)
        {
            moved = axis.ToUpperInvariant() switch
            {
                "X" => Grid.TryMoveXLine(internalIndex, millimetres, out error),
                "Y" => Grid.TryMoveYLine(internalIndex, millimetres, out error),
                _ => FailUnknownAxis(out error)
            };
            if (!moved) return false;

            var map = Configuration.Map with
            {
                XLines = Grid.XLines.ToArray(),
                YLines = Grid.YLines.ToArray()
            };
            Configuration = Configuration with { Map = map };
            ReprocessUnsafe();
        }

        Changed?.Invoke();
        return true;
    }

    public bool TryToggleCell(int row, int column, out string error)
    {
        lock (_gate)
        {
            if (ObstacleMap.Get(row, column).IsProtected)
            {
                error = "任务点 2～5 所在格不能设置为障碍。";
                return false;
            }

            if (!ObstacleMap.TryToggleManual(row, column, out var manualMask))
            {
                error = "Fixed forbidden cells cannot be changed.";
                return false;
            }

            Configuration = Configuration with
            {
                Map = Configuration.Map with { ManualBlockedMask = manualMask }
            };
            ReprocessUnsafe();
        }

        error = string.Empty;
        Changed?.Invoke();
        return true;
    }

    public void SetParserCounters(long validFrames, long checksumErrors, long lengthErrors, long discardedBytes)
    {
        lock (_gate)
        {
            RadarValidFrames = validFrames;
            RadarChecksumErrors = checksumErrors;
            RadarLengthErrors = lengthErrors;
            RadarDiscardedBytes = discardedBytes;
        }
        Changed?.Invoke();
    }

    public void SetMode(ApplicationMode mode)
    {
        Mode = mode;
        Changed?.Invoke();
    }

    public void SetTheme(MapTheme theme)
    {
        Theme = theme;
        Changed?.Invoke();
    }

    public void SetPointCloudViewMode(PointCloudViewMode mode)
    {
        lock (_gate) ViewMode = mode;
        Changed?.Invoke();
    }

    public void SetRadarStatus(StatusFrame frame, DateTimeOffset? receivedAt = null)
    {
        ArgumentNullException.ThrowIfNull(frame);
        lock (_gate)
        {
            RadarStatus = RadarStatusSnapshot.From(frame, receivedAt ?? DateTimeOffset.UtcNow);
        }
        Changed?.Invoke();
    }

    public void ClearRadarStatus()
    {
        lock (_gate) RadarStatus = null;
        Changed?.Invoke();
    }

    public void AddLog(string message)
    {
        if (string.IsNullOrWhiteSpace(message)) return;
        lock (_gate)
        {
            _logs.Add($"{DateTimeOffset.Now:HH:mm:ss} {message}");
            if (_logs.Count > 200) _logs.RemoveRange(0, _logs.Count - 200);
        }
        Changed?.Invoke();
    }

    private void ReprocessUnsafe()
    {
        _rawFieldPoints = RawScan is null
            ? []
            : RawScan.Points
                .Select(point => CoordinateTransformer.ToField(point, Configuration.Map.RadarXmm, Configuration.Map.RadarYmm))
                .ToArray();
        _filteredPoints = RawScan is null
            ? []
            : PointFilter.Apply(RawScan.Points, Configuration.Filter).ToArray();
        _fieldPoints = _filteredPoints
            .Select(point => CoordinateTransformer.ToField(point, Configuration.Map.RadarXmm, Configuration.Map.RadarYmm))
            .ToArray();
        ObstacleMap = ObstacleClassifier.Classify(
            Grid,
            _fieldPoints,
            Configuration.Map.ObstaclePointThreshold,
            Configuration.Map.ManualBlockedMask,
            MissionPointCatalog.ProtectedCellMask);
        NormalizeManualMaskUnsafe();
        Path = PlanConfiguredPathUnsafe();
    }

    private static void ValidateConfiguration(AppConfiguration configuration)
    {
        ArgumentNullException.ThrowIfNull(configuration);
        _ = ConfigurationPayloadCodec.Encode(configuration);
        if (!Enum.IsDefined(configuration.PlanningMode))
            throw new ArgumentException("Unknown planning mode.", nameof(configuration));
        if (configuration.MissionRoute is null)
            throw new ArgumentException("Mission route settings are required.", nameof(configuration));
        var missionErrors = configuration.MissionRoute.Validate();
        if (missionErrors.Count > 0)
            throw new ArgumentException(string.Join(" ", missionErrors), nameof(configuration));
    }

    private void NormalizeManualMaskUnsafe()
    {
        if (Configuration.Map.ManualBlockedMask == ObstacleMap.ManualMask) return;
        Configuration = Configuration with
        {
            Map = Configuration.Map with { ManualBlockedMask = ObstacleMap.ManualMask }
        };
    }

    private PathPlan PlanConfiguredPathUnsafe() => Configuration.PlanningMode switch
    {
        PlanningMode.MissionRoute => MissionPathPlanner.Plan(
            Grid,
            ObstacleMap,
            Configuration.MissionRoute,
            Configuration.Map.RadarXmm,
            Configuration.Map.RadarYmm),
        PlanningMode.PointToPoint => PathPlanner.Plan(
            Grid,
            ObstacleMap,
            Configuration.Start,
            Configuration.Goal,
            Configuration.Map.RadarXmm,
            Configuration.Map.RadarYmm),
        _ => PathPlan.Failed("Unknown planning mode.")
    };

    private static bool FailUnknownAxis(out string error)
    {
        error = "Boundary axis must be X or Y.";
        return false;
    }
}
