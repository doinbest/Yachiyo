namespace Lds50cHost.Core.Configuration;

public sealed record MapSettings
{
    public const double FieldSizeMm = 2400;
    public const double ActiveMinimumMm = 150;
    public const double ActiveMaximumMm = 2250;
    public const double MinimumCellSizeMm = 50;

    private double[] _xLines = [];
    private double[] _yLines = [];

    public double RadarXmm { get; init; }
    public double RadarYmm { get; init; }
    public IReadOnlyList<double> XLines
    {
        get => Array.AsReadOnly(_xLines);
        init => _xLines = value.ToArray();
    }

    public IReadOnlyList<double> YLines
    {
        get => Array.AsReadOnly(_yLines);
        init => _yLines = value.ToArray();
    }

    public int ObstaclePointThreshold { get; init; }
    public uint FixedBlockedMask { get; init; }
    public uint ManualBlockedMask { get; init; }

    public IReadOnlyList<string> Validate()
    {
        var errors = new List<string>();
        if (RadarXmm is < 0 or > FieldSizeMm || RadarYmm is < 0 or > FieldSizeMm)
            errors.Add("Radar coordinates must lie inside the field.");
        ValidateLines(XLines, "X", errors);
        ValidateLines(YLines, "Y", errors);
        if (ObstaclePointThreshold < 1)
            errors.Add("Obstacle point threshold must be at least one.");
        return errors;
    }

    private static void ValidateLines(IReadOnlyList<double> lines, string axis, List<string> errors)
    {
        if (lines.Count != 6)
        {
            errors.Add($"{axis} grid must contain exactly six boundary lines.");
            return;
        }

        if (lines[0] != ActiveMinimumMm || lines[^1] != ActiveMaximumMm)
            errors.Add($"{axis} outer boundaries must remain at 150 and 2250 mm.");

        for (var index = 1; index < lines.Count; index++)
        {
            if (lines[index] - lines[index - 1] < MinimumCellSizeMm)
            {
                errors.Add($"{axis} cell widths must be at least 50 mm.");
                break;
            }
        }
    }
}
