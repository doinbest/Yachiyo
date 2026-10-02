namespace Lds50cHost.Core.Configuration;

public sealed record FilterSettings
{
    public double MinAngleDeg { get; init; }
    public double MaxAngleDeg { get; init; }
    public ushort MinDistanceMm { get; init; }
    public ushort MaxDistanceMm { get; init; }
    public byte MinEnergy { get; init; }
    public byte MaxEnergy { get; init; }

    public bool ContainsAngle(double angleDeg)
    {
        if (MinAngleDeg == 0 && MaxAngleDeg == 360) return true;

        var angle = NormalizeAngle(angleDeg);
        var minimum = NormalizeAngle(MinAngleDeg);
        var maximum = NormalizeAngle(MaxAngleDeg);

        return MinAngleDeg <= MaxAngleDeg
            ? angle >= minimum && angle <= maximum
            : angle >= minimum || angle <= maximum;
    }

    public IReadOnlyList<string> Validate()
    {
        var errors = new List<string>();
        if (MinAngleDeg is < 0 or > 360 || MaxAngleDeg is < 0 or > 360)
            errors.Add("Angle limits must be between 0 and 360 degrees.");
        if (MinDistanceMm > MaxDistanceMm)
            errors.Add("Minimum distance cannot exceed maximum distance.");
        if (MinEnergy > MaxEnergy)
            errors.Add("Minimum energy cannot exceed maximum energy.");
        return errors;
    }

    private static double NormalizeAngle(double angleDeg)
    {
        var normalized = angleDeg % 360d;
        return normalized < 0 ? normalized + 360d : normalized;
    }
}
