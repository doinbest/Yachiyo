namespace Lds50cHost.Services;

public static class RadarSerialSettings
{
    public static IReadOnlyList<int> SupportedBaudRates { get; } =
        new[] { 384000, 500000, 768000, 921600 };

    public const int DefaultBaudRate = 500000;
}
