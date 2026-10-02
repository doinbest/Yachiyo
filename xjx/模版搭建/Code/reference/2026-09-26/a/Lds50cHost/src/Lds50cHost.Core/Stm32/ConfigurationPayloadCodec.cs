using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Stm32;

public static class ConfigurationPayloadCodec
{
    public const byte PayloadVersion = 1;
    private const int VersionOneLength = 47;

    public static byte[] Encode(AppConfiguration configuration)
    {
        ArgumentNullException.ThrowIfNull(configuration);
        var errors = configuration.Filter.Validate().Concat(configuration.Map.Validate()).ToArray();
        if (errors.Length > 0) throw new ArgumentException(string.Join(" ", errors), nameof(configuration));
        ValidateEndpoint(configuration.Start, nameof(configuration.Start));
        ValidateEndpoint(configuration.Goal, nameof(configuration.Goal));

        var bytes = new List<byte>(VersionOneLength) { PayloadVersion };
        AddUInt16(bytes, ToMillimetres(configuration.Map.RadarXmm, "Radar X"));
        AddUInt16(bytes, ToMillimetres(configuration.Map.RadarYmm, "Radar Y"));
        for (var index = 1; index <= 4; index++) AddUInt16(bytes, ToMillimetres(configuration.Map.XLines[index], $"X line {index}"));
        for (var index = 1; index <= 4; index++) AddUInt16(bytes, ToMillimetres(configuration.Map.YLines[index], $"Y line {index}"));
        AddUInt16(bytes, ToAngleTenths(configuration.Filter.MinAngleDeg));
        AddUInt16(bytes, ToAngleTenths(configuration.Filter.MaxAngleDeg));
        AddUInt16(bytes, configuration.Filter.MinDistanceMm);
        AddUInt16(bytes, configuration.Filter.MaxDistanceMm);
        bytes.Add(configuration.Filter.MinEnergy);
        bytes.Add(configuration.Filter.MaxEnergy);
        AddUInt16(bytes, checked((ushort)configuration.Map.ObstaclePointThreshold));
        AddUInt32(bytes, configuration.Map.FixedBlockedMask);
        AddUInt32(bytes, configuration.Map.ManualBlockedMask);
        AddEndpoint(bytes, configuration.Start);
        AddEndpoint(bytes, configuration.Goal);
        return bytes.ToArray();
    }

    public static AppConfiguration Decode(ReadOnlySpan<byte> payload)
    {
        if (payload.Length == 0 || payload[0] != PayloadVersion)
            throw new NotSupportedException($"Unsupported configuration payload version {(payload.Length == 0 ? -1 : payload[0])}.");
        if (payload.Length != VersionOneLength)
            throw new ArgumentException($"Configuration version 1 payload must be {VersionOneLength} bytes.", nameof(payload));

        var offset = 1;
        var radarX = ReadUInt16(payload, ref offset);
        var radarY = ReadUInt16(payload, ref offset);
        var xLines = new double[] { MapSettings.ActiveMinimumMm, 0, 0, 0, 0, MapSettings.ActiveMaximumMm };
        var yLines = new double[] { MapSettings.ActiveMinimumMm, 0, 0, 0, 0, MapSettings.ActiveMaximumMm };
        for (var index = 1; index <= 4; index++) xLines[index] = ReadUInt16(payload, ref offset);
        for (var index = 1; index <= 4; index++) yLines[index] = ReadUInt16(payload, ref offset);

        var filter = new FilterSettings
        {
            MinAngleDeg = ReadUInt16(payload, ref offset) / 10d,
            MaxAngleDeg = ReadUInt16(payload, ref offset) / 10d,
            MinDistanceMm = ReadUInt16(payload, ref offset),
            MaxDistanceMm = ReadUInt16(payload, ref offset),
            MinEnergy = payload[offset++],
            MaxEnergy = payload[offset++]
        };
        var map = new MapSettings
        {
            RadarXmm = radarX,
            RadarYmm = radarY,
            XLines = xLines,
            YLines = yLines,
            ObstaclePointThreshold = ReadUInt16(payload, ref offset),
            FixedBlockedMask = ReadUInt32(payload, ref offset),
            ManualBlockedMask = ReadUInt32(payload, ref offset)
        };
        var start = ReadEndpoint(payload, ref offset);
        var goal = ReadEndpoint(payload, ref offset);
        var configuration = new AppConfiguration(filter, map, start, goal);
        var errors = filter.Validate().Concat(map.Validate()).ToArray();
        if (errors.Length > 0) throw new ArgumentException(string.Join(" ", errors), nameof(payload));
        return configuration;
    }

    private static void AddEndpoint(List<byte> bytes, MapEndpoint endpoint)
    {
        bytes.Add(checked((byte)endpoint.Row));
        bytes.Add(checked((byte)endpoint.Column));
        bytes.Add((byte)endpoint.Anchor);
    }

    private static MapEndpoint ReadEndpoint(ReadOnlySpan<byte> payload, ref int offset)
    {
        var row = payload[offset++];
        var column = payload[offset++];
        var anchor = (EndpointAnchor)payload[offset++];
        var endpoint = new MapEndpoint(row, column, anchor);
        ValidateEndpoint(endpoint, nameof(payload));
        return endpoint;
    }

    private static void ValidateEndpoint(MapEndpoint endpoint, string parameterName)
    {
        if (endpoint.Row is < 1 or > 5 || endpoint.Column is < 1 or > 5)
            throw new ArgumentException("Endpoint row and column must be between 1 and 5.", parameterName);
        if (!Enum.IsDefined(endpoint.Anchor))
            throw new ArgumentException("Endpoint anchor is not recognized.", parameterName);
    }

    private static ushort ToMillimetres(double value, string name)
    {
        if (!double.IsFinite(value) || value < 0 || value > ushort.MaxValue)
            throw new ArgumentOutOfRangeException(name);
        return checked((ushort)Math.Round(value, MidpointRounding.AwayFromZero));
    }

    private static ushort ToAngleTenths(double value) => ToMillimetres(value * 10d, "Angle");

    private static void AddUInt16(List<byte> bytes, ushort value)
    {
        bytes.Add((byte)value);
        bytes.Add((byte)(value >> 8));
    }

    private static void AddUInt32(List<byte> bytes, uint value)
    {
        AddUInt16(bytes, (ushort)value);
        AddUInt16(bytes, (ushort)(value >> 16));
    }

    private static ushort ReadUInt16(ReadOnlySpan<byte> payload, ref int offset)
    {
        var value = (ushort)(payload[offset] | (payload[offset + 1] << 8));
        offset += 2;
        return value;
    }

    private static uint ReadUInt32(ReadOnlySpan<byte> payload, ref int offset)
    {
        var low = ReadUInt16(payload, ref offset);
        var high = ReadUInt16(payload, ref offset);
        return low | ((uint)high << 16);
    }
}
