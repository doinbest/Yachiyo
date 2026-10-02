using Lds50cHost.Core.Planning;

namespace Lds50cHost.Core.Stm32;

public readonly record struct PathEntry(byte Row, byte Column, ushort Xmm, ushort Ymm);
public readonly record struct PathSegment(ushort DistanceMm, CardinalDirection Direction, TurnInstruction Turn);

public sealed class PathPayload
{
    private readonly PathEntry[] _entries;
    private readonly PathSegment[] _segments;

    public PathPayload(byte version, IEnumerable<PathEntry> entries, IEnumerable<PathSegment> segments)
    {
        Version = version;
        _entries = entries.ToArray();
        _segments = segments.ToArray();
    }

    public byte Version { get; }
    public IReadOnlyList<PathEntry> Entries => Array.AsReadOnly(_entries);
    public IReadOnlyList<PathSegment> Segments => Array.AsReadOnly(_segments);
}

public static class PathPayloadCodec
{
    public const byte PayloadVersion = 1;
    public const int MaximumEntryCount = 409;

    public static byte[] Encode(PathPlan plan)
    {
        ArgumentNullException.ThrowIfNull(plan);
        if (!plan.Success) throw new ArgumentException("Only a successful path can be encoded.", nameof(plan));
        if (plan.Cells.Count != plan.Waypoints.Count)
            throw new ArgumentException("Each path cell must have one waypoint.", nameof(plan));
        if (plan.SegmentDistancesMm.Count != plan.Directions.Count || plan.Directions.Count != plan.Turns.Count)
            throw new ArgumentException("Each route segment must include distance, direction, and turn data.", nameof(plan));

        var entries = plan.Cells.Select((cell, index) => new PathEntry(
            checked((byte)cell.Row),
            checked((byte)cell.Column),
            ToUInt16(plan.Waypoints[index].Xmm, "Waypoint X"),
            ToUInt16(plan.Waypoints[index].Ymm, "Waypoint Y")));
        var segments = plan.SegmentDistancesMm.Select((distance, index) => new PathSegment(
            ToUInt16(distance, "Segment distance"),
            plan.Directions[index],
            plan.Turns[index]));
        return Encode(new PathPayload(PayloadVersion, entries, segments));
    }

    public static byte[] Encode(PathPayload payload)
    {
        ArgumentNullException.ThrowIfNull(payload);
        if (payload.Version != PayloadVersion)
            throw new NotSupportedException($"Unsupported path payload version {payload.Version}.");
        if (payload.Entries.Count > MaximumEntryCount)
            throw new ArgumentException($"A path cannot contain more than {MaximumEntryCount} entries.", nameof(payload));
        if (payload.Segments.Count != Math.Max(0, payload.Entries.Count - 1))
            throw new ArgumentException("Path segment count must be one less than entry count.", nameof(payload));

        var bytes = new List<byte> { PayloadVersion };
        AddUInt16(bytes, checked((ushort)payload.Entries.Count));
        foreach (var entry in payload.Entries)
        {
            if (entry.Row is < 1 or > 5 || entry.Column is < 1 or > 5)
                throw new ArgumentException("Path cells must be inside the 5 x 5 grid.", nameof(payload));
            bytes.Add(entry.Row);
            bytes.Add(entry.Column);
            AddUInt16(bytes, entry.Xmm);
            AddUInt16(bytes, entry.Ymm);
        }

        AddUInt16(bytes, checked((ushort)payload.Segments.Count));
        foreach (var segment in payload.Segments)
        {
            if (!Enum.IsDefined(segment.Direction) || !Enum.IsDefined(segment.Turn))
                throw new ArgumentException("Path direction or turn is not recognized.", nameof(payload));
            AddUInt16(bytes, segment.DistanceMm);
            bytes.Add((byte)segment.Direction);
            bytes.Add((byte)segment.Turn);
        }

        if (bytes.Count > Stm32FrameCodec.MaximumPayloadLength)
            throw new ArgumentException("Path payload exceeds the maximum STM32 frame payload length.", nameof(payload));
        return bytes.ToArray();
    }

    public static PathPayload Decode(ReadOnlySpan<byte> payload)
    {
        if (payload.Length == 0 || payload[0] != PayloadVersion)
            throw new NotSupportedException($"Unsupported path payload version {(payload.Length == 0 ? -1 : payload[0])}.");
        if (payload.Length < 5) throw new ArgumentException("Path payload is truncated.", nameof(payload));

        var offset = 1;
        var entryCount = ReadUInt16(payload, ref offset);
        if (entryCount > MaximumEntryCount)
            throw new ArgumentException($"A path cannot contain more than {MaximumEntryCount} entries.", nameof(payload));
        var minimumLength = 1 + 2 + entryCount * 6 + 2;
        if (payload.Length < minimumLength) throw new ArgumentException("Path payload is truncated.", nameof(payload));

        var entries = new PathEntry[entryCount];
        for (var index = 0; index < entries.Length; index++)
        {
            var row = payload[offset++];
            var column = payload[offset++];
            if (row is < 1 or > 5 || column is < 1 or > 5)
                throw new ArgumentException("Path cells must be inside the 5 x 5 grid.", nameof(payload));
            entries[index] = new PathEntry(row, column, ReadUInt16(payload, ref offset), ReadUInt16(payload, ref offset));
        }

        var segmentCount = ReadUInt16(payload, ref offset);
        if (segmentCount != Math.Max(0, entryCount - 1))
            throw new ArgumentException("Path segment count must be one less than entry count.", nameof(payload));
        if (payload.Length != offset + segmentCount * 4)
            throw new ArgumentException("Path payload length does not match its counts.", nameof(payload));

        var segments = new PathSegment[segmentCount];
        for (var index = 0; index < segments.Length; index++)
        {
            var distance = ReadUInt16(payload, ref offset);
            var direction = (CardinalDirection)payload[offset++];
            var turn = (TurnInstruction)payload[offset++];
            if (!Enum.IsDefined(direction) || !Enum.IsDefined(turn))
                throw new ArgumentException("Path direction or turn is not recognized.", nameof(payload));
            segments[index] = new PathSegment(distance, direction, turn);
        }

        return new PathPayload(PayloadVersion, entries, segments);
    }

    private static ushort ToUInt16(double value, string name)
    {
        if (!double.IsFinite(value) || value < 0 || value > ushort.MaxValue)
            throw new ArgumentOutOfRangeException(name);
        return checked((ushort)Math.Round(value, MidpointRounding.AwayFromZero));
    }

    private static void AddUInt16(List<byte> bytes, ushort value)
    {
        bytes.Add((byte)value);
        bytes.Add((byte)(value >> 8));
    }

    private static ushort ReadUInt16(ReadOnlySpan<byte> payload, ref int offset)
    {
        var value = (ushort)(payload[offset] | (payload[offset + 1] << 8));
        offset += 2;
        return value;
    }
}
