namespace Lds50cHost.Core.RadarProtocol;

public sealed class Lds50cParser
{
    private static readonly byte[][] Headers =
    [
        [0xCE, 0xFA],
        [0xCF, 0xFA],
        [0x53, 0x54],
        [0xCE, 0xCE, 0xCE, 0xCE]
    ];

    private readonly List<byte> _buffer = [];
    private readonly int _maximumPointCount;

    public Lds50cParser(int maximumPointCount = 4096)
    {
        if (maximumPointCount is < 1 or > ushort.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(maximumPointCount));

        _maximumPointCount = maximumPointCount;
    }

    public long ValidFrames { get; private set; }
    public long ChecksumErrors { get; private set; }
    public long LengthErrors { get; private set; }
    public long DiscardedBytes { get; private set; }

    public IReadOnlyList<LdsFrame> Feed(ReadOnlySpan<byte> bytes)
    {
        foreach (var value in bytes) _buffer.Add(value);

        var frames = new List<LdsFrame>();
        while (_buffer.Count > 0)
        {
            var headerIndex = FindCompleteHeader();
            if (headerIndex < 0)
            {
                RetainPotentialHeaderPrefix();
                break;
            }

            if (headerIndex > 0) RemovePrefix(headerIndex, countAsDiscarded: true);

            var result = _buffer[0] switch
            {
                0xCE when _buffer.Count >= 2 && _buffer[1] == 0xFA => ParseMeasurement(frames, false),
                0xCF when _buffer.Count >= 2 && _buffer[1] == 0xFA => ParseMeasurement(frames, true),
                0x53 => ParseStatus(frames),
                0xCE => ParseAlarm(frames),
                _ => ParseResult.Invalid
            };

            if (result == ParseResult.NeedMore) break;
            if (result == ParseResult.Invalid) RemovePrefix(1, countAsDiscarded: true);
        }

        return frames;
    }

    public void Reset()
    {
        _buffer.Clear();
        ValidFrames = 0;
        ChecksumErrors = 0;
        LengthErrors = 0;
        DiscardedBytes = 0;
    }

    private ParseResult ParseMeasurement(List<LdsFrame> frames, bool fixedResolution)
    {
        const int commonHeaderLength = 6;
        if (_buffer.Count < commonHeaderLength) return ParseResult.NeedMore;

        var pointCount = ReadUInt16(2);
        if (pointCount > _maximumPointCount)
        {
            LengthErrors++;
            return ParseResult.Invalid;
        }

        var metadataLength = fixedResolution ? 8 : 6;
        var frameLength = checked(metadataLength + pointCount * 3 + 2);
        if (_buffer.Count < frameLength) return ParseResult.NeedMore;

        var startAngleTenths = ReadUInt16(4);
        ushort? sectorAngleTenths = fixedResolution ? ReadUInt16(6) : null;
        var measurements = new RawMeasurement[pointCount];
        var offset = metadataLength;
        for (var index = 0; index < measurements.Length; index++)
        {
            measurements[index] = new RawMeasurement(_buffer[offset], ReadUInt16(offset + 1));
            offset += 3;
        }

        var expectedChecksum = ReadUInt16(offset);
        var actualChecksum = LdsChecksum.ComputeMeasurement(
            pointCount,
            startAngleTenths,
            sectorAngleTenths,
            measurements);
        if (actualChecksum != expectedChecksum)
        {
            ChecksumErrors++;
            RemovePrefix(frameLength, countAsDiscarded: true);
            return ParseResult.Consumed;
        }

        frames.Add(new MeasurementFrame(
            fixedResolution,
            pointCount,
            startAngleTenths,
            sectorAngleTenths,
            measurements));
        ValidFrames++;
        RemovePrefix(frameLength, countAsDiscarded: false);
        return ParseResult.Consumed;
    }

    private ParseResult ParseStatus(List<LdsFrame> frames)
    {
        const int frameLength = 8;
        if (_buffer.Count < frameLength) return ParseResult.NeedMore;

        if (_buffer[6] != 0x45 || _buffer[7] != 0x44)
        {
            LengthErrors++;
            return ParseResult.Invalid;
        }

        frames.Add(new StatusFrame(_buffer[2], new byte[] { _buffer[3], _buffer[4], _buffer[5] }));
        ValidFrames++;
        RemovePrefix(frameLength, countAsDiscarded: false);
        return ParseResult.Consumed;
    }

    private ParseResult ParseAlarm(List<LdsFrame> frames)
    {
        const int frameLength = 6;
        if (_buffer.Count < frameLength) return ParseResult.NeedMore;

        frames.Add(new AlarmFrame(ReadUInt16(4)));
        ValidFrames++;
        RemovePrefix(frameLength, countAsDiscarded: false);
        return ParseResult.Consumed;
    }

    private int FindCompleteHeader()
    {
        for (var offset = 0; offset < _buffer.Count; offset++)
        {
            foreach (var header in Headers)
            {
                if (MatchesAt(offset, header)) return offset;
            }
        }

        return -1;
    }

    private bool MatchesAt(int offset, IReadOnlyList<byte> header)
    {
        if (_buffer.Count - offset < header.Count) return false;
        for (var index = 0; index < header.Count; index++)
        {
            if (_buffer[offset + index] != header[index]) return false;
        }

        return true;
    }

    private void RetainPotentialHeaderPrefix()
    {
        var maximumPrefixLength = Math.Min(_buffer.Count, Headers.Max(header => header.Length - 1));
        for (var length = maximumPrefixLength; length > 0; length--)
        {
            foreach (var header in Headers)
            {
                if (length >= header.Length) continue;

                var matches = true;
                for (var index = 0; index < length; index++)
                {
                    if (_buffer[_buffer.Count - length + index] == header[index]) continue;
                    matches = false;
                    break;
                }

                if (!matches) continue;
                RemovePrefix(_buffer.Count - length, countAsDiscarded: true);
                return;
            }
        }

        RemovePrefix(_buffer.Count, countAsDiscarded: true);
    }

    private ushort ReadUInt16(int offset) =>
        unchecked((ushort)(_buffer[offset] | (_buffer[offset + 1] << 8)));

    private void RemovePrefix(int count, bool countAsDiscarded)
    {
        if (count <= 0) return;
        _buffer.RemoveRange(0, count);
        if (countAsDiscarded) DiscardedBytes += count;
    }

    private enum ParseResult
    {
        NeedMore,
        Consumed,
        Invalid
    }
}
