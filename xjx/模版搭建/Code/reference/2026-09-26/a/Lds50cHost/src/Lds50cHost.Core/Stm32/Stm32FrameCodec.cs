namespace Lds50cHost.Core.Stm32;

public sealed class Stm32FrameCodec
{
    public const int MaximumPayloadLength = 4096;

    private readonly List<byte> _buffer = [];

    public long ValidFrames { get; private set; }
    public long CrcErrors { get; private set; }
    public long LengthErrors { get; private set; }
    public long DiscardedBytes { get; private set; }

    public static byte[] Encode(Stm32Frame frame)
    {
        ArgumentNullException.ThrowIfNull(frame);
        var payloadLength = frame.PayloadSpan.Length;
        var bytes = new byte[10 + payloadLength];
        bytes[0] = 0xA5;
        bytes[1] = 0x5A;
        bytes[2] = frame.Version;
        bytes[3] = (byte)frame.Command;
        WriteUInt16(bytes, 4, frame.Sequence);
        WriteUInt16(bytes, 6, checked((ushort)payloadLength));
        frame.PayloadSpan.CopyTo(bytes.AsSpan(8));
        var crc = Crc16Ccitt.Compute(bytes.AsSpan(2, 6 + payloadLength));
        WriteUInt16(bytes, 8 + payloadLength, crc);
        return bytes;
    }

    public IReadOnlyList<Stm32Frame> Feed(ReadOnlySpan<byte> bytes)
    {
        foreach (var value in bytes) _buffer.Add(value);

        var frames = new List<Stm32Frame>();
        while (_buffer.Count > 0)
        {
            var prefixIndex = FindPrefix();
            if (prefixIndex < 0)
            {
                var keep = _buffer[^1] == 0xA5 ? 1 : 0;
                RemovePrefix(_buffer.Count - keep, true);
                break;
            }

            if (prefixIndex > 0) RemovePrefix(prefixIndex, true);
            if (_buffer.Count < 8) break;

            var payloadLength = ReadUInt16(6);
            if (payloadLength > MaximumPayloadLength)
            {
                LengthErrors++;
                RemovePrefix(1, true);
                continue;
            }

            var frameLength = 10 + payloadLength;
            if (_buffer.Count < frameLength) break;

            var crcBytes = _buffer.Skip(2).Take(6 + payloadLength).ToArray();
            var actualCrc = Crc16Ccitt.Compute(crcBytes);
            var expectedCrc = ReadUInt16(8 + payloadLength);
            if (actualCrc != expectedCrc)
            {
                CrcErrors++;
                RemovePrefix(frameLength, true);
                continue;
            }

            var payload = _buffer.Skip(8).Take(payloadLength).ToArray();
            frames.Add(new Stm32Frame(_buffer[2], (Stm32Command)_buffer[3], ReadUInt16(4), payload));
            ValidFrames++;
            RemovePrefix(frameLength, false);
        }

        return frames;
    }

    public void Reset()
    {
        _buffer.Clear();
        ValidFrames = 0;
        CrcErrors = 0;
        LengthErrors = 0;
        DiscardedBytes = 0;
    }

    private int FindPrefix()
    {
        for (var index = 0; index + 1 < _buffer.Count; index++)
        {
            if (_buffer[index] == 0xA5 && _buffer[index + 1] == 0x5A) return index;
        }

        return -1;
    }

    private ushort ReadUInt16(int offset) =>
        unchecked((ushort)(_buffer[offset] | (_buffer[offset + 1] << 8)));

    private static void WriteUInt16(Span<byte> destination, int offset, ushort value)
    {
        destination[offset] = (byte)value;
        destination[offset + 1] = (byte)(value >> 8);
    }

    private void RemovePrefix(int count, bool discarded)
    {
        if (count <= 0) return;
        _buffer.RemoveRange(0, count);
        if (discarded) DiscardedBytes += count;
    }
}
