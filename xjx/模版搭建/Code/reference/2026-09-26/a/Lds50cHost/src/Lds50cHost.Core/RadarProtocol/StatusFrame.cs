namespace Lds50cHost.Core.RadarProtocol;

public sealed record StatusFrame : LdsFrame
{
    private readonly byte[] _reservedBytes;

    public StatusFrame(byte flags, ReadOnlySpan<byte> reservedBytes)
    {
        if (reservedBytes.Length != 3)
            throw new ArgumentException("A status frame contains exactly three reserved bytes.", nameof(reservedBytes));

        Flags = flags;
        _reservedBytes = reservedBytes.ToArray();
    }

    public byte Flags { get; }
    public IReadOnlyList<byte> ReservedBytes => Array.AsReadOnly(_reservedBytes);
    public bool UsesMillimetres => (Flags & 0b0001) != 0;
    public bool EnergyEnabled => (Flags & 0b0010) != 0;
    public bool TrailingPointRemovalEnabled => (Flags & 0b0100) != 0;
    public bool FilterEnabled => (Flags & 0b1000) != 0;
}
