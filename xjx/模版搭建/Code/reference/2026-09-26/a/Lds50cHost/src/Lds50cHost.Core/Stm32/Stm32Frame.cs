namespace Lds50cHost.Core.Stm32;

public sealed class Stm32Frame
{
    private readonly byte[] _payload;

    public Stm32Frame(byte version, Stm32Command command, ushort sequence, IEnumerable<byte> payload)
    {
        ArgumentNullException.ThrowIfNull(payload);
        _payload = payload.ToArray();
        if (_payload.Length > Stm32FrameCodec.MaximumPayloadLength)
            throw new ArgumentException("STM32 payload cannot exceed 4096 bytes.", nameof(payload));

        Version = version;
        Command = command;
        Sequence = sequence;
    }

    public byte Version { get; }
    public Stm32Command Command { get; }
    public ushort Sequence { get; }
    public IReadOnlyList<byte> Payload => Array.AsReadOnly(_payload);
    internal ReadOnlySpan<byte> PayloadSpan => _payload;
}
