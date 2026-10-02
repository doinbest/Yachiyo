namespace Lds50cHost.Core.Stm32;

public enum Stm32Command : byte
{
    StageConfiguration = 0x10,
    ApplyConfiguration = 0x11,
    SaveConfiguration = 0x12,
    ReadConfiguration = 0x13,
    SetPath = 0x20,
    Ack = 0x7E,
    Nack = 0x7F
}
