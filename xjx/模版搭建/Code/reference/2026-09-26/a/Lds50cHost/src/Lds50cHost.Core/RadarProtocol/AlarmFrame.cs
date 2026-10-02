namespace Lds50cHost.Core.RadarProtocol;

public sealed record AlarmFrame(ushort Code) : LdsFrame;
