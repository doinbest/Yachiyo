namespace Lds50cHost.Core.Planning;

public sealed record PathWaypoint(double Xmm, double Ymm, int Row, int Column);

public enum CardinalDirection : byte
{
    North,
    East,
    South,
    West
}

public enum TurnInstruction : byte
{
    Start,
    Straight,
    Left,
    Right,
    Reverse
}
