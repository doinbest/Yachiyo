namespace Lds50cHost.Core.Planning;

public enum EndpointAnchor
{
    CellCenter,
    RadarOrigin,
    GoalZoneOne
}

public sealed record MapEndpoint(int Row, int Column, EndpointAnchor Anchor)
{
    public static MapEndpoint StartZone2 { get; } = new(1, 1, EndpointAnchor.RadarOrigin);
    public static MapEndpoint GoalZone1 { get; } = new(1, 5, EndpointAnchor.GoalZoneOne);

    public static MapEndpoint Cell(int row, int column) => new(row, column, EndpointAnchor.CellCenter);
}
