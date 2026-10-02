using Lds50cHost.Core.Mapping;

namespace Lds50cHost.Core.Planning;

public sealed record MissionPoint(int Number, MapEndpoint Endpoint, double Xmm, double Ymm);

public static class MissionPointCatalog
{
    public static uint ProtectedCellMask { get; } =
        1u << GridCell.ToBitIndex(3, 1) |
        1u << GridCell.ToBitIndex(5, 3) |
        1u << GridCell.ToBitIndex(3, 5) |
        1u << GridCell.ToBitIndex(1, 3);

    public static MissionPoint Resolve(
        int number,
        GridDefinition grid,
        double radarXmm,
        double radarYmm)
    {
        ArgumentNullException.ThrowIfNull(grid);
        if (number == 1)
            return new MissionPoint(number, MapEndpoint.StartZone2, radarXmm, radarYmm);

        var endpoint = number switch
        {
            2 => MapEndpoint.Cell(3, 1),
            3 => MapEndpoint.Cell(5, 3),
            4 => MapEndpoint.Cell(3, 5),
            5 => MapEndpoint.Cell(1, 3),
            _ => throw new ArgumentOutOfRangeException(nameof(number), "任务点编号只能是 1～5。")
        };
        var cell = grid.GetCell(endpoint.Row, endpoint.Column);
        return new MissionPoint(number, endpoint, cell.CenterXmm, cell.CenterYmm);
    }

    public static IReadOnlyList<MissionPoint> ResolveAll(
        GridDefinition grid,
        double radarXmm,
        double radarYmm) =>
        Enumerable.Range(1, 5)
            .Select(number => Resolve(number, grid, radarXmm, radarYmm))
            .ToArray();
}
