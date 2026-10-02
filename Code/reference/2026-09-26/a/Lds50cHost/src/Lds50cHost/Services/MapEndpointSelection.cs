using Lds50cHost.Core.Mapping;
using Lds50cHost.Core.Planning;

namespace Lds50cHost.Services;

public static class MapEndpointSelection
{
    public const string StartZoneTwoValue = "start-zone-2";
    public const string GoalZoneOneValue = "goal-zone-1";

    public static string ToValue(MapEndpoint endpoint) => endpoint.Anchor switch
    {
        EndpointAnchor.RadarOrigin => StartZoneTwoValue,
        EndpointAnchor.GoalZoneOne => GoalZoneOneValue,
        _ => $"cell-{GridCell.ToBitIndex(endpoint.Row, endpoint.Column)}"
    };

    public static bool TryParse(string? value, bool isStart, out MapEndpoint endpoint)
    {
        if (isStart && value == StartZoneTwoValue)
        {
            endpoint = MapEndpoint.StartZone2;
            return true;
        }

        if (!isStart && value == GoalZoneOneValue)
        {
            endpoint = MapEndpoint.GoalZone1;
            return true;
        }

        if (value?.StartsWith("cell-", StringComparison.Ordinal) == true &&
            int.TryParse(value.AsSpan(5), out var index) && index is >= 0 and < 25)
        {
            endpoint = MapEndpoint.Cell(index / 5 + 1, index % 5 + 1);
            return true;
        }

        endpoint = MapEndpoint.Cell(1, 1);
        return false;
    }
}
