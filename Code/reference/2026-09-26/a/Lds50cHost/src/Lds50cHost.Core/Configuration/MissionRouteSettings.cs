namespace Lds50cHost.Core.Configuration;

public sealed record MissionRouteSettings
{
    public const int MinimumPointCount = 2;
    public const int MaximumPointCount = 16;

    private int[] _sequence = [];

    public IReadOnlyList<int> Sequence
    {
        get => Array.AsReadOnly(_sequence);
        init => _sequence = value?.ToArray() ?? [];
    }

    public static MissionRouteSettings CreateDefault() => new()
    {
        Sequence = new[] { 1, 5, 4, 2, 3, 4, 2, 3, 1 }
    };

    public IReadOnlyList<string> Validate()
    {
        var errors = new List<string>();
        if (_sequence.Length is < MinimumPointCount or > MaximumPointCount)
            errors.Add($"任务路线必须包含 {MinimumPointCount}～{MaximumPointCount} 个编号。");
        if (_sequence.Any(number => number is < 1 or > 5))
            errors.Add("任务点编号只能是 1～5。");
        if (_sequence.Length > 0 && (_sequence[0] != 1 || _sequence[^1] != 1))
            errors.Add("任务路线必须从 1 出发并回到 1。");
        if (_sequence.Zip(_sequence.Skip(1)).Any(pair => pair.First == pair.Second))
            errors.Add("相邻任务点不能使用相同编号。");
        return errors;
    }
}
