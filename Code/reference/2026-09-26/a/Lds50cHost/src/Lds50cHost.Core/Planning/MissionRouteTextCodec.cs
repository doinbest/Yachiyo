using System.Globalization;
using Lds50cHost.Core.Configuration;

namespace Lds50cHost.Core.Planning;

public static class MissionRouteTextCodec
{
    private static readonly char[] Separators = ['-', ',', '，', ' ', '\t', '\r', '\n'];

    public static bool TryParse(string? text, out MissionRouteSettings settings, out string error)
    {
        settings = MissionRouteSettings.CreateDefault();
        if (string.IsNullOrWhiteSpace(text))
        {
            error = "请输入任务路线。";
            return false;
        }

        var tokens = text.Split(Separators, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        var sequence = new int[tokens.Length];
        for (var index = 0; index < tokens.Length; index++)
        {
            if (!int.TryParse(tokens[index], NumberStyles.None, CultureInfo.InvariantCulture, out sequence[index]))
            {
                error = $"无法识别任务点编号“{tokens[index]}”。";
                return false;
            }
        }

        var parsed = new MissionRouteSettings { Sequence = sequence };
        var errors = parsed.Validate();
        if (errors.Count > 0)
        {
            error = string.Join(" ", errors);
            return false;
        }

        settings = parsed;
        error = string.Empty;
        return true;
    }

    public static string Format(MissionRouteSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        var errors = settings.Validate();
        if (errors.Count > 0) throw new ArgumentException(string.Join(" ", errors), nameof(settings));
        return string.Join('-', settings.Sequence);
    }
}
