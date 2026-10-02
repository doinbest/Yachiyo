using Lds50cHost.Core.Configuration;

namespace Lds50cHost.Core.Mapping;

public sealed class GridDefinition
{
    private readonly double[] _xLines;
    private readonly double[] _yLines;

    public GridDefinition(MapSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        var errors = settings.Validate();
        if (errors.Count > 0)
            throw new ArgumentException(string.Join(" ", errors), nameof(settings));

        _xLines = settings.XLines.ToArray();
        _yLines = settings.YLines.ToArray();
        FixedBlockedMask = settings.FixedBlockedMask & 0x01FF_FFFFu;
    }

    public IReadOnlyList<double> XLines => Array.AsReadOnly(_xLines);
    public IReadOnlyList<double> YLines => Array.AsReadOnly(_yLines);
    public uint FixedBlockedMask { get; }
    public IReadOnlyList<GridCell> Cells =>
        Enumerable.Range(1, 5)
            .SelectMany(row => Enumerable.Range(1, 5).Select(column => GetCell(row, column)))
            .ToArray();

    public GridCell GetCell(int row, int column)
    {
        var bitIndex = GridCell.ToBitIndex(row, column);
        _ = bitIndex;
        return new GridCell(
            row,
            column,
            _xLines[column - 1],
            _xLines[column],
            _yLines[row - 1],
            _yLines[row]);
    }

    public GridCell? Locate(double xmm, double ymm)
    {
        var columnIndex = LocateAxis(_xLines, xmm);
        var rowIndex = LocateAxis(_yLines, ymm);
        return columnIndex < 0 || rowIndex < 0
            ? null
            : GetCell(rowIndex + 1, columnIndex + 1);
    }

    public bool TryMoveXLine(int internalIndex, double xmm, out string error) =>
        TryMoveLine(_xLines, internalIndex, xmm, "X", out error);

    public bool TryMoveYLine(int internalIndex, double ymm, out string error) =>
        TryMoveLine(_yLines, internalIndex, ymm, "Y", out error);

    private static int LocateAxis(IReadOnlyList<double> lines, double value)
    {
        if (!double.IsFinite(value) || value < lines[0] || value > lines[^1]) return -1;
        if (value == lines[^1]) return lines.Count - 2;

        for (var index = 0; index < lines.Count - 1; index++)
        {
            if (value < lines[index + 1]) return index;
        }

        return -1;
    }

    private static bool TryMoveLine(
        double[] lines,
        int internalIndex,
        double value,
        string axis,
        out string error)
    {
        if (internalIndex is < 0 or > 3)
        {
            error = $"{axis} internal line index must be between 0 and 3.";
            return false;
        }

        if (!double.IsFinite(value))
        {
            error = $"{axis} boundary must be a finite number.";
            return false;
        }

        var lineIndex = internalIndex + 1;
        if (value - lines[lineIndex - 1] < MapSettings.MinimumCellSizeMm ||
            lines[lineIndex + 1] - value < MapSettings.MinimumCellSizeMm)
        {
            error = $"{axis} boundary must keep both adjacent cells at least 50 mm wide.";
            return false;
        }

        lines[lineIndex] = value;
        error = string.Empty;
        return true;
    }
}
