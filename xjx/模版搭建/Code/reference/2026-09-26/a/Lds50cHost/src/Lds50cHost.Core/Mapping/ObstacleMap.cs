namespace Lds50cHost.Core.Mapping;

[Flags]
public enum ObstacleFlags : byte
{
    None = 0,
    Fixed = 1,
    Manual = 2,
    Scanned = 4
}

public sealed record CellObstacle(GridCell Cell, ObstacleFlags Flags, int ScannedPointCount, bool IsProtected = false)
{
    public bool IsBlocked => Flags != ObstacleFlags.None;
}

public sealed class ObstacleMap
{
    private readonly CellObstacle[] _cells;

    public ObstacleMap(IEnumerable<CellObstacle> cells, uint manualMask)
    {
        _cells = cells.OrderBy(cell => cell.Cell.BitIndex).ToArray();
        if (_cells.Length != 25)
            throw new ArgumentException("An obstacle map must contain exactly 25 cells.", nameof(cells));

        ManualMask = manualMask & 0x01FF_FFFFu;
    }

    public uint ManualMask { get; }
    public IReadOnlyList<CellObstacle> Cells => Array.AsReadOnly(_cells);

    public CellObstacle Get(int row, int column) => _cells[GridCell.ToBitIndex(row, column)];

    public bool IsBlocked(int row, int column) => Get(row, column).IsBlocked;

    public bool TryToggleManual(int row, int column, out uint updatedManualMask)
    {
        var cell = Get(row, column);
        if (cell.Flags.HasFlag(ObstacleFlags.Fixed) || cell.IsProtected)
        {
            updatedManualMask = ManualMask;
            return false;
        }

        updatedManualMask = ManualMask ^ (1u << cell.Cell.BitIndex);
        return true;
    }

    public ObstacleMap WithProtectedCells(uint protectedMask)
    {
        protectedMask &= 0x01FF_FFFFu;
        var cells = _cells.Select(cell =>
        {
            var requested = (protectedMask & (1u << cell.Cell.BitIndex)) != 0;
            var isProtected = (cell.IsProtected || requested) && !cell.Flags.HasFlag(ObstacleFlags.Fixed);
            var flags = isProtected
                ? cell.Flags & ~(ObstacleFlags.Manual | ObstacleFlags.Scanned)
                : cell.Flags;
            return cell with { Flags = flags, IsProtected = isProtected };
        });
        return new ObstacleMap(cells, ManualMask & ~protectedMask);
    }
}
