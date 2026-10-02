using Lds50cHost.Core.Models;

namespace Lds50cHost.Core.Mapping;

public static class ObstacleClassifier
{
    public static ObstacleMap Classify(
        GridDefinition grid,
        IEnumerable<FieldPoint> points,
        int threshold,
        uint manualMask,
        uint protectedMask = 0)
    {
        ArgumentNullException.ThrowIfNull(grid);
        ArgumentNullException.ThrowIfNull(points);
        if (threshold < 1) throw new ArgumentOutOfRangeException(nameof(threshold));

        var counts = new int[25];
        foreach (var point in points)
        {
            var cell = grid.Locate(point.Xmm, point.Ymm);
            if (cell is not null) counts[cell.BitIndex]++;
        }

        var cells = new CellObstacle[25];
        foreach (var cell in grid.Cells)
        {
            var bit = 1u << cell.BitIndex;
            var flags = ObstacleFlags.None;
            if ((grid.FixedBlockedMask & bit) != 0) flags |= ObstacleFlags.Fixed;
            if ((manualMask & bit) != 0) flags |= ObstacleFlags.Manual;
            if (counts[cell.BitIndex] >= threshold) flags |= ObstacleFlags.Scanned;
            cells[cell.BitIndex] = new CellObstacle(cell, flags, counts[cell.BitIndex]);
        }

        return new ObstacleMap(cells, manualMask).WithProtectedCells(protectedMask);
    }
}
