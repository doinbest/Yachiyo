namespace Lds50cHost.Core.Mapping;

public sealed record GridCell(
    int Row,
    int Column,
    double MinXmm,
    double MaxXmm,
    double MinYmm,
    double MaxYmm)
{
    public int BitIndex => ToBitIndex(Row, Column);
    public double CenterXmm => (MinXmm + MaxXmm) / 2d;
    public double CenterYmm => (MinYmm + MaxYmm) / 2d;

    public static int ToBitIndex(int row, int column)
    {
        if (row is < 1 or > 5) throw new ArgumentOutOfRangeException(nameof(row));
        if (column is < 1 or > 5) throw new ArgumentOutOfRangeException(nameof(column));
        return (row - 1) * 5 + column - 1;
    }
}
