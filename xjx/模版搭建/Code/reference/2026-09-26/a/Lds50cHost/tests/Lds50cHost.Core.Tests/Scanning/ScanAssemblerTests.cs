using Lds50cHost.Core.RadarProtocol;
using Lds50cHost.Core.Scanning;

namespace Lds50cHost.Core.Tests.Scanning;

public sealed class ScanAssemblerTests
{
    [Fact]
    public void Push_FixedResolutionUsesTheInclusiveSectorEndpoints()
    {
        var assembler = CreateArmedAssembler();
        var frame = FixedFrame(100, 100, 3);

        var result = assembler.Push(frame);

        Assert.Equal(new[] { 10d, 15d, 20d }, result.NewPoints.Select(point => point.AngleDeg));
    }

    [Fact]
    public void Push_AOnePointFixedPacketUsesItsStartAngle()
    {
        var assembler = CreateArmedAssembler();

        var result = assembler.Push(FixedFrame(1234, 500, 1));

        var point = Assert.Single(result.NewPoints);
        Assert.Equal(123.4d, point.AngleDeg);
    }

    [Fact]
    public void Push_NormalPacketWaitsForTheNextStartAndUsesDeltaDividedByCount()
    {
        var assembler = CreateArmedAssembler();

        var pending = assembler.Push(NormalFrame(100, 2));
        var resolved = assembler.Push(NormalFrame(200, 1));

        Assert.Empty(pending.NewPoints);
        Assert.Equal(new[] { 10d, 15d }, resolved.NewPoints.Select(point => point.AngleDeg));
    }

    [Fact]
    public void Push_DiscardsThePartialRevolutionBeforeTheFirstWrap()
    {
        var assembler = new ScanAssembler(minimumPointsPerScan: 1);

        var first = assembler.Push(FixedFrame(1800, 50, 2));
        var second = assembler.Push(FixedFrame(2700, 50, 2));
        var firstWrap = assembler.Push(FixedFrame(100, 50, 2));

        Assert.Empty(first.NewPoints);
        Assert.Empty(second.NewPoints);
        Assert.Equal(new[] { 10d, 15d }, firstWrap.NewPoints.Select(point => point.AngleDeg));
        Assert.Null(firstWrap.CompletedScan);
    }

    [Fact]
    public void Push_EmitsExactlyOneCompleteScanAtTheSecondWrap()
    {
        var assembler = new ScanAssembler(minimumPointsPerScan: 30);
        var completed = new List<RadarScan>();

        foreach (var frame in new[]
        {
            FixedFrame(3500, 40, 10),
            FixedFrame(0, 40, 10),
            FixedFrame(1200, 40, 10),
            FixedFrame(2400, 40, 10),
            FixedFrame(3500, 40, 10),
            FixedFrame(0, 40, 10)
        })
        {
            var result = assembler.Push(frame);
            if (result.CompletedScan is not null) completed.Add(result.CompletedScan);
        }

        var scan = Assert.Single(completed);
        Assert.Equal(40, scan.Points.Count);
        Assert.Equal(0d, scan.Points[0].AngleDeg);
        Assert.Equal(350d, scan.Points[30].AngleDeg);
    }

    [Fact]
    public void Push_EmptyOrMissingPacketsCannotProduceAScan()
    {
        var assembler = new ScanAssembler(minimumPointsPerScan: 30);

        assembler.Push(FixedFrame(3500, 0, 0));
        assembler.Push(FixedFrame(0, 0, 0));
        assembler.Push(FixedFrame(1800, 0, 0));
        var secondWrap = assembler.Push(FixedFrame(0, 0, 0));

        Assert.Null(secondWrap.CompletedScan);
    }

    private static ScanAssembler CreateArmedAssembler()
    {
        var assembler = new ScanAssembler(minimumPointsPerScan: 1);
        assembler.Push(FixedFrame(3500, 0, 0));
        assembler.Push(FixedFrame(0, 0, 0));
        return assembler;
    }

    private static MeasurementFrame FixedFrame(ushort start, ushort sector, int count) =>
        new(
            true,
            checked((ushort)count),
            start,
            sector,
            Enumerable.Range(0, count)
                .Select(index => new RawMeasurement(checked((byte)(index + 1)), checked((ushort)(100 + index))))
                .ToArray());

    private static MeasurementFrame NormalFrame(ushort start, int count) =>
        new(
            false,
            checked((ushort)count),
            start,
            null,
            Enumerable.Range(0, count)
                .Select(index => new RawMeasurement(checked((byte)(index + 1)), checked((ushort)(200 + index))))
                .ToArray());
}
