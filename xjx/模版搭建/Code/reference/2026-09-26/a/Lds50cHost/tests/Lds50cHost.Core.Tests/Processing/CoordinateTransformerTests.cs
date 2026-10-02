using Lds50cHost.Core.Models;
using Lds50cHost.Core.Processing;

namespace Lds50cHost.Core.Tests.Processing;

public sealed class CoordinateTransformerTests
{
    [Fact]
    public void ToField_MapsZeroDegreesUpFromTheRadar()
    {
        var source = new RadarPoint(0, 1000, 25);

        var field = CoordinateTransformer.ToField(source, 230, 230);

        AssertClose(230, field.Xmm);
        AssertClose(1230, field.Ymm);
        Assert.Equal(source, field.Source);
    }

    [Fact]
    public void ToField_MapsNinetyDegreesRightFromTheRadar()
    {
        var field = CoordinateTransformer.ToField(new RadarPoint(90, 1000, 25), 230, 230);

        AssertClose(1230, field.Xmm);
        AssertClose(230, field.Ymm);
    }

    private static void AssertClose(double expected, double actual) =>
        Assert.True(Math.Abs(expected - actual) < 1e-9, $"Expected {expected}, got {actual}.");
}
