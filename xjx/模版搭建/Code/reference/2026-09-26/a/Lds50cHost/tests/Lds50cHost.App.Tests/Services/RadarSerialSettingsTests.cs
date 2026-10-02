using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class RadarSerialSettingsTests
{
    [Fact]
    public void SupportedBaudRates_MatchTheLds50cManual()
    {
        Assert.Equal(new[] { 384000, 500000, 768000, 921600 }, RadarSerialSettings.SupportedBaudRates);
        Assert.Equal(500000, RadarSerialSettings.DefaultBaudRate);
    }
}
