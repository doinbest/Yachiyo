using System.Globalization;
using System.Text;
using Lds50cHost.Core.Export;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.Scanning;

namespace Lds50cHost.Core.Tests.Export;

public sealed class RadarScanCsvExporterTests
{
    [Fact]
    public void Export_WritesBomHeaderOrderedRowsAndInvariantAngles()
    {
        var scan = new RadarScan(
            [
                new RadarPoint(0, 123, 4),
                new RadarPoint(12.3456789, 456, 7)
            ],
            new DateTimeOffset(2026, 9, 28, 9, 0, 0, TimeSpan.Zero));

        var bytes = RadarScanCsvExporter.Export(scan);

        Assert.Equal(new byte[] { 0xEF, 0xBB, 0xBF }, bytes[..3]);
        Assert.Equal(
            "index,angle_deg,distance_mm,energy\r\n" +
            "0,0,123,4\r\n" +
            "1,12.345679,456,7\r\n",
            Encoding.UTF8.GetString(bytes[3..]));
    }

    [Fact]
    public void Export_CommaDecimalCultureStillUsesADecimalPoint()
    {
        var previousCulture = CultureInfo.CurrentCulture;
        var previousUiCulture = CultureInfo.CurrentUICulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("de-DE");
            CultureInfo.CurrentUICulture = CultureInfo.GetCultureInfo("de-DE");
            var scan = new RadarScan(
                [new RadarPoint(1.5, 2000, 99)],
                DateTimeOffset.UtcNow);

            var bytes = RadarScanCsvExporter.Export(scan);

            Assert.Equal(
                "index,angle_deg,distance_mm,energy\r\n0,1.5,2000,99\r\n",
                Encoding.UTF8.GetString(bytes[3..]));
        }
        finally
        {
            CultureInfo.CurrentCulture = previousCulture;
            CultureInfo.CurrentUICulture = previousUiCulture;
        }
    }
}
