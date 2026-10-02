using System.Globalization;
using System.Text;
using Lds50cHost.Core.Scanning;

namespace Lds50cHost.Core.Export;

public static class RadarScanCsvExporter
{
    private static readonly UTF8Encoding Utf8WithBom = new(encoderShouldEmitUTF8Identifier: true);

    public static byte[] Export(RadarScan scan)
    {
        ArgumentNullException.ThrowIfNull(scan);

        var csv = new StringBuilder("index,angle_deg,distance_mm,energy\r\n");
        for (var index = 0; index < scan.Points.Count; index++)
        {
            var point = scan.Points[index];
            csv.Append(index.ToString(CultureInfo.InvariantCulture)).Append(',')
                .Append(point.AngleDeg.ToString("0.######", CultureInfo.InvariantCulture)).Append(',')
                .Append(point.DistanceMm.ToString(CultureInfo.InvariantCulture)).Append(',')
                .Append(point.Energy.ToString(CultureInfo.InvariantCulture)).Append("\r\n");
        }

        var preamble = Utf8WithBom.GetPreamble();
        var content = Utf8WithBom.GetBytes(csv.ToString());
        var result = new byte[preamble.Length + content.Length];
        preamble.CopyTo(result, 0);
        content.CopyTo(result, preamble.Length);
        return result;
    }
}
