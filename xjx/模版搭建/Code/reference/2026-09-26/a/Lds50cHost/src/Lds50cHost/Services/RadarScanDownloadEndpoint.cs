using Lds50cHost.Core.Export;
using Lds50cHost.Core.Scanning;
using Microsoft.AspNetCore.Http;

namespace Lds50cHost.Services;

public static class RadarScanDownloadEndpoint
{
    public static IResult DownloadLatest(ApplicationState state) =>
        CreateResult(state, RadarScanCsvExporter.Export);

    internal static IResult CreateResult(
        ApplicationState state,
        Func<RadarScan, byte[]> exporter)
    {
        ArgumentNullException.ThrowIfNull(state);
        ArgumentNullException.ThrowIfNull(exporter);

        var scan = state.RawScan;
        if (scan is null) return Results.NotFound();

        try
        {
            var bytes = exporter(scan);
            var fileName = $"lds50c-scan-{scan.CompletedAt:yyyyMMdd-HHmmss}.csv";
            return Results.File(bytes, "text/csv; charset=utf-8", fileName);
        }
        catch (Exception exception)
        {
            state.AddLog($"Radar CSV export failed: {exception.Message}");
            return Results.Problem(
                title: "Radar scan export failed.",
                detail: exception.Message,
                statusCode: StatusCodes.Status500InternalServerError);
        }
    }
}
