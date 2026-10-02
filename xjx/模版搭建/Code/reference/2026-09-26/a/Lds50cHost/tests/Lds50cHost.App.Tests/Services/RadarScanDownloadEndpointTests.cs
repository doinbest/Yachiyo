using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Export;
using Lds50cHost.Core.Models;
using Lds50cHost.Core.Scanning;
using Lds50cHost.Services;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;

namespace Lds50cHost.App.Tests.Services;

public sealed class RadarScanDownloadEndpointTests
{
    [Fact]
    public async Task DownloadLatest_WithoutACompleteScanReturnsNotFound()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());

        var response = await ExecuteAsync(RadarScanDownloadEndpoint.DownloadLatest(state));

        Assert.Equal(StatusCodes.Status404NotFound, response.StatusCode);
        Assert.Empty(response.Body);
    }

    [Fact]
    public async Task DownloadLatest_WithACompleteScanReturnsNamedCsv()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var scan = CreateScan(321, new DateTimeOffset(2026, 9, 28, 9, 12, 34, TimeSpan.Zero));
        state.SetRawScan(scan);

        var response = await ExecuteAsync(RadarScanDownloadEndpoint.DownloadLatest(state));

        Assert.Equal(StatusCodes.Status200OK, response.StatusCode);
        Assert.Equal("text/csv; charset=utf-8", response.ContentType);
        Assert.Contains("lds50c-scan-20260928-091234.csv", response.ContentDisposition ?? string.Empty);
        Assert.Equal(RadarScanCsvExporter.Export(scan), response.Body);
    }

    [Fact]
    public async Task DownloadLatest_StateReplacementAfterCreationCannotChangeTheResponseSnapshot()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var original = CreateScan(400, new DateTimeOffset(2026, 9, 28, 10, 0, 0, TimeSpan.Zero));
        state.SetRawScan(original);
        var result = RadarScanDownloadEndpoint.DownloadLatest(state);

        state.SetRawScan(CreateScan(900, new DateTimeOffset(2026, 9, 28, 10, 0, 1, TimeSpan.Zero)));
        var response = await ExecuteAsync(result);

        Assert.Equal(RadarScanCsvExporter.Export(original), response.Body);
    }

    [Fact]
    public async Task CreateResult_ExporterFailureReturnsServerErrorLogsAndKeepsTheScan()
    {
        var state = new ApplicationState(AppConfiguration.CreateDefault());
        var scan = CreateScan(500, DateTimeOffset.UtcNow);
        state.SetRawScan(scan);

        var result = RadarScanDownloadEndpoint.CreateResult(
            state,
            _ => throw new InvalidOperationException("test export failure"));
        var response = await ExecuteAsync(result);

        Assert.Equal(StatusCodes.Status500InternalServerError, response.StatusCode);
        Assert.Contains(state.Logs, line => line.Contains("test export failure", StringComparison.Ordinal));
        Assert.True(ReferenceEquals(scan, state.RawScan));
    }

    private static RadarScan CreateScan(ushort distanceMm, DateTimeOffset completedAt) =>
        new([new RadarPoint(12.5, distanceMm, 42)], completedAt);

    private static async Task<ResponseSnapshot> ExecuteAsync(IResult result)
    {
        using var services = new ServiceCollection()
            .AddLogging()
            .BuildServiceProvider();
        var body = new MemoryStream();
        var context = new DefaultHttpContext
        {
            RequestServices = services
        };
        context.Response.Body = body;

        await result.ExecuteAsync(context);

        return new ResponseSnapshot(
            context.Response.StatusCode,
            context.Response.ContentType,
            context.Response.Headers.ContentDisposition.ToString(),
            body.ToArray());
    }

    private sealed record ResponseSnapshot(
        int StatusCode,
        string? ContentType,
        string? ContentDisposition,
        byte[] Body);
}
