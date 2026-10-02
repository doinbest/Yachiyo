using System.Text.Json;
using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Stm32;
using Lds50cHost.Services;

namespace Lds50cHost.App.Tests.Services;

public sealed class ConfigurationStoreTests
{
    [Fact]
    public async Task SaveAndLoad_PlanningModeAndMissionRouteRoundTrip()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var store = new ConfigurationStore(Path.Combine(directory, "settings.json"));
            var configuration = AppConfiguration.CreateDefault() with
            {
                PlanningMode = PlanningMode.PointToPoint,
                MissionRoute = new MissionRouteSettings { Sequence = new[] { 1, 2, 1 } }
            };

            await store.SaveAsync(configuration, CancellationToken.None);
            var loaded = await store.LoadAsync(CancellationToken.None);

            Assert.Equal(PlanningMode.PointToPoint, loaded.Configuration.PlanningMode);
            Assert.Equal(new[] { 1, 2, 1 }, loaded.Configuration.MissionRoute.Sequence);
            Assert.Null(loaded.Warning);
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public async Task Load_LegacyJsonPreservesOldFieldsAndAddsMissionDefaults()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var path = Path.Combine(directory, "settings.json");
            var defaults = AppConfiguration.CreateDefault();
            var legacy = new
            {
                defaults.Filter,
                Map = defaults.Map with { RadarXmm = 333 },
                defaults.Start,
                defaults.Goal
            };
            await File.WriteAllTextAsync(
                path,
                JsonSerializer.Serialize(legacy),
                CancellationToken.None);

            var loaded = await new ConfigurationStore(path).LoadAsync(CancellationToken.None);

            Assert.Equal(333, loaded.Configuration.Map.RadarXmm);
            Assert.Equal(PlanningMode.MissionRoute, loaded.Configuration.PlanningMode);
            Assert.Equal(MissionRouteSettings.CreateDefault().Sequence, loaded.Configuration.MissionRoute.Sequence);
            Assert.Null(loaded.Warning);
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public async Task Load_InvalidMissionRouteFallsBackToDefaultsWithAWarning()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var path = Path.Combine(directory, "settings.json");
            var invalid = AppConfiguration.CreateDefault() with
            {
                MissionRoute = new MissionRouteSettings { Sequence = new[] { 1, 2 } }
            };
            await File.WriteAllTextAsync(
                path,
                JsonSerializer.Serialize(invalid),
                CancellationToken.None);

            var loaded = await new ConfigurationStore(path).LoadAsync(CancellationToken.None);

            Assert.Equal(MissionRouteSettings.CreateDefault().Sequence, loaded.Configuration.MissionRoute.Sequence);
            Assert.False(string.IsNullOrWhiteSpace(loaded.Warning));
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public async Task SaveAndLoad_ValidConfigurationRoundTrips()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var store = new ConfigurationStore(Path.Combine(directory, "settings.json"));
            var configuration = AppConfiguration.CreateDefault();

            await store.SaveAsync(configuration, CancellationToken.None);
            var loaded = await store.LoadAsync(CancellationToken.None);

            Assert.Equal(ConfigurationPayloadCodec.Encode(configuration), ConfigurationPayloadCodec.Encode(loaded.Configuration));
            Assert.True(string.IsNullOrEmpty(loaded.Warning));
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public async Task Load_CorruptJsonFallsBackToDefaultsWithAWarning()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var path = Path.Combine(directory, "settings.json");
            await File.WriteAllTextAsync(path, "{not-json", CancellationToken.None);
            var store = new ConfigurationStore(path);

            var loaded = await store.LoadAsync(CancellationToken.None);

            Assert.Equal(
                ConfigurationPayloadCodec.Encode(AppConfiguration.CreateDefault()),
                ConfigurationPayloadCodec.Encode(loaded.Configuration));
            Assert.True(!string.IsNullOrWhiteSpace(loaded.Warning));
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public async Task Save_RejectsInvalidConfiguration()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var store = new ConfigurationStore(Path.Combine(directory, "settings.json"));
            var defaults = AppConfiguration.CreateDefault();
            var invalid = defaults with { Map = defaults.Map with { RadarXmm = -1 } };

            await Assert.ThrowsAsync<ArgumentException>(async () =>
                await store.SaveAsync(invalid, CancellationToken.None));
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    private static string CreateTemporaryDirectory()
    {
        var path = Path.Combine(Path.GetTempPath(), "Lds50cHost.Tests", Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(path);
        return path;
    }
}
