using System.Text.Json;
using Lds50cHost.Core.Configuration;
using Lds50cHost.Core.Stm32;

namespace Lds50cHost.Services;

public sealed record ConfigurationLoadResult(AppConfiguration Configuration, string? Warning);

public sealed class ConfigurationStore
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true,
        WriteIndented = true
    };

    private readonly string _path;

    public ConfigurationStore(string? path = null)
    {
        _path = path ?? Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Lds50cHost",
            "settings.json");
    }

    public async Task SaveAsync(AppConfiguration configuration, CancellationToken cancellationToken)
    {
        ValidateConfiguration(configuration);
        var directory = Path.GetDirectoryName(_path);
        if (!string.IsNullOrEmpty(directory)) Directory.CreateDirectory(directory);

        var json = JsonSerializer.Serialize(configuration, JsonOptions);
        var temporaryPath = _path + ".tmp";
        try
        {
            await File.WriteAllTextAsync(temporaryPath, json, cancellationToken);
            File.Move(temporaryPath, _path, overwrite: true);
        }
        finally
        {
            if (File.Exists(temporaryPath)) File.Delete(temporaryPath);
        }
    }

    public async Task<ConfigurationLoadResult> LoadAsync(CancellationToken cancellationToken)
    {
        if (!File.Exists(_path)) return new ConfigurationLoadResult(AppConfiguration.CreateDefault(), null);

        try
        {
            var json = await File.ReadAllTextAsync(_path, cancellationToken);
            var configuration = JsonSerializer.Deserialize<AppConfiguration>(json, JsonOptions) ??
                throw new JsonException("The configuration document was empty.");
            ValidateConfiguration(configuration);
            return new ConfigurationLoadResult(configuration, null);
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception exception) when (exception is JsonException or IOException or UnauthorizedAccessException or ArgumentException or OverflowException)
        {
            return new ConfigurationLoadResult(
                AppConfiguration.CreateDefault(),
                $"Saved configuration could not be used; defaults were loaded. {exception.Message}");
        }
    }

    private static void ValidateConfiguration(AppConfiguration configuration)
    {
        ArgumentNullException.ThrowIfNull(configuration);
        _ = ConfigurationPayloadCodec.Encode(configuration);
        if (!Enum.IsDefined(configuration.PlanningMode))
            throw new ArgumentException("Unknown planning mode.", nameof(configuration));
        if (configuration.MissionRoute is null)
            throw new ArgumentException("Mission route settings are required.", nameof(configuration));
        var missionErrors = configuration.MissionRoute.Validate();
        if (missionErrors.Count > 0)
            throw new ArgumentException(string.Join(" ", missionErrors), nameof(configuration));
    }
}
