using Lds50cHost.Components;
using Lds50cHost.Services;

var builder = WebApplication.CreateBuilder(new WebApplicationOptions
{
    Args = args,
    ContentRootPath = AppContext.BaseDirectory
});
builder.Logging.ClearProviders();
builder.Logging.AddConsole();
builder.Logging.AddDebug();

var configurationStore = new ConfigurationStore();
var loadedConfiguration = await configurationStore.LoadAsync(CancellationToken.None);
var applicationState = new ApplicationState(loadedConfiguration.Configuration);
if (!string.IsNullOrWhiteSpace(loadedConfiguration.Warning))
    applicationState.AddLog(loadedConfiguration.Warning);

// Add services to the container.
builder.Services.AddRazorComponents()
    .AddInteractiveServerComponents();
builder.Services.AddSingleton(configurationStore);
builder.Services.AddSingleton(applicationState);
builder.Services.AddSingleton<SerialByteTransport>();
builder.Services.AddSingleton<IByteTransport>(services => services.GetRequiredService<SerialByteTransport>());
builder.Services.AddSingleton<RadarCoordinator>();
builder.Services.AddSingleton(services => new Stm32Coordinator(
    services.GetRequiredService<IByteTransport>(),
    TimeSpan.FromSeconds(2)));

var app = builder.Build();

// Configure the HTTP request pipeline.
if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/Error", createScopeForErrors: true);
    // The default HSTS value is 30 days. You may want to change this for production scenarios, see https://aka.ms/aspnetcore-hsts.
    app.UseHsts();
}
app.UseStatusCodePagesWithReExecute("/not-found", createScopeForStatusCodePages: true);
app.UseAntiforgery();

app.MapStaticAssets();
app.MapGet(
    "/api/radar/latest-scan.csv",
    (ApplicationState state) => RadarScanDownloadEndpoint.DownloadLatest(state));
app.MapRazorComponents<App>()
    .AddInteractiveServerRenderMode();

app.Run();
