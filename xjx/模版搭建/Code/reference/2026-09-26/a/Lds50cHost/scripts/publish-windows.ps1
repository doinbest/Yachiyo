[CmdletBinding()]
param(
    [string]$Configuration = "Release",
    [string]$Runtime = "win-x64"
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$privateDotnet = Join-Path $projectRoot ".tools\dotnet\dotnet.exe"
$dotnet = if (Test-Path -LiteralPath $privateDotnet) { $privateDotnet } else { "dotnet" }
$project = Join-Path $projectRoot "src\Lds50cHost\Lds50cHost.csproj"
$packageSource = Join-Path $projectRoot "packages"
$packageCache = Join-Path $projectRoot ".tools\nuget"
$output = Join-Path $projectRoot "artifacts\publish\$Runtime"
$runtimePackages = @(
    "microsoft.netcore.app.runtime.win-x64.10.0.11.nupkg",
    "microsoft.aspnetcore.app.runtime.win-x64.10.0.11.nupkg",
    "microsoft.windowsdesktop.app.runtime.win-x64.10.0.11.nupkg"
)

$env:DOTNET_CLI_HOME = Join-Path $projectRoot ".tools\dotnet-home"
$env:DOTNET_SKIP_FIRST_TIME_EXPERIENCE = "1"
$env:DOTNET_NOLOGO = "1"

$publishArguments = @(
    "publish", $project,
    "--configuration", $Configuration,
    "--runtime", $Runtime,
    "--self-contained", "true",
    "--output", $output,
    "--packages", $packageCache,
    "--force",
    "-p:NuGetAudit=false",
    "-p:PublishSingleFile=false",
    "-p:DebugType=None",
    "-p:DebugSymbols=false"
)

if ($runtimePackages.Where({ -not (Test-Path -LiteralPath (Join-Path $packageSource $_)) }).Count -eq 0) {
    $publishArguments += @("--source", $packageSource)
}

& $dotnet @publishArguments

if ($LASTEXITCODE -ne 0) { throw "dotnet publish failed with exit code $LASTEXITCODE." }

$executable = Join-Path $output "Lds50cHost.exe"
if (-not (Test-Path -LiteralPath $executable)) { throw "Publish completed without $executable." }

Write-Host "Published self-contained Windows application: $output"
Write-Host "Run Lds50cHost.exe, then open the listening URL shown in its console."
