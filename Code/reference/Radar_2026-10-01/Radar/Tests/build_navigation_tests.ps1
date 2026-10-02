param(
  [switch]$CompileOnly
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$objectRoot = Join-Path $PSScriptRoot 'obj-navigation'
$armcc = 'C:\Keil_v5\ARM\ARMCC\bin\armcc.exe'
$armlink = 'C:\Keil_v5\ARM\ARMCC\bin\armlink.exe'

New-Item -ItemType Directory -Force -Path $objectRoot | Out-Null

$sources = @(
  (Join-Path $PSScriptRoot 'test_navigation.c'),
  (Join-Path $projectRoot 'Core\Src\lds_dma_cursor.c'),
  (Join-Path $projectRoot 'Core\Src\navigation_config.c'),
  (Join-Path $projectRoot 'Core\Src\lds_result_queue.c'),
  (Join-Path $projectRoot 'Core\Src\nav_sine_q15.c'),
  (Join-Path $projectRoot 'Core\Src\nav_math.c'),
  (Join-Path $projectRoot 'Core\Src\grid_map.c'),
  (Join-Path $projectRoot 'Core\Src\scan_mapper.c'),
  (Join-Path $projectRoot 'Core\Src\path_planner.c'),
  (Join-Path $projectRoot 'Core\Src\navigation.c'),
  (Join-Path $projectRoot 'Core\Src\navigation_report.c')
) | Where-Object { Test-Path -LiteralPath $_ }

$objects = @()
foreach ($source in $sources)
{
  $object = Join-Path $objectRoot (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
  & $armcc --cpu Cortex-M3 --c99 -c -g -O0 --apcs=interwork `
    -D NAVIGATION_UNIT_TEST `
    -I (Join-Path $projectRoot 'Core\Inc') -o $object $source
  if ($LASTEXITCODE -ne 0)
  {
    exit $LASTEXITCODE
  }
  $objects += $object
}

if ($CompileOnly)
{
  exit 0
}

$output = Join-Path $PSScriptRoot 'navigation_tests.axf'
$map = Join-Path $PSScriptRoot 'navigation_tests.map'
& $armlink --cpu Cortex-M3 --entry main --ro-base 0x08000000 `
  --rw-base 0x20000000 --map --list $map -o $output @objects
exit $LASTEXITCODE
