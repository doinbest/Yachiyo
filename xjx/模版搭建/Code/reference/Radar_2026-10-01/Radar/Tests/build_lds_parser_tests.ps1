$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$objectRoot = Join-Path $PSScriptRoot 'obj-lds-parser'
$armcc = 'C:\Keil_v5\ARM\ARMCC\bin\armcc.exe'
$armlink = 'C:\Keil_v5\ARM\ARMCC\bin\armlink.exe'

New-Item -ItemType Directory -Force -Path $objectRoot | Out-Null

$sources = @(
  (Join-Path $PSScriptRoot 'test_lds_parser.c'),
  (Join-Path $projectRoot 'Core\Src\lds_parser.c')
)

$objects = @()
foreach ($source in $sources)
{
  $object = Join-Path $objectRoot (([IO.Path]::GetFileNameWithoutExtension($source)) + '.o')
  & $armcc --cpu Cortex-M3 --c99 -c -g -O0 --apcs=interwork `
    -D ARM_SIM_TEST `
    -I (Join-Path $projectRoot 'Core\Inc') -o $object $source
  if ($LASTEXITCODE -ne 0)
  {
    exit $LASTEXITCODE
  }
  $objects += $object
}

$output = Join-Path $PSScriptRoot 'lds_parser_tests.axf'
$map = Join-Path $PSScriptRoot 'lds_parser_tests.map'
& $armlink --cpu Cortex-M3 --entry main --ro-base 0x08000000 `
  --rw-base 0x20000000 --map --list $map -o $output @objects
exit $LASTEXITCODE
