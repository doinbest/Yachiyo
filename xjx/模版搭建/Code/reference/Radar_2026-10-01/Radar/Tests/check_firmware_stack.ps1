param(
  [string]$AxfPath = (Join-Path (Split-Path -Parent $PSScriptRoot) 'MDK-ARM\Radar\Radar.axf')
)

$ErrorActionPreference = 'Stop'

$fromelf = 'C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe'
if (-not (Test-Path -LiteralPath $AxfPath))
{
  throw "Firmware image not found: $AxfPath"
}

$disassembly = (& $fromelf --text -c $AxfPath) -join "`n"
if ($LASTEXITCODE -ne 0)
{
  exit $LASTEXITCODE
}

function Get-StackAllocation([string]$FunctionName)
{
  $escaped = [Regex]::Escape($FunctionName)
  $match = [Regex]::Match(
      $disassembly,
      "(?ms)^\s+i\.$escaped\s*.*?(?=^\s+i\.|\z)")
  if (-not $match.Success)
  {
    throw "Function not found in disassembly: $FunctionName"
  }

  $maximum = 0
  foreach ($allocation in [Regex]::Matches(
      $match.Value,
      'SUB\s+sp,sp,#0x([0-9a-fA-F]+)'))
  {
    $bytes = [Convert]::ToInt32($allocation.Groups[1].Value, 16)
    if ($bytes -gt $maximum)
    {
      $maximum = $bytes
    }
  }
  return $maximum
}

$limits = @{
  'LdsReceiver_Poll' = 128
  'Navigation_Poll' = 128
  'NavigationReport_Emit' = 256
  'emit_header' = 256
  'emit_waypoint' = 256
  'PathPlanner_BuildMissionPathLimited' = 384
}

$failed = $false
foreach ($functionName in $limits.Keys)
{
  $bytes = Get-StackAllocation $functionName
  Write-Output "$functionName local_stack=$bytes limit=$($limits[$functionName])"
  if ($bytes -gt $limits[$functionName])
  {
    $failed = $true
  }
}

if ($failed)
{
  Write-Error 'Firmware stack allocation limit exceeded.'
  exit 1
}
