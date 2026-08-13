[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][double[]]$WorkspaceMin,
    [Parameter(Mandatory=$true)][double[]]$WorkspaceMax,
    [Parameter(Mandatory=$true)][string]$MotionLease,
    [Parameter(Mandatory=$true)][string]$PreflightReport,
    [string]$EvidenceJsonl,
    [string]$RobotIp = '192.168.1.10',
    [string]$RobotUser = 'admin',
    [string]$PythonPath
)

$ErrorActionPreference = 'Stop'

function Assert-ThreeFiniteValues([double[]]$Values, [string]$Label) {
    if ($Values.Count -ne 3) {
        throw "$Label must contain exactly three values"
    }
    foreach ($value in $Values) {
        if ([double]::IsNaN($value) -or [double]::IsInfinity($value)) {
            throw "$Label values must be finite"
        }
    }
}

function Resolve-RequiredFile([string]$Value, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Value) -or -not (Test-Path -LiteralPath $Value -PathType Leaf)) {
        throw "$Label must be an existing file"
    }
    $sourceItem = Get-Item -LiteralPath $Value -Force
    if (($sourceItem.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "$Label must not be a symbolic link or junction"
    }
    return (Resolve-Path -LiteralPath $Value).Path
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$startupHookGate = Resolve-RequiredFile `
    (Join-Path $PSScriptRoot 'assert_no_untracked_python_startup_hooks.ps1') `
    'PythonStartupHookGate'
& $startupHookGate -ProjectRoot $projectRoot

Assert-ThreeFiniteValues $WorkspaceMin 'WorkspaceMin'
Assert-ThreeFiniteValues $WorkspaceMax 'WorkspaceMax'
$resolvedLease = Resolve-RequiredFile $MotionLease 'MotionLease'
$resolvedPreflight = Resolve-RequiredFile $PreflightReport 'PreflightReport'
if (-not (Test-Path -LiteralPath $resolvedLease -PathType Leaf)) {
    throw 'MotionLease must resolve to a file'
}
if (-not (Test-Path -LiteralPath $resolvedPreflight -PathType Leaf)) {
    throw 'PreflightReport must resolve to a file'
}

$pythonCandidate = $PythonPath
if ([string]::IsNullOrWhiteSpace($pythonCandidate)) {
    $pythonCandidate = (Get-Command python -CommandType Application -ErrorAction Stop).Source
}
$resolvedPython = Resolve-RequiredFile $pythonCandidate 'PythonPath'

$launchArguments = @(
    '-m', 'kinova_teleop.main',
    '--backend', 'kortex', '--enable-hardware',
    '--input', 'pico-udp',
    '--robot-ip', $RobotIp, '--robot-user', $RobotUser,
    '--control-hz', '40', '--scale', '0.25', '--stale-timeout', '0.2',
    '--max-linear-speed', '0.005', '--max-angular-speed-deg', '2',
    '--workspace-min'
) + @($WorkspaceMin | ForEach-Object { $_.ToString('R', [Globalization.CultureInfo]::InvariantCulture) }) + @(
    '--workspace-max'
) + @($WorkspaceMax | ForEach-Object { $_.ToString('R', [Globalization.CultureInfo]::InvariantCulture) }) + @(
    '--motion-lease', $resolvedLease,
    '--preflight-report', $resolvedPreflight
)

if (-not [string]::IsNullOrWhiteSpace($EvidenceJsonl)) {
    $resolvedEvidence = Resolve-RequiredFile $EvidenceJsonl 'EvidenceJsonl'
    $launchArguments += @('--evidence-jsonl', $resolvedEvidence)
}

& $resolvedPython @launchArguments
exit $LASTEXITCODE
