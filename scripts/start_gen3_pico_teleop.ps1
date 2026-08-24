[CmdletBinding()]
param(
    [string]$RobotHost = '192.168.1.10',
    [string]$RobotUser = 'admin',
    [double]$Scale = 1.0,
    [double]$MaxLinearSpeed = 0.05,
    [Parameter(Mandatory=$true)][double[]]$WorkspaceMin,
    [Parameter(Mandatory=$true)][double[]]$WorkspaceMax,
    [Parameter(Mandatory=$true)][string]$MotionLease,
    [Parameter(Mandatory=$true)][string]$PreflightReport,
    [Parameter(Mandatory=$true)][string]$OperatorCalibration,
    [Parameter(Mandatory=$true)][switch]$EnableGripper,
    [Parameter(Mandatory=$true)][string]$RunId,
    [Parameter(Mandatory=$true)][string]$LeaseOwner,
    [double]$GripperTriggerMin = 0.0,
    [double]$GripperTriggerMax = 1.0,
    [double]$GripperBinaryThreshold = 0.9,
    [switch]$InvertTranslation,
    [double]$LinearGain = 1.0,
    [double[]]$TranslationAxisGain = @(1.0, 1.0, 1.0),
    [string]$PythonPath
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Assert-NoReparsePath([string]$Value, [string]$Label) {
    $currentPath = [IO.Path]::GetFullPath($Value)
    while (-not [string]::IsNullOrWhiteSpace($currentPath)) {
        $item = Get-Item -LiteralPath $currentPath -Force -ErrorAction Stop
        if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
            throw "$Label must not contain a symbolic link or junction"
        }
        $parent = [IO.Directory]::GetParent($currentPath)
        if ($null -eq $parent) {
            break
        }
        $currentPath = $parent.FullName
    }
}

function Resolve-RequiredFile([string]$Value, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Value)) {
        throw "$Label must be an existing regular file"
    }
    $fullPath = [IO.Path]::GetFullPath($Value)
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        throw "$Label must be an existing regular file"
    }
    Assert-NoReparsePath $fullPath $Label
    return (Resolve-Path -LiteralPath $fullPath).Path
}

function Resolve-RequiredDirectory([string]$Value, [string]$Label) {
    $fullPath = [IO.Path]::GetFullPath($Value)
    if (-not (Test-Path -LiteralPath $fullPath -PathType Container)) {
        throw "$Label must be an existing directory"
    }
    Assert-NoReparsePath $fullPath $Label
    return (Resolve-Path -LiteralPath $fullPath).Path
}

function Assert-PositiveFinite([double]$Value, [string]$Label) {
    if ([double]::IsNaN($Value) -or [double]::IsInfinity($Value) -or $Value -le 0.0) {
        throw "$Label must be positive and finite"
    }
}

function Assert-WorkspaceBounds([double[]]$Minimum, [double[]]$Maximum) {
    if ($Minimum.Count -ne 3 -or $Maximum.Count -ne 3) {
        throw 'WorkspaceMin and WorkspaceMax must each contain exactly three values'
    }
    for ($axis = 0; $axis -lt 3; $axis++) {
        $lower = $Minimum[$axis]
        $upper = $Maximum[$axis]
        if (
            [double]::IsNaN($lower) -or
            [double]::IsInfinity($lower) -or
            [double]::IsNaN($upper) -or
            [double]::IsInfinity($upper)
        ) {
            throw 'WorkspaceMin and WorkspaceMax values must be finite'
        }
        if ($lower -ge $upper) {
            throw 'WorkspaceMin must be strictly less than WorkspaceMax on every axis'
        }
    }
}

function Assert-GripperTriggerRange([double]$Minimum, [double]$Maximum) {
    if (
        [double]::IsNaN($Minimum) -or
        [double]::IsInfinity($Minimum) -or
        [double]::IsNaN($Maximum) -or
        [double]::IsInfinity($Maximum) -or
        $Minimum -lt 0.0 -or
        $Maximum -gt 1.0 -or
        $Minimum -ge $Maximum
    ) {
        throw 'GripperTriggerMin and GripperTriggerMax must be finite, ordered, and within [0, 1]'
    }
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$startupHookGate = Resolve-RequiredFile (Join-Path $PSScriptRoot 'assert_no_untracked_python_startup_hooks.ps1') 'PythonStartupHookGate'
& $startupHookGate -ProjectRoot $projectRoot

$picoHost = '0.0.0.0'
$picoPort = 15031
$controlHz = 40.0
$staleTimeout = 0.2
$picoCheckTimeout = 15.0
$picoCheckSamples = 10

if ([string]::IsNullOrWhiteSpace($RobotHost)) { throw 'RobotHost must not be empty' }
if ($RobotUser -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$') { throw 'RobotUser is invalid' }
if (-not $EnableGripper) { throw 'EnableGripper is required for advanced PICO teleoperation' }
Assert-PositiveFinite $Scale 'Scale'
Assert-PositiveFinite $MaxLinearSpeed 'MaxLinearSpeed'
Assert-WorkspaceBounds $WorkspaceMin $WorkspaceMax
Assert-GripperTriggerRange $GripperTriggerMin $GripperTriggerMax
if (
    [double]::IsNaN($GripperBinaryThreshold) -or
    [double]::IsInfinity($GripperBinaryThreshold) -or
    $GripperBinaryThreshold -lt 0.0 -or
    $GripperBinaryThreshold -gt 1.0
) {
    throw 'GripperBinaryThreshold must be finite and within [0, 1]'
}
Assert-PositiveFinite $LinearGain 'LinearGain'
if ($LinearGain -gt 2.0) { throw 'LinearGain must not exceed 2.0' }
if ($TranslationAxisGain.Count -ne 3) {
    throw 'TranslationAxisGain must contain exactly three values'
}
foreach ($gain in $TranslationAxisGain) {
    if (
        [double]::IsNaN($gain) -or
        [double]::IsInfinity($gain) -or
        $gain -eq 0.0 -or
        [math]::Abs($gain) -gt 2.0
    ) {
        throw 'TranslationAxisGain values must be finite, nonzero, and have absolute value at most 2.0'
    }
}

$pythonCandidate = $PythonPath
if ([string]::IsNullOrWhiteSpace($pythonCandidate)) {
    $pythonCandidate = Join-Path $projectRoot '.venv-kortex/Scripts/python.exe'
}
$python = Resolve-RequiredFile $pythonCandidate 'PythonPath'
$lease = Resolve-RequiredFile $MotionLease 'MotionLease'
$preflight = Resolve-RequiredFile $PreflightReport 'PreflightReport'
$calibration = Resolve-RequiredFile $OperatorCalibration 'OperatorCalibration'

$resultsDir = Join-Path $projectRoot 'results'
if (-not (Test-Path -LiteralPath $resultsDir -PathType Container)) {
    New-Item -ItemType Directory -Path $resultsDir | Out-Null
}
$resultsDir = Resolve-RequiredDirectory $resultsDir 'ResultsDirectory'
$evidence = Join-Path $resultsDir ("evidence-gen3-pico-advanced-" + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '-' + [Guid]::NewGuid().ToString('N') + '.jsonl')
if (Test-Path -LiteralPath $evidence) { throw 'Evidence path must be absent before motion' }

$invariantCulture = [Globalization.CultureInfo]::InvariantCulture
$motionArgs = @(
    '-m', 'kinova_teleop.main',
    '--backend', 'kortex',
    '--enable-hardware',
    '--advanced-pico-teleop',
    '--input', 'pico-udp',
    '--pico-host', $picoHost,
    '--pico-port', $picoPort.ToString($invariantCulture),
    '--robot-ip', $RobotHost,
    '--robot-user', $RobotUser,
    '--translation-only',
    '--responsive-translation-profile',
    '--recover-stale-input',
    '--gripper',
    '--gripper-trigger-min', $GripperTriggerMin.ToString('R', $invariantCulture),
    '--gripper-trigger-max', $GripperTriggerMax.ToString('R', $invariantCulture),
    '--gripper-binary-threshold', $GripperBinaryThreshold.ToString('R', $invariantCulture),
    '--linear-gain', $LinearGain.ToString('R', $invariantCulture),
    '--translation-axis-gain'
) + @($TranslationAxisGain | ForEach-Object {
    $_.ToString('R', $invariantCulture)
}) + @(
    '--control-hz', $controlHz.ToString('R', $invariantCulture),
    '--stale-timeout', $staleTimeout.ToString('R', $invariantCulture),
    '--scale', $Scale.ToString('R', $invariantCulture),
    '--max-linear-speed', $MaxLinearSpeed.ToString('R', $invariantCulture),
    '--max-angular-speed-deg', '2',
    '--workspace-min'
) + @($WorkspaceMin | ForEach-Object { $_.ToString('R', $invariantCulture) }) + @(
    '--workspace-max'
) + @($WorkspaceMax | ForEach-Object { $_.ToString('R', $invariantCulture) }) + @(
    '--motion-lease', $lease,
    '--run-id', $RunId,
    '--lease-owner', $LeaseOwner,
    '--preflight-report', $preflight,
    '--operator-calibration', $calibration,
    '--evidence-jsonl', $evidence
)
if ($InvertTranslation) {
    $motionArgs += '--invert-translation'
}

Remove-Item Env:KINOVA_PASSWORD -ErrorAction SilentlyContinue
$validationArgs = $motionArgs + '--validate-motion-package'
& $python @validationArgs
$offlineExitCode = $LASTEXITCODE
if ($offlineExitCode -ne 0) {
    throw "Offline motion package validation failed with exit code $offlineExitCode"
}

$picoV2GateProgram = @'
import math
import sys
import time

from kinova_teleop.main import create_pico_udp_input

source = create_pico_udp_input(
    host=sys.argv[1],
    port=int(sys.argv[2]),
    stale_timeout=float(sys.argv[3]),
    allow_stale_source_handoff=False,
)
required_samples = int(sys.argv[4])
deadline = time.monotonic() + float(sys.argv[5])
last_timestamp = None
read_count = 0
try:
    while read_count < required_samples:
        if time.monotonic() >= deadline:
            raise RuntimeError('timed out waiting for fresh PICO V2 Trigger samples')
        sample = source.read()
        if not sample.valid:
            time.sleep(0.01)
            continue
        if last_timestamp is not None and sample.timestamp_ns <= last_timestamp:
            time.sleep(0.01)
            continue
        if not sample.trigger_available or not math.isfinite(sample.trigger):
            raise RuntimeError('PICO V2 Trigger capability is required')
        if not sample.grip < 0.8:
            raise RuntimeError('Grip must remain released below 0.8 throughout the PICO input gate')
        last_timestamp = sample.timestamp_ns
        read_count += 1
finally:
    source.close()
'@
$picoGateArgs = @(
    '-c', $picoV2GateProgram,
    $picoHost,
    $picoPort.ToString($invariantCulture),
    $staleTimeout.ToString('R', $invariantCulture),
    $picoCheckSamples.ToString($invariantCulture),
    $picoCheckTimeout.ToString('R', $invariantCulture)
)
& $python @picoGateArgs
$picoExitCode = $LASTEXITCODE
if ($picoExitCode -ne 0) {
    throw "PICO V2 Trigger input gate failed with exit code $picoExitCode"
}

$securePassword = Read-Host -AsSecureString -Prompt 'Kinova password for this child process only'
$passwordBstr = [IntPtr]::Zero
$motionExitCode = 1
try {
    $passwordBstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($securePassword)
    $env:KINOVA_PASSWORD = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($passwordBstr)
    & $python @motionArgs
    $motionExitCode = $LASTEXITCODE
} finally {
    Remove-Item Env:KINOVA_PASSWORD -ErrorAction SilentlyContinue
    if ($passwordBstr -ne [IntPtr]::Zero) {
        [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($passwordBstr)
    }
    if ($null -ne $securePassword) {
        $securePassword.Dispose()
    }
}

Write-Host "Evidence path for this trial: $evidence"
if ($motionExitCode -ne 0) {
    Write-Error "PICO advanced teleop child failed with exit code $motionExitCode" -ErrorAction Continue
    $global:LASTEXITCODE = $motionExitCode
    $host.SetShouldExit($motionExitCode)
    return
}
