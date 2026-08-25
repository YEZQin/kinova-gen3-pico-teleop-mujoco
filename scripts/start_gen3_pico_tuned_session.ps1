[CmdletBinding()]
param(
    [string]$RobotHost,
    [string]$RobotUser = 'admin',
    [Parameter(Mandatory=$true)][string]$OperatorCalibration,
    [Parameter(Mandatory=$true)][double[]]$WorkspaceMin,
    [Parameter(Mandatory=$true)][double[]]$WorkspaceMax,
    [string]$LeaseOwner = 'operator',
    [string]$SessionRoot,
    [double]$Scale = 1.0,
    [double]$MaxLinearSpeed = 0.05,
    [double]$PackageLinearSpeed = 0.02,
    [double]$LinearGain = 1.5,
    [double[]]$TranslationAxisGain = @(-2.0, 1.0, 1.0),
    [double]$GripperBinaryThreshold = 0.9,
    [switch]$ConfirmPhysicalChecks,
    [string]$PythonPath
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Assert-PositiveFinite([double]$Value, [string]$Label) {
    if ([double]::IsNaN($Value) -or [double]::IsInfinity($Value) -or $Value -le 0.0) {
        throw "$Label must be positive and finite"
    }
}

function Assert-WorkspaceBounds([double[]]$Minimum, [double[]]$Maximum) {
    if ($Minimum.Count -ne 3 -or $Maximum.Count -ne 3) {
        throw 'WorkspaceMin and WorkspaceMax must each contain exactly three values'
    }
    $maximumSpan = @(1.2, 1.2, 0.64)
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
        if (($upper - $lower) -gt ($maximumSpan[$axis] + 1e-12)) {
            throw 'Workspace span exceeds the package cap of 1.2 m, 1.2 m, and 0.64 m'
        }
    }
    if ($Minimum[2] -lt 0.0) {
        throw 'WorkspaceMin Z must not be below zero'
    }
}

function Assert-TranslationAxisGain([double[]]$Value) {
    if ($Value.Count -ne 3) {
        throw 'TranslationAxisGain must contain exactly three values'
    }
    foreach ($gain in $Value) {
        if (
            [double]::IsNaN($gain) -or
            [double]::IsInfinity($gain) -or
            $gain -eq 0.0 -or
            [math]::Abs($gain) -gt 2.0
        ) {
            throw 'TranslationAxisGain values must be finite, nonzero, and have absolute value at most 2.0'
        }
    }
}

function Assert-NoReparsePath([string]$Value, [string]$Label) {
    $currentPath = [IO.Path]::GetFullPath($Value)
    while (-not [string]::IsNullOrWhiteSpace($currentPath)) {
        if (Test-Path -LiteralPath $currentPath) {
            $item = Get-Item -LiteralPath $currentPath -Force -ErrorAction Stop
            if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
                throw "$Label must not contain a symbolic link or junction"
            }
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
    Assert-NoReparsePath $fullPath $Label
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        throw "$Label must be an existing regular file"
    }
    return (Resolve-Path -LiteralPath $fullPath).Path
}

function Resolve-OrCreateRegularDirectory([string]$Value, [string]$Label) {
    $fullPath = [IO.Path]::GetFullPath($Value)
    Assert-NoReparsePath $fullPath $Label
    if (Test-Path -LiteralPath $fullPath) {
        if (-not (Test-Path -LiteralPath $fullPath -PathType Container)) {
            throw "$Label must be a directory"
        }
    } else {
        New-Item -ItemType Directory -Path $fullPath | Out-Null
    }
    Assert-NoReparsePath $fullPath $Label
    return (Resolve-Path -LiteralPath $fullPath).Path
}

function ConvertTo-PowerShellLiteral([string]$Value) {
    return "'" + $Value.Replace("'", "''") + "'"
}

if (-not $ConfirmPhysicalChecks) {
    throw '-ConfirmPhysicalChecks is required after inspecting the complete swept workspace, E-stop/Web Stop, observer, cables, fixture, speed level, and load/TCP.'
}
if ([string]::IsNullOrWhiteSpace($RobotHost)) { throw 'RobotHost must not be empty' }
if ($RobotUser -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$') { throw 'RobotUser is invalid' }
if ($LeaseOwner -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$') { throw 'LeaseOwner is invalid' }
Assert-WorkspaceBounds $WorkspaceMin $WorkspaceMax
Assert-PositiveFinite $Scale 'Scale'
if ($Scale -gt 1.0) { throw 'Scale must not exceed 1.0 for the tuned public profile' }
Assert-PositiveFinite $MaxLinearSpeed 'MaxLinearSpeed'
if ($MaxLinearSpeed -gt 0.05) { throw 'MaxLinearSpeed must not exceed 0.05 m/s for the tuned public profile' }
Assert-PositiveFinite $PackageLinearSpeed 'PackageLinearSpeed'
if ($PackageLinearSpeed -gt 0.02) { throw 'PackageLinearSpeed must not exceed 0.02 m/s' }
Assert-PositiveFinite $LinearGain 'LinearGain'
if ($LinearGain -gt 2.0) { throw 'LinearGain must not exceed 2.0' }
Assert-TranslationAxisGain $TranslationAxisGain
if (
    [double]::IsNaN($GripperBinaryThreshold) -or
    [double]::IsInfinity($GripperBinaryThreshold) -or
    $GripperBinaryThreshold -lt 0.0 -or
    $GripperBinaryThreshold -gt 1.0
) {
    throw 'GripperBinaryThreshold must be finite and within [0, 1]'
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$startupHookGate = Resolve-RequiredFile (Join-Path $PSScriptRoot 'assert_no_untracked_python_startup_hooks.ps1') 'PythonStartupHookGate'
& $startupHookGate -ProjectRoot $projectRoot

$pythonCandidate = $PythonPath
if ([string]::IsNullOrWhiteSpace($pythonCandidate)) {
    $pythonCandidate = Join-Path $projectRoot '.venv-kortex/Scripts/python.exe'
}
$python = Resolve-RequiredFile $pythonCandidate 'PythonPath'
$calibrationSource = Resolve-RequiredFile $OperatorCalibration 'OperatorCalibration'
$prepareScript = Resolve-RequiredFile (Join-Path $PSScriptRoot 'prepare_gen3_hardware.ps1') 'PrepareGen3HardwareScript'
$advancedLauncher = Resolve-RequiredFile (Join-Path $PSScriptRoot 'start_gen3_pico_teleop.ps1') 'AdvancedPicoTeleopLauncher'

$sessionRootCandidate = $SessionRoot
if ([string]::IsNullOrWhiteSpace($sessionRootCandidate)) {
    $sessionRootCandidate = Join-Path $projectRoot 'sessions'
}
$sessionRootResolved = Resolve-OrCreateRegularDirectory $sessionRootCandidate 'SessionRoot'
$sessionName = 'gen3-pico-tuned-' + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '-' + ([Guid]::NewGuid().ToString('N').Substring(0, 12))
$sessionDirectory = Join-Path $sessionRootResolved $sessionName
$localConfig = Join-Path $sessionDirectory 'local-config'
if (Test-Path -LiteralPath $sessionDirectory) {
    throw 'Generated session directory already exists'
}
New-Item -ItemType Directory -Path $localConfig | Out-Null
$localConfig = (Resolve-Path -LiteralPath $localConfig).Path

$calibrationTarget = Join-Path $localConfig 'operator-axes.json'
if (Test-Path -LiteralPath $calibrationTarget) {
    throw 'Generated calibration target already exists'
}
Copy-Item -LiteralPath $calibrationSource -Destination $calibrationTarget -ErrorAction Stop

$t0Path = Join-Path $localConfig 't0.json'
Write-Host "Session local-config: $localConfig"
& $prepareScript -RobotIp $RobotHost -RobotUser $RobotUser -Output $t0Path -PythonPath $python

$invariantCulture = [Globalization.CultureInfo]::InvariantCulture
$packageArgs = @(
    '-m', 'kinova_teleop.public_hardware_setup',
    'package',
    '--t0', $t0Path,
    '--calibration', $calibrationTarget,
    '--workspace-min'
) + @($WorkspaceMin | ForEach-Object { $_.ToString('R', $invariantCulture) }) + @(
    '--workspace-max'
) + @($WorkspaceMax | ForEach-Object { $_.ToString('R', $invariantCulture) }) + @(
    '--linear-speed', $PackageLinearSpeed.ToString('R', $invariantCulture),
    '--owner', $LeaseOwner,
    '--output-dir', $localConfig,
    '--workspace-clear',
    '--physical-estop-reachable',
    '--teach-pendant-stop-reachable',
    '--second-observer-present',
    '--cable-slack-checked',
    '--device-fixture-checked',
    '--speed-level-checked',
    '--workspace-bounds-checked',
    '--load-tcp-checked'
)
& $python @packageArgs
if ($LASTEXITCODE -ne 0) {
    throw "Package generation failed with exit code $LASTEXITCODE; session artifacts remain at $localConfig"
}

$profilePath = Join-Path $localConfig 'teleop-profile.json'
$profile = Get-Content -LiteralPath $profilePath -Raw | ConvertFrom-Json
if ([string]::IsNullOrWhiteSpace([string]$profile.run_id)) {
    throw 'Generated teleop profile is missing run_id'
}
if ([string]::IsNullOrWhiteSpace([string]$profile.lease_owner)) {
    throw 'Generated teleop profile is missing lease_owner'
}

$motionLease = Join-Path $localConfig 'motion-lease.json'
$preflightReport = Join-Path $localConfig 'reviewed-preflight.json'
$commandParts = @(
    '&', (ConvertTo-PowerShellLiteral $advancedLauncher),
    '-RobotHost', (ConvertTo-PowerShellLiteral $RobotHost),
    '-RobotUser', (ConvertTo-PowerShellLiteral $RobotUser),
    '-MotionLease', (ConvertTo-PowerShellLiteral $motionLease),
    '-PreflightReport', (ConvertTo-PowerShellLiteral $preflightReport),
    '-OperatorCalibration', (ConvertTo-PowerShellLiteral $calibrationTarget),
    '-RunId', (ConvertTo-PowerShellLiteral ([string]$profile.run_id)),
    '-LeaseOwner', (ConvertTo-PowerShellLiteral ([string]$profile.lease_owner)),
    '-WorkspaceMin', ('([double[]]@(' + (($WorkspaceMin | ForEach-Object { $_.ToString('R', $invariantCulture) }) -join ',') + '))'),
    '-WorkspaceMax', ('([double[]]@(' + (($WorkspaceMax | ForEach-Object { $_.ToString('R', $invariantCulture) }) -join ',') + '))'),
    '-Scale', $Scale.ToString('R', $invariantCulture),
    '-MaxLinearSpeed', $MaxLinearSpeed.ToString('R', $invariantCulture),
    '-GripperBinaryThreshold', $GripperBinaryThreshold.ToString('R', $invariantCulture),
    '-LinearGain', $LinearGain.ToString('R', $invariantCulture),
    '-TranslationAxisGain', ('([double[]]@(' + (($TranslationAxisGain | ForEach-Object { $_.ToString('R', $invariantCulture) }) -join ',') + '))'),
    '-EnableGripper',
    '-PythonPath', (ConvertTo-PowerShellLiteral $python)
)
$advancedCommand = $commandParts -join ' '
& powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -Command $advancedCommand
$advancedExitCode = $LASTEXITCODE
Write-Host "Session local-config: $localConfig"
if ($advancedExitCode -ne 0) {
    throw "Tuned PICO teleop failed with exit code $advancedExitCode; session artifacts remain at $localConfig"
}
Write-Host "Tuned PICO teleop completed. Session local-config: $localConfig"
