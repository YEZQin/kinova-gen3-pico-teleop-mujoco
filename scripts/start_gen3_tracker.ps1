[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$TrackerSerial,
    [string]$RobotHost,
    [string]$RobotUser = 'admin',
    [double[]]$WorkspaceMin,
    [double[]]$WorkspaceMax,
    [switch]$WideWorkspace,
    [double]$Scale = 1.0,
    [double]$MaxLinearSpeed = 0.05,
    [double]$Kp = 3.3,
    [double]$Ki = 0.15,
    [double]$Kd = 0.02,
    [double[]]$Rotation,
    [switch]$CheckInput,
    [switch]$EnableHardware,
    [string]$PythonPath,
    [string]$SdkDirectory
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if ([string]::IsNullOrWhiteSpace($PythonPath)) {
    $PythonPath = Join-Path $projectRoot '.venv-kortex\Scripts\python.exe'
    if (-not (Test-Path -LiteralPath $PythonPath)) {
        $PythonPath = Join-Path $projectRoot '..\kortex-merge\.venv-kortex\Scripts\python.exe'
    }
}
if (-not $SdkDirectory) {
    $builtSdk = Join-Path $projectRoot '.local-deps\tracker-binding\venv\Lib\site-packages'
    if (Test-Path -LiteralPath $builtSdk -PathType Container) { $SdkDirectory = $builtSdk }
}
if (-not (Test-Path -LiteralPath $PythonPath -PathType Leaf)) {
    throw 'PythonPath must identify the Python 3.11 environment with Kortex and XRoboToolkit installed'
}
$python = (Resolve-Path -LiteralPath $PythonPath).Path
if ($CheckInput -and $EnableHardware) { throw 'Choose CheckInput or EnableHardware, not both' }
if ($WideWorkspace) {
    if ($PSBoundParameters.ContainsKey('WorkspaceMin') -or $PSBoundParameters.ContainsKey('WorkspaceMax')) {
        throw 'WideWorkspace cannot be combined with WorkspaceMin or WorkspaceMax'
    }
    $WorkspaceMin = [double[]]@(-10, -10, 0)
    $WorkspaceMax = [double[]]@(10, 10, 10)
}
$culture = [Globalization.CultureInfo]::InvariantCulture
$arguments = @('-m', 'kinova_teleop.tracker_main', '--tracker-serial', $TrackerSerial,
    '--scale', $Scale.ToString('R', $culture),
    '--max-linear-speed', $MaxLinearSpeed.ToString('R', $culture),
    '--kp', $Kp.ToString('R', $culture), '--ki', $Ki.ToString('R', $culture),
    '--kd', $Kd.ToString('R', $culture))
if ($CheckInput) { $arguments += '--check-input' }
elseif ($EnableHardware) {
    if ([string]::IsNullOrWhiteSpace($RobotHost) -or $WorkspaceMin.Count -ne 3 -or $WorkspaceMax.Count -ne 3) {
        throw 'Hardware requires RobotHost, WorkspaceMin and WorkspaceMax (absolute base coordinates in metres)'
    }
    $arguments += @('--enable-hardware', '--robot-ip', $RobotHost, '--robot-user', $RobotUser, '--workspace-min')
    $arguments += @($WorkspaceMin | ForEach-Object { $_.ToString('R', $culture) })
    $arguments += '--workspace-max'
    $arguments += @($WorkspaceMax | ForEach-Object { $_.ToString('R', $culture) })
} else { $arguments += '--dry-run' }
if ($Rotation) {
    if ($Rotation.Count -ne 9) { throw 'Rotation must contain nine row-major values' }
    $arguments += '--rotation'
    $arguments += @($Rotation | ForEach-Object { $_.ToString('R', $culture) })
}
$oldPythonPath = $env:PYTHONPATH
try {
    # PYTHONPATH supplies this checkout and an optional locally built extension.
    $env:PYTHONPATH = $projectRoot
    if ($SdkDirectory) { $env:PYTHONPATH += ';' + (Resolve-Path -LiteralPath $SdkDirectory).Path }
    Push-Location $projectRoot
    try { & $python @arguments; $runCode = $LASTEXITCODE } finally { Pop-Location }
} finally { $env:PYTHONPATH = $oldPythonPath }
if ($runCode -ne 0) { throw "Tracker program exited with code $runCode" }
