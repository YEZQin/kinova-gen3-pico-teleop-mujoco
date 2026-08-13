[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$RobotIp,
    [string]$RobotUser = 'admin',
    [string]$Output = 'local-config/t0.json',
    [string]$PythonPath
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Resolve-RequiredFile([string]$Value, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Value) -or -not (Test-Path -LiteralPath $Value -PathType Leaf)) {
        throw "$Label must be an existing regular file"
    }
    $item = Get-Item -LiteralPath $Value -Force
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "$Label must not be a symbolic link or junction"
    }
    return (Resolve-Path -LiteralPath $Value).Path
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$startupHookGate = Resolve-RequiredFile (Join-Path $PSScriptRoot 'assert_no_untracked_python_startup_hooks.ps1') 'PythonStartupHookGate'
& $startupHookGate -ProjectRoot $projectRoot

$pythonCandidate = $PythonPath
if ([string]::IsNullOrWhiteSpace($pythonCandidate)) {
    $pythonCandidate = Join-Path $projectRoot '.venv-kortex/Scripts/python.exe'
}
$python = Resolve-RequiredFile $pythonCandidate 'PythonPath'

$securePassword = Read-Host -AsSecureString -Prompt 'Kinova password for read-only T0 only'
$passwordBstr = [IntPtr]::Zero
try {
    $passwordBstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($securePassword)
    $env:KINOVA_PASSWORD = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($passwordBstr)
    & $python -m kinova_teleop.public_hardware_setup t0 --robot-ip $RobotIp --robot-user $RobotUser --output $Output
    if ($LASTEXITCODE -ne 0) { throw "Read-only T0 failed with exit code $LASTEXITCODE" }
} finally {
    Remove-Item Env:KINOVA_PASSWORD -ErrorAction SilentlyContinue
    if ($passwordBstr -ne [IntPtr]::Zero) { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($passwordBstr) }
    $securePassword.Dispose()
}
