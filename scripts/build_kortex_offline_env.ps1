[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$PythonExe,
    [Parameter(Mandatory=$true)][string]$KortexWheel,
    [Parameter(Mandatory=$true)][string]$Wheelhouse,
    [Parameter(Mandatory=$true)][string]$TargetDir
)

$ErrorActionPreference = 'Stop'

function Resolve-RequiredPath([string]$Value, [string]$Label) {
    if (-not (Test-Path -LiteralPath $Value)) {
        throw "$Label does not exist: $Value"
    }
    return (Resolve-Path -LiteralPath $Value).Path
}

$resolvedPython = Resolve-RequiredPath $PythonExe 'PythonExe'
$resolvedKortexWheel = Resolve-RequiredPath $KortexWheel 'KortexWheel'
$resolvedWheelhouse = Resolve-RequiredPath $Wheelhouse 'Wheelhouse'
$resolvedTarget = [IO.Path]::GetFullPath($TargetDir)
if (Test-Path -LiteralPath $resolvedTarget) {
    throw "TargetDir already exists: $resolvedTarget"
}

& $resolvedPython -m venv $resolvedTarget
$venvPython = Join-Path $resolvedTarget 'Scripts/python.exe'
if (-not (Test-Path -LiteralPath $venvPython)) {
    throw "venv Python was not created: $venvPython"
}
& $venvPython -m pip install --no-index --find-links $resolvedWheelhouse $resolvedKortexWheel
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
& $venvPython -m pip install --no-index --find-links $resolvedWheelhouse -e "${repoRoot}[dev]"
$kortexImport = & $venvPython -c "import kortex_api; print(kortex_api.__file__)"
if (-not $kortexImport) {
    throw 'kortex_api import acceptance gate failed'
}
$lockPath = Join-Path $resolvedTarget 'requirements.lock.txt'
& $venvPython -m pip freeze | Set-Content -Encoding UTF8 $lockPath

$wheelHash = (Get-FileHash -LiteralPath $resolvedKortexWheel -Algorithm SHA256).Hash.ToLowerInvariant()
$lockHash = (Get-FileHash -LiteralPath $lockPath -Algorithm SHA256).Hash.ToLowerInvariant()
$repoCommit = (& git -C $repoRoot rev-parse HEAD).Trim()
$pythonVersion = (& $venvPython --version).Trim()
$environment = [ordered]@{
    python_executable = $resolvedPython
    python_version = $pythonVersion
    kortex_wheel = [IO.Path]::GetFileName($resolvedKortexWheel)
    kortex_wheel_sha256 = $wheelHash
    wheelhouse = $resolvedWheelhouse
    repository_commit = $repoCommit
    requirements_lock_sha256 = $lockHash
    kortex_import = ($kortexImport | Select-Object -Last 1).ToString()
}
$environment | ConvertTo-Json -Depth 5 | Set-Content -Encoding UTF8 (Join-Path $resolvedTarget 'environment.json')
Write-Output "Created offline Kortex environment at $resolvedTarget"
