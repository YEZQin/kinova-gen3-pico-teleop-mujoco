[CmdletBinding()]
param(
    [string]$OutputPath = 'local-config/operator-axes.json',
    [ValidateRange(1, 10000)][int]$SamplesPerPose = 20,
    [string]$PythonPath
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$localConfigRoot = Join-Path $projectRoot 'local-config'
if (-not (Test-Path -LiteralPath $localConfigRoot -PathType Container)) {
    New-Item -ItemType Directory -Path $localConfigRoot | Out-Null
}
$resolvedConfigRoot = (Resolve-Path -LiteralPath $localConfigRoot).Path
$candidateOutput = if ([IO.Path]::IsPathFullyQualified($OutputPath)) {
    [IO.Path]::GetFullPath($OutputPath)
} else {
    [IO.Path]::GetFullPath((Join-Path $projectRoot $OutputPath))
}
$allowedPrefix = $resolvedConfigRoot.TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
if (-not $candidateOutput.StartsWith($allowedPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'OutputPath must be beneath local-config.'
}

$venvPython = Join-Path $projectRoot '.venv-kortex\Scripts\python.exe'
$python = if (-not [string]::IsNullOrWhiteSpace($PythonPath)) {
    $PythonPath
} elseif (Test-Path -LiteralPath $venvPython -PathType Leaf) {
    $venvPython
} else {
    throw 'Expected .venv-kortex\Scripts\python.exe; use -PythonPath only for an explicit local interpreter.'
}

Write-Output 'This captures PICO input only; it never connects to Kortex.'
Push-Location -LiteralPath $projectRoot
try {
    & $python -m kinova_teleop.calibration_capture `
        --output $candidateOutput --samples-per-pose $SamplesPerPose
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
