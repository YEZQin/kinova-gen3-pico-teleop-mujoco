[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$ProjectRoot
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($ProjectRoot) -or
    -not (Test-Path -LiteralPath $ProjectRoot -PathType Container)) {
    throw 'ProjectRoot must be an existing directory.'
}

$resolvedProject = (Resolve-Path -LiteralPath $ProjectRoot).Path
$git = (Get-Command git -CommandType Application -ErrorAction Stop |
    Select-Object -First 1).Source

$insideWorktree = & $git -C $resolvedProject rev-parse --is-inside-work-tree 2>$null
if ($LASTEXITCODE -ne 0 -or $insideWorktree -ne 'true') {
    throw 'ProjectRoot must be a Git worktree.'
}

$modulePattern = '^(sitecustomize|usercustomize)(\.py|\.pyc|(\.[A-Za-z0-9_-]+)*\.(pyd|so))$'
$packagePattern = '^__init__(\.py|\.pyc|(\.[A-Za-z0-9_-]+)*\.(pyd|so))$'
$candidates = @(
    Get-ChildItem -LiteralPath $resolvedProject -Force -File | Where-Object {
        $_.Name -match $modulePattern
    }
)

$packageDirectories = Get-ChildItem -LiteralPath $resolvedProject -Force -Directory |
    Where-Object { $_.Name -ieq 'sitecustomize' -or $_.Name -ieq 'usercustomize' }
foreach ($directory in $packageDirectories) {
    $candidates += @(
        Get-ChildItem -LiteralPath $directory.FullName -Force -File | Where-Object {
            $_.Name -match $packagePattern
        }
    )
}

$projectPrefix = $resolvedProject.TrimEnd('\', '/') + [IO.Path]::DirectorySeparatorChar
foreach ($candidate in $candidates) {
    if (-not $candidate.FullName.StartsWith(
            $projectPrefix,
            [StringComparison]::OrdinalIgnoreCase
        )) {
        throw "Python startup hook resolved outside ProjectRoot: $($candidate.FullName)"
    }
    $relativePath = $candidate.FullName.Substring($projectPrefix.Length)
    $tracked = & $git -C $resolvedProject ls-files --cached -- $relativePath
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot inspect Python startup hook: $relativePath"
    }
    if ([string]::IsNullOrWhiteSpace(($tracked -join "`n"))) {
        throw "Untracked Python startup hook is forbidden: $relativePath"
    }
    $dirty = & $git -C $resolvedProject status --porcelain --untracked-files=all -- $relativePath
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot validate Python startup hook: $relativePath"
    }
    if (-not [string]::IsNullOrWhiteSpace(($dirty -join "`n"))) {
        throw "Dirty Python startup hook is forbidden: $relativePath"
    }
}
