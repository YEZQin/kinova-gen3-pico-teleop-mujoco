[CmdletBinding()]
param(
    [string]$PythonExe,
    [string]$ReleaseManifest,
    [string]$DownloadDirectory,
    [string]$VenvDirectory,
    [string]$ApkPath,
    [string]$KortexWheel,
    [switch]$InstallApk,
    [string]$DeviceSerial,
    [switch]$SkipOfflineTests
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Resolve-RequiredFile([string]$Value, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Value) -or
        -not (Test-Path -LiteralPath $Value -PathType Leaf)) {
        throw "$Label must be an existing file: $Value"
    }
    $item = Get-Item -LiteralPath $Value -Force
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "$Label must not be a symbolic link or junction: $Value"
    }
    return (Resolve-Path -LiteralPath $Value).Path
}

function Resolve-OrCreateDirectory([string]$Value, [string]$Label) {
    $resolved = [IO.Path]::GetFullPath($Value)
    if (-not (Test-Path -LiteralPath $resolved)) {
        New-Item -ItemType Directory -Path $resolved -Force | Out-Null
    }
    if (-not (Test-Path -LiteralPath $resolved -PathType Container)) {
        throw "$Label must be a directory: $resolved"
    }
    $item = Get-Item -LiteralPath $resolved -Force
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "$Label must not be a symbolic link or junction: $resolved"
    }
    return (Resolve-Path -LiteralPath $resolved).Path
}

function Resolve-ExistingVenv([string]$Value) {
    $resolved = [IO.Path]::GetFullPath($Value)
    if (-not (Test-Path -LiteralPath $resolved)) {
        return $resolved
    }
    if (-not (Test-Path -LiteralPath $resolved -PathType Container)) {
        throw "VenvDirectory must be a directory: $resolved"
    }
    $venvPython = Join-Path $resolved 'Scripts/python.exe'
    if (-not (Test-Path -LiteralPath $venvPython -PathType Leaf)) {
        throw "VenvDirectory already exists but is not a Python virtual environment: $resolved"
    }
    return (Resolve-Path -LiteralPath $resolved).Path
}

function Get-ManifestAsset([object[]]$Assets, [string]$Extension, [string]$Label) {
    $matches = @($Assets | Where-Object { $_.name -like "*$Extension" })
    if ($matches.Count -ne 1) {
        throw "Release manifest must contain exactly one $Label asset"
    }
    return $matches[0]
}

function Resolve-OptionalAsset([string]$Value, [object]$Asset, [string]$DefaultPath, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Value)) {
        return $DefaultPath
    }
    $resolved = Resolve-RequiredFile $Value $Label
    if ([IO.Path]::GetFileName($resolved) -cne $Asset.name) {
        throw "$Label filename must match the manifest asset: $($Asset.name)"
    }
    return $resolved
}

function Test-AllowedReleaseRedirect([Uri]$Uri) {
    if ($Uri.Scheme -ne 'https') {
        return $false
    }
    # GitHub serves release attachments through these GitHub-controlled hosts.
    $allowedHosts = @(
        'github.com',
        'objects.githubusercontent.com',
        'release-assets.githubusercontent.com',
        'github-releases.githubusercontent.com'
    )
    return $allowedHosts -contains $Uri.Host.ToLowerInvariant()
}

function Resolve-ReleaseAssetDownloadUri(
    [string]$Value,
    [ScriptBlock]$RequestFactory
) {
    $currentUri = New-Object Uri($Value)
    if (-not (Test-AllowedReleaseRedirect $currentUri) -or
        $currentUri.Host -cne 'github.com' -or
        $currentUri.AbsolutePath -notmatch '^/YEZQin/kinova-gen3-pico-teleop-mujoco/releases/download/') {
        throw 'Release asset URL must start at the approved HTTPS GitHub release URL.'
    }
    $maximumRedirects = 5
    for ($redirectCount = 0; $redirectCount -le $maximumRedirects; $redirectCount++) {
        if ($null -eq $RequestFactory) {
            $request = [Net.HttpWebRequest]::Create($currentUri)
        } else {
            $request = & $RequestFactory $currentUri
        }
        $request.Method = 'HEAD'
        $request.AllowAutoRedirect = $false
        $request.Timeout = 30000
        try {
            $response = $request.GetResponse()
        } catch [Net.WebException] {
            $response = $_.Exception.Response
            if ($null -eq $response) {
                throw "Cannot resolve approved release asset URL: $currentUri"
            }
        }
        try {
            $statusCode = [int]$response.StatusCode
            if ($statusCode -lt 300 -or $statusCode -gt 399) {
                if ($statusCode -ge 200 -and $statusCode -lt 300) {
                    return $currentUri.AbsoluteUri
                }
                throw "Release asset URL returned HTTP ${statusCode}: $currentUri"
            }
            $location = $response.Headers['Location']
            if ([string]::IsNullOrWhiteSpace($location)) {
                throw "Release asset redirect has no Location header: $currentUri"
            }
            $nextUri = New-Object Uri($currentUri, $location)
            if (-not (Test-AllowedReleaseRedirect $nextUri)) {
                throw "Release asset redirect leaves the approved HTTPS GitHub hosts: $nextUri"
            }
            $currentUri = $nextUri
        } finally {
            $response.Close()
        }
    }
    throw "Release asset URL exceeded $maximumRedirects approved redirects."
}

if ($env:OS -ne 'Windows_NT') {
    throw 'bootstrap_public_teleop.ps1 must run on Windows.'
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$startupHookGate = Resolve-RequiredFile `
    (Join-Path $PSScriptRoot 'assert_no_untracked_python_startup_hooks.ps1') `
    'PythonStartupHookGate'
& $startupHookGate -ProjectRoot $projectRoot
if ($LASTEXITCODE -ne 0) {
    throw "Python startup hook gate failed with exit code $LASTEXITCODE"
}

if ([string]::IsNullOrWhiteSpace($ReleaseManifest)) {
    $ReleaseManifest = Join-Path $projectRoot 'release/public-release-assets.json'
}
if ([string]::IsNullOrWhiteSpace($DownloadDirectory)) {
    $DownloadDirectory = Join-Path $projectRoot 'downloads'
}
if ([string]::IsNullOrWhiteSpace($VenvDirectory)) {
    $VenvDirectory = Join-Path $projectRoot '.venv-kortex'
}

$pythonCandidate = $PythonExe
if ([string]::IsNullOrWhiteSpace($pythonCandidate)) {
    $pyLauncher = Get-Command py -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -ne $pyLauncher) {
        $launcherCandidate = (& $pyLauncher.Source -3.11 -c 'import sys; print(sys.executable)' |
            Select-Object -Last 1).ToString().Trim()
        if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($launcherCandidate) -and
            (Test-Path -LiteralPath $launcherCandidate -PathType Leaf)) {
            $pythonCandidate = $launcherCandidate
        }
    }
    if ([string]::IsNullOrWhiteSpace($pythonCandidate)) {
        $pythonCandidate = (Get-Command python -CommandType Application -ErrorAction Stop |
            Select-Object -First 1).Source
    }
}
$resolvedPython = Resolve-RequiredFile $pythonCandidate 'PythonExe'
$pythonVersion = (& $resolvedPython --version 2>&1 | Select-Object -Last 1).ToString().Trim()
if ($LASTEXITCODE -ne 0 -or $pythonVersion -notmatch '^Python 3\.11\.\d+') {
    throw "PythonExe must be Python 3.11; received: $pythonVersion"
}

$resolvedManifest = Resolve-RequiredFile $ReleaseManifest 'ReleaseManifest'

$manifestLoadScript = @'
from pathlib import Path
import kinova_teleop.release_assets as release_assets

release_assets.load_release_manifest(Path(__import__('sys').argv[1]))
'@
$manifestLoadArguments = @('-c', $manifestLoadScript, $resolvedManifest)
& $resolvedPython @manifestLoadArguments
if ($LASTEXITCODE -ne 0) {
    throw "Release manifest validation failed with exit code $LASTEXITCODE"
}

$manifest = Get-Content -LiteralPath $resolvedManifest -Raw | ConvertFrom-Json
$assets = @($manifest.assets)
$apkAsset = Get-ManifestAsset $assets '.apk' 'APK'
$kortexAsset = Get-ManifestAsset $assets '.whl' 'Kortex wheel'
$resolvedDownloadDirectory = Resolve-OrCreateDirectory $DownloadDirectory 'DownloadDirectory'

$downloadedPaths = @()
foreach ($asset in $assets) {
    $destination = Join-Path $resolvedDownloadDirectory $asset.name
    if (-not (Test-Path -LiteralPath $destination)) {
        $approvedDownloadUri = Resolve-ReleaseAssetDownloadUri $asset.download_url
        # The resolver follows only the documented, bounded GitHub release asset host allow-list.
        Invoke-WebRequest -Uri $approvedDownloadUri -OutFile $destination `
            -MaximumRedirection 0 -UseBasicParsing
    }
    $downloadedPaths += $destination
}

$assetVerificationScript = @'
from pathlib import Path
import sys
import kinova_teleop.release_assets as release_assets

manifest = release_assets.load_release_manifest(Path(sys.argv[1]))
assets_by_name = {asset.name: asset for asset in manifest.assets}
for raw_path in sys.argv[2:]:
    path = Path(raw_path)
    asset = assets_by_name.get(path.name)
    if asset is None:
        raise ValueError(f"asset path is not declared in the release manifest: {path.name}")
    release_assets.verify_release_asset(path, asset)
'@
$assetVerificationArguments = @('-c', $assetVerificationScript, $resolvedManifest) + $downloadedPaths
& $resolvedPython @assetVerificationArguments
if ($LASTEXITCODE -ne 0) {
    throw "Downloaded release asset verification failed with exit code $LASTEXITCODE"
}

$resolvedApk = Resolve-OptionalAsset $ApkPath $apkAsset `
    (Join-Path $resolvedDownloadDirectory $apkAsset.name) 'ApkPath'
$resolvedKortexWheel = Resolve-OptionalAsset $KortexWheel $kortexAsset `
    (Join-Path $resolvedDownloadDirectory $kortexAsset.name) 'KortexWheel'
$selectedAssetPaths = @($resolvedApk, $resolvedKortexWheel)
$selectedAssetVerificationArguments = @('-c', $assetVerificationScript, $resolvedManifest) + $selectedAssetPaths
& $resolvedPython @selectedAssetVerificationArguments
if ($LASTEXITCODE -ne 0) {
    throw "Selected release asset verification failed with exit code $LASTEXITCODE"
}

$resolvedVenvDirectory = Resolve-ExistingVenv $VenvDirectory
if (-not (Test-Path -LiteralPath $resolvedVenvDirectory)) {
    $venvCreateArguments = @('-m', 'venv', $resolvedVenvDirectory)
    & $resolvedPython @venvCreateArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Virtual environment creation failed with exit code $LASTEXITCODE"
    }
}
$venvPython = Resolve-RequiredFile (Join-Path $resolvedVenvDirectory 'Scripts/python.exe') 'VenvPython'

$kortexInstallArguments = @('-m', 'pip', 'install', $resolvedKortexWheel)
& $venvPython @kortexInstallArguments
if ($LASTEXITCODE -ne 0) {
    throw "Kortex wheel installation failed with exit code $LASTEXITCODE"
}
$protobufInstallArguments = @('-m', 'pip', 'install', 'protobuf==3.20.0')
& $venvPython @protobufInstallArguments
if ($LASTEXITCODE -ne 0) {
    throw "Pinned protobuf installation failed with exit code $LASTEXITCODE"
}
$projectInstallArguments = @('-m', 'pip', 'install', '-e', "$projectRoot[dev]")
& $venvPython @projectInstallArguments
if ($LASTEXITCODE -ne 0) {
    throw "Project dependency installation failed with exit code $LASTEXITCODE"
}

$importCheckArguments = @('-c', 'import kortex_api; print(kortex_api.__file__)')
& $venvPython @importCheckArguments
if ($LASTEXITCODE -ne 0) {
    throw "kortex_api import-only check failed with exit code $LASTEXITCODE"
}

if (-not $SkipOfflineTests) {
    $pytestArguments = @('-m', 'pytest', 'tests/test_cli.py', '-q', '-p', 'no:cacheprovider')
    & $venvPython @pytestArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Focused pytest failed with exit code $LASTEXITCODE"
    }
    $dryRunArguments = @(
        '-m', 'kinova_teleop.main', '--dry-run', '--headless', '--steps', '2000'
    )
    & $venvPython @dryRunArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Headless dry run failed with exit code $LASTEXITCODE"
    }
}

$installedDeviceSerial = $null
if ($InstallApk) {
    $adb = (Get-Command adb -CommandType Application -ErrorAction Stop |
        Select-Object -First 1).Source
    # adb devices is queried only after the explicit APK-install switch.
    $adbLines = @(& $adb devices)
    if ($LASTEXITCODE -ne 0) {
        throw "ADB device discovery failed with exit code $LASTEXITCODE"
    }
    $deviceSerials = @(
        $adbLines | ForEach-Object {
            if ($_ -match '^(?<serial>\S+)\s+device\s*$') {
                $Matches['serial']
            }
        }
    )
    if ($deviceSerials.Count -ne 1) {
        throw 'APK installation requires exactly one authorized ADB device.'
    }
    $installedDeviceSerial = $deviceSerials[0]
    if (-not [string]::IsNullOrWhiteSpace($DeviceSerial) -and
        $installedDeviceSerial -cne $DeviceSerial) {
        throw 'The requested DeviceSerial is not the exactly one authorized ADB device.'
    }
    # The sole device mutation is: adb -s SERIAL install -r APK.
    $adbInstallArguments = @('-s', $installedDeviceSerial, 'install', '-r', $resolvedApk)
    & $adb @adbInstallArguments
    if ($LASTEXITCODE -ne 0) {
        throw "APK installation failed with exit code $LASTEXITCODE"
    }
}

$git = (Get-Command git -CommandType Application -ErrorAction Stop |
    Select-Object -First 1).Source
$repositoryCommit = (& $git -C $projectRoot rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($repositoryCommit)) {
    throw 'Unable to determine the repository commit for the installation receipt.'
}
$venvVersion = (& $venvPython --version 2>&1 | Select-Object -Last 1).ToString().Trim()
$receiptDirectory = Resolve-OrCreateDirectory (Join-Path $projectRoot 'local-config') 'ReceiptDirectory'
$receipt = [ordered]@{
    schema_version = '1.0'
    bootstrap_python_version = $pythonVersion
    venv_python_version = $venvVersion
    repository_commit = $repositoryCommit
    asset_sha256 = [ordered]@{
        apk = (Get-FileHash -LiteralPath $resolvedApk -Algorithm SHA256).Hash.ToLowerInvariant()
        kortex_wheel = (Get-FileHash -LiteralPath $resolvedKortexWheel -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    apk_installed = [bool]$InstallApk
    installed_device_serial = $installedDeviceSerial
}
$receiptPath = Join-Path $receiptDirectory 'install-receipt.json'
$receiptJson = $receipt | ConvertTo-Json -Depth 5
[IO.File]::WriteAllText(
    $receiptPath,
    $receiptJson + [Environment]::NewLine,
    (New-Object Text.UTF8Encoding($false))
)
Write-Output "Public PICO Kortex bootstrap completed. Receipt: $receiptPath"
