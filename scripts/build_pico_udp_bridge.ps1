[CmdletBinding()]
param(
    [string]$UnityPath,
    [switch]$Install,
    [string]$DeviceSerial,
    [string]$AdbPath = 'C:\adb\adb.exe',
    [string]$BuiltApkPath,
    [string]$ArtifactsPath,
    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

class NativeProcessFailure : System.Exception {
    [int]$ExitCode

    NativeProcessFailure([string]$message, [int]$exitCode) : base($message) {
        $this.ExitCode = $exitCode
    }
}

function Format-Token {
    param([Parameter(Mandatory = $true)][string]$Value)
    if ($Value -match '[\s"]') {
        return '"' + $Value.Replace('"', '\"') + '"'
    }
    return $Value
}

function Write-CommandLine {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$Arguments
    )
    $tokens = @((Format-Token $Executable)) + @($Arguments | ForEach-Object {
        Format-Token ([string]$_)
    })
    Write-Output ('& ' + ($tokens -join ' '))
}

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$Arguments,
        [Parameter(Mandatory = $true)][string]$Description
    )
    & $Executable @Arguments
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw [NativeProcessFailure]::new("$Description failed with exit code $exitCode.", $exitCode)
    }
}

function Invoke-CheckedGui {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$Arguments,
        [Parameter(Mandatory = $true)][string]$Description
    )
    $argumentLine = ($Arguments | ForEach-Object {
        Format-Token ([string]$_)
    }) -join ' '
    $process = Start-Process `
        -FilePath $Executable `
        -ArgumentList $argumentLine `
        -WindowStyle Hidden `
        -Wait `
        -PassThru
    if ($process.ExitCode -ne 0) {
        throw [NativeProcessFailure]::new(
            "$Description failed with exit code $($process.ExitCode).",
            $process.ExitCode
        )
    }
}

function Remove-ExpectedOutputLeaf {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$AllowedDirectory,
        [Parameter(Mandatory = $true)][string]$Description
    )
    $resolvedPath = [System.IO.Path]::GetFullPath($Path)
    $resolvedDirectory = [System.IO.Path]::GetFullPath($AllowedDirectory)
    $directoryPrefix = $resolvedDirectory.TrimEnd(
        [System.IO.Path]::DirectorySeparatorChar,
        [System.IO.Path]::AltDirectorySeparatorChar
    ) + [System.IO.Path]::DirectorySeparatorChar
    if (-not $resolvedPath.StartsWith($directoryPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "$Description output must be inside its configured output directory: $resolvedPath"
    }
    if ([System.IO.Path]::GetFileName($resolvedPath) -eq '') {
        throw "$Description output must name a file: $resolvedPath"
    }
    if (Test-Path -LiteralPath $resolvedPath -PathType Container) {
        throw "$Description output must be a file, not a directory: $resolvedPath"
    }
    if (Test-Path -LiteralPath $resolvedPath -PathType Leaf) {
        Remove-Item -LiteralPath $resolvedPath -Force -ErrorAction Stop
    }
}

function Get-AuthorizedAdbDevices {
    param([Parameter(Mandatory = $true)][string]$AdbPath)
    $lines = & $AdbPath 'devices' '-l' 2>&1
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw [NativeProcessFailure]::new("ADB device listing failed with exit code $exitCode.", $exitCode)
    }

    $devices = @()
    foreach ($line in $lines) {
        $match = [regex]::Match([string]$line, '^(?<serial>\S+)\s+(?<state>\S+)(?:\s+(?<details>.*))?$')
        if ($match.Success -and $match.Groups['state'].Value -eq 'device') {
            $devices += [pscustomobject]@{
                Serial = $match.Groups['serial'].Value
                Details = $match.Groups['details'].Value
            }
        }
    }
    return @($devices)
}

function Get-AdbSelector {
    param(
        [Parameter(Mandatory = $true)][string]$AdbPath,
        [Parameter(Mandatory = $true)][object[]]$Devices,
        [string]$RequestedSerial
    )
    if (-not [string]::IsNullOrWhiteSpace($RequestedSerial)) {
        $matching = @($Devices | Where-Object { $_.Serial -eq $RequestedSerial })
        if ($matching.Count -ne 1) {
            throw "Requested ADB serial '$RequestedSerial' is not an authorized device."
        }
        return @('-s', $RequestedSerial)
    }
    if ($Devices.Count -ne 1) {
        throw "Exactly one authorized physical USB device is required; found $($Devices.Count). Use -DeviceSerial to select one explicitly."
    }
    $state = & $AdbPath '-d' 'get-state' 2>&1
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw [NativeProcessFailure]::new("ADB USB device check failed with exit code $exitCode.", $exitCode)
    }
    if (@($state) -notcontains 'device') {
        throw 'The authorized ADB device is not a single physical USB device.'
    }
    return @('-d')
}

try {
    if ([string]::IsNullOrWhiteSpace($UnityPath)) {
        throw 'Unity executable must be supplied with -UnityPath.'
    }
    $repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    $projectPath = Join-Path $repositoryRoot 'pico_unity_bridge'
    $resolvedArtifactsPath = if ([string]::IsNullOrWhiteSpace($ArtifactsPath)) {
        Join-Path $repositoryRoot 'artifacts'
    } else {
        [System.IO.Path]::GetFullPath($ArtifactsPath)
    }
    $apkPath = Join-Path $resolvedArtifactsPath 'kinova-pico-udp-bridge.apk'
    $builtApkPath = if ([string]::IsNullOrWhiteSpace($BuiltApkPath)) {
        Join-Path $projectPath 'Builds\KinovaPicoBridge-development.apk'
    } else {
        [System.IO.Path]::GetFullPath($BuiltApkPath)
    }
    $builtApkDirectory = [System.IO.Path]::GetDirectoryName($builtApkPath)
    $testResultsPath = Join-Path $resolvedArtifactsPath 'kinova-pico-editmode-results.xml'
    $testLogPath = Join-Path $resolvedArtifactsPath 'kinova-pico-editmode.log'
    $buildLogPath = Join-Path $resolvedArtifactsPath 'kinova-pico-build.log'

    $testArguments = @(
        '-batchmode', '-nographics',
        '-projectPath', $projectPath,
        '-runTests', '-testPlatform', 'EditMode',
        '-testResults', $testResultsPath,
        '-logFile', $testLogPath
    )
    $buildArguments = @(
        '-batchmode', '-nographics',
        '-projectPath', $projectPath,
        '-executeMethod', 'Yezqin.KinovaPico.Editor.BuildKinovaPicoBridge.Build',
        '-logFile', $buildLogPath
    )

    Write-CommandLine -Executable $UnityPath -Arguments $testArguments
    Write-CommandLine -Executable $UnityPath -Arguments $buildArguments
    Write-Output ("APK artifact: " + (Format-Token $apkPath))
    if ($Install) {
        Write-CommandLine -Executable $AdbPath -Arguments @('devices', '-l')
        $dryRunSelector = if ([string]::IsNullOrWhiteSpace($DeviceSerial)) {
            @('-d')
        } else {
            @('-s', $DeviceSerial)
        }
        $dryRunInstallArguments = @(@($dryRunSelector) + @('install', '-r', $apkPath))
        Write-CommandLine -Executable $AdbPath -Arguments $dryRunInstallArguments
    }
    if ($DryRun) {
        exit 0
    }

    if (-not (Test-Path -LiteralPath $UnityPath -PathType Leaf)) {
        throw "Unity executable was not found: $UnityPath"
    }
    New-Item -ItemType Directory -Force -Path $resolvedArtifactsPath | Out-Null
    Remove-ExpectedOutputLeaf -Path $testResultsPath -AllowedDirectory $resolvedArtifactsPath `
        -Description 'Unity EditMode test results'
    Invoke-CheckedGui -Executable $UnityPath -Arguments $testArguments -Description 'Unity EditMode tests'
    if (-not (Test-Path -LiteralPath $testResultsPath -PathType Leaf) -or
        (Get-Item -LiteralPath $testResultsPath).Length -le 0) {
        throw "Unity EditMode test results were not written: $testResultsPath"
    }
    [xml]$testResults = Get-Content -LiteralPath $testResultsPath -Raw
    $testRun = $testResults.'test-run'
    if ($null -eq $testRun -or $testRun.result -ne 'Passed' -or [int]$testRun.failed -ne 0) {
        throw "Unity EditMode tests did not pass; inspect $testResultsPath."
    }

    Remove-ExpectedOutputLeaf -Path $builtApkPath -AllowedDirectory $builtApkDirectory `
        -Description 'Unity APK build'
    Remove-ExpectedOutputLeaf -Path $apkPath -AllowedDirectory $resolvedArtifactsPath `
        -Description 'APK artifact'
    Invoke-CheckedGui -Executable $UnityPath -Arguments $buildArguments -Description 'Unity APK build'
    if (-not (Test-Path -LiteralPath $builtApkPath -PathType Leaf) -or (Get-Item -LiteralPath $builtApkPath).Length -le 0) {
        throw "Unity did not produce a nonempty APK: $builtApkPath"
    }
    Copy-Item -LiteralPath $builtApkPath -Destination $apkPath -Force
    if ((Get-Item -LiteralPath $apkPath).Length -le 0) {
        throw "Artifact APK is empty: $apkPath"
    }

    if ($Install) {
        if (-not (Test-Path -LiteralPath $AdbPath -PathType Leaf)) {
            throw "ADB was not found: $AdbPath"
        }
        $devices = Get-AuthorizedAdbDevices -AdbPath $AdbPath
        $selector = Get-AdbSelector -AdbPath $AdbPath -Devices $devices -RequestedSerial $DeviceSerial
        $installArguments = @(@($selector) + @('install', '-r', $apkPath))
        Invoke-Checked -Executable $AdbPath -Arguments $installArguments -Description 'ADB APK install'
    }
}
catch {
    $exitCode = if ($_.Exception -is [NativeProcessFailure]) {
        $_.Exception.ExitCode
    } else {
        1
    }
    [Console]::Error.WriteLine($_.Exception.Message)
    exit $exitCode
}
