[CmdletBinding()]
param(
    [double]$Scale = 0.5,
    [int]$Samples = 20,
    [double]$CheckTimeout = 30.0,
    [switch]$ManualPicoStart,
    [string]$DeviceSerial,
    [string]$AdbPath = 'C:\adb\adb.exe',
    [string]$PythonPath,
    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

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

function Write-ManualStartInstruction {
    Write-Warning 'Start Kinova PICO Bridge manually on the headset, then continue with the UDP preflight.'
}

function Get-AuthorizedPicoDevices {
    param([Parameter(Mandatory = $true)][string]$AdbPath)
    $lines = & $AdbPath 'devices' '-l' 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw "ADB device listing failed with exit code $LASTEXITCODE."
    }

    $devices = @()
    foreach ($line in $lines) {
        $match = [regex]::Match([string]$line, '^(?<serial>\S+)\s+(?<state>\S+)(?:\s+(?<details>.*))?$')
        if ($match.Success -and $match.Groups['state'].Value -eq 'device') {
            $devices += [pscustomobject]@{
                Serial = $match.Groups['serial'].Value
                IsPico = $match.Groups['details'].Value -match `
                    '(?i)(?:model|product):(?:\S*pico\S*|A9210)(?:\s|$)'
            }
        }
    }
    return @($devices)
}

function Get-PicoSelector {
    param(
        [Parameter(Mandatory = $true)][string]$AdbPath,
        [Parameter(Mandatory = $true)][object[]]$Devices,
        [string]$RequestedSerial
    )
    if (-not [string]::IsNullOrWhiteSpace($RequestedSerial)) {
        $matching = @($Devices | Where-Object { $_.Serial -eq $RequestedSerial })
        if ($matching.Count -ne 1) {
            throw "Requested ADB serial '$RequestedSerial' is unavailable or unauthorized."
        }
        return @('-s', $RequestedSerial)
    }
    if ($Devices.Count -gt 1) {
        throw "Multiple authorized USB devices were found; use -DeviceSerial to choose one."
    }
    if ($Devices.Count -eq 0 -or -not $Devices[0].IsPico) {
        return $null
    }
    $state = & $AdbPath '-d' 'get-state' 2>&1
    if ($LASTEXITCODE -ne 0 -or (@($state) -notcontains 'device')) {
        return $null
    }
    return @('-d')
}

function Start-PicoAppOrExplainManualFallback {
    param(
        [Parameter(Mandatory = $true)][string]$AdbPath,
        [string]$RequestedSerial,
        [switch]$DryRunMode
    )
    $monkeyArguments = @('-p', 'com.yezqin.kinovapicobridge', '-c', 'android.intent.category.LAUNCHER', '1')
    if ($DryRunMode) {
        $selector = if ([string]::IsNullOrWhiteSpace($RequestedSerial)) { @('-d') } else { @('-s', $RequestedSerial) }
        $dryRunLaunchArguments = @(@($selector) + @('shell', 'monkey') + $monkeyArguments)
        Write-CommandLine -Executable $AdbPath -Arguments $dryRunLaunchArguments
        return
    }
    if (-not (Test-Path -LiteralPath $AdbPath -PathType Leaf)) {
        Write-ManualStartInstruction
        return
    }
    try {
        $devices = Get-AuthorizedPicoDevices -AdbPath $AdbPath
        $selector = Get-PicoSelector -AdbPath $AdbPath -Devices $devices -RequestedSerial $RequestedSerial
        if ($null -eq $selector) {
            Write-ManualStartInstruction
            return
        }
        $launchArguments = @(@($selector) + @('shell', 'monkey') + $monkeyArguments)
        & $AdbPath @launchArguments
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "ADB could not start the PICO app (exit code $LASTEXITCODE)."
            Write-ManualStartInstruction
        }
    }
    catch {
        if ($_.Exception.Message -like 'Multiple authorized USB devices*') {
            throw
        }
        Write-Warning ("ADB unavailable or not ready: " + $_.Exception.Message)
        Write-ManualStartInstruction
    }
}

function Write-PreflightGuidance {
    Write-Warning 'PICO UDP preflight failed. Check the same LAN/VLAN, AP isolation, VPN, left-controller wake and tracking, Grip release, and UDP 15031 firewall. No firewall rule was changed.'
}

try {
    if ($Scale -le 0.0 -or [double]::IsNaN($Scale) -or [double]::IsInfinity($Scale)) {
        throw '-Scale must be a positive finite number.'
    }
    if ($Samples -le 0) {
        throw '-Samples must be positive.'
    }
    if ($CheckTimeout -le 0.0 -or [double]::IsNaN($CheckTimeout) -or [double]::IsInfinity($CheckTimeout)) {
        throw '-CheckTimeout must be a positive finite number.'
    }

    $repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    $venvPython = Join-Path $repositoryRoot '.venv\Scripts\python.exe'
    $pythonPath = if (-not [string]::IsNullOrWhiteSpace($PythonPath)) {
        $PythonPath
    } elseif (Test-Path -LiteralPath $venvPython -PathType Leaf) {
        $venvPython
    } else {
        'python.exe'
    }
    $preflightArguments = @('-m', 'kinova_teleop.main', '--input', 'pico-udp', '--check-input', '--samples', "$Samples", '--check-timeout', "$CheckTimeout")
    $launchArguments = @('-m', 'kinova_teleop.main', '--input', 'pico-udp', '--scale', "$Scale")

    if (-not $ManualPicoStart) {
        Start-PicoAppOrExplainManualFallback -AdbPath $adbPath -RequestedSerial $DeviceSerial -DryRunMode:$DryRun
    }
    Write-Output 'PICO UDP discovery/preflight uses port 15031; no fixed host IP is configured.'
    Write-CommandLine -Executable $pythonPath -Arguments $preflightArguments
    Write-CommandLine -Executable $pythonPath -Arguments $launchArguments
    if ($DryRun) {
        exit 0
    }

    Push-Location -LiteralPath $repositoryRoot
    try {
        & $pythonPath @preflightArguments
        $preflightExitCode = $LASTEXITCODE
        if ($preflightExitCode -ne 0) {
            Write-PreflightGuidance
            exit $preflightExitCode
        }
        & $pythonPath @launchArguments
        $launchExitCode = $LASTEXITCODE
        exit $launchExitCode
    }
    finally {
        Pop-Location
    }
}
catch {
    Write-Error $_.Exception.Message
    exit 1
}
