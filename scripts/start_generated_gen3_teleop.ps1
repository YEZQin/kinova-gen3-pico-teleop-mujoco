[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)][string]$Profile,
    [string]$PythonPath
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Resolve-RequiredFile([string]$Value, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Value) -or -not (Test-Path -LiteralPath $Value -PathType Leaf)) { throw "$Label must be an existing file" }
    $item = Get-Item -LiteralPath $Value -Force
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) { throw "$Label must not be a symbolic link or junction" }
    return (Resolve-Path -LiteralPath $Value).Path
}

function Resolve-ProfileFile([string]$Root, [string]$Relative, [string]$Label) {
    if ([string]::IsNullOrWhiteSpace($Relative) -or [IO.Path]::IsPathRooted($Relative) -or $Relative.Contains('..')) { throw "$Label must be a relative filename" }
    $path = Resolve-RequiredFile (Join-Path $Root $Relative) $Label
    if ((Split-Path -Parent $path) -cne $Root) { throw "$Label must stay beside the profile" }
    return $path
}

$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$startupHookGate = Resolve-RequiredFile (Join-Path $PSScriptRoot 'assert_no_untracked_python_startup_hooks.ps1') 'PythonStartupHookGate'
& $startupHookGate -ProjectRoot $projectRoot
$profilePath = Resolve-RequiredFile $Profile 'Profile'
$profileRoot = Split-Path -Parent $profilePath
try { $configuration = Get-Content -LiteralPath $profilePath -Raw | ConvertFrom-Json } catch { throw 'Profile must be strict JSON' }
$expected = @('schema_version','device','robot_ip','robot_user','motion_lease','preflight_report','calibration','workspace_min_m','workspace_max_m','max_linear_speed_m_s','run_id','lease_owner','code_revision','calibration_sha256','driver_sha256','reference_pose_m','workspace_midpoint_m')
if (@($configuration.PSObject.Properties.Name | Sort-Object) -join ',' -cne @($expected | Sort-Object) -join ',' -or $configuration.schema_version -cne '1.0' -or $configuration.device -cne 'gen3') { throw 'Profile schema is invalid' }
$lease = Resolve-ProfileFile $profileRoot $configuration.motion_lease 'MotionLease'
$reviewed = Resolve-ProfileFile $profileRoot $configuration.preflight_report 'PreflightReport'
$calibration = Resolve-ProfileFile $profileRoot $configuration.calibration 'Calibration'
$sha256Pattern = '^[0-9a-f]{64}$'
if ($configuration.code_revision -notmatch '^[0-9a-f]{40}$' -or $configuration.calibration_sha256 -notmatch $sha256Pattern -or $configuration.driver_sha256 -notmatch $sha256Pattern) { throw 'Profile hashes are invalid' }
$git = (Get-Command git -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
$currentRevision = (& $git -C $projectRoot rev-parse HEAD).Trim().ToLowerInvariant()
if ($LASTEXITCODE -ne 0 -or $currentRevision -cne $configuration.code_revision) { throw 'Profile code revision does not match the current checkout' }
if ((Get-FileHash -LiteralPath $calibration -Algorithm SHA256).Hash.ToLowerInvariant() -cne $configuration.calibration_sha256) { throw 'Profile calibration hash does not match the loaded artifact' }
$driverSource = Resolve-RequiredFile (Join-Path $projectRoot 'kinova_teleop/kortex_backend.py') 'DriverSource'
if ((Get-FileHash -LiteralPath $driverSource -Algorithm SHA256).Hash.ToLowerInvariant() -cne $configuration.driver_sha256) { throw 'Profile driver hash does not match the current code' }
$pythonCandidate = $PythonPath
if ([string]::IsNullOrWhiteSpace($pythonCandidate)) { $pythonCandidate = Join-Path $projectRoot '.venv-kortex/Scripts/python.exe' }
$python = Resolve-RequiredFile $pythonCandidate 'PythonPath'
$motionArgs = @('-m','kinova_teleop.main','--backend','kortex','--enable-hardware','--input','pico-udp','--robot-ip',$configuration.robot_ip,'--robot-user',$configuration.robot_user,'--workspace-min') + @($configuration.workspace_min_m | ForEach-Object { $_.ToString([Globalization.CultureInfo]::InvariantCulture) }) + @('--workspace-max') + @($configuration.workspace_max_m | ForEach-Object { $_.ToString([Globalization.CultureInfo]::InvariantCulture) }) + @('--max-linear-speed',$configuration.max_linear_speed_m_s.ToString([Globalization.CultureInfo]::InvariantCulture),'--max-angular-speed-deg','2','--translation-only','--expanded-translation-envelope','--responsive-translation-profile','--recover-stale-input','--stale-timeout','0.2','--control-hz','40','--scale','0.8','--operator-calibration',$calibration,'--motion-lease',$lease,'--preflight-report',$reviewed,'--run-id',$configuration.run_id,'--lease-owner',$configuration.lease_owner)
& $python @($motionArgs + '--validate-motion-package')
if ($LASTEXITCODE -ne 0) { throw "Offline motion package validation failed with exit code $LASTEXITCODE" }
& $python -m kinova_teleop.main --input pico-udp --check-input --samples 10 --check-timeout 15
if ($LASTEXITCODE -ne 0) { throw "PICO input gate failed with exit code $LASTEXITCODE" }
if ((Read-Host -Prompt 'Type HARDWARE-READY to continue') -cne 'HARDWARE-READY') { throw 'Current physical checklist confirmation was not accepted' }
$evidence = Join-Path $profileRoot ("evidence-" + [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ') + '-' + [Guid]::NewGuid().ToString('N') + '.jsonl')
$securePassword = Read-Host -AsSecureString -Prompt 'Kinova password for this child process only'
$passwordBstr = [IntPtr]::Zero
try {
    $passwordBstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($securePassword)
    $env:KINOVA_PASSWORD = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($passwordBstr)
    & $python @($motionArgs + @('--evidence-jsonl',$evidence))
    exit $LASTEXITCODE
} finally {
    Remove-Item Env:KINOVA_PASSWORD -ErrorAction SilentlyContinue
    if ($passwordBstr -ne [IntPtr]::Zero) { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($passwordBstr) }
    $securePassword.Dispose()
}
