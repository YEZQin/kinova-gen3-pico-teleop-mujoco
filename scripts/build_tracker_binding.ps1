[CmdletBinding()]
param(
    [string]$Python = '',
    [string]$Source = '',
    [string]$Sdk = ''
)

# Build/import only: never calls SDK init, starts services, or connects devices.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot '..\..'))
if (-not $Python) { $Python = Join-Path $projectRoot '..\kortex-merge\.venv-kortex\Scripts\python.exe' }
if (-not $Source) { $Source = Join-Path $repoRoot '.local-deps\XRoboToolkit-PC-Service-Pybind' }
if (-not $Sdk) { $Sdk = Join-Path $repoRoot '.local-deps\XRoboToolkit-PC-Service-Windows\v1.0.0\SDK\x64' }
$Python = (Resolve-Path -LiteralPath $Python).Path
$Source = (Resolve-Path -LiteralPath $Source).Path
$Sdk = (Resolve-Path -LiteralPath $Sdk).Path
foreach ($required in @((Join-Path $Source 'bindings\py_bindings.cpp'), (Join-Path $Source 'include\nlohmann\json.hpp'), (Join-Path $Sdk 'PXREARobotSDK.lib'), (Join-Path $Sdk 'PXREARobotSDK.dll'))) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) { throw "Missing prerequisite: $required" }
}
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Install Visual Studio 2022 C++ build tools (x64) and Windows SDK.' }
$visualStudio = & $vswhere -latest -version '[17.0,18.0)' -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $visualStudio) { throw 'Visual Studio 2022 x64 C++ tools are missing.' }
& $Python -c "import sys,struct; assert sys.version_info[:2] == (3,11) and struct.calcsize('P') == 8, 'Python 3.11 x64 required'"
if ($LASTEXITCODE -ne 0) { throw 'Unsupported Python interpreter.' }

$outputRoot = Join-Path $projectRoot '.local-deps\tracker-binding'
# Refuse links/junctions before creating or reusing managed build paths.
foreach ($candidate in @((Join-Path $projectRoot '.local-deps'), $outputRoot)) {
    if ((Test-Path -LiteralPath $candidate) -and ((Get-Item -LiteralPath $candidate).Attributes -band [IO.FileAttributes]::ReparsePoint)) { throw "Refusing linked output: $candidate" }
}
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
$venv = Join-Path $outputRoot 'venv'
$buildPython = Join-Path $venv 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $buildPython)) {
    & $Python -m venv $venv
    if ($LASTEXITCODE -ne 0) { throw 'Isolated venv creation failed.' }
}
& $buildPython -m pip install 'cmake==3.31.6' 'pybind11==2.13.6'
if ($LASTEXITCODE -ne 0) { throw 'Build dependency installation failed.' }
$cmake = Join-Path $venv 'Scripts\cmake.exe'
$pybindCmake = & $buildPython -m pybind11 --cmakedir
if ($LASTEXITCODE -ne 0) { throw 'pybind11 discovery failed.' }
$cmakeSource = Join-Path $outputRoot 'cmake-source'
New-Item -ItemType Directory -Path $cmakeSource -Force | Out-Null
@'
cmake_minimum_required(VERSION 3.20)
project(TrackerBinding LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(PYBIND11_FINDPYTHON ON)
find_package(Python 3.11 EXACT REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(xrobotoolkit_sdk "${BINDING_SOURCE}/bindings/py_bindings.cpp")
target_include_directories(xrobotoolkit_sdk PRIVATE "${BINDING_SOURCE}/include")
target_link_libraries(xrobotoolkit_sdk PRIVATE "${SDK_DIRECTORY}/PXREARobotSDK.lib")
target_compile_options(xrobotoolkit_sdk PRIVATE /utf-8)
'@ | Set-Content -LiteralPath (Join-Path $cmakeSource 'CMakeLists.txt') -Encoding utf8
$build = Join-Path $outputRoot 'build'
$cmakeBindingSource = $Source.Replace('\', '/')
$cmakeSdk = $Sdk.Replace('\', '/')
& $cmake -S $cmakeSource -B $build -G 'Visual Studio 17 2022' -A x64 "-DCMAKE_GENERATOR_INSTANCE=$visualStudio" "-DPython_EXECUTABLE=$buildPython" "-Dpybind11_DIR=$pybindCmake" "-DBINDING_SOURCE=$cmakeBindingSource" "-DSDK_DIRECTORY=$cmakeSdk"
if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }
& $cmake --build $build --config Release --parallel 2
if ($LASTEXITCODE -ne 0) { throw 'Binding compilation failed.' }
$modules = @(Get-ChildItem -LiteralPath (Join-Path $build 'Release') -Filter 'xrobotoolkit_sdk*.pyd' -File)
if ($modules.Count -ne 1) { throw "Expected exactly one compiled module, found $($modules.Count)." }
$sitePackages = Join-Path $venv 'Lib\site-packages'
Copy-Item -LiteralPath $modules[0].FullName -Destination $sitePackages -Force
Copy-Item -LiteralPath (Join-Path $Sdk 'PXREARobotSDK.dll') -Destination $sitePackages -Force
& $buildPython -c "import xrobotoolkit_sdk as s; names=('num_motion_data_available','get_motion_tracker_pose','get_motion_tracker_velocity','get_motion_tracker_acceleration','get_motion_tracker_serial_numbers','get_motion_timestamp_ns'); assert all(callable(getattr(s,n,None)) for n in names); print('Import/API check passed (SDK NOT initialized):',s.__file__); print('\n'.join(names))"
if ($LASTEXITCODE -ne 0) { throw 'Binding import/API validation failed.' }
Write-Output "Tracker binding interpreter: $buildPython"
