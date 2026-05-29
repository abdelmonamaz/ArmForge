<#
.SYNOPSIS
    Compile ArmForge from the command line, exactly like Qt Creator does —
    without opening the IDE.

.DESCRIPTION
    Qt Creator hides three things that a plain terminal doesn't have:
      1. The MSVC dev toolchain (cl.exe, link.exe) on PATH       -> loaded via vcvars64.bat
      2. jom (the parallel nmake clone used as CMAKE_MAKE_PROGRAM) -> added to PATH
      3. Qt's own bundled CMake (not the system one, if any)      -> called by full path

    Values below were read directly from this project's
    .qtcreator/CMakeLists.txt.user and build/.../CMakeCache.txt, so they
    match the "Desktop Qt 6.11.0 MSVC2022 64bit" kit exactly.

.PARAMETER Reconfigure
    Re-run CMake's configure step before building (use after editing
    CMakeLists.txt or adding/removing source files).

.PARAMETER Clean
    Build the "clean" target instead of "all".

.PARAMETER Run
    Launch ArmForge.exe after a successful build.

.EXAMPLE
    .\build.ps1
    .\build.ps1 -Reconfigure
    .\build.ps1 -Run
#>

param(
    [switch]$Reconfigure,
    [switch]$Clean,
    [switch]$Run
)

$ErrorActionPreference = "Stop"

# ---- Paths discovered from this project's Qt Creator / CMake configuration ----
$ProjectDir  = $PSScriptRoot
$BuildDir    = Join-Path $ProjectDir "build\Desktop_Qt_6_11_0_MSVC2022_64bit-Debug"
$CMakeExe    = "C:\Qt\Tools\CMake_64\bin\cmake.exe"
$JomDir      = "C:\Qt\Tools\QtCreator\bin\jom"
$QtPrefix    = "C:\Qt\6.11.0\msvc2022_64"
$VcVarsBat   = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
$CudaRoot    = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.3"
$ExePath     = Join-Path $BuildDir "ArmForge.exe"

# ---- 1. Load the MSVC x64 developer environment (cl.exe, link.exe, INCLUDE, LIB) ----
if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    Write-Host "Loading MSVC dev environment from vcvars64.bat..." -ForegroundColor Cyan
    if (-not (Test-Path $VcVarsBat)) {
        throw "vcvars64.bat not found at: $VcVarsBat"
    }
    cmd /c "`"$VcVarsBat`" && set" | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') {
            Set-Item -Path "env:\$($Matches[1])" -Value $Matches[2]
        }
    }
}

# ---- 2. Make sure jom is reachable (CMAKE_MAKE_PROGRAM for the JOM generator) ----
if (-not (Get-Command jom.exe -ErrorAction SilentlyContinue)) {
    $env:Path = "$JomDir;$env:Path"
}

# ---- 2b. Make the CUDA Toolkit discoverable (nvcc on PATH + CUDA_PATH) ----
# Needed for CMake's check_language(CUDA)/find_package(CUDAToolkit) to locate
# nvcc when building outside Qt Creator / a fresh shell that the installer's
# system-wide CUDA_PATH hasn't reached yet.
if ((Test-Path $CudaRoot) -and -not (Get-Command nvcc.exe -ErrorAction SilentlyContinue)) {
    $env:Path     = "$CudaRoot\bin;$env:Path"
    $env:CUDA_PATH = $CudaRoot
}

# ---- 3. Sanity checks ----
foreach ($tool in @("cl.exe", "link.exe", "jom.exe")) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "$tool still not found on PATH after environment setup."
    }
}
if (-not (Test-Path $CMakeExe)) {
    throw "Qt's bundled CMake not found at: $CMakeExe"
}

# ---- 4. Configure (only if requested or the build dir doesn't exist yet) ----
$needsConfigure = $Reconfigure -or -not (Test-Path (Join-Path $BuildDir "CMakeCache.txt"))
if ($needsConfigure) {
    Write-Host "Configuring (CMake, generator: NMake Makefiles JOM)..." -ForegroundColor Cyan
    & $CMakeExe -S $ProjectDir -B $BuildDir `
        -DCMAKE_BUILD_TYPE:STRING=Debug `
        -DCMAKE_GENERATOR:STRING="NMake Makefiles JOM" `
        -DCMAKE_PREFIX_PATH:PATH=$QtPrefix `
        -DQT_QMAKE_EXECUTABLE:FILEPATH="$QtPrefix\bin\qmake.exe" `
        -DCMAKE_CXX_COMPILER:FILEPATH=cl.exe `
        -DCMAKE_C_COMPILER:FILEPATH=cl.exe
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit $LASTEXITCODE)." }
}

# ---- 5. Build ----
$target = if ($Clean) { "clean" } else { "all" }
Write-Host "Building target '$target'..." -ForegroundColor Cyan
& $CMakeExe --build $BuildDir --target $target
if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)." }

Write-Host "Build OK -> $ExePath" -ForegroundColor Green

# ---- 6. Optionally run ----
if ($Run -and -not $Clean) {
    if (-not (Test-Path $ExePath)) { throw "Executable not found: $ExePath" }
    Write-Host "Launching ArmForge.exe..." -ForegroundColor Cyan
    & $ExePath
}
