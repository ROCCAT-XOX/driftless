# Builds GTSAM 4.3a1, gtsam_points and GLIM (CPU only, no viewer, no OpenCV, no ROS)
# plus the spike driver on Windows x64 with MSVC + vcpkg. Everything lives under C:\Tools.
#
#   powershell -ExecutionPolicy Bypass -File build-windows.ps1 [-Stage all|vcpkg|gtsam|gtsam_points|glim|driver]
#
# Prerequisites (installed once, see README):
#   C:\Tools\VSBuildTools   VS 2022 Build Tools, workload VCTools (+ CMake, Clang components)
#   C:\Tools\vcpkg          git clone https://github.com/microsoft/vcpkg + bootstrap-vcpkg.bat
param(
  [string]$Stage = "all",
  [int]$Jobs = 8
)
$ErrorActionPreference = "Stop"

$Root    = "C:\Tools\glim-spike"
$Src     = "$Root\src"
$Build   = "$Root\build"
$Prefix  = "$Root\prefix"
$Logs    = "$Root\logs"
$Vcpkg   = "C:\Tools\vcpkg"
$Triplet = "x64-windows"
$Here    = Split-Path -Parent $MyInvocation.MyCommand.Path
$Git     = "C:\Program Files\Git\cmd\git.exe"

$EIGEN_REF        = "3.4.1"
$GTSAM_REF        = "4.3a1"
$GTSAM_POINTS_REF = "8e94654558dfbe7ddb0341de300daaf17e25ee02"
$GLIM_REF         = "1a197c999e4865f5cab7c9cd25474128413ebbca"

New-Item -ItemType Directory -Force $Src, $Build, $Prefix, $Logs | Out-Null
$env:VCPKG_DEFAULT_BINARY_CACHE = "C:\Tools\vcpkg-cache"
$env:VCPKG_DOWNLOADS = "C:\Tools\vcpkg-downloads"
New-Item -ItemType Directory -Force $env:VCPKG_DEFAULT_BINARY_CACHE, $env:VCPKG_DOWNLOADS | Out-Null

# MSVC developer environment (cl, link, ninja, cmake).
& "C:\Tools\VSBuildTools\Common7\Tools\Launch-VsDevShell.ps1" -Arch amd64 -HostArch amd64 -SkipAutomaticLocation | Out-Null

function Run([string]$log, [scriptblock]$cmd) {
  # Windows PowerShell 5 turns native stderr into errors; only the exit code counts here.
  $ErrorActionPreference = "Continue"
  & $cmd *> "$Logs\$log.log"
  $code = $LASTEXITCODE
  $ErrorActionPreference = "Stop"
  if ($code -ne 0) { throw "$log failed (exit $code), see $Logs\$log.log" }
}

function Fetch([string]$name, [string]$url, [string]$ref) {
  $dir = "$Src\$name"
  if (-not (Test-Path $dir)) { & $Git clone -q $url $dir }
  & $Git -C $dir fetch -q origin
  & $Git -C $dir checkout -q -f $ref
}

# gtsam_points uses M_PI; windows.h must not define min/max; export_factors.cpp needs /bigobj.
$env:CXXFLAGS = "/D_USE_MATH_DEFINES /DNOMINMAX /bigobj"

$Common = @(
  "-G", "Ninja",
  "-DCMAKE_BUILD_TYPE=Release",
  "-DCMAKE_INSTALL_PREFIX=$Prefix",
  "-DCMAKE_PREFIX_PATH=$Prefix",
  "-DCMAKE_TOOLCHAIN_FILE=$Vcpkg\scripts\buildsystems\vcpkg.cmake",
  "-DVCPKG_TARGET_TRIPLET=$Triplet",
  # gtsam_points/glim use OpenMP tasks (OpenMP 3); MSVC's default /openmp is 2.0.
  "-DOpenMP_CXX_FLAGS=/openmp:llvm",
  "-DOpenMP_CXX_LIB_NAMES=",
  "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
)

function CMakeBuild([string]$name, [string]$srcdir, [string[]]$extra) {
  $b = "$Build\$name"
  Run "$name-cmake"   { cmake -S $srcdir -B $b @Common @extra }
  Run "$name-build"   { cmake --build $b -j $Jobs -- -k 0 }
  Run "$name-install" { cmake --install $b }
  Write-Output "== $name ok"
}

if ($Stage -in "all", "vcpkg") {
  Run "vcpkg" { & "$Vcpkg\vcpkg.exe" install --triplet $Triplet --disable-metrics `
    spdlog nlohmann-json `
    boost-serialization boost-filesystem boost-graph boost-thread boost-program-options `
    boost-date-time boost-timer boost-chrono boost-regex boost-format boost-system `
    boost-poly-collection boost-functional boost-iterator boost-property-map boost-range `
    boost-utility boost-smart-ptr }
  Write-Output "== vcpkg ok"
}
if ($Stage -in "all", "eigen") {
  # vcpkg ships Eigen 5; GTSAM 4.3a1 / gtsam_points are tested with Eigen 3.4 (as on macOS).
  Fetch eigen https://gitlab.com/libeigen/eigen.git $EIGEN_REF
  CMakeBuild eigen "$Src\eigen" @("-DEIGEN_BUILD_TESTING=OFF", "-DEIGEN_BUILD_DOC=OFF", "-DBUILD_TESTING=OFF")
}
if ($Stage -in "all", "gtsam") {
  Fetch gtsam https://github.com/borglab/gtsam.git $GTSAM_REF
  CMakeBuild gtsam "$Src\gtsam" @(
    "-DGTSAM_BUILD_TESTS=OFF", "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF", "-DGTSAM_BUILD_UNSTABLE=ON",
    "-DGTSAM_WITH_TBB=OFF", "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF", "-DGTSAM_BUILD_PYTHON=OFF",
    "-DGTSAM_USE_SYSTEM_EIGEN=ON",
    # Static: GTSAM's dllimport'ed templates (BayesTreeMarginalizationHelper<ISAM2Ext>) and
    # exported std::vector<size_t> members do not link into a shared gtsam_points.
    "-DBUILD_SHARED_LIBS=OFF")
}
if ($Stage -in "all", "gtsam_points") {
  Fetch gtsam_points https://github.com/koide3/gtsam_points.git $GTSAM_POINTS_REF
  foreach ($p in Get-ChildItem "$Here\patches\gtsam_points-*.patch") { & $Git -C "$Src\gtsam_points" apply $p.FullName }
  CMakeBuild gtsam_points "$Src\gtsam_points" @(
    "-DBUILD_WITH_CUDA=OFF", "-DBUILD_WITH_TBB=OFF", "-DBUILD_WITH_OPENMP=ON",
    "-DBUILD_WITH_MARCH_NATIVE=OFF", "-DBUILD_SHARED_LIBS=OFF")
}
if ($Stage -in "all", "glim") {
  Fetch glim https://github.com/koide3/glim.git $GLIM_REF
  foreach ($p in Get-ChildItem "$Here\patches\glim-*.patch") { & $Git -C "$Src\glim" apply $p.FullName }
  CMakeBuild glim "$Src\glim" @(
    "-DBUILD_WITH_CUDA=OFF", "-DBUILD_WITH_VIEWER=OFF", "-DBUILD_WITH_OPENCV=OFF",
    "-DBUILD_WITH_MARCH_NATIVE=OFF", "-DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON")
}
if ($Stage -in "all", "driver") {
  CMakeBuild driver "$Here\driver" @("-DGLIM_MODULE_DIR=$($Prefix -replace '\\','/')/bin")
}
