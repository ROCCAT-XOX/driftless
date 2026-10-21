#!/usr/bin/env bash
# Builds GTSAM 4.3a1, gtsam_points and GLIM (CPU only, no viewer, no OpenCV, no ROS)
# plus the spike driver into spikes/glim/.deps/prefix. macOS arm64, Homebrew.
#
#   spikes/glim/build-macos.sh            # build everything
#   JOBS=4 spikes/glim/build-macos.sh     # fewer parallel jobs
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DEPS="$HERE/.deps"
PREFIX="$DEPS/prefix"
JOBS="${JOBS:-8}"
OMP="$(brew --prefix libomp)"
EIGEN="$(brew --prefix eigen@3)"
BREW="$(brew --prefix)"

# Pinned revisions (tested 2026-10-20).
GTSAM_REF=4.3a1                                       # 2f3e56c0ddbd3a1aa54ed043643b553d26a069f6
GTSAM_POINTS_REF=8e94654558dfbe7ddb0341de300daaf17e25ee02  # v1.2.2+
GLIM_REF=1a197c999e4865f5cab7c9cd25474128413ebbca          # v1.2.2+

brew install boost spdlog nlohmann-json eigen@3 libomp cmake

mkdir -p "$DEPS/src" "$DEPS/build" "$DEPS/logs"

fetch() { # name url ref
  local dir="$DEPS/src/$1"
  if [ ! -d "$dir" ]; then git clone -q "$2" "$dir"; fi
  git -C "$dir" fetch -q --depth 1 origin "$3" 2>/dev/null || git -C "$dir" fetch -q origin
  git -C "$dir" checkout -q -f "$3"
}
fetch gtsam https://github.com/borglab/gtsam.git "$GTSAM_REF"
fetch gtsam_points https://github.com/koide3/gtsam_points.git "$GTSAM_POINTS_REF"
fetch glim https://github.com/koide3/glim.git "$GLIM_REF"
for p in "$HERE"/patches/glim-*.patch; do git -C "$DEPS/src/glim" apply "$p"; done
for p in "$HERE"/patches/gtsam_points-*.patch; do git -C "$DEPS/src/gtsam_points" apply "$p"; done

COMMON=(
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
  -DCMAKE_PREFIX_PATH="$PREFIX;$EIGEN;$BREW"
  -DCMAKE_INSTALL_RPATH="$PREFIX/lib"
  # Apple clang has no bundled OpenMP; use Homebrew's libomp.
  -DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp -I$OMP/include"
  -DOpenMP_CXX_LIB_NAMES=omp
  -DOpenMP_omp_LIBRARY="$OMP/lib/libomp.dylib"
)

build() { # name srcdir extra-args...
  local name="$1" src="$2"; shift 2
  local b="$DEPS/build/$name"
  mkdir -p "$b"
  echo "== $name"
  cmake -S "$src" -B "$b" "${COMMON[@]}" "$@" > "$DEPS/logs/$name-cmake.log" 2>&1
  cmake --build "$b" -j "$JOBS" > "$DEPS/logs/$name-build.log" 2>&1
  cmake --install "$b" > "$DEPS/logs/$name-install.log" 2>&1
}

build gtsam "$DEPS/src/gtsam" \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
  -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DEigen3_DIR="$EIGEN/share/eigen3/cmake"
build gtsam_points "$DEPS/src/gtsam_points" \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_TBB=OFF -DBUILD_WITH_OPENMP=ON -DBUILD_WITH_MARCH_NATIVE=OFF -DBUILD_SHARED_LIBS=ON
build glim "$DEPS/src/glim" \
  -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_VIEWER=OFF -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_MARCH_NATIVE=OFF
build driver "$HERE/driver" -DGLIM_MODULE_DIR="$PREFIX/lib"

echo "OK: $PREFIX/bin/glim_driftless"
