#!/bin/bash
set -euo pipefail

# Project root = directory of this script
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ARCH="$(uname -m)"
CALIBUR_PLATFORM=""
CALIBUR_L4T=""
BUILD_SUBDIR=""

if [[ "$ARCH" == "x86_64" ]]; then
    CALIBUR_PLATFORM="x86"
    BUILD_SUBDIR="x86"

elif [[ "$ARCH" == "aarch64" ]]; then
    CALIBUR_PLATFORM="jetson"

    # Try to auto-detect L4T version from /etc/nv_tegra_release
    if [[ -f /etc/nv_tegra_release ]]; then
        if grep -q "R36" /etc/nv_tegra_release; then
            CALIBUR_L4T="36"
        elif grep -q "R38" /etc/nv_tegra_release; then
            CALIBUR_L4T="38"
        fi
    fi

    if [[ -z "$CALIBUR_L4T" ]]; then
        echo "Warning: could not auto-detect L4T version, defaulting to 36"
        CALIBUR_L4T="36"
    fi

    BUILD_SUBDIR="jetson${CALIBUR_L4T}"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

BUILD_DIR="${ROOT_DIR}/build/${BUILD_SUBDIR}"

# Local dependencies inside repo:
RERUN_LOCAL_DIR="${ROOT_DIR}/third_party/rerun_sdk/rerun_cpp_sdk"

# IMPORTANT: Rerun builds Arrow from the "apache-arrow-18.0.0/cpp" folder.
ARROW_LOCAL_DIR="${ROOT_DIR}/third_party/arrow/apache-arrow-18.0.0/cpp"

echo "==> ROOT_DIR         : ${ROOT_DIR}"
echo "==> ARCH             : ${ARCH}"
echo "==> CALIBUR_PLATFORM : ${CALIBUR_PLATFORM}"
[[ -n "${CALIBUR_L4T:-}" ]] && echo "==> CALIBUR_L4T      : ${CALIBUR_L4T}"
echo "==> BUILD_DIR        : ${BUILD_DIR}"
echo "==> RERUN_LOCAL_DIR  : ${RERUN_LOCAL_DIR}"
echo "==> ARROW_LOCAL_DIR  : ${ARROW_LOCAL_DIR}"

# Sanity checks
if [[ ! -d "${RERUN_LOCAL_DIR}" ]]; then
    echo ""
    echo "ERROR: Rerun SDK not found at:"
    echo "  ${RERUN_LOCAL_DIR}"
    echo ""
    exit 1
fi

if [[ ! -f "${ARROW_LOCAL_DIR}/CMakeLists.txt" ]]; then
    echo ""
    echo "ERROR: Arrow C++ source not found (expected CMakeLists.txt) at:"
    echo "  ${ARROW_LOCAL_DIR}"
    echo ""
    echo "You said Arrow is under third_party/arrow. It should look like:"
    echo "  third_party/arrow/apache-arrow-18.0.0/cpp/CMakeLists.txt"
    echo ""
    exit 1
fi

# Clean build dir
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# Configure
CMAKE_ARGS=(
  -S "${ROOT_DIR}"
  -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE=Debug
  -DCUDAToolkit_ROOT=/usr/local/cuda-12.9
  -DCALIBUR_PLATFORM="${CALIBUR_PLATFORM}"
  -DFETCHCONTENT_SOURCE_DIR_RERUN_SDK="${RERUN_LOCAL_DIR}"
  # THIS is the missing piece: tell FetchContent to use local Arrow instead of downloading
  -DFETCHCONTENT_SOURCE_DIR_ARROW_CPP="${ARROW_LOCAL_DIR}"
)

if [[ -n "${CALIBUR_L4T:-}" ]]; then
  CMAKE_ARGS+=(-DCALIBUR_L4T="${CALIBUR_L4T}")
fi

echo ""
echo "==> Configuring with:"
printf '    %q \\\n' cmake "${CMAKE_ARGS[@]}"
echo ""

cmake "${CMAKE_ARGS[@]}"

# Build
echo "==> Building..."
cmake --build "${BUILD_DIR}" -- -j"$(nproc)"
echo "==> Done."
