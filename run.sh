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

    # Fallback / sanity
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

echo "==> ARCH             : ${ARCH}"
echo "==> CALIBUR_PLATFORM : ${CALIBUR_PLATFORM}"
[[ -n "${CALIBUR_L4T:-}" ]] && echo "==> CALIBUR_L4T      : ${CALIBUR_L4T}"
echo "==> BUILD_DIR        : ${BUILD_DIR}"

# Clean and (re)create build dir
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"

# Configure
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCUDAToolkit_ROOT=/usr/local/cuda-12.9 \
    -DCALIBUR_PLATFORM="${CALIBUR_PLATFORM}" \
    ${CALIBUR_L4T:+-DCALIBUR_L4T="${CALIBUR_L4T}"}


# Build
cmake --build "${BUILD_DIR}" -- -j"$(nproc)"

# Run test program if it exists
# if [[ -x "${ROOT_DIR}/bin/test_config" ]]; then
#     echo "==> Running bin/test_config"
#     (cd "${ROOT_DIR}/bin" && ./test_config)
# else
#     echo "bin/test_config not found or not executable, skipping."
# fi
