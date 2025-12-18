# calibur/worker/CMakeLists.txt

# ===================== RERUN (always enabled, robust) =====================
include(FetchContent)

# Avoid re-downloading if already present (helpful for flaky networks / CI)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

# Pin a version (avoid /latest/ redirect + CDN weirdness)
set(RERUN_SDK_VERSION "0.27.3")
set(RERUN_SDK_URL
    "https://github.com/rerun-io/rerun/releases/download/${RERUN_SDK_VERSION}/rerun_cpp_sdk.zip"
)

# If you pass -DFETCHCONTENT_SOURCE_DIR_RERUN_SDK=/path/to/rerun_sdk
# CMake will use that local folder and skip downloading automatically.
if (DEFINED FETCHCONTENT_SOURCE_DIR_RERUN_SDK)
    message(STATUS "Rerun SDK: using local source dir: ${FETCHCONTENT_SOURCE_DIR_RERUN_SDK}")
else()
    message(STATUS "Rerun SDK: no local dir provided; will download from: ${RERUN_SDK_URL}")
endif()

FetchContent_Declare(
    rerun_sdk
    URL ${RERUN_SDK_URL}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(rerun_sdk)
# ==========================================================================


set(WORKER_SOURCES
    camera_worker.cpp
    detection_worker.cpp
    imu_worker.cpp
    pf_worker.cpp
    prediction_worker.cpp
    usb_worker.cpp
    yolo_worker.cpp
    display_worker.cpp
)

# Create the static library target
add_library(calibur_worker_core STATIC ${WORKER_SOURCES})

# Rerun requires C++17+
target_compile_features(calibur_worker_core PUBLIC cxx_std_17)

# Optional macro if you want to guard any rerun-only code (even though it's always on)
target_compile_definitions(calibur_worker_core PUBLIC CALIBUR_ENABLE_RERUN=1)

# Target-specific includes (Internal headers)
target_include_directories(calibur_worker_core
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/calibur
        ${CMAKE_SOURCE_DIR}/calibur/pose/include
        ${CMAKE_SOURCE_DIR}/calibur/pf
)

# Link all dependencies
target_link_libraries(calibur_worker_core
    PUBLIC
        calibur_core        # contains files from /calibur/
    PRIVATE
        calibur_deps
        yolo_infer
        calibur_pf
        rerun_sdk
)
