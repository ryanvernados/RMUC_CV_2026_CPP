# calibur/worker/CMakeLists.txt


# ===================== [RERUN CHANGE] begin =====================
# Rerun C++ SDK via FetchContent (requires CMake 3.14+ for FetchContent_MakeAvailable)
include(FetchContent)

FetchContent_Declare(
    rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)

# ===================== [RERUN CHANGE] end =======================


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

# ===================== [RERUN CHANGE] begin =====================
# Rerun requires C++17+
target_compile_features(calibur_worker_core PUBLIC cxx_std_17)
# ===================== [RERUN CHANGE] end =======================

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
    PRIVATE
        calibur_deps
        yolo_infer
        calibur_pf


        # ===================== [RERUN CHANGE] =====================
        rerun_sdk
        # ==========================================================
)