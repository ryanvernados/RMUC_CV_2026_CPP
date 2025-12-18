# ROOT CMakeLists.txt

cmake_minimum_required(VERSION 3.10) 
# Explicitly set the compiler path as a fallback
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "Path to nvcc compiler")

project(calibur LANGUAGES CXX CUDA)

# --- Policies (Set all policies early and correctly) ---
if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
if (POLICY CMP0104)
    cmake_policy(SET CMP0104 NEW)
endif()
if (POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif() # Important for modern target commands

# --- CUDA Setup ---
find_package(CUDAToolkit REQUIRED) 

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ============================================== CHange this to -O3 when deploying! =======================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic -O3 -g -Wall")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# =========================================================
#  Platform detection: x86_64 Linux vs Jetson (aarch64)
# =========================================================
set(CALIBUR_PLATFORM "auto" CACHE STRING "calibur target platform (auto|x86|jetson)")

if (CALIBUR_PLATFORM STREQUAL "auto")
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(CALIBUR_PLATFORM "jetson")
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(CALIBUR_PLATFORM "x86")
    else()
        message(FATAL_ERROR "Unknown arch '${CMAKE_SYSTEM_PROCESSOR}', please set CALIBUR_PLATFORM manually.")
    endif()
endif()

message(STATUS "CALIBUR_PLATFORM = ${CALIBUR_PLATFORM}")

# For Jetson: which L4T (36 = JP6.2, 38 = JP7.x)?
set(CALIBUR_L4T "" CACHE STRING "L4T major version (36 for JetPack 6.x, 38 for JetPack 7.x)")

add_library(calibur_platform INTERFACE)

if (CALIBUR_PLATFORM STREQUAL "jetson")
    target_compile_definitions(calibur_platform INTERFACE CALIBUR_PLATFORM_JETSON=1)

    # Try auto-detect L4T if user didn’t set it
    if (NOT CALIBUR_L4T AND EXISTS "/etc/nv_tegra_release")
        file(READ "/etc/nv_tegra_release" NV_TEGRA_REL)
        if (NV_TEGRA_REL MATCHES "R36")
            set(CALIBUR_L4T "36" CACHE STRING "detected L4T version" FORCE)
        elseif (NV_TEGRA_REL MATCHES "R38")
            set(CALIBUR_L4T "38" CACHE STRING "detected L4T version" FORCE)
        endif()
    endif()

    message(STATUS "CALIBUR_L4T = ${CALIBUR_L4T}")

    if (CALIBUR_L4T STREQUAL "36")
        target_compile_definitions(calibur_platform INTERFACE CALIBUR_L4T36=1)
        target_include_directories(calibur_platform INTERFACE
            /usr/include/aarch64-linux-gnu
        )
        target_link_directories(calibur_platform INTERFACE
            /usr/lib/aarch64-linux-gnu
        )
        set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "Target CUDA architecture" FORCE) 
        
    elseif (CALIBUR_L4T STREQUAL "38")
        target_compile_definitions(calibur_platform INTERFACE CALIBUR_L4T38=1)
        target_include_directories(calibur_platform INTERFACE
            /usr/include/aarch64-linux-gnu
        )
        target_link_directories(calibur_platform INTERFACE
            /usr/lib/aarch64-linux-gnu
        )
        set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "Target CUDA architecture" FORCE) 

    else()
        message(WARNING "Unknown L4T version; set CALIBUR_L4T to 36 or 38 explicitly.")
    endif()

elseif (CALIBUR_PLATFORM STREQUAL "x86")
    target_compile_definitions(calibur_platform INTERFACE CALIBUR_PLATFORM_X86=1)

    target_include_directories(calibur_platform INTERFACE
        /usr/include/x86_64-linux-gnu
    )
    target_link_directories(calibur_platform INTERFACE
        /usr/lib/x86_64-linux-gnu
    )
    set(CMAKE_CUDA_ARCHITECTURES 75 86 CACHE STRING "Target CUDA architecture" FORCE)
    
else()
    message(FATAL_ERROR "Unsupported CALIBUR_PLATFORM='${CALIBUR_PLATFORM}'")
endif()

# ----------------- Dependencies -----------------

# Use the full component list to ensure all OpenCV targets are found, but rely on legacy linking
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)
find_package(Eigen3 REQUIRED)

list(APPEND CMAKE_PREFIX_PATH
     "${CMAKE_SOURCE_DIR}/apps/yaml-cpp/lib/cmake")
find_package(yaml-cpp REQUIRED)

# Manually add OpenCV include directory for NVCC/C++ compilation
# This must be done because we're not using the imported targets.
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /opt/MVS/include    
    ${CMAKE_SOURCE_DIR}/calibur/camera
)

link_directories(
    /opt/MVS/lib/aarch64
    /usr/lib/aarch64-linux-gnu
)


# Optionally expose common deps via another interface target
add_library(calibur_deps INTERFACE)

# --- FIX: Revert to legacy OpenCV linking, keep modern for others ---
target_link_libraries(calibur_deps INTERFACE
    calibur_platform
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    # Use legacy variables for linking OpenCV libraries
    ${OpenCV_LIBS}
    CUDA::cudart 
)
# -----------------------------------------------------------------


# ----------------- Subdirectories -----------------
add_subdirectory(calibur/camera)
add_subdirectory(calibur/imu)
add_subdirectory(calibur/pf)
add_subdirectory(calibur/pose)
add_subdirectory(calibur/worker)
add_subdirectory(calibur/)


# # Ensure USB communication and logger get compiled into the worker core library
# if (TARGET calibur_worker_core)
#     target_sources(calibur_worker_core PRIVATE
#         ${CMAKE_SOURCE_DIR}/calibur/usb_communication.cpp
#         ${CMAKE_SOURCE_DIR}/calibur/log.cpp
#     )
# endif()

# ----------------- Executable -----------------
add_executable(calibur_worker
    main.cpp
    # tests/test_imu.cc
)

target_link_libraries(calibur_worker
    PRIVATE
        MvCameraControl
        calibur_worker_core
        calibur_imu
        calibur_pf
        yolo_infer
        calibur_deps
)