cmake_minimum_required(VERSION 3.18)

project(yolo11_pose LANGUAGES CXX CUDA)

# ===================== Language Standards =====================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

add_definitions(-DAPI_EXPORTS)

set(CMAKE_BUILD_TYPE Release)

# ===================== CUDA Toolkit =====================
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)

# Include CUDA headers (from toolkit automatically)
include_directories(${CUDAToolkit_INCLUDE_DIRS})
link_directories(${CUDAToolkit_LIBRARY_DIR})

# ===================== TensorRT =====================
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    message("Embed_platform on")
    include_directories(/usr/include/aarch64-linux-gnu)
    link_directories(/usr/lib/aarch64-linux-gnu)
else()
    message("Embed_platform off")
    include_directories(/usr/include/x86_64-linux-gnu)
    link_directories(/usr/lib/x86_64-linux-gnu)
endif()

# ===================== OpenCV =====================
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ===================== YOLO Infer Library =====================
include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE SRCS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.cu
)

add_library(yolo_infer SHARED ${SRCS})

# Required to avoid nvcc <-> gcc ABI mismatch
target_compile_options(yolo_infer PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda -Xcompiler -fPIC>
)

target_link_libraries(yolo_infer
    CUDA::cudart
    CUDA::cuda_driver
    nvinfer
    nvonnxparser
    ${OpenCV_LIBS}
)

# ===================== Main Executable =====================
add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
target_link_libraries(main yolo_infer)
