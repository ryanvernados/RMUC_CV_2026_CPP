# calibur/pose/CMakeLists.txt

# Define the list of sources for the yolo_infer library (mix of CXX and CUDA)
set(POSE_SOURCES
    src/calibrator.cpp
    src/infer.cpp
    src/postprocess.cu
    src/preprocess.cu
)

# Create the shared library target
add_library(yolo_infer SHARED ${POSE_SOURCES})

# Required for shared libraries
set_target_properties(yolo_infer PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Target-specific includes (headers within this library's scope)
# ASSUMPTION: infer.h is located in ${CMAKE_CURRENT_SOURCE_DIR}/include
target_include_directories(yolo_infer
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include # Expose public headers (including infer.h)
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src     # Internal sources
)

# Link dependencies 
target_link_libraries(yolo_infer
    PUBLIC
        calibur_deps        # Brings in CUDA, OpenCV, platform definitions
        nvinfer             # Legacy link for TensorRT Engine
        nvonnxparser        # Legacy link for ONNX Parser
)