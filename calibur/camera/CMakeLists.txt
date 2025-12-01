# calibur/camera/CMakeLists.txt

add_executable(test_camera
    camera.cpp
)

# Header search paths
target_include_directories(test_camera
    PRIVATE
        ${CMAKE_SOURCE_DIR}              # for "calibur/camera/..."
        ${CMAKE_CURRENT_SOURCE_DIR}      # for "MvCameraControl.h"
        /opt/MVS/include                 # HIK SDK headers
)

# Where to find the .so/.a for HIK SDK
target_link_directories(test_camera
    PRIVATE
        /opt/MVS/lib/aarch64
)

# Libraries to link
target_link_libraries(test_camera
    PRIVATE
        ${OpenCV_LIBS}
        MvCameraControl                  # libMvCameraControl.so
)
