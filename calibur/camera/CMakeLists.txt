# calibur/camera/CMakeLists.txt

add_executable(calibur_camera_test
    camera.cpp
)

# Header search paths
target_include_directories(calibur_camera_test
    PRIVATE
        ${CMAKE_SOURCE_DIR}              # for "calibur/camera/..."
        ${CMAKE_CURRENT_SOURCE_DIR}      # for "MvCameraControl.h"
        /opt/MVS/include                 # HIK SDK headers
)

# Where to find the .so/.a for HIK SDK
target_link_directories(calibur_camera_test
    PRIVATE
        /opt/MVS/lib/aarch64
)

# Libraries to link
target_link_libraries(calibur_camera_test
    PRIVATE
        ${OpenCV_LIBS}
        MvCameraControl                  # libMvCameraControl.so
)
