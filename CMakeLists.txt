cmake_minimum_required(VERSION 3.5)
project(calibur)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic -O3 -g -Wall -Werror")

include_directories(.)
include_directories(/apps/yaml-cpp/include)
link_directories(/apps/yaml-cpp/lib)
link_directories(/usr/lib/aarch64-linux-gnu)
find_library(YAMLCPP yaml-cpp)
message("***", ${YAMLCPP})

find_package(PkgConfig REQUIRED)
pkg_check_modules(OpenCV4 REQUIRED opencv4)

include_directories(${OpenCV4_INCLUDE_DIRS})
link_directories(${OpenCV4_LIBRARY_DIRS})
add_definitions(${OpenCV4_CFLAGS_OTHER})

set(LIB_SRC 
    calibur/log.cpp
    calibur/util.cpp 
    calibur/config.cc 
    calibur/usb_communication.cpp
)

include_directories(
    /opt/MVS/include
    /usr/include/opencv4
    ${CMAKE_SOURCE_DIR}/calibur/camera
)

link_directories(
    /opt/MVS/lib/aarch64
    /usr/lib/aarch64-linux-gnu
)

add_library(calibur SHARED ${LIB_SRC})
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(test_camera calibur tests/test_camera.cc)
target_include_directories(test_camera PRIVATE
    /usr/include/opencv4
)
target_link_directories(test_camera PRIVATE
    /usr/lib/aarch64-linux-gnu
)
target_link_libraries(test_camera calibur MvCameraControl 
    opencv_highgui
    opencv_videoio
    opencv_imgproc
    opencv_core
)


add_executable(test_yolo11 calibur tests/test_yolo.cc)
target_include_directories(test_yolo11 PRIVATE
    /usr/include/opencv4
)
target_link_directories(test_yolo11 PRIVATE
    /usr/lib/aarch64-linux-gnu
)
target_link_libraries(test_yolo11 calibur 
    opencv_core 
    opencv_highgui 
    opencv_imgproc
    opencv_videoio
    opencv_imgcodecs
    opencv_dnn
)

add_executable(test tests/test.cc)
target_link_libraries(test calibur ${YAMLCPP})

add_executable(test_config tests/test_config.cc)
target_link_libraries(test_config calibur ${YAMLCPP})

add_executable(test_send_data tests/test_send_data.cc)
target_link_libraries(test_send_data calibur ${YAMLCPP})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


