cmake_minimum_required(VERSION 3.5)
project(calibur)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic -O3 -g -Wall -Werror")

include_directories(.)
include_directories(/apps/yaml-cpp/include)
link_directories(/apps/yaml-cpp/lib)

find_library(YAMLCPP yaml-cpp)
message("***", ${YAMLCPP})

set(LIB_SRC 
    calibur/log.cpp
    calibur/util.cpp 
    calibur/config.cc 
    calibur/usb_communication.cpp
)

# Add executable
add_executable(camera_test camera_test.cpp)

# Link OpenCV
target_link_libraries(camera_test ${OpenCV_LIBS})

add_library(calibur SHARED ${LIB_SRC})
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(test tests/test.cc)
target_link_libraries(test calibur ${YAMLCPP})

add_executable(test_config tests/test_config.cc)
target_link_libraries(test_config calibur ${YAMLCPP})

add_executable(test_send_data tests/test_send_data)
target_link_libraries(test_send_data calibur ${YAMLCPP})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


