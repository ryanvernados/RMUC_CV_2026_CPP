cmake_minimum_required(VERSION 3.20)
project(calibur)

# -----------------------
# C++ standard and flags
# -----------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall -Werror")

# -----------------------
# Prebuilt YAML-CPP
# -----------------------
# Path to YAML-CPP headers and lib inside repo or system
set(YAMLCPP_INCLUDE "D:/yaml-cpp/include")
set(YAMLCPP_LIB "D:/yaml-cpp/lib/yaml-cpp.lib")

# -----------------------
# OpenCV
# -----------------------
# Set path to folder containing OpenCVConfig.cmake
set(OpenCV_DIR "D:/OpenCV/build/x64/vc16")
find_package(OpenCV REQUIRED)

# -----------------------
# Source files for library
# -----------------------
set(LIB_SRC 
    calibur/log.cpp
    calibur/util.cpp 
    calibur/config.cc 
    calibur/usb_communication.cpp
)

# -----------------------
# Build calibur as shared library
# -----------------------
add_library(calibur SHARED ${LIB_SRC})
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include YAML-CPP and OpenCV headers
target_include_directories(calibur PRIVATE 
    ${YAMLCPP_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}       # optional: include root project headers
)

# Link libraries to calibur
target_link_libraries(calibur PRIVATE 
    ${YAMLCPP_LIB}
    ${OpenCV_LIBS}
)

# -----------------------
# Executables
# -----------------------
add_executable(test tests/test.cc)
add_executable(test_config tests/test_config.cc)
add_executable(test_send_data tests/test_send_data.cc)

# Link executables to calibur + YAML-CPP + OpenCV
target_link_libraries(test PRIVATE calibur ${YAMLCPP_LIB} ${OpenCV_LIBS})
target_link_libraries(test_config PRIVATE calibur ${YAMLCPP_LIB} ${OpenCV_LIBS})
target_link_libraries(test_send_data PRIVATE calibur ${YAMLCPP_LIB} ${OpenCV_LIBS})

# -----------------------
# Output paths
# -----------------------
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
