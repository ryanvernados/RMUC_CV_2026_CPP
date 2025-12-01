# calibur/imu/CMakeLists.txt

set(IMU_SOURCES
    imu_reader.cpp
)

add_library(calibur_imu STATIC ${IMU_SOURCES})


target_link_directories(calibur_imu
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
