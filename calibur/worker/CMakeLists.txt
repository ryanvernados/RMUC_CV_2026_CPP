# calibur/worker/CMakeLists.txt

set(WORKER_SOURCES
    camera_worker.cpp
    detection_worker.cpp
    imu_worker.cpp
    pf_worker.cpp
    prediction_worker.cpp
    usb_worker.cpp
    yolo_worker.cpp
)

# Create the static library target
add_library(calibur_worker_core STATIC ${WORKER_SOURCES})

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
)