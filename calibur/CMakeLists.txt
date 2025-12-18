add_library(calibur_core STATIC
    usb_communication.cpp
)

target_include_directories(calibur_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
