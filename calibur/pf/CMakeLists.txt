# calibur/pf/CMakeLists.txt (Updated)

# Define the list of sources for this library
set(PF_SOURCES
    rbpf.cu
    rbpf.cuh
)

# Create the static library target
add_library(calibur_pf STATIC ${PF_SOURCES})

set_property(SOURCE rbpf.cu PROPERTY COMPILE_FLAGS "--expt-relaxed-constexpr")

# --- FIX: Manually add worker and pose includes needed by rbpf.cu's headers ---
target_include_directories(calibur_pf
    PUBLIC
        . # for rbpf.cuh
        ${PROJECT_SOURCE_DIR}/calibur/worker     # Need workers.hpp and types.hpp
        ${PROJECT_SOURCE_DIR}/calibur/pose/include # Need infer.h
)

# Link common dependencies (including CUDA and OpenCV includes propagated from root)
target_link_libraries(calibur_pf
    PUBLIC
        calibur_deps
)