cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Workaround for GCC 14.x: manually set compiler features before project()
if(NOT DEFINED CMAKE_CXX_COMPILER_ID)
    set(CMAKE_CXX_COMPILER_ID "GNU")
endif()
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX17_STANDARD_COMPILE_OPTION "-std=c++17")
    set(CMAKE_CXX17_EXTENSION_COMPILE_OPTION "-std=gnu++17")
endif()

project(cuda_icp LANGUAGES CXX CUDA)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set CUDA standard
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Set CUDA architectures (compute capability 5.0+, adjust based on your GPU)
# Common values: 50, 52, 60, 61, 70, 75, 80, 86, 89, 90
set(CMAKE_CUDA_ARCHITECTURES 75 86)

# Find required packages
# Note: CUDA is already enabled as a language above, no need for find_package(CUDA)

# Try to find Eigen3, or use environment variable from devenv
if(DEFINED ENV{EIGEN3_INCLUDE_DIR})
    set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
    message(STATUS "Using Eigen3 from environment: ${EIGEN3_INCLUDE_DIR}")
    # Create an imported target for compatibility
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}"
    )
else()
    find_package(Eigen3 REQUIRED)
endif()

find_package(PCL REQUIRED COMPONENTS common io search visualization)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${PCL_LIBRARY_DIRS}
)

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Compilation flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3 --use_fast_math -allow-unsupported-compiler")

# Source files
set(SOURCES
    src/main.cpp
    src/icp_cpu.cpp
    src/point_cloud_io.cpp
    src/visualization.cpp
)

set(CUDA_SOURCES
    src/icp_cuda.cu
)

# Create executable
add_executable(cuda_icp ${SOURCES} ${CUDA_SOURCES})

# Link libraries
target_link_libraries(cuda_icp
    ${PCL_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    Eigen3::Eigen
)

# Set CUDA properties
set_target_properties(cuda_icp PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

# Install target
install(TARGETS cuda_icp DESTINATION bin)
