cmake_minimum_required(VERSION 3.10)
project(ByteTrack3D)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/ByteTrack.cpp
    src/KalmanFilter.cpp
    src/lapjv.cpp
    src/object.cpp
    src/STrack.cpp
)

# Library
add_library(ByteTrack3D ${SOURCES})

# Link libraries
target_link_libraries(ByteTrack3D ${OpenCV_LIBS})

# Example executable
add_executable(example src/example.cpp)
target_link_libraries(example ByteTrack3D ${OpenCV_LIBS})

# Compiler options
target_compile_options(ByteTrack3D PRIVATE -Wall -Wextra)
target_compile_options(example PRIVATE -Wall -Wextra)

# Install targets
install(TARGETS ByteTrack3D DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Tests
enable_testing()
add_test(NAME example_test COMMAND example)