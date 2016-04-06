cmake_minimum_required(VERSION 3.1.0)
project(motion-synthesizer)

# Local CMake Modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Get me C++11!
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(motion-synthesizer main.cpp)
target_include_directories(motion-synthesizer PRIVATE ${EIGEN3_INCLUDE_DIR})
