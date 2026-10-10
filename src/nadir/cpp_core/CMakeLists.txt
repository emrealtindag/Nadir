cmake_minimum_required(VERSION 3.14)
project(nadir_cpp_core)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find pybind11
find_package(pybind11 REQUIRED)

pybind11_add_module(nadir_ekf
    ekf.cpp
    pybind_wrapper.cpp
)

# Optimize for performance
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(nadir_ekf PRIVATE -O3 -ffast-math -march=native -Wall -Wextra)
elseif(MSVC)
    target_compile_options(nadir_ekf PRIVATE /O2 /fp:fast /W4)
endif()
