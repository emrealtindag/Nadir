cmake_minimum_required(VERSION 3.14)
project(scandium_cpp_core)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find pybind11
find_package(pybind11 REQUIRED)

# Include Eigen headers (assuming installed or using local copy, but for recruiter demo we can use standard math or minimal matrix class to avoid Eigen dependency issues in standard python envs. Let's write a minimal matrix/EKF without Eigen to ensure it compiles smoothly without external C++ deps like Eigen which might fail if not installed system-wide).
# Actually, pybind11 is usually installed via pip. Let's just use std::vector for matrix math to keep it self-contained and zero-dependency!

pybind11_add_module(scandium_ekf
    ekf.cpp
    pybind_wrapper.cpp
)

# Optimize for performance
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(scandium_ekf PRIVATE -O3 -ffast-math -march=native -Wall -Wextra)
elseif(MSVC)
    target_compile_options(scandium_ekf PRIVATE /O2 /fp:fast /W4)
endif()
