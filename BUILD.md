# Nadir Build Instructions (C++ Core)

The Nadir perception pipeline utilizes a high-performance C++ Extended Kalman Filter (EKF) bound to Python via `pybind11`. This ensures real-time deterministic sensor fusion (mitigating up to 40% high-frequency jitter) while maintaining a clean Python high-level API.

## Requirements
- CMake >= 3.14
- C++17 Compatible Compiler (GCC 9+, Clang 10+, or MSVC 2019+)
- Python 3.11+ (with development headers)
- `pybind11` (`pip install pybind11`)

## Build Steps (Linux / Ubuntu)

1. Ensure prerequisites are installed:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake python3-dev
pip install pybind11
```

2. Run the automated build script:
```bash
chmod +x scripts/build_cpp.sh
./scripts/build_cpp.sh
```

3. Verification:
The build script automatically places `nadir_ekf.so` (or `.pyd` on Windows) into `src/nadir/perception/`. 
You can verify it loads by running the unit tests:
```bash
poetry run pytest tests/unit/test_ekf.py -v
```

## Build Steps (Windows / MSVC)
If building natively on Windows (though WSL2 is recommended for ROS/MAVLink compat):
1. Open "x64 Native Tools Command Prompt for VS".
2. Navigate to `src/nadir/cpp_core`.
3. Run:
```cmd
mkdir build
cd build
cmake -G "Visual Studio 17 2022" -A x64 ..
cmake --build . --config Release
copy Release\nadir_ekf*.pyd ..\..\perception\
```
