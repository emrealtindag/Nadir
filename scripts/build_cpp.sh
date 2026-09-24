#!/bin/bash
set -e

echo "Building Nadir C++ Core (EKF)..."

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CPP_DIR="$PROJECT_ROOT/src/nadir/cpp_core"
BUILD_DIR="$CPP_DIR/build"

# Check if pybind11 is available
python3 -c "import pybind11" || { echo "pybind11 not found! Please run 'pip install pybind11' first."; exit 1; }

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake ..
make -j4

echo "Copying compiled shared library to perception module..."
cp *.so ../../perception/ || cp *.pyd ../../perception/ || echo "Warning: Could not auto-copy the library. Ensure it's in the python path."

echo "Build complete."
