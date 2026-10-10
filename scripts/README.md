# Utility and Automation Scripts

This directory contains standalone scripts for simulation, calibration, training, benchmarking, and development.

## Script Overview

| Script | Description |
| :--- | :--- |
| **`bake_params.py`** | Injects optimized Nadir precision landing parameters (`PLND_ENABLED`, `WP_SPD`, `PLND_TYPE`) into ArduPilot SITL `gazebo-iris.parm`. |
| **`benchmark_latency.py`** | Benchmarks end-to-end inference and pose estimation latency across pipeline components. |
| **`build_cpp.sh`** | Compiles the high-performance C++ EKF shared library with pybind11. |
| **`calibrate_camera.py`** | Interactive camera intrinsics calibration tool using checkerboard patterns. |
| **`gen_aruco.py`** | Generates high-contrast ArUco and fiducial markers for physical prints and simulation textures. |
| **`generate_demo_gif.py`** | Generates HUD demonstration GIF animations from saved frames. |
| **`run_sitl_ardupilot.sh`** | Shell launcher for ArduPilot SITL multirotor simulation. |
| **`run_sitl_px4.sh`** | Shell launcher for PX4 SITL multirotor simulation. |
| **`train_segmentation.py`** | PyTorch training pipeline for semantic segmentation of safe landing terrain. |
