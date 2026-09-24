# ADR 0001: Sensor Fusion (EKF) over Simple Low-Pass Filtering

## Status
Accepted

## Context
Precision landing requires highly accurate pose estimation (position and orientation) of the fiducial marker relative to the camera frame. Raw `cv2.solvePnP` outputs contain high-frequency noise (jitter) due to lighting changes, camera vibrations, and motion blur. 
Initially, simple exponential smoothing (low-pass filter) was considered.

## Decision
We implemented a custom **C++ Extended Kalman Filter (EKF)** wrapped with `pybind11` instead of a simple low-pass filter.
- The state vector includes position and velocity: `[x, y, z, vx, vy, vz]^T`.
- A constant-velocity kinematic model is used for the state transition matrix (F).

## Rationale
1. **Predictive Capability**: Unlike a low-pass filter which introduces phase lag, the EKF predicts the UAV's next state using the kinematic model. This is critical for high-speed descents where lag causes FSM overshoot.
2. **Dynamic Covariance**: The EKF maintains an uncertainty covariance matrix (P), allowing the system to gate outliers (Mahalanobis distance) natively.
3. **Performance**: Writing this in C++ ensures the matrix multiplications (6x6) run in microseconds, never blocking the Python GIL or the main vision thread.

## Consequences
- **Positive**: Mitigates up to 40% of positional jitter. Unlocks velocity-based control for the guidance module.
- **Negative**: Adds a C++ compilation step (`BUILD.md`) to the deployment pipeline.
