# Nadir Documentation

Welcome to the official technical documentation for **Nadir**, a production-grade precision landing system for autonomous multirotor and Unmanned Aerial Vehicle (UAV) platforms.

---

## What is Nadir?

Nadir is companion computer software engineered to solve the problem of high-accuracy autonomous landing in GNSS-degraded or GNSS-denied environments. It integrates:

- **Computer Vision & Fiducials**: High-speed ArUco and AprilTag marker detection with sub-pixel corner refinement.
- **Sensor Fusion (EKF)**: C++ Extended Kalman Filter with outlier rejection and coordinate frame transformations (Camera to Body FRD / NED).
- **Landability Analysis**: Real-time landing zone safety evaluation utilizing YOLOv8 and semantic segmentation to detect hazards, obstacles, and humans.
- **Deterministic FSM**: 9-state Finite State Machine guaranteeing safe transitions and automatic failsafes.
- **MAVLink Protocol**: Native publishing of `LANDING_TARGET` messages directly to ArduPilot and PX4 autopilots.

---

## Documentation Navigation

- [System Architecture](architecture.md)
- [MAVLink Integration](mavlink.md)
- [ArduPilot SITL & Gazebo Integration](ardupilot.md)
- [PX4 Autopilot Integration](px4.md)
- [Simulation Environments](simulation.md)
- [Landability Analysis](landability.md)
- [YOLOv8 Model Card](model_card.md)
- [Configuration Reference](configuration.md)
- [Testing & Quality Assurance](testing.md)
- [Deployment & Companion Computers](deployment.md)
