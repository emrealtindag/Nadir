# ArduPilot SITL Integration Guide

This guide details configuring and launching ArduPilot Software-In-The-Loop (SITL) with Nadir.

---

## Architecture Overview

```
+--------------------------------------------------------+
|                     Gazebo Harmonic                    |
|  (Iris Quadcopter + Downward Camera + Landing Runway)  |
+---------------------------+----------------------------+
                            | GStreamer Video (Port 5600)
                            v
+-------------------+       +----------------------------+
|   ArduPilot SITL  |<----->|          Nadir             |
|   (ArduCopter)    |  TCP  |  (Vision + EKF + FSM + AI) |
|                   |  5762 |                            |
+-------------------+       +----------------------------+
         ^
         | UDP 14550
         v
+-------------------+
|  Mission Planner  |
|    or QGC / GCS   |
+-------------------+
```

---

## Launch Steps

### 1. Launch Gazebo Harmonic Simulation
```bash
gz sim -v4 -r simulation/gazebo/worlds/nadir_world.sdf
```

### 2. Launch ArduPilot SITL
In a terminal with the ArduPilot environment:
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
```

### 3. Launch Nadir Companion Software
```bash
poetry run nadir run --config configs/default.yaml
```

---

## Autonomous Flight Sequence

1. **Arm & Takeoff**:
   In the MAVProxy console:
   ```text
   GUIDED> mode GUIDED
   GUIDED> arm throttle
   GUIDED> takeoff 10
   ```

2. **Waypoint Navigation**:
   Switch to `AUTO` mode to follow the approach route:
   ```text
   GUIDED> mode AUTO
   ```

3. **Autonomous Lock & Precision Descent**:
   Once the UAV enters the field of view of the landing pad:
   - Nadir detects the ArUco marker.
   - The HUD visualizer locks onto target center coordinates.
   - Nadir commands ArduPilot into `LAND` mode (Mode 9).
   - High-rate `LANDING_TARGET` messages center the aircraft directly above the landing pad.
   - Touchdown occurs with sub-20 cm accuracy.
