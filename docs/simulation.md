# Simulation Environments Guide

Nadir supports simulation testing across multiple platforms, primarily focusing on **Gazebo Harmonic (Gz Sim)** alongside software-in-the-loop (SITL) autopilots.

---

## 1. Gazebo Harmonic (Primary)

The primary simulation environment is located in `simulation/gazebo/`:
- **World File**: `simulation/gazebo/worlds/nadir_world.sdf`
- **Models & Textures**: `simulation/gazebo/models/aruco_marker.png`

### Prerequisites
- Ubuntu 22.04 LTS or 24.04 LTS (WSL2 supported)
- Gazebo Harmonic (`gz-harmonic`)
- ArduPilot Gazebo Plugin (`ardupilot_gazebo`)

### Launching the Simulation World
```bash
gz sim -v4 -r simulation/gazebo/worlds/nadir_world.sdf
```

---

## 2. Microsoft AirSim (Legacy / Synthetic Benchmarking)

For synthetic visual degradation tests (fog, rain, extreme wind), Nadir includes an AirSim client bridge in `src/nadir/sim/airsim_bridge.py`.

```bash
poetry run nadir sim airsim --config configs/airsim_demo.yaml
```

---

## 3. Video Playback & Offline Testing

For testing without running heavy 3D simulation engines, Nadir can ingest pre-recorded MP4 flight logs:

```yaml
camera:
  source: video_file
  path: "donerkanatlihassasinis.mp4"
  loop: true
```
