# Simulation Environment (Gazebo Harmonic & ArduPilot SITL)

This directory contains the simulation assets, models, and world files used for testing the Nadir precision landing system.

## Directory Structure
```
simulation/
├── gazebo/
│   ├── models/
│   │   └── aruco_marker.png      # Texture for the precision landing pad
│   └── worlds/
│       └── nadir_world.sdf       # Gazebo Harmonic world with asphalt road, vehicles, and landing pad
└── README.md
```

## Running the Simulation

### 1. Launch Gazebo Harmonic
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/nadir_sim/src/ardupilot_gazebo/build
export GZ_SIM_RESOURCE_PATH=$HOME/nadir_sim/src/ardupilot_gazebo/models
gz sim -v4 -r simulation/gazebo/worlds/nadir_world.sdf
```

### 2. Launch ArduPilot SITL (ArduCopter)
```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

### 3. Run Nadir Precision Landing System
```bash
poetry run nadir run
```

### 4. Execute Autonomous Mission
In the ArduPilot console:
```text
mode guided
arm throttle
takeoff 10
mode auto
```
The drone will cruise forward, detect the landing pad via the nadir camera, lock target, and execute closed-loop precision landing.
