# PX4 Autopilot Integration Guide

Nadir supports integration with PX4 Autopilot (`v1.14+`) via the `landing_target_estimator` subsystem.

---

## PX4 Parameters

Set the following parameters via QGroundControl or MAVLink shell:

```ini
# Landing Target Estimator Configuration
LTEST_MODE 2            # 2 = MAVLink LANDING_TARGET messages
LTEST_POS_UNC_IN 0.1    # Initial position uncertainty (m)
LTEST_VEL_UNC_IN 0.1    # Initial velocity uncertainty (m/s)

# Precision Land Flight Task
MPC_LAND_SPEED 0.6      # Descent velocity during landing (m/s)
PLD_BTOUT 5.0           # Precision landing timeout (s)
```

---

## Running Nadir with PX4 SITL

```bash
# Initialize PX4 SITL
./scripts/run_sitl_px4.sh

# Run Nadir with PX4 profile
poetry run nadir run --config configs/px4_sitl.yaml
```
