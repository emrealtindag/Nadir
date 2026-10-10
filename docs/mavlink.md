# MAVLink Integration Guide

Nadir integrates with open-source flight stacks (ArduPilot and PX4) using the standard MAVLink protocol.

---

## MAVLink Protocol Specification

### `LANDING_TARGET` Message (#149)

Nadir streams MAVLink `LANDING_TARGET` packets at 10–50 Hz (default: 20 Hz).

```c
mavlink_landing_target_t {
    uint64_t time_usec;        // Timestamp (microseconds since boot)
    uint8_t  target_num;       // Storage index / target ID (default: 0)
    uint8_t  frame;            // MAV_FRAME_BODY_FRD = 12
    float    angle_x;          // Target angular offset along Right axis (radians)
    float    angle_y;          // Target angular offset along Forward axis (radians)
    float    distance;         // Distance to target in meters
    float    size_x;           // Target size X in radians
    float    size_y;           // Target size Y in radians
    float    x;                // X position in specified frame (m)
    float    y;                // Y position in specified frame (m)
    float    z;                // Z position in specified frame (m)
    float    q[4];             // Quaternion rotation of target
    uint8_t  type;             // LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2
    uint8_t  position_valid;   // 1 if 3D position is valid, 0 if angles only
}
```

---

## ArduPilot Precision Landing Parameters

To enable precision landing in ArduPilot (`ArduCopter 4.4+`):

```ini
# Precision Landing Subsystem
PLND_ENABLED   1        # Enable precision landing
PLND_TYPE      1        # 1 = MAVLink Companion Computer
PLND_EST_TYPE  0        # 0 = Raw sensor angles, 1 = Internal EKF

# Position offsets relative to aircraft CG (m)
PLND_CAM_POS_X 0.0
PLND_CAM_POS_Y 0.0
PLND_CAM_POS_Z 0.0

# Telemetry Serial Port
SERIAL2_PROTOCOL 2      # MAVLink 2
SERIAL2_BAUD     921    # 921600 baud
```

---

## Transport Configuration

Configured in `configs/default.yaml`:

```yaml
mavlink:
  transport: tcp           # Options: tcp, udp, serial
  udp:
    address: "127.0.0.1"
    port: 5762            # SITL companion link port
  serial:
    device: "/dev/ttyAMA0"
    baud: 921600
  system_id: 42           # Companion Computer System ID
  component_id: 200       # Precision Landing Component ID
  target_system_id: 1
  target_component_id: 1
  landing_target_rate_hz: 20
```
