# Configuration Reference

Nadir configurations are managed via YAML files validated with Pydantic schemas (`src/nadir/config/schema.py`).

---

## Configuration File Breakdown

```yaml
# configs/default.yaml

project:
  name: Nadir
  run_id: auto
  mode: sitl                 # Options: sitl, hitl, physical
  log_level: INFO            # DEBUG, INFO, WARNING, ERROR
  output_dir: ./runs

camera:
  source: uvc                # Options: uvc, video_file, airsim
  device_index: "tcp://127.0.0.1:5600" # Camera index or GStreamer endpoint
  width: 640
  height: 480
  fps: 30
  intrinsics_path: configs/camera/calib_example.yaml
  extrinsics_path: configs/camera/extrinsics_example.yaml
  undistort: true

fiducials:
  backend: aruco            # Options: aruco, apriltag
  marker_size_m: 0.20       # Size of marker square in meters
  target_id_allowlist: [1]  # Operational target security filter
  aruco:
    dictionary: DICT_4X4_100
    refine: true

pose:
  frame: MAV_FRAME_BODY_NED
  filter:
    type: exp_smooth
    alpha: 0.35
    outlier_mahalanobis: 4.0

mavlink:
  transport: tcp            # tcp, udp, serial
  udp:
    address: "127.0.0.1"
    port: 5762
  system_id: 42
  component_id: 200
  landing_target_rate_hz: 20

control:
  enable_fsm: true
  fsm_rate_hz: 20
  thresholds:
    acquire_confidence: 0.70
    align_error_m: 0.25
    abort_landability: 0.40

landability:
  enabled: true
  method: heuristic         # Options: heuristic, ml
  ml:
    model_path: "weights/best.pt"
    device: cpu             # cpu or cuda
    score_threshold: 0.6
```
