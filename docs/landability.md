# Landability Analysis & Spatial Obstacle Clearance Guide

The landability analysis subsystem (`src/nadir/perception/landability`) assesses the physical safety of the touchdown zone prior to and during descent. It provides deterministic safety gates preventing the UAV from landing on obstacles, ground vehicles, or human personnel.

---

## Architecture Overview

```
      Downward Camera Frame (640x480 @ 30 Hz)
                         │
        ┌────────────────┴────────────────┐
        ▼                                 ▼
   ArUco / AprilTag                YOLOv8 Detection
   Fiducial PnP Pose              (`yolo_plugin.py`)
        │                                 │
        ▼                                 ▼
Landing Target ROI Box              Obstacle Boxes
 [Center, 2.5x Marker Size]        (Vehicles, Humans, Objects)
        │                                 │
        └────────────────┬────────────────┘
                         ▼
        Spatial Intersection & Clearance Gating
                         │
        ┌────────────────┴────────────────┐
        │                                 │
[Overlap Area > 0]               [Overlap Area == 0]
Obstacle Intersects Pad          Obstacle Clear of Pad
        │                                 │
        ▼                                 ▼
Score: 0.20 (< 0.40)             Score: 1.0 (Safe)
FSM: ABORT (Go-Around)           FSM: DESCEND / LAND
```

---

## 1. Spatial Clearance & Intersection Gating

When the aircraft acquires the landing target via downward computer vision, it constructs a 2D bounding perimeter representing the **Landing Target Region of Interest (ROI)**:

$$\text{Pad ROI} = \left[ c_x - \frac{W_p}{2}, c_y - \frac{H_p}{2}, c_x + \frac{W_p}{2}, c_y + \frac{H_p}{2} \right]$$

where $(c_x, c_y)$ is the pixel centroid of the detected landing pad and $(W_p, H_p)$ represents $2.5\times$ the marker dimensions to account for rotor wash clearance and ground footprint.

### Mathematical Intersection Check

For each detected obstacle bounding box $B_{\text{obs}} = [x_1^o, y_1^o, x_2^o, y_2^o]$ with confidence above threshold ($\tau \ge 0.35$):

$$\Delta x = \max\left(0, \, \min(x_2^o, x_2^p) - \max(x_1^o, x_1^p)\right)$$

$$\Delta y = \max\left(0, \, \min(y_2^o, y_2^p) - \max(y_1^o, y_1^p)\right)$$

$$\text{Intersection Area} = \Delta x \cdot \Delta y$$

### Decision Boundaries:
1. **Direct Collision Hazard ($\text{Intersection Area} > 0$):**
   - The detected object encroaches upon or directly overlaps the landing pad.
   - Subsystem flags: `obstacle_in_landing_zone`.
   - Score assigned: `0.20`.
   - **FSM Action:** Triggers immediate `LandingState.ABORT` (Go-Around climb).
2. **Clearance Verified Safe ($\text{Intersection Area} = 0$):**
   - Objects are detected in the camera field of view (e.g., parked vehicles along the runway corridor in the Gazebo SITL demo), but maintain full spatial clearance outside the pad boundary.
   - Subsystem flags: `spatial_clearance_verified`.
   - Score assigned: `1.0`.
   - **FSM Action:** Final precision descent authorized; mode switches to ArduCopter `LAND` (Mode 9).

---

## 2. Detection Estimators

### A. YOLOv8 Deep Learning Plugin (`yolo_plugin.py`)
- Executes real-time inference using fine-tuned neural network weights (`weights/best.pt`).
- Detects dynamic ground targets: vehicles, personnel, debris, and irregular obstacles.
- Overlays real-time bounding boxes and tracking crosshairs directly onto the operator HUD:
  - **Orange Box / Crosshair:** Object detected with safe spatial clearance.
  - **Red Box / Crosshair:** Object encroaching upon the landing zone (collision warning).

### B. Heuristic Estimator (`heuristics.py`)
- Analyzes surface texture entropy (`texture_var_min`) to reject untextured, specular, water, or reflective surfaces.
- Tracks Farnebäck dense optical flow vectors (`motion_threshold`) to spot moving surfaces or rotor wash surface turbulence.

---

## 3. Configuration Parameters

In `configs/nadir_sim.yaml`:

```yaml
landability:
  method: "yolo"              # "yolo" | "heuristic" | "ml"
  weights_path: "weights/best.pt"
  confidence_threshold: 0.35
  abort_landability: 0.40     # FSM abort threshold
  clearance_margin_factor: 2.5 # Clearance perimeter multiplier
```
