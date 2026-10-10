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
        │                           [best.pt: Tasit, Insan, UAP, UAI]
        ▼                                 │
Landing Target ROI Box                    ▼
 [Center, 2.5x Marker Size]        Obstacle Bounding Boxes
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
        ├──────────────────────┐          ▼
        │                      │   Score: 1.0 (Safe)
  Class 1: Insan         Class 0: Tasit   FSM: DESCEND / LAND
        │                      │
        ▼                      ▼
Score: 0.00            Score: 0.20 (< 0.40)
FSM: ABORT             FSM: ABORT
(human_present)        (low_landability)
```

---

## 1. Multi-Class Object Categorization (`weights/best.pt`)

The fine-tuned YOLOv8 weights contain 4 distinct object classes:

| Class ID | Class Label | Subsystem Role | Behavior on Pad Intersection |
|:--------:|:-----------:|:--------------:|:----------------------------|
| **0** | `Tasit` (Vehicle / Car) | Dynamic Obstacle | Sets `score = 0.20`, raises `vehicle_present` & `obstacle_in_landing_zone` $\rightarrow$ FSM `ABORT` |
| **1** | `Insan` (Human / Person) | Critical Personnel Hazard | Sets `score = 0.00`, raises `human_present` & `obstacle_in_landing_zone` $\rightarrow$ FSM `ABORT` (human priority) |
| **2** | `UAP` (Uçak/İHA İniş Alanı) | Designated Landing Platform | Visual validation for target platform |
| **3** | `UAI` (İniş Alanı) | Touchdown Zone | Visual boundary confirmation |

---

## 2. Spatial Clearance & Intersection Gating

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
   - **Human in Zone (`Insan`, class 1):** Strict zero-tolerance abort. Score drops to `0.0`, triggering instant `LandingState.ABORT` with abort reason `human_present`.
   - **Vehicle in Zone (`Tasit`, class 0):** Direct spatial collision hazard. Score drops to `0.20` ($< 0.40$ threshold), triggering instant `LandingState.ABORT` with abort reason `low_landability`.
2. **Clearance Verified Safe ($\text{Intersection Area} = 0$):**
   - Objects are detected in the camera field of view (e.g., parked vehicles along the runway corridor in the Gazebo SITL demo), but maintain full spatial clearance outside the pad boundary.
   - Subsystem flags: `spatial_clearance_verified`.
   - Score assigned: `1.0`.
   - **FSM Action:** Final precision descent authorized; mode switches to ArduCopter `LAND` (Mode 9).

---

## 3. Real-Time HUD Visualization

The operator HUD (`cv2.imshow("Nadir AI HUD")`) dynamically overlays:
- **`CLEAR: Tasit (0.88)` / Orange Box:** Ground vehicle detected with full spatial clearance outside the touchdown zone.
- **`COLLISION: Tasit (0.92)` / Red Box:** Vehicle encroaching upon or intersecting the landing pad perimeter.
- **`COLLISION: Insan (0.95)` / Flashing Red Box:** Human personnel detected inside the landing perimeter (triggering immediate go-around).

---

## 4. Configuration Parameters

In `configs/nadir_sim.yaml`:

```yaml
landability:
  method: "yolo"              # "yolo" | "heuristic" | "ml"
  weights_path: "weights/best.pt"
  confidence_threshold: 0.35
  abort_landability: 0.40     # FSM abort threshold
  clearance_margin_factor: 2.5 # Clearance perimeter multiplier
```
